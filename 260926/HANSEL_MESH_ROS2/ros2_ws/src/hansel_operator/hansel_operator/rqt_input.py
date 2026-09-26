"""RQT keyboard boundary using the existing motion, servo and service contracts."""
import time
import uuid

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from hansel_interfaces.msg import MotionCommand, EmergencyStop, HeadServoCommand
from hansel_interfaces.srv import SetDriveEnabled, DetachUnit
from .keyboard_control import KeyboardSession


class RqtInput:
    def __init__(self, node, model, report):
        self.node, self.model, self.report = node, model, report
        self.source = 'rqt:' + uuid.uuid4().hex
        self.sequence = 0
        self.pending = {}
        self._deadline = 0.0
        self._timed_out = False
        motion_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        estop_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.motion = node.create_publisher(MotionCommand, '/hansel/system/command/motion', motion_qos)
        self.estop = node.create_publisher(EmergencyStop, '/hansel/system/command/estop', estop_qos)
        self.head = node.create_publisher(HeadServoCommand,
            f'/hansel/{model.topology.head}/command/front_servo', 10)
        self.drives = {u:node.create_client(SetDriveEnabled,f'/hansel/{u}/set_drive_enabled')
                       for u in model.topology.units}
        self.detach = node.create_client(DetachUnit, '/hansel/detach/execute')
        self.session = KeyboardSession(self.publish_motion)

    @property
    def busy(self):
        return bool(self.pending)

    def _message(self, kind):
        msg = kind()
        msg.stamp = self.node.get_clock().now().to_msg()
        self.sequence += 1
        msg.sequence, msg.source = self.sequence, self.source
        return msg

    def publish_motion(self, command):
        msg = self._message(MotionCommand)
        msg.command, msg.speed_scale = command, 1.0
        self.motion.publish(msg)

    def stop(self):
        self.session.command, self.session.held_key = 'stop', None
        self.publish_motion('stop')

    def set_enabled(self, enabled):
        if self.busy or (enabled and (self.model.busy or self.model.recovery_required)):
            self.report('Drive request blocked: configuration or drive request still pending')
            return
        targets = self.model.active or ()
        if not targets or any(not self.drives[u].service_is_ready() for u in targets):
            self.report('Drive request not sent: active targets or drive services unavailable')
            return
        self.stop()
        self._deadline, self._timed_out = time.monotonic() + 3., False
        for unit in targets:
            request = SetDriveEnabled.Request()
            request.enabled, request.source = bool(enabled), self.source
            try:
                self.pending[unit] = self.drives[unit].call_async(request)
            except Exception as exc:
                self.report(f'{unit}: drive request failed: {exc}')

    def key(self, key):
        if self.session.key(key):
            return True
        if not self.session.active:
            return False
        if key in ('g','b'):
            self.set_enabled(key == 'g')
        elif key in ('!','r'):
            self.stop()
            msg = self._message(EmergencyStop)
            msg.engaged, msg.reason = key == '!', 'RQT keyboard'
            self.estop.publish(msg)
        elif key in ('u','j','k'):
            msg = self._message(HeadServoCommand)
            msg.command = {'u':HeadServoCommand.UP_STEP,'j':HeadServoCommand.DOWN_STEP,
                           'k':HeadServoCommand.CENTER}[key]
            self.head.publish(msg)
        elif key in ('f','v'):
            self.publish_motion('front_motor_forward' if key == 'f' else 'front_motor_stop')
        elif key.isdigit() and key != '0':
            rears = [u for u in self.model.topology.units if self.model.topology.roles[u] == 'rear']
            index = int(key)-1
            if index >= len(rears) or rears[index] not in (self.model.active or ()):
                self.report('Detach target is not an active configured rear unit')
            elif self.model.busy or self.model.recovery_required or self.busy:
                self.report('Detach blocked while a configuration/drive operation is pending')
            elif not self.detach.service_is_ready():
                self.report('Detach coordinator unavailable')
            else:
                self.stop()
                request = DetachUnit.Request()
                request.released_unit_id, request.mode, request.source = rears[index], DetachUnit.Request.MANUAL, self.source
                try:
                    self.pending['detach '+rears[index]] = self.detach.call_async(request)
                    self._deadline, self._timed_out = time.monotonic()+3., False
                except Exception as exc:
                    self.report(f'Detach request failed: {exc}')
        elif key == 'p':
            self.stop()
        else:
            return False
        return True

    def poll(self):
        for unit, future in list(self.pending.items()):
            if not future.done():
                continue
            try:
                response = future.result()
                accepted = response is not None and response.accepted
                self.report(f'{unit}: {"accepted" if accepted else "rejected"}: '
                            + (response.message if response is not None else 'no response'))
            except Exception as exc:
                self.report(f'{unit}: request failed: {exc}')
            del self.pending[unit]
        if self.pending and not self._timed_out and time.monotonic() >= self._deadline:
            self.report('Drive/detach response unresolved: ' + ', '.join(self.pending)
                        + '. Waiting for late responses; check actual unit state.')
            self._timed_out = True

    def close(self):
        self.session.focus(False)
        for client in [*self.drives.values(),self.detach]:
            self.node.destroy_client(client)
        for publisher in (self.motion,self.head,self.estop):
            self.node.destroy_publisher(publisher)
