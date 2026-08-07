"""Single-key operator control matching HANSEL_MESH mesh_control_client.py."""

from __future__ import annotations

import select
import sys
import threading

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from hansel_interfaces.msg import EmergencyStop, HeadServoCommand, MotionCommand
from hansel_interfaces.srv import DetachUnit, SetDriveEnabled


LIVE_KEYS = {
    "w": "forward", "s": "backward", "a": "left", "d": "right",
    "x": "stop", " ": "stop",
    "e": "forward_right", "q": "forward_left",
    "c": "backward_right", "z": "backward_left",
}
ONE_SHOT_KEYS = {
    "u": HeadServoCommand.UP_STEP,
    "j": HeadServoCommand.DOWN_STEP,
    "k": HeadServoCommand.CENTER,
}
DETACH_KEYS = {"1": "node1", "2": "node2", "3": "node3"}


class OperatorInput(Node):
    def __init__(self) -> None:
        super().__init__("operator_input")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("send_interval_s", 0.10)
        self.declare_parameter("active_targets", ["head", "node1", "node2"])
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.active_command = "stop"
        self.sequence = 0
        self._lock = threading.Lock()
        motion_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        estop_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.motion_pub = self.create_publisher(MotionCommand, "/hansel/system/command/motion", motion_qos)
        self.estop_pub = self.create_publisher(EmergencyStop, "/hansel/system/command/estop", estop_qos)
        self.head_pub = self.create_publisher(HeadServoCommand, "/hansel/head/command/front_servo", 10)
        self.detach_client = self.create_client(DetachUnit, "/hansel/detach/execute")
        self.enable_clients = {
            unit: self.create_client(SetDriveEnabled, f"/hansel/{unit}/set_drive_enabled")
            for unit in self.get_parameter("active_targets").value
        }
        self.create_timer(float(self.get_parameter("send_interval_s").value), self._publish_active)
        self.add_on_set_parameters_callback(self._on_parameters)
        threading.Thread(target=self._input_loop, daemon=True).start()

    def _next(self) -> int:
        with self._lock:
            self.sequence += 1
            return self.sequence

    def _publish_motion(self, command: str) -> None:
        msg = MotionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sequence = self._next()
        msg.command = command
        msg.speed_scale = float(self.speed_scale)
        msg.source = "keyboard"
        self.motion_pub.publish(msg)

    def _publish_active(self) -> None:
        self._publish_motion(self.active_command)

    def _head(self, command: int) -> None:
        msg = HeadServoCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sequence = self._next()
        msg.command = command
        msg.source = "keyboard"
        self.head_pub.publish(msg)

    def _estop(self, engaged: bool) -> None:
        msg = EmergencyStop()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sequence = self._next()
        msg.engaged = engaged
        msg.source = "keyboard"
        msg.reason = "operator keyboard"
        self.estop_pub.publish(msg)

    def _enable_all(self) -> None:
        for client in self.enable_clients.values():
            if client.service_is_ready():
                request = SetDriveEnabled.Request()
                request.enabled = True
                request.source = "keyboard"
                client.call_async(request)

    def _detach(self, unit: str) -> None:
        self.active_command = "stop"
        self._publish_active()
        if not self.detach_client.service_is_ready():
            self.get_logger().warning("detach coordinator unavailable")
            return
        request = DetachUnit.Request()
        request.released_unit_id = unit
        request.mode = DetachUnit.Request.MANUAL
        request.source = "keyboard"
        self.detach_client.call_async(request)

    def _handle_key(self, key: str) -> bool:
        key = key.lower()
        if key == "\x03":
            return False
        if key in LIVE_KEYS:
            self.active_command = LIVE_KEYS[key]
            self._publish_active()
            self.get_logger().info(f"command={self.active_command} speed={self.speed_scale:.2f}")
        elif key in ONE_SHOT_KEYS:
            self._head(ONE_SHOT_KEYS[key])
        elif key == "f":
            self._publish_motion("front_motor_forward")
        elif key == "v":
            self._publish_motion("front_motor_stop")
        elif key in DETACH_KEYS:
            self._detach(DETACH_KEYS[key])
        elif key == "!":
            self.active_command = "stop"; self._estop(True)
        elif key == "r":
            self.active_command = "stop"; self._estop(False)
        elif key == "g":
            self._enable_all()
        elif key == "p":
            return False
        return True

    def _input_loop(self) -> None:
        print(
            "HANSEL keys: W/S forward/back, A/D spin, Q/E forward curve, "
            "Z/C backward curve, X/Space stop, U/J head ±step, K center, "
            "F/V front motor, 1/2/3 detach, ! E-stop, R clear, G enable, P quit"
        )
        if termios is None or tty is None or not sys.stdin.isatty():
            self.get_logger().warning("interactive POSIX terminal required for single-key control")
            return
        fd = sys.stdin.fileno()
        previous = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready and not self._handle_key(sys.stdin.read(1)):
                    self.active_command = "stop"
                    self._publish_active()
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, previous)

    def _on_parameters(self, parameters):
        try:
            for parameter in parameters:
                if parameter.name == "speed_scale":
                    value = float(parameter.value)
                    if not 0.0 <= value <= 1.0:
                        raise ValueError("speed_scale must be within 0..1")
                    self.speed_scale = value
                elif parameter.name == "send_interval_s":
                    return SetParametersResult(
                        successful=False,
                        reason="send_interval_s requires node restart",
                    )
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OperatorInput()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
