"""Per-unit ROS 2 controller using HANSEL_MESH semantic motion commands."""

from __future__ import annotations

import math
import copy
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from hansel_interfaces.msg import (
    ActiveChain,
    EmergencyStop,
    HeadServoCommand,
    HeadAngleState,
    MotionCommand,
    UnitState,
    WheelState,
)
from hansel_interfaces.srv import PrepareDetach, SetDriveEnabled, TriggerDetach

from .core import (
    GitHubMotionMapper,
    HeadServoStepper,
    MotionInput,
    OperationState,
    PidController,
    RampLimiter,
    SafetyController,
    steering_pwm,
)
from .hardware import DummyHardwareAdapter, NanoSerialHardwareAdapter
from .rpi_gpio import GPIO_DEFAULTS, RaspberryPiHardwareAdapter


class UnitController(Node):
    def __init__(self) -> None:
        super().__init__("unit_controller")
        # Parameter services, drive-enable and control callbacks share one serialized group.
        self._callbacks = self.default_callback_group
        self._declare_parameters()

        self.unit_id = str(self.get_parameter("unit_id").value)
        self.role = str(self.get_parameter("role").value)
        self.control_period_s = float(self.get_parameter("control_period_s").value)
        self.publish_divisor = max(1, int(self.get_parameter("publish_divisor").value))
        self.encoder_counts_per_revolution = float(
            self.get_parameter("encoder_counts_per_revolution").value
        )

        self.mapper = self._new_mapper()
        self.safety = SafetyController(
            watchdog_requires_enable=bool(self.get_parameter("watchdog_requires_enable").value),
            command_timeout_s=float(self.get_parameter("command_timeout_s").value),
            max_command_age_s=float(self.get_parameter("max_command_age_s").value),
        )
        self.left_pid = self._new_pid("left")
        self.right_pid = self._new_pid("right")
        ramp = float(self.get_parameter("pwm_ramp_percent_per_second").value)
        self.left_ramp = RampLimiter(ramp)
        self.right_ramp = RampLimiter(ramp)
        self.head_stepper = HeadServoStepper(
            minimum_angle_deg=float(self.get_parameter("head_min_angle_deg").value),
            maximum_angle_deg=float(self.get_parameter("head_max_angle_deg").value),
            center_angle_deg=float(self.get_parameter("head_center_angle_deg").value),
            step_angle_deg=float(self.get_parameter("head_step_angle_deg").value),
        )
        self.hardware = self._create_hardware()
        self._push_servo_configuration()

        self.operation_state = OperationState.ACTIVE
        self.status_message = "ready"
        self.current_command = "stop"
        self.target_left_cps = 0.0
        self.target_right_cps = 0.0
        self.actual_left_cps = 0.0
        self.actual_right_cps = 0.0
        self.left_pwm = 0.0
        self.right_pwm = 0.0
        self.commanded_head_angle = self.head_stepper.current_angle_deg
        self._last_command_stamp = None
        self._last_control_time = time.monotonic()
        self._tick_count = 0
        self._detach_inhibit = False
        self._detach_hold_sequence = -1
        self._released_hold_sequence = -1
        self._io_error = ""
        self._hardware_retry_at = 0.0
        self._last_head_sequence_by_source: dict[str, int] = {}
        self._closed = False

        motion_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.wheel_publisher = self.create_publisher(WheelState, "state/wheels", 10)
        self.unit_publisher = self.create_publisher(UnitState, "state/unit", 10)
        self.angle_publisher = self.create_publisher(HeadAngleState, "state/front_angle", 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_subscription(
            MotionCommand, "command/motion", self._on_motion, motion_qos,
            callback_group=self._callbacks,
        )
        self.create_subscription(
            EmergencyStop, "/hansel/system/command/estop", self._on_estop,
            latched_qos, callback_group=self._callbacks,
        )
        self.create_subscription(
            ActiveChain, "/hansel/system/state/active_chain", self._on_active_chain,
            latched_qos, callback_group=self._callbacks,
        )
        if self.role == "head":
            self.create_subscription(
                HeadServoCommand, "command/front_servo",
                self._on_head_servo_command, 10, callback_group=self._callbacks,
            )

        self.create_service(
            PrepareDetach, "prepare_detach", self._on_prepare_detach,
            callback_group=self._callbacks,
        )
        self.create_service(
            TriggerDetach, "trigger_detach", self._on_trigger_detach,
            callback_group=self._callbacks,
        )
        self.create_service(
            SetDriveEnabled, "set_drive_enabled", self._on_set_drive_enabled,
            callback_group=self._callbacks,
        )
        self.create_timer(self.control_period_s, self._control_tick,
                          callback_group=self._callbacks)
        self.create_timer(1.0, self._publish_diagnostics,
                          callback_group=self._callbacks)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self.get_logger().info(
            f"unit={self.unit_id} role={self.role} "
            f"backend={self.get_parameter('hardware_backend').value}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "unit_id": "head",
            "role": "head",
            "hardware_backend": "dummy",
            "control_period_s": 0.05,
            "publish_divisor": 2,
            "command_timeout_s": 0.5,
            "watchdog_requires_enable": True,
            "max_command_age_s": 0.5,
            "straight_rpm": 120.0,
            "turn_rpm": 102.0,
            "full_speed_cps_left": 800.0,
            "full_speed_cps_right": 800.0,
            "turn_inner_ratio": 0.45,
            "turn_outer_ratio": 1.0,
            "mild_turn_inner_ratio": 0.75,
            "mild_turn_outer_ratio": 1.0,
            "spin_ratio": 0.85,
            "node_slow_ratio": 0.45,
            "left_target_scale": 1.0,
            "right_target_scale": 1.0,
            "left_kp": 0.035,
            "left_ki": 0.015,
            "left_kd": 0.0,
            "right_kp": 0.035,
            "right_ki": 0.015,
            "right_kd": 0.0,
            "pid_integral_limit": 500.0,
            "minimum_pwm_percent": 25.0,
            "maximum_pwm_percent": 75.0,
            "pwm_ramp_percent_per_second": 220.0,
            "dummy_response_time_s": 0.2,
            "encoder_counts_per_revolution": 400.0,
            "head_min_angle_deg": -180.0,
            "head_max_angle_deg": 180.0,
            "head_center_angle_deg": 0.0,
            "head_step_angle_deg": 5.0,
            "head_min_pulse_us": 500,
            "head_center_pulse_us": 1500,
            "head_max_pulse_us": 2500,
            "head_servo_hold": True,
            "detach_duration_s": 0.35,
            "detach_press_pulse_us": 1515,
            "detach_rest_pulse_us": 800,
            "nano_serial_port": "/dev/ttyUSB0",
            "nano_serial_baudrate": 115200,
            "nano_serial_timeout_s": 0.02,
            "front_follow_drive": True,
        }
        defaults.update(GPIO_DEFAULTS)
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    @staticmethod
    def _rpm_to_cps(rpm: float, counts_per_revolution: float) -> float:
        return float(rpm) * float(counts_per_revolution) / 60.0

    def _configured_speed_cps(self, value_getter=None) -> tuple[float, float]:
        get_value = value_getter or (lambda name: self.get_parameter(name).value)
        straight_rpm = float(get_value("straight_rpm"))
        turn_rpm = float(get_value("turn_rpm"))
        counts_per_revolution = float(get_value("encoder_counts_per_revolution"))
        if straight_rpm <= 0.0 or turn_rpm <= 0.0:
            raise ValueError("straight_rpm and turn_rpm must be positive")
        if counts_per_revolution <= 0.0:
            raise ValueError("encoder_counts_per_revolution must be positive")
        return (
            self._rpm_to_cps(straight_rpm, counts_per_revolution),
            self._rpm_to_cps(turn_rpm, counts_per_revolution),
        )

    def _new_mapper(self) -> GitHubMotionMapper:
        p = self.get_parameter
        straight_cps, turn_cps = self._configured_speed_cps()
        return GitHubMotionMapper(
            role=self.role,
            full_speed_cps_left=straight_cps,
            full_speed_cps_right=straight_cps,
            turn_inner_ratio=float(p("turn_inner_ratio").value),
            turn_outer_ratio=float(p("turn_outer_ratio").value),
            mild_turn_inner_ratio=float(p("mild_turn_inner_ratio").value),
            mild_turn_outer_ratio=float(p("mild_turn_outer_ratio").value),
            spin_ratio=float(p("spin_ratio").value),
            turn_speed_cps_left=turn_cps,
            turn_speed_cps_right=turn_cps,
            node_slow_ratio=float(p("node_slow_ratio").value),
            left_target_scale=float(p("left_target_scale").value),
            right_target_scale=float(p("right_target_scale").value),
        )

    def _new_pid(self, side: str) -> PidController:
        p = self.get_parameter
        straight_cps, turn_cps = self._configured_speed_cps()
        return PidController(
            float(p(f"{side}_kp").value),
            float(p(f"{side}_ki").value),
            float(p(f"{side}_kd").value),
            output_limit=float(p("maximum_pwm_percent").value),
            integral_limit=float(p("pid_integral_limit").value),
            minimum_output=float(p("minimum_pwm_percent").value),
            maximum_target=max(straight_cps, turn_cps),
        )

    def _create_hardware(self):
        backend = str(self.get_parameter("hardware_backend").value)
        p = self.get_parameter
        if backend == "dummy":
            straight_cps, turn_cps = self._configured_speed_cps()
            maximum_cps = max(straight_cps, turn_cps)
            return DummyHardwareAdapter(
                maximum_cps,
                maximum_cps,
                float(p("dummy_response_time_s").value),
            )
        if backend == "nano_serial":
            return NanoSerialHardwareAdapter(
                port=str(p("nano_serial_port").value),
                baudrate=int(p("nano_serial_baudrate").value),
                read_timeout_s=float(p("nano_serial_timeout_s").value),
                front_follow_drive=bool(p("front_follow_drive").value),
                motor_pwm_limit_percent=float(p("maximum_pwm_percent").value),
            )
        if backend == "rpi_gpio":
            if self.role != "rear":
                raise ValueError("rpi_gpio is for rear units only; head uses nano_serial")
            options = {name.removeprefix("gpio_"): p(name).value for name in GPIO_DEFAULTS}
            return RaspberryPiHardwareAdapter(
                **options, motor_pwm_limit_percent=float(p("maximum_pwm_percent").value)
            )
        raise ValueError(f"unsupported hardware_backend: {backend}")

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _on_motion(self, msg: MotionCommand) -> None:
        if self._detach_inhibit:
            return
        wall_now = self.get_clock().now().nanoseconds / 1e9
        monotonic_now = time.monotonic()
        motion = MotionInput(
            stamp_seconds=self._stamp_seconds(msg.stamp),
            sequence=int(msg.sequence),
            command=str(msg.command),
            speed_scale=float(msg.speed_scale),
            source=str(msg.source),
        )
        accepted, reason = self.safety.accept_motion(motion, wall_now, monotonic_now)
        if not accepted:
            self.get_logger().warning(f"motion rejected: {reason}")
            return
        normalized = self.mapper.normalize_command(msg.command)
        if self.role == "head" and normalized in {
            "front_motor_forward", "front_motor_backward", "front_motor_stop"
        }:
            self._clear_drive_targets()
            self.current_command = normalized
            self.target_left_cps = self.target_right_cps = 0.0
            try:
                if not self._io_error:
                    self.hardware.set_front_motor(normalized, float(msg.speed_scale))
            except OSError as exc:
                self._record_io_error(exc)
            self.status_message = f"{normalized} speed={msg.speed_scale:.2f}"
            return

        self.hardware.clear_front_motor()
        targets = self.mapper.map(normalized, msg.speed_scale)
        self.current_command = targets.command
        self.target_left_cps = targets.left_cps
        self.target_right_cps = targets.right_cps
        fixed_pwm = steering_pwm(targets.command, self.left_pid.output_limit)
        if fixed_pwm is not None:
            # Open-loop steering requests PWM, not a closed-loop CPS setpoint.
            self.target_left_cps = self.target_right_cps = 0.0
        self._last_command_stamp = msg.stamp
        self.operation_state = (
            OperationState.STOPPED
            if fixed_pwm is None and targets.left_cps == 0.0 and targets.right_cps == 0.0
            else OperationState.ACTIVE
        )
        self.status_message = f"command={targets.command} speed={msg.speed_scale:.2f}"
        if fixed_pwm is not None:
            self.status_message = f"command={targets.command} open-loop PWM={fixed_pwm}"

    def _on_head_servo_command(self, msg: HeadServoCommand) -> None:
        if self._io_error:
            self.get_logger().warning("head servo command skipped during hardware reconnect")
            return
        source = msg.source or "unknown"
        previous = self._last_head_sequence_by_source.get(source)
        if previous is not None and msg.sequence <= previous:
            return
        self._last_head_sequence_by_source[source] = int(msg.sequence)
        if msg.command == HeadServoCommand.UP_STEP:
            angle = self.head_stepper.up()
        elif msg.command == HeadServoCommand.DOWN_STEP:
            angle = self.head_stepper.down()
        elif msg.command == HeadServoCommand.CENTER:
            angle = self.head_stepper.center()
        elif msg.command == HeadServoCommand.MIN_LIMIT:
            angle = self.head_stepper.minimum()
        elif msg.command == HeadServoCommand.MAX_LIMIT:
            angle = self.head_stepper.maximum()
        else:
            return
        self.commanded_head_angle = angle
        try:
            self.hardware.set_head_angle(angle)
        except OSError as exc:
            self._record_io_error(exc)
        self._publish_head_state()

    def _on_estop(self, msg: EmergencyStop) -> None:
        self.safety.set_estop(msg.engaged)
        if msg.engaged:
            self._force_zero_outputs(disable=True)
            self.operation_state = OperationState.ESTOP
            self.status_message = msg.reason or "E-stop"
        else:
            self.operation_state = OperationState.STOPPED
            self.status_message = "E-stop cleared; explicit enable required"

    def _on_active_chain(self, msg: ActiveChain) -> None:
        if self.unit_id in msg.relay_assumed_units:
            self.safety.set_relay_assumed()
            self._force_zero_outputs(disable=True)
            self.operation_state = OperationState.RELAY_ASSUMED
            self.status_message = "relay assumed; propulsion permanently disabled"

    def _on_prepare_detach(self, request, response):
        sequence = int(request.request_sequence)
        if request.hold:
            if sequence > self._released_hold_sequence and sequence >= self._detach_hold_sequence:
                self._detach_hold_sequence = sequence
                self._detach_inhibit = True
        else:
            self._released_hold_sequence = max(self._released_hold_sequence, sequence)
            if sequence >= self._detach_hold_sequence:
                self._detach_inhibit = False
        # A temporary hold must not clear the operator's enable state.
        response.stopped = self._force_zero_outputs()
        response.message = "safe-stop hold active" if request.hold else "safe-stop hold released"
        return response

    def _on_trigger_detach(self, request, response):
        if self.safety.relay_assumed:
            response.accepted = False
            response.command_completed = False
            response.message = "released unit cannot actuate another detach"
            return response
        if not self._force_zero_outputs():
            response.accepted = False
            response.command_completed = False
            response.message = "could not stop actuator unit before detach"
            return response
        self.operation_state = OperationState.DETACHING
        response.accepted = True
        try:
            self.hardware.trigger_detach(
                float(self.get_parameter("detach_duration_s").value),
                int(self.get_parameter("detach_press_pulse_us").value),
                int(self.get_parameter("detach_rest_pulse_us").value),
            )
            self.hardware.wait_detach(timeout_s=3.0)
            response.accepted = True
            response.command_completed = True
            response.message = f"servo press/rest completed for {request.released_unit_id}; physical release unverified"
            self.operation_state = OperationState.STOPPED
        except Exception as exc:
            response.command_completed = False
            response.message = str(exc)
            self.status_message = f"detach completion unconfirmed: {exc}"
            self.operation_state = OperationState.STOPPED
        return response

    def _on_set_drive_enabled(self, request, response):
        if request.enabled:
            accepted, message = self.safety.request_drive_enable()
        else:
            self.safety.drive_enabled = False
            self._force_zero_outputs(disable=True)
            accepted, message = True, "drive disabled"
        response.accepted = accepted
        response.message = message
        if accepted:
            self.operation_state = OperationState.STOPPED
        return response

    def _control_tick(self) -> None:
        now = time.monotonic()
        dt = max(1e-4, now - self._last_control_time)
        self._last_control_time = now
        if self.safety.tick(now):
            self._force_zero_outputs(disable=self.safety.watchdog_requires_enable)
            self.operation_state = OperationState.STOPPED
            self.status_message = ("command watchdog timeout; enable required"
                                   if self.safety.watchdog_requires_enable
                                   else "command watchdog timeout; waiting for fresh command")

        if self._io_error:
            if now >= self._hardware_retry_at:
                self._hardware_retry_at = now + 1.0
                try:
                    replacement = self.hardware.recover()
                    if replacement is not None:
                        self.hardware = replacement
                    self.hardware.update(0.0, 0.0, False, dt)
                    self._io_error = ""
                    self._clear_drive_targets()
                    self.status_message = "hardware recovered; waiting for fresh command"
                    self.operation_state = OperationState.STOPPED
                except OSError as exc:
                    self._io_error = str(exc)
                    self.status_message = f"hardware retry: {exc}"
            self._tick_count += 1
            if self._tick_count % self.publish_divisor == 0:
                self._publish_state()
            return

        if self.safety.drive_allowed and not self._detach_inhibit:
            fixed = steering_pwm(self.current_command, self.left_pid.output_limit)
            if fixed is not None:
                self.left_pid.reset(); self.right_pid.reset()
                left_requested, right_requested = fixed
            else:
                left_mag = self.left_pid.step(self.target_left_cps, self.actual_left_cps, dt)
                right_mag = self.right_pid.step(self.target_right_cps, self.actual_right_cps, dt)
                left_requested = math.copysign(left_mag, self.target_left_cps) if self.target_left_cps else 0.0
                right_requested = math.copysign(right_mag, self.target_right_cps) if self.target_right_cps else 0.0
            self.left_pwm = self.left_ramp.step(left_requested, dt)
            self.right_pwm = self.right_ramp.step(right_requested, dt)
        else:
            self.left_pid.reset(); self.right_pid.reset()
            self.left_ramp.reset(); self.right_ramp.reset()
            self.left_pwm = self.right_pwm = 0.0

        try:
            self.actual_left_cps, self.actual_right_cps = self.hardware.update(
                self.left_pwm, self.right_pwm, self.safety.drive_allowed and not self._detach_inhibit, dt
            )
        except OSError as exc:
            self._record_io_error(exc)
        except Exception as exc:
            self.safety.set_fault()
            self.operation_state = OperationState.FAULT
            self.status_message = f"hardware error: {exc}"
            self.left_pwm = self.right_pwm = 0.0

        self._tick_count += 1
        if self._tick_count % self.publish_divisor == 0:
            self._publish_state()

    def _clear_drive_targets(self) -> None:
        self.current_command = "stop"
        self.target_left_cps = self.target_right_cps = 0.0
        self.left_pwm = self.right_pwm = 0.0
        self.left_pid.reset(); self.right_pid.reset()
        self.left_ramp.reset(); self.right_ramp.reset()
        self.hardware.clear_front_motor()

    def _record_io_error(self, exc: Exception) -> None:
        if not self._io_error:
            self._hardware_retry_at = time.monotonic() + 1.0
        self._io_error = str(exc)
        self._clear_drive_targets()
        self.actual_left_cps = self.actual_right_cps = 0.0
        self.operation_state = OperationState.STOPPED
        self.status_message = f"hardware retry: {exc}"

    def _force_zero_outputs(self, disable: bool = False) -> bool:
        self._clear_drive_targets()
        if disable:
            self.safety.drive_enabled = False
        try:
            self.hardware.update(0.0, 0.0, False, self.control_period_s)
            return True
        except Exception as exc:
            self.status_message = f"could not write motor stop: {exc}"
            return False

    def _cps_to_rpm(self, cps: float) -> float:
        if self.encoder_counts_per_revolution <= 0:
            return 0.0
        return float(cps) * 60.0 / self.encoder_counts_per_revolution

    def _publish_state(self) -> None:
        stamp = self.get_clock().now().to_msg()
        wheel = WheelState()
        wheel.stamp = stamp
        wheel.command = self.current_command
        wheel.target_left_cps = float(self.target_left_cps)
        wheel.target_right_cps = float(self.target_right_cps)
        wheel.actual_left_cps = float(self.actual_left_cps)
        wheel.actual_right_cps = float(self.actual_right_cps)
        wheel.target_left_rpm = self._cps_to_rpm(self.target_left_cps)
        wheel.target_right_rpm = self._cps_to_rpm(self.target_right_cps)
        wheel.actual_left_rpm = self._cps_to_rpm(self.actual_left_cps)
        wheel.actual_right_rpm = self._cps_to_rpm(self.actual_right_cps)
        wheel.pwm_left = float(self.left_pwm)
        wheel.pwm_right = float(self.right_pwm)
        wheel.drive_enabled = self.safety.drive_allowed and not self._detach_inhibit and not self._io_error
        sample = self.hardware.encoder_sample if not self._io_error else None
        wheel.encoder_counts_valid = sample is not None
        if sample is not None:
            wheel.encoder_epoch = sample.epoch
            wheel.encoder_sample_id = sample.sequence
            wheel.left_encoder_count = sample.left
            wheel.right_encoder_count = sample.right
        self.wheel_publisher.publish(wheel)

        state = UnitState()
        state.stamp = stamp
        state.unit_id = self.unit_id
        state.role = UnitState.ROLE_HEAD if self.role == "head" else UnitState.ROLE_REAR
        state.operation_state = int(self.operation_state)
        state.active_for_drive = self.safety.drive_allowed and not self._detach_inhibit and not self._io_error
        if self._last_command_stamp is not None:
            state.last_command_stamp = self._last_command_stamp
        state.status_message = self.status_message
        self.unit_publisher.publish(state)
        if self.role == "head":
            self._publish_head_state()

    def _publish_head_state(self) -> None:
        msg = HeadAngleState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.commanded_angle_deg = float(self.commanded_head_angle)
        msg.moving = self.hardware.head_moving
        self.angle_publisher.publish(msg)

    def _publish_diagnostics(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = f"hansel/{self.unit_id}/unit_controller"
        status.hardware_id = self.unit_id
        status.level = DiagnosticStatus.ERROR if self.operation_state == OperationState.FAULT else DiagnosticStatus.OK
        if self._io_error:
            status.level = DiagnosticStatus.WARN
        status.message = self.status_message
        values = {
            "command": self.current_command,
            "drive_enabled": str(self.safety.drive_allowed).lower(),
            "target_left_cps": f"{self.target_left_cps:.2f}",
            "target_right_cps": f"{self.target_right_cps:.2f}",
            **self.hardware.diagnostic_items(),
        }
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        msg.status = [status]
        self.diagnostics_publisher.publish(msg)

    def _push_servo_configuration(self) -> None:
        configure = getattr(self.hardware, "configure_servo", None)
        if configure is None:
            return
        p = self.get_parameter
        configure(
            min_angle=float(p("head_min_angle_deg").value),
            center_angle=float(p("head_center_angle_deg").value),
            max_angle=float(p("head_max_angle_deg").value),
            min_pulse_us=int(p("head_min_pulse_us").value),
            center_pulse_us=int(p("head_center_pulse_us").value),
            max_pulse_us=int(p("head_max_pulse_us").value),
            hold=bool(p("head_servo_hold").value),
        )

    def _on_parameters_changed(self, parameters):
        values = {parameter.name: parameter.value for parameter in parameters}
        try:
            restart_only = {"watchdog_requires_enable", "hardware_backend", "nano_serial_port",
                            "nano_serial_baudrate", "nano_serial_timeout_s"} | set(GPIO_DEFAULTS)
            if restart_only & values.keys():
                raise ValueError("hardware backend/serial settings require node restart")
            mapper_names = {
                "straight_rpm", "turn_rpm", "turn_inner_ratio", "turn_outer_ratio",
                "mild_turn_inner_ratio", "mild_turn_outer_ratio", "spin_ratio",
                "node_slow_ratio", "left_target_scale", "right_target_scale",
                "encoder_counts_per_revolution",
            }
            pid_names = {"left_kp", "left_ki", "left_kd", "right_kp", "right_ki", "right_kd",
                         "pid_integral_limit", "minimum_pwm_percent", "maximum_pwm_percent"}
            head_names = {"head_min_angle_deg", "head_center_angle_deg",
                          "head_max_angle_deg", "head_step_angle_deg"}
            servo_names = head_names | {"head_min_pulse_us", "head_center_pulse_us",
                                        "head_max_pulse_us", "head_servo_hold"}
            actuator_names = mapper_names | pid_names | servo_names | {
                "pwm_ramp_percent_per_second", "front_follow_drive", "detach_duration_s",
                "detach_press_pulse_us", "detach_rest_pulse_us",
            }
            if actuator_names & values.keys() and self.safety.drive_enabled:
                raise ValueError("Disable drive before changing actuator parameters")
            for name in actuator_names & values.keys():
                item = values[name]
                if isinstance(item, (int, float)) and not isinstance(item, bool) and not math.isfinite(item):
                    raise ValueError(f"{name} must be finite")

            def value(name):
                return values.get(name, self.get_parameter(name).value)

            # Build and validate every candidate before touching the active controller.
            mapper, left_pid, right_pid = self.mapper, self.left_pid, self.right_pid
            left_ramp, right_ramp = self.left_ramp, self.right_ramp
            stepper = self.head_stepper
            speed_changed = bool(mapper_names & values.keys())
            pid_changed = speed_changed or bool(pid_names & values.keys())
            maximum_target = None
            if speed_changed:
                straight, turn = self._configured_speed_cps(value)
                mapper = GitHubMotionMapper(
                    role=self.role, full_speed_cps_left=straight, full_speed_cps_right=straight,
                    turn_speed_cps_left=turn, turn_speed_cps_right=turn,
                    **{name:float(value(name)) for name in mapper_names - {
                        "straight_rpm", "turn_rpm", "encoder_counts_per_revolution"}},
                )
            if pid_changed:
                minimum, maximum = float(value("minimum_pwm_percent")), float(value("maximum_pwm_percent"))
                if not 0 <= minimum <= maximum <= 100 or maximum <= 0:
                    raise ValueError("PWM must satisfy 0 <= minimum <= maximum <= 100, maximum > 0")
                maximum_target = max(self._configured_speed_cps(value))
                def make_pid(side):
                    return PidController(
                        value(f"{side}_kp"), value(f"{side}_ki"), value(f"{side}_kd"),
                        output_limit=maximum, integral_limit=value("pid_integral_limit"),
                        minimum_output=minimum, maximum_target=maximum_target,
                    )
                left_pid, right_pid = make_pid("left"), make_pid("right")
            if "pwm_ramp_percent_per_second" in values:
                ramp = float(value("pwm_ramp_percent_per_second"))
                if ramp <= 0:
                    raise ValueError("PWM ramp must be positive")
                left_ramp, right_ramp = RampLimiter(ramp), RampLimiter(ramp)
            if head_names & values.keys():
                low, center, high = (float(value(n)) for n in
                                     ("head_min_angle_deg", "head_center_angle_deg", "head_max_angle_deg"))
                if not -180 <= low < center < high <= 180:
                    raise ValueError("Head limits must satisfy -180 <= min < center < max <= 180")
                stepper = copy.copy(self.head_stepper)
                stepper.reconfigure(minimum_angle_deg=low, maximum_angle_deg=high,
                                    center_angle_deg=center, step_angle_deg=float(value("head_step_angle_deg")))
            servo_configuration = None
            if servo_names & values.keys():
                low, center, high = (int(value(n)) for n in
                                     ("head_min_pulse_us", "head_center_pulse_us", "head_max_pulse_us"))
                if not 400 <= low < center < high <= 2600:
                    raise ValueError("Head pulses must satisfy 400 <= min < center < max <= 2600")
                servo_configuration = dict(
                    min_angle=float(value("head_min_angle_deg")),
                    center_angle=float(value("head_center_angle_deg")),
                    max_angle=float(value("head_max_angle_deg")),
                    min_pulse_us=low, center_pulse_us=center, max_pulse_us=high,
                    hold=bool(value("head_servo_hold")),
                )
            if {"detach_duration_s", "detach_press_pulse_us", "detach_rest_pulse_us"} & values.keys():
                if not 0.02 <= float(value("detach_duration_s")) <= 5.0:
                    raise ValueError("Detach duration must be 0.02..5 seconds")
                if any(not 500 <= int(value(n)) <= 2500 for n in ("detach_press_pulse_us", "detach_rest_pulse_us")):
                    raise ValueError("Detach pulses must be 500..2500 us")

            if servo_configuration is not None:
                self.hardware.configure_servo(**servo_configuration)
            if "maximum_pwm_percent" in values and isinstance(
                self.hardware, (NanoSerialHardwareAdapter, RaspberryPiHardwareAdapter)
            ):
                self.hardware.set_motor_pwm_limit(value("maximum_pwm_percent"))
            self.mapper, self.left_pid, self.right_pid = mapper, left_pid, right_pid
            self.left_ramp, self.right_ramp = left_ramp, right_ramp
            self.head_stepper = stepper
            self.commanded_head_angle = stepper.current_angle_deg
            if maximum_target is not None and isinstance(self.hardware, DummyHardwareAdapter):
                self.hardware.full_speed_cps_left = maximum_target
                self.hardware.full_speed_cps_right = maximum_target
            if "encoder_counts_per_revolution" in values:
                self.encoder_counts_per_revolution = float(value("encoder_counts_per_revolution"))
            if "front_follow_drive" in values and hasattr(self.hardware, "front_follow_drive"):
                self.hardware.front_follow_drive = bool(value("front_follow_drive"))
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._force_zero_outputs(disable=True)
        self.hardware.close()

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitController()
    executor = MultiThreadedExecutor(num_threads=4)
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
