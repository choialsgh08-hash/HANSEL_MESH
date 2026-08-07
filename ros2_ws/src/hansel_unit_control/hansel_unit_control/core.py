"""ROS-independent HANSEL control primitives.

The semantic command names, steering ratios, rear-unit normalization and
feed-forward + PID output shape intentionally follow HANSEL_MESH.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


class OperationState(IntEnum):
    INITIALIZING = 0
    STOPPED = 1
    ACTIVE = 2
    DETACHING = 3
    RELAY_ASSUMED = 4
    ESTOP = 5
    FAULT = 6


DRIVE_COMMANDS = {
    "forward", "backward", "left", "right",
    "forward_left", "forward_right", "backward_left", "backward_right",
    "mild_forward_left", "mild_forward_right",
    "mild_backward_left", "mild_backward_right",
    "slow_forward", "slow_backward",
}

HEAD_ONLY_COMMANDS = {
    "head_servo_up", "head_servo_down", "head_servo_center",
    "head_servo_min", "head_servo_max",
    "front_motor_forward", "front_motor_backward", "front_motor_stop",
}


@dataclass(frozen=True)
class MotionInput:
    stamp_seconds: float
    sequence: int
    command: str
    speed_scale: float
    source: str


@dataclass(frozen=True)
class WheelTargets:
    left_cps: float
    right_cps: float
    command: str


class GitHubMotionMapper:
    """Map HANSEL_MESH semantic commands to signed wheel CPS targets."""

    def __init__(
        self,
        role: str,
        full_speed_cps_left: float = 800.0,
        full_speed_cps_right: float = 800.0,
        turn_inner_ratio: float = 0.45,
        turn_outer_ratio: float = 1.0,
        mild_turn_inner_ratio: float = 0.75,
        mild_turn_outer_ratio: float = 1.0,
        spin_ratio: float = 0.85,
        turn_speed_cps_left: float | None = None,
        turn_speed_cps_right: float | None = None,
        node_slow_ratio: float = 0.45,
        left_target_scale: float = 1.0,
        right_target_scale: float = 1.0,
    ) -> None:
        if role not in {"head", "rear"}:
            raise ValueError(f"unsupported role: {role}")
        if full_speed_cps_left <= 0 or full_speed_cps_right <= 0:
            raise ValueError("full speed CPS values must be positive")
        for name, value in {
            "turn_inner_ratio": turn_inner_ratio,
            "turn_outer_ratio": turn_outer_ratio,
            "mild_turn_inner_ratio": mild_turn_inner_ratio,
            "mild_turn_outer_ratio": mild_turn_outer_ratio,
            "spin_ratio": spin_ratio,
            "node_slow_ratio": node_slow_ratio,
        }.items():
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be within 0..1")
        self.role = role
        self.full_speed_cps_left = float(full_speed_cps_left)
        self.full_speed_cps_right = float(full_speed_cps_right)
        self.turn_inner_ratio = float(turn_inner_ratio)
        self.turn_outer_ratio = float(turn_outer_ratio)
        self.mild_turn_inner_ratio = float(mild_turn_inner_ratio)
        self.mild_turn_outer_ratio = float(mild_turn_outer_ratio)
        self.spin_ratio = float(spin_ratio)
        self.turn_speed_cps_left = (
            self.full_speed_cps_left * self.spin_ratio
            if turn_speed_cps_left is None else float(turn_speed_cps_left)
        )
        self.turn_speed_cps_right = (
            self.full_speed_cps_right * self.spin_ratio
            if turn_speed_cps_right is None else float(turn_speed_cps_right)
        )
        if self.turn_speed_cps_left <= 0.0 or self.turn_speed_cps_right <= 0.0:
            raise ValueError("turn speed CPS values must be positive")
        self.node_slow_ratio = float(node_slow_ratio)
        self.left_target_scale = float(left_target_scale)
        self.right_target_scale = float(right_target_scale)

    @staticmethod
    def normalize_command(command: str) -> str:
        return command.strip().lower().replace("-", "_").replace(" ", "_")

    @classmethod
    def normalize_for_role(cls, command: str, role: str) -> str:
        command = cls.normalize_command(command)
        if role == "head":
            return command
        if command in {
            "forward_left", "forward_right",
            "mild_forward_left", "mild_forward_right",
        }:
            return "slow_forward"
        if command in {
            "backward_left", "backward_right",
            "mild_backward_left", "mild_backward_right",
        }:
            return "slow_backward"
        if command in {"left", "right"} or command in HEAD_ONLY_COMMANDS:
            return "stop"
        return command

    def map(self, command: str, speed_scale: float) -> WheelTargets:
        command = self.normalize_for_role(command, self.role)
        speed = max(0.0, min(1.0, float(speed_scale)))

        def left(ratio: float, sign: float = 1.0) -> float:
            return sign * self.full_speed_cps_left * ratio * speed * self.left_target_scale

        def right(ratio: float, sign: float = 1.0) -> float:
            return sign * self.full_speed_cps_right * ratio * speed * self.right_target_scale

        def turn_left(sign: float = 1.0) -> float:
            return sign * self.turn_speed_cps_left * speed * self.left_target_scale

        def turn_right(sign: float = 1.0) -> float:
            return sign * self.turn_speed_cps_right * speed * self.right_target_scale

        table = {
            "stop": (0.0, 0.0),
            "forward": (left(1.0), right(1.0)),
            "backward": (left(1.0, -1.0), right(1.0, -1.0)),
            "left": (turn_left(-1.0), turn_right(1.0)),
            "right": (turn_left(1.0), turn_right(-1.0)),
            "forward_left": (left(self.turn_inner_ratio), right(self.turn_outer_ratio)),
            "forward_right": (left(self.turn_outer_ratio), right(self.turn_inner_ratio)),
            "backward_left": (left(self.turn_outer_ratio, -1.0), right(self.turn_inner_ratio, -1.0)),
            "backward_right": (left(self.turn_inner_ratio, -1.0), right(self.turn_outer_ratio, -1.0)),
            "mild_forward_left": (left(self.mild_turn_inner_ratio), right(self.mild_turn_outer_ratio)),
            "mild_forward_right": (left(self.mild_turn_outer_ratio), right(self.mild_turn_inner_ratio)),
            "mild_backward_left": (left(self.mild_turn_outer_ratio, -1.0), right(self.mild_turn_inner_ratio, -1.0)),
            "mild_backward_right": (left(self.mild_turn_inner_ratio, -1.0), right(self.mild_turn_outer_ratio, -1.0)),
            "slow_forward": (left(self.node_slow_ratio), right(self.node_slow_ratio)),
            "slow_backward": (left(self.node_slow_ratio, -1.0), right(self.node_slow_ratio, -1.0)),
        }
        values = table.get(command)
        if values is None:
            values = (0.0, 0.0)
            command = "stop"
        return WheelTargets(values[0], values[1], command)


class PidController:
    """HANSEL_MESH minimum-PWM feed-forward plus PID correction."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = 100.0,
        integral_limit: float = 500.0,
        minimum_output: float = 25.0,
        maximum_target: float = 800.0,
    ) -> None:
        if output_limit <= 0 or maximum_target <= 0:
            raise ValueError("PID output and target limits must be positive")
        if not 0 <= minimum_output <= output_limit:
            raise ValueError("minimum_output must be within output range")
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = float(output_limit)
        self.integral_limit = float(integral_limit)
        self.minimum_output = float(minimum_output)
        self.maximum_target = float(maximum_target)
        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0

    def step(self, target_cps: float, measured_cps: float, dt: float) -> float:
        target = abs(float(target_cps))
        if target <= 0.0 or dt <= 0.0:
            self.reset()
            return 0.0
        target = min(target, self.maximum_target)
        measured = abs(float(measured_cps))
        error = target - measured
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * dt),
        )
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        feed_forward = self.minimum_output + (
            target / self.maximum_target
        ) * (self.output_limit - self.minimum_output)
        requested = feed_forward + self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(0.0, min(self.output_limit, requested))


class RampLimiter:
    def __init__(self, max_change_per_second: float) -> None:
        if max_change_per_second <= 0:
            raise ValueError("max_change_per_second must be positive")
        self.max_change_per_second = float(max_change_per_second)
        self.value = 0.0

    def reset(self, value: float = 0.0) -> None:
        self.value = float(value)

    def step(self, target: float, dt: float) -> float:
        max_delta = self.max_change_per_second * max(0.0, dt)
        self.value += max(-max_delta, min(max_delta, float(target) - self.value))
        return self.value


class HeadServoStepper:
    """GitHub-style one-shot U/J/K stepping with a -180..+180 logical range."""

    ABSOLUTE_MIN = -180.0
    ABSOLUTE_MAX = 180.0

    def __init__(self, minimum_angle_deg=-180.0, maximum_angle_deg=180.0,
                 center_angle_deg=0.0, step_angle_deg=2.0) -> None:
        self.reconfigure(
            minimum_angle_deg=minimum_angle_deg,
            maximum_angle_deg=maximum_angle_deg,
            center_angle_deg=center_angle_deg,
            step_angle_deg=step_angle_deg,
            keep_current=False,
        )

    def reconfigure(self, *, minimum_angle_deg: float, maximum_angle_deg: float,
                    center_angle_deg: float, step_angle_deg: float,
                    keep_current: bool = True) -> float:
        minimum = max(self.ABSOLUTE_MIN, float(minimum_angle_deg))
        maximum = min(self.ABSOLUTE_MAX, float(maximum_angle_deg))
        center = float(center_angle_deg)
        step = float(step_angle_deg)
        if minimum >= maximum:
            raise ValueError("head minimum angle must be below maximum")
        if not minimum <= center <= maximum:
            raise ValueError("head center angle must be inside limits")
        if step <= 0:
            raise ValueError("head step angle must be positive")
        previous = getattr(self, "current_angle_deg", center)
        self.minimum_angle_deg = minimum
        self.maximum_angle_deg = maximum
        self.center_angle_deg = center
        self.step_angle_deg = step
        self.current_angle_deg = self._clamp(previous if keep_current else center)
        return self.current_angle_deg

    def _clamp(self, value: float) -> float:
        return max(self.minimum_angle_deg, min(self.maximum_angle_deg, float(value)))

    def set_angle(self, value: float) -> float:
        self.current_angle_deg = self._clamp(value)
        return self.current_angle_deg

    def up(self) -> float:
        return self.set_angle(self.current_angle_deg + self.step_angle_deg)

    def down(self) -> float:
        return self.set_angle(self.current_angle_deg - self.step_angle_deg)

    def center(self) -> float:
        return self.set_angle(self.center_angle_deg)

    def minimum(self) -> float:
        return self.set_angle(self.minimum_angle_deg)

    def maximum(self) -> float:
        return self.set_angle(self.maximum_angle_deg)


class SafetyController:
    def __init__(self, command_timeout_s: float = 0.5, max_command_age_s: float = 0.5) -> None:
        self.command_timeout_s = float(command_timeout_s)
        self.max_command_age_s = float(max_command_age_s)
        self.drive_enabled = True
        self.estop_latched = False
        self.fault_latched = False
        self.relay_assumed = False
        self.timed_out = False
        self.last_receive_monotonic: float | None = None
        self.last_sequence_by_source: dict[str, int] = {}

    def accept_motion(self, command: MotionInput, wall_now: float, monotonic_now: float):
        source = command.source or "unknown"
        if self.estop_latched:
            return False, "E-stop is latched"
        if self.fault_latched:
            return False, "fault is latched"
        if self.relay_assumed:
            return False, "unit is relay-assumed"
        if not self.drive_enabled:
            return False, "explicit drive enable required"
        age = wall_now - command.stamp_seconds
        if age > self.max_command_age_s or age < -self.max_command_age_s:
            return False, "stale or future-dated command"
        previous = self.last_sequence_by_source.get(source)
        if previous is not None and command.sequence <= previous:
            return False, "non-increasing sequence"
        self.last_sequence_by_source[source] = int(command.sequence)
        self.last_receive_monotonic = float(monotonic_now)
        self.timed_out = False
        return True, "accepted"

    def tick(self, monotonic_now: float) -> bool:
        if self.last_receive_monotonic is None or not self.drive_enabled:
            return False
        if monotonic_now - self.last_receive_monotonic <= self.command_timeout_s:
            return False
        self.timed_out = True
        self.drive_enabled = False
        return True

    @property
    def drive_allowed(self) -> bool:
        return self.drive_enabled and not (
            self.estop_latched or self.fault_latched or self.relay_assumed
        )

    def set_estop(self, engaged: bool) -> None:
        self.estop_latched = bool(engaged)
        if engaged:
            self.drive_enabled = False

    def set_fault(self) -> None:
        self.fault_latched = True
        self.drive_enabled = False

    def set_relay_assumed(self) -> None:
        self.relay_assumed = True
        self.drive_enabled = False

    def request_drive_enable(self):
        if self.estop_latched:
            return False, "clear E-stop first"
        if self.fault_latched:
            return False, "fault remains latched"
        if self.relay_assumed:
            return False, "detached unit cannot be re-enabled"
        self.drive_enabled = True
        self.timed_out = False
        self.last_receive_monotonic = None
        return True, "drive enabled"
