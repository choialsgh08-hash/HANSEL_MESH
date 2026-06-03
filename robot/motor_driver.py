#!/usr/bin/env python3
"""GPIO motor, encoder, and servo control for HANSEL_MESH units.

The pin map and PID defaults are based on the previous HANSEL_GRETEL
Head/Node control scripts, but the network server is kept separate so mesh
traffic stays end-to-end and the hardware layer is easier to test.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

try:
    import RPi.GPIO as GPIO
except ImportError:  # Allows laptop-side syntax tests.
    GPIO = None

try:
    import pigpio
except ImportError:
    pigpio = None


def env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def clamp_pwm(value: float) -> float:
    return clamp(value, 0.0, 100.0)


def clamp_angle(angle: float, min_angle: int = 0, max_angle: int = 180) -> int:
    return int(clamp(angle, min_angle, max_angle))


@dataclass(frozen=True)
class MotorPins:
    ena: int
    in1: int
    in2: int
    enb: int
    in3: int
    in4: int


@dataclass(frozen=True)
class EncoderPins:
    left_a: int
    left_b: int
    right_a: int
    right_b: int


@dataclass(frozen=True)
class PidGains:
    kp_left: float = 0.035
    ki_left: float = 0.015
    kd_left: float = 0.0
    kp_right: float = 0.035
    ki_right: float = 0.015
    kd_right: float = 0.0


@dataclass(frozen=True)
class RoleConfig:
    role: str
    drive_pins: MotorPins
    encoder_pins: EncoderPins
    detach_servo_pin: Optional[int]
    head_servo_pin: Optional[int]
    front_motor_pins: Optional[MotorPins]
    full_speed_cps_left: float
    full_speed_cps_right: float
    default_speed_scale: float
    node_slow_ratio: float


DEFAULT_DRIVE_PINS = MotorPins(18, 23, 24, 13, 27, 22)
DEFAULT_ENCODER_PINS = EncoderPins(20, 21, 16, 26)
DEFAULT_FRONT_MOTOR_PINS = MotorPins(12, 3, 8, 19, 5, 7)


def role_config(role: str) -> RoleConfig:
    normalized = role.lower()
    if normalized not in {"head", "node1", "node2", "node3"}:
        raise ValueError(f"unsupported role for motor controller: {role}")

    is_head = normalized == "head"
    default_scale = 1.0

    return RoleConfig(
        role=normalized,
        drive_pins=DEFAULT_DRIVE_PINS,
        encoder_pins=DEFAULT_ENCODER_PINS,
        detach_servo_pin=6,
        head_servo_pin=17 if is_head else None,
        front_motor_pins=DEFAULT_FRONT_MOTOR_PINS if is_head and env_bool("HANSEL_FRONT_MOTOR_ENABLED", True) else None,
        full_speed_cps_left=env_float("HANSEL_FULL_SPEED_CPS_LEFT", 800.0),
        full_speed_cps_right=env_float("HANSEL_FULL_SPEED_CPS_RIGHT", 800.0),
        default_speed_scale=env_float("HANSEL_SPEED_SCALE", default_scale),
        node_slow_ratio=env_float("HANSEL_NODE_SLOW_RATIO", 0.45),
    )


class DryRunRobotController:
    """Controller used on laptops or when GPIO is unavailable."""

    def __init__(self, role: str, reason: str = "dry-run") -> None:
        self.role = role.lower()
        self.reason = reason
        self.running = False
        self.last_command = "stop"

    def start(self) -> None:
        self.running = True
        print(f"[{self.role}] dry-run motor controller active: {self.reason}")

    def handle_command(self, command: str, message: Optional[dict] = None) -> None:
        self.last_command = command
        print(f"[{self.role}] dry-run command={command} message={message or {}}")

    def stop(self) -> None:
        if self.running:
            print(f"[{self.role}] dry-run stop")
        self.running = False


class GpioRobotController:
    """Two-wheel PID drive controller with encoder feedback."""

    PWM_FREQ = int(env_float("HANSEL_PWM_FREQ", 1000))
    CONTROL_INTERVAL = env_float("HANSEL_CONTROL_INTERVAL", 0.05)
    ENCODER_POLL_INTERVAL = env_float("HANSEL_ENCODER_POLL_INTERVAL", 0.001)
    MIN_PWM = env_float("HANSEL_MIN_PWM", 25.0)
    MAX_PWM = env_float("HANSEL_MAX_PWM", 100.0)
    PWM_RAMP_PER_SEC = env_float("HANSEL_PWM_RAMP_PER_SEC", 220.0)
    INTEGRAL_LIMIT = env_float("HANSEL_PID_INTEGRAL_LIMIT", 500.0)

    TURN_INNER_RATIO = env_float("HANSEL_TURN_INNER_RATIO", 0.45)
    TURN_OUTER_RATIO = env_float("HANSEL_TURN_OUTER_RATIO", 1.0)
    MILD_TURN_INNER_RATIO = env_float("HANSEL_MILD_TURN_INNER_RATIO", 0.75)
    MILD_TURN_OUTER_RATIO = env_float("HANSEL_MILD_TURN_OUTER_RATIO", 1.0)
    SPIN_RATIO = env_float("HANSEL_SPIN_RATIO", 0.85)

    SERVO_FREQ = 50
    DETACH_REST_ANGLE = int(env_float("HANSEL_DETACH_REST_ANGLE", 20))
    DETACH_PRESS_ANGLE = int(env_float("HANSEL_DETACH_PRESS_ANGLE", 75))
    DETACH_PRESS_TIME = env_float("HANSEL_DETACH_PRESS_TIME", 0.35)
    HEAD_SERVO_MIN_ANGLE = int(env_float("HANSEL_HEAD_SERVO_MIN_ANGLE", 20))
    HEAD_SERVO_MAX_ANGLE = int(env_float("HANSEL_HEAD_SERVO_MAX_ANGLE", 150))
    HEAD_SERVO_CENTER_ANGLE = int(env_float("HANSEL_HEAD_SERVO_CENTER_ANGLE", 70))
    HEAD_SERVO_STEP_ANGLE = int(env_float("HANSEL_HEAD_SERVO_STEP_ANGLE", 2))
    HEAD_SERVO_MIN_PULSE_US = int(env_float("HANSEL_HEAD_SERVO_MIN_PULSE_US", 600))
    HEAD_SERVO_MAX_PULSE_US = int(env_float("HANSEL_HEAD_SERVO_MAX_PULSE_US", 2400))
    FRONT_MOTOR_KEY_PWM = env_float("HANSEL_FRONT_MOTOR_KEY_PWM", 100.0)

    DEBUG_PRINT_INTERVAL = env_float("HANSEL_DEBUG_PRINT_INTERVAL", 0.5)

    def __init__(self, config: RoleConfig) -> None:
        if GPIO is None:
            raise RuntimeError("RPi.GPIO is not installed")

        self.config = config
        self.role = config.role
        self.gains = PidGains(
            kp_left=env_float("HANSEL_KP_LEFT", 0.035),
            ki_left=env_float("HANSEL_KI_LEFT", 0.015),
            kd_left=env_float("HANSEL_KD_LEFT", 0.0),
            kp_right=env_float("HANSEL_KP_RIGHT", 0.035),
            ki_right=env_float("HANSEL_KI_RIGHT", 0.015),
            kd_right=env_float("HANSEL_KD_RIGHT", 0.0),
        )

        self.left_reverse = env_bool("HANSEL_LEFT_REVERSE", True)
        self.right_reverse = env_bool("HANSEL_RIGHT_REVERSE", True)
        self.front_left_reverse = env_bool("HANSEL_FRONT_LEFT_REVERSE", True)
        self.front_right_reverse = env_bool("HANSEL_FRONT_RIGHT_REVERSE", True)
        self.front_follow_drive = env_bool("HANSEL_FRONT_MOTOR_FOLLOW_DRIVE", self.role == "head")
        self.front_speed_ratio = env_float("HANSEL_FRONT_MOTOR_SPEED_RATIO", 1.0)

        self.running = threading.Event()
        self.encoder_lock = threading.Lock()
        self.target_lock = threading.Lock()

        self.left_count = 0
        self.right_count = 0
        self.left_last_state = 0
        self.right_last_state = 0

        self.left_target_cps = 0.0
        self.right_target_cps = 0.0
        self.left_direction = "stop"
        self.right_direction = "stop"
        self.left_pwm_value = 0.0
        self.right_pwm_value = 0.0
        self.left_integral = 0.0
        self.right_integral = 0.0
        self.left_prev_error = 0.0
        self.right_prev_error = 0.0
        self.current_head_servo_angle = self.HEAD_SERVO_CENTER_ANGLE
        self.last_debug_time = 0.0

        self.pwm_left = None
        self.pwm_right = None
        self.detach_servo_pwm = None
        self.head_servo_pwm = None
        self.front_pwm_left = None
        self.front_pwm_right = None
        self.pigpio_pi = None

        self.threads: List[threading.Thread] = []

    def start(self) -> None:
        print(f"[{self.role}] starting GPIO motor controller")
        self._check_duplicate_pins()
        self._setup_gpio()
        self.stop_all()
        self.running.set()

        self._initialize_encoder_state()

        self._start_thread("encoder", self._encoder_poll_loop)
        self._start_thread("pid", self._control_loop)

        if self.config.detach_servo_pin is not None and env_bool("HANSEL_DETACH_REST_ON_BOOT", False):
            self.detach_servo_rest()

        if self.config.head_servo_pin is not None and env_bool("HANSEL_HEAD_SERVO_CENTER_ON_BOOT", False):
            self.set_head_servo_angle(self.HEAD_SERVO_CENTER_ANGLE)

        print(f"[{self.role}] controller ready")

    def _start_thread(self, name: str, target: Callable[[], None]) -> None:
        thread = threading.Thread(target=target, name=f"{self.role}-{name}", daemon=True)
        thread.start()
        self.threads.append(thread)

    def _check_duplicate_pins(self) -> None:
        pins = {
            "ENA_PIN": self.config.drive_pins.ena,
            "IN1_PIN": self.config.drive_pins.in1,
            "IN2_PIN": self.config.drive_pins.in2,
            "ENB_PIN": self.config.drive_pins.enb,
            "IN3_PIN": self.config.drive_pins.in3,
            "IN4_PIN": self.config.drive_pins.in4,
            "LEFT_ENC_A": self.config.encoder_pins.left_a,
            "LEFT_ENC_B": self.config.encoder_pins.left_b,
            "RIGHT_ENC_A": self.config.encoder_pins.right_a,
            "RIGHT_ENC_B": self.config.encoder_pins.right_b,
        }
        if self.config.detach_servo_pin is not None:
            pins["DETACH_SERVO_PIN"] = self.config.detach_servo_pin
        if self.config.head_servo_pin is not None:
            pins["HEAD_SERVO_PIN"] = self.config.head_servo_pin
        if self.config.front_motor_pins is not None:
            pins.update(
                {
                    "FRONT_ENA_PIN": self.config.front_motor_pins.ena,
                    "FRONT_IN1_PIN": self.config.front_motor_pins.in1,
                    "FRONT_IN2_PIN": self.config.front_motor_pins.in2,
                    "FRONT_ENB_PIN": self.config.front_motor_pins.enb,
                    "FRONT_IN3_PIN": self.config.front_motor_pins.in3,
                    "FRONT_IN4_PIN": self.config.front_motor_pins.in4,
                }
            )

        used: Dict[int, str] = {}
        for name, pin in pins.items():
            if pin in used:
                raise RuntimeError(f"GPIO pin conflict: {name} and {used[pin]} both use GPIO{pin}")
            used[pin] = name
        print(f"[{self.role}] GPIO pin check OK")

    def _setup_gpio(self) -> None:
        try:
            GPIO.cleanup()
        except Exception:
            pass

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        pins = self.config.drive_pins
        for pin in (pins.ena, pins.in1, pins.in2, pins.enb, pins.in3, pins.in4):
            GPIO.setup(pin, GPIO.OUT)

        enc = self.config.encoder_pins
        for pin in (enc.left_a, enc.left_b, enc.right_a, enc.right_b):
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.pwm_left = GPIO.PWM(pins.ena, self.PWM_FREQ)
        self.pwm_right = GPIO.PWM(pins.enb, self.PWM_FREQ)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        if self.config.detach_servo_pin is not None:
            GPIO.setup(self.config.detach_servo_pin, GPIO.OUT)
            self.detach_servo_pwm = GPIO.PWM(self.config.detach_servo_pin, self.SERVO_FREQ)
            self.detach_servo_pwm.start(0)

        front = self.config.front_motor_pins
        if front is not None:
            for pin in (front.ena, front.in1, front.in2, front.enb, front.in3, front.in4):
                GPIO.setup(pin, GPIO.OUT)
            self.front_pwm_left = GPIO.PWM(front.ena, self.PWM_FREQ)
            self.front_pwm_right = GPIO.PWM(front.enb, self.PWM_FREQ)
            self.front_pwm_left.start(0)
            self.front_pwm_right.start(0)

        if self.config.head_servo_pin is not None:
            self._setup_head_servo()

    def _setup_head_servo(self) -> None:
        if self.config.head_servo_pin is None:
            return

        if pigpio is None:
            print(f"[{self.role}] pigpio module missing; using RPi.GPIO PWM for head servo")
            GPIO.setup(self.config.head_servo_pin, GPIO.OUT)
            self.head_servo_pwm = GPIO.PWM(self.config.head_servo_pin, self.SERVO_FREQ)
            self.head_servo_pwm.start(0)
            return

        self.pigpio_pi = pigpio.pi()
        if not self.pigpio_pi.connected:
            print(f"[{self.role}] pigpiod not running; using RPi.GPIO PWM for head servo")
            self.pigpio_pi = None
            GPIO.setup(self.config.head_servo_pin, GPIO.OUT)
            self.head_servo_pwm = GPIO.PWM(self.config.head_servo_pin, self.SERVO_FREQ)
            self.head_servo_pwm.start(0)
            return

        self.pigpio_pi.set_mode(self.config.head_servo_pin, pigpio.OUTPUT)
        self.pigpio_pi.set_servo_pulsewidth(self.config.head_servo_pin, 0)

    def _read_encoder_state(self, pin_a: int, pin_b: int) -> int:
        a = GPIO.input(pin_a)
        b = GPIO.input(pin_b)
        return (a << 1) | b

    def _initialize_encoder_state(self) -> None:
        enc = self.config.encoder_pins
        try:
            self.left_last_state = self._read_encoder_state(enc.left_a, enc.left_b)
            self.right_last_state = self._read_encoder_state(enc.right_a, enc.right_b)
        except Exception as exc:
            raise RuntimeError(
                "Encoder GPIO read failed. Check encoder pin wiring, "
                "BCM numbering, and run the server with sudo."
            ) from exc

    @staticmethod
    def _update_quadrature_count(last_state: int, new_state: int, current_count: int) -> int:
        transition = (last_state << 2) | new_state
        if transition in (0b0001, 0b0111, 0b1110, 0b1000):
            current_count += 1
        elif transition in (0b0010, 0b1011, 0b1101, 0b0100):
            current_count -= 1
        return current_count

    def _encoder_poll_loop(self) -> None:
        enc = self.config.encoder_pins
        print(f"[{self.role}] encoder polling loop started")
        while self.running.is_set():
            try:
                with self.encoder_lock:
                    left_state = self._read_encoder_state(enc.left_a, enc.left_b)
                    self.left_count = self._update_quadrature_count(
                        self.left_last_state,
                        left_state,
                        self.left_count,
                    )
                    self.left_last_state = left_state

                    right_state = self._read_encoder_state(enc.right_a, enc.right_b)
                    self.right_count = self._update_quadrature_count(
                        self.right_last_state,
                        right_state,
                        self.right_count,
                    )
                    self.right_last_state = right_state
            except Exception as exc:
                print(f"[{self.role}] encoder polling error: {exc}")
                self.running.clear()
                try:
                    self.stop_all()
                except Exception:
                    pass
                break

            time.sleep(self.ENCODER_POLL_INTERVAL)

    def _apply_left_direction(self, direction: str) -> None:
        direction = self._reverse_direction_if_needed(direction, self.left_reverse)
        pins = self.config.drive_pins
        self._apply_hbridge_direction(pins.in1, pins.in2, direction)

    def _apply_right_direction(self, direction: str) -> None:
        direction = self._reverse_direction_if_needed(direction, self.right_reverse)
        pins = self.config.drive_pins
        self._apply_hbridge_direction(pins.in3, pins.in4, direction)

    @staticmethod
    def _apply_hbridge_direction(pin_a: int, pin_b: int, direction: str) -> None:
        if direction == "forward":
            GPIO.output(pin_a, GPIO.HIGH)
            GPIO.output(pin_b, GPIO.LOW)
        elif direction == "backward":
            GPIO.output(pin_a, GPIO.LOW)
            GPIO.output(pin_b, GPIO.HIGH)
        else:
            GPIO.output(pin_a, GPIO.LOW)
            GPIO.output(pin_b, GPIO.LOW)

    @staticmethod
    def _reverse_direction_if_needed(direction: str, reverse: bool) -> str:
        if not reverse:
            return direction
        if direction == "forward":
            return "backward"
        if direction == "backward":
            return "forward"
        return "stop"

    def _set_drive_target(
        self,
        left_cps: float,
        left_dir: str,
        right_cps: float,
        right_dir: str,
    ) -> None:
        with self.target_lock:
            self.left_target_cps = abs(float(left_cps))
            self.right_target_cps = abs(float(right_cps))
            self.left_direction = left_dir
            self.right_direction = right_dir

            self._apply_left_direction(left_dir)
            self._apply_right_direction(right_dir)

            if self.front_follow_drive and self.config.front_motor_pins is not None:
                self._apply_front_left_direction(left_dir)
                self._apply_front_right_direction(right_dir)

            if self.left_target_cps == 0:
                self.left_integral = 0.0
                self.left_prev_error = 0.0
            if self.right_target_cps == 0:
                self.right_integral = 0.0
                self.right_prev_error = 0.0

    def _scaled_cps(self, side: str, ratio: float, message: Optional[dict] = None) -> float:
        speed = self.config.default_speed_scale
        if message is not None and "speed" in message:
            try:
                speed = float(message["speed"])
            except (TypeError, ValueError):
                speed = self.config.default_speed_scale
        speed = clamp(speed, 0.0, 1.0)
        full = self.config.full_speed_cps_left if side == "left" else self.config.full_speed_cps_right
        return full * ratio * speed

    def forward(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", 1.0, message),
            "forward",
            self._scaled_cps("right", 1.0, message),
            "forward",
        )

    def backward(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", 1.0, message),
            "backward",
            self._scaled_cps("right", 1.0, message),
            "backward",
        )

    def forward_left(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.TURN_INNER_RATIO, message),
            "forward",
            self._scaled_cps("right", self.TURN_OUTER_RATIO, message),
            "forward",
        )

    def forward_right(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.TURN_OUTER_RATIO, message),
            "forward",
            self._scaled_cps("right", self.TURN_INNER_RATIO, message),
            "forward",
        )

    def backward_left(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.TURN_OUTER_RATIO, message),
            "backward",
            self._scaled_cps("right", self.TURN_INNER_RATIO, message),
            "backward",
        )

    def backward_right(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.TURN_INNER_RATIO, message),
            "backward",
            self._scaled_cps("right", self.TURN_OUTER_RATIO, message),
            "backward",
        )

    def mild_backward_left(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.MILD_TURN_OUTER_RATIO, message),
            "backward",
            self._scaled_cps("right", self.MILD_TURN_INNER_RATIO, message),
            "backward",
        )

    def mild_backward_right(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.MILD_TURN_INNER_RATIO, message),
            "backward",
            self._scaled_cps("right", self.MILD_TURN_OUTER_RATIO, message),
            "backward",
        )

    def mild_forward_left(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.MILD_TURN_INNER_RATIO, message),
            "forward",
            self._scaled_cps("right", self.MILD_TURN_OUTER_RATIO, message),
            "forward",
        )

    def mild_forward_right(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.MILD_TURN_OUTER_RATIO, message),
            "forward",
            self._scaled_cps("right", self.MILD_TURN_INNER_RATIO, message),
            "forward",
        )

    def left(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.SPIN_RATIO, message),
            "backward",
            self._scaled_cps("right", self.SPIN_RATIO, message),
            "forward",
        )

    def right(self, message: Optional[dict] = None) -> None:
        self._set_drive_target(
            self._scaled_cps("left", self.SPIN_RATIO, message),
            "forward",
            self._scaled_cps("right", self.SPIN_RATIO, message),
            "backward",
        )

    def slow_forward(self, message: Optional[dict] = None) -> None:
        speed = self.config.node_slow_ratio
        if message is not None and "speed" in message:
            try:
                speed = min(float(message["speed"]), self.config.node_slow_ratio)
            except (TypeError, ValueError):
                speed = self.config.node_slow_ratio
        self._set_drive_target(
            self.config.full_speed_cps_left * speed,
            "forward",
            self.config.full_speed_cps_right * speed,
            "forward",
        )

    def slow_backward(self, message: Optional[dict] = None) -> None:
        speed = self.config.node_slow_ratio
        if message is not None and "speed" in message:
            try:
                speed = min(float(message["speed"]), self.config.node_slow_ratio)
            except (TypeError, ValueError):
                speed = self.config.node_slow_ratio
        self._set_drive_target(
            self.config.full_speed_cps_left * speed,
            "backward",
            self.config.full_speed_cps_right * speed,
            "backward",
        )

    def stop_all(self, message: Optional[dict] = None) -> None:
        with self.target_lock:
            self.left_target_cps = 0.0
            self.right_target_cps = 0.0
            self.left_pwm_value = 0.0
            self.right_pwm_value = 0.0
            self.left_integral = 0.0
            self.right_integral = 0.0
            self.left_prev_error = 0.0
            self.right_prev_error = 0.0

            self._apply_left_direction("stop")
            self._apply_right_direction("stop")
            if self.pwm_left is not None:
                self.pwm_left.ChangeDutyCycle(0)
            if self.pwm_right is not None:
                self.pwm_right.ChangeDutyCycle(0)
            self.front_motor_stop()

    def _compute_pid_pwm(
        self,
        target_cps: float,
        measured_cps: float,
        max_cps: float,
        kp: float,
        ki: float,
        kd: float,
        integral: float,
        prev_error: float,
        previous_pwm: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        if target_cps <= 0:
            return 0.0, 0.0, 0.0
        if max_cps <= 0:
            return 0.0, 0.0, 0.0

        target_cps = min(target_cps, max_cps)
        error = target_cps - measured_cps
        integral = clamp(integral + error * dt, -self.INTEGRAL_LIMIT, self.INTEGRAL_LIMIT)
        derivative = (error - prev_error) / dt if dt > 0 else 0.0

        feed_forward = self.MIN_PWM + (target_cps / max_cps) * (self.MAX_PWM - self.MIN_PWM)
        pid_output = kp * error + ki * integral + kd * derivative
        requested_pwm = clamp_pwm(feed_forward + pid_output)

        max_delta = self.PWM_RAMP_PER_SEC * dt
        pwm = clamp(requested_pwm, previous_pwm - max_delta, previous_pwm + max_delta)
        return pwm, integral, error

    def _control_loop(self) -> None:
        print(f"[{self.role}] PID control loop started")
        prev_left_count = 0
        prev_right_count = 0
        prev_time = time.time()

        while self.running.is_set():
            time.sleep(self.CONTROL_INTERVAL)

            now = time.time()
            dt = now - prev_time
            if dt <= 0:
                continue

            with self.encoder_lock:
                current_left_count = self.left_count
                current_right_count = self.right_count

            delta_left = current_left_count - prev_left_count
            delta_right = current_right_count - prev_right_count
            prev_left_count = current_left_count
            prev_right_count = current_right_count
            prev_time = now

            measured_left_cps = abs(delta_left) / dt
            measured_right_cps = abs(delta_right) / dt

            with self.target_lock:
                self.left_pwm_value, self.left_integral, self.left_prev_error = self._compute_pid_pwm(
                    self.left_target_cps,
                    measured_left_cps,
                    self.config.full_speed_cps_left,
                    self.gains.kp_left,
                    self.gains.ki_left,
                    self.gains.kd_left,
                    self.left_integral,
                    self.left_prev_error,
                    self.left_pwm_value,
                    dt,
                )

                self.right_pwm_value, self.right_integral, self.right_prev_error = self._compute_pid_pwm(
                    self.right_target_cps,
                    measured_right_cps,
                    self.config.full_speed_cps_right,
                    self.gains.kp_right,
                    self.gains.ki_right,
                    self.gains.kd_right,
                    self.right_integral,
                    self.right_prev_error,
                    self.right_pwm_value,
                    dt,
                )

                left_pwm = 0.0 if self.left_target_cps <= 0 else self.left_pwm_value
                right_pwm = 0.0 if self.right_target_cps <= 0 else self.right_pwm_value

            self.pwm_left.ChangeDutyCycle(left_pwm)
            self.pwm_right.ChangeDutyCycle(right_pwm)

            if self.front_follow_drive and self.config.front_motor_pins is not None:
                self._apply_front_pwm(left_pwm * self.front_speed_ratio, right_pwm * self.front_speed_ratio)

            if now - self.last_debug_time >= self.DEBUG_PRINT_INTERVAL:
                self.last_debug_time = now
                with self.target_lock:
                    target_left = self.left_target_cps
                    target_right = self.right_target_cps
                    pwm_left = self.left_pwm_value
                    pwm_right = self.right_pwm_value
                if target_left > 0 or target_right > 0:
                    print(
                        f"[{self.role}] "
                        f"L target={target_left:.1f} cps measured={measured_left_cps:.1f} pwm={pwm_left:.1f} | "
                        f"R target={target_right:.1f} cps measured={measured_right_cps:.1f} pwm={pwm_right:.1f}"
                    )

    def _apply_front_left_direction(self, direction: str) -> None:
        front = self.config.front_motor_pins
        if front is None:
            return
        direction = self._reverse_direction_if_needed(direction, self.front_left_reverse)
        self._apply_hbridge_direction(front.in1, front.in2, direction)

    def _apply_front_right_direction(self, direction: str) -> None:
        front = self.config.front_motor_pins
        if front is None:
            return
        direction = self._reverse_direction_if_needed(direction, self.front_right_reverse)
        self._apply_hbridge_direction(front.in3, front.in4, direction)

    def _apply_front_pwm(self, left_pwm: float, right_pwm: float) -> None:
        if self.front_pwm_left is None or self.front_pwm_right is None:
            return
        self.front_pwm_left.ChangeDutyCycle(clamp_pwm(left_pwm))
        self.front_pwm_right.ChangeDutyCycle(clamp_pwm(right_pwm))

    def front_motor_forward(self, message: Optional[dict] = None) -> None:
        if self.config.front_motor_pins is None:
            print(f"[{self.role}] front motors are not configured")
            return
        self._apply_front_left_direction("forward")
        self._apply_front_right_direction("forward")
        self._apply_front_pwm(self.FRONT_MOTOR_KEY_PWM, self.FRONT_MOTOR_KEY_PWM)

    def front_motor_backward(self, message: Optional[dict] = None) -> None:
        if self.config.front_motor_pins is None:
            print(f"[{self.role}] front motors are not configured")
            return
        self._apply_front_left_direction("backward")
        self._apply_front_right_direction("backward")
        self._apply_front_pwm(self.FRONT_MOTOR_KEY_PWM, self.FRONT_MOTOR_KEY_PWM)

    def front_motor_stop(self, message: Optional[dict] = None) -> None:
        if self.config.front_motor_pins is None:
            return
        self._apply_front_left_direction("stop")
        self._apply_front_right_direction("stop")
        self._apply_front_pwm(0, 0)

    @staticmethod
    def _angle_to_duty(angle: int) -> float:
        angle = clamp_angle(angle)
        return 2.5 + angle / 18.0

    def set_detach_servo_angle(self, angle: int, hold: bool = False) -> None:
        if self.detach_servo_pwm is None:
            print(f"[{self.role}] detach servo is not configured")
            return
        angle = clamp_angle(angle)
        duty = self._angle_to_duty(angle)
        print(f"[{self.role}] detach servo angle={angle} duty={duty:.2f}%")
        self.detach_servo_pwm.ChangeDutyCycle(duty)
        if not hold:
            time.sleep(0.12)
            self.detach_servo_pwm.ChangeDutyCycle(0)

    def detach_servo_rest(self, message: Optional[dict] = None) -> None:
        self.set_detach_servo_angle(self.DETACH_REST_ANGLE, hold=False)

    def detach_servo_press(self, message: Optional[dict] = None) -> None:
        self.set_detach_servo_angle(self.DETACH_PRESS_ANGLE, hold=True)
        time.sleep(self.DETACH_PRESS_TIME)
        self.set_detach_servo_angle(self.DETACH_REST_ANGLE, hold=False)

    def _head_servo_pulse_us(self, angle: int) -> int:
        angle = clamp_angle(angle, self.HEAD_SERVO_MIN_ANGLE, self.HEAD_SERVO_MAX_ANGLE)
        span = self.HEAD_SERVO_MAX_PULSE_US - self.HEAD_SERVO_MIN_PULSE_US
        return int(self.HEAD_SERVO_MIN_PULSE_US + (angle / 180.0) * span)

    def set_head_servo_angle(self, angle: int) -> None:
        if self.config.head_servo_pin is None:
            print(f"[{self.role}] head servo is not available")
            return
        self.current_head_servo_angle = clamp_angle(
            angle,
            self.HEAD_SERVO_MIN_ANGLE,
            self.HEAD_SERVO_MAX_ANGLE,
        )

        if self.pigpio_pi is not None:
            pulse = self._head_servo_pulse_us(self.current_head_servo_angle)
            print(f"[{self.role}] head servo angle={self.current_head_servo_angle} pulse={pulse}us")
            self.pigpio_pi.set_servo_pulsewidth(self.config.head_servo_pin, pulse)
            if not env_bool("HANSEL_HEAD_SERVO_HOLD", False):
                time.sleep(0.08)
                self.pigpio_pi.set_servo_pulsewidth(self.config.head_servo_pin, 0)
            return

        if self.head_servo_pwm is not None:
            duty = self._angle_to_duty(self.current_head_servo_angle)
            print(f"[{self.role}] head servo angle={self.current_head_servo_angle} duty={duty:.2f}%")
            self.head_servo_pwm.ChangeDutyCycle(duty)
            if not env_bool("HANSEL_HEAD_SERVO_HOLD", False):
                time.sleep(0.12)
                self.head_servo_pwm.ChangeDutyCycle(0)
            return

        print(f"[{self.role}] head servo is not available")

    def head_servo_up_step(self, message: Optional[dict] = None) -> None:
        self.set_head_servo_angle(self.current_head_servo_angle + self.HEAD_SERVO_STEP_ANGLE)

    def head_servo_down_step(self, message: Optional[dict] = None) -> None:
        self.set_head_servo_angle(self.current_head_servo_angle - self.HEAD_SERVO_STEP_ANGLE)

    def head_servo_center(self, message: Optional[dict] = None) -> None:
        self.set_head_servo_angle(self.HEAD_SERVO_CENTER_ANGLE)

    def head_servo_min(self, message: Optional[dict] = None) -> None:
        self.set_head_servo_angle(self.HEAD_SERVO_MIN_ANGLE)

    def head_servo_max(self, message: Optional[dict] = None) -> None:
        self.set_head_servo_angle(self.HEAD_SERVO_MAX_ANGLE)

    def handle_command(self, command: str, message: Optional[dict] = None) -> None:
        normalized = command.strip().lower().replace("-", "_").replace(" ", "_")
        if not normalized:
            return

        if self.role != "head":
            normalized = self._normalize_node_command(normalized)

        command_map: Dict[str, Callable[[Optional[dict]], None]] = {
            "forward": self.forward,
            "backward": self.backward,
            "left": self.left,
            "right": self.right,
            "stop": self.stop_all,
            "forward_left": self.forward_left,
            "forward_right": self.forward_right,
            "backward_left": self.backward_left,
            "backward_right": self.backward_right,
            "mild_forward_left": self.mild_forward_left,
            "mild_forward_right": self.mild_forward_right,
            "mild_backward_left": self.mild_backward_left,
            "mild_backward_right": self.mild_backward_right,
            "slow_forward": self.slow_forward,
            "slow_backward": self.slow_backward,
            "detach_press": self.detach_servo_press,
            "detach_rest": self.detach_servo_rest,
            "head_servo_up": self.head_servo_up_step,
            "head_servo_down": self.head_servo_down_step,
            "head_servo_center": self.head_servo_center,
            "head_servo_min": self.head_servo_min,
            "head_servo_max": self.head_servo_max,
            "servo_up": self.head_servo_up_step,
            "servo_down": self.head_servo_down_step,
            "servo_center": self.head_servo_center,
            "servo_min": self.head_servo_min,
            "servo_max": self.head_servo_max,
            "front_motor_forward": self.front_motor_forward,
            "front_motor_backward": self.front_motor_backward,
            "front_motor_stop": self.front_motor_stop,
            "front_forward": self.front_motor_forward,
            "front_backward": self.front_motor_backward,
            "front_stop": self.front_motor_stop,
        }

        action = command_map.get(normalized)
        if action is None:
            print(f"[{self.role}] unknown command: {command}")
            return

        print(f"[{self.role}] command={normalized}")
        action(message)

    @staticmethod
    def _normalize_node_command(command: str) -> str:
        """Nodes do not steer; they only drive straight, slow, or stop."""
        forward_steering = {
            "forward_left",
            "forward_right",
            "mild_forward_left",
            "mild_forward_right",
        }
        backward_steering = {
            "backward_left",
            "backward_right",
            "mild_backward_left",
            "mild_backward_right",
        }
        spin_commands = {"left", "right"}

        if command in forward_steering:
            return "slow_forward"
        if command in backward_steering:
            return "slow_backward"
        if command in spin_commands:
            return "stop"
        if command.startswith("head_servo_") or command.startswith("front_"):
            return "stop"
        return command

    def stop(self) -> None:
        print(f"[{self.role}] stopping GPIO controller")
        self.running.clear()
        time.sleep(0.1)
        try:
            self.stop_all()
        except Exception:
            pass

        for pwm in (
            self.pwm_left,
            self.pwm_right,
            self.front_pwm_left,
            self.front_pwm_right,
            self.detach_servo_pwm,
            self.head_servo_pwm,
        ):
            try:
                if pwm is not None:
                    pwm.ChangeDutyCycle(0)
                    pwm.stop()
            except Exception:
                pass

        if self.pigpio_pi is not None and self.config.head_servo_pin is not None:
            try:
                self.pigpio_pi.set_servo_pulsewidth(self.config.head_servo_pin, 0)
                self.pigpio_pi.stop()
            except Exception:
                pass

        try:
            GPIO.cleanup()
        except Exception:
            pass
        print(f"[{self.role}] GPIO controller stopped")


def build_robot_controller(role: str, dry_run: bool = False):
    if dry_run:
        return DryRunRobotController(role, "--dry-run requested")
    if GPIO is None:
        return DryRunRobotController(role, "RPi.GPIO not installed")
    return GpioRobotController(role_config(role))
