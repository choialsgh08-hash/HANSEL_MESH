"""Hardware adapters for laptop simulation and Arduino Nano deployment.

The uploaded wiring plan assigns wheel motors, rear encoders, the 150 kg head
servo and SG90 detach servo to one Arduino Nano. Raspberry Pi owns only the
high-level ROS node and exchanges a compact newline protocol over USB serial.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
import threading
import time
import uuid


class HardwareIOError(OSError):
    """Retryable transport failure, not a permanent actuator fault."""


@dataclass(frozen=True)
class EncoderSample:
    epoch: str
    sequence: int
    left: int
    right: int
    received_at: float


class HardwareAdapter(ABC):
    @property
    def encoder_sample(self) -> EncoderSample | None:
        return None

    def recover(self) -> HardwareAdapter | None:
        """Retry an existing transport; update(0, 0, False, dt) verifies it."""

    def clear_front_motor(self) -> None:
        pass

    @property
    def head_moving(self) -> bool:
        return False

    def wait_detach(self, timeout_s: float) -> None:
        """Return after command completion, not proof of mechanical separation."""
        raise NotImplementedError("backend does not report detach completion")

    @abstractmethod
    def update(self, left_pwm: float, right_pwm: float, drive_enabled: bool,
               dt: float) -> tuple[float, float]:
        """Apply signed PWM percentages and return measured signed CPS."""

    @abstractmethod
    def set_front_motor(self, command: str, speed_scale: float) -> None:
        """Apply one-shot front_motor_forward/backward/stop."""

    @abstractmethod
    def set_head_angle(self, angle_deg: float) -> None:
        """Apply a logical head angle in the configured -180..+180 range."""

    @abstractmethod
    def configure_servo(self, **kwargs) -> None:
        """Update logical angle and pulse calibration."""

    @abstractmethod
    def trigger_detach(self, duration_s: float, press_pulse_us: int,
                       rest_pulse_us: int) -> None:
        """Press and hold the SG90; rest_pulse_us is a legacy protocol field."""

    @abstractmethod
    def diagnostic_items(self) -> dict[str, str]:
        pass

    @abstractmethod
    def close(self) -> None:
        pass


class DummyHardwareAdapter(HardwareAdapter):
    def wait_detach(self, timeout_s: float) -> None:
        pass

    def __init__(self, full_speed_cps_left: float, full_speed_cps_right: float,
                 response_time_s: float = 0.2) -> None:
        self.full_speed_cps_left = float(full_speed_cps_left)
        self.full_speed_cps_right = float(full_speed_cps_right)
        self.response_time_s = max(0.01, float(response_time_s))
        self.left_cps = 0.0
        self.right_cps = 0.0
        self.head_angle = 0.0
        self.front_command = "follow"
        self.detach_count = 0

    def update(self, left_pwm: float, right_pwm: float, drive_enabled: bool,
               dt: float) -> tuple[float, float]:
        left_target = right_target = 0.0
        if drive_enabled:
            left_target = self.full_speed_cps_left * max(-1.0, min(1.0, left_pwm / 100.0))
            right_target = self.full_speed_cps_right * max(-1.0, min(1.0, right_pwm / 100.0))
        alpha = min(1.0, max(0.0, dt) / self.response_time_s)
        self.left_cps += (left_target - self.left_cps) * alpha
        self.right_cps += (right_target - self.right_cps) * alpha
        return self.left_cps, self.right_cps

    def set_front_motor(self, command: str, speed_scale: float) -> None:
        del speed_scale
        self.front_command = command

    def set_head_angle(self, angle_deg: float) -> None:
        self.head_angle = max(-180.0, min(180.0, float(angle_deg)))

    def configure_servo(self, **kwargs) -> None:
        del kwargs

    def trigger_detach(self, duration_s: float, press_pulse_us: int,
                       rest_pulse_us: int) -> None:
        del duration_s, press_pulse_us, rest_pulse_us
        self.detach_count += 1

    def diagnostic_items(self) -> dict[str, str]:
        return {
            "backend": "dummy",
            "head_angle_deg": f"{self.head_angle:.1f}",
            "front_command": self.front_command,
            "detach_commands": str(self.detach_count),
        }

    def close(self) -> None:
        self.left_cps = self.right_cps = 0.0


class NanoSerialHardwareAdapter(HardwareAdapter):
    """Compact serial bridge to robot/firmware/hansel_nano_bridge.

    Protocol (one ASCII line per message):
      Pi -> Nano
        M,<left_signed_pwm>,<right_signed_pwm>,<front_follow:0|1>
        F,<-1|0|1>,<pwm_percent>
        H,<logical_angle_deg>
        S,<min>,<center>,<max>,<min_us>,<center_us>,<max_us>,<hold:0|1>
        D,<press_ms>,<press_us>,<legacy_rest_us>,<token>  (press and hold)
        X
      Nano -> Pi
        T,<left_cps>,<right_cps>,<head_angle>,<moving>,<left_count>,<right_count>,<ms>
        B,boot
        D,<token>,done|busy
        E,<message>
    """

    def __init__(self, port: str, baudrate: int = 115200,
                 read_timeout_s: float = 0.02,
                 front_follow_drive: bool = True,
                 motor_pwm_limit_percent: float = 75.0) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required for nano_serial backend") from exc
        if not port:
            raise ValueError("nano_serial_port must not be empty")
        self.port_name = str(port)
        self.baudrate = int(baudrate)
        self.front_follow_drive = bool(front_follow_drive)
        self.set_motor_pwm_limit(motor_pwm_limit_percent)
        self._serial = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            timeout=float(read_timeout_s),
            write_timeout=0.2,
            exclusive=True,
        )
        time.sleep(1.8)
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._left_cps = self._right_cps = self._head_angle = 0.0
        self._last_state_monotonic: float | None = None
        self._invalid_lines = 0
        self._last_error = ""
        self._read_error = ""
        self._encoder_epoch = uuid.uuid4().hex
        self._encoder_sequence = 0
        self._encoder_sample = None
        self._sample_tick = None
        self._front_override = None
        self._servo_config_line = None
        self._configuration_pending = False
        self._needs_reopen = False
        self._ready_after = 0.0
        self._head_moving = False
        self._detach_done = threading.Event()
        self._detach_token = time.monotonic_ns() % 0xFFFFFFFF
        self._detach_result = ""
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        self._send("Q")

    def set_motor_pwm_limit(self, limit_percent: float) -> None:
        limit = float(limit_percent)
        if not math.isfinite(limit) or not 0.0 < limit <= 100.0:
            raise ValueError("motor PWM limit must be within 0..100 percent")
        self.motor_pwm_limit_percent = limit

    def _send(self, line: str) -> None:
        data = (line.rstrip("\n") + "\n").encode("ascii")
        with self._write_lock:
            self._serial.write(data)
            self._serial.flush()

    def _reader_loop(self) -> None:
        buffer = b""
        previous_port = None
        while not self._stop.is_set():
            port = self._serial
            if port is not previous_port:
                buffer = b""
                previous_port = port
            try:
                raw = port.readline()
                if not raw:
                    continue
                if port is self._serial:
                    buffer += raw
                    if len(buffer) > 1024:
                        buffer = b""
                        raise ValueError("oversized Nano line")
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        self._consume_line(line.decode("ascii", errors="strict").strip())
            except (ValueError, UnicodeError):
                self._invalid_lines += 1
            except OSError as exc:
                if port is self._serial:
                    self._read_error = str(exc)
                self._stop.wait(0.05)

    def _consume_line(self, line: str) -> None:
        parts = line.split(",")
        if parts[0] == "T" and len(parts) >= 4:
            values = tuple(float(value) for value in parts[1:4])
            if not all(math.isfinite(value) for value in values):
                raise ValueError("non-finite Nano state")
            counts = tuple(int(value) for value in parts[5:8]) if len(parts) == 8 else None
            if counts and (not all(-2**31 <= v < 2**31 for v in counts[:2])
                           or not 0 <= counts[2] < 2**32):
                raise ValueError("invalid Nano counters")
            now = time.monotonic()
            with self._state_lock:
                if counts:
                    tick = counts[2]
                    previous = self._sample_tick
                    if previous is not None:
                        if tick == previous:
                            return
                        if (tick - previous) % 2**32 > 2**31:
                            self._encoder_epoch = uuid.uuid4().hex
                            self._configuration_pending = True
                    self._sample_tick = tick
                    self._encoder_sequence += 1
                    self._encoder_sample = EncoderSample(
                        self._encoder_epoch, self._encoder_sequence, counts[0], counts[1], now)
                else:
                    self._encoder_sample = None  # Old firmware is never a count source.
                self._left_cps, self._right_cps, self._head_angle = values
                self._last_state_monotonic = now
                self._read_error = ""
                self._head_moving = len(parts) >= 5 and parts[4] == "1"
        elif parts[0] == "B" and parts[1:] == ["boot"]:
            self._reset_feedback()
        elif parts[0] == "D" and len(parts) == 3:
            if int(parts[1]) == self._detach_token:
                self._detach_result = parts[2]
                self._detach_done.set()
        elif parts[0] == "E":
            self._last_error = ",".join(parts[1:])

    def _reset_feedback(self) -> None:
        with self._state_lock:
            self._encoder_epoch = uuid.uuid4().hex
            self._encoder_sample = None
            self._sample_tick = None
            self._last_state_monotonic = None
            self._left_cps = self._right_cps = 0.0
            self._configuration_pending = True

    def _feedback_fresh(self) -> bool:
        return (not self._read_error and self._last_state_monotonic is not None
                and time.monotonic() - self._last_state_monotonic <= 0.35)

    @property
    def encoder_sample(self) -> EncoderSample | None:
        with self._state_lock:
            return self._encoder_sample if self._feedback_fresh() else None

    def recover(self) -> None:
        if time.monotonic() < self._ready_after:
            raise HardwareIOError("waiting for Nano startup")
        if not self._needs_reopen and self._feedback_fresh():
            try:
                self._send("X")
                if self._servo_config_line:
                    self._send(self._servo_config_line)
                self._configuration_pending = False
                return
            except OSError:
                self._needs_reopen = True
                raise
        import serial
        with self._write_lock:
            self._serial.close()
            self._reset_feedback()
            self._serial = serial.Serial(self.port_name, self.baudrate, timeout=0.02,
                                         write_timeout=0.2, exclusive=True)
        self._needs_reopen = False
        self._ready_after = time.monotonic() + 1.8
        raise HardwareIOError("Nano reopened; waiting for telemetry")

    def update(self, left_pwm: float, right_pwm: float, drive_enabled: bool,
               dt: float) -> tuple[float, float]:
        del dt
        if not self._feedback_fresh():
            self._front_override = None
            self._send("X")
            raise HardwareIOError("Nano telemetry missing/stale; reconnecting")
        if self._configuration_pending:
            if self._servo_config_line:
                self._send(self._servo_config_line)
            self._configuration_pending = False
        limit = self.motor_pwm_limit_percent
        left = max(-limit, min(limit, float(left_pwm))) if drive_enabled else 0.0
        right = max(-limit, min(limit, float(right_pwm))) if drive_enabled else 0.0
        if not drive_enabled:
            self.clear_front_motor()
        if self._front_override is not None:
            self._send(self._front_override)
        else:
            self._send(f"M,{left:.3f},{right:.3f},{1 if self.front_follow_drive else 0}")
        with self._state_lock:
            return self._left_cps, self._right_cps

    def set_front_motor(self, command: str, speed_scale: float) -> None:
        mode = {
            "front_motor_forward": 1,
            "front_motor_backward": -1,
            "front_motor_stop": 0,
        }.get(command)
        if mode is None:
            raise ValueError(f"unsupported front motor command: {command}")
        pwm = max(
            0.0,
            min(self.motor_pwm_limit_percent, float(speed_scale) * 100.0),
        )
        self._front_override = f"F,{mode},{pwm if mode else 0.0:.3f}"
        self._send(self._front_override)

    def clear_front_motor(self) -> None:
        self._front_override = None

    def set_head_angle(self, angle_deg: float) -> None:
        angle = max(-180.0, min(180.0, float(angle_deg)))
        self._send(f"H,{angle:.3f}")

    @property
    def head_moving(self) -> bool:
        with self._state_lock:
            return self._head_moving

    def configure_servo(self, **kwargs) -> None:
        self._servo_config_line = (
            "S,{min_angle:.3f},{center_angle:.3f},{max_angle:.3f},"
            "{min_pulse_us:d},{center_pulse_us:d},{max_pulse_us:d},{hold:d}".format(
                min_angle=float(kwargs["min_angle"]),
                center_angle=float(kwargs["center_angle"]),
                max_angle=float(kwargs["max_angle"]),
                min_pulse_us=int(kwargs["min_pulse_us"]),
                center_pulse_us=int(kwargs["center_pulse_us"]),
                max_pulse_us=int(kwargs["max_pulse_us"]),
                hold=1 if bool(kwargs["hold"]) else 0,
            )
        )
        self._send(self._servo_config_line)

    def trigger_detach(self, duration_s: float, press_pulse_us: int,
                       rest_pulse_us: int) -> None:
        press_ms = int(max(50.0, min(2000.0, float(duration_s) * 1000.0)))
        self._detach_token = (self._detach_token + 1) % 0xFFFFFFFF
        self._detach_result = ""
        self._detach_done.clear()
        self._send(f"D,{press_ms},{int(press_pulse_us)},{int(rest_pulse_us)},{self._detach_token}")

    def wait_detach(self, timeout_s: float) -> None:
        if not self._detach_done.wait(timeout_s):
            raise TimeoutError("Nano detach completion missing; check connection and flash updated bridge firmware")
        if self._detach_result != "done":
            raise RuntimeError(f"Nano detach: {self._detach_result}")

    def diagnostic_items(self) -> dict[str, str]:
        with self._state_lock:
            age = "never" if self._last_state_monotonic is None else (
                f"{time.monotonic() - self._last_state_monotonic:.2f}"
            )
            return {
                "backend": "nano_serial",
                "serial_port": self.port_name,
                "serial_baudrate": str(self.baudrate),
                "state_age_s": age,
                "encoder_counts_valid": str(self._encoder_sample is not None and self._feedback_fresh()).lower(),
                "invalid_lines": str(self._invalid_lines),
                "last_nano_error": self._last_error or "none",
                "head_angle_deg": f"{self._head_angle:.1f}",
                "wiring": "Nano D5/D6 shared PWM; D9 head; D10 detach",
            }

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        try:
            self._send("X")
        except Exception:
            pass
        self._thread.join(timeout=0.5)
        try:
            self._serial.close()
        except Exception:
            pass
