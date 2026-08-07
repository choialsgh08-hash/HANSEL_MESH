"""Hardware adapters for laptop simulation and Arduino Nano deployment.

The uploaded wiring plan assigns wheel motors, rear encoders, the 150 kg head
servo and SG90 detach servo to one Arduino Nano. Raspberry Pi owns only the
high-level ROS node and exchanges a compact newline protocol over USB serial.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import math
import threading
import time


class HardwareAdapter(ABC):
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
        """Run the SG90 detach press/rest sequence."""

    @abstractmethod
    def diagnostic_items(self) -> dict[str, str]:
        pass

    @abstractmethod
    def close(self) -> None:
        pass


class DummyHardwareAdapter(HardwareAdapter):
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
        D,<press_ms>,<press_us>,<rest_us>
        X
      Nano -> Pi
        T,<left_signed_cps>,<right_signed_cps>,<head_logical_angle>
        E,<message>
    """

    def __init__(self, port: str, baudrate: int = 115200,
                 read_timeout_s: float = 0.02,
                 front_follow_drive: bool = True) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required for nano_serial backend") from exc
        if not port:
            raise ValueError("nano_serial_port must not be empty")
        self.port_name = str(port)
        self.baudrate = int(baudrate)
        self.front_follow_drive = bool(front_follow_drive)
        self._serial = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            timeout=float(read_timeout_s),
            write_timeout=0.2,
        )
        time.sleep(1.8)
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._left_cps = self._right_cps = self._head_angle = 0.0
        self._last_state_monotonic: float | None = None
        self._invalid_lines = 0
        self._last_error = ""
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        self._send("Q")

    def _send(self, line: str) -> None:
        data = (line.rstrip("\n") + "\n").encode("ascii")
        with self._write_lock:
            self._serial.write(data)
            self._serial.flush()

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="strict").strip()
                parts = line.split(",")
                if parts[0] == "T" and len(parts) >= 4:
                    values = tuple(float(value) for value in parts[1:4])
                    if not all(math.isfinite(value) for value in values):
                        raise ValueError("non-finite Nano state")
                    with self._state_lock:
                        self._left_cps, self._right_cps, self._head_angle = values
                        self._last_state_monotonic = time.monotonic()
                elif parts[0] == "E":
                    self._last_error = ",".join(parts[1:])
            except Exception:
                self._invalid_lines += 1

    def update(self, left_pwm: float, right_pwm: float, drive_enabled: bool,
               dt: float) -> tuple[float, float]:
        del dt
        left = max(-100.0, min(100.0, float(left_pwm))) if drive_enabled else 0.0
        right = max(-100.0, min(100.0, float(right_pwm))) if drive_enabled else 0.0
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
        pwm = max(0.0, min(100.0, float(speed_scale) * 100.0))
        self._send(f"F,{mode},{pwm:.3f}")

    def set_head_angle(self, angle_deg: float) -> None:
        angle = max(-180.0, min(180.0, float(angle_deg)))
        self._send(f"H,{angle:.3f}")

    def configure_servo(self, **kwargs) -> None:
        self._send(
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

    def trigger_detach(self, duration_s: float, press_pulse_us: int,
                       rest_pulse_us: int) -> None:
        press_ms = int(max(50.0, min(2000.0, float(duration_s) * 1000.0)))
        self._send(f"D,{press_ms},{int(press_pulse_us)},{int(rest_pulse_us)}")

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
