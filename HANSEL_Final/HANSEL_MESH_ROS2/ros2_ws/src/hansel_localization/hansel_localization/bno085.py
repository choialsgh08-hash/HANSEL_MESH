"""Lazy-loaded BNO085 sources; a read never reuses a cached sample."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .core import yaw_quaternion


@dataclass(frozen=True)
class Bno085Sample:
    quaternion_xyzw: tuple[float, float, float, float]


def normalized_quaternion_xyzw(
    quaternion: object,
    *,
    min_norm: float = 0.85,
    max_norm: float = 1.15,
) -> tuple[float, float, float, float]:
    """Validate and normalize one BNO085 quaternion before odometry uses it."""
    try:
        values = tuple(float(value) for value in quaternion)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError("BNO085 quaternion is not numeric") from exc
    if len(values) != 4:
        raise ValueError(f"BNO085 quaternion length is {len(values)}, expected 4")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("BNO085 quaternion contains a non-finite value")

    norm = math.sqrt(sum(value * value for value in values))
    if norm < min_norm or norm > max_norm:
        raise ValueError(f"BNO085 quaternion norm out of range: {norm:.3f}")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


class Bno085I2cSource:
    def __init__(self, frequency_hz: float = 20.0) -> None:
        import board  # type: ignore
        import busio  # type: ignore
        from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR  # type: ignore
        from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore

        if not math.isfinite(frequency_hz) or frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be positive")
        self._report_id = BNO_REPORT_GAME_ROTATION_VECTOR
        self._i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        try:
            self._sensor = BNO08X_I2C(self._i2c)
            interval_us = max(5000, int(1_000_000.0 / frequency_hz))
            self._sensor.enable_feature(
                BNO_REPORT_GAME_ROTATION_VECTOR, report_interval=interval_us
            )
        except Exception:
            self.close()
            raise

    def read(self) -> Bno085Sample | None:
        # Adafruit bno08x 1.3.3 has no public "new report" indicator. Its
        # property pumps packets then returns _readings, even when none arrived.
        # Keep the feature key but invalidate its value before each pump.
        self._sensor._readings[self._report_id] = None
        quaternion = self._sensor.game_quaternion
        if quaternion is None:
            return None
        # The Adafruit driver cache is all-zero until the first enabled sensor
        # report arrives. This is a startup wait condition, not a corrupt frame.
        try:
            if all(abs(float(value)) < 1e-9 for value in quaternion):
                return None
        except (TypeError, ValueError):
            pass
        return Bno085Sample(normalized_quaternion_xyzw(quaternion))

    def close(self) -> None:
        deinit = getattr(self._i2c, "deinit", None)
        if callable(deinit):
            deinit()


class RvcFrameBuffer:
    """Bounded stream framing around Adafruit's checksum/units decoder."""

    def __init__(self, decode_frame) -> None:
        self._decode_frame = decode_frame
        self._buffer = bytearray()
        self.valid_frames = 0
        self.invalid_frames = 0

    def clear(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> Bno085Sample | None:
        self._buffer.extend(data)
        latest = None
        while True:
            start = self._buffer.find(b"\xaa\xaa")
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer.endswith(b"\xaa") else b""
                return latest
            del self._buffer[:start]
            if len(self._buffer) < 19:
                return latest
            heading = self._decode_frame(bytes(self._buffer[2:19]))
            if heading is None:
                self.invalid_frames += 1
                del self._buffer[:1]
                continue
            del self._buffer[:19]
            yaw_deg = float(heading[0])
            if not math.isfinite(yaw_deg) or not -180.0 <= yaw_deg <= 180.0:
                self.invalid_frames += 1
                continue
            self.valid_frames += 1
            # Current odometry is planar: use RVC yaw, not raw acceleration
            # integration. A stationary robot can produce identical NEW frames.
            latest = Bno085Sample(yaw_quaternion(math.radians(yaw_deg)))


class Bno085RvcSource:
    """Nonblocking 115200-baud RVC input, independent of Nano's USB port."""

    def __init__(self, port: str = "/dev/serial0", max_age_s: float = 0.35) -> None:
        import serial  # type: ignore
        from adafruit_bno08x_rvc import BNO08x_RVC  # type: ignore

        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be positive")
        # Pin this private decoder to 1.0.24. Its public heading property clears
        # the buffer and blocks; it cannot safely service a ROS timer on silence.
        self.frames = RvcFrameBuffer(BNO08x_RVC._parse_frame)
        self._max_age_s = max_age_s
        self._last_poll_s = time.monotonic()
        self.backlog_drops = 0
        self.bytes_received = 0
        self.last_byte_s = None
        self._uart = serial.Serial(port, 115200, timeout=0, exclusive=True)
        try:
            self._uart.reset_input_buffer()
        except Exception:
            self.close()
            raise

    def read(self) -> Bno085Sample | None:
        now = time.monotonic()
        poll_gap = now - self._last_poll_s
        self._last_poll_s = now
        available = self._uart.in_waiting
        # RVC normally produces 19-byte packets at 100 Hz. After a scheduler
        # stall, discard the queue and wait for a new packet instead of dating
        # an old buffered heading as "now". Limit all reads to one snapshot.
        if poll_gap > self._max_age_s or available > 2048:
            self._uart.reset_input_buffer()
            self.frames.clear()
            self.backlog_drops += 1
            return None
        if not available:
            return None
        data = self._uart.read(available)
        self.bytes_received += len(data)
        if data:
            self.last_byte_s = now
        return self.frames.feed(data)

    def silence_reason(self) -> str:
        if self.last_byte_s is None:
            return "UART received 0 bytes; check BNO085 power, TX/RX wiring and RVC mode (115200)"
        if time.monotonic() - self.last_byte_s > self._max_age_s:
            return "UART byte stream stopped; check power and wiring"
        return "UART bytes received but no fresh valid RVC frame; check mode/baud/checksum"

    def close(self) -> None:
        self._uart.close()
