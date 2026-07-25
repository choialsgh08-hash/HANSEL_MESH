"""Common monotonic time/identity source for Head Pi sensor producers."""

from __future__ import annotations

from pathlib import Path
import threading
import time
import uuid
from typing import Callable, Optional

from common.sensor_contract import SensorHeader


LINUX_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")


def read_linux_boot_id(path: Path = LINUX_BOOT_ID_PATH) -> str:
    """Read the kernel boot UUID used to separate monotonic clock domains."""

    try:
        value = Path(path).read_text(encoding="ascii").strip()
    except OSError as exc:
        raise RuntimeError(f"cannot read Linux boot ID from {path}") from exc
    if not value:
        raise RuntimeError(f"Linux boot ID is empty: {path}")
    return value


def new_producer_id(prefix: str) -> str:
    if not prefix or any(character.isspace() for character in prefix):
        raise ValueError("producer prefix must be non-empty and contain no spaces")
    return f"{prefix}-{uuid.uuid4()}"


class SensorHeaderFactory:
    """Thread-safe sequence and Head-host timestamp generator for one stream."""

    def __init__(
        self,
        mission_id: str,
        unit_id: str,
        boot_id: str,
        producer_id: str,
        stream_id: str,
        frame_id: Optional[str] = None,
        calibration_id: Optional[str] = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        wall_time_ns: Callable[[], int] = time.time_ns,
        include_wall_time: bool = False,
        timestamp_source: str = "host_capture",
    ) -> None:
        self.mission_id = mission_id
        self.unit_id = unit_id
        self.boot_id = boot_id
        self.producer_id = producer_id
        self.stream_id = stream_id
        self.frame_id = frame_id
        self.calibration_id = calibration_id
        self._monotonic_ns = monotonic_ns
        self._wall_time_ns = wall_time_ns
        self._include_wall_time = include_wall_time
        self.timestamp_source = timestamp_source
        self._next_seq = 1
        self._last_monotonic_ns: Optional[int] = None
        self._lock = threading.Lock()

        # Validate all static fields before a producer starts hardware I/O.
        SensorHeader(
            mission_id=mission_id,
            unit_id=unit_id,
            boot_id=boot_id,
            producer_id=producer_id,
            stream_id=stream_id,
            seq=1,
            monotonic_ns=0,
            frame_id=frame_id,
            calibration_id=calibration_id,
            timestamp_source=timestamp_source,
        )

    def next(
        self,
        capture_monotonic_ns: Optional[int] = None,
        sensor_timestamp_ns: Optional[int] = None,
        timestamp_uncertainty_ns: Optional[int] = None,
    ) -> SensorHeader:
        """Create the next header.

        Pass ``capture_monotonic_ns`` when the driver already produced a
        host-clock observation anchor.  Also pass the source-specific timing
        quality value when the exact measurement time is unavailable.  For
        buffered USB/UART this value is heuristic, not a strict error bound.
        Otherwise the factory samples the host clock immediately.
        """

        with self._lock:
            captured = (
                self._monotonic_ns()
                if capture_monotonic_ns is None
                else capture_monotonic_ns
            )
            if (
                self._last_monotonic_ns is not None
                and captured < self._last_monotonic_ns
            ):
                raise ValueError(
                    "capture_monotonic_ns regressed within one stream"
                )
            wall = self._wall_time_ns() if self._include_wall_time else None
            seq = self._next_seq
            header = SensorHeader(
                mission_id=self.mission_id,
                unit_id=self.unit_id,
                boot_id=self.boot_id,
                producer_id=self.producer_id,
                stream_id=self.stream_id,
                seq=seq,
                monotonic_ns=captured,
                sensor_timestamp_ns=sensor_timestamp_ns,
                frame_id=self.frame_id,
                calibration_id=self.calibration_id,
                wall_time_ns=wall,
                timestamp_source=self.timestamp_source,
                timestamp_uncertainty_ns=timestamp_uncertainty_ns,
            )
            self._next_seq += 1
            self._last_monotonic_ns = captured
            return header
