#!/usr/bin/env python3
"""Standalone front-radar operator view for HANSEL_MESH.

The viewer deliberately stays separate from motor control and camera
processes.  It consumes the canonical mission JSONL produced by
``python -m sensors radar-live`` so only one process ever owns the radar UART.
Without IMU/odometry this is a current, robot-relative view rather than a map.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import random
import sys
import threading
import time
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple
from urllib.parse import urlsplit


REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from common.sensor_contract import (  # noqa: E402
    RadarFrame,
    RadarPoint,
    SensorHeader,
    SensorHealth,
)
from sensors.mission_log import (  # noqa: E402
    DEFAULT_MAX_LINE_BYTES,
    decode_log_entry,
    iter_replay,
)


WEB_ROOT = Path(__file__).resolve().parent / "web"
RADAR_STREAM_ID = "radar/front"
API_VERSION = 1


@dataclass(frozen=True)
class RadarAxes:
    """Map TI/native x-y values into screen forward/lateral coordinates.

    The IWRL6432 demo convention is X right and Y forward.  ``lateral_m`` in
    the web API is positive to screen/robot right.
    """

    forward_axis: str = "y"
    forward_sign: int = 1
    lateral_axis: str = "x"
    lateral_sign: int = 1

    def __post_init__(self) -> None:
        if self.forward_axis not in {"x", "y"}:
            raise ValueError("forward_axis must be x or y")
        if self.lateral_axis not in {"x", "y"}:
            raise ValueError("lateral_axis must be x or y")
        if self.forward_axis == self.lateral_axis:
            raise ValueError("forward_axis and lateral_axis must differ")
        if self.forward_sign not in {-1, 1}:
            raise ValueError("forward_sign must be -1 or 1")
        if self.lateral_sign not in {-1, 1}:
            raise ValueError("lateral_sign must be -1 or 1")

    def map_point(self, point: RadarPoint) -> Tuple[float, float]:
        values = {"x": point.x_m, "y": point.y_m}
        return (
            values[self.forward_axis] * self.forward_sign,
            values[self.lateral_axis] * self.lateral_sign,
        )

    def to_dict(self) -> Dict[str, object]:
        return {
            "forward_axis": self.forward_axis,
            "forward_sign": self.forward_sign,
            "lateral_axis": self.lateral_axis,
            "lateral_sign": self.lateral_sign,
            "lateral_positive": "right",
            "frame": "robot_relative_uncalibrated",
        }


class RadarFrontState:
    """Thread-safe latest-frame model with explicit stale/fault semantics."""

    def __init__(
        self,
        source_mode: str,
        axes: RadarAxes = RadarAxes(),
        max_points: int = 768,
        max_range_m: float = 20.0,
        min_forward_m: float = 0.05,
        stale_after_s: float = 0.75,
        fault_after_s: float = 2.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_points < 1:
            raise ValueError("max_points must be positive")
        if max_range_m <= 0 or not math.isfinite(max_range_m):
            raise ValueError("max_range_m must be finite and positive")
        if min_forward_m < 0 or not math.isfinite(min_forward_m):
            raise ValueError("min_forward_m must be finite and non-negative")
        if stale_after_s <= 0 or not math.isfinite(stale_after_s):
            raise ValueError("stale_after_s must be finite and positive")
        if fault_after_s <= stale_after_s or not math.isfinite(fault_after_s):
            raise ValueError("fault_after_s must be greater than stale_after_s")

        self.source_mode = source_mode
        self.axes = axes
        self.max_points = max_points
        self.max_range_m = max_range_m
        self.min_forward_m = min_forward_m
        self.stale_after_s = stale_after_s
        self.fault_after_s = fault_after_s
        self._clock = clock
        self._lock = threading.Lock()
        self._frame: Optional[Dict[str, object]] = None
        self._frame_received_at: Optional[float] = None
        self._arrival_times: Deque[float] = deque(maxlen=64)
        self._frames_received = 0
        self._valid_frames = 0
        self._incomplete_frames = 0
        self._frame_gaps_total = 0
        self._sensor_sequence_gaps_total = 0
        self._sensor_sequence_errors_total = 0
        self._producer_drops_total = 0
        self._writer_drops_total = 0
        self._parse_errors_total = 0
        self._device_discontinuities_total = 0
        self._log_sequence_errors_total = 0
        self._last_sensor_seq_by_producer: Dict[str, int] = {}
        self._source_error: Optional[str] = None
        self._source_note = "waiting for first radar frame"
        self._degraded_reason: Optional[str] = None
        self._health_status: Optional[str] = None
        self._health_detail: Optional[str] = None
        self._replay_ended = False

    def ingest(
        self,
        record: object,
        received_at: Optional[float] = None,
    ) -> bool:
        """Ingest one canonical sensor record.

        Incomplete point-cloud frames update diagnostics but never replace the
        last complete frame shown to the operator.
        """

        now = self._clock() if received_at is None else received_at
        if isinstance(record, SensorHealth):
            if record.subject_stream_id != RADAR_STREAM_ID:
                return False
            with self._lock:
                self._health_status = record.status
                self._health_detail = record.detail
                self._frame_gaps_total = max(
                    self._frame_gaps_total,
                    record.seq_gaps_total,
                )
                self._parse_errors_total = max(
                    self._parse_errors_total,
                    record.parse_errors_total,
                )
                self._producer_drops_total = max(
                    self._producer_drops_total,
                    record.producer_drops_total,
                )
                self._writer_drops_total = max(
                    self._writer_drops_total,
                    record.writer_drops_total,
                )
                self._device_discontinuities_total = max(
                    self._device_discontinuities_total,
                    record.device_discontinuities_total,
                )
                if record.status not in {"ok", "starting"}:
                    self._degraded_reason = f"sensor_health_{record.status}"
            return True

        if not isinstance(record, RadarFrame):
            return False
        if record.header.stream_id != RADAR_STREAM_ID:
            return False

        with self._lock:
            self._frames_received += 1
            self._replay_ended = False
            self._source_error = None
            self._source_note = "receiving radar frames"
            previous_sensor_seq = self._last_sensor_seq_by_producer.get(
                record.header.producer_id
            )
            sensor_sequence_issue: Optional[str] = None
            if previous_sensor_seq is not None:
                if record.header.seq > previous_sensor_seq + 1:
                    self._sensor_sequence_gaps_total += (
                        record.header.seq - previous_sensor_seq - 1
                    )
                    sensor_sequence_issue = "sensor_sequence_gap"
                elif record.header.seq <= previous_sensor_seq:
                    self._sensor_sequence_errors_total += 1
                    sensor_sequence_issue = "sensor_sequence_discontinuity"
            self._last_sensor_seq_by_producer[
                record.header.producer_id
            ] = record.header.seq
            self._frame_gaps_total += record.dropped_frames_since_previous
            if record.frame_transition in {
                "duplicate",
                "reset_or_out_of_order",
            }:
                self._device_discontinuities_total += 1
                self._degraded_reason = "device_frame_discontinuity"

            if not record.complete:
                self._incomplete_frames += 1
                self._degraded_reason = "incomplete_point_cloud"
                return False

            mapped: List[Tuple[float, List[Optional[float]]]] = []
            for point in record.points:
                forward_m, lateral_m = self.axes.map_point(point)
                distance_m = math.hypot(forward_m, lateral_m)
                if (
                    forward_m < self.min_forward_m
                    or distance_m > self.max_range_m
                ):
                    continue
                mapped.append(
                    (
                        distance_m,
                        [
                            round(forward_m, 5),
                            round(lateral_m, 5),
                            round(point.z_m, 5),
                            round(
                                point.radial_velocity_mps,
                                5,
                            ),
                            (
                                None
                                if point.snr_db is None
                                else round(point.snr_db, 3)
                            ),
                        ],
                    )
                )

            mapped.sort(key=lambda item: item[0])
            eligible_points = len(mapped)
            display_points = [
                item[1] for item in mapped[: self.max_points]
            ]
            nearest_corridor = min(
                (
                    item[0]
                    for item in mapped
                    if abs(float(item[1][1])) <= 0.6
                ),
                default=None,
            )
            self._frame = {
                "number": record.frame_number,
                "subframe": record.subframe_number,
                "seq": record.header.seq,
                "producer_id": record.header.producer_id,
                "transition": record.frame_transition,
                "dropped_since_previous": (
                    record.dropped_frames_since_previous
                ),
                "complete": True,
                "source_format": record.source_format,
                "sdk_version": record.sdk_version,
                "profile_id": record.profile_id,
                "calibration_id": record.header.calibration_id,
                "capture_baudrate": record.capture_baudrate,
                "source_point_count": len(record.points),
                "eligible_point_count": eligible_points,
                "display_point_count": len(display_points),
                "truncated": eligible_points > len(display_points),
                "point_fields": [
                    "forward_m",
                    "lateral_m",
                    "height_m",
                    "radial_velocity_mps",
                    "snr_db",
                ],
                "nearest_corridor_m": (
                    None
                    if nearest_corridor is None
                    else round(nearest_corridor, 3)
                ),
                "points": display_points,
            }
            self._frame_received_at = now
            self._arrival_times.append(now)
            self._valid_frames += 1
            if record.dropped_frames_since_previous:
                self._degraded_reason = "frame_gap"
            elif sensor_sequence_issue is not None:
                self._degraded_reason = sensor_sequence_issue
        return True

    def reset_sensor_sequence_tracking(self) -> None:
        """Forget replay-only sequence baselines without clearing diagnostics.

        Degraded diagnostics remain latched for the lifetime of this viewer
        process so a short gap cannot disappear between browser polls.
        """

        with self._lock:
            self._last_sensor_seq_by_producer.clear()

    def note_parse_error(self, detail: str) -> None:
        with self._lock:
            self._parse_errors_total += 1
            self._source_error = detail[:240]
            self._degraded_reason = "invalid_log_record"

    def note_log_sequence_error(self, detail: str) -> None:
        with self._lock:
            self._log_sequence_errors_total += 1
            self._source_error = detail[:240]
            self._degraded_reason = "log_sequence_discontinuity"

    def set_source_note(self, note: str) -> None:
        with self._lock:
            self._source_note = note[:240]

    def set_source_error(self, detail: str) -> None:
        with self._lock:
            self._source_error = detail[:240]
            self._source_note = "source error"

    def mark_replay_end(self) -> None:
        with self._lock:
            self._replay_ended = True
            self._source_note = "replay finished"

    def snapshot(self, now: Optional[float] = None) -> Dict[str, object]:
        current = self._clock() if now is None else now
        with self._lock:
            frame = self._frame
            received_at = self._frame_received_at
            age_s = None if received_at is None else max(0.0, current - received_at)
            fps = 0.0
            if len(self._arrival_times) > 1:
                elapsed = self._arrival_times[-1] - self._arrival_times[0]
                if elapsed > 0:
                    fps = (len(self._arrival_times) - 1) / elapsed

            if frame is None:
                status = "fault" if self._source_error else "waiting"
            elif self._replay_ended:
                status = "replay_end"
            elif age_s is not None and age_s > self.fault_after_s:
                status = "fault"
            elif age_s is not None and age_s > self.stale_after_s:
                status = "stale"
            elif (
                frame.get("source_format") == "ti-mmwave-none"
            ):
                status = "degraded"
            elif self._degraded_reason is not None:
                status = "degraded"
            else:
                status = "live"

            warning = self._warning_for(status, frame)
            return {
                "version": API_VERSION,
                "status": status,
                "warning": warning,
                "source": {
                    "mode": self.source_mode,
                    "note": self._source_note,
                    "error": self._source_error,
                },
                "age_ms": (
                    None if age_s is None else round(age_s * 1000.0)
                ),
                "fps": round(fps, 2),
                "axes": self.axes.to_dict(),
                "limits": {
                    "max_points": self.max_points,
                    "max_range_m": self.max_range_m,
                    "min_forward_m": self.min_forward_m,
                    "stale_after_ms": round(self.stale_after_s * 1000.0),
                    "fault_after_ms": round(self.fault_after_s * 1000.0),
                    "display_limit_only": True,
                },
                "frame": frame,
                "counters": {
                    "frames_received": self._frames_received,
                    "valid_frames": self._valid_frames,
                    "incomplete_frames": self._incomplete_frames,
                    "frame_gaps_total": self._frame_gaps_total,
                    "sensor_sequence_gaps_total": (
                        self._sensor_sequence_gaps_total
                    ),
                    "sensor_sequence_errors_total": (
                        self._sensor_sequence_errors_total
                    ),
                    "producer_drops_total": self._producer_drops_total,
                    "writer_drops_total": self._writer_drops_total,
                    "parse_errors_total": self._parse_errors_total,
                    "device_discontinuities_total": (
                        self._device_discontinuities_total
                    ),
                    "log_sequence_errors_total": (
                        self._log_sequence_errors_total
                    ),
                },
                "health": {
                    "status": self._health_status,
                    "detail": self._health_detail,
                    "degraded_reason": self._degraded_reason,
                },
            }

    def _warning_for(
        self,
        status: str,
        frame: Optional[Dict[str, object]],
    ) -> str:
        if status == "waiting":
            return "레이더 프레임 대기 중 — 주행하지 마세요"
        if status == "fault":
            return "RADAR FAULT — 즉시 정지"
        if status == "stale":
            return "RADAR STALE — 오래된 화면, 즉시 정지"
        if status == "replay_end":
            return "REPLAY END — 마지막 프레임이 고정되어 있습니다"
        if status == "degraded":
            if (
                frame is not None
                and frame.get("source_format") == "ti-mmwave-none"
            ):
                return (
                    "RADAR DEGRADED — point-cloud TLV 없음, cfg 확인"
                )
            return "RADAR DEGRADED — 누락/불완전 프레임 확인"
        if frame is not None and frame["display_point_count"] == 0:
            return "NO RETURNS — 빈 공간이라는 뜻이 아닙니다"
        return "현재 프레임 · 로봇 상대 좌표 (SLAM 아님)"


def make_demo_frame(
    frame_number: int,
    monotonic_ns: int,
    seed: int = 6432,
) -> RadarFrame:
    """Create a deterministic native-axis rubble scene for UI testing."""

    generator = random.Random(seed + frame_number)
    points: List[RadarPoint] = []
    if frame_number % 120 != 90:
        for side in (-1.0, 1.0):
            for index in range(24):
                forward = 0.65 + index * 0.31
                lateral = side * (
                    2.05
                    + 0.22 * math.sin(index * 0.55 + side)
                    + generator.uniform(-0.07, 0.07)
                )
                points.append(
                    RadarPoint(
                        x_m=lateral,
                        y_m=forward,
                        z_m=generator.uniform(-0.25, 0.55),
                        radial_velocity_mps=generator.uniform(-0.06, 0.06),
                        snr_db=generator.uniform(12.0, 27.0),
                        noise_db=generator.uniform(4.0, 8.0),
                    )
                )

        for _ in range(25):
            forward = generator.uniform(1.0, 7.4)
            center = 0.55 * math.sin(forward * 1.6)
            lateral = center + generator.uniform(-0.55, 0.55)
            points.append(
                RadarPoint(
                    x_m=lateral,
                    y_m=forward,
                    z_m=generator.uniform(-0.45, 0.8),
                    radial_velocity_mps=generator.uniform(-0.08, 0.08),
                    snr_db=generator.uniform(8.0, 23.0),
                    noise_db=generator.uniform(4.0, 9.0),
                )
            )

        phase = (frame_number % 160) / 160.0
        target_forward = 5.6 - 2.8 * phase
        target_lateral = 0.9 * math.sin(frame_number * 0.045)
        for _ in range(7):
            points.append(
                RadarPoint(
                    x_m=target_lateral + generator.uniform(-0.18, 0.18),
                    y_m=target_forward + generator.uniform(-0.15, 0.15),
                    z_m=generator.uniform(0.1, 1.2),
                    radial_velocity_mps=-0.7 + generator.uniform(-0.08, 0.08),
                    snr_db=generator.uniform(18.0, 31.0),
                    noise_db=generator.uniform(3.0, 7.0),
                )
            )

    dropped = 1 if frame_number > 0 and frame_number % 100 == 0 else 0
    transition = "gap" if dropped else (
        "first" if frame_number == 0 else "consecutive"
    )
    return RadarFrame(
        header=SensorHeader(
            mission_id="radar-view-demo",
            unit_id="head",
            boot_id="demo-boot",
            producer_id="demo-radar",
            stream_id=RADAR_STREAM_ID,
            seq=frame_number + 1,
            monotonic_ns=monotonic_ns,
            frame_id="radar_native",
            calibration_id="uncalibrated",
            timestamp_source="synthetic",
        ),
        frame_number=frame_number,
        subframe_number=0,
        complete=True,
        dropped_frames_since_previous=dropped,
        points=tuple(points),
        source_format="synthetic-iwrl6432",
        sdk_version="demo-1.0",
        frame_transition=transition,
        profile_id="demo-iwrl6432-front",
        capture_baudrate=115200,
    )


class DemoSource:
    def __init__(
        self,
        state: RadarFrontState,
        stop_event: threading.Event,
        rate_hz: float,
        seed: int,
    ) -> None:
        self.state = state
        self.stop_event = stop_event
        self.rate_hz = rate_hz
        self.seed = seed

    def run(self) -> None:
        frame_number = 0
        period = 1.0 / self.rate_hz
        self.state.set_source_note("synthetic IWRL6432-style scene")
        while not self.stop_event.is_set():
            started = time.monotonic()
            frame = make_demo_frame(
                frame_number,
                time.monotonic_ns(),
                self.seed,
            )
            self.state.ingest(frame, received_at=started)
            frame_number += 1
            self.stop_event.wait(max(0.0, period - (time.monotonic() - started)))


class MissionLogFollower:
    """Follow completed JSONL records without ever opening the radar UART."""

    def __init__(
        self,
        path: Path,
        state: RadarFrontState,
        stop_event: threading.Event,
        poll_s: float = 0.05,
    ) -> None:
        self.path = Path(path)
        self.state = state
        self.stop_event = stop_event
        self.poll_s = poll_s

    def run(self) -> None:
        expected_log_seq: Optional[int] = None
        while not self.stop_event.is_set():
            try:
                with self.path.open("rb") as handle:
                    self._seek_to_live_tail(handle)
                    expected_log_seq = None
                    self.state.set_source_note(f"following {self.path}")
                    expected_log_seq = self._follow_open_file(
                        handle,
                        expected_log_seq,
                    )
            except FileNotFoundError:
                self.state.set_source_note(f"waiting for {self.path}")
                self.stop_event.wait(self.poll_s)
            except OSError as exc:
                self.state.set_source_error(
                    f"cannot read mission log: {type(exc).__name__}: {exc}"
                )
                self.stop_event.wait(min(1.0, self.poll_s * 10.0))

    @staticmethod
    def _seek_to_live_tail(handle: object) -> None:
        """Skip completed backlog while preserving an in-progress last line."""

        handle.seek(0, os.SEEK_END)  # type: ignore[attr-defined]
        end = handle.tell()  # type: ignore[attr-defined]
        if end == 0:
            return

        handle.seek(end - 1)  # type: ignore[attr-defined]
        if handle.read(1) == b"\n":  # type: ignore[attr-defined]
            handle.seek(end)  # type: ignore[attr-defined]
            return

        scan_start = max(0, end - DEFAULT_MAX_LINE_BYTES - 1)
        handle.seek(scan_start)  # type: ignore[attr-defined]
        tail = handle.read(end - scan_start)  # type: ignore[attr-defined]
        last_newline = tail.rfind(b"\n")
        if last_newline < 0:
            handle.seek(scan_start)  # type: ignore[attr-defined]
        else:
            handle.seek(  # type: ignore[attr-defined]
                scan_start + last_newline + 1
            )

    def _follow_open_file(
        self,
        handle: object,
        expected_log_seq: Optional[int],
    ) -> Optional[int]:
        line_number = 0
        while not self.stop_event.is_set():
            position = handle.tell()  # type: ignore[attr-defined]
            raw = handle.readline(DEFAULT_MAX_LINE_BYTES + 2)  # type: ignore[attr-defined]
            if raw:
                if len(raw) > DEFAULT_MAX_LINE_BYTES:
                    self.state.note_parse_error("mission log line is too large")
                    if not raw.endswith(b"\n"):
                        self._skip_to_newline(handle)
                    expected_log_seq = None
                    continue
                if not raw.endswith(b"\n"):
                    handle.seek(position)  # type: ignore[attr-defined]
                    self.stop_event.wait(self.poll_s)
                    continue
                line_number += 1
                try:
                    entry = decode_log_entry(
                        raw[:-1].removesuffix(b"\r"),
                        line_number=line_number,
                    )
                except (
                    TypeError,
                    ValueError,
                    OverflowError,
                    UnicodeError,
                    RecursionError,
                ) as exc:
                    self.state.note_parse_error(
                        f"invalid mission log line {line_number}: {exc}"
                    )
                    expected_log_seq = None
                    continue
                if (
                    expected_log_seq is not None
                    and entry.log_seq != expected_log_seq
                ):
                    self.state.note_log_sequence_error(
                        f"expected log_seq {expected_log_seq}, "
                        f"got {entry.log_seq}"
                    )
                expected_log_seq = entry.log_seq + 1
                self.state.ingest(entry.record)
                continue

            try:
                current_path_stat = self.path.stat()
                open_stat = os.fstat(handle.fileno())  # type: ignore[attr-defined]
            except (FileNotFoundError, OSError):
                return expected_log_seq
            if (
                current_path_stat.st_size < position
                or (
                    getattr(current_path_stat, "st_ino", 0)
                    and getattr(open_stat, "st_ino", 0)
                    and current_path_stat.st_ino != open_stat.st_ino
                )
            ):
                return None
            self.stop_event.wait(self.poll_s)
        return expected_log_seq

    @staticmethod
    def _skip_to_newline(handle: object) -> None:
        while True:
            chunk = handle.readline(DEFAULT_MAX_LINE_BYTES + 2)  # type: ignore[attr-defined]
            if not chunk or chunk.endswith(b"\n"):
                return


class MissionLogReplay:
    def __init__(
        self,
        path: Path,
        state: RadarFrontState,
        stop_event: threading.Event,
        speed: float,
        loop: bool,
    ) -> None:
        self.path = Path(path)
        self.state = state
        self.stop_event = stop_event
        self.speed = speed
        self.loop = loop

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.state.set_source_note(f"replaying {self.path}")
                for entry in iter_replay(
                    self.path,
                    speed=self.speed,
                    sleep=self._interruptible_sleep,
                ):
                    if self.stop_event.is_set():
                        return
                    self.state.ingest(entry.record)
            except (OSError, RuntimeError, ValueError) as exc:
                self.state.set_source_error(
                    f"replay failed: {type(exc).__name__}: {exc}"
                )
                return
            if not self.loop:
                self.state.mark_replay_end()
                return
            self.state.reset_sensor_sequence_tracking()

    def _interruptible_sleep(self, seconds: float) -> None:
        self.stop_event.wait(seconds)


def build_handler(
    state: RadarFrontState,
    web_root: Path = WEB_ROOT,
    quiet: bool = False,
) -> type:
    static_files = {
        "/": ("radar_front.html", "text/html; charset=utf-8"),
        "/radar_front.html": (
            "radar_front.html",
            "text/html; charset=utf-8",
        ),
        "/radar_panel.js": (
            "radar_panel.js",
            "text/javascript; charset=utf-8",
        ),
    }

    class RadarRequestHandler(BaseHTTPRequestHandler):
        server_version = "HANSELRadarFront/1"

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/api/radar":
                payload = json.dumps(
                    state.snapshot(),
                    ensure_ascii=False,
                    separators=(",", ":"),
                    allow_nan=False,
                ).encode("utf-8")
                self._send(200, "application/json; charset=utf-8", payload)
                return
            static = static_files.get(path)
            if static is None:
                self._send(
                    404,
                    "text/plain; charset=utf-8",
                    b"not found\n",
                )
                return
            filename, content_type = static
            try:
                payload = (web_root / filename).read_bytes()
            except OSError:
                self._send(
                    500,
                    "text/plain; charset=utf-8",
                    b"radar UI asset unavailable\n",
                )
                return
            self._send(200, content_type, payload)

        def _send(
            self,
            status: int,
            content_type: str,
            payload: bytes,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format_text: str, *args: object) -> None:
            if not quiet:
                super().log_message(format_text, *args)

    return RadarRequestHandler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Serve an IMU-free, robot-relative front radar operator view. "
            "This tool never sends motor commands."
        )
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--demo",
        action="store_true",
        help="show a deterministic synthetic rubble scene",
    )
    source.add_argument(
        "--follow",
        metavar="MISSION.jsonl",
        help="follow the mission log written by sensors radar-live",
    )
    source.add_argument(
        "--replay",
        metavar="MISSION.jsonl",
        help="replay a completed mission log",
    )
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8081)
    parser.add_argument("--max-points", type=int, default=768)
    parser.add_argument("--max-range-m", type=float, default=20.0)
    parser.add_argument("--min-forward-m", type=float, default=0.05)
    parser.add_argument("--stale-after", type=float, default=0.75)
    parser.add_argument("--fault-after", type=float, default=2.0)
    parser.add_argument(
        "--forward-axis",
        choices=("x", "y"),
        default="y",
        help="native axis used as forward (TI IWRL6432 default: y)",
    )
    parser.add_argument(
        "--forward-sign",
        type=int,
        choices=(-1, 1),
        default=1,
    )
    parser.add_argument(
        "--lateral-axis",
        choices=("x", "y"),
        default="x",
        help="native axis used as right-positive lateral (TI default: x)",
    )
    parser.add_argument(
        "--lateral-sign",
        type=int,
        choices=(-1, 1),
        default=1,
    )
    parser.add_argument("--demo-rate-hz", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=6432)
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="replay speed; 0 replays as fast as possible",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="loop completed replay input",
    )
    parser.add_argument("--quiet", action="store_true")
    return parser


def parse_args(
    argv: Optional[Sequence[str]] = None,
) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.http_port <= 65535:
        parser.error("--http-port must be between 1 and 65535")
    if not 1 <= args.max_points <= 8192:
        parser.error("--max-points must be between 1 and 8192")
    if not math.isfinite(args.max_range_m) or args.max_range_m <= 0:
        parser.error("--max-range-m must be finite and positive")
    if not math.isfinite(args.min_forward_m) or args.min_forward_m < 0:
        parser.error("--min-forward-m must be finite and non-negative")
    if not math.isfinite(args.stale_after) or args.stale_after <= 0:
        parser.error("--stale-after must be finite and positive")
    if (
        not math.isfinite(args.fault_after)
        or args.fault_after <= args.stale_after
    ):
        parser.error("--fault-after must be greater than --stale-after")
    if not math.isfinite(args.demo_rate_hz) or args.demo_rate_hz <= 0:
        parser.error("--demo-rate-hz must be finite and positive")
    if not math.isfinite(args.speed) or args.speed < 0:
        parser.error("--speed must be finite and non-negative")
    if args.forward_axis == args.lateral_axis:
        parser.error("--forward-axis and --lateral-axis must differ")
    if args.loop and not args.replay:
        parser.error("--loop is only valid with --replay")
    return args


def run(args: argparse.Namespace) -> int:
    axes = RadarAxes(
        forward_axis=args.forward_axis,
        forward_sign=args.forward_sign,
        lateral_axis=args.lateral_axis,
        lateral_sign=args.lateral_sign,
    )
    mode = "demo" if args.demo else ("follow" if args.follow else "replay")
    state = RadarFrontState(
        source_mode=mode,
        axes=axes,
        max_points=args.max_points,
        max_range_m=args.max_range_m,
        min_forward_m=args.min_forward_m,
        stale_after_s=args.stale_after,
        fault_after_s=args.fault_after,
    )
    stop_event = threading.Event()
    if args.demo:
        source_runner = DemoSource(
            state,
            stop_event,
            args.demo_rate_hz,
            args.seed,
        ).run
    elif args.follow:
        source_runner = MissionLogFollower(
            Path(args.follow),
            state,
            stop_event,
        ).run
    else:
        source_runner = MissionLogReplay(
            Path(args.replay),
            state,
            stop_event,
            args.speed,
            args.loop,
        ).run

    source_thread = threading.Thread(
        target=source_runner,
        name=f"radar-front-{mode}",
        daemon=True,
    )
    source_thread.start()
    server = ThreadingHTTPServer(
        (args.bind, args.http_port),
        build_handler(state, quiet=args.quiet),
    )
    server.daemon_threads = True
    print(
        f"HANSEL front radar: http://{args.bind}:{args.http_port} "
        f"(mode={mode})"
    )
    print(
        "TI/native axes: "
        f"forward={args.forward_sign:+d}*{args.forward_axis}, "
        f"right={args.lateral_sign:+d}*{args.lateral_axis}"
    )
    print("Operator aid only: current robot-relative frame, no IMU/SLAM.")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print()
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        source_thread.join(timeout=2.0)
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        return run(parse_args(argv))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
