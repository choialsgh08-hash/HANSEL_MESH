"""Incremental evidence watchdog for one radar capture epoch."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import BinaryIO

from common.sensor_contract import RadarFrame
from sensors.mission_log import DEFAULT_MAX_LINE_BYTES, decode_log_entry


_FIRMWARE_LOW_POWER_TIMING_ASSERT = (
    b"Error: No Sufficient Time for getting into Low Power Modes."
)
_CONTINUITY_BYTES = 4 * 1024
_RAW_READ_BYTES = 64 * 1024
_MISSION_READ_BYTES = DEFAULT_MAX_LINE_BYTES + 1


@dataclass(frozen=True)
class ExpectedRadarEvidence:
    profile_id: str
    heatmap_azimuth_bins: int
    heatmap_range_bins: int
    heatmap_range_step_m: float


@dataclass(frozen=True)
class RadarWatchdogSnapshot:
    verified: bool
    consecutive_good_frames: int
    last_frame_observed_s: float | None
    latest_frame_number: int | None
    fault_reason: str | None


class RadarEpochWatchdog:
    """Tail mission and raw files while supervising one capture epoch."""

    def __init__(
        self,
        *,
        mission_path: Path,
        raw_path: Path,
        expected: ExpectedRadarEvidence,
        started_at_s: float,
        first_frame_timeout_s: float,
        frame_timeout_s: float,
        required_consecutive_frames: int,
        verification_timeout_s: float,
    ) -> None:
        self._mission_path = Path(mission_path)
        self._raw_path = Path(raw_path)
        self._expected = expected
        self._started_at_s = started_at_s
        self._first_frame_timeout_s = first_frame_timeout_s
        self._frame_timeout_s = frame_timeout_s
        self._required_consecutive_frames = required_consecutive_frames
        self._verification_timeout_s = verification_timeout_s

        self._mission_identity: tuple[int, int] | None = None
        self._mission_offset = 0
        self._mission_continuity_tail = b""
        self._mission_partial = b""
        self._mission_line_number = 0
        self._raw_identity: tuple[int, int] | None = None
        self._raw_offset = 0
        self._raw_continuity_tail = b""
        self._raw_overlap = b""

        self._verified = False
        self._consecutive_good_frames = 0
        self._last_frame_observed_s: float | None = None
        self._latest_frame_number: int | None = None
        self._fault_reason: str | None = None

    def poll(self, now_s: float) -> RadarWatchdogSnapshot:
        if self._fault_reason is None:
            self._poll_raw()
        if self._fault_reason is None:
            self._poll_mission(now_s)
        if self._fault_reason is None:
            self._apply_deadlines(now_s)
        return self._snapshot()

    def _poll_mission(self, now_s: float) -> None:
        handle = self._open_evidence(
            self._mission_path,
            self._mission_identity,
            "mission_evidence_invalid",
        )
        if handle is None:
            return
        with handle:
            stat = os.fstat(handle.fileno())
            identity = self._file_identity(stat)
            if self._mission_identity is None:
                self._mission_identity = identity
            elif identity != self._mission_identity:
                self._fault_reason = "mission_evidence_invalid"
                return
            if stat.st_size < self._mission_offset:
                self._fault_reason = "mission_evidence_invalid"
                return
            if not self._continuity_matches(
                handle,
                self._mission_offset,
                self._mission_continuity_tail,
            ):
                self._fault_reason = "mission_evidence_invalid"
                return
            handle.seek(self._mission_offset)
            chunk = handle.read(_MISSION_READ_BYTES)
            self._mission_offset += len(chunk)
            self._mission_continuity_tail = self._updated_continuity_tail(
                self._mission_continuity_tail,
                chunk,
            )

        buffered = self._mission_partial + chunk
        lines = buffered.split(b"\n")
        self._mission_partial = lines.pop()
        if len(self._mission_partial) > DEFAULT_MAX_LINE_BYTES:
            self._fault_reason = "mission_evidence_invalid"
            return

        for line in lines:
            if len(line) > DEFAULT_MAX_LINE_BYTES:
                self._fault_reason = "mission_evidence_invalid"
                return
            self._mission_line_number += 1
            try:
                entry = decode_log_entry(
                    line,
                    line_number=self._mission_line_number,
                )
            except (UnicodeError, ValueError):
                self._fault_reason = "mission_evidence_invalid"
                return
            if isinstance(entry.record, RadarFrame):
                self._observe_frame(entry.record, now_s)

    def _poll_raw(self) -> None:
        handle = self._open_evidence(
            self._raw_path,
            self._raw_identity,
            "raw_evidence_invalid",
        )
        if handle is None:
            return
        with handle:
            stat = os.fstat(handle.fileno())
            identity = self._file_identity(stat)
            if self._raw_identity is None:
                self._raw_identity = identity
            elif identity != self._raw_identity:
                self._fault_reason = "raw_evidence_invalid"
                return
            if stat.st_size < self._raw_offset:
                self._fault_reason = "raw_evidence_invalid"
                return
            if not self._continuity_matches(
                handle,
                self._raw_offset,
                self._raw_continuity_tail,
            ):
                self._fault_reason = "raw_evidence_invalid"
                return
            handle.seek(self._raw_offset)
            chunk = handle.read(_RAW_READ_BYTES)
            self._raw_offset += len(chunk)
            self._raw_continuity_tail = self._updated_continuity_tail(
                self._raw_continuity_tail,
                chunk,
            )

        candidate = self._raw_overlap + chunk
        if _FIRMWARE_LOW_POWER_TIMING_ASSERT in candidate:
            self._fault_reason = "firmware_low_power_timing_assert"
            return
        overlap_bytes = len(_FIRMWARE_LOW_POWER_TIMING_ASSERT) - 1
        self._raw_overlap = candidate[-overlap_bytes:]

    def _open_evidence(
        self,
        path: Path,
        identity: tuple[int, int] | None,
        invalid_reason: str,
    ) -> BinaryIO | None:
        try:
            return path.open("rb")
        except FileNotFoundError:
            if identity is not None:
                self._fault_reason = invalid_reason
            return None

    @staticmethod
    def _file_identity(stat: os.stat_result) -> tuple[int, int]:
        return stat.st_dev, stat.st_ino

    @staticmethod
    def _continuity_matches(
        handle: BinaryIO,
        offset: int,
        continuity_tail: bytes,
    ) -> bool:
        if not continuity_tail:
            return True
        handle.seek(offset - len(continuity_tail))
        return handle.read(len(continuity_tail)) == continuity_tail

    @staticmethod
    def _updated_continuity_tail(
        continuity_tail: bytes,
        chunk: bytes,
    ) -> bytes:
        return (continuity_tail + chunk)[-_CONTINUITY_BYTES:]

    def _observe_frame(self, frame: RadarFrame, now_s: float) -> None:
        self._last_frame_observed_s = now_s
        self._latest_frame_number = frame.frame_number
        if self._qualifies(frame):
            self._consecutive_good_frames += 1
            verification_deadline = (
                self._started_at_s + self._verification_timeout_s
            )
            if (
                self._consecutive_good_frames
                >= self._required_consecutive_frames
                and now_s < verification_deadline
            ):
                self._verified = True
        else:
            self._consecutive_good_frames = 0

    def _qualifies(self, frame: RadarFrame) -> bool:
        heatmap = frame.heatmap
        return (
            frame.complete
            and frame.profile_id == self._expected.profile_id
            and heatmap is not None
            and heatmap.azimuth_bins
            == self._expected.heatmap_azimuth_bins
            and heatmap.range_bins == self._expected.heatmap_range_bins
            and heatmap.range_step_m
            == self._expected.heatmap_range_step_m
        )

    def _apply_deadlines(self, now_s: float) -> None:
        if self._last_frame_observed_s is None:
            if now_s >= self._started_at_s + self._first_frame_timeout_s:
                self._fault_reason = "radar_frame_timeout"
            return
        if (
            now_s - self._last_frame_observed_s
            > self._frame_timeout_s
        ):
            self._fault_reason = "radar_frame_timeout"
            return
        if (
            not self._verified
            and now_s
            >= self._started_at_s + self._verification_timeout_s
        ):
            self._fault_reason = "radar_verification_timeout"

    def _snapshot(self) -> RadarWatchdogSnapshot:
        return RadarWatchdogSnapshot(
            verified=self._verified,
            consecutive_good_frames=self._consecutive_good_frames,
            last_frame_observed_s=self._last_frame_observed_s,
            latest_frame_number=self._latest_frame_number,
            fault_reason=self._fault_reason,
        )


__all__ = [
    "ExpectedRadarEvidence",
    "RadarEpochWatchdog",
    "RadarWatchdogSnapshot",
]
