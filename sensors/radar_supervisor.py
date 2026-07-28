"""Stable configuration and artifact contracts for radar supervision."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Mapping

from common.sensor_contract import validate_sensor_id


class SupervisorState(str, Enum):
    WAIT_PORT = "WAIT_PORT"
    RESET_TARGET = "RESET_TARGET"
    CONFIGURE = "CONFIGURE"
    START_CAPTURE = "START_CAPTURE"
    VERIFY_FRAMES = "VERIFY_FRAMES"
    SWITCH_VIEWER = "SWITCH_VIEWER"
    RUNNING = "RUNNING"
    RECOVERING = "RECOVERING"
    STOPPED = "STOPPED"


@dataclass(frozen=True)
class EpochPaths:
    mission: Path
    raw: Path
    raw_index: Path
    runtime_dir: Path
    capture_stdout: Path
    capture_stderr: Path
    viewer_stdout: Path
    viewer_stderr: Path


def _require_path(value: object, name: str) -> Path:
    if not isinstance(value, Path):
        raise ValueError(f"{name} must be a Path")
    return value


def _require_positive_int(value: object, name: str, maximum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if value <= 0 or (maximum is not None and value > maximum):
        raise ValueError(f"{name} is out of range")
    return value


def _require_positive_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return result


@dataclass(frozen=True)
class RadarSupervisorConfig:
    repository_root: Path
    output_root: Path
    profile_path: Path
    calibration_path: Path
    run_id: str
    mission_id: str = "radar-board-live"
    profile_id: str = (
        "lsdk-05.05.04.02-presence-near-"
        "heatmap16-elev8-cfar15-10hz-v1"
    )
    explicit_port: str | None = None
    xds_serial: str | None = None
    reset_executable: Path | None = None
    initial_baud: int = 115_200
    data_baud: int = 1_250_000
    heatmap_azimuth_bins: int = 16
    heatmap_range_bins: int = 128
    heatmap_range_step_m: float = 0.09765625
    first_frame_timeout_s: float = 3.0
    frame_timeout_s: float = 2.5
    verification_timeout_s: float = 3.0
    verification_frames: int = 5
    retry_initial_s: float = 0.5
    retry_max_s: float = 5.0
    poll_interval_s: float = 0.05
    http_bind: str = "127.0.0.1"
    http_port: int = 8081
    viewer_max_range_m: float = 3.0
    viewer_history_s: float = 0.3

    def __post_init__(self) -> None:
        for name in (
            "repository_root",
            "output_root",
            "profile_path",
            "calibration_path",
        ):
            _require_path(getattr(self, name), name)
        if self.reset_executable is not None:
            _require_path(self.reset_executable, "reset_executable")
        for name in ("run_id", "mission_id", "profile_id"):
            validate_sensor_id(getattr(self, name), name)
        for name in (
            "initial_baud",
            "data_baud",
            "heatmap_azimuth_bins",
            "heatmap_range_bins",
            "verification_frames",
        ):
            _require_positive_int(getattr(self, name), name)
        _require_positive_int(self.http_port, "http_port", 65_535)
        for name in (
            "heatmap_range_step_m",
            "first_frame_timeout_s",
            "frame_timeout_s",
            "verification_timeout_s",
            "retry_initial_s",
            "retry_max_s",
            "poll_interval_s",
            "viewer_max_range_m",
            "viewer_history_s",
        ):
            _require_positive_float(getattr(self, name), name)
        if self.retry_initial_s > self.retry_max_s:
            raise ValueError("retry_initial_s must not exceed retry_max_s")
        if not isinstance(self.http_bind, str) or not self.http_bind:
            raise ValueError("http_bind must be non-empty")


def _epoch_label(epoch: int) -> str:
    return f"e{epoch:03d}"


def allocate_epoch_paths(root: Path, run_id: str, epoch: int) -> EpochPaths:
    """Return unused paths for one capture epoch without creating them."""

    root = _require_path(root, "root")
    validate_sensor_id(run_id, "run_id")
    epoch = _require_positive_int(epoch, "epoch")
    prefix = f"radar-board-live-{run_id}-{_epoch_label(epoch)}"
    runtime_dir = root / "runtime" / f"radar-board-live-{run_id}"
    paths = EpochPaths(
        mission=root / "missions" / f"{prefix}.jsonl",
        raw=root / "captures" / f"{prefix}.bin",
        raw_index=root / "captures" / f"{prefix}.bin.chunks.jsonl",
        runtime_dir=runtime_dir,
        capture_stdout=runtime_dir / f"{_epoch_label(epoch)}-capture.stdout.log",
        capture_stderr=runtime_dir / f"{_epoch_label(epoch)}-capture.stderr.log",
        viewer_stdout=runtime_dir / f"{_epoch_label(epoch)}-viewer.stdout.log",
        viewer_stderr=runtime_dir / f"{_epoch_label(epoch)}-viewer.stderr.log",
    )
    artifacts = (
        paths.mission,
        paths.raw,
        paths.raw_index,
        paths.capture_stdout,
        paths.capture_stderr,
        paths.viewer_stdout,
        paths.viewer_stderr,
    )
    for artifact in artifacts:
        if artifact.exists():
            raise FileExistsError(f"epoch artifact already exists: {artifact}")
    return paths


def manifest_path(root: Path, run_id: str) -> Path:
    """Return the single-run manifest location without creating it."""

    root = _require_path(root, "root")
    validate_sensor_id(run_id, "run_id")
    return root / "runtime" / f"radar-supervisor-{run_id}.json"


def write_manifest_atomic(path: Path, payload: Mapping[str, object]) -> None:
    """Durably replace a manifest without exposing a partial destination."""

    path = _require_path(path, "path")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.",
            suffix=".tmp",
            dir=path.parent,
        )
        temporary = Path(temporary_name)
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        raise
