"""Injected contracts and verified initial startup for radar supervision."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Callable, Iterable, Mapping, Protocol

from common.sensor_contract import validate_sensor_id
from sensors.radar_watchdog import RadarWatchdogSnapshot
from sensors.ti_radar_control import (
    RadarPortIdentity,
    load_commands,
    select_application_port,
    validate_profile_result,
)


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


class SupervisorChild(Protocol):
    role: str
    pid: int

    def poll(self) -> int | None:
        raise NotImplementedError


class SupervisorStopResult(Protocol):
    role: str
    pid: int
    exit_code: int
    escalation: str


class RadarProcessManager(Protocol):
    def start_capture(
        self,
        port: RadarPortIdentity,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> SupervisorChild:
        raise NotImplementedError

    def switch_viewer(
        self,
        current: SupervisorChild | None,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> SupervisorChild:
        raise NotImplementedError

    def stop_child(
        self,
        child: SupervisorChild,
    ) -> SupervisorStopResult:
        raise NotImplementedError

    def stop_owned_children(self) -> tuple[SupervisorStopResult, ...]:
        raise NotImplementedError


class SupervisorWatchdog(Protocol):
    def poll(self, now_s: float) -> RadarWatchdogSnapshot:
        raise NotImplementedError


@dataclass(frozen=True)
class RadarSupervisorDependencies:
    port_provider: Callable[[], Iterable[object]]
    reset_target: Callable[
        [RadarPortIdentity, RadarSupervisorConfig],
        bool,
    ]
    configure: Callable[
        [RadarPortIdentity, RadarSupervisorConfig],
        Mapping[str, object],
    ]
    processes: RadarProcessManager
    watchdog_factory: Callable[
        [EpochPaths, RadarSupervisorConfig, float],
        SupervisorWatchdog,
    ]
    monotonic: Callable[[], float]
    sleep: Callable[[float], None]
    utc_now: Callable[[], datetime]


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


def _utc_timestamp(value: datetime) -> str:
    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError("utc_now must return an aware datetime")
    normalized = value.astimezone(timezone.utc).replace(microsecond=0)
    return normalized.isoformat().replace("+00:00", "Z")


class _InitialVerificationError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        reason: str,
        capture_exit_code: int | None = None,
    ) -> None:
        super().__init__(message)
        self.reason = reason
        self.capture_exit_code = capture_exit_code


class RadarSupervisor:
    """Start and verify the first radar capture epoch with injected effects."""

    def __init__(
        self,
        config: RadarSupervisorConfig,
        dependencies: RadarSupervisorDependencies,
    ) -> None:
        self._config = config
        self._dependencies = dependencies
        self._state = SupervisorState.WAIT_PORT
        self._epoch = 0
        self._recovery_count = 0
        self._port: RadarPortIdentity | None = None
        self._paths: EpochPaths | None = None
        self._epochs: list[dict[str, object]] = []
        self._process_events: list[dict[str, object]] = []
        self._owned_children: dict[int, SupervisorChild] = {}
        self._capture_pid: int | None = None
        self._process_manager_engaged = False
        self._verified_consecutive_frames = 0
        self._last_reason = "initialized"

    def run(self, stop_requested: Callable[[], bool]) -> None:
        self._transition(SupervisorState.WAIT_PORT, "waiting_for_port")
        port = self._wait_for_port(stop_requested)
        if port is None:
            self._shutdown()
            return
        self._port = port

        self._transition(SupervisorState.RESET_TARGET, "initial_reset")
        self._dependencies.reset_target(port, self._config)

        self._transition(SupervisorState.WAIT_PORT, "waiting_for_port")
        port = self._wait_for_port(stop_requested)
        if port is None:
            self._shutdown()
            return
        self._port = port

        self._transition(SupervisorState.CONFIGURE, "configuring")
        profile_result = self._dependencies.configure(port, self._config)
        commands = load_commands(self._config.profile_path)
        try:
            validate_profile_result(
                profile_result,
                expected_commands=len(commands),
                require_first_magic=False,
            )
        except Exception:
            self._transition(SupervisorState.STOPPED, "configuration_failed")
            raise

        self._epoch = 1
        self._paths = allocate_epoch_paths(
            self._config.output_root,
            self._config.run_id,
            self._epoch,
        )
        started_at_s = self._dependencies.monotonic()
        started_at = _utc_timestamp(self._dependencies.utc_now())
        self._epochs.append(
            {
                "epoch": self._epoch,
                "mission_path": str(self._paths.mission),
                "raw_path": str(self._paths.raw),
                "raw_index_path": str(self._paths.raw_index),
                "started_at": started_at,
                "ended_at": None,
                "end_reason": None,
                "capture_exit_code": None,
            }
        )

        self._transition(SupervisorState.START_CAPTURE, "starting_capture")
        self._process_manager_engaged = True
        try:
            capture = self._dependencies.processes.start_capture(
                port,
                self._paths,
                self._config,
            )
            self._register_started_child(capture, expected_role="capture")
            self._capture_pid = capture.pid
            self._transition(
                SupervisorState.VERIFY_FRAMES,
                "verifying_frames",
            )

            watchdog = self._dependencies.watchdog_factory(
                self._paths,
                self._config,
                started_at_s,
            )
            snapshot = self._wait_until_verified(
                capture,
                watchdog,
                stop_requested,
            )
            if snapshot is None:
                self._shutdown()
                return

            self._verified_consecutive_frames = (
                snapshot.consecutive_good_frames
            )
            self._transition(
                SupervisorState.SWITCH_VIEWER,
                "switching_viewer",
            )
            viewer = self._dependencies.processes.switch_viewer(
                None,
                self._paths,
                self._config,
            )
            self._register_started_child(viewer, expected_role="viewer")
        except Exception as exc:
            reason = "initial_startup_failed"
            capture_exit_code = None
            if isinstance(exc, _InitialVerificationError):
                reason = exc.reason
                capture_exit_code = exc.capture_exit_code
            if capture_exit_code is not None:
                self._set_capture_exit_code(capture_exit_code)
            self._stop_registered_children()
            self._finalize_epoch(reason=reason)
            self._transition(SupervisorState.STOPPED, reason)
            raise

        self._transition(SupervisorState.RUNNING, "verified_frames")

        while not stop_requested():
            self._dependencies.sleep(self._config.poll_interval_s)
        self._shutdown()

    def _wait_for_port(
        self,
        stop_requested: Callable[[], bool],
    ) -> RadarPortIdentity | None:
        while True:
            if stop_requested():
                return None
            try:
                return select_application_port(
                    self._dependencies.port_provider(),
                    explicit_port=self._config.explicit_port,
                    xds_serial=self._config.xds_serial,
                )
            except RuntimeError:
                if stop_requested():
                    return None
                self._dependencies.sleep(self._config.poll_interval_s)

    def _wait_until_verified(
        self,
        capture: SupervisorChild,
        watchdog: SupervisorWatchdog,
        stop_requested: Callable[[], bool],
    ) -> RadarWatchdogSnapshot | None:
        while True:
            if stop_requested():
                return None
            exit_code = capture.poll()
            if exit_code is not None:
                raise _InitialVerificationError(
                    f"radar capture exited before verification: {exit_code}",
                    reason="capture_exited_before_verification",
                    capture_exit_code=exit_code,
                )
            snapshot = watchdog.poll(self._dependencies.monotonic())
            if snapshot.fault_reason is not None:
                raise _InitialVerificationError(
                    f"radar watchdog fault before verification: "
                    f"{snapshot.fault_reason}",
                    reason=snapshot.fault_reason,
                )
            if snapshot.verified is True:
                return snapshot
            self._dependencies.sleep(self._config.poll_interval_s)

    def _register_started_child(
        self,
        child: SupervisorChild,
        *,
        expected_role: str,
    ) -> None:
        if child.role != expected_role:
            raise RuntimeError(
                f"expected {expected_role!r} child, got {child.role!r}"
            )
        if (
            isinstance(child.pid, bool)
            or not isinstance(child.pid, int)
            or child.pid <= 0
            or child.pid in self._owned_children
        ):
            raise RuntimeError(f"invalid or duplicate owned child PID: {child.pid!r}")
        self._owned_children[child.pid] = child
        self._process_events.append(
            {
                "role": child.role,
                "pid": child.pid,
                "action": "started",
                "escalation": None,
                "exit_code": None,
            }
        )

    def _stop_registered_children(self) -> None:
        if not self._process_manager_engaged:
            return
        results = self._dependencies.processes.stop_owned_children()
        self._process_manager_engaged = False
        for result in results:
            child = self._owned_children.get(result.pid)
            if child is None or child.role != result.role:
                continue
            self._process_events.append(
                {
                    "role": result.role,
                    "pid": result.pid,
                    "action": "stopped",
                    "escalation": result.escalation,
                    "exit_code": result.exit_code,
                }
            )
            if result.pid == self._capture_pid:
                self._set_capture_exit_code(result.exit_code)

    def _set_capture_exit_code(self, exit_code: int) -> None:
        if self._epochs and self._epochs[-1]["capture_exit_code"] is None:
            self._epochs[-1]["capture_exit_code"] = exit_code

    def _finalize_epoch(
        self,
        *,
        reason: str,
        capture_exit_code: int | None = None,
    ) -> None:
        if not self._epochs or self._epochs[-1]["ended_at"] is not None:
            return
        if capture_exit_code is not None:
            self._set_capture_exit_code(capture_exit_code)
        self._epochs[-1]["ended_at"] = _utc_timestamp(
            self._dependencies.utc_now()
        )
        self._epochs[-1]["end_reason"] = reason

    def _shutdown(self) -> None:
        self._stop_registered_children()
        self._finalize_epoch(reason="shutdown")
        self._transition(SupervisorState.STOPPED, "shutdown")

    def _transition(self, state: SupervisorState, reason: str) -> None:
        self._state = state
        self._last_reason = reason
        write_manifest_atomic(
            manifest_path(self._config.output_root, self._config.run_id),
            self._manifest_payload(),
        )

    def _manifest_payload(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "run_id": self._config.run_id,
            "state": self._state.value,
            "epoch": self._epoch,
            "recovery_count": self._recovery_count,
            "port": self._port.device if self._port is not None else None,
            "xds_serial": (
                self._port.serial_number
                if self._port is not None
                else self._config.xds_serial
            ),
            "mission_path": (
                str(self._paths.mission) if self._paths is not None else None
            ),
            "raw_path": (
                str(self._paths.raw) if self._paths is not None else None
            ),
            "last_reason": self._last_reason,
            "verified_consecutive_frames": self._verified_consecutive_frames,
            "epochs": [dict(epoch) for epoch in self._epochs],
            "process_events": [
                dict(process_event) for process_event in self._process_events
            ],
        }


__all__ = [
    "EpochPaths",
    "RadarProcessManager",
    "RadarSupervisor",
    "RadarSupervisorConfig",
    "RadarSupervisorDependencies",
    "SupervisorChild",
    "SupervisorState",
    "SupervisorStopResult",
    "SupervisorWatchdog",
    "allocate_epoch_paths",
    "manifest_path",
    "write_manifest_atomic",
]
