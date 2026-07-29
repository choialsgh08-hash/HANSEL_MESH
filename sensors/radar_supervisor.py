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
import time
from typing import Callable, Iterable, Mapping, Protocol

from common.sensor_contract import validate_sensor_id
from sensors.radar_owner_lock import (
    RADAR_OWNER_LOCK_ROOT,
    RADAR_UART_LOCK_ROOT,
    RadarOwnerLock,
    acquire_radar_owner_lock,
)
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
    reset_unavailable_reason: str | None = None
    initial_baud: int = 115_200
    data_baud: int = 1_250_000
    heatmap_azimuth_bins: int = 16
    heatmap_range_bins: int = 128
    heatmap_range_step_m: float = 0.09765625
    first_frame_timeout_s: float = 3.0
    frame_timeout_s: float = 2.5
    verification_timeout_s: float = 5.0
    verification_frames: int = 30
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
        if (
            self.reset_unavailable_reason is not None
            and not isinstance(self.reset_unavailable_reason, str)
        ):
            raise ValueError("reset_unavailable_reason must be a string or None")
        if (
            self.reset_executable is not None
            and self.reset_unavailable_reason is not None
        ):
            raise ValueError(
                "reset_unavailable_reason requires reset_executable to be None"
            )
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
        replace_retry_delays_s = (0.01, 0.02, 0.04, 0.08, 0.16)
        for delay_s in replace_retry_delays_s:
            try:
                os.replace(temporary, path)
            except PermissionError:
                time.sleep(delay_s)
            else:
                break
        else:
            os.replace(temporary, path)
    except BaseException:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        raise


def reserve_manifest(path: Path, payload: Mapping[str, object]) -> None:
    """Create the first run manifest exclusively without replacing history."""

    path = _require_path(path, "path")
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        descriptor = os.open(
            path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o600,
        )
    except FileExistsError as exc:
        raise FileExistsError(
            f"radar supervisor manifest already exists: {path}"
        ) from exc
    try:
        with os.fdopen(
            descriptor,
            "w",
            encoding="utf-8",
            newline="\n",
        ) as handle:
            json.dump(
                payload,
                handle,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
    except BaseException:
        try:
            path.unlink()
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


class _ViewerRetryFault(RuntimeError):
    def __init__(
        self,
        fault: tuple[str, int | None, bool],
    ) -> None:
        reason, capture_exit_code, port_absence_seen = fault
        super().__init__(f"radar fault during viewer retry: {reason}")
        self.reason = reason
        self.capture_exit_code = capture_exit_code
        self.port_absence_seen = port_absence_seen


class RadarSupervisor:
    """Supervise verified radar capture epochs using only injected effects."""

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
        self._latched_xds_serial: str | None = None
        self._owner_lock: RadarOwnerLock | None = None
        self._uart_lock: RadarOwnerLock | None = None
        self._paths: EpochPaths | None = None
        self._verified_paths: EpochPaths | None = None
        self._epochs: list[dict[str, object]] = []
        self._process_events: list[dict[str, object]] = []
        self._owned_children: dict[int, SupervisorChild] = {}
        self._active_capture: SupervisorChild | None = None
        self._active_viewer: SupervisorChild | None = None
        self._active_watchdog: SupervisorWatchdog | None = None
        self._process_manager_engaged = False
        self._verified_consecutive_frames = 0
        self._last_reason = "initialized"
        self._retry_delay_s = self._config.retry_initial_s
        self._fatal_reason = "initial_startup_failed"

    def run(self, stop_requested: Callable[[], bool]) -> None:
        reserve_manifest(
            manifest_path(self._config.output_root, self._config.run_id),
            self._manifest_payload(),
        )
        try:
            try:
                if not self._attempt_until_running(
                    stop_requested,
                    recovering=False,
                    usb_absence_seen=False,
                ):
                    self._shutdown()
                    return

                while True:
                    fault = self._monitor_until_fault(stop_requested)
                    if fault is None:
                        self._shutdown()
                        return
                    reason, capture_exit_code, port_absence_seen = fault
                    self._recovery_count += 1
                    self._transition(SupervisorState.RECOVERING, reason)
                    if capture_exit_code is not None:
                        self._set_capture_exit_code(capture_exit_code)
                    if self._active_capture is not None:
                        self._stop_active_child(self._active_capture)
                    self._active_watchdog = None
                    self._finalize_epoch(reason=reason)
                    if not self._backoff(stop_requested):
                        self._shutdown()
                        return
                    if not self._attempt_until_running(
                        stop_requested,
                        recovering=True,
                        usb_absence_seen=port_absence_seen,
                    ):
                        self._shutdown()
                        return
            except Exception:
                cleanup_error: BaseException | None = None
                try:
                    self._stop_registered_children()
                except BaseException as error:
                    cleanup_error = error
                self._finalize_epoch(reason=self._fatal_reason)
                self._restore_verified_paths()
                self._transition(SupervisorState.STOPPED, self._fatal_reason)
                if cleanup_error is not None:
                    raise cleanup_error
                raise
        finally:
            if self._uart_lock is not None:
                self._uart_lock.release()
                self._uart_lock = None
            if self._owner_lock is not None:
                self._owner_lock.release()
                self._owner_lock = None

    def _attempt_until_running(
        self,
        stop_requested: Callable[[], bool],
        *,
        recovering: bool,
        usb_absence_seen: bool,
    ) -> bool:
        while True:
            attempt_usb_absence_seen = usb_absence_seen
            usb_absence_seen = False
            self._transition(SupervisorState.WAIT_PORT, "waiting_for_port")
            port = self._wait_for_port(stop_requested)
            if port is None:
                return False
            self._port = port
            if self._owner_lock is None:
                self._owner_lock = acquire_radar_owner_lock(
                    RADAR_OWNER_LOCK_ROOT,
                    self._latched_xds_serial or port.serial_number,
                    self._config.run_id,
                )
            if self._uart_lock is None:
                self._uart_lock = acquire_radar_owner_lock(
                    RADAR_UART_LOCK_ROOT,
                    self._latched_xds_serial or port.serial_number,
                    self._config.run_id,
                )

            reset_reason = "recovery_reset" if recovering else "initial_reset"
            self._transition(SupervisorState.RESET_TARGET, reset_reason)
            try:
                reset_executed = self._dependencies.reset_target(
                    port,
                    self._config,
                )
            except Exception:
                self._transition(SupervisorState.RESET_TARGET, "reset_failed")
                if not self._backoff(stop_requested):
                    return False
                continue

            if reset_executed or not recovering:
                self._transition(SupervisorState.WAIT_PORT, "waiting_for_port")
                port = self._wait_for_port(stop_requested)
                if port is None:
                    return False
                self._port = port
            elif not attempt_usb_absence_seen:
                port = self._wait_for_usb_cycle(stop_requested)
                if port is None:
                    return False
                self._port = port

            self._transition(SupervisorState.CONFIGURE, "configuring")
            try:
                profile_result = self._dependencies.configure(
                    self._port,
                    self._config,
                )
                commands = load_commands(self._config.profile_path)
                validate_profile_result(
                    profile_result,
                    expected_commands=len(commands),
                    require_first_magic=False,
                )
            except Exception:
                self._transition(
                    SupervisorState.CONFIGURE,
                    "configuration_failed",
                )
                if not self._backoff(stop_requested):
                    return False
                continue

            self._epoch += 1
            self._paths = allocate_epoch_paths(
                self._config.output_root,
                self._config.run_id,
                self._epoch,
            )
            started_at_s = self._dependencies.monotonic()
            self._epochs.append(
                {
                    "epoch": self._epoch,
                    "mission_path": str(self._paths.mission),
                    "raw_path": str(self._paths.raw),
                    "raw_index_path": str(self._paths.raw_index),
                    "started_at": _utc_timestamp(
                        self._dependencies.utc_now()
                    ),
                    "ended_at": None,
                    "end_reason": None,
                    "capture_exit_code": None,
                }
            )

            self._transition(SupervisorState.START_CAPTURE, "starting_capture")
            self._process_manager_engaged = True
            if self._uart_lock is None:
                raise RuntimeError(
                    "capture start requires the verified UART ownership lease"
                )
            self._uart_lock.release()
            self._uart_lock = None
            try:
                capture = self._dependencies.processes.start_capture(
                    self._port,
                    self._paths,
                    self._config,
                )
            except Exception:
                self._finalize_epoch(reason="capture_start_failed")
                self._restore_verified_paths()
                self._transition(
                    SupervisorState.RECOVERING,
                    "capture_start_failed",
                )
                if not self._backoff(stop_requested):
                    return False
                continue
            self._register_started_child(capture, expected_role="capture")
            self._active_capture = capture
            self._persist_manifest()

            self._transition(SupervisorState.VERIFY_FRAMES, "verifying_frames")
            try:
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
            except _InitialVerificationError as exc:
                if exc.capture_exit_code is not None:
                    self._set_capture_exit_code(exc.capture_exit_code)
                if self._active_capture is not None:
                    self._stop_active_child(self._active_capture)
                self._finalize_epoch(reason=exc.reason)
                self._restore_verified_paths()
                self._transition(SupervisorState.RECOVERING, exc.reason)
                if not self._backoff(stop_requested):
                    return False
                continue
            if snapshot is None:
                return False

            self._active_watchdog = watchdog
            self._verified_paths = self._paths
            self._verified_consecutive_frames = (
                snapshot.consecutive_good_frames
            )
            self._transition(SupervisorState.SWITCH_VIEWER, "switching_viewer")
            if self._active_viewer is not None:
                self._stop_active_child(self._active_viewer)
            self._fatal_reason = (
                "viewer_switch_failed"
                if recovering
                else "initial_startup_failed"
            )
            try:
                viewer = self._start_viewer_with_retry(
                    self._paths,
                    stop_requested,
                    reason="viewer_switch_failed",
                )
            except _ViewerRetryFault as exc:
                if exc.capture_exit_code is not None:
                    self._set_capture_exit_code(exc.capture_exit_code)
                if self._active_capture is not None:
                    self._stop_active_child(self._active_capture)
                self._active_watchdog = None
                self._finalize_epoch(reason=exc.reason)
                self._restore_verified_paths()
                self._transition(SupervisorState.RECOVERING, exc.reason)
                usb_absence_seen = exc.port_absence_seen
                if not self._backoff(stop_requested):
                    return False
                continue
            if viewer is None:
                return False
            self._register_started_child(viewer, expected_role="viewer")
            self._active_viewer = viewer
            self._persist_manifest()

            self._fatal_reason = "running_manifest_write_failed"
            self._transition(SupervisorState.RUNNING, "verified_frames")
            return True

    def _monitor_until_fault(
        self,
        stop_requested: Callable[[], bool],
    ) -> tuple[str, int | None, bool] | None:
        if (
            self._active_capture is None
            or self._active_viewer is None
            or self._active_watchdog is None
            or self._paths is None
        ):
            raise RuntimeError("RUNNING requires active radar children and watchdog")
        while True:
            self._fatal_reason = "running_stop_check_failed"
            if stop_requested():
                return None

            capture_exit_code = self._active_capture.poll()
            if capture_exit_code is not None:
                return "capture_exited", capture_exit_code, False

            present, _ = self._matching_port_inventory()
            if not present:
                return "application_port_lost", None, True

            snapshot = self._active_watchdog.poll(
                self._dependencies.monotonic()
            )
            self._verified_consecutive_frames = (
                snapshot.consecutive_good_frames
            )
            if snapshot.fault_reason is not None:
                return snapshot.fault_reason, None, False

            if self._active_viewer.poll() is not None:
                viewer = self._active_viewer
                self._stop_active_child(viewer)
                self._fatal_reason = "viewer_restart_failed"
                self._transition(
                    SupervisorState.SWITCH_VIEWER,
                    "viewer_restarting",
                )
                try:
                    replacement = self._start_viewer_with_retry(
                        self._paths,
                        stop_requested,
                        reason="viewer_restart_failed",
                    )
                except _ViewerRetryFault as exc:
                    return (
                        exc.reason,
                        exc.capture_exit_code,
                        exc.port_absence_seen,
                    )
                if replacement is None:
                    return None
                self._register_started_child(
                    replacement,
                    expected_role="viewer",
                )
                self._active_viewer = replacement
                self._persist_manifest()
                self._transition(SupervisorState.RUNNING, "viewer_restarted")

            self._fatal_reason = "running_sleep_failed"
            self._dependencies.sleep(self._config.poll_interval_s)

    def _start_viewer_with_retry(
        self,
        paths: EpochPaths,
        stop_requested: Callable[[], bool],
        *,
        reason: str,
    ) -> SupervisorChild | None:
        retrying = False
        while True:
            if stop_requested():
                return None
            if retrying:
                fault = self._poll_retained_radar_fault()
                if fault is not None:
                    raise _ViewerRetryFault(fault)
            try:
                viewer = self._dependencies.processes.switch_viewer(
                    None,
                    paths,
                    self._config,
                )
            except Exception:
                self._transition(SupervisorState.SWITCH_VIEWER, reason)
                fault = self._poll_retained_radar_fault()
                if fault is not None:
                    raise _ViewerRetryFault(fault)
                if not self._backoff(stop_requested):
                    return None
                retrying = True
                continue

            fault = self._poll_retained_radar_fault()
            if fault is not None:
                self._register_started_child(
                    viewer,
                    expected_role="viewer",
                )
                self._active_viewer = viewer
                self._persist_manifest()
                self._stop_active_child(viewer)
                raise _ViewerRetryFault(fault)
            return viewer

    def _poll_retained_radar_fault(
        self,
    ) -> tuple[str, int | None, bool] | None:
        if self._active_capture is None or self._active_watchdog is None:
            raise RuntimeError(
                "viewer retry requires an active capture and watchdog"
            )
        capture_exit_code = self._active_capture.poll()
        if capture_exit_code is not None:
            return "capture_exited", capture_exit_code, False

        present, _ = self._matching_port_inventory()
        if not present:
            return "application_port_lost", None, True

        snapshot = self._active_watchdog.poll(
            self._dependencies.monotonic()
        )
        self._verified_consecutive_frames = (
            snapshot.consecutive_good_frames
        )
        if snapshot.fault_reason is not None:
            return snapshot.fault_reason, None, False
        return None

    def _matching_port_inventory(
        self,
    ) -> tuple[bool, RadarPortIdentity | None]:
        inventory = tuple(self._dependencies.port_provider())
        target_serial = self._latched_xds_serial or self._config.xds_serial
        candidates = []
        for port in inventory:
            device_value = getattr(port, "device", "")
            description_value = getattr(port, "description", "")
            serial_value = getattr(port, "serial_number", "")
            location_value = getattr(port, "location", "")
            device = "" if device_value is None else str(device_value).strip()
            description = (
                ""
                if description_value is None
                else str(description_value).strip()
            )
            serial_number = (
                "" if serial_value is None else str(serial_value).strip()
            )
            location = (
                ""
                if location_value is None
                else str(location_value).strip()
            )
            if self._config.explicit_port is not None:
                if device != self._config.explicit_port:
                    continue
            if (getattr(port, "vid", None), getattr(port, "pid", None)) != (
                0x0451,
                0xBEF3,
            ):
                continue
            if "Application/User UART" not in description:
                continue
            if "Auxiliary" in description:
                continue
            if location.casefold().endswith(".3") or not serial_number:
                continue
            if target_serial is not None and serial_number != target_serial:
                continue
            candidates.append(port)
        if len(candidates) != 1:
            return bool(candidates), None
        selected = select_application_port(
            candidates,
            explicit_port=self._config.explicit_port,
            xds_serial=target_serial,
        )
        if self._latched_xds_serial is None:
            self._latched_xds_serial = selected.serial_number
        return True, selected

    def _wait_for_port(
        self,
        stop_requested: Callable[[], bool],
    ) -> RadarPortIdentity | None:
        while True:
            if stop_requested():
                return None
            _, selected = self._matching_port_inventory()
            if selected is not None:
                return selected
            if stop_requested():
                return None
            self._dependencies.sleep(self._config.poll_interval_s)

    def _wait_for_usb_cycle(
        self,
        stop_requested: Callable[[], bool],
    ) -> RadarPortIdentity | None:
        self._transition(
            SupervisorState.RECOVERING,
            "reset_tool_unavailable_waiting_for_usb_cycle",
        )
        absence_seen = False
        while True:
            if stop_requested():
                return None
            present, selected = self._matching_port_inventory()
            if not present:
                absence_seen = True
            elif absence_seen and selected is not None:
                return selected
            if stop_requested():
                return None
            self._dependencies.sleep(self._config.poll_interval_s)

    def _backoff(self, stop_requested: Callable[[], bool]) -> bool:
        if stop_requested():
            return False
        delay_s = self._retry_delay_s
        self._dependencies.sleep(delay_s)
        self._retry_delay_s = min(
            self._config.retry_max_s,
            delay_s * 2.0,
        )
        return not stop_requested()

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
            if (
                snapshot.verified is True
                and snapshot.consecutive_good_frames
                >= self._config.verification_frames
            ):
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

    def _stop_active_child(self, child: SupervisorChild) -> None:
        registered = self._owned_children.get(child.pid)
        if registered is not child or registered.role != child.role:
            raise RuntimeError("requested child is not the registered active child")
        result = self._dependencies.processes.stop_child(child)
        if result.pid != child.pid or result.role != child.role:
            raise RuntimeError(
                "stop_child result does not match registered active child"
            )
        if self._active_capture is child:
            self._set_capture_exit_code(result.exit_code)
        self._record_stop_result(child, result)
        del self._owned_children[child.pid]
        if self._active_capture is child:
            self._active_capture = None
        if self._active_viewer is child:
            self._active_viewer = None

    def _record_stop_result(
        self,
        child: SupervisorChild,
        result: SupervisorStopResult,
    ) -> None:
        if child.pid != result.pid or child.role != result.role:
            raise RuntimeError(
                "cleanup result does not match registered active child"
            )
        self._process_events.append(
            {
                "role": result.role,
                "pid": result.pid,
                "action": "stopped",
                "escalation": result.escalation,
                "exit_code": result.exit_code,
            }
        )
        self._persist_manifest()

    def _stop_registered_children(self) -> None:
        if not self._process_manager_engaged:
            return
        last_error: BaseException | None = None
        validation_errors: list[RuntimeError] = []
        try:
            for _attempt in range(2):
                results: tuple[SupervisorStopResult, ...]
                try:
                    results = (
                        self._dependencies.processes.stop_owned_children()
                    )
                    last_error = None
                except BaseException as error:
                    last_error = error
                    partial_results = getattr(error, "results", ())
                    results = (
                        tuple(partial_results)
                        if isinstance(partial_results, (list, tuple))
                        else ()
                    )

                for result in results:
                    child = self._owned_children.get(result.pid)
                    if child is None or child.role != result.role:
                        validation_errors.append(
                            RuntimeError(
                                "cleanup result names an unknown or mismatched "
                                f"owned child: {result.role!r}/{result.pid!r}"
                            )
                        )
                        continue
                    if self._active_capture is child:
                        self._set_capture_exit_code(result.exit_code)
                    self._record_stop_result(child, result)
                    del self._owned_children[result.pid]
                    if self._active_capture is child:
                        self._active_capture = None
                    if self._active_viewer is child:
                        self._active_viewer = None

                if validation_errors or last_error is None:
                    break
        finally:
            self._process_manager_engaged = False
            self._active_watchdog = None

        if validation_errors:
            raise validation_errors[0]
        if last_error is not None:
            raise last_error
        if self._owned_children:
            remaining = sorted(
                (child.role, child.pid)
                for child in self._owned_children.values()
            )
            raise RuntimeError(
                f"owned-child cleanup omitted active children: {remaining!r}"
            )

    def _set_capture_exit_code(self, exit_code: int) -> None:
        if (
            self._epochs
            and self._epochs[-1]["ended_at"] is None
            and self._epochs[-1]["capture_exit_code"] is None
        ):
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
        self._persist_manifest()

    def _shutdown(self) -> None:
        cleanup_error: BaseException | None = None
        try:
            self._stop_registered_children()
        except BaseException as error:
            cleanup_error = error
        self._finalize_epoch(reason="shutdown")
        self._restore_verified_paths()
        self._transition(SupervisorState.STOPPED, "shutdown")
        if cleanup_error is not None:
            raise cleanup_error

    def _restore_verified_paths(self) -> None:
        self._paths = self._verified_paths

    def _transition(self, state: SupervisorState, reason: str) -> None:
        self._state = state
        self._last_reason = reason
        self._persist_manifest()

    def _persist_manifest(self) -> None:
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
                self._latched_xds_serial
                or (
                    self._port.serial_number
                    if self._port is not None
                    else self._config.xds_serial
                )
            ),
            "reset_capability": {
                "available": self._config.reset_executable is not None,
                "reason": (
                    None
                    if self._config.reset_executable is not None
                    else self._config.reset_unavailable_reason
                ),
            },
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
    "reserve_manifest",
    "write_manifest_atomic",
]
