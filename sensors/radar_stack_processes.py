"""Owned child-process management for the radar capture and viewer stack."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import sys
from typing import Callable, Literal

from sensors.radar_supervisor import EpochPaths, RadarSupervisorConfig
from sensors.ti_radar_control import RadarPortIdentity


@dataclass(frozen=True)
class ChildStopResult:
    role: str
    pid: int
    exit_code: int
    escalation: Literal["already_exited", "graceful", "terminate", "kill"]


@dataclass
class ManagedChild:
    role: str
    process: subprocess.Popen
    owned: bool = True

    @property
    def pid(self) -> int:
        return self.process.pid

    def poll(self) -> int | None:
        return self.process.poll()

    def stop(self, grace_s: float = 2.0) -> ChildStopResult:
        if not self.owned:
            raise RuntimeError("refusing to signal an unowned process")
        exit_code = self.process.poll()
        if exit_code is not None:
            return ChildStopResult(
                self.role,
                self.process.pid,
                exit_code,
                "already_exited",
            )
        if os.name == "nt":
            self.process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        escalation: Literal["graceful", "terminate", "kill"] = "graceful"
        try:
            exit_code = self.process.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            escalation = "terminate"
            self.process.terminate()
            try:
                exit_code = self.process.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                escalation = "kill"
                self.process.kill()
                exit_code = self.process.wait(timeout=grace_s)
        return ChildStopResult(self.role, self.process.pid, exit_code, escalation)


@dataclass(frozen=True)
class _RegisteredChild:
    child: ManagedChild
    process: subprocess.Popen
    role: str
    pid: int


@dataclass(frozen=True)
class _ChildStopFailure:
    role: str
    pid: int
    error: BaseException


class _OwnedChildrenStopError(RuntimeError):
    """Reports failures after all owned children have been given a stop attempt."""

    def __init__(
        self,
        failures: tuple[_ChildStopFailure, ...],
        results: tuple[ChildStopResult, ...],
    ) -> None:
        super().__init__(f"failed to stop {len(failures)} owned radar child process(es)")
        self.failures = failures
        self.results = results


def build_capture_command(
    port: RadarPortIdentity,
    paths: EpochPaths,
    config: RadarSupervisorConfig,
) -> list[str]:
    return [
        sys.executable,
        "-m",
        "sensors",
        "radar-live",
        "--port",
        port.device,
        "--baud",
        str(config.data_baud),
        "--allow-elided-empty-point-tlv",
        "--allow-nonzero-padding",
        "--heatmap-azimuth-bins",
        str(config.heatmap_azimuth_bins),
        "--heatmap-range-bins",
        str(config.heatmap_range_bins),
        "--heatmap-range-step-m",
        str(config.heatmap_range_step_m),
        "--output",
        str(paths.mission),
        "--raw-output",
        str(paths.raw),
        "--raw-index",
        str(paths.raw_index),
        "--mission-id",
        config.mission_id,
        "--profile-id",
        config.profile_id,
        "--calibration-id",
        "uncalibrated",
    ]


def build_viewer_command(
    paths: EpochPaths,
    config: RadarSupervisorConfig,
) -> list[str]:
    return [
        sys.executable,
        "monitor/radar_front.py",
        "--follow",
        str(paths.mission),
        "--clutter-calibration",
        str(config.calibration_path),
        "--bind",
        config.http_bind,
        "--http-port",
        str(config.http_port),
        "--max-range-m",
        f"{config.viewer_max_range_m:g}",
        "--history-window",
        str(config.viewer_history_s),
        "--quiet",
    ]


class RadarStackProcesses:
    """Start only radar-stack children and stop only those children again."""

    def __init__(
        self,
        popen_factory: Callable[..., subprocess.Popen] = subprocess.Popen,
    ) -> None:
        self._popen_factory = popen_factory
        self._owned_by_identity: dict[int, _RegisteredChild] = {}
        self._owned_by_pid: dict[int, _RegisteredChild] = {}

    def _register(self, child: ManagedChild) -> ManagedChild:
        if not child.owned:
            raise RuntimeError("refusing to register an unowned process")
        process = child.process
        pid = process.pid
        role = child.role
        if pid in self._owned_by_pid:
            raise RuntimeError(f"duplicate owned process pid: {pid}")
        registered = _RegisteredChild(child, process, role, pid)
        self._owned_by_identity[id(child)] = registered
        self._owned_by_pid[pid] = registered
        return child

    def _start(
        self,
        role: str,
        command: list[str],
        stdout_path: Path,
        stderr_path: Path,
        config: RadarSupervisorConfig,
    ) -> ManagedChild:
        stdout_path.parent.mkdir(parents=True, exist_ok=True)
        stderr_path.parent.mkdir(parents=True, exist_ok=True)
        popen_kwargs: dict[str, object] = {
            "cwd": config.repository_root,
        }
        if os.name == "nt":
            startupinfo = subprocess.STARTUPINFO()
            startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startupinfo.wShowWindow = subprocess.SW_HIDE
            popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
            popen_kwargs["startupinfo"] = startupinfo
        else:
            popen_kwargs["start_new_session"] = True
        with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open(
            "w", encoding="utf-8"
        ) as stderr:
            process = self._popen_factory(
                command,
                stdout=stdout,
                stderr=stderr,
                **popen_kwargs,
            )
        child = ManagedChild(role, process)
        try:
            return self._register(child)
        except BaseException as registration_error:
            try:
                child.stop()
            except BaseException as cleanup_error:
                registration_error.add_note(
                    "post-registration cleanup failed: " f"{cleanup_error!r}"
                )
            raise

    def start_capture(
        self,
        port: RadarPortIdentity,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> ManagedChild:
        return self._start(
            "capture",
            build_capture_command(port, paths, config),
            paths.capture_stdout,
            paths.capture_stderr,
            config,
        )

    def switch_viewer(
        self,
        current: ManagedChild | None,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> ManagedChild:
        if current is not None:
            self.stop_child(current)
        return self._start(
            "viewer",
            build_viewer_command(paths, config),
            paths.viewer_stdout,
            paths.viewer_stderr,
            config,
        )

    def stop_child(self, child: ManagedChild) -> ChildStopResult:
        registered = self._owned_by_identity.get(id(child))
        if (
            not child.owned
            or registered is None
            or registered.child is not child
            or self._owned_by_pid.get(registered.pid) is not registered
            or child.role != registered.role
            or child.process is not registered.process
            or child.pid != registered.pid
            or registered.process.pid != registered.pid
        ):
            raise RuntimeError("refusing to signal an unregistered child")
        result = ManagedChild(registered.role, registered.process).stop()
        del self._owned_by_identity[id(child)]
        del self._owned_by_pid[registered.pid]
        return result

    def stop_owned_children(self) -> tuple[ChildStopResult, ...]:
        results: list[ChildStopResult] = []
        failures: list[_ChildStopFailure] = []
        for registered in tuple(self._owned_by_pid.values()):
            try:
                results.append(self.stop_child(registered.child))
            except BaseException as error:
                failures.append(
                    _ChildStopFailure(registered.role, registered.pid, error)
                )
        if failures:
            raise _OwnedChildrenStopError(tuple(failures), tuple(results))
        return tuple(results)


__all__ = [
    "ChildStopResult",
    "ManagedChild",
    "RadarStackProcesses",
    "build_capture_command",
    "build_viewer_command",
]
