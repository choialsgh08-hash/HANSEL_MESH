from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

from common.sensor_contract import (
    RadarFrame,
    RadarHeatmap,
    SensorHeader,
    SensorHealth,
)
from sensors.mission_log import decode_log_entry, encode_log_entry
from sensors.radar_stack_processes import ManagedChild, RadarStackProcesses
from sensors.radar_supervisor import (
    EpochPaths,
    RadarSupervisor,
    RadarSupervisorConfig,
    RadarSupervisorDependencies,
    manifest_path,
)
from sensors.radar_watchdog import (
    ExpectedRadarEvidence,
    RadarEpochWatchdog,
    RadarWatchdogSnapshot,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]

_PROCESS_HELPER = r"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time


running = True


def request_stop(signum, frame):
    del signum, frame
    global running
    running = False


def append_event(path, **payload):
    payload["time_ns"] = time.monotonic_ns()
    data = (
        json.dumps(payload, sort_keys=True, separators=(",", ":"))
        + "\n"
    ).encode("utf-8")
    descriptor = os.open(
        path,
        os.O_APPEND | os.O_CREAT | os.O_WRONLY,
        0o600,
    )
    try:
        os.write(descriptor, data)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


parser = argparse.ArgumentParser()
parser.add_argument("--role", choices=("capture", "viewer", "outside"), required=True)
parser.add_argument("--epoch", required=True)
parser.add_argument("--events", required=True)
parser.add_argument("--mission", required=True)
parser.add_argument("--raw")
parser.add_argument("--raw-index")
parser.add_argument("--frames")
parser.add_argument("--health")
parser.add_argument("--frame-delay", type=float, default=0.04)
parser.add_argument("--health-delay", type=float, default=0.02)
parser.add_argument("--parent-lease")
parser.add_argument("--repository-root")
args = parser.parse_args()

parent_watcher = None
if args.parent_lease is not None:
    if args.repository_root is None:
        raise RuntimeError("--repository-root is required with --parent-lease")
    sys.path.insert(0, args.repository_root)
    from sensors.radar_parent_lease import start_parent_death_watcher

    parent_watcher = start_parent_death_watcher(Path(args.parent_lease))
    if not parent_watcher.ready.wait(5.0):
        raise RuntimeError("parent-death watcher did not become ready")


def should_run():
    return (
        running
        and (
            parent_watcher is None
            or not parent_watcher.stop_requested.is_set()
        )
    )


for handled_signal in (signal.SIGINT, signal.SIGTERM):
    signal.signal(handled_signal, request_stop)
if hasattr(signal, "SIGBREAK"):
    signal.signal(signal.SIGBREAK, request_stop)

pid = os.getpid()
append_event(
    args.events,
    action="started",
    role=args.role,
    epoch=args.epoch,
    pid=pid,
    mission=args.mission,
)
try:
    if args.role == "capture":
        mission = Path(args.mission)
        raw = Path(args.raw)
        raw_index = Path(args.raw_index)
        for path in (mission, raw, raw_index):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.open("xb").close()
        frame_lines = Path(args.frames).read_bytes().splitlines(keepends=True)
        health_line = Path(args.health).read_bytes()
        frames_written = 0
        with (
            mission.open("ab", buffering=0) as mission_handle,
            raw.open("ab", buffering=0) as raw_handle,
            raw_index.open("ab", buffering=0) as index_handle,
        ):
            for frame_number, line in enumerate(frame_lines, 1):
                if not should_run():
                    break
                mission_handle.write(line)
                raw_bytes = (
                    f"{args.epoch}:raw-frame:{frame_number}\n"
                ).encode("ascii")
                raw_offset = raw_handle.tell()
                raw_handle.write(raw_bytes)
                index_handle.write(
                    (
                        json.dumps(
                            {
                                "epoch": args.epoch,
                                "frame": frame_number,
                                "offset": raw_offset,
                                "size": len(raw_bytes),
                            },
                            sort_keys=True,
                            separators=(",", ":"),
                        )
                        + "\n"
                    ).encode("utf-8")
                )
                for handle in (mission_handle, raw_handle, index_handle):
                    os.fsync(handle.fileno())
                frames_written += 1
                append_event(
                    args.events,
                    action="frame_written",
                    role=args.role,
                    epoch=args.epoch,
                    pid=pid,
                    frame=frame_number,
                )
                time.sleep(args.frame_delay)
            append_event(
                args.events,
                action="frames_complete",
                role=args.role,
                epoch=args.epoch,
                pid=pid,
                count=frames_written,
            )
            health_count = 0
            while should_run():
                mission_handle.write(health_line)
                os.fsync(mission_handle.fileno())
                health_count += 1
                if health_count <= 3:
                    append_event(
                        args.events,
                        action="health_written",
                        role=args.role,
                        epoch=args.epoch,
                        pid=pid,
                        count=health_count,
                    )
                time.sleep(args.health_delay)
    else:
        while should_run():
            time.sleep(0.02)
finally:
    append_event(
        args.events,
        action="stopped",
        role=args.role,
        epoch=args.epoch,
        pid=pid,
        exit_code=0,
    )
"""


def _application_port(device: str) -> object:
    return SimpleNamespace(
        device=device,
        vid=0x0451,
        pid=0xBEF3,
        serial_number="RI32",
        description="XDS110 Class Application/User UART",
        location="1-2",
    )


def _argument(command: list[str], name: str) -> str:
    return command[command.index(name) + 1]


def _mission_records(path: Path) -> list[object]:
    if not path.exists():
        return []
    return [
        decode_log_entry(line, line_number=index).record
        for index, line in enumerate(path.read_bytes().splitlines(), 1)
    ]


class _HelperPopenRouter:
    def __init__(
        self,
        *,
        helper: Path,
        events: Path,
        frame_sources: dict[str, Path],
        health_sources: dict[str, Path],
    ) -> None:
        self.helper = helper
        self.events = events
        self.frame_sources = frame_sources
        self.health_sources = health_sources
        self.processes: dict[int, subprocess.Popen] = {}
        self.roles: dict[int, str] = {}
        self.epochs: dict[int, str] = {}
        self.capture_ports: list[str] = []
        self.capture_missions: list[Path] = []
        self.viewer_missions: list[Path] = []
        self.viewer_frame_counts: list[int] = []

    def __call__(
        self,
        command: list[str],
        **kwargs: object,
    ) -> subprocess.Popen:
        if "radar-live" in command:
            role = "capture"
            mission = Path(_argument(command, "--output"))
            raw = Path(_argument(command, "--raw-output"))
            raw_index = Path(_argument(command, "--raw-index"))
            epoch = mission.stem[-4:]
            self.capture_ports.append(_argument(command, "--port"))
            self.capture_missions.append(mission)
            event_file = self.events / f"{role}-{epoch}.jsonl"
            helper_command = [
                sys.executable,
                str(self.helper),
                "--role",
                role,
                "--epoch",
                epoch,
                "--events",
                str(event_file),
                "--mission",
                str(mission),
                "--raw",
                str(raw),
                "--raw-index",
                str(raw_index),
                "--frames",
                str(self.frame_sources[epoch]),
                "--health",
                str(self.health_sources[epoch]),
            ]
        else:
            role = "viewer"
            mission = Path(_argument(command, "--follow"))
            epoch = mission.stem[-4:]
            self.viewer_missions.append(mission)
            self.viewer_frame_counts.append(
                sum(
                    isinstance(record, RadarFrame)
                    for record in _mission_records(mission)
                )
            )
            event_file = self.events / f"{role}-{epoch}.jsonl"
            helper_command = [
                sys.executable,
                str(self.helper),
                "--role",
                role,
                "--epoch",
                epoch,
                "--events",
                str(event_file),
                "--mission",
                str(mission),
            ]
        helper_command.extend(
            [
                "--parent-lease",
                _argument(command, "--supervisor-parent-lease"),
                "--repository-root",
                str(REPOSITORY_ROOT),
            ]
        )
        process = subprocess.Popen(helper_command, **kwargs)
        self.processes[process.pid] = process
        self.roles[process.pid] = role
        self.epochs[process.pid] = epoch
        return process


class _TrackedRadarStackProcesses(RadarStackProcesses):
    def __init__(self, router: _HelperPopenRouter) -> None:
        super().__init__(popen_factory=router)
        self.router = router
        self.stop_requests: list[int] = []
        self.alive_before_stop: dict[int, bool] = {}
        self.viewer_stop_observations: dict[int, tuple[str, int]] = {}

    def stop_child(self, child: ManagedChild):
        self.stop_requests.append(child.pid)
        self.alive_before_stop[child.pid] = child.poll() is None
        if child.role == "viewer" and self.router.capture_missions:
            latest_mission = self.router.capture_missions[-1]
            self.viewer_stop_observations[child.pid] = (
                latest_mission.stem[-4:],
                sum(
                    isinstance(record, RadarFrame)
                    for record in _mission_records(latest_mission)
                ),
            )
        return super().stop_child(child)


class _PortSequence:
    def __init__(self) -> None:
        self.inventory_calls: list[list[str]] = []
        self.reset_ports: list[str] = []
        self.configure_ports: list[str] = []
        self._initial_absence_pending = True
        self._recovery_reset_seen = False
        self._recovery_absence_pending = True

    def inventory(self) -> list[object]:
        if self._initial_absence_pending:
            self._initial_absence_pending = False
            result: list[object] = []
        elif not self._recovery_reset_seen:
            result = [_application_port("COM3")]
        elif self._recovery_absence_pending:
            self._recovery_absence_pending = False
            result = []
        else:
            result = [_application_port("COM7")]
        self.inventory_calls.append(
            [str(getattr(port, "device")) for port in result]
        )
        return result

    def reset(
        self,
        port: object,
        config: RadarSupervisorConfig,
    ) -> bool:
        del config
        device = str(getattr(port, "device"))
        self.reset_ports.append(device)
        if len(self.reset_ports) == 2:
            self._recovery_reset_seen = True
        return False

    def configure(
        self,
        port: object,
        config: RadarSupervisorConfig,
    ) -> dict[str, object]:
        del config
        self.configure_ports.append(str(getattr(port, "device")))
        return {
            "commands_completed": 2,
            "new_baud_prompt_observed": True,
            "first_magic_observed": True,
        }


class _RecordingWatchdog:
    def __init__(
        self,
        watchdog: RadarEpochWatchdog,
        history: list[tuple[float, RadarWatchdogSnapshot]],
        started_at_s: float,
    ) -> None:
        self.watchdog = watchdog
        self.history = history
        self.started_at_s = started_at_s

    def poll(self, now_s: float):
        snapshot = self.watchdog.poll(now_s)
        self.history.append((now_s - self.started_at_s, snapshot))
        return snapshot


def _write_epoch_records(
    directory: Path,
    config: RadarSupervisorConfig,
    epoch: str,
) -> tuple[Path, Path]:
    frame_source = directory / f"{epoch}-frames.jsonl"
    health_source = directory / f"{epoch}-health.jsonl"
    heatmap = RadarHeatmap(
        data=bytes([37]) * (
            config.heatmap_azimuth_bins * config.heatmap_range_bins
        ),
        range_bins=config.heatmap_range_bins,
        azimuth_bins=config.heatmap_azimuth_bins,
        range_step_m=config.heatmap_range_step_m,
        tlv_type=304,
        motion_mode="major",
        floor_db=-90.0,
        ceiling_db=10.0,
    )
    producer_id = f"capture-{epoch}"
    frame_bytes = []
    for frame_number in range(1, 6):
        frame = RadarFrame(
            header=SensorHeader(
                mission_id=config.mission_id,
                unit_id="radar-board",
                boot_id=f"boot-{epoch}",
                producer_id=producer_id,
                stream_id="radar/front",
                seq=frame_number,
                monotonic_ns=frame_number * 100_000_000,
            ),
            frame_number=frame_number,
            subframe_number=0,
            complete=True,
            dropped_frames_since_previous=0,
            points=(),
            frame_transition=(
                "first" if frame_number == 1 else "consecutive"
            ),
            profile_id=config.profile_id,
            heatmap=heatmap,
        )
        frame_bytes.append(encode_log_entry(frame_number, frame) + b"\n")
    frame_source.write_bytes(b"".join(frame_bytes))
    health = SensorHealth(
        header=SensorHeader(
            mission_id=config.mission_id,
            unit_id="radar-board",
            boot_id=f"boot-{epoch}",
            producer_id=producer_id,
            stream_id="health/radar",
            seq=1,
            monotonic_ns=600_000_000,
        ),
        subject_stream_id="radar/front",
        status="ok",
        observed_rate_hz=10.0,
        last_sample_monotonic_ns=500_000_000,
    )
    health_source.write_bytes(encode_log_entry(6, health) + b"\n")
    return frame_source, health_source


def _events(path: Path) -> list[dict[str, object]]:
    if not path.exists():
        return []
    event_files = sorted(path.glob("*.jsonl")) if path.is_dir() else [path]
    merged: list[tuple[int, str, int, dict[str, object]]] = []
    for event_file in event_files:
        raw = event_file.read_bytes()
        complete_lines = raw.split(b"\n")
        if complete_lines[-1]:
            complete_lines.pop()
        else:
            complete_lines.pop()
        for line_number, line in enumerate(complete_lines, 1):
            event = json.loads(line)
            merged.append(
                (
                    int(event["time_ns"]),
                    event_file.name,
                    line_number,
                    event,
                )
            )
    merged.sort(key=lambda row: row[:3])
    return [event for _, _, _, event in merged]


def _watchdog_transitions(
    histories: list[list[tuple[float, RadarWatchdogSnapshot]]],
) -> list[list[tuple[float, bool, int, str | None]]]:
    result = []
    for history in histories:
        transitions = []
        previous: tuple[bool, int, str | None] | None = None
        for elapsed_s, snapshot in history:
            current = (
                snapshot.verified,
                snapshot.consecutive_good_frames,
                snapshot.fault_reason,
            )
            if current != previous:
                transitions.append((round(elapsed_s, 3), *current))
                previous = current
        result.append(transitions)
    return result


def _stop_unowned_helper(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    if os.name == "nt":
        process.send_signal(signal.CTRL_BREAK_EVENT)
    else:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


class RadarSupervisorSubprocessIntegrationTests(unittest.TestCase):
    def test_real_owned_children_recover_across_immutable_epochs(self) -> None:
        with tempfile.TemporaryDirectory() as directory_name:
            directory = Path(directory_name)
            helper = directory / "radar_process_helper.py"
            helper.write_text(_PROCESS_HELPER, encoding="utf-8")
            events_path = directory / "process-events"
            events_path.mkdir()
            profile = directory / "profile.cfg"
            profile.write_text("sensorStop\nsensorStart\n", encoding="utf-8")
            config = RadarSupervisorConfig(
                repository_root=REPOSITORY_ROOT,
                output_root=directory / "output",
                profile_path=profile,
                calibration_path=directory / "unused-calibration.json",
                run_id="subprocess-recovery",
                xds_serial="RI32",
                reset_unavailable_reason="integration reset is disabled",
                first_frame_timeout_s=1.5,
                frame_timeout_s=1.0,
                verification_timeout_s=2.0,
                verification_frames=5,
                retry_initial_s=0.01,
                retry_max_s=0.02,
                poll_interval_s=0.01,
                http_port=18_081,
            )
            epoch_sources = {
                epoch: _write_epoch_records(directory, config, epoch)
                for epoch in ("e001", "e002")
            }
            router = _HelperPopenRouter(
                helper=helper,
                events=events_path,
                frame_sources={
                    epoch: sources[0]
                    for epoch, sources in epoch_sources.items()
                },
                health_sources={
                    epoch: sources[1]
                    for epoch, sources in epoch_sources.items()
                },
            )
            processes = _TrackedRadarStackProcesses(router)
            ports = _PortSequence()
            watchdog_histories: list[
                list[tuple[float, RadarWatchdogSnapshot]]
            ] = []

            def watchdog_factory(
                paths: EpochPaths,
                settings: RadarSupervisorConfig,
                started_at: float,
            ) -> _RecordingWatchdog:
                history: list[tuple[float, RadarWatchdogSnapshot]] = []
                watchdog_histories.append(history)
                return _RecordingWatchdog(
                    RadarEpochWatchdog(
                        mission_path=paths.mission,
                        raw_path=paths.raw,
                        expected=ExpectedRadarEvidence(
                            profile_id=settings.profile_id,
                            heatmap_azimuth_bins=(
                                settings.heatmap_azimuth_bins
                            ),
                            heatmap_range_bins=settings.heatmap_range_bins,
                            heatmap_range_step_m=(
                                settings.heatmap_range_step_m
                            ),
                        ),
                        started_at_s=started_at,
                        first_frame_timeout_s=(
                            settings.first_frame_timeout_s
                        ),
                        frame_timeout_s=settings.frame_timeout_s,
                        required_consecutive_frames=(
                            settings.verification_frames
                        ),
                        verification_timeout_s=(
                            settings.verification_timeout_s
                        ),
                    ),
                    history,
                    started_at,
                )

            dependencies = RadarSupervisorDependencies(
                port_provider=ports.inventory,
                reset_target=ports.reset,
                configure=ports.configure,
                processes=processes,
                watchdog_factory=watchdog_factory,
                monotonic=time.monotonic,
                sleep=time.sleep,
                utc_now=lambda: datetime.now(timezone.utc),
            )
            supervisor = RadarSupervisor(config, dependencies)
            manifest = manifest_path(config.output_root, config.run_id)
            deadline = time.monotonic() + 8.0
            timed_out = False
            running_e002: dict[str, object] | None = None
            e001_final_bytes: tuple[bytes, bytes, bytes] | None = None
            outside: subprocess.Popen | None = None

            outside_kwargs: dict[str, object] = {}
            if os.name == "nt":
                startupinfo = subprocess.STARTUPINFO()
                startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                startupinfo.wShowWindow = subprocess.SW_HIDE
                outside_kwargs["creationflags"] = (
                    subprocess.CREATE_NEW_PROCESS_GROUP
                )
                outside_kwargs["startupinfo"] = startupinfo
            else:
                outside_kwargs["start_new_session"] = True
            outside = subprocess.Popen(
                [
                    sys.executable,
                    str(helper),
                    "--role",
                    "outside",
                    "--epoch",
                    "outside",
                    "--events",
                    str(events_path / "outside.jsonl"),
                    "--mission",
                    str(directory / "outside-unused.jsonl"),
                ],
                **outside_kwargs,
            )

            def stop_after_second_epoch_is_running() -> bool:
                nonlocal timed_out, running_e002, e001_final_bytes
                if time.monotonic() >= deadline:
                    timed_out = True
                    return True
                if not manifest.exists():
                    return False
                payload = json.loads(manifest.read_text(encoding="utf-8"))
                if payload["state"] != "RUNNING" or payload["epoch"] != 2:
                    return False
                if not any(
                    event["role"] == "viewer"
                    and event["epoch"] == "e002"
                    and event["action"] == "started"
                    for event in _events(events_path)
                ):
                    return False
                running_e002 = payload
                e001 = payload["epochs"][0]
                e001_final_bytes = tuple(
                    Path(str(e001[name])).read_bytes()
                    for name in (
                        "mission_path",
                        "raw_path",
                        "raw_index_path",
                    )
                )
                return True

            try:
                supervisor.run(stop_after_second_epoch_is_running)

                diagnostics = {
                    "manifest": (
                        json.loads(manifest.read_text(encoding="utf-8"))
                        if manifest.exists()
                        else None
                    ),
                    "events": _events(events_path),
                    "inventories": ports.inventory_calls[:12],
                    "watchdog_transitions": _watchdog_transitions(
                        watchdog_histories
                    ),
                }
                self.assertFalse(
                    timed_out,
                    "supervisor did not reach RUNNING e002 before the "
                    f"bounded deadline: {diagnostics!r}",
                )
                self.assertIsNotNone(running_e002)
                self.assertIsNotNone(e001_final_bytes)
                assert running_e002 is not None
                assert e001_final_bytes is not None

                final_payload = json.loads(
                    manifest.read_text(encoding="utf-8")
                )
                self.assertEqual(running_e002["state"], "RUNNING")
                self.assertEqual(running_e002["epoch"], 2)
                self.assertEqual(
                    running_e002["recovery_count"],
                    1,
                    {
                        "running": running_e002,
                        "final": final_payload,
                        "events": _events(events_path),
                        "viewer_frame_counts": router.viewer_frame_counts,
                        "watchdog_transitions": _watchdog_transitions(
                            watchdog_histories
                        ),
                    },
                )
                self.assertEqual(
                    running_e002["verified_consecutive_frames"],
                    5,
                )
                self.assertIsNone(
                    running_e002["epochs"][1]["ended_at"]
                )
                self.assertEqual(final_payload["state"], "STOPPED")
                self.assertEqual(final_payload["recovery_count"], 1)
                self.assertEqual(
                    router.capture_ports,
                    ["COM3", "COM7"],
                )
                self.assertEqual(
                    ports.configure_ports,
                    ["COM3", "COM7"],
                )
                self.assertEqual(ports.reset_ports, ["COM3", "COM3"])
                self.assertEqual(final_payload["port"], "COM7")
                self.assertEqual(final_payload["xds_serial"], "RI32")
                self.assertEqual(ports.inventory_calls[0], [])
                self.assertIn([], ports.inventory_calls[1:])

                epochs = final_payload["epochs"]
                self.assertEqual([epoch["epoch"] for epoch in epochs], [1, 2])
                e001, e002 = epochs
                self.assertEqual(e001["end_reason"], "radar_frame_timeout")
                self.assertIsNotNone(e001["ended_at"])
                self.assertIsNotNone(e001["capture_exit_code"])
                self.assertEqual(
                    e001["capture_exit_code"],
                    0,
                    {
                        "events": _events(events_path),
                        "capture_stderr": (
                            config.output_root
                            / "runtime"
                            / f"radar-board-live-{config.run_id}"
                            / "e001-capture.stderr.log"
                        ).read_text(encoding="utf-8"),
                        "watchdog_transitions": _watchdog_transitions(
                            watchdog_histories
                        ),
                    },
                )
                self.assertNotEqual(
                    e001["mission_path"],
                    e002["mission_path"],
                )
                self.assertNotEqual(e001["raw_path"], e002["raw_path"])
                self.assertNotEqual(
                    e001["raw_index_path"],
                    e002["raw_index_path"],
                )
                self.assertEqual(
                    running_e002["mission_path"],
                    e002["mission_path"],
                )
                self.assertEqual(
                    running_e002["raw_path"],
                    e002["raw_path"],
                )
                self.assertEqual(
                    router.viewer_missions,
                    [
                        Path(str(e001["mission_path"])),
                        Path(str(e002["mission_path"])),
                    ],
                )
                self.assertEqual(router.viewer_frame_counts, [5, 5])
                self.assertEqual(len(watchdog_histories), 2)
                first_watchdog = [
                    snapshot for _, snapshot in watchdog_histories[0]
                ]
                second_watchdog = [
                    snapshot for _, snapshot in watchdog_histories[1]
                ]
                first_verified = next(
                    snapshot
                    for snapshot in first_watchdog
                    if snapshot.verified
                )
                self.assertEqual(
                    first_verified.consecutive_good_frames,
                    5,
                )
                self.assertEqual(
                    first_watchdog[-1].fault_reason,
                    "radar_frame_timeout",
                )
                self.assertEqual(
                    first_watchdog[-1].consecutive_good_frames,
                    5,
                )
                self.assertTrue(second_watchdog[-1].verified)
                self.assertEqual(
                    second_watchdog[-1].consecutive_good_frames,
                    5,
                )
                self.assertIsNone(second_watchdog[-1].fault_reason)

                e001_records = _mission_records(
                    Path(str(e001["mission_path"]))
                )
                self.assertEqual(
                    sum(
                        isinstance(record, RadarFrame)
                        for record in e001_records
                    ),
                    5,
                )
                self.assertGreaterEqual(
                    sum(
                        isinstance(record, SensorHealth)
                        for record in e001_records
                    ),
                    2,
                )
                self.assertEqual(
                    tuple(
                        Path(str(e001[name])).read_bytes()
                        for name in (
                            "mission_path",
                            "raw_path",
                            "raw_index_path",
                        )
                    ),
                    e001_final_bytes,
                )

                owned_pids = set(router.processes)
                self.assertEqual(set(processes.stop_requests), owned_pids)
                self.assertEqual(len(processes.stop_requests), 4)
                self.assertTrue(all(processes.alive_before_stop.values()))
                self.assertNotIn(outside.pid, processes.stop_requests)
                self.assertIsNone(outside.poll())
                self.assertTrue(
                    all(
                        process.poll() is not None
                        for process in router.processes.values()
                    )
                )

                viewer_pids = [
                    pid
                    for pid in router.processes
                    if router.roles[pid] == "viewer"
                ]
                old_viewer_pid, new_viewer_pid = viewer_pids
                self.assertEqual(
                    processes.viewer_stop_observations[old_viewer_pid],
                    ("e002", 5),
                )
                self.assertEqual(
                    router.viewer_frame_counts,
                    [5, 5],
                )
                self.assertEqual(
                    router.viewer_missions[-1],
                    Path(str(e002["mission_path"])),
                )

                process_events = final_payload["process_events"]
                for pid, process in router.processes.items():
                    matching = [
                        event
                        for event in process_events
                        if event["pid"] == pid
                    ]
                    self.assertEqual(
                        [event["action"] for event in matching],
                        ["started", "stopped"],
                    )
                    self.assertEqual(
                        [event["role"] for event in matching],
                        [router.roles[pid], router.roles[pid]],
                    )
                    self.assertIsNone(matching[0]["exit_code"])
                    self.assertEqual(
                        matching[1]["exit_code"],
                        process.returncode,
                    )
                    self.assertEqual(process.returncode, 0)
                self.assertEqual(
                    {
                        event["pid"]
                        for event in process_events
                        if event["action"] == "started"
                    },
                    owned_pids,
                )
                self.assertEqual(
                    {
                        event["pid"]
                        for event in process_events
                        if event["action"] == "stopped"
                    },
                    owned_pids,
                )
            finally:
                cleanup_failures: list[BaseException] = []
                try:
                    processes.stop_owned_children()
                except BaseException as error:
                    cleanup_failures.append(error)
                for process in router.processes.values():
                    try:
                        _stop_unowned_helper(process)
                    except BaseException as error:
                        cleanup_failures.append(error)
                try:
                    _stop_unowned_helper(outside)
                except BaseException as error:
                    cleanup_failures.append(error)
                if cleanup_failures:
                    raise ExceptionGroup(
                        "radar integration helper cleanup failed",
                        cleanup_failures,
                    )


if __name__ == "__main__":
    unittest.main()
