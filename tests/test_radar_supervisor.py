"""Contract tests for radar supervisor artifacts and configuration."""

from __future__ import annotations

from dataclasses import FrozenInstanceError, dataclass, fields, replace
from datetime import datetime, timezone
import inspect
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest import mock

from sensors.radar_watchdog import RadarWatchdogSnapshot
from sensors.radar_supervisor import (
    EpochPaths,
    RadarProcessManager,
    RadarSupervisor,
    RadarSupervisorConfig,
    RadarSupervisorDependencies,
    SupervisorChild,
    SupervisorStopResult,
    SupervisorState,
    SupervisorWatchdog,
    allocate_epoch_paths,
    manifest_path,
    write_manifest_atomic,
)


class RadarSupervisorContractTests(unittest.TestCase):
    def make_config(self, **changes: object) -> RadarSupervisorConfig:
        values: dict[str, object] = {
            "repository_root": Path("repository"),
            "output_root": Path("output"),
            "profile_path": Path("profile.cfg"),
            "calibration_path": Path("calibration.json"),
            "run_id": "20260729-010000",
        }
        values.update(changes)
        return RadarSupervisorConfig(**values)  # type: ignore[arg-type]

    def test_public_states_and_config_defaults_are_stable(self) -> None:
        self.assertEqual(
            [(state.name, state.value) for state in SupervisorState],
            [
                ("WAIT_PORT", "WAIT_PORT"),
                ("RESET_TARGET", "RESET_TARGET"),
                ("CONFIGURE", "CONFIGURE"),
                ("START_CAPTURE", "START_CAPTURE"),
                ("VERIFY_FRAMES", "VERIFY_FRAMES"),
                ("SWITCH_VIEWER", "SWITCH_VIEWER"),
                ("RUNNING", "RUNNING"),
                ("RECOVERING", "RECOVERING"),
                ("STOPPED", "STOPPED"),
            ],
        )
        config = self.make_config()
        self.assertEqual(
            {field.name for field in fields(EpochPaths)},
            {
                "mission",
                "raw",
                "raw_index",
                "runtime_dir",
                "capture_stdout",
                "capture_stderr",
                "viewer_stdout",
                "viewer_stderr",
            },
        )
        self.assertEqual(config.mission_id, "radar-board-live")
        self.assertEqual(
            config.profile_id,
            "lsdk-05.05.04.02-presence-near-"
            "heatmap16-elev8-cfar15-10hz-v1",
        )
        self.assertIsNone(config.explicit_port)
        self.assertIsNone(config.xds_serial)
        self.assertIsNone(config.reset_executable)
        self.assertEqual(config.initial_baud, 115_200)
        self.assertEqual(config.data_baud, 1_250_000)
        self.assertEqual(config.heatmap_azimuth_bins, 16)
        self.assertEqual(config.heatmap_range_bins, 128)
        self.assertEqual(config.heatmap_range_step_m, 0.09765625)
        self.assertEqual(config.first_frame_timeout_s, 3.0)
        self.assertEqual(config.frame_timeout_s, 2.5)
        self.assertEqual(config.verification_timeout_s, 3.0)
        self.assertEqual(config.verification_frames, 5)
        self.assertEqual(config.retry_initial_s, 0.5)
        self.assertEqual(config.retry_max_s, 5.0)
        self.assertEqual(config.poll_interval_s, 0.05)
        self.assertEqual(config.http_bind, "127.0.0.1")
        self.assertEqual(config.http_port, 8081)
        self.assertEqual(config.viewer_max_range_m, 3.0)
        self.assertEqual(config.viewer_history_s, 0.3)
        with self.assertRaises(FrozenInstanceError):
            config.run_id = "replacement"  # type: ignore[misc]
        paths = EpochPaths(*(Path(str(index)) for index in range(8)))
        with self.assertRaises(FrozenInstanceError):
            paths.mission = Path("replacement")  # type: ignore[misc]

    def test_config_rejects_invalid_numeric_and_identifier_values(self) -> None:
        integer_fields = (
            "initial_baud",
            "data_baud",
            "heatmap_azimuth_bins",
            "heatmap_range_bins",
            "verification_frames",
        )
        for field in integer_fields:
            for value in (0, -1, True, 1.0, "1"):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        self.make_config(**{field: value})

        positive_float_fields = (
            "heatmap_range_step_m",
            "first_frame_timeout_s",
            "frame_timeout_s",
            "verification_timeout_s",
            "retry_initial_s",
            "retry_max_s",
            "poll_interval_s",
            "viewer_max_range_m",
            "viewer_history_s",
        )
        for field in positive_float_fields:
            for value in (0, -0.1, True, math.nan, math.inf, -math.inf):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        self.make_config(**{field: value})
        with self.assertRaises(ValueError):
            self.make_config(first_frame_timeout_s=10**10_000)

        for value in (0, -1, True, 1.0, "1", 65_536):
            with self.subTest(http_port=value):
                with self.assertRaises(ValueError):
                    self.make_config(http_port=value)

        for field in ("run_id", "mission_id", "profile_id"):
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    self.make_config(**{field: "invalid identifier"})

        with self.assertRaises(ValueError):
            self.make_config(retry_initial_s=2.0, retry_max_s=1.0)
        with self.assertRaises(ValueError):
            self.make_config(http_bind="")

    def test_epoch_paths_are_exact_distinct_and_do_not_create_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "output"
            epoch_one = allocate_epoch_paths(root, "20260729-010000", 1)
            runtime = root / "runtime" / "radar-board-live-20260729-010000"
            self.assertEqual(
                epoch_one,
                EpochPaths(
                    mission=root / "missions" / "radar-board-live-20260729-010000-e001.jsonl",
                    raw=root / "captures" / "radar-board-live-20260729-010000-e001.bin",
                    raw_index=root / "captures" / "radar-board-live-20260729-010000-e001.bin.chunks.jsonl",
                    runtime_dir=runtime,
                    capture_stdout=runtime / "e001-capture.stdout.log",
                    capture_stderr=runtime / "e001-capture.stderr.log",
                    viewer_stdout=runtime / "e001-viewer.stdout.log",
                    viewer_stderr=runtime / "e001-viewer.stderr.log",
                ),
            )
            self.assertFalse(root.exists())
            epoch_two = allocate_epoch_paths(root, "20260729-010000", 2)
            self.assertNotEqual(epoch_one, epoch_two)
            self.assertTrue(all("e002" in str(path) for path in (
                epoch_two.mission,
                epoch_two.raw,
                epoch_two.raw_index,
                epoch_two.capture_stdout,
                epoch_two.capture_stderr,
                epoch_two.viewer_stdout,
                epoch_two.viewer_stderr,
            )))
            with self.assertRaises(ValueError):
                allocate_epoch_paths(root, "20260729-010000", 0)
            for invalid_epoch in (-1, True):
                with self.subTest(epoch=invalid_epoch):
                    with self.assertRaises(ValueError):
                        allocate_epoch_paths(root, "20260729-010000", invalid_epoch)

    def test_epoch_paths_reject_each_existing_epoch_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = allocate_epoch_paths(root, "20260729-010000", 1)
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
                with self.subTest(artifact=artifact.name):
                    artifact.parent.mkdir(parents=True, exist_ok=True)
                    artifact.write_text("occupied", encoding="utf-8")
                    with self.assertRaisesRegex(FileExistsError, "epoch artifact"):
                        allocate_epoch_paths(root, "20260729-010000", 1)
                    artifact.unlink()

    def test_manifest_path_is_exact_and_validates_run_id(self) -> None:
        root = Path("output")
        self.assertEqual(
            manifest_path(root, "20260729-010000"),
            root / "runtime" / "radar-supervisor-20260729-010000.json",
        )
        with self.assertRaises(ValueError):
            manifest_path(root, "invalid identifier")

    def test_atomic_manifest_is_readable_durable_and_cleans_temp(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime" / "manifest.json"
            original_fsync = os.fsync
            original_replace = os.replace
            events: list[str] = []
            temp_sources: list[Path] = []

            def fsync_spy(file_descriptor: int) -> None:
                events.append("fsync")
                original_fsync(file_descriptor)

            def replace_spy(source: object, destination: object) -> None:
                events.append("replace")
                temp_sources.append(Path(source))
                original_replace(source, destination)

            with mock.patch("sensors.radar_supervisor.os.fsync", fsync_spy), mock.patch(
                "sensors.radar_supervisor.os.replace", replace_spy
            ):
                write_manifest_atomic(path, {"z": 1, "a": {"greeting": "한글"}})

            content = path.read_text(encoding="utf-8")
            self.assertTrue(content.endswith("\n"))
            self.assertEqual(json.loads(content), {"a": {"greeting": "한글"}, "z": 1})
            self.assertLess(content.index('  "a": {'), content.index('  "z": 1'))
            self.assertEqual(events, ["fsync", "replace"])
            self.assertEqual(temp_sources[0].parent, path.parent)
            self.assertFalse(temp_sources[0].exists())
            self.assertEqual(list(path.parent.glob("*.tmp")), [])

    def test_atomic_manifest_failure_preserves_destination_and_cleans_temp(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime" / "manifest.json"
            path.parent.mkdir()
            path.write_text("old destination\n", encoding="utf-8")

            with self.assertRaises(TypeError):
                write_manifest_atomic(path, {"bad": {1, 2, 3}})
            self.assertEqual(path.read_text(encoding="utf-8"), "old destination\n")
            self.assertEqual(list(path.parent.glob("*.tmp")), [])

            with mock.patch(
                "sensors.radar_supervisor.os.fsync",
                side_effect=OSError("fsync failed"),
            ):
                with self.assertRaisesRegex(OSError, "fsync failed"):
                    write_manifest_atomic(path, {"new": "manifest"})
            self.assertEqual(path.read_text(encoding="utf-8"), "old destination\n")
            self.assertEqual(list(path.parent.glob("*.tmp")), [])

            with mock.patch(
                "sensors.radar_supervisor.os.replace",
                side_effect=OSError("replace failed"),
            ):
                with self.assertRaisesRegex(OSError, "replace failed"):
                    write_manifest_atomic(path, {"new": "manifest"})
            self.assertEqual(path.read_text(encoding="utf-8"), "old destination\n")
            self.assertEqual(list(path.parent.glob("*.tmp")), [])


@dataclass
class FakeChild:
    role: str
    pid: int
    exit_code: int | None = None

    def poll(self) -> int | None:
        return self.exit_code


@dataclass
class FakeStopResult:
    role: str
    pid: int
    exit_code: int
    escalation: str


def application_port(device: str, serial: str = "RI32") -> object:
    return SimpleNamespace(
        device=device,
        vid=0x0451,
        pid=0xBEF3,
        serial_number=serial,
        description="XDS110 Class Application/User UART",
        location="1-2",
    )


class FakeWatchdog:
    def __init__(
        self,
        snapshots: list[RadarWatchdogSnapshot],
        actions: list[str],
    ) -> None:
        self.snapshots = list(snapshots)
        self.actions = actions
        self.poll_count = 0

    def poll(self, now_s: float) -> RadarWatchdogSnapshot:
        del now_s
        self.poll_count += 1
        snapshot = self.snapshots.pop(0) if len(self.snapshots) > 1 else self.snapshots[0]
        if snapshot.verified:
            self.actions.append(f"verified:{snapshot.consecutive_good_frames}")
        return snapshot


class FakeProcesses:
    def __init__(self, actions: list[str]) -> None:
        self.actions = actions
        self.capture = FakeChild("capture", 41001)
        self.viewer = FakeChild("viewer", 41002)
        self.started_capture: FakeChild | None = None
        self.started_viewer: FakeChild | None = None
        self.capture_port: object | None = None
        self.viewer_watchdog_poll_count: int | None = None
        self.watchdog: FakeWatchdog | None = None
        self.stop_results: tuple[FakeStopResult, ...] = (
            FakeStopResult("capture", 41001, 0, "graceful"),
            FakeStopResult("viewer", 41002, 0, "graceful"),
        )
        self.stop_calls = 0
        self.switch_error: Exception | None = None

    def start_capture(
        self,
        port: object,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> FakeChild:
        del config
        self.capture_port = port
        self.actions.append(f"capture:{paths.mission.stem[-4:]}")
        self.started_capture = self.capture
        return self.capture

    def switch_viewer(
        self,
        current: FakeChild | None,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> FakeChild:
        del current, config
        self.actions.append(f"viewer:{paths.mission.stem[-4:]}")
        if self.switch_error is not None:
            raise self.switch_error
        self.started_viewer = self.viewer
        if self.watchdog is not None:
            self.viewer_watchdog_poll_count = self.watchdog.poll_count
        return self.viewer

    def stop_child(self, child: FakeChild) -> FakeStopResult:
        return FakeStopResult(child.role, child.pid, 0, "graceful")

    def stop_owned_children(self) -> tuple[FakeStopResult, ...]:
        self.stop_calls += 1
        return self.stop_results


class SupervisorFixture:
    def __init__(
        self,
        directory: str,
        *,
        ports: list[list[object]] | None = None,
        reset_result: bool = True,
        profile_result: dict[str, object] | None = None,
        snapshots: list[RadarWatchdogSnapshot] | None = None,
    ) -> None:
        self.root = Path(directory)
        self.actions: list[str] = []
        self.now_s = 100.0
        self.utc = datetime(2026, 7, 29, 1, 0, 0, tzinfo=timezone.utc)
        self.port_inventories = list(
            ports
            if ports is not None
            else [[application_port("COM3")], [application_port("COM3")]]
        )
        self.reset_result = reset_result
        self.profile_result = profile_result or {
            "commands_completed": 2,
            "new_baud_prompt_observed": True,
            "first_magic_observed": True,
        }
        self.processes = FakeProcesses(self.actions)
        self.watchdog = FakeWatchdog(
            snapshots
            or [
                RadarWatchdogSnapshot(False, count, 100.0, count, None)
                for count in range(1, 5)
            ]
            + [RadarWatchdogSnapshot(True, 5, 100.0, 5, None)],
            self.actions,
        )
        self.processes.watchdog = self.watchdog
        profile_path = self.root / "profile.cfg"
        profile_path.write_text("sensorStop\nsensorStart\n", encoding="utf-8")
        self.config = RadarSupervisorConfig(
            repository_root=self.root,
            output_root=self.root / "output",
            profile_path=profile_path,
            calibration_path=self.root / "calibration.json",
            run_id="board-live",
            xds_serial="RI32",
        )
        self.dependencies = RadarSupervisorDependencies(
            port_provider=self.port_provider,
            reset_target=self.reset_target,
            configure=self.configure,
            processes=self.processes,
            watchdog_factory=self.watchdog_factory,
            monotonic=self.monotonic,
            sleep=self.sleep,
            utc_now=self.utc_now,
        )

    def port_provider(self) -> list[object]:
        inventory = (
            self.port_inventories.pop(0)
            if len(self.port_inventories) > 1
            else self.port_inventories[0]
        )
        if len(inventory) == 1:
            self.actions.append(f"wait_port:{getattr(inventory[0], 'device')}")
        return inventory

    def reset_target(
        self,
        port: object,
        config: RadarSupervisorConfig,
    ) -> bool:
        del config
        self.actions.append(f"reset:{getattr(port, 'serial_number')}")
        return self.reset_result

    def configure(
        self,
        port: object,
        config: RadarSupervisorConfig,
    ) -> dict[str, object]:
        del config
        self.actions.append(f"configure:{getattr(port, 'device')}")
        return self.profile_result

    def watchdog_factory(
        self,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
        started_at_s: float,
    ) -> FakeWatchdog:
        del paths, config
        self.watchdog.started_at_s = started_at_s  # type: ignore[attr-defined]
        return self.watchdog

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, delay_s: float) -> None:
        self.now_s += delay_s

    def utc_now(self) -> datetime:
        return self.utc

    @property
    def manifest(self) -> Path:
        return manifest_path(self.config.output_root, self.config.run_id)

    def stop_when_running(self) -> bool:
        if not self.manifest.exists():
            return False
        payload = json.loads(self.manifest.read_text(encoding="utf-8"))
        if payload["state"] == "RUNNING":
            if not self.actions or self.actions[-1] != "running:e001":
                self.actions.append("running:e001")
            return True
        return False


def snapshot(
    *,
    verified: bool = True,
    frames: int = 5,
    fault: str | None = None,
) -> RadarWatchdogSnapshot:
    return RadarWatchdogSnapshot(
        verified,
        frames,
        100.0 if frames else None,
        frames if frames else None,
        fault,
    )


class RecoveryProcesses:
    def __init__(self, actions: list[str]) -> None:
        self.actions = actions
        self.children: list[FakeChild] = []
        self.stopped: set[int] = set()
        self.capture_ports: list[str] = []
        self.stop_override: FakeStopResult | None = None
        self.artifact_bytes: dict[Path, bytes] = {}

    def start_capture(
        self,
        port: object,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> FakeChild:
        del config
        child = FakeChild("capture", 51_000 + len(self.children) + 1)
        self.children.append(child)
        self.capture_ports.append(str(getattr(port, "device")))
        self.actions.append(f"capture:{paths.mission.stem[-4:]}")
        for path, content in (
            (paths.mission, f"mission-{child.pid}".encode()),
            (paths.raw, f"raw-{child.pid}".encode()),
        ):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
            self.artifact_bytes[path] = content
        return child

    def switch_viewer(
        self,
        current: FakeChild | None,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
    ) -> FakeChild:
        del config
        self.actions.append(
            f"viewer:{paths.mission.stem[-4:]}:"
            f"{'none' if current is None else current.pid}"
        )
        child = FakeChild("viewer", 52_000 + len(self.children) + 1)
        self.children.append(child)
        return child

    def stop_child(self, child: FakeChild) -> FakeStopResult:
        self.actions.append(f"stop:{child.role}:{child.pid}")
        if self.stop_override is not None:
            return self.stop_override
        self.stopped.add(child.pid)
        return FakeStopResult(child.role, child.pid, child.exit_code or 0, "graceful")

    def stop_owned_children(self) -> tuple[FakeStopResult, ...]:
        results = []
        for child in self.children:
            if child.pid in self.stopped:
                continue
            self.stopped.add(child.pid)
            results.append(
                FakeStopResult(
                    child.role,
                    child.pid,
                    child.exit_code or 0,
                    "graceful",
                )
            )
        return tuple(results)


class RecoveryFixture:
    def __init__(
        self,
        directory: str,
        *,
        ports: list[list[object]] | None = None,
        resets: list[bool | Exception] | None = None,
        configurations: list[dict[str, object] | Exception] | None = None,
        watchdogs: list[list[RadarWatchdogSnapshot]] | None = None,
    ) -> None:
        self.root = Path(directory)
        self.actions: list[str] = []
        self.sleeps: list[float] = []
        self.now_s = 100.0
        self.utc = datetime(2026, 7, 29, 1, 0, 0, tzinfo=timezone.utc)
        self.port_inventories = list(
            ports
            if ports is not None
            else [[application_port("COM3")]]
        )
        self.resets = list(resets if resets is not None else [True])
        self.configurations = list(
            configurations
            if configurations is not None
            else [
                {
                    "commands_completed": 2,
                    "new_baud_prompt_observed": True,
                    "first_magic_observed": True,
                }
            ]
        )
        self.watchdogs = list(
            watchdogs if watchdogs is not None else [[snapshot()]]
        )
        self.processes = RecoveryProcesses(self.actions)
        profile_path = self.root / "profile.cfg"
        profile_path.write_text("sensorStop\nsensorStart\n", encoding="utf-8")
        self.config = RadarSupervisorConfig(
            repository_root=self.root,
            output_root=self.root / "output",
            profile_path=profile_path,
            calibration_path=self.root / "calibration.json",
            run_id="recovery",
            xds_serial="RI32",
        )
        self.dependencies = RadarSupervisorDependencies(
            port_provider=self.port_provider,
            reset_target=self.reset_target,
            configure=self.configure,
            processes=self.processes,
            watchdog_factory=self.watchdog_factory,
            monotonic=self.monotonic,
            sleep=self.sleep,
            utc_now=self.utc_now,
        )

    @staticmethod
    def _next(values: list[object]) -> object:
        return values.pop(0) if len(values) > 1 else values[0]

    def port_provider(self) -> list[object]:
        inventory = self._next(self.port_inventories)
        assert isinstance(inventory, list)
        self.actions.append(
            "ports:" + ",".join(str(getattr(port, "device")) for port in inventory)
        )
        return inventory

    def reset_target(
        self,
        port: object,
        config: RadarSupervisorConfig,
    ) -> bool:
        del config
        self.actions.append(f"reset:{getattr(port, 'device')}")
        result = self._next(self.resets)
        if isinstance(result, Exception):
            raise result
        assert isinstance(result, bool)
        return result

    def configure(
        self,
        port: object,
        config: RadarSupervisorConfig,
    ) -> dict[str, object]:
        del config
        self.actions.append(f"configure:{getattr(port, 'device')}")
        result = self._next(self.configurations)
        if isinstance(result, Exception):
            raise result
        assert isinstance(result, dict)
        return result

    def watchdog_factory(
        self,
        paths: EpochPaths,
        config: RadarSupervisorConfig,
        started_at_s: float,
    ) -> FakeWatchdog:
        del paths, config, started_at_s
        values = self._next(self.watchdogs)
        assert isinstance(values, list)
        return FakeWatchdog(values, self.actions)

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, delay_s: float) -> None:
        self.sleeps.append(delay_s)
        self.now_s += delay_s

    def utc_now(self) -> datetime:
        return self.utc

    @property
    def manifest(self) -> Path:
        return manifest_path(self.config.output_root, self.config.run_id)

    def payload(self) -> dict[str, object]:
        return json.loads(self.manifest.read_text(encoding="utf-8"))


class RadarSupervisorStartupTests(unittest.TestCase):
    def test_public_dependency_contracts_and_constructor_are_side_effect_free(self) -> None:
        self.assertEqual(
            list(inspect.signature(RadarSupervisor).parameters),
            ["config", "dependencies"],
        )
        self.assertEqual(
            list(inspect.signature(RadarSupervisor.run).parameters),
            ["self", "stop_requested"],
        )
        self.assertEqual(
            [field.name for field in fields(RadarSupervisorDependencies)],
            [
                "port_provider",
                "reset_target",
                "configure",
                "processes",
                "watchdog_factory",
                "monotonic",
                "sleep",
                "utc_now",
            ],
        )
        for contract in (
            SupervisorChild,
            SupervisorStopResult,
            RadarProcessManager,
            SupervisorWatchdog,
        ):
            self.assertTrue(inspect.isclass(contract))
        self.assertEqual(
            list(inspect.signature(RadarProcessManager.start_capture).parameters),
            ["self", "port", "paths", "config"],
        )
        self.assertEqual(
            list(inspect.signature(RadarProcessManager.switch_viewer).parameters),
            ["self", "current", "paths", "config"],
        )
        self.assertEqual(
            list(inspect.signature(RadarProcessManager.stop_child).parameters),
            ["self", "child"],
        )
        self.assertEqual(
            list(
                inspect.signature(
                    RadarProcessManager.stop_owned_children
                ).parameters
            ),
            ["self"],
        )
        self.assertEqual(
            list(inspect.signature(SupervisorChild.poll).parameters),
            ["self"],
        )
        self.assertEqual(
            list(inspect.signature(SupervisorWatchdog.poll).parameters),
            ["self", "now_s"],
        )

        def unexpected(*args: object, **kwargs: object) -> object:
            del args, kwargs
            raise AssertionError("constructor touched an injected dependency")

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = RadarSupervisorConfig(
                repository_root=root,
                output_root=root / "output",
                profile_path=root / "profile.cfg",
                calibration_path=root / "calibration.json",
                run_id="board-live",
            )
            dependencies = RadarSupervisorDependencies(
                port_provider=unexpected,
                reset_target=unexpected,
                configure=unexpected,
                processes=mock.Mock(),
                watchdog_factory=unexpected,
                monotonic=unexpected,
                sleep=unexpected,
                utc_now=unexpected,
            )
            RadarSupervisor(config, dependencies)
            self.assertFalse(config.output_root.exists())
            with self.assertRaises(FrozenInstanceError):
                dependencies.sleep = lambda delay: None  # type: ignore[misc]

    def test_healthy_startup_has_exact_order_and_stops_explicitly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(directory)
            RadarSupervisor(fixture.config, fixture.dependencies).run(
                fixture.stop_when_running
            )
            self.assertEqual(
                fixture.actions,
                [
                    "wait_port:COM3",
                    "reset:RI32",
                    "wait_port:COM3",
                    "configure:COM3",
                    "capture:e001",
                    "verified:5",
                    "viewer:e001",
                    "running:e001",
                ],
            )

    def test_second_discovery_reenumeration_uses_com9_and_same_serial(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(
                directory,
                ports=[
                    [application_port("COM3")],
                    [application_port("COM9")],
                ],
            )
            RadarSupervisor(fixture.config, fixture.dependencies).run(
                fixture.stop_when_running
            )
            self.assertEqual(fixture.actions[:4], [
                "wait_port:COM3",
                "reset:RI32",
                "wait_port:COM9",
                "configure:COM9",
            ])
            self.assertEqual(
                getattr(fixture.processes.capture_port, "device"),
                "COM9",
            )
            self.assertEqual(
                getattr(fixture.processes.capture_port, "serial_number"),
                "RI32",
            )
            payload = json.loads(fixture.manifest.read_text(encoding="utf-8"))
            self.assertEqual(payload["port"], "COM9")
            self.assertEqual(payload["xds_serial"], "RI32")

    def test_reset_unavailable_is_allowed_once_during_initial_startup(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(directory, reset_result=False)
            RadarSupervisor(fixture.config, fixture.dependencies).run(
                fixture.stop_when_running
            )
            self.assertEqual(fixture.actions.count("reset:RI32"), 1)
            self.assertIn("configure:COM3", fixture.actions)
            self.assertIn("running:e001", fixture.actions)

    def test_bad_configuration_results_fail_before_epoch_allocation(self) -> None:
        invalid_results = (
            {
                "commands_completed": 1,
                "new_baud_prompt_observed": True,
                "first_magic_observed": True,
            },
            {
                "commands_completed": 2,
                "new_baud_prompt_observed": False,
                "first_magic_observed": True,
            },
        )
        for result in invalid_results:
            with self.subTest(result=result), tempfile.TemporaryDirectory() as directory:
                fixture = SupervisorFixture(directory, profile_result=result)
                stopped = False

                def stop_on_retry(delay_s: float) -> None:
                    nonlocal stopped
                    fixture.sleep(delay_s)
                    stopped = True

                dependencies = replace(
                    fixture.dependencies,
                    sleep=stop_on_retry,
                )
                with mock.patch(
                    "sensors.radar_supervisor.allocate_epoch_paths",
                    wraps=allocate_epoch_paths,
                ) as allocate:
                    RadarSupervisor(fixture.config, dependencies).run(
                        lambda: stopped
                    )
                allocate.assert_not_called()
                self.assertIsNone(fixture.processes.started_capture)

    def test_missing_first_magic_can_continue_only_through_watchdog_verification(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(
                directory,
                profile_result={
                    "commands_completed": 2,
                    "new_baud_prompt_observed": True,
                    "first_magic_observed": False,
                },
            )
            RadarSupervisor(fixture.config, fixture.dependencies).run(
                fixture.stop_when_running
            )
            self.assertIn("verified:5", fixture.actions)
            self.assertIn("viewer:e001", fixture.actions)

    def test_viewer_switch_waits_for_fifth_verified_frame(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(directory)
            RadarSupervisor(fixture.config, fixture.dependencies).run(
                fixture.stop_when_running
            )
            self.assertEqual(fixture.processes.viewer_watchdog_poll_count, 5)
            self.assertLess(
                fixture.actions.index("verified:5"),
                fixture.actions.index("viewer:e001"),
            )

    def test_capture_exit_and_watchdog_fault_do_not_start_viewer(self) -> None:
        cases = ("capture_exit", "watchdog_fault")
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                snapshots = [
                    RadarWatchdogSnapshot(
                        False,
                        0,
                        None,
                        None,
                        "radar_verification_timeout",
                    )
                ]
                fixture = SupervisorFixture(directory, snapshots=snapshots)
                if case == "capture_exit":
                    fixture.processes.capture.exit_code = 7
                stopped = False

                def stop_on_retry(delay_s: float) -> None:
                    nonlocal stopped
                    fixture.sleep(delay_s)
                    stopped = True

                dependencies = replace(
                    fixture.dependencies,
                    sleep=stop_on_retry,
                )
                RadarSupervisor(fixture.config, dependencies).run(
                    lambda: stopped
                )
                self.assertIsNone(fixture.processes.started_viewer)
                self.assertEqual(fixture.processes.stop_calls, 1)
                payload = json.loads(
                    fixture.manifest.read_text(encoding="utf-8")
                )
                self.assertEqual(payload["state"], "STOPPED")
                self.assertIsNotNone(payload["epochs"][0]["ended_at"])
                if case == "capture_exit":
                    self.assertEqual(fixture.watchdog.poll_count, 0)
                    self.assertEqual(
                        payload["epochs"][0]["capture_exit_code"],
                        7,
                    )
                    self.assertEqual(
                        payload["epochs"][0]["end_reason"],
                        "capture_exited_before_verification",
                    )
                else:
                    self.assertEqual(
                        payload["epochs"][0]["end_reason"],
                        "radar_verification_timeout",
                    )

    def test_post_capture_dependency_errors_stop_and_finalize_e001(self) -> None:
        cases = ("watchdog_factory", "switch_viewer")
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                fixture = SupervisorFixture(directory)
                if case == "watchdog_factory":
                    def fail_watchdog_factory(
                        paths: EpochPaths,
                        config: RadarSupervisorConfig,
                        started_at_s: float,
                    ) -> SupervisorWatchdog:
                        del paths, config, started_at_s
                        raise LookupError("watchdog factory failed")

                    dependencies = replace(
                        fixture.dependencies,
                        watchdog_factory=fail_watchdog_factory,
                    )
                else:
                    fixture.processes.switch_error = LookupError(
                        "viewer switch failed"
                    )
                    dependencies = fixture.dependencies

                with self.assertRaises(LookupError):
                    RadarSupervisor(fixture.config, dependencies).run(
                        lambda: False
                    )

                self.assertEqual(fixture.processes.stop_calls, 1)
                payload = json.loads(
                    fixture.manifest.read_text(encoding="utf-8")
                )
                self.assertEqual(payload["state"], "STOPPED")
                self.assertEqual(
                    payload["epochs"][0]["end_reason"],
                    "initial_startup_failed",
                )
                self.assertIsNotNone(payload["epochs"][0]["ended_at"])
                self.assertNotIn(
                    41002,
                    [event["pid"] for event in payload["process_events"]],
                )

    def test_running_transition_and_sleep_errors_stop_and_finalize_e001(self) -> None:
        cases = (
            ("manifest", "running_manifest_write_failed", OSError),
            ("sleep", "running_sleep_failed", RuntimeError),
        )
        for case, expected_reason, error_type in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                fixture = SupervisorFixture(directory)
                original_write = write_manifest_atomic
                original_error = error_type(f"{case} injected failure")
                running_write_failed = False

                def fail_running_write_once(
                    path: Path,
                    payload: dict[str, object],
                ) -> None:
                    nonlocal running_write_failed
                    if (
                        payload["state"] == "RUNNING"
                        and not running_write_failed
                    ):
                        running_write_failed = True
                        raise original_error
                    original_write(path, payload)

                def fail_running_sleep(delay_s: float) -> None:
                    if fixture.processes.started_viewer is not None:
                        raise original_error
                    fixture.sleep(delay_s)

                dependencies = fixture.dependencies
                writer = (
                    fail_running_write_once
                    if case == "manifest"
                    else original_write
                )
                if case == "sleep":
                    dependencies = replace(
                        dependencies,
                        sleep=fail_running_sleep,
                    )

                with mock.patch(
                    "sensors.radar_supervisor.write_manifest_atomic",
                    writer,
                ):
                    with self.assertRaises(error_type) as raised:
                        RadarSupervisor(fixture.config, dependencies).run(
                            lambda: False
                        )

                self.assertIs(raised.exception, original_error)
                self.assertEqual(fixture.processes.stop_calls, 1)
                payload = json.loads(
                    fixture.manifest.read_text(encoding="utf-8")
                )
                self.assertEqual(payload["state"], "STOPPED")
                self.assertEqual(payload["last_reason"], expected_reason)
                self.assertEqual(
                    payload["epochs"][0]["end_reason"],
                    expected_reason,
                )
                self.assertEqual(
                    payload["epochs"][0]["capture_exit_code"],
                    0,
                )
                self.assertIsNotNone(payload["epochs"][0]["ended_at"])
                self.assertEqual(
                    [
                        (event["role"], event["action"], event["exit_code"])
                        for event in payload["process_events"]
                    ],
                    [
                        ("capture", "started", None),
                        ("viewer", "started", None),
                        ("capture", "stopped", 0),
                        ("viewer", "stopped", 0),
                    ],
                )
                if case == "manifest":
                    self.assertTrue(running_write_failed)

    def test_stop_requested_during_port_wait_and_verification_stops_cleanly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(directory)
            RadarSupervisor(fixture.config, fixture.dependencies).run(lambda: True)
            self.assertEqual(fixture.actions, [])
            payload = json.loads(fixture.manifest.read_text(encoding="utf-8"))
            self.assertEqual(payload["state"], "STOPPED")
            self.assertEqual(payload["epochs"], [])

        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(
                directory,
                ports=[
                    [
                        application_port("COM3"),
                        application_port("COM9"),
                    ],
                    [
                        application_port("COM3"),
                        application_port("COM9"),
                    ],
                ],
            )

            def stop_after_ambiguous_inventory() -> bool:
                return len(fixture.port_inventories) == 1

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                stop_after_ambiguous_inventory
            )
            self.assertNotIn("reset:RI32", fixture.actions)
            self.assertIsNone(fixture.processes.started_capture)

        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(
                directory,
                snapshots=[
                    RadarWatchdogSnapshot(False, 0, None, None, None),
                ],
            )

            def stop_after_capture() -> bool:
                return fixture.processes.started_capture is not None

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                stop_after_capture
            )
            self.assertIsNone(fixture.processes.started_viewer)
            payload = json.loads(fixture.manifest.read_text(encoding="utf-8"))
            self.assertEqual(payload["state"], "STOPPED")
            self.assertEqual(payload["epochs"][0]["end_reason"], "shutdown")

    def test_manifest_transitions_exact_rows_and_unknown_stop_pid_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = SupervisorFixture(directory)
            fixture.processes.stop_results = (
                FakeStopResult("capture", 41001, 23, "terminate"),
                FakeStopResult("unowned", 99999, 91, "kill"),
                FakeStopResult("viewer", 41002, 0, "graceful"),
            )
            states: list[str] = []
            running_payloads: list[dict[str, object]] = []
            original_write = write_manifest_atomic

            def recording_write(path: Path, payload: dict[str, object]) -> None:
                state = str(payload["state"])
                if not states or states[-1] != state:
                    states.append(state)
                if state == "RUNNING":
                    running_payloads.append(
                        json.loads(json.dumps(payload))
                    )
                original_write(path, payload)

            with mock.patch(
                "sensors.radar_supervisor.write_manifest_atomic",
                recording_write,
            ):
                RadarSupervisor(fixture.config, fixture.dependencies).run(
                    fixture.stop_when_running
                )

            self.assertEqual(
                states,
                [
                    "WAIT_PORT",
                    "RESET_TARGET",
                    "WAIT_PORT",
                    "CONFIGURE",
                    "START_CAPTURE",
                    "VERIFY_FRAMES",
                    "SWITCH_VIEWER",
                    "RUNNING",
                    "STOPPED",
                ],
            )
            self.assertGreaterEqual(len(running_payloads), 1)
            running_payload = running_payloads[0]
            self.assertEqual(running_payload["last_reason"], "verified_frames")
            self.assertEqual(running_payload["verified_consecutive_frames"], 5)
            self.assertEqual(
                running_payload["epochs"][0]["ended_at"],  # type: ignore[index]
                None,
            )
            self.assertEqual(
                [
                    (event["role"], event["action"])  # type: ignore[index]
                    for event in running_payload["process_events"]  # type: ignore[union-attr]
                ],
                [("capture", "started"), ("viewer", "started")],
            )
            payload = json.loads(fixture.manifest.read_text(encoding="utf-8"))
            paths = allocate_epoch_paths(
                fixture.config.output_root,
                "board-live",
                1,
            )
            self.assertEqual(payload["schema_version"], 1)
            self.assertEqual(payload["run_id"], "board-live")
            self.assertEqual(payload["state"], "STOPPED")
            self.assertEqual(payload["epoch"], 1)
            self.assertEqual(payload["recovery_count"], 0)
            self.assertEqual(payload["port"], "COM3")
            self.assertEqual(payload["xds_serial"], "RI32")
            self.assertEqual(payload["mission_path"], str(paths.mission))
            self.assertEqual(payload["raw_path"], str(paths.raw))
            self.assertEqual(payload["last_reason"], "shutdown")
            self.assertEqual(
                payload["epochs"],
                [
                    {
                        "epoch": 1,
                        "mission_path": str(paths.mission),
                        "raw_path": str(paths.raw),
                        "raw_index_path": str(paths.raw_index),
                        "started_at": "2026-07-29T01:00:00Z",
                        "ended_at": "2026-07-29T01:00:00Z",
                        "end_reason": "shutdown",
                        "capture_exit_code": 23,
                    }
                ],
            )
            self.assertEqual(
                payload["process_events"],
                [
                    {
                        "role": "capture",
                        "pid": 41001,
                        "action": "started",
                        "escalation": None,
                        "exit_code": None,
                    },
                    {
                        "role": "viewer",
                        "pid": 41002,
                        "action": "started",
                        "escalation": None,
                        "exit_code": None,
                    },
                    {
                        "role": "capture",
                        "pid": 41001,
                        "action": "stopped",
                        "escalation": "terminate",
                        "exit_code": 23,
                    },
                    {
                        "role": "viewer",
                        "pid": 41002,
                        "action": "stopped",
                        "escalation": "graceful",
                        "exit_code": 0,
                    },
                ],
            )


class RadarSupervisorRecoveryTests(unittest.TestCase):
    @staticmethod
    def _stop_after_running_epoch(
        fixture: RecoveryFixture,
        epoch: int,
    ) -> bool:
        return (
            fixture.manifest.exists()
            and fixture.payload()["state"] == "RUNNING"
            and fixture.payload()["epoch"] == epoch
        )

    def test_healthy_running_frames_do_not_reset_configure_or_rotate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                watchdogs=[[snapshot(), snapshot(), snapshot()]],
            )

            def stop_after_one_monitor_poll() -> bool:
                return bool(fixture.sleeps)

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                stop_after_one_monitor_poll
            )

            payload = fixture.payload()
            self.assertEqual(fixture.actions.count("reset:COM3"), 1)
            self.assertEqual(fixture.actions.count("configure:COM3"), 1)
            self.assertEqual(payload["epoch"], 1)
            self.assertEqual(payload["recovery_count"], 0)
            self.assertEqual(len(payload["epochs"]), 1)

    def test_watchdog_faults_create_one_recovery_episode_and_switch_e002(self) -> None:
        for reason in (
            "radar_frame_timeout",
            "firmware_low_power_timing_assert",
        ):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as directory:
                fixture = RecoveryFixture(
                    directory,
                    watchdogs=[
                        [snapshot(), snapshot(verified=False, fault=reason)],
                        [snapshot()],
                    ],
                )
                stop_checks = 0

                def stop_after_e002_running() -> bool:
                    nonlocal stop_checks
                    stop_checks += 1
                    return (
                        fixture.manifest.exists()
                        and fixture.payload()["state"] == "RUNNING"
                        and fixture.payload()["epoch"] == 2
                    ) or stop_checks >= 50

                RadarSupervisor(fixture.config, fixture.dependencies).run(
                    stop_after_e002_running
                )

                payload = fixture.payload()
                self.assertEqual(payload["epoch"], 2)
                self.assertEqual(payload["recovery_count"], 1)
                self.assertEqual(payload["epochs"][0]["end_reason"], reason)
                stop_capture = next(
                    index
                    for index, action in enumerate(fixture.actions)
                    if action.startswith("stop:capture:")
                )
                recovery_reset = fixture.actions.index("reset:COM3", 2)
                second_capture = fixture.actions.index("capture:e002")
                stop_viewer = next(
                    index
                    for index, action in enumerate(fixture.actions)
                    if action.startswith("stop:viewer:")
                )
                second_viewer = next(
                    index
                    for index, action in enumerate(fixture.actions)
                    if action.startswith("viewer:e002:")
                )
                self.assertLess(stop_capture, recovery_reset)
                self.assertLess(second_capture, stop_viewer)
                self.assertLess(stop_viewer, second_viewer)

    def test_port_loss_recovers_on_com9_and_capture_exit_uses_same_flow(self) -> None:
        cases = ("port_loss", "capture_exit")
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                ports = None
                if case == "port_loss":
                    ports = [
                        [application_port("COM3")],
                        [application_port("COM3")],
                        [],
                        [application_port("COM9")],
                        [application_port("COM9")],
                    ]
                fixture = RecoveryFixture(
                    directory,
                    ports=ports,
                    watchdogs=[[snapshot(), snapshot()], [snapshot()]],
                )
                armed = False
                stop_checks = 0

                def stop_after_recovery() -> bool:
                    nonlocal armed, stop_checks
                    stop_checks += 1
                    if (
                        case == "capture_exit"
                        and not armed
                        and fixture.manifest.exists()
                        and fixture.payload()["state"] == "RUNNING"
                    ):
                        capture = next(
                            child
                            for child in reversed(fixture.processes.children)
                            if child.role == "capture"
                        )
                        capture.exit_code = 17
                        armed = True
                    return (
                        fixture.manifest.exists()
                        and fixture.payload()["state"] == "RUNNING"
                        and fixture.payload()["epoch"] == 2
                    ) or stop_checks >= 50

                RadarSupervisor(fixture.config, fixture.dependencies).run(
                    stop_after_recovery
                )

                payload = fixture.payload()
                self.assertEqual(payload["recovery_count"], 1)
                self.assertEqual(payload["epoch"], 2)
                if case == "port_loss":
                    self.assertEqual(fixture.processes.capture_ports, ["COM3", "COM9"])
                    self.assertEqual(payload["port"], "COM9")
                else:
                    self.assertEqual(
                        payload["epochs"][0]["capture_exit_code"],
                        17,
                    )

    def test_viewer_exit_restarts_same_epoch_without_radar_recovery(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                watchdogs=[[snapshot(), snapshot(), snapshot()]],
            )
            armed = False
            stop_checks = 0

            def stop_after_viewer_restart() -> bool:
                nonlocal armed, stop_checks
                stop_checks += 1
                viewers = [
                    child
                    for child in fixture.processes.children
                    if child.role == "viewer"
                ]
                if viewers and not armed:
                    viewers[-1].exit_code = 9
                    armed = True
                    return False
                return len(viewers) == 2 or stop_checks >= 50

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                stop_after_viewer_restart
            )

            payload = fixture.payload()
            self.assertEqual(payload["epoch"], 1)
            self.assertEqual(payload["recovery_count"], 0)
            self.assertEqual(fixture.actions.count("reset:COM3"), 1)
            self.assertEqual(fixture.actions.count("configure:COM3"), 1)
            self.assertEqual(
                [action for action in fixture.actions if action.startswith("viewer:")],
                ["viewer:e001:none", "viewer:e001:none"],
            )

    def test_old_viewer_stays_active_until_replacement_five_frame_gate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                watchdogs=[
                    [
                        snapshot(),
                        snapshot(verified=False, frames=0, fault="radar_frame_timeout"),
                    ],
                    [
                        snapshot(verified=False, frames=1),
                        snapshot(verified=False, frames=2),
                        snapshot(verified=False, frames=0),
                        snapshot(verified=False, frames=1),
                        snapshot(verified=False, frames=2),
                        snapshot(verified=False, frames=3),
                        snapshot(verified=False, frames=4),
                        snapshot(),
                    ],
                ],
            )

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                lambda: self._stop_after_running_epoch(fixture, 2)
            )

            verified = [
                index
                for index, action in enumerate(fixture.actions)
                if action == "verified:5"
            ]
            old_viewer_stop = next(
                index
                for index, action in enumerate(fixture.actions)
                if action.startswith("stop:viewer:")
            )
            self.assertEqual(len(verified), 2)
            self.assertLess(verified[1], old_viewer_stop)
            self.assertEqual(fixture.payload()["epochs"][0]["end_reason"], "radar_frame_timeout")

    def test_recovery_attempt_failures_do_not_start_new_fault_episodes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                watchdogs=[
                    [
                        snapshot(),
                        snapshot(
                            verified=False,
                            frames=0,
                            fault="radar_frame_timeout",
                        ),
                    ],
                    [
                        snapshot(
                            verified=False,
                            frames=0,
                            fault="radar_verification_timeout",
                        )
                    ],
                    [snapshot()],
                ],
            )

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                lambda: self._stop_after_running_epoch(fixture, 3)
            )

            payload = fixture.payload()
            self.assertEqual(payload["recovery_count"], 1)
            self.assertEqual(
                [row["epoch"] for row in payload["epochs"]],
                [1, 2, 3],
            )
            first_viewer_stop = next(
                index
                for index, action in enumerate(fixture.actions)
                if action.startswith("stop:viewer:")
            )
            third_epoch_verified = [
                index
                for index, action in enumerate(fixture.actions)
                if action == "verified:5"
            ][1]
            self.assertLess(third_epoch_verified, first_viewer_stop)
            self.assertEqual(
                [
                    delay
                    for delay in fixture.sleeps
                    if delay != fixture.config.poll_interval_s
                ],
                [0.5],
            )

    def test_failures_back_off_with_cap_and_preserve_epoch_rules(self) -> None:
        invalid_profile = {
            "commands_completed": 1,
            "new_baud_prompt_observed": True,
            "first_magic_observed": False,
        }
        valid_profile = {
            "commands_completed": 2,
            "new_baud_prompt_observed": True,
            "first_magic_observed": False,
        }
        cases = {
            "reset": {
                "resets": [RuntimeError("reset") for _ in range(5)] + [True],
            },
            "configuration": {
                "configurations": [invalid_profile for _ in range(5)]
                + [valid_profile],
            },
            "verification": {
                "watchdogs": [
                    [
                        snapshot(
                            verified=False,
                            frames=0,
                            fault="radar_verification_timeout",
                        )
                    ]
                    for _ in range(5)
                ]
                + [[snapshot()]],
            },
        }
        for case, options in cases.items():
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                fixture = RecoveryFixture(directory, **options)  # type: ignore[arg-type]
                expected_epoch = 6 if case == "verification" else 1

                RadarSupervisor(fixture.config, fixture.dependencies).run(
                    lambda: self._stop_after_running_epoch(
                        fixture,
                        expected_epoch,
                    )
                )

                filtered = [
                    delay
                    for delay in fixture.sleeps
                    if delay != fixture.config.poll_interval_s
                ]
                self.assertEqual(filtered[:5], [0.5, 1.0, 2.0, 4.0, 5.0])
                payload = fixture.payload()
                if case in ("reset", "configuration"):
                    self.assertEqual(
                        [row["epoch"] for row in payload["epochs"]],
                        [1],
                    )
                    self.assertEqual(
                        [
                            action
                            for action in fixture.actions
                            if action.startswith("capture:")
                        ],
                        ["capture:e001"],
                    )
                else:
                    self.assertEqual(
                        [row["epoch"] for row in payload["epochs"]],
                        [1, 2, 3, 4, 5, 6],
                    )
                    for path, content in fixture.processes.artifact_bytes.items():
                        self.assertEqual(path.read_bytes(), content)

    def test_no_reset_tool_waits_for_real_usb_cycle_after_silent_fault(self) -> None:
        ambiguous = [
            application_port("COM3"),
            application_port("COM4"),
        ]
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                ports=[
                    [application_port("COM3")],
                    [application_port("COM3")],
                    [application_port("COM3")],
                    [application_port("COM3")],
                    ambiguous,
                    [],
                    [application_port("COM9")],
                ],
                resets=[True, False],
                watchdogs=[
                    [
                        snapshot(),
                        snapshot(
                            verified=False,
                            frames=0,
                            fault="radar_frame_timeout",
                        ),
                    ],
                    [snapshot()],
                ],
            )
            reasons: list[str] = []
            original_write = write_manifest_atomic

            def recording_write(path: Path, payload: dict[str, object]) -> None:
                reasons.append(str(payload["last_reason"]))
                original_write(path, payload)

            with mock.patch(
                "sensors.radar_supervisor.write_manifest_atomic",
                recording_write,
            ):
                RadarSupervisor(fixture.config, fixture.dependencies).run(
                    lambda: self._stop_after_running_epoch(fixture, 2)
                )

            self.assertIn(
                "reset_tool_unavailable_waiting_for_usb_cycle",
                reasons,
            )
            second_configure = fixture.actions.index("configure:COM9")
            ambiguous_poll = fixture.actions.index("ports:COM3,COM4")
            absent_poll = fixture.actions.index("ports:")
            reappeared_poll = fixture.actions.index("ports:COM9")
            self.assertLess(ambiguous_poll, absent_poll)
            self.assertLess(absent_poll, reappeared_poll)
            self.assertLess(reappeared_poll, second_configure)
            self.assertEqual(fixture.processes.capture_ports, ["COM3", "COM9"])

    def test_no_reset_tool_uses_already_observed_port_loss(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                ports=[
                    [application_port("COM3")],
                    [application_port("COM3")],
                    [],
                    [application_port("COM9")],
                ],
                resets=[True, False],
                watchdogs=[[snapshot()], [snapshot()]],
            )

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                lambda: self._stop_after_running_epoch(fixture, 2)
            )

            self.assertEqual(fixture.processes.capture_ports, ["COM3", "COM9"])
            self.assertIn("configure:COM9", fixture.actions)
            self.assertEqual(fixture.payload()["epochs"][0]["end_reason"], "application_port_lost")

    def test_shutdown_during_backoff_and_usb_cycle_wait_is_bounded(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                resets=[RuntimeError("reset unavailable")],
            )
            stop = False

            def stop_after_backoff(delay_s: float) -> None:
                nonlocal stop
                fixture.sleep(delay_s)
                stop = True

            dependencies = replace(
                fixture.dependencies,
                sleep=stop_after_backoff,
            )
            RadarSupervisor(fixture.config, dependencies).run(lambda: stop)
            self.assertEqual(fixture.payload()["state"], "STOPPED")
            self.assertEqual(fixture.payload()["epochs"], [])

        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                ports=[
                    [application_port("COM3")],
                    [application_port("COM3")],
                    [application_port("COM3")],
                    [application_port("COM3")],
                ],
                resets=[True, False],
                watchdogs=[
                    [
                        snapshot(),
                        snapshot(
                            verified=False,
                            frames=0,
                            fault="radar_frame_timeout",
                        ),
                    ]
                ],
            )

            def stop_in_usb_gate() -> bool:
                return (
                    fixture.manifest.exists()
                    and fixture.payload()["last_reason"]
                    == "reset_tool_unavailable_waiting_for_usb_cycle"
                )

            RadarSupervisor(fixture.config, fixture.dependencies).run(
                stop_in_usb_gate
            )
            payload = fixture.payload()
            self.assertEqual(payload["state"], "STOPPED")
            self.assertEqual(payload["recovery_count"], 1)
            self.assertEqual(len(payload["epochs"]), 1)

    def test_manifest_after_recovery_has_exact_epoch_and_process_history(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fixture = RecoveryFixture(
                directory,
                watchdogs=[
                    [
                        snapshot(),
                        snapshot(
                            verified=False,
                            frames=0,
                            fault="firmware_low_power_timing_assert",
                        ),
                    ],
                    [snapshot()],
                ],
            )
            RadarSupervisor(fixture.config, fixture.dependencies).run(
                lambda: self._stop_after_running_epoch(fixture, 2)
            )
            payload = fixture.payload()
            e001_mission = (
                fixture.config.output_root
                / "missions"
                / "radar-board-live-recovery-e001.jsonl"
            )
            e002_mission = (
                fixture.config.output_root
                / "missions"
                / "radar-board-live-recovery-e002.jsonl"
            )
            e002_raw = (
                fixture.config.output_root
                / "captures"
                / "radar-board-live-recovery-e002.bin"
            )
            self.assertEqual(payload["epoch"], 2)
            self.assertEqual(payload["recovery_count"], 1)
            self.assertEqual(payload["mission_path"], str(e002_mission))
            self.assertEqual(payload["raw_path"], str(e002_raw))
            self.assertEqual(
                [row["epoch"] for row in payload["epochs"]],
                [1, 2],
            )
            self.assertEqual(
                payload["epochs"][0]["mission_path"],
                str(e001_mission),
            )
            self.assertEqual(
                payload["epochs"][0]["end_reason"],
                "firmware_low_power_timing_assert",
            )
            self.assertEqual(
                [
                    (event["role"], event["pid"], event["action"])
                    for event in payload["process_events"]
                ],
                [
                    ("capture", 51001, "started"),
                    ("viewer", 52002, "started"),
                    ("capture", 51001, "stopped"),
                    ("capture", 51003, "started"),
                    ("viewer", 52002, "stopped"),
                    ("viewer", 52004, "started"),
                    ("capture", 51003, "stopped"),
                    ("viewer", 52004, "stopped"),
                ],
            )

    def test_mismatched_stop_child_result_raises_and_is_not_recorded(self) -> None:
        for result in (
            FakeStopResult("viewer", 99999, 0, "graceful"),
            FakeStopResult("capture", 52002, 0, "graceful"),
        ):
            with self.subTest(result=result), tempfile.TemporaryDirectory() as directory:
                fixture = RecoveryFixture(
                    directory,
                    watchdogs=[[snapshot(), snapshot()]],
                )
                armed = False

                def trigger_viewer_exit() -> bool:
                    nonlocal armed
                    viewers = [
                        child
                        for child in fixture.processes.children
                        if child.role == "viewer"
                    ]
                    if viewers and not armed:
                        viewers[0].exit_code = 9
                        fixture.processes.stop_override = result
                        armed = True
                    return False

                with self.assertRaises(RuntimeError):
                    RadarSupervisor(fixture.config, fixture.dependencies).run(
                        trigger_viewer_exit
                    )
                payload = fixture.payload()
                self.assertNotIn(
                    (result.role, result.pid),
                    [
                        (event["role"], event["pid"])
                        for event in payload["process_events"]
                        if event["action"] == "stopped"
                    ],
                )


if __name__ == "__main__":
    unittest.main()
