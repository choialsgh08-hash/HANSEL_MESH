"""Contract tests for radar supervisor artifacts and configuration."""

from __future__ import annotations

from dataclasses import fields
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from sensors.radar_supervisor import (
    EpochPaths,
    RadarSupervisorConfig,
    SupervisorState,
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

    def test_config_rejects_invalid_numeric_and_identifier_values(self) -> None:
        integer_fields = (
            "initial_baud",
            "data_baud",
            "heatmap_azimuth_bins",
            "heatmap_range_bins",
            "verification_frames",
        )
        for field in integer_fields:
            for value in (0, -1, True):
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

        for value in (0, -1, True, 65_536):
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


if __name__ == "__main__":
    unittest.main()
