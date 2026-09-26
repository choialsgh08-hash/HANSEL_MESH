import argparse
import contextlib
import io
import hashlib
import json
from pathlib import Path
import signal
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

from sensors.cli import (
    command_inspect,
    command_radar_bin,
    command_radar_index,
    command_radar_live,
    main,
)
from common.sensor_json import canonical_json_bytes
from sensors.mission_log import MissionLogWriter
from sensors.radar_owner_lock import RADAR_UART_LOCK_ROOT
from sensors.radar_parent_lease import create_parent_death_lease
from sensors.radar_capture import RadarCaptureStats
from tests.test_radar_capture import (
    one_point_heatmap_packet,
    one_point_packet,
)


REAL_PROFILE = (
    "lsdk-05.05.04.02-presence-near-heatmap16-elev8-cfar15-10hz-v1"
)
REAL_FIXTURE = (
    Path(__file__).parent
    / "fixtures"
    / "radar_clutter_20260728_f2286_f2291.jsonl"
)


class SensorCliTests(unittest.TestCase):
    def invoke_calibrate(self, input_path, output_path, *extra):
        argv = [
            "sensors",
            "radar-calibrate",
            str(input_path),
            "--output",
            str(output_path),
            *extra,
        ]
        stdout = io.StringIO()
        stderr = io.StringIO()
        with mock.patch("sys.argv", argv), contextlib.redirect_stdout(
            stdout
        ), contextlib.redirect_stderr(stderr):
            try:
                result = main()
            except SystemExit as error:
                result = error.code
        return result, stdout.getvalue(), stderr.getvalue()

    def test_radar_calibrate_writes_deterministic_profile_bound_model(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "model.json"
            result, stdout, _ = self.invoke_calibrate(
                REAL_FIXTURE,
                output,
                "--min-frames",
                "6",
            )
            self.assertEqual(result, 0)
            self.assertEqual(
                json.loads(output.read_text("utf-8"))["profile_id"],
                REAL_PROFILE,
            )
            self.assertEqual(json.loads(stdout)["frames_used"], 6)

    def test_radar_calibrate_refuses_existing_output_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "model.json"
            output.write_text("owned", encoding="utf-8")
            result, _, stderr = self.invoke_calibrate(
                REAL_FIXTURE,
                output,
                "--min-frames",
                "6",
            )
            self.assertNotEqual(result, 0)
            self.assertEqual(output.read_text("utf-8"), "owned")
            self.assertIn("--overwrite", stderr)

    def test_radar_calibrate_rejects_mixed_profiles(self):
        with tempfile.TemporaryDirectory() as directory:
            mixed = Path(directory) / "mixed.jsonl"
            entries = [
                json.loads(line)
                for line in REAL_FIXTURE.read_text("utf-8").splitlines()
            ]
            entries[-1]["record"]["payload"]["profile_id"] = "other-profile"
            mixed.write_text(
                "".join(
                    json.dumps(
                        entry,
                        ensure_ascii=False,
                        sort_keys=True,
                        separators=(",", ":"),
                    )
                    + "\n"
                    for entry in entries
                ),
                encoding="utf-8",
            )
            output = Path(directory) / "model.json"
            result, _, stderr = self.invoke_calibrate(
                mixed,
                output,
                "--min-frames",
                "6",
            )
            self.assertNotEqual(result, 0)
            self.assertIn("mixed profile", stderr)

    def test_radar_calibrate_rejects_insufficient_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "model.json"
            result, _, stderr = self.invoke_calibrate(
                REAL_FIXTURE,
                output,
                "--min-frames",
                "7",
            )
            self.assertNotEqual(result, 0)
            self.assertIn("at least 7", stderr)

    def test_radar_calibrate_rejects_minimum_below_five(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "model.json"
            result, _, stderr = self.invoke_calibrate(
                REAL_FIXTURE,
                output,
                "--min-frames",
                "4",
            )
            self.assertNotEqual(result, 0)
            self.assertFalse(output.exists())
            self.assertIn("--min-frames", stderr)

    def test_radar_calibrate_accepts_minimum_of_five(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "model.json"
            result, stdout, stderr = self.invoke_calibrate(
                REAL_FIXTURE,
                output,
                "--min-frames",
                "5",
            )
            self.assertEqual(result, 0, stderr)
            self.assertTrue(output.exists())
            self.assertEqual(json.loads(stdout)["frames_used"], 6)

    def test_heatmap_cli_requires_range_bins_with_other_settings(self):
        stderr = io.StringIO()
        with mock.patch(
            "sys.argv",
            [
                "sensors",
                "radar-bin",
                "unused.bin",
                "--heatmap-azimuth-bins",
                "4",
                "--heatmap-range-step-m",
                "0.05",
            ],
        ), contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                main()

        self.assertEqual(raised.exception.code, 2)
        self.assertIn("must be supplied together", stderr.getvalue())

    def test_empty_radar_binary_is_not_reported_usable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.bin"
            path.write_bytes(b"")
            args = argparse.Namespace(
                path=str(path),
                header_size="40",
                float_point_tlv=1,
                side_info_tlv=7,
                compressed_point_tlv=301,
                tlv_length_includes_header=False,
                chunk_bytes=4096,
                frames=False,
            )
            with contextlib.redirect_stdout(io.StringIO()):
                result = command_radar_bin(args)
            self.assertEqual(result, 2)

    def test_radar_binary_with_frame_gap_is_not_reported_usable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gap.bin"
            path.write_bytes(
                one_point_packet(frame_number=1)
                + one_point_packet(frame_number=3)
            )
            args = argparse.Namespace(
                path=str(path),
                header_size="40",
                float_point_tlv=1,
                side_info_tlv=7,
                compressed_point_tlv=301,
                tlv_length_includes_header=False,
                chunk_bytes=4096,
                frames=False,
            )
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                result = command_radar_bin(args)
            report = json.loads(output.getvalue())
            self.assertEqual(result, 2)
            self.assertEqual(report["radar_frame_gaps"], 1)

    def test_radar_binary_startup_resync_requires_explicit_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mid-stream.bin"
            prefix = b"garbage-prefix"
            path.write_bytes(prefix + one_point_packet())
            args = argparse.Namespace(
                path=str(path),
                header_size="40",
                float_point_tlv=1,
                side_info_tlv=7,
                compressed_point_tlv=301,
                tlv_length_includes_header=False,
                chunk_bytes=4096,
                frames=False,
                allow_startup_resync=False,
            )

            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                strict_result = command_radar_bin(args)
            report = json.loads(output.getvalue())
            self.assertEqual(strict_result, 2)
            self.assertEqual(
                report["startup_sync_discarded_bytes"],
                len(prefix),
            )
            self.assertEqual(report["post_sync_discarded_bytes"], 0)

            args.allow_startup_resync = True
            with contextlib.redirect_stdout(io.StringIO()):
                resync_result = command_radar_bin(args)
            self.assertEqual(resync_result, 0)

    def test_radar_binary_reports_decoded_heatmap(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "heatmap.bin"
            path.write_bytes(one_point_heatmap_packet())
            args = argparse.Namespace(
                path=str(path),
                header_size="40",
                float_point_tlv=1,
                side_info_tlv=7,
                compressed_point_tlv=301,
                heatmap_azimuth_bins=4,
                heatmap_range_bins=2,
                heatmap_range_step_m=0.05,
                tlv_length_includes_header=False,
                chunk_bytes=4096,
                frames=False,
            )

            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                result = command_radar_bin(args)
            report = json.loads(output.getvalue())

            self.assertEqual(result, 0)
            self.assertEqual(report["heatmap_frames"], 1)
            self.assertEqual(report["major_heatmap_frames"], 1)
            self.assertEqual(report["missing_heatmap_frames"], 0)
            self.assertEqual(report["heatmap_cells_decoded"], 8)
            self.assertTrue(report["heatmap_expected"])
            self.assertEqual(report["heatmap_azimuth_bins"], 4)
            self.assertEqual(report["heatmap_range_bins"], 2)
            self.assertEqual(report["heatmap_range_step_m"], 0.05)

    def test_empty_mission_log_is_not_reported_healthy(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.jsonl"
            with MissionLogWriter(path):
                pass
            args = argparse.Namespace(path=str(path))
            with contextlib.redirect_stdout(io.StringIO()):
                result = command_inspect(args)
            self.assertEqual(result, 2)

    def test_live_capture_forwards_raw_index_and_rejects_zero_frames(self):
        stats = RadarCaptureStats(
            frames_decoded=0,
            point_cloud_frames=0,
            float_point_frames=0,
            compressed_point_frames=0,
            empty_point_frames=0,
            nonzero_padding_frames=0,
            missing_point_tlv_frames=0,
            complete_frames=0,
            incomplete_frames=0,
            points_decoded=0,
            radar_frame_gaps=0,
            device_discontinuities=0,
            writer_drops=0,
            parser_errors=0,
            parser_discarded_bytes=0,
            startup_sync_parse_errors=0,
            startup_sync_discarded_bytes=0,
            post_sync_parse_errors=0,
            post_sync_discarded_bytes=0,
            buffered_tail_bytes=0,
            raw_bytes=0,
            max_timing_quality_metric_ns=0,
            mission_log="mission.jsonl",
            raw_capture="radar.bin",
            raw_index="timing.jsonl",
        )
        args = argparse.Namespace(
            port="COM5",
            baud=115200,
            output="mission.jsonl",
            mission_id="mission-1",
            profile_id="sdk5502-profile",
            calibration_id="uncalibrated",
            unit_id="head",
            boot_id=None,
            raw_output="radar.bin",
            raw_index="timing.jsonl",
            duration=1.0,
            read_bytes=1024,
            serial_timeout=0.01,
            overwrite=False,
            header_size="40",
            allow_elided_empty_point_tlv=True,
            allow_nonzero_padding=True,
            heatmap_azimuth_bins=4,
            heatmap_range_bins=2,
            heatmap_range_step_m=0.05,
        )
        with mock.patch(
            "sensors.cli.capture_radar_uart",
            return_value=stats,
        ) as capture, contextlib.redirect_stdout(io.StringIO()):
            result = command_radar_live(args)

        self.assertEqual(result, 2)
        self.assertEqual(
            capture.call_args.kwargs["raw_index"],
            Path("timing.jsonl"),
        )
        self.assertTrue(
            capture.call_args.kwargs["allow_elided_empty_point_tlv"]
        )
        self.assertTrue(capture.call_args.kwargs["allow_nonzero_padding"])
        self.assertEqual(
            capture.call_args.kwargs["heatmap_azimuth_bins"],
            4,
        )
        self.assertEqual(
            capture.call_args.kwargs["heatmap_range_bins"],
            2,
        )
        self.assertEqual(
            capture.call_args.kwargs["heatmap_range_step_m"],
            0.05,
        )

    def test_managed_radar_live_holds_uart_lease_and_watches_parent(self):
        stats = RadarCaptureStats(
            frames_decoded=0,
            point_cloud_frames=0,
            float_point_frames=0,
            compressed_point_frames=0,
            empty_point_frames=0,
            nonzero_padding_frames=0,
            missing_point_tlv_frames=0,
            complete_frames=0,
            incomplete_frames=0,
            points_decoded=0,
            radar_frame_gaps=0,
            device_discontinuities=0,
            writer_drops=0,
            parser_errors=0,
            parser_discarded_bytes=0,
            startup_sync_parse_errors=0,
            startup_sync_discarded_bytes=0,
            post_sync_parse_errors=0,
            post_sync_discarded_bytes=0,
            buffered_tail_bytes=0,
            raw_bytes=0,
            max_timing_quality_metric_ns=0,
            mission_log="mission.jsonl",
            raw_capture="radar.bin",
            raw_index="timing.jsonl",
        )
        with tempfile.TemporaryDirectory() as directory:
            lease = create_parent_death_lease(
                Path(directory) / "leases",
                "capture",
            )
            args = argparse.Namespace(
                port="COM5",
                baud=115200,
                output="mission.jsonl",
                mission_id="mission-1",
                profile_id="sdk5502-profile",
                calibration_id="uncalibrated",
                unit_id="head",
                boot_id=None,
                raw_output="radar.bin",
                raw_index="timing.jsonl",
                duration=1.0,
                read_bytes=1024,
                serial_timeout=0.01,
                health_interval=0.5,
                overwrite=False,
                header_size="40",
                allow_elided_empty_point_tlv=True,
                allow_nonzero_padding=True,
                heatmap_azimuth_bins=4,
                heatmap_range_bins=2,
                heatmap_range_step_m=0.05,
                supervisor_parent_lease=lease.path,
                xds_owner_serial="RI32",
                xds_owner_run_id="managed-run",
            )
            uart_lock = mock.Mock()
            observed_stop_requests = []

            def capture_managed(**kwargs):
                observed_stop_requests.append(
                    kwargs["stop_requested"]()
                )
                return stats

            try:
                with mock.patch(
                    "sensors.cli.capture_radar_uart",
                    side_effect=capture_managed,
                ) as capture, mock.patch(
                    "sensors.cli.acquire_radar_owner_lock",
                    return_value=uart_lock,
                    create=True,
                ) as acquire, contextlib.redirect_stdout(io.StringIO()):
                    result = command_radar_live(args)
            finally:
                lease.release()
                deadline = time.monotonic() + 5.0
                while lease.path.exists() and time.monotonic() < deadline:
                    time.sleep(0.01)

        self.assertEqual(result, 2)
        acquire.assert_called_once_with(
            RADAR_UART_LOCK_ROOT,
            "RI32",
            "managed-run",
        )
        stop_requested = capture.call_args.kwargs["stop_requested"]
        self.assertTrue(callable(stop_requested))
        self.assertEqual(observed_stop_requests, [False])
        uart_lock.release.assert_called_once_with()

    def test_radar_live_converts_sigterm_inside_capture_and_writes_final_footer(self):
        class InterruptingSerial:
            def __enter__(self):
                return self

            def __exit__(self, *unused):
                return False

            def read(self, size):
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)

        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            mission = directory / "mission.jsonl"
            raw = directory / "radar.bin"
            index = directory / "radar.index.jsonl"
            args = argparse.Namespace(
                port="COM3", baud=1250000, output=str(mission),
                mission_id="mission-1", profile_id="sdk5502-profile",
                calibration_id="uncalibrated", unit_id="head", boot_id="boot-1",
                raw_output=str(raw), raw_index=str(index), duration=0.0,
                read_bytes=1024, serial_timeout=0.01, health_interval=0.5,
                overwrite=False, header_size="40",
                allow_elided_empty_point_tlv=True, allow_nonzero_padding=True,
                heatmap_azimuth_bins=4, heatmap_range_bins=2,
                heatmap_range_step_m=0.05,
            )
            previous = signal.getsignal(signal.SIGTERM)
            with mock.patch.dict(
                sys.modules,
                {"serial": types.SimpleNamespace(Serial=lambda **kwargs: InterruptingSerial())},
            ), contextlib.redirect_stdout(io.StringIO()):
                command_radar_live(args)

            self.assertIs(signal.getsignal(signal.SIGTERM), previous)
            self.assertEqual(
                json.loads(index.read_text("utf-8").splitlines()[-1])["stop_reason"],
                "keyboard_interrupt",
            )
            self.assertIn("health/radar", mission.read_text("utf-8"))

    def test_radar_live_converts_sigbreak_inside_capture_when_available(self):
        class FakeSignals:
            SIGTERM = 15
            SIGBREAK = 21

            def __init__(self):
                self.handlers = {self.SIGTERM: "term-before", self.SIGBREAK: "break-before"}

            def getsignal(self, signum):
                return self.handlers[signum]

            def signal(self, signum, handler):
                self.handlers[signum] = handler

        fake_signals = FakeSignals()
        args = argparse.Namespace(
            port="COM3", baud=1250000, output="mission.jsonl",
            mission_id="mission-1", profile_id="sdk5502-profile",
            calibration_id="uncalibrated", unit_id="head", boot_id=None,
            raw_output=None, raw_index=None, duration=0.0, read_bytes=1024,
            serial_timeout=0.01, health_interval=0.5, overwrite=False,
            header_size="40", allow_elided_empty_point_tlv=False,
            allow_nonzero_padding=False, heatmap_azimuth_bins=None,
            heatmap_range_bins=None, heatmap_range_step_m=None,
        )

        def capture_with_break(**kwargs):
            fake_signals.getsignal(fake_signals.SIGBREAK)(fake_signals.SIGBREAK, None)

        with mock.patch("sensors.cli.signal", fake_signals), mock.patch(
            "sensors.cli.capture_radar_uart", side_effect=capture_with_break
        ):
            with self.assertRaises(KeyboardInterrupt):
                command_radar_live(args)

        self.assertEqual(fake_signals.handlers[FakeSignals.SIGTERM], "term-before")
        self.assertEqual(fake_signals.handlers[FakeSignals.SIGBREAK], "break-before")

    def test_radar_index_uses_default_sidecar_and_returns_healthy(self):
        metadata = {
            "mission_id": "mission-1",
            "unit_id": "head",
            "boot_id": "boot-1",
            "producer_id": "radar-reader-1",
            "profile_id": "sdk5502-profile",
            "calibration_id": "uncalibrated",
            "baudrate": 115200,
        }
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "radar.bin"
            index = Path(f"{raw}.chunks.jsonl")
            raw.write_bytes(b"ab")
            chunk = {
                "index_version": 1,
                "record_type": "uart_chunk",
                **metadata,
                "chunk_seq": 1,
                "byte_offset": 0,
                "byte_length": 2,
                "read_started_ns": 100,
                "read_finished_ns": 110,
                "observation_midpoint_ns": 105,
                "timing_quality_metric_ns": 10,
            }
            footer = {
                "index_version": 1,
                "record_type": "capture_end",
                **metadata,
                "chunks": 1,
                "raw_bytes": 2,
                "raw_sha256": hashlib.sha256(b"ab").hexdigest(),
                "frames_decoded": 1,
                "ended_monotonic_ns": 120,
                "stop_reason": "test_complete",
            }
            index.write_bytes(
                canonical_json_bytes(chunk)
                + b"\n"
                + canonical_json_bytes(footer)
                + b"\n"
            )
            args = argparse.Namespace(path=str(raw), index=None)
            with contextlib.redirect_stdout(io.StringIO()):
                result = command_radar_index(args)
            self.assertEqual(result, 0)


if __name__ == "__main__":
    unittest.main()
