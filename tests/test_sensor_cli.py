import argparse
import contextlib
import io
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from sensors.cli import (
    command_inspect,
    command_radar_bin,
    command_radar_index,
    command_radar_live,
)
from common.sensor_json import canonical_json_bytes
from sensors.mission_log import MissionLogWriter
from sensors.radar_capture import RadarCaptureStats
from tests.test_radar_capture import one_point_packet


class SensorCliTests(unittest.TestCase):
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
            missing_point_tlv_frames=0,
            complete_frames=0,
            incomplete_frames=0,
            points_decoded=0,
            radar_frame_gaps=0,
            device_discontinuities=0,
            writer_drops=0,
            parser_errors=0,
            parser_discarded_bytes=0,
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
