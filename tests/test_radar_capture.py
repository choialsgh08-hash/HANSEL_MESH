import struct
import hashlib
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

from common.sensor_json import strict_json_loads
from sensors.mission_log import iter_mission_log
from sensors.radar_capture import (
    capture_radar_uart,
    classify_frame_transition,
    estimate_uart_observation_time,
    frame_gap,
)
from sensors.raw_capture_index import inspect_uart_chunk_index
from sensors.ti_mmwave import TI_MAGIC_WORD


def one_point_packet(frame_number=1):
    units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 1, 0)
    point = struct.pack("<hhhhBB", 10, 20, 0, -2, 8, 3)
    payload = units + point
    body = struct.pack("<II", 301, len(payload)) + payload
    padding = b"\x00" * ((-(40 + len(body))) % 32)
    total = 40 + len(body) + len(padding)
    header = TI_MAGIC_WORD + struct.pack(
        "<8I",
        0x05050002,
        total,
        0xA6432,
        frame_number,
        123,
        1,
        1,
        0,
    )
    return header + body + padding


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reads = [one_point_packet()]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return None

    def read(self, size):
        if self.reads:
            return self.reads.pop(0)
        raise KeyboardInterrupt


class ResettingFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reads = [
            one_point_packet(frame_number=1000),
            one_point_packet(frame_number=1),
        ]


class PeriodicDiagnosticFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.read_count = 0

    def read(self, size):
        self.read_count += 1
        if self.read_count == 1:
            return b"not-a-ti-frame" * 8
        if self.read_count == 2:
            time.sleep(0.02)
            return b""
        raise KeyboardInterrupt


class RadarCaptureTests(unittest.TestCase):
    def test_live_capture_writes_raw_frame_and_health_record(self):
        fake_serial_module = SimpleNamespace(Serial=FakeSerial)
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            root = Path(directory)
            mission = root / "mission.jsonl"
            raw = root / "radar.bin"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=115200,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="sdk5502-test-profile",
                calibration_id="uncalibrated",
                boot_id="boot-1",
                raw_capture=raw,
            )

            self.assertEqual(stats.frames_decoded, 1)
            self.assertEqual(stats.points_decoded, 1)
            self.assertEqual(stats.point_cloud_frames, 1)
            self.assertEqual(stats.parser_errors, 0)
            self.assertEqual(raw.read_bytes(), one_point_packet())
            self.assertEqual(stats.raw_index, f"{raw}.chunks.jsonl")
            raw_index = Path(stats.raw_index)
            index_lines = raw_index.read_bytes().splitlines()
            self.assertEqual(len(index_lines), 2)
            index_record = strict_json_loads(index_lines[0])
            self.assertEqual(index_record["record_type"], "uart_chunk")
            self.assertEqual(index_record["byte_offset"], 0)
            self.assertEqual(
                index_record["byte_length"],
                len(one_point_packet()),
            )
            self.assertEqual(
                index_record["profile_id"],
                "sdk5502-test-profile",
            )
            self.assertEqual(index_record["baudrate"], 115200)
            self.assertGreater(
                index_record["timing_quality_metric_ns"],
                0,
            )
            footer = strict_json_loads(index_lines[1])
            self.assertEqual(footer["record_type"], "capture_end")
            self.assertEqual(footer["chunks"], 1)
            self.assertEqual(footer["raw_bytes"], len(one_point_packet()))
            self.assertEqual(
                footer["raw_sha256"],
                hashlib.sha256(one_point_packet()).hexdigest(),
            )
            self.assertEqual(footer["frames_decoded"], 1)
            self.assertEqual(footer["stop_reason"], "keyboard_interrupt")
            index_report = inspect_uart_chunk_index(raw, raw_index)
            self.assertTrue(index_report["healthy"], index_report["errors"])
            entries = list(iter_mission_log(mission))
            self.assertEqual(len(entries), 2)
            self.assertEqual(type(entries[0].record).__name__, "RadarFrame")
            self.assertEqual(type(entries[1].record).__name__, "SensorHealth")
            self.assertEqual(
                entries[0].record.header.timestamp_source,
                "uart_read_midpoint",
            )
            self.assertGreater(
                entries[0].record.header.timestamp_uncertainty_ns,
                0,
            )
            self.assertEqual(entries[0].record.frame_transition, "first")
            self.assertEqual(
                entries[0].record.profile_id,
                "sdk5502-test-profile",
            )
            self.assertEqual(
                entries[0].record.header.calibration_id,
                "uncalibrated",
            )
            self.assertGreater(stats.max_timing_quality_metric_ns, 0)

    def test_frame_gap_handles_normal_sequence_gap_and_wrap(self):
        self.assertEqual(frame_gap(None, 10), 0)
        self.assertEqual(frame_gap(10, 11), 0)
        self.assertEqual(frame_gap(10, 14), 3)
        self.assertEqual(frame_gap(0xFFFFFFFF, 0), 0)
        self.assertEqual(frame_gap(0xFFFFFFFE, 1), 2)

    def test_frame_reset_or_out_of_order_is_not_reported_as_billions_lost(self):
        self.assertEqual(frame_gap(1000, 1), 0)
        self.assertEqual(frame_gap(5, 5), 0)
        self.assertEqual(
            classify_frame_transition(1000, 1),
            (0, "reset_or_out_of_order"),
        )
        self.assertEqual(
            classify_frame_transition(5, 5),
            (0, "duplicate"),
        )
        self.assertEqual(
            classify_frame_transition(0xFFFFFFFF, 0),
            (0, "wrap"),
        )

    def test_device_reset_starts_a_new_radar_producer_epoch(self):
        fake_serial_module = SimpleNamespace(Serial=ResettingFakeSerial)
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=115200,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="sdk5502-test-profile",
                boot_id="boot-1",
            )

            entries = list(iter_mission_log(mission))
            radar_records = [entry.record for entry in entries[:-1]]
            self.assertEqual(stats.frames_decoded, 2)
            self.assertEqual(stats.device_discontinuities, 1)
            self.assertEqual(
                [record.frame_transition for record in radar_records],
                ["first", "reset_or_out_of_order"],
            )
            self.assertNotEqual(
                radar_records[0].header.producer_id,
                radar_records[1].header.producer_id,
            )
            self.assertEqual(
                [record.header.seq for record in radar_records],
                [1, 1],
            )
            health = entries[-1].record
            self.assertEqual(health.status, "degraded")
            self.assertEqual(health.device_discontinuities_total, 1)

    def test_live_parser_diagnostics_are_emitted_before_capture_end(self):
        fake_serial_module = SimpleNamespace(
            Serial=PeriodicDiagnosticFakeSerial
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            capture_radar_uart(
                port="COM_TEST",
                baudrate=115200,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="sdk5502-test-profile",
                boot_id="boot-1",
                health_interval_s=0.01,
            )

            health_records = [
                entry.record
                for entry in iter_mission_log(mission)
                if type(entry.record).__name__ == "SensorHealth"
            ]
            self.assertGreaterEqual(len(health_records), 2)
            self.assertEqual(health_records[0].status, "degraded")
            self.assertIn(
                "health_kind=periodic",
                health_records[0].detail,
            )

    def test_raw_and_json_log_must_not_share_a_path(self):
        with self.assertRaisesRegex(ValueError, "different files"):
            capture_radar_uart(
                port="COM_TEST",
                baudrate=115200,
                mission_log=Path("same.bin"),
                mission_id="mission-1",
                raw_capture=Path("same.bin"),
            )

    def test_uart_observation_time_reports_heuristic_timing_scale(self):
        midpoint_ns, timing_scale_ns = estimate_uart_observation_time(
            read_started_ns=1_000_000_000,
            read_finished_ns=1_004_000_000,
            chunk_bytes=100,
            baudrate=100_000,
            serial_timeout_s=0.005,
        )
        self.assertEqual(midpoint_ns, 1_002_000_000)
        # The 10 ms line-time term dominates this heuristic, but does not
        # include hidden USB/XDS110 buffering.
        self.assertEqual(timing_scale_ns, 10_000_000)

    def test_explicit_raw_index_requires_raw_capture(self):
        with self.assertRaisesRegex(ValueError, "requires raw_capture"):
            capture_radar_uart(
                port="COM_TEST",
                baudrate=115200,
                mission_log=Path("mission.jsonl"),
                mission_id="mission-1",
                raw_index=Path("timing.jsonl"),
            )

    def test_invalid_profile_id_is_rejected_before_capture(self):
        with self.assertRaisesRegex(ValueError, "profile_id is invalid"):
            capture_radar_uart(
                port="COM_TEST",
                baudrate=115200,
                mission_log=Path("mission.jsonl"),
                mission_id="mission-1",
                profile_id="contains spaces",
            )


if __name__ == "__main__":
    unittest.main()
