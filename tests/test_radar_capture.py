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
from monitor.radar_front import RadarFrontState
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


def one_point_heatmap_packet(frame_number=1, tlv_type=304):
    units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 1, 0)
    point = struct.pack("<hhhhBB", 10, 20, 0, -2, 8, 3)
    point_payload = units + point
    heatmap_payload = struct.pack(
        "<8I",
        0,
        1,
        10,
        100,
        1_000,
        10_000,
        100_000,
        1_000_000,
    )
    body = (
        struct.pack("<II", 301, len(point_payload))
        + point_payload
        + struct.pack("<II", tlv_type, len(heatmap_payload))
        + heatmap_payload
    )
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
        2,
        0,
    )
    return header + body + padding


def elided_empty_point_packet(frame_number=1):
    payload = b"\x01\x02\x03\x04"
    body = struct.pack("<II", 302, len(payload)) + payload
    padding = b"\x00" * ((-(40 + len(body))) % 32)
    total = 40 + len(body) + len(padding)
    header = TI_MAGIC_WORD + struct.pack(
        "<8I",
        0x05050402,
        total,
        0xA6432,
        frame_number,
        0,
        0,
        1,
        0,
    )
    return header + body + padding


def nonzero_padding_packet(frame_number=1):
    raw = bytearray(one_point_packet(frame_number=frame_number))
    raw[-1] = 0xA5
    return bytes(raw)


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


class ElidedEmptyPointFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reads = [elided_empty_point_packet()]


class MidStreamAttachFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reads = [
            one_point_packet(frame_number=1)[17:],
            one_point_packet(frame_number=2),
        ]


class DelayedMidStreamAttachFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.read_count = 0

    def read(self, size):
        self.read_count += 1
        if self.read_count == 1:
            return one_point_packet(frame_number=1)[17:]
        if self.read_count == 2:
            time.sleep(0.02)
            return b""
        if self.read_count == 3:
            return one_point_packet(frame_number=2)
        raise KeyboardInterrupt


class PostSyncCorruptionFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reads = [
            one_point_packet(frame_number=1),
            b"corruption" + one_point_packet(frame_number=2),
        ]


class NonzeroPaddingFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reads = [nonzero_padding_packet()]


class HeatmapFakeSerial(FakeSerial):
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reads = [one_point_heatmap_packet()]


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
            self.assertEqual(stats.empty_point_frames, 0)
            self.assertEqual(stats.nonzero_padding_frames, 0)
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

    def test_live_capture_persists_heatmap_and_health_counters(self):
        fake_serial_module = SimpleNamespace(Serial=HeatmapFakeSerial)
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            root = Path(directory)
            mission = root / "mission.jsonl"
            raw = root / "radar.bin"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="sdk5504-heatmap-test",
                boot_id="boot-1",
                raw_capture=raw,
                heatmap_azimuth_bins=4,
                heatmap_range_bins=2,
                heatmap_range_step_m=0.05,
            )

            self.assertEqual(stats.heatmap_frames, 1)
            self.assertEqual(stats.major_heatmap_frames, 1)
            self.assertEqual(stats.minor_heatmap_frames, 0)
            self.assertEqual(stats.missing_heatmap_frames, 0)
            self.assertEqual(stats.heatmap_cells_decoded, 8)
            self.assertEqual(stats.heatmap_azimuth_bins, 4)
            self.assertEqual(stats.heatmap_range_bins, 2)
            self.assertEqual(stats.heatmap_range_step_m, 0.05)
            records = [entry.record for entry in iter_mission_log(mission)]
            self.assertEqual(records[0].heatmap.range_bins, 2)
            self.assertEqual(records[0].heatmap.azimuth_bins, 4)
            self.assertEqual(records[0].heatmap.range_step_m, 0.05)
            self.assertEqual(records[-1].status, "ok")
            self.assertIn("heatmap_frames=1", records[-1].detail)
            self.assertIn("heatmap_cells_decoded=8", records[-1].detail)
            self.assertIn("heatmap_expected=true", records[-1].detail)
            self.assertIn("heatmap_azimuth_bins=4", records[-1].detail)
            self.assertIn("heatmap_range_bins=2", records[-1].detail)
            self.assertIn("heatmap_range_step_m=0.05", records[-1].detail)

            index_lines = Path(stats.raw_index).read_bytes().splitlines()
            index_record = strict_json_loads(index_lines[0])
            footer = strict_json_loads(index_lines[1])
            self.assertEqual(index_record["heatmap_azimuth_bins"], 4)
            self.assertEqual(index_record["heatmap_range_bins"], 2)
            self.assertEqual(index_record["heatmap_range_step_m"], 0.05)
            self.assertEqual(footer["heatmap_azimuth_bins"], 4)
            self.assertEqual(footer["heatmap_range_bins"], 2)
            self.assertEqual(footer["heatmap_range_step_m"], 0.05)

    def test_requested_but_missing_heatmap_degrades_health(self):
        fake_serial_module = SimpleNamespace(Serial=FakeSerial)
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="sdk5504-heatmap-test",
                boot_id="boot-1",
                heatmap_azimuth_bins=4,
                heatmap_range_bins=2,
                heatmap_range_step_m=0.05,
            )

            health = list(iter_mission_log(mission))[-1].record
            self.assertEqual(stats.heatmap_frames, 0)
            self.assertEqual(stats.missing_heatmap_frames, 1)
            self.assertEqual(health.status, "degraded")
            self.assertIn("missing_heatmap_frames=1", health.detail)

    def test_heatmap_capture_configuration_requires_all_dimensions(self):
        with self.assertRaisesRegex(ValueError, "supplied together"):
            capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=Path("mission.jsonl"),
                mission_id="mission-1",
                heatmap_azimuth_bins=4,
                heatmap_range_step_m=0.05,
            )

    def test_official_demo_elided_empty_frame_is_healthy_with_opt_in(self):
        fake_serial_module = SimpleNamespace(
            Serial=ElidedEmptyPointFakeSerial
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="lsdk-05.05.04.02-test",
                boot_id="boot-1",
                allow_elided_empty_point_tlv=True,
            )

            entries = list(iter_mission_log(mission))
            radar = entries[0].record
            health = entries[-1].record
            self.assertEqual(stats.frames_decoded, 1)
            self.assertEqual(stats.point_cloud_frames, 1)
            self.assertEqual(stats.empty_point_frames, 1)
            self.assertEqual(stats.missing_point_tlv_frames, 0)
            self.assertEqual(radar.source_format, "ti-mmwave-empty")
            self.assertEqual(radar.points, ())
            self.assertEqual(health.status, "ok")

    def test_mid_stream_attach_is_recorded_but_not_permanently_degraded(self):
        fake_serial_module = SimpleNamespace(Serial=MidStreamAttachFakeSerial)
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="lsdk-05.05.04.02-test",
                boot_id="boot-1",
            )

            health = list(iter_mission_log(mission))[-1].record
            self.assertEqual(stats.frames_decoded, 1)
            self.assertGreater(stats.parser_discarded_bytes, 0)
            self.assertEqual(
                stats.startup_sync_discarded_bytes,
                stats.parser_discarded_bytes,
            )
            self.assertEqual(stats.post_sync_discarded_bytes, 0)
            self.assertEqual(stats.post_sync_parse_errors, 0)
            self.assertEqual(health.status, "ok")
            self.assertEqual(health.parse_errors_total, 0)
            self.assertIn(
                "startup_sync_discarded_bytes=",
                health.detail,
            )

    def test_periodic_startup_sync_does_not_latch_viewer_degraded(self):
        fake_serial_module = SimpleNamespace(
            Serial=DelayedMidStreamAttachFakeSerial
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="lsdk-05.05.04.02-test",
                boot_id="boot-1",
                health_interval_s=0.01,
            )

            records = [entry.record for entry in iter_mission_log(mission)]
            health_records = [
                record
                for record in records
                if type(record).__name__ == "SensorHealth"
            ]
            self.assertEqual(health_records[0].status, "starting")
            self.assertEqual(health_records[0].parse_errors_total, 0)
            self.assertEqual(health_records[-1].status, "ok")
            self.assertEqual(stats.post_sync_discarded_bytes, 0)

            state = RadarFrontState("follow")
            for record in records:
                state.ingest(record)
            self.assertEqual(state.snapshot()["status"], "live")

    def test_post_sync_corruption_degrades_live_health(self):
        fake_serial_module = SimpleNamespace(
            Serial=PostSyncCorruptionFakeSerial
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="lsdk-05.05.04.02-test",
                boot_id="boot-1",
            )

            health = list(iter_mission_log(mission))[-1].record
            self.assertGreater(stats.post_sync_discarded_bytes, 0)
            self.assertEqual(health.status, "degraded")

    def test_nonzero_padding_compatibility_is_preserved_in_diagnostics(self):
        fake_serial_module = SimpleNamespace(Serial=NonzeroPaddingFakeSerial)
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            sys.modules,
            {"serial": fake_serial_module},
        ):
            mission = Path(directory) / "mission.jsonl"
            stats = capture_radar_uart(
                port="COM_TEST",
                baudrate=1_250_000,
                mission_log=mission,
                mission_id="mission-1",
                profile_id="lsdk-05.05.04.02-test",
                boot_id="boot-1",
                allow_nonzero_padding=True,
            )

            health = list(iter_mission_log(mission))[-1].record
            self.assertEqual(stats.nonzero_padding_frames, 1)
            self.assertEqual(health.status, "ok")
            self.assertIn("nonzero_padding_frames=1", health.detail)

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
            self.assertEqual(health_records[0].status, "starting")
            self.assertEqual(health_records[-1].status, "degraded")
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
