import json
import hashlib
import tempfile
from pathlib import Path
import unittest

from common.sensor_json import canonical_json_bytes
from sensors.raw_capture_index import (
    MAX_UART_CHUNK_INDEX_LINE_BYTES,
    inspect_uart_chunk_index,
)


METADATA = {
    "mission_id": "mission-1",
    "unit_id": "head",
    "boot_id": "boot-1",
    "producer_id": "radar-reader-1",
    "profile_id": "sdk5502-profile",
    "calibration_id": "uncalibrated",
    "baudrate": 115200,
}


def chunk_record(
    chunk_seq,
    byte_offset,
    byte_length,
    read_started_ns,
    read_finished_ns,
    **metadata_overrides,
):
    metadata = dict(METADATA)
    metadata.update(metadata_overrides)
    return {
        "index_version": 1,
        "record_type": "uart_chunk",
        **metadata,
        "chunk_seq": chunk_seq,
        "byte_offset": byte_offset,
        "byte_length": byte_length,
        "read_started_ns": read_started_ns,
        "read_finished_ns": read_finished_ns,
        "observation_midpoint_ns": (
            read_started_ns + read_finished_ns
        )
        // 2,
        "timing_quality_metric_ns": 10_000_000,
    }


def footer(chunks, raw_data, **metadata_overrides):
    metadata = dict(METADATA)
    metadata.update(metadata_overrides)
    return {
        "index_version": 1,
        "record_type": "capture_end",
        **metadata,
        "chunks": chunks,
        "raw_bytes": len(raw_data),
        "raw_sha256": hashlib.sha256(raw_data).hexdigest(),
        "frames_decoded": chunks,
        "ended_monotonic_ns": 1_000,
        "stop_reason": "test_complete",
    }


def encode_lines(*records):
    return b"".join(
        canonical_json_bytes(record) + b"\n" for record in records
    )


class RawCaptureIndexTests(unittest.TestCase):
    def test_healthy_index_matches_raw_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"abcdef")
            index.write_bytes(
                encode_lines(
                    chunk_record(1, 0, 2, 100, 110),
                    chunk_record(2, 2, 4, 120, 140),
                    footer(2, b"abcdef"),
                )
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertTrue(report["healthy"], report["errors"])
            self.assertEqual(report["records"], 3)
            self.assertEqual(report["chunks"], 2)
            self.assertEqual(report["indexed_raw_bytes"], 6)
            self.assertEqual(report["actual_raw_bytes"], 6)
            self.assertTrue(report["footer_present"])
            self.assertEqual(report["profile_id"], "sdk5502-profile")
            self.assertEqual(report["baudrate"], 115200)
            json.dumps(report)

    def test_missing_footer_is_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"ab")
            index.write_bytes(
                encode_lines(chunk_record(1, 0, 2, 100, 110))
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertFalse(report["healthy"])
            self.assertTrue(
                any("missing capture_end" in error for error in report["errors"])
            )

    def test_offset_and_metadata_changes_are_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"abcdef")
            index.write_bytes(
                encode_lines(
                    chunk_record(1, 0, 2, 100, 110),
                    chunk_record(
                        2,
                        3,
                        4,
                        120,
                        140,
                        profile_id="different-profile",
                    ),
                    footer(2, b"abcdef"),
                )
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertFalse(report["healthy"])
            self.assertTrue(
                any(
                    "expected byte_offset 2" in error
                    for error in report["errors"]
                )
            )
            self.assertTrue(
                any("metadata changed" in error for error in report["errors"])
            )

    def test_overlapping_read_times_are_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"abcd")
            index.write_bytes(
                encode_lines(
                    chunk_record(1, 0, 2, 100, 120),
                    chunk_record(2, 2, 2, 110, 130),
                    footer(2, b"abcd"),
                )
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertFalse(report["healthy"])
            self.assertTrue(
                any("regressed or overlapped" in error for error in report["errors"])
            )

    def test_truncated_malformed_final_line_returns_unhealthy_report(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"ab")
            index.write_bytes(
                encode_lines(chunk_record(1, 0, 2, 100, 110))
                + b'{"index_version":1'
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertFalse(report["healthy"])
            self.assertTrue(
                any("incomplete final line" in error for error in report["errors"])
            )
            self.assertTrue(
                any("invalid JSON record" in error for error in report["errors"])
            )

    def test_same_size_raw_content_change_fails_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"UVWXYZ")
            index.write_bytes(
                encode_lines(
                    chunk_record(1, 0, 6, 100, 110),
                    footer(1, b"abcdef"),
                )
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertFalse(report["healthy"])
            self.assertTrue(
                any("SHA-256" in error for error in report["errors"])
            )

    def test_footer_must_be_the_only_last_record(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"ab")
            index.write_bytes(
                encode_lines(
                    chunk_record(1, 0, 2, 100, 110),
                    footer(1, b"ab"),
                    footer(1, b"ab"),
                )
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertFalse(report["healthy"])
            self.assertEqual(report["footer_count"], 2)
            self.assertTrue(
                any("record appears after capture_end" in error for error in report["errors"])
            )

    def test_overlong_line_is_rejected_without_unbounded_read(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "radar.bin"
            index = root / "radar.bin.chunks.jsonl"
            raw.write_bytes(b"")
            index.write_bytes(
                b"{" + b"x" * MAX_UART_CHUNK_INDEX_LINE_BYTES + b"}\n"
            )

            report = inspect_uart_chunk_index(raw, index)

            self.assertFalse(report["healthy"])
            self.assertTrue(
                any("line exceeds" in error for error in report["errors"])
            )


if __name__ == "__main__":
    unittest.main()
