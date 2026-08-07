from __future__ import annotations

import struct
import unittest

import _paths  # noqa: F401
from hansel_camera_bridge.rtp_metrics import RtpMetricsTracker, parse_rtp_header


def packet(sequence: int, timestamp: int, marker: bool = True, size: int = 100) -> bytes:
    second = 96 | (0x80 if marker else 0)
    header = struct.pack("!BBHII", 0x80, second, sequence, timestamp, 1234)
    return header + bytes(max(0, size - len(header)))


class RtpMetricsTests(unittest.TestCase):
    def test_parse_standard_rtp_header(self) -> None:
        parsed = parse_rtp_header(packet(42, 9000))
        self.assertEqual(parsed.sequence, 42)
        self.assertEqual(parsed.timestamp, 9000)
        self.assertTrue(parsed.marker)
        self.assertEqual(parsed.header_length, 12)

    def test_sequence_gap_counts_packet_loss(self) -> None:
        tracker = RtpMetricsTracker()
        tracker.observe(packet(10, 1000), 1.0)
        tracker.observe(packet(13, 2000), 1.1)
        snapshot = tracker.snapshot(2.0)
        self.assertEqual(snapshot.total_packets, 2)
        self.assertEqual(snapshot.lost_packets, 2)
        self.assertAlmostEqual(snapshot.loss_rate, 0.5)

    def test_sequence_wrap_does_not_create_false_loss(self) -> None:
        tracker = RtpMetricsTracker()
        tracker.observe(packet(65535, 1000), 1.0)
        tracker.observe(packet(0, 2000), 1.1)
        snapshot = tracker.snapshot(2.0)
        self.assertEqual(snapshot.lost_packets, 0)

    def test_receive_timeout_and_fps_are_operator_side(self) -> None:
        tracker = RtpMetricsTracker(receive_timeout_s=0.5)
        tracker.observe(packet(1, 1000), 1.0)
        tracker.observe(packet(2, 2000), 1.25)
        live = tracker.snapshot(1.5)
        self.assertTrue(live.receiving)
        self.assertAlmostEqual(live.receive_fps, 4.0)
        stale = tracker.snapshot(2.0)
        self.assertFalse(stale.receiving)


if __name__ == "__main__":
    unittest.main()
