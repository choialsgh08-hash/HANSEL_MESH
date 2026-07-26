import math
import struct
import unittest

from common.sensor_contract import SensorHeader
from sensors.ti_mmwave import (
    DOCUMENTED_HEADER_SIZE,
    TI_MAGIC_WORD,
    TiMmwavePacketParser,
    TiMmwaveParseError,
    TiMmwaveStreamDecoder,
)


def packet(tlvs, num_points, header_size=40, frame=1, padding=True):
    body = bytearray(b"\x00" * (header_size - 40))
    for tlv_type, payload in tlvs:
        body.extend(struct.pack("<II", tlv_type, len(payload)))
        body.extend(payload)
    total_without_padding = 40 + len(body)
    pad_count = (-total_without_padding) % 32 if padding else 0
    total = total_without_padding + pad_count
    header = TI_MAGIC_WORD + struct.pack(
        "<8I",
        0x05050002,
        total,
        0xA6432,
        frame,
        123456,
        num_points,
        len(tlvs),
        0,
    )
    return header + bytes(body) + b"\x00" * pad_count


class TiMmwaveTests(unittest.TestCase):
    def test_float_points_and_side_info_with_40_byte_header(self):
        points = struct.pack(
            "<8f",
            1.0,
            2.0,
            3.0,
            -0.5,
            4.0,
            5.0,
            6.0,
            0.25,
        )
        side = struct.pack("<4h", 120, 30, 200, 40)
        parsed = TiMmwavePacketParser().parse_packet(
            packet([(1, points), (7, side)], 2, header_size=40)
        )
        self.assertEqual(parsed.header_size, 40)
        self.assertEqual(parsed.point_format, "float")
        self.assertTrue(parsed.complete)
        self.assertEqual(len(parsed.points), 2)
        self.assertEqual(parsed.points[0].snr_db, 12.0)
        self.assertEqual(parsed.points[1].noise_db, 4.0)
        self.assertAlmostEqual(parsed.points[0].radial_velocity_mps, -0.5)

    def test_compressed_points_with_documented_52_byte_header(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 2, 0)
        points = struct.pack("<hhhhBB", 100, -20, 3, -4, 10, 2)
        points += struct.pack("<hhhhBB", -50, 40, 0, 5, 20, 3)
        parsed = TiMmwavePacketParser(
            header_size=DOCUMENTED_HEADER_SIZE
        ).parse_packet(
            packet(
                [(301, units + points)],
                2,
                header_size=DOCUMENTED_HEADER_SIZE,
            )
        )
        self.assertEqual(parsed.header_size, 52)
        self.assertEqual(parsed.point_format, "compressed")
        self.assertEqual(len(parsed.points), 2)
        self.assertAlmostEqual(parsed.points[0].x_m, 1.0, places=6)
        self.assertAlmostEqual(parsed.points[0].y_m, -0.2, places=6)
        self.assertAlmostEqual(parsed.points[0].radial_velocity_mps, -0.4, places=6)
        self.assertAlmostEqual(parsed.points[1].snr_db, 10.0, places=6)

    def test_zero_point_frame_is_valid(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        parsed = TiMmwavePacketParser(header_size=40).parse_packet(
            packet([(301, units)], 0, header_size=40)
        )
        self.assertTrue(parsed.complete)
        self.assertEqual(parsed.points, ())

    def test_point_count_mismatch_is_visible_not_silently_changed(self):
        points = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
        parsed = TiMmwavePacketParser(header_size=40).parse_packet(
            packet([(1, points)], 2, header_size=40)
        )
        self.assertFalse(parsed.complete)
        self.assertIn("point_count_mismatch", parsed.warnings[0])

    def test_nan_point_is_rejected(self):
        points = struct.pack("<4f", math.nan, 2.0, 3.0, 4.0)
        with self.assertRaises(TiMmwaveParseError):
            TiMmwavePacketParser(header_size=40).parse_packet(
                packet([(1, points)], 1, header_size=40)
            )

    def test_stream_decoder_handles_noise_and_split_chunks(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 1, 0)
        point_data = struct.pack("<hhhhBB", 1, 2, 3, 4, 5, 6)
        raw = packet([(301, units + point_data)], 1, frame=42)
        decoder = TiMmwaveStreamDecoder()
        self.assertEqual(
            decoder.feed(
                b"noise" + raw[:17],
                receipt_monotonic_ns=1_000,
            ),
            [],
        )
        frames = decoder.feed(raw[17:], receipt_monotonic_ns=2_000)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].header.frame_number, 42)
        self.assertEqual(decoder.discarded_bytes, 5)
        self.assertEqual(decoder.buffered_bytes, 0)
        self.assertEqual(frames[0].host_receipt_monotonic_ns, 1_000)
        self.assertEqual(frames[0].host_receipt_uncertainty_ns, 0)

    def test_receipt_time_follows_magic_byte_after_old_noise(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        raw = packet([(301, units)], 0)
        decoder = TiMmwaveStreamDecoder()
        decoder.feed(b"old-noise", receipt_monotonic_ns=10)
        frames = decoder.feed(raw, receipt_monotonic_ns=20)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].host_receipt_monotonic_ns, 20)

    def test_false_magic_with_plausible_long_length_does_not_stall_resync(self):
        false_header = TI_MAGIC_WORD + struct.pack(
            "<8I",
            0,
            1_000_000,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        valid = packet([(301, units)], 0, frame=9)
        decoder = TiMmwaveStreamDecoder()
        frames = decoder.feed(
            false_header + valid,
            receipt_monotonic_ns=30,
        )
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].header.frame_number, 9)
        self.assertEqual(decoder.parse_errors, 1)

    def test_multiple_frames_in_one_read_keep_uncertainty_visible(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        first = packet([(301, units)], 0, frame=1)
        second = packet([(301, units)], 0, frame=2)
        decoder = TiMmwaveStreamDecoder()
        frames = decoder.feed(
            first + second,
            receipt_monotonic_ns=500,
            receipt_uncertainty_ns=80,
        )
        self.assertEqual([frame.header.frame_number for frame in frames], [1, 2])
        self.assertEqual(
            [frame.host_receipt_monotonic_ns for frame in frames],
            [500, 500],
        )
        self.assertEqual(
            [frame.host_receipt_uncertainty_ns for frame in frames],
            [80, 80],
        )

    def test_unknown_tlv_is_preserved_as_metadata(self):
        parsed = TiMmwavePacketParser(header_size=40).parse_packet(
            packet([(999, b"abcd")], 0, header_size=40)
        )
        self.assertEqual(parsed.unknown_tlvs, ((999, 4),))
        self.assertTrue(parsed.complete)

    def test_conversion_keeps_native_frame_explicit(self):
        points = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
        parsed = TiMmwavePacketParser(header_size=40).parse_packet(
            packet([(1, points)], 1, header_size=40)
        )
        header = SensorHeader(
            mission_id="mission-1",
            unit_id="head",
            boot_id="boot-1",
            producer_id="radar-reader-1",
            stream_id="radar/front",
            seq=1,
            monotonic_ns=123,
            frame_id="radar_native",
        )
        record = parsed.to_sensor_record(header)
        self.assertEqual(record.header.frame_id, "radar_native")
        self.assertEqual(record.points[0].x_m, 1.0)
        self.assertEqual(record.sdk_version, "5.5.0.2")
        self.assertEqual(record.device_time_cycles, 123456)


if __name__ == "__main__":
    unittest.main()
