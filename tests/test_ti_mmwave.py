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

    def test_range_azimuth_heatmap_is_explicit_opt_in_and_quantized(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        raw_values = (0, 1, 10, 100, 1_000, 10_000, 100_000, 1_000_000)
        heatmap_payload = struct.pack("<8I", *raw_values)
        raw = packet(
            [(301, units), (304, heatmap_payload)],
            0,
            header_size=40,
        )

        legacy = TiMmwavePacketParser(header_size=40).parse_packet(raw)
        parsed = TiMmwavePacketParser(
            header_size=40,
            heatmap_azimuth_bins=4,
            heatmap_range_bins=2,
            heatmap_range_step_m=0.05,
        ).parse_packet(raw)

        self.assertIsNone(legacy.heatmap)
        self.assertIn((304, len(heatmap_payload)), legacy.unknown_tlvs)
        self.assertIsNotNone(parsed.heatmap)
        self.assertEqual(parsed.heatmap.range_bins, 2)
        self.assertEqual(parsed.heatmap.azimuth_bins, 4)
        self.assertEqual(parsed.heatmap.motion_mode, "major")
        self.assertEqual(parsed.heatmap.tlv_type, 304)
        self.assertEqual(parsed.heatmap.range_step_m, 0.05)
        self.assertEqual(len(parsed.heatmap.data), len(raw_values))
        self.assertEqual(parsed.heatmap.data[0], 0)
        self.assertEqual(max(parsed.heatmap.data), 255)
        self.assertAlmostEqual(parsed.heatmap.floor_db, 1.2)
        self.assertAlmostEqual(parsed.heatmap.ceiling_db, 118.8)

        sensor_record = parsed.to_sensor_record(
            SensorHeader(
                mission_id="mission-1",
                unit_id="head",
                boot_id="boot-1",
                producer_id="producer-1",
                stream_id="radar/front",
                seq=1,
                monotonic_ns=1,
            )
        )
        self.assertEqual(sensor_record.heatmap.data, parsed.heatmap.data)
        self.assertEqual(sensor_record.heatmap.motion_mode, "major")

    def test_minor_heatmap_mode_and_zero_frame_are_supported(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        parsed = TiMmwavePacketParser(
            heatmap_azimuth_bins=2,
            heatmap_range_bins=2,
            heatmap_range_step_m=0.1,
        ).parse_packet(
            packet(
                [(301, units), (305, struct.pack("<4I", 0, 0, 0, 0))],
                0,
            )
        )

        self.assertEqual(parsed.heatmap.motion_mode, "minor")
        self.assertEqual(parsed.heatmap.data, b"\x00" * 4)
        self.assertEqual(parsed.heatmap.floor_db, 0.0)
        self.assertEqual(parsed.heatmap.ceiling_db, 1.0)

    def test_heatmap_configuration_and_shape_are_strict(self):
        with self.assertRaisesRegex(ValueError, "supplied together"):
            TiMmwavePacketParser(heatmap_azimuth_bins=4)
        with self.assertRaisesRegex(ValueError, "positive"):
            TiMmwavePacketParser(
                heatmap_azimuth_bins=4,
                heatmap_range_bins=2,
                heatmap_range_step_m=0.0,
            )
        with self.assertRaisesRegex(ValueError, "range_bins.*positive"):
            TiMmwavePacketParser(
                heatmap_azimuth_bins=4,
                heatmap_range_bins=0,
                heatmap_range_step_m=0.05,
            )
        with self.assertRaisesRegex(TiMmwaveParseError, "configured shape"):
            TiMmwavePacketParser(
                heatmap_azimuth_bins=2,
                heatmap_range_bins=2,
                heatmap_range_step_m=0.05,
            ).parse_packet(
                packet([(304, struct.pack("<8I", *range(8)))], 0)
            )
        with self.assertRaisesRegex(TiMmwaveParseError, "multiple"):
            TiMmwavePacketParser(
                heatmap_azimuth_bins=2,
                heatmap_range_bins=1,
                heatmap_range_step_m=0.05,
            ).parse_packet(
                packet(
                    [
                        (304, struct.pack("<2I", 1, 2)),
                        (305, struct.pack("<2I", 3, 4)),
                    ],
                    0,
                )
            )

    def test_official_demo_elided_empty_point_tlv_is_explicit_opt_in(self):
        raw = packet([(302, b"\x01\x02\x03\x04")], 0, header_size=40)
        strict = TiMmwavePacketParser(header_size=40).parse_packet(raw)
        compatible = TiMmwavePacketParser(
            header_size=40,
            allow_elided_empty_point_tlv=True,
        ).parse_packet(raw)

        self.assertEqual(strict.point_format, "none")
        self.assertEqual(compatible.point_format, "empty")
        self.assertTrue(compatible.complete)
        self.assertEqual(compatible.points, ())
        self.assertIn("empty_point_tlv_elided", compatible.warnings)

    def test_elided_point_tlv_is_not_accepted_for_nonzero_header_count(self):
        parsed = TiMmwavePacketParser(
            header_size=40,
            allow_elided_empty_point_tlv=True,
        ).parse_packet(
            packet([(302, b"\x01\x02\x03\x04")], 1, header_size=40)
        )

        self.assertEqual(parsed.point_format, "none")
        self.assertFalse(parsed.complete)
        self.assertIn("point_count_mismatch", parsed.warnings[0])

    def test_elided_empty_rejects_side_info_only_and_header_only_packets(self):
        side_info = struct.pack("<2h", 10, 20)
        parser = TiMmwavePacketParser(
            header_size=40,
            allow_elided_empty_point_tlv=True,
        )

        side_only = parser.parse_packet(
            packet([(7, side_info)], 0, header_size=40)
        )
        header_only = parser.parse_packet(
            packet([], 0, header_size=40)
        )

        self.assertEqual(side_only.point_format, "none")
        self.assertEqual(header_only.point_format, "none")

    def test_nonzero_padding_requires_guarded_opt_in(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        raw = bytearray(packet([(301, units)], 0, header_size=40))
        raw[-1] = 0xA5

        with self.assertRaisesRegex(
            TiMmwaveParseError,
            "non-zero bytes",
        ):
            TiMmwavePacketParser(header_size=40).parse_packet(bytes(raw))

        parsed = TiMmwavePacketParser(
            header_size=40,
            allow_nonzero_padding=True,
        ).parse_packet(bytes(raw))
        self.assertTrue(parsed.complete)
        self.assertIn("nonzero_padding:", parsed.warnings[0])

    def test_nonzero_padding_opt_in_still_requires_32_byte_alignment(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        raw = bytearray(packet([(301, units)], 0, header_size=40))
        raw[-2] = 0xA5
        del raw[-1]
        struct.pack_into("<I", raw, 12, len(raw))

        with self.assertRaisesRegex(
            TiMmwaveParseError,
            "valid 32-byte padding",
        ):
            TiMmwavePacketParser(
                header_size=40,
                allow_nonzero_padding=True,
            ).parse_packet(bytes(raw))

    def test_nonzero_padding_cannot_hide_an_undeclared_tlv(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        raw = bytearray(
            packet(
                [(301, units), (306, b"\x11" * 20)],
                0,
                header_size=40,
            )
        )
        struct.pack_into("<I", raw, 32, 1)
        struct.pack_into("<I", raw, 72, 100)

        with self.assertRaisesRegex(
            TiMmwaveParseError,
            "plausible undeclared TLV",
        ):
            TiMmwavePacketParser(
                header_size=40,
                allow_nonzero_padding=True,
            ).parse_packet(bytes(raw))

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
        self.assertTrue(decoder.synchronized)
        self.assertEqual(decoder.startup_sync_discarded_bytes, 5)
        self.assertEqual(decoder.post_sync_discarded_bytes, 0)
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
        self.assertEqual(decoder.startup_sync_parse_errors, 1)
        self.assertEqual(decoder.post_sync_parse_errors, 0)

    def test_corruption_after_first_valid_frame_remains_visible(self):
        units = struct.pack("<4f2H", 0.01, 0.1, 0.5, 1.0, 0, 0)
        first = packet([(301, units)], 0, frame=1)
        second = packet([(301, units)], 0, frame=2)
        decoder = TiMmwaveStreamDecoder()

        self.assertEqual(len(decoder.feed(first)), 1)
        frames = decoder.feed(b"corruption" + second)

        self.assertEqual(len(frames), 1)
        self.assertEqual(decoder.startup_sync_discarded_bytes, 0)
        self.assertEqual(
            decoder.post_sync_discarded_bytes,
            len(b"corruption"),
        )

    def test_unsynchronized_noise_remains_an_operational_error(self):
        decoder = TiMmwaveStreamDecoder()
        decoder.feed(b"not-a-frame-yet")

        self.assertFalse(decoder.synchronized)
        self.assertGreater(decoder.discarded_bytes, 0)
        self.assertEqual(
            decoder.startup_sync_discarded_bytes,
            decoder.discarded_bytes,
        )
        self.assertEqual(decoder.post_sync_discarded_bytes, 0)

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
