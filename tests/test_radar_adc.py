import math
import struct
import unittest

from common.sensor_contract import RadarAdcSnapshot
from monitor.radar_adc import (
    RadarAdcAngularSurface,
    adc_range_angle_frame,
    adc_range_profile,
    fused_adc_heatmap_measurements,
)


def directional_adc_snapshot(
    azimuth_deg: float,
    *,
    target_bin: int = 4,
    count: int = 256,
) -> RadarAdcSnapshot:
    """Synthesize one EVM BPM chirp pair for a far-field azimuth tone."""

    # xWRL6432 EVM antGeometryCfg, virtual order Tx1/Rx1..3, Tx2/Rx1..3.
    columns = (0, 1, 2, 1, 2, 3)
    compensation = (1, -1, 1, -1, 1, -1)
    # Generate the native EVM phase ramp for the requested map azimuth.
    phase_step = math.pi * math.sin(math.radians(azimuth_deg))
    virtual = []
    for column, coefficient in zip(columns, compensation):
        phase = phase_step * column
        # Undo the design compensation so processing restores the requested
        # virtual-array phase progression.
        virtual.append([
            coefficient * 900.0 * math.cos(
                2.0 * math.pi * target_bin * sample / count + phase
            )
            for sample in range(count)
        ])
    first_chirp = []
    second_chirp = []
    for rx in range(3):
        tx0 = virtual[rx]
        tx1 = virtual[3 + rx]
        first_chirp.append([round(a + b) for a, b in zip(tx0, tx1)])
        second_chirp.append([round(a - b) for a, b in zip(tx0, tx1)])
    values = [
        value
        for channel in first_chirp + second_chirp
        for value in channel
    ]
    return RadarAdcSnapshot(
        data=struct.pack(f"<{len(values)}h", *values),
        samples_per_chirp=count,
        rx_channels=3,
        chirps=2,
        sample_bits=12,
        tlv_type=316,
    )


def combined_directional_adc_snapshot(*azimuths: float) -> RadarAdcSnapshot:
    snapshots = [directional_adc_snapshot(value) for value in azimuths]
    unpacked = [
        struct.unpack(f"<{len(snapshot.data) // 2}h", snapshot.data)
        for snapshot in snapshots
    ]
    values = [
        round(sum(samples) / len(unpacked))
        for samples in zip(*unpacked)
    ]
    return RadarAdcSnapshot(
        data=struct.pack(f"<{len(values)}h", *values),
        samples_per_chirp=256,
        rx_channels=3,
        chirps=2,
        sample_bits=12,
        tlv_type=316,
    )


def weighted_directional_adc_snapshot(
    first_azimuth: float,
    second_azimuth: float,
    second_weight: float,
) -> RadarAdcSnapshot:
    first = directional_adc_snapshot(first_azimuth)
    second = directional_adc_snapshot(second_azimuth)
    first_values = struct.unpack(f"<{len(first.data) // 2}h", first.data)
    second_values = struct.unpack(f"<{len(second.data) // 2}h", second.data)
    values = [
        round((left + second_weight * right) / (1.0 + second_weight))
        for left, right in zip(first_values, second_values)
    ]
    return RadarAdcSnapshot(
        data=struct.pack(f"<{len(values)}h", *values),
        samples_per_chirp=256,
        rx_channels=3,
        chirps=2,
        sample_bits=12,
        tlv_type=316,
    )


class RadarAdcRangeProfileTests(unittest.TestCase):
    def test_bpm_chirp_pair_recovers_range_tone_without_angle_claim(self):
        count = 256
        target_bin = 7
        values = []
        ping_channels = []
        pong_channels = []
        for rx in range(3):
            tx0 = [1000.0 * math.cos(2 * math.pi * target_bin * n / count)
                   for n in range(count)]
            tx1 = [600.0 * math.sin(2 * math.pi * target_bin * n / count)
                   for n in range(count)]
            ping_channels.append([round(a + b) for a, b in zip(tx0, tx1)])
            pong_channels.append([round(a - b) for a, b in zip(tx0, tx1)])
        for channel in ping_channels + pong_channels:
            values.extend(channel)
        snapshot = RadarAdcSnapshot(
            data=struct.pack(f"<{len(values)}h", *values),
            samples_per_chirp=count,
            rx_channels=3,
            chirps=2,
            sample_bits=12,
            tlv_type=316,
        )

        profile = adc_range_profile(snapshot, range_step_m=0.05859375)

        self.assertEqual(profile["source"], "uart_last_chirp_pair_adc")
        self.assertEqual(profile["direction_semantics"], "range_only")
        self.assertEqual(profile["range_bins"], 128)
        self.assertEqual(profile["peak_bin"], target_bin)
        self.assertAlmostEqual(
            profile["peak_range_m"], target_bin * 0.05859375, places=6,
        )
        self.assertEqual(len(profile["values"]), 128)
        self.assertTrue(all(0 <= value <= 255 for value in profile["values"]))

    def test_zero_adc_is_finite_and_does_not_invent_a_peak(self):
        snapshot = RadarAdcSnapshot(
            data=b"\x00\x00" * (256 * 3 * 2),
            samples_per_chirp=256,
            rx_channels=3,
            chirps=2,
            sample_bits=12,
            tlv_type=316,
        )
        profile = adc_range_profile(snapshot, range_step_m=0.05)
        self.assertIsNone(profile["peak_bin"])
        self.assertEqual(profile["values"], [0] * 128)
        self.assertTrue(math.isfinite(profile["floor_db"]))
        self.assertTrue(math.isfinite(profile["ceiling_db"]))


class RadarAdcAngularSurfaceTests(unittest.TestCase):
    def test_complex_virtual_array_recovers_left_and_right_azimuth(self):
        for expected_deg in (-33.0, 36.0):
            with self.subTest(expected_deg=expected_deg):
                frame = adc_range_angle_frame(
                    directional_adc_snapshot(expected_deg),
                    range_step_m=0.05859375,
                )
                peak = next(
                    item for item in frame["peaks"]
                    if item["range_index"] == 4
                )
                self.assertAlmostEqual(
                    peak["azimuth_deg"], expected_deg, delta=3.0,
                )
                self.assertAlmostEqual(peak["range_m"], 0.234375, places=6)
                self.assertEqual(frame["direction_semantics"], "range_azimuth")
                self.assertEqual(frame["phase_calibration"], "evm_design_default")
                self.assertFalse(frame["complete_frame_adc"])

    def test_one_range_row_keeps_two_beam_separated_reflections(self):
        frame = adc_range_angle_frame(
            combined_directional_adc_snapshot(-34.0, 34.0),
            range_step_m=0.05859375,
        )

        peaks = [
            peak for peak in frame["peaks"]
            if peak["range_index"] == 4
        ]
        self.assertEqual(len(peaks), 2)
        self.assertAlmostEqual(peaks[0]["azimuth_deg"], -34.0, delta=6.0)
        self.assertAlmostEqual(peaks[1]["azimuth_deg"], 34.0, delta=6.0)

    def test_sidelobes_within_one_physical_beam_do_not_duplicate_a_target(self):
        frame = adc_range_angle_frame(
            directional_adc_snapshot(-50.0),
            range_step_m=0.05859375,
        )

        peaks = [
            peak for peak in frame["peaks"]
            if peak["range_index"] == 4
        ]
        self.assertEqual(len(peaks), 1)
        self.assertAlmostEqual(peaks[0]["azimuth_deg"], -50.0, delta=3.0)

    def test_below_row_baseline_second_component_is_not_a_peak(self):
        frame = adc_range_angle_frame(
            weighted_directional_adc_snapshot(-34.0, 34.0, 0.2),
            range_step_m=0.05859375,
        )

        peaks = [
            peak for peak in frame["peaks"]
            if peak["range_index"] == 4
        ]
        self.assertEqual(len(peaks), 1)
        self.assertAlmostEqual(peaks[0]["azimuth_deg"], -34.0, delta=3.0)

    def test_zero_adc_does_not_invent_directional_peaks(self):
        snapshot = RadarAdcSnapshot(
            data=b"\x00\x00" * (256 * 3 * 2),
            samples_per_chirp=256,
            rx_channels=3,
            chirps=2,
            sample_bits=12,
            tlv_type=316,
        )
        frame = adc_range_angle_frame(snapshot, range_step_m=0.05859375)
        self.assertEqual(frame["peaks"], [])
        self.assertTrue(all(cell["intensity"] == 0 for cell in frame["cells"]))

    def test_direct_leakage_bins_are_visible_but_never_promoted_as_peaks(self):
        frame = adc_range_angle_frame(
            directional_adc_snapshot(24.0, target_bin=2),
            range_step_m=0.05859375,
        )

        self.assertGreater(max(
            cell["intensity"] for cell in frame["cells"]
            if cell["range_index"] == 2
        ), 0.0)
        self.assertFalse(any(
            peak["range_index"] <= 2 for peak in frame["peaks"]
        ))

    def test_peak_requires_three_of_five_complete_frames(self):
        surface = RadarAdcAngularSurface()
        snapshot = directional_adc_snapshot(27.0)
        for timestamp in (0.0, 0.2):
            surface.ingest(snapshot, range_step_m=0.05859375, received_at=timestamp)
        self.assertFalse(any(
            peak["verified"] for peak in surface.payload(0.2)["peaks"]
        ))

        surface.ingest(snapshot, range_step_m=0.05859375, received_at=0.4)
        peak = next(
            item for item in surface.payload(0.4)["peaks"]
            if item["range_index"] == 4
        )
        self.assertTrue(peak["verified"])
        self.assertEqual(peak["hits"], 3)
        self.assertGreater(peak["confidence"], 0.5)

    def test_small_peak_jitter_counts_as_one_physical_beam(self):
        surface = RadarAdcAngularSurface()
        for timestamp, angle in zip((0.0, 0.2, 0.4), (26.0, 30.0, 34.0)):
            surface.ingest(
                directional_adc_snapshot(angle),
                range_step_m=0.05859375,
                received_at=timestamp,
            )

        peaks = [
            peak for peak in surface.payload(0.4)["peaks"]
            if peak["range_index"] == 4 and peak["verified"]
        ]
        self.assertEqual(len(peaks), 1)
        self.assertEqual(peaks[0]["hits"], 3)
        self.assertAlmostEqual(peaks[0]["azimuth_deg"], 30.0, delta=5.0)

    def test_only_temporally_verified_heatmap_agreement_enters_map(self):
        adc_surface = {
            "range_step_m": 0.05859375,
            "angular_resolution_deg": 29.0,
            "peaks": [{
                "range_index": 4,
                "range_m": 0.234375,
                "azimuth_deg": 30.0,
                "forward_m": 0.202975,
                "lateral_m": 0.117188,
                "relative_db": 7.0,
                "coherence": 0.92,
                "confidence": 0.86,
                "hits": 3,
                "verified": True,
                "angular_uncertainty_deg": 14.5,
            }],
        }
        heatmap_cells = [{
            "range_index": 4,
            "range_m": 0.234375,
            "azimuth_deg": 24.0,
            "forward_m": 0.214,
            "lateral_m": 0.095,
            "relative_db": 4.0,
            "confidence": 0.9,
            "hits": 3,
            "verified": True,
            "range_uncertainty_m": 0.029297,
        }]

        measurements = fused_adc_heatmap_measurements(
            adc_surface, heatmap_cells,
        )

        self.assertEqual(len(measurements), 1)
        self.assertEqual(measurements[0]["source"], "adc_heatmap_fused")
        self.assertGreater(measurements[0]["measurement_probability"], 0.75)
        self.assertAlmostEqual(measurements[0]["azimuth_deg"], 30.0)

        adc_surface["peaks"][0]["verified"] = False
        self.assertEqual(
            fused_adc_heatmap_measurements(adc_surface, heatmap_cells),
            [],
        )

    def test_opposite_side_heatmap_cannot_confirm_adc_peak(self):
        adc_surface = {
            "range_step_m": 0.05859375,
            "angular_resolution_deg": 29.0,
            "peaks": [{
                "range_index": 4, "range_m": 0.234375,
                "azimuth_deg": 30.0, "forward_m": 0.202975,
                "lateral_m": 0.117188, "relative_db": 7.0,
                "coherence": 0.92, "confidence": 0.86,
                "hits": 3, "verified": True,
                "angular_uncertainty_deg": 14.5,
            }],
        }
        heatmap_cells = [{
            "range_index": 4, "range_m": 0.234375,
            "azimuth_deg": -24.0, "relative_db": 5.0,
            "confidence": 0.9, "hits": 3, "verified": True,
            "range_uncertainty_m": 0.029297,
        }]
        self.assertEqual(
            fused_adc_heatmap_measurements(adc_surface, heatmap_cells),
            [],
        )
