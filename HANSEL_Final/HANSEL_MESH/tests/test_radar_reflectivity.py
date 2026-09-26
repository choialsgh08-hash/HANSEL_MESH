import math
import unittest

from common.sensor_contract import RadarHeatmap
from common.radar_geometry import RadarAxes
from monitor.radar_reflectivity import RadarReflectivitySurface
from sensors.radar_calibration import RadarClutterModel


def heatmap(values, *, range_bins=6, azimuth_bins=8):
    return RadarHeatmap(
        data=bytes(values),
        range_bins=range_bins,
        azimuth_bins=azimuth_bins,
        range_step_m=0.05,
        tlv_type=304,
        motion_mode="major",
        floor_db=40.0,
        ceiling_db=100.0,
    )


class RadarReflectivitySurfaceTests(unittest.TestCase):
    def test_calibrated_surface_removes_invalid_ring_and_common_gain_not_data(self):
        range_bins = 6
        azimuth_bins = 8
        cell_count = range_bins * azimuth_bins
        model = RadarClutterModel(
            schema_version=1,
            calibration_id="radar-clutter-reflectivity-test",
            profile_id="reflectivity-test",
            axes=RadarAxes(),
            range_bins=range_bins,
            azimuth_bins=azimuth_bins,
            range_step_m=0.05,
            motion_mode="major",
            point_clusters=(),
            heatmap_median_db=(40.0,) * cell_count,
            heatmap_mad_db=(0.2,) * cell_count,
        )
        # Row 1 is inside the board profile's invalid 7 cm near field.  All
        # valid rows have the same +6 dB receiver drift; row 4 also contains
        # a broad two-bin wall lobe.  The wall must survive common-mode
        # compensation while the drift and invalid ring do not become walls.
        values = [0] * cell_count
        values[1 * azimuth_bins:(1 + 1) * azimuth_bins] = [255] * azimuth_bins
        for range_index in range(2, range_bins):
            values[
                range_index * azimuth_bins:(range_index + 1) * azimuth_bins
            ] = [26] * azimuth_bins
        values[4 * azimuth_bins + 5] = 77
        values[4 * azimuth_bins + 6] = 77
        surface = RadarReflectivitySurface(
            radius_m=0.30,
            clutter_model=model,
            valid_min_range_m=0.07,
        )

        for frame_index in range(3):
            surface.ingest_heatmap(
                heatmap(values), 1.0 + frame_index * 0.1, lateral_sign=1
            )
        payload = surface.payload(1.2)

        self.assertEqual(
            payload["normalization"],
            "current_range_common_mode_physical_beam",
        )
        self.assertFalse(payload["room_baseline_used"])
        self.assertEqual(payload["valid_min_range_m"], 0.07)
        self.assertTrue(payload["calibrated"])
        self.assertEqual(payload["independent_beam_count"], 5)
        self.assertTrue(payload["cells"])
        self.assertTrue(all(cell["range_index"] >= 2 for cell in payload["cells"]))
        wall = [
            cell for cell in payload["cells"]
            if cell["range_index"] == 4 and cell["azimuth_index"] in {5, 6}
        ]
        background = [
            cell for cell in payload["cells"]
            if cell["range_index"] == 4 and cell["azimuth_index"] in {1, 2}
        ]
        self.assertTrue(all(cell["intensity"] > 0.70 for cell in wall))
        self.assertTrue(all(cell["intensity"] < 0.55 for cell in background))
        self.assertTrue(all("calibrated_residual_db" in cell for cell in payload["cells"]))

    def test_first_frame_publishes_every_trusted_cell_without_strength_cutoff(self):
        values = list(range(48))
        surface = RadarReflectivitySurface(radius_m=0.30)

        surface.ingest_heatmap(heatmap(values), 1.0, lateral_sign=1)
        payload = surface.payload(1.0)

        # Five non-zero range rows and seven bins inside the +/-60 degree FOV.
        self.assertEqual(len(payload["cells"]), 35)
        self.assertTrue(all(cell["hits"] == 1 for cell in payload["cells"]))
        self.assertTrue(all(not cell["verified"] for cell in payload["cells"]))
        self.assertTrue(all(0.0 <= cell["intensity"] <= 1.0 for cell in payload["cells"]))
        self.assertTrue(all(0.0 <= cell["confidence"] <= 1.0 for cell in payload["cells"]))
        self.assertIn(
            (1, 1),
            {(cell["range_index"], cell["azimuth_index"]) for cell in payload["cells"]},
        )

    def test_room_baseline_never_changes_current_scene_wall_geometry(self):
        """A calibration captured beside one wall must not imprint that room."""
        range_bins = 6
        azimuth_bins = 8
        common = dict(
            schema_version=1,
            profile_id="reflectivity-test",
            axes=RadarAxes(),
            range_bins=range_bins,
            azimuth_bins=azimuth_bins,
            range_step_m=0.05,
            motion_mode="major",
            point_clusters=(),
            heatmap_mad_db=(0.2,) * (range_bins * azimuth_bins),
        )
        flat_model = RadarClutterModel(
            calibration_id="flat-room",
            heatmap_median_db=(40.0,) * (range_bins * azimuth_bins),
            **common,
        )
        wall_shaped_model = RadarClutterModel(
            calibration_id="old-wall-room",
            heatmap_median_db=tuple(
                40.0 + (18.0 if index % azimuth_bins in {2, 3} else 0.0)
                for index in range(range_bins * azimuth_bins)
            ),
            **common,
        )
        values = [35] * (range_bins * azimuth_bins)
        values[4 * azimuth_bins + 5] = 220
        values[4 * azimuth_bins + 6] = 220

        payloads = []
        for model in (flat_model, wall_shaped_model):
            surface = RadarReflectivitySurface(
                radius_m=0.30,
                clutter_model=model,
                valid_min_range_m=0.07,
            )
            surface.ingest_heatmap(
                heatmap(values), 1.0, lateral_sign=1
            )
            payloads.append(surface.payload(1.0))

        self.assertFalse(payloads[0]["room_baseline_used"])
        self.assertFalse(payloads[1]["room_baseline_used"])
        first = [
            (cell["range_index"], cell["azimuth_index"], cell["relative_db"], cell["intensity"])
            for cell in payloads[0]["cells"]
        ]
        second = [
            (cell["range_index"], cell["azimuth_index"], cell["relative_db"], cell["intensity"])
            for cell in payloads[1]["cells"]
        ]
        self.assertEqual(first, second)

    def test_three_frames_raise_verification_without_hiding_early_frames(self):
        surface = RadarReflectivitySurface(radius_m=0.30)
        scan = heatmap([32] * 48)

        surface.ingest_heatmap(scan, 1.000, lateral_sign=1)
        self.assertTrue(surface.payload(1.000)["cells"])
        self.assertFalse(surface.payload(1.000)["cells"][0]["verified"])
        surface.ingest_heatmap(scan, 1.125, lateral_sign=1)
        surface.ingest_heatmap(scan, 1.250, lateral_sign=1)

        cells = surface.payload(1.250)["cells"]
        self.assertTrue(cells)
        self.assertTrue(all(cell["verified"] for cell in cells))
        self.assertTrue(all(cell["hits"] == 3 for cell in cells))
        self.assertTrue(all(cell["persistence"] == 0.6 for cell in cells))

    def test_dropped_windows_reduce_confidence_but_keep_current_measurement_visible(self):
        surface = RadarReflectivitySurface(radius_m=0.30)
        scan = heatmap([64] * 48)
        surface.ingest_heatmap(scan, 1.000, lateral_sign=1)
        surface.ingest_heatmap(
            scan,
            1.375,
            lateral_sign=1,
            elapsed_frame_windows=3,
        )

        cells = surface.payload(1.375)["cells"]
        self.assertTrue(cells)
        self.assertTrue(all(not cell["verified"] for cell in cells))
        self.assertTrue(all(cell["hits"] == 2 for cell in cells))

    def test_mirrored_heatmaps_produce_mirrored_robot_relative_evidence(self):
        original = [20] * 48
        mirrored = [20] * 48
        # Spatial-FFT bins 2 and 6 represent the same magnitude angle on
        # opposite sides of boresight for an eight-bin heatmap.
        original[3 * 8 + 2] = 240
        mirrored[3 * 8 + 6] = 240
        left = RadarReflectivitySurface(radius_m=0.30)
        right = RadarReflectivitySurface(radius_m=0.30)
        left.ingest_heatmap(heatmap(original), 2.0, lateral_sign=1)
        right.ingest_heatmap(heatmap(mirrored), 2.0, lateral_sign=1)

        left_peak = next(
            cell for cell in left.payload(2.0)["cells"]
            if cell["range_index"] == 3 and cell["azimuth_index"] == 2
        )
        right_peak = next(
            cell for cell in right.payload(2.0)["cells"]
            if cell["range_index"] == 3 and cell["azimuth_index"] == 6
        )
        self.assertAlmostEqual(left_peak["forward_m"], right_peak["forward_m"])
        self.assertAlmostEqual(left_peak["lateral_m"], -right_peak["lateral_m"])
        self.assertAlmostEqual(left_peak["intensity"], right_peak["intensity"])

    def test_payload_describes_full_soft_evidence_and_physical_uncertainty(self):
        surface = RadarReflectivitySurface(radius_m=0.30)
        surface.ingest_heatmap(heatmap([80] * 48), 3.0, lateral_sign=-1)

        payload = surface.payload(3.0)

        self.assertEqual(payload["radius_m"], 0.30)
        self.assertEqual(payload["history_frames"], 5)
        self.assertEqual(payload["required_hits"], 3)
        self.assertEqual(payload["retention"], "continuous_five_frame_evidence")
        self.assertEqual(payload["pose_mode"], "sensor_relative_short_memory")
        self.assertEqual(payload["source"], "full_range_azimuth_heatmap")
        self.assertEqual(payload["angular_resolution_deg"], 29.0)
        self.assertEqual(payload["range_step_m"], 0.05)
        cell = payload["cells"][0]
        self.assertEqual(cell["range_uncertainty_m"], 0.025)
        self.assertEqual(cell["angular_uncertainty_deg"], 14.5)

    def test_reset_clears_surface_history(self):
        surface = RadarReflectivitySurface(radius_m=0.30)
        surface.ingest_heatmap(heatmap([100] * 48), 1.0, lateral_sign=1)
        self.assertTrue(surface.payload(1.0)["cells"])

        surface.reset()

        self.assertEqual(surface.payload(1.0)["cells"], [])

    def test_rejects_invalid_time_window_and_lateral_sign(self):
        surface = RadarReflectivitySurface(radius_m=0.30)
        scan = heatmap([0] * 48)

        with self.assertRaises(ValueError):
            surface.ingest_heatmap(scan, math.nan, lateral_sign=1)
        with self.assertRaises(ValueError):
            surface.ingest_heatmap(scan, 1.0, lateral_sign=0)
        with self.assertRaises(ValueError):
            surface.ingest_heatmap(
                scan, 1.0, lateral_sign=1, elapsed_frame_windows=0
            )
        with self.assertRaises(TypeError):
            surface.ingest_heatmap(
                scan, 1.0, lateral_sign=1, elapsed_frame_windows=True
            )
        with self.assertRaises(ValueError):
            surface.payload(math.inf)


if __name__ == "__main__":
    unittest.main()
