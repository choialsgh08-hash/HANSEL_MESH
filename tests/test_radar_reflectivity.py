import math
import unittest

from monitor.radar_reflectivity import RadarReflectivitySurface


class RadarReflectivitySurfaceTests(unittest.TestCase):
    def test_requires_three_complete_frame_hits_before_publishing(self):
        surface = RadarReflectivitySurface()

        surface.ingest([(0.401, 0.061, 0.8)], 1.000)
        surface.ingest([(0.399, 0.059, 0.9)], 1.125)

        self.assertEqual(surface.payload(1.125)["cells"], [])

        surface.ingest([(0.400, 0.060, 1.0)], 1.250)
        cell = surface.payload(1.250)["cells"][0]
        self.assertEqual(cell["forward_m"], 0.4)
        self.assertEqual(cell["lateral_m"], 0.05)
        self.assertEqual(cell["hits"], 3)
        self.assertEqual(cell["persistence"], 0.6)
        self.assertAlmostEqual(cell["intensity"], 0.9)
        self.assertEqual(cell["age_ms"], 0)

    def test_dropped_windows_are_misses_and_cannot_manufacture_persistence(self):
        surface = RadarReflectivitySurface()

        surface.ingest([(0.20, 0.10, 1.0)], 1.000)
        surface.ingest(
            [(0.20, 0.10, 1.0)],
            1.375,
            elapsed_frame_windows=3,
        )

        self.assertEqual(surface.payload(1.375)["cells"], [])

    def test_three_missing_frames_remove_a_previously_visible_cell(self):
        surface = RadarReflectivitySurface()
        for index in range(5):
            surface.ingest([(0.25, -0.10, 0.75)], 1.0 + index * 0.125)
        self.assertEqual(len(surface.payload(1.5)["cells"]), 1)

        for index in range(3):
            surface.ingest([], 1.625 + index * 0.125)

        self.assertEqual(surface.payload(1.875)["cells"], [])

    def test_reset_clears_surface_history(self):
        surface = RadarReflectivitySurface()
        for index in range(3):
            surface.ingest([(0.30, 0.0, 0.8)], 1.0 + index * 0.125)
        self.assertEqual(len(surface.payload(1.25)["cells"]), 1)

        surface.reset()

        self.assertEqual(surface.payload(1.25)["cells"], [])

    def test_invalid_observations_are_ignored_and_payload_remains_finite(self):
        surface = RadarReflectivitySurface()
        malformed = [
            (math.nan, 0.0, 0.5),
            (0.1, math.inf, 0.5),
            (0.1, 0.0, math.nan),
            (True, 0.0, 0.5),
            (0.1, False, 0.5),
            (0.1, 0.0, True),
            (0.6, 0.0, 1.0),
            (-0.1, 0.0, 1.0),
        ]
        for index in range(3):
            surface.ingest(malformed, 1.0 + index * 0.125)

        payload = surface.payload(1.25)

        self.assertEqual(payload["cells"], [])
        self.assertTrue(all(
            not isinstance(value, float) or math.isfinite(value)
            for value in payload.values()
        ))

    def test_payload_metadata_and_cell_order_are_deterministic(self):
        surface = RadarReflectivitySurface()
        observations = [
            (0.30, 0.10, 0.6),
            (0.20, 0.10, 0.7),
            (0.20, -0.10, 0.8),
        ]
        for index in range(3):
            surface.ingest(observations, 1.0 + index * 0.125)

        payload = surface.payload(1.25)

        self.assertEqual(payload["radius_m"], 0.5)
        self.assertEqual(payload["resolution_m"], 0.025)
        self.assertEqual(payload["history_frames"], 5)
        self.assertEqual(payload["required_hits"], 3)
        self.assertEqual(
            payload["retention"], "three_of_five_complete_frames"
        )
        self.assertEqual(payload["pose_mode"], "sensor_relative_short_memory")
        self.assertEqual(payload["source"], "calibrated_heatmap_residual")
        self.assertEqual(
            [
                (cell["forward_m"], cell["lateral_m"])
                for cell in payload["cells"]
            ],
            [(0.2, -0.1), (0.2, 0.1), (0.3, 0.1)],
        )

    def test_rejects_invalid_time_and_elapsed_window_inputs(self):
        surface = RadarReflectivitySurface()

        with self.assertRaises(ValueError):
            surface.ingest([], math.nan)
        with self.assertRaises(ValueError):
            surface.ingest([], 1.0, elapsed_frame_windows=0)
        with self.assertRaises(TypeError):
            surface.ingest([], 1.0, elapsed_frame_windows=True)
        with self.assertRaises(ValueError):
            surface.payload(math.inf)


if __name__ == "__main__":
    unittest.main()
