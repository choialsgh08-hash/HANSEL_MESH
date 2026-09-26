import unittest

from monitor.radar_inverse_sensor import first_surface_measurements


def surface_cell(range_m, azimuth_deg, relative_db, *, scene_change=1.0):
    return {
        "range_m": range_m,
        "azimuth_deg": azimuth_deg,
        "relative_db": relative_db,
        "confidence": 1.0,
        "persistence": 1.0,
        "scene_change_score": scene_change,
        "range_uncertainty_m": 0.029,
    }


class FirstSurfaceMeasurementTests(unittest.TestCase):
    def test_exact_near_boundary_is_display_only_censored_evidence(self):
        cells = []
        for range_m, relative_db in ((0.12, 8.0), (0.18, 3.0)):
            cells.extend(surface_cell(range_m, angle, relative_db)
                         for angle in (18.0, 24.0, 30.0))

        measurements = first_surface_measurements(
            cells, minimum_range_m=0.12, maximum_range_m=0.30,
            exclude_near_boundary=True,
        )

        positive = next(item for item in measurements
                        if item["azimuth_deg"] == 24.0)
        self.assertAlmostEqual(positive["range_m"], 0.18)

    def test_verified_adc_peak_selects_its_matching_heatmap_surface(self):
        cells = []
        for range_m, relative_db in ((0.12, 2.0), (0.24, 8.0)):
            cells.extend(surface_cell(range_m, angle, relative_db)
                         for angle in (18.0, 24.0, 30.0))

        measurements = first_surface_measurements(
            cells, minimum_range_m=0.07, maximum_range_m=0.50,
            corroborating_peaks=({
                "range_m": 0.24,
                "azimuth_deg": 26.0,
                "relative_db": 5.0,
                "coherence": 0.8,
                "verified": True,
            },),
        )

        positive = next(item for item in measurements
                        if item["azimuth_deg"] > 0.0)
        self.assertAlmostEqual(positive["range_m"], 0.24)
        self.assertAlmostEqual(positive["azimuth_deg"], 26.0)

    def test_verified_adc_direction_replaces_conflicting_heatmap_lobe(self):
        cells = [
            surface_cell(0.18, angle, 4.0)
            for angle in (18.0, 24.0, 30.0)
        ]

        measurements = first_surface_measurements(
            cells, minimum_range_m=0.07, maximum_range_m=0.50,
            corroborating_peaks=({
                "range_m": 0.18,
                "azimuth_deg": -34.0,
                "relative_db": 4.0,
                "coherence": 0.7,
                "verified": True,
            },),
        )

        same_surface = [
            item for item in measurements
            if abs(item["range_m"] - 0.18) < 1e-9
        ]
        self.assertEqual(len(same_surface), 1)
        self.assertAlmostEqual(same_surface[0]["azimuth_deg"], -34.0)
        self.assertLess(same_surface[0]["lateral_m"], 0.0)

    def test_below_row_baseline_adc_component_does_not_move_surface(self):
        cells = [
            surface_cell(0.18, angle, 4.0)
            for angle in (18.0, 24.0, 30.0)
        ]

        measurements = first_surface_measurements(
            cells, minimum_range_m=0.07, maximum_range_m=0.50,
            corroborating_peaks=({
                "range_m": 0.18,
                "azimuth_deg": -34.0,
                "relative_db": -0.3,
                "coherence": 0.7,
                "verified": True,
            },),
        )

        same_surface = [
            item for item in measurements
            if abs(item["range_m"] - 0.18) < 1e-9
        ]
        self.assertEqual(len(same_surface), 1)
        self.assertAlmostEqual(same_surface[0]["azimuth_deg"], 24.0)

    def test_unverified_adc_direction_does_not_move_heatmap_surface(self):
        cells = [
            surface_cell(0.18, angle, 4.0)
            for angle in (18.0, 24.0, 30.0)
        ]

        measurements = first_surface_measurements(
            cells, minimum_range_m=0.07, maximum_range_m=0.50,
            corroborating_peaks=({
                "range_m": 0.18,
                "azimuth_deg": -34.0,
                "relative_db": 4.0,
                "coherence": 0.7,
                "verified": False,
            },),
        )

        same_surface = [
            item for item in measurements
            if abs(item["range_m"] - 0.18) < 1e-9
        ]
        self.assertEqual(len(same_surface), 1)
        self.assertAlmostEqual(same_surface[0]["azimuth_deg"], 24.0)

    def test_scene_change_from_an_old_room_image_does_not_move_geometry(self):
        cells = []
        for range_m, relative_db, scene_change in (
            (0.18, 2.0, 0.0), (0.30, 5.0, 1.0)
        ):
            cells.extend(surface_cell(
                range_m, angle, relative_db, scene_change=scene_change,
            ) for angle in (18.0, 24.0, 30.0))

        measurements = first_surface_measurements(
            cells, minimum_range_m=0.07, maximum_range_m=0.50,
        )

        positive = next(item for item in measurements
                        if item["azimuth_deg"] == 24.0)
        self.assertAlmostEqual(positive["range_m"], 0.18)


if __name__ == "__main__":
    unittest.main()
