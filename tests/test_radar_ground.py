import math
import unittest

from monitor.radar_ground import RadarMountGeometry


class RadarMountGeometryTests(unittest.TestCase):
    def test_zero_pitch_maps_sensor_height_to_ground_height(self):
        geometry = RadarMountGeometry()

        forward, lateral, height = geometry.robot_coordinates(
            0.08,
            -0.02,
            -0.05,
        )

        self.assertAlmostEqual(forward, 0.08)
        self.assertAlmostEqual(lateral, -0.02)
        self.assertAlmostEqual(height, 0.015)

    def test_positive_pitch_rotates_forward_axis_upward(self):
        geometry = RadarMountGeometry(sensor_pitch_deg=30.0)

        forward, lateral, height = geometry.robot_coordinates(
            0.20,
            -0.03,
            -0.10,
        )

        self.assertAlmostEqual(forward, 0.2232050808, places=9)
        self.assertEqual(lateral, -0.03)
        self.assertAlmostEqual(height, 0.0783974596, places=9)

    def test_ground_gate_keeps_raised_return_at_same_horizontal_distance(self):
        geometry = RadarMountGeometry()

        self.assertTrue(geometry.is_ground(0.08, 0.0, -0.05))
        self.assertTrue(geometry.is_ground(0.08, 0.0, -0.03))
        self.assertFalse(geometry.is_ground(0.08, 0.0, 0.0))

    def test_ground_uncertainty_fringe_is_hidden_without_widening_the_fov(self):
        geometry = RadarMountGeometry()

        self.assertEqual(
            geometry.ground_classification(0.08, 0.0, -0.030),
            "ground",
        )
        self.assertEqual(
            geometry.ground_classification(0.08, 0.0, -0.024),
            "ground_fringe",
        )
        self.assertIsNone(
            geometry.ground_classification(0.08, 0.0, -0.014),
        )
        self.assertTrue(geometry.is_ground(0.08, 0.0, -0.024))

    def test_ground_gate_does_not_hide_returns_outside_local_forward_window(self):
        geometry = RadarMountGeometry()

        self.assertFalse(geometry.is_ground(-0.01, 0.0, -0.065))
        self.assertFalse(geometry.is_ground(0.61, 0.0, -0.065))

    def test_nonfinite_point_height_is_not_treated_as_ground(self):
        geometry = RadarMountGeometry()

        self.assertFalse(geometry.is_ground(0.08, 0.0, None))
        self.assertFalse(geometry.is_ground(0.08, 0.0, math.nan))
        self.assertFalse(geometry.is_ground(0.08, 0.0, math.inf))

    def test_payload_exposes_auditable_hardware_binding(self):
        geometry = RadarMountGeometry()

        self.assertEqual(
            geometry.payload(),
            {
                "sensor_height_m": 0.065,
                "sensor_pitch_deg": 0.0,
                "ground_rejection_height_m": 0.035,
                "ground_uncertainty_m": 0.015,
                "effective_rejection_height_m": 0.05,
                "filter_range_m": 0.60,
                "pitch_convention": "positive_up",
                "status": "configured",
            },
        )

    def test_invalid_mount_geometry_fails_during_construction(self):
        cases = (
            {"sensor_height_m": True},
            {"sensor_height_m": math.nan},
            {"sensor_height_m": math.inf},
            {"sensor_height_m": -0.01},
            {"sensor_pitch_deg": False},
            {"sensor_pitch_deg": math.nan},
            {"sensor_pitch_deg": math.inf},
            {"ground_rejection_height_m": True},
            {"ground_rejection_height_m": -0.01},
            {"ground_rejection_height_m": math.inf},
            {"ground_rejection_height_m": 0.065},
            {"ground_rejection_height_m": 0.066},
            {"ground_uncertainty_m": True},
            {"ground_uncertainty_m": -0.01},
            {"ground_uncertainty_m": math.inf},
            {"ground_uncertainty_m": 0.03},
            {"filter_range_m": False},
            {"filter_range_m": 0.0},
            {"filter_range_m": -0.01},
            {"filter_range_m": math.inf},
        )
        for values in cases:
            with self.subTest(values=values):
                with self.assertRaises((TypeError, ValueError)):
                    RadarMountGeometry(**values)


if __name__ == "__main__":
    unittest.main()
