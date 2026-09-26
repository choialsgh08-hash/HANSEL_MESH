import unittest

from monitor.radar_tracking import (
    DEFAULT_RANGE_STEP_M,
    VERIFIED_NEAR_8HZ_PROFILE_ID,
    height_band,
    point_association_gate,
    point_quality_score,
    resolve_range_step_m,
    robust_point_estimate,
)


class RadarTrackingTests(unittest.TestCase):
    def test_range_step_precedence_never_borrows_from_unknown_profile(self):
        cases = (
            (0.08, 0.09, True, "unknown", False, 0.08),
            (None, 0.09, True, "unknown", False, 0.09),
            (0.0, 0.09, True, "unknown", False, 0.09),
            (None, None, False, VERIFIED_NEAR_8HZ_PROFILE_ID, False,
             DEFAULT_RANGE_STEP_M),
            (None, None, False, "unknown", True, DEFAULT_RANGE_STEP_M),
            (float("nan"), None, False, "unknown", False, None),
            (None, None, False, "unknown", False, None),
        )
        for heatmap, calibrated, bound, profile, synthetic, expected in cases:
            with self.subTest(profile=profile, heatmap=heatmap):
                self.assertEqual(resolve_range_step_m(
                    heatmap_range_step_m=heatmap,
                    calibrated_range_step_m=calibrated,
                    calibration_profile_matches=bound,
                    frame_profile_id=profile,
                    synthetic=synthetic,
                ), expected)

    def test_quality_uses_snr_without_subtracting_ti_noise_db(self):
        self.assertEqual(point_quality_score(None, 90.0), 0.50)
        self.assertEqual(point_quality_score(15.0, 90.0), 0.50)
        self.assertEqual(point_quality_score(24.0, 70.0), 0.80)

    def test_association_separates_range_and_range_scaled_lateral_gates(self):
        self.assertTrue(point_association_gate(
            0.40, 0.0, 0.50, 0.0, DEFAULT_RANGE_STEP_M,
        ))
        self.assertFalse(point_association_gate(
            0.40, 0.0, 0.40, 0.25, DEFAULT_RANGE_STEP_M,
        ))

    def test_median_rejects_one_position_spike(self):
        estimate = robust_point_estimate([
            (0.40, 0.00, 0.02), (0.41, 0.01, 0.02),
            (0.39, -0.01, 0.03), (0.40, 0.00, 0.02),
            (0.63, 0.22, 0.40),
        ], residual_limit_m=0.13)
        self.assertAlmostEqual(estimate.forward_m, 0.40, places=2)
        self.assertAlmostEqual(estimate.lateral_m, 0.00, places=2)
        self.assertFalse(estimate.latest_inlier)

    def test_height_bands_are_sensor_relative(self):
        self.assertEqual(height_band(-0.101), "below_sensor")
        self.assertEqual(height_band(-0.10), "sensor_level")
        self.assertEqual(height_band(0.10), "sensor_level")
        self.assertEqual(height_band(0.101), "above_sensor")
        self.assertEqual(height_band(None), "unknown")
