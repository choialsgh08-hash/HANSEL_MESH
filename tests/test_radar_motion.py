import math
import unittest

from monitor.radar_motion import RadarMotionEstimator


def translated_pairs(count=5, delta_forward=-0.02):
    old = [(0.25 + index * 0.08, -0.16 + index * 0.07) for index in range(count)]
    return [(point, (point[0] + delta_forward, point[1])) for point in old]


def rotated_pairs(yaw_deg):
    angle = math.radians(yaw_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        (old, (
            cosine * old[0] - sine * old[1],
            sine * old[0] + cosine * old[1],
        ))
        for old, _ in translated_pairs(5, 0.0)
    ]


class RadarMotionEstimatorTests(unittest.TestCase):
    def test_requires_five_confirmed_correspondences(self):
        result = RadarMotionEstimator().update(translated_pairs(4), 4)
        self.assertFalse(result["applied"])
        self.assertEqual(result["status"], "insufficient_points")

    def test_rejects_match_ratio_below_seventy_percent(self):
        result = RadarMotionEstimator().update(translated_pairs(5), 8)
        self.assertFalse(result["applied"])
        self.assertEqual(result["status"], "low_confidence")

    def test_rejects_rmse_over_five_centimeters(self):
        pairs = translated_pairs(5)
        pairs[2] = (pairs[2][0], (pairs[2][1][0], pairs[2][1][1] + 0.16))
        result = RadarMotionEstimator(stabilization_frames=1).update(pairs, 5)
        self.assertFalse(result["applied"])
        self.assertEqual(result["status"], "high_rmse")
        self.assertGreater(result["rmse_m"], 0.05)

    def test_rejects_translation_and_yaw_limits_independently(self):
        translation = RadarMotionEstimator(stabilization_frames=1).update(
            translated_pairs(5, -0.081), 5,
        )
        yaw = RadarMotionEstimator(stabilization_frames=1).update(
            rotated_pairs(8.01), 5,
        )
        self.assertEqual(translation["status"], "implausible_motion")
        self.assertEqual(yaw["status"], "implausible_motion")
        self.assertFalse(translation["applied"])
        self.assertFalse(yaw["applied"])

    def test_invalid_values_and_anchor_counts_are_guarded(self):
        pairs = translated_pairs(5)
        pairs[0] = (pairs[0][0], (float("nan"), pairs[0][1][1]))
        self.assertEqual(
            RadarMotionEstimator().update(pairs, 5)["status"],
            "invalid_input",
        )
        self.assertEqual(
            RadarMotionEstimator().update(translated_pairs(5), 4)["status"],
            "invalid_input",
        )
        self.assertEqual(
            RadarMotionEstimator().update([], 0)["status"],
            "insufficient_points",
        )

    def test_sparse_invalid_correspondences_are_invalid_input(self):
        invalid_cases = (
            (((0.25, -0.16), (float("nan"), -0.16)),),
            ((None, None),),
        )
        for correspondences in invalid_cases:
            with self.subTest(correspondences=correspondences):
                result = RadarMotionEstimator().update(correspondences, 1)
                self.assertEqual(result["status"], "invalid_input")
                self.assertFalse(result["applied"])

    def test_sparse_mapping_correspondence_is_invalid_input(self):
        try:
            result = RadarMotionEstimator().update(
                (({}, (0.23, -0.16)),),
                1,
            )
        except KeyError as error:
            self.fail(f"malformed correspondence raised KeyError: {error}")
        self.assertEqual(result["status"], "invalid_input")
        self.assertFalse(result["applied"])

    def test_sparse_points_require_exactly_two_coordinates(self):
        invalid_cases = (
            (((0.25,), (0.23, -0.16)),),
            (((0.25, -0.16, 0.0), (0.23, -0.16)),),
            (((0.25, -0.16), (0.23,)),),
            (((0.25, -0.16), (0.23, -0.16, 0.0)),),
        )
        for correspondences in invalid_cases:
            with self.subTest(correspondences=correspondences):
                result = RadarMotionEstimator().update(correspondences, 1)
                self.assertEqual(result["status"], "invalid_input")
                self.assertFalse(result["applied"])

    def test_unrepresentable_coordinates_are_invalid_input(self):
        pairs = translated_pairs(5)
        pairs[0] = ((10 ** 1000, pairs[0][0][1]), pairs[0][1])
        result = RadarMotionEstimator().update(pairs, 5)
        self.assertEqual(result["status"], "invalid_input")
        self.assertFalse(result["applied"])

    def test_finite_coordinates_that_overflow_the_solve_are_invalid_input(self):
        pairs = translated_pairs(5)
        pairs[0] = (pairs[0][0], (1e200, -1e200))
        result = RadarMotionEstimator().update(pairs, 5)
        self.assertEqual(result["status"], "invalid_input")
        self.assertEqual(result["confidence"], 0.0)
        self.assertFalse(result["applied"])

    def test_inconsistent_estimate_restarts_stabilization_and_reset_forgets_it(self):
        estimator = RadarMotionEstimator(stabilization_frames=2)
        estimator.update(translated_pairs(5, -0.02), 5)
        unstable = estimator.update(translated_pairs(5, 0.04), 5)
        self.assertEqual(unstable["status"], "unstable")
        self.assertFalse(unstable["applied"])
        self.assertTrue(estimator.update(
            translated_pairs(5, 0.04), 5,
        )["applied"])
        estimator.reset()
        after_reset = estimator.update(translated_pairs(5, 0.04), 5)
        self.assertEqual(after_reset["status"], "stabilizing")
        self.assertFalse(after_reset["applied"])

    def test_two_consistent_five_match_estimates_are_applied(self):
        estimator = RadarMotionEstimator(stabilization_frames=2)
        first = estimator.update(translated_pairs(5), 5)
        second = estimator.update(translated_pairs(5), 5)
        self.assertEqual(first["status"], "stabilizing")
        self.assertFalse(first["applied"])
        self.assertEqual(second["status"], "ok")
        self.assertTrue(second["applied"])
        self.assertAlmostEqual(second["transform_forward_m"], -0.02, delta=0.005)

    def test_stability_gate_uses_unrounded_translation(self):
        estimator = RadarMotionEstimator(stabilization_frames=2)
        estimator.update(translated_pairs(5, 0.00004), 5)
        result = estimator.update(translated_pairs(5, 0.050049), 5)
        self.assertEqual(result["status"], "unstable")
        self.assertFalse(result["applied"])

    def test_ego_payload_is_inverse_transform_and_confidence_is_bounded(self):
        yaw = math.radians(5.0)
        translation = (0.02, -0.01)
        pairs = []
        for old, _ in translated_pairs(5, 0.0):
            rotated = (
                math.cos(yaw) * old[0] - math.sin(yaw) * old[1],
                math.sin(yaw) * old[0] + math.cos(yaw) * old[1],
            )
            pairs.append((old, (
                rotated[0] + translation[0],
                rotated[1] + translation[1],
            )))
        result = RadarMotionEstimator(stabilization_frames=1).update(pairs, 5)
        expected_ego = (
            -(math.cos(yaw) * translation[0] + math.sin(yaw) * translation[1]),
            -(-math.sin(yaw) * translation[0] + math.cos(yaw) * translation[1]),
        )
        self.assertAlmostEqual(result["transform_forward_m"], 0.02, places=4)
        self.assertAlmostEqual(result["transform_lateral_m"], -0.01, places=4)
        self.assertAlmostEqual(result["transform_yaw_deg"], 5.0, places=3)
        self.assertAlmostEqual(result["ego_forward_m"], expected_ego[0], places=4)
        self.assertAlmostEqual(result["ego_lateral_m"], expected_ego[1], places=4)
        self.assertAlmostEqual(result["ego_yaw_deg"], -5.0, places=3)
        self.assertGreaterEqual(result["confidence"], 0.0)
        self.assertLessEqual(result["confidence"], 1.0)
