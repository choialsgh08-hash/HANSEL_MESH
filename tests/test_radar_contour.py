import math
import unittest

from monitor.radar_contour import CalibratedContourEstimator


RANGE_STEP_M = 0.05859375
AZIMUTH_BINS = 32


def sample(azimuth_index, range_index, prominence_db=4.0):
    argument = 2.0 * (
        azimuth_index - AZIMUTH_BINS / 2.0
    ) / AZIMUTH_BINS
    angle = math.asin(max(-1.0, min(1.0, argument)))
    range_m = range_index * RANGE_STEP_M
    return {
        "azimuth_index": azimuth_index,
        "range_index": range_index,
        "forward_m": range_m * math.cos(angle),
        "lateral_m": -range_m * math.sin(angle),
        "prominence_db": prominence_db,
        "residual_db": prominence_db,
    }


class CalibratedContourEstimatorTests(unittest.TestCase):
    def test_confirms_independent_front_and_diagonal_segments(self):
        observations = [
            *(sample(index, 3) for index in range(14, 21)),
            *(sample(index, 5) for index in range(21, 27)),
        ]
        estimator = CalibratedContourEstimator()

        for _ in range(3):
            estimator.ingest(observations)

        payload = estimator.payload()
        self.assertEqual(payload["status"], "confirmed")
        self.assertEqual(len(payload["segments"]), 2)
        front, diagonal = sorted(
            payload["segments"],
            key=lambda segment: min(point[0] for point in (
                segment["start"], segment["end"],
            )),
        )
        self.assertGreater(abs(front["heading_deg"]), 80.0)
        self.assertAlmostEqual(
            (front["start"][0] + front["end"][0]) / 2.0,
            0.174,
            delta=0.01,
        )
        self.assertGreater(diagonal["heading_deg"], 55.0)
        self.assertLess(diagonal["heading_deg"], 70.0)
        self.assertEqual(front["support_cells"], 7)
        self.assertEqual(diagonal["support_cells"], 6)

    def test_range_discontinuity_splits_adjacent_azimuth_surfaces(self):
        observations = [
            *(sample(index, 3) for index in range(10, 15)),
            *(sample(index, 5) for index in range(15, 20)),
        ]
        estimator = CalibratedContourEstimator()
        for _ in range(3):
            estimator.ingest(observations)

        self.assertEqual(len(estimator.payload()["segments"]), 2)

    def test_one_bin_corner_is_two_surfaces_not_a_bridging_diagonal(self):
        observations = [
            *(sample(index, 3) for index in range(18, 22)),
            *(sample(index, 2) for index in range(22, 30)),
        ]
        estimator = CalibratedContourEstimator()
        for _ in range(3):
            estimator.ingest(observations)

        payload = estimator.payload()
        self.assertEqual(payload["status"], "confirmed")
        self.assertEqual(len(payload["segments"]), 2)
        self.assertTrue(all(
            segment["span_m"] >= 0.03
            for segment in payload["segments"]
        ))

    def test_two_bins_and_nonadjacent_scatter_cannot_invent_segments(self):
        observations = [
            sample(5, 3),
            sample(6, 3),
            sample(10, 4),
            sample(14, 5),
            sample(18, 3),
        ]
        estimator = CalibratedContourEstimator()
        for _ in range(5):
            estimator.ingest(observations)

        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_dropped_frame_window_breaks_three_frame_confirmation(self):
        observations = [sample(index, 3) for index in range(14, 20)]
        estimator = CalibratedContourEstimator()
        estimator.ingest(observations)
        estimator.ingest(observations, elapsed_frame_windows=2)

        self.assertEqual(estimator.payload()["status"], "unavailable")

        estimator.ingest(observations)
        self.assertEqual(estimator.payload()["status"], "confirmed")


if __name__ == "__main__":
    unittest.main()
