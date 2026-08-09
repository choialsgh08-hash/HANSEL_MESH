import math
import unittest

from monitor.radar_corridor import ReflectivityCorridorEstimator


WALL_ONE = [
    (0.150, -0.250, 0.503),
    (0.200, -0.225, 0.526),
    (0.225, -0.175, 0.519),
    (0.250, -0.150, 0.474),
    (0.275, -0.100, 0.520),
    (0.275, -0.075, 0.541),
    (0.300, -0.025, 0.557),
    (0.300, 0.000, 0.543),
    (0.300, 0.025, 0.467),
    (0.350, 0.150, 0.682),
    (0.375, 0.100, 0.768),
    (0.425, 0.250, 0.447),
]

WALL_TWO = [
    (0.325, -0.375, 0.422),
    (0.375, -0.300, 0.535),
    (0.450, -0.175, 0.640),
    (0.475, -0.125, 0.739),
    (0.475, -0.050, 0.808),
    (0.500, 0.000, 0.806),
]

LIVE_CLUTTERED_RIGHT_CORRIDOR = [
    (0.100, -0.275, 0.4973),
    (0.150, -0.250, 0.5007),
    (0.150, -0.125, 0.3440),
    (0.150, 0.250, 0.4805),
    (0.175, -0.100, 0.2681),
    (0.200, -0.350, 0.2296),
    (0.200, -0.225, 0.4649),
    (0.200, 0.225, 0.4279),
    (0.225, -0.175, 0.4009),
    (0.225, 0.175, 0.3400),
    (0.250, -0.300, 0.3580),
    (0.250, -0.150, 0.6087),
    (0.250, 0.150, 0.3287),
    (0.275, -0.100, 0.6226),
    (0.275, -0.075, 0.6131),
    (0.275, 0.075, 0.4792),
    (0.275, 0.100, 0.4514),
    (0.300, -0.250, 0.5296),
    (0.300, -0.025, 0.5925),
    (0.300, 0.000, 0.5144),
    (0.300, 0.025, 0.4049),
    (0.325, -0.375, 0.4474),
    (0.350, -0.200, 0.5126),
    (0.350, -0.150, 0.4442),
    (0.350, 0.150, 0.7119),
    (0.350, 0.200, 0.4035),
    (0.375, -0.300, 0.5717),
    (0.375, -0.100, 0.3545),
    (0.375, 0.100, 0.8133),
    (0.375, 0.300, 0.2870),
    (0.400, -0.050, 0.5588),
    (0.400, 0.000, 0.7211),
    (0.400, 0.050, 0.7988),
    (0.425, -0.250, 0.5250),
    (0.425, 0.250, 0.4889),
    (0.450, -0.175, 0.5073),
    (0.450, 0.175, 0.6966),
    (0.475, -0.125, 0.6347),
    (0.475, -0.050, 0.7607),
    (0.475, 0.050, 0.8020),
    (0.475, 0.125, 0.7900),
    (0.500, 0.000, 0.8027),
]


def cells(rows):
    return [
        {
            "forward_m": forward,
            "lateral_m": lateral,
            "intensity": intensity,
        }
        for forward, lateral, intensity in rows
    ]


class ReflectivityCorridorEstimatorTests(unittest.TestCase):
    def test_live_extra_reflections_do_not_displace_the_diagonal_wall_pair(self):
        estimator = ReflectivityCorridorEstimator()

        estimator.ingest(cells(LIVE_CLUTTERED_RIGHT_CORRIDOR))
        payload = estimator.payload()

        self.assertEqual(payload["status"], "candidate")
        self.assertGreater(payload["measured_heading_deg"], 60.0)
        self.assertEqual(payload["guidance_heading_deg"], 55.0)
        self.assertAlmostEqual(payload["width_m"], 0.19321, places=4)
        self.assertEqual(len(payload["walls"]), 2)

    def test_recorded_diagonal_pair_confirms_after_three_frames(self):
        estimator = ReflectivityCorridorEstimator()
        observed = cells(WALL_ONE + WALL_TWO)

        estimator.ingest(observed)
        first = estimator.payload()
        estimator.ingest(observed)
        second = estimator.payload()
        estimator.ingest(observed)
        confirmed = estimator.payload()

        self.assertEqual(first["status"], "candidate")
        self.assertEqual(first["hits"], 1)
        self.assertEqual(second["status"], "candidate")
        self.assertEqual(second["hits"], 2)
        self.assertEqual(confirmed["status"], "confirmed")
        self.assertEqual(confirmed["hits"], 3)
        self.assertEqual(confirmed["guidance_heading_deg"], 55.0)
        self.assertGreater(confirmed["measured_heading_deg"], 60.0)
        self.assertLess(confirmed["measured_heading_deg"], 70.0)
        self.assertGreater(confirmed["width_m"], 0.16)
        self.assertLess(confirmed["width_m"], 0.23)
        self.assertGreater(confirmed["confidence"], 0.5)
        self.assertEqual(
            [wall["wall_id"] for wall in confirmed["walls"]],
            ["wall_1", "wall_2"],
        )
        for wall in confirmed["walls"]:
            self.assertGreaterEqual(wall["support_cells"], 4)
            self.assertGreaterEqual(wall["span_m"], 0.14)
            self.assertGreater(wall["mean_intensity"], 0.30)
            self.assertEqual(len(wall["start"]), 2)
            self.assertEqual(len(wall["end"]), 2)
            self.assertTrue(all(math.isfinite(value) for value in wall["start"]))
            self.assertTrue(all(math.isfinite(value) for value in wall["end"]))

    def test_input_order_cannot_change_the_confirmed_payload(self):
        forward = ReflectivityCorridorEstimator()
        reverse = ReflectivityCorridorEstimator()
        observed = cells(WALL_ONE + WALL_TWO)

        for _ in range(3):
            forward.ingest(observed)
            reverse.ingest(list(reversed(observed)))

        self.assertEqual(forward.payload(), reverse.payload())

    def test_dropped_windows_remove_confirmation_and_reset_removes_geometry(self):
        estimator = ReflectivityCorridorEstimator()
        observed = cells(WALL_ONE + WALL_TWO)
        for _ in range(3):
            estimator.ingest(observed)
        self.assertEqual(estimator.payload()["status"], "confirmed")

        estimator.ingest([], elapsed_frame_windows=3)
        retained = estimator.payload()
        self.assertEqual(retained["status"], "candidate")
        self.assertEqual(retained["hits"], 2)
        estimator.ingest([], elapsed_frame_windows=2)
        self.assertEqual(estimator.payload()["status"], "unavailable")

        estimator.ingest(observed)
        self.assertEqual(estimator.payload()["status"], "candidate")
        estimator.reset()
        self.assertEqual(estimator.payload(), {
            "status": "unavailable",
            "method": "deterministic_parallel_ransac",
            "history_frames": 5,
            "required_hits": 3,
            "hits": 0,
            "measured_heading_deg": None,
            "guidance_heading_deg": None,
            "width_m": None,
            "confidence": 0.0,
            "walls": [],
        })

    def test_single_wall_and_scatter_do_not_invent_a_corridor(self):
        estimator = ReflectivityCorridorEstimator()
        for _ in range(3):
            estimator.ingest(cells(WALL_ONE))
        self.assertEqual(estimator.payload()["status"], "unavailable")

        scatter = cells([
            (0.10, -0.30, 0.8),
            (0.13, 0.24, 0.7),
            (0.21, -0.02, 0.6),
            (0.29, 0.31, 0.9),
            (0.38, -0.27, 0.7),
            (0.49, 0.08, 0.8),
        ])
        for _ in range(3):
            estimator.ingest(scatter)
        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_elapsed_frame_windows_must_be_a_positive_integer(self):
        estimator = ReflectivityCorridorEstimator()
        for invalid in (True, 0, -1, 1.5):
            with self.subTest(invalid=invalid):
                with self.assertRaises((TypeError, ValueError)):
                    estimator.ingest([], elapsed_frame_windows=invalid)


if __name__ == "__main__":
    unittest.main()
