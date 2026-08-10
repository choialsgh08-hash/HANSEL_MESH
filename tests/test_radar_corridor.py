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

PHOTO_LAYOUT_POINT_WALL = [
    (0.243, -0.027, 0.95),
    (0.363, -0.071, 0.95),
]

PHOTO_LAYOUT_HEATMAP_WALL = [
    (0.100, -0.100, 0.74),
    (0.200, -0.136, 0.77),
    (0.300, -0.173, 0.72),
    (0.400, -0.209, 0.70),
]

PHOTO_LAYOUT_CENTER_ARC = [
    (0.100, -0.047, 0.82),
    (0.200, -0.083, 0.84),
    (0.300, -0.119, 0.80),
    (0.400, -0.156, 0.81),
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
    def test_heatmap_only_live_reflections_cannot_invent_a_far_diagonal_pair(self):
        estimator = ReflectivityCorridorEstimator()

        estimator.ingest(cells(LIVE_CLUTTERED_RIGHT_CORRIDOR))
        payload = estimator.payload()

        self.assertEqual(payload["status"], "unavailable")
        self.assertEqual(payload["walls"], [])

    def test_point_seeded_photo_layout_confirms_after_three_frames(self):
        estimator = ReflectivityCorridorEstimator()
        observed = cells(PHOTO_LAYOUT_HEATMAP_WALL + PHOTO_LAYOUT_CENTER_ARC)
        point_observations = cells(PHOTO_LAYOUT_POINT_WALL)

        estimator.ingest(observed, point_observations=point_observations)
        first = estimator.payload()
        estimator.ingest(observed, point_observations=point_observations)
        second = estimator.payload()
        estimator.ingest(observed, point_observations=point_observations)
        confirmed = estimator.payload()

        self.assertEqual(first["status"], "candidate")
        self.assertEqual(first["hits"], 1)
        self.assertEqual(second["status"], "candidate")
        self.assertEqual(second["hits"], 2)
        self.assertEqual(confirmed["status"], "confirmed")
        self.assertEqual(confirmed["hits"], 3)
        self.assertLess(confirmed["guidance_heading_deg"], -10.0)
        self.assertGreater(confirmed["measured_heading_deg"], -25.0)
        self.assertLess(confirmed["measured_heading_deg"], -10.0)
        self.assertGreater(confirmed["width_m"], 0.10)
        self.assertLess(confirmed["width_m"], 0.16)
        self.assertGreater(confirmed["confidence"], 0.4)
        self.assertEqual(
            [wall["wall_id"] for wall in confirmed["walls"]],
            ["wall_1", "wall_2"],
        )
        offsets = [wall["normal_offset_m"] for wall in confirmed["walls"]]
        self.assertLess(offsets[0], 0.0)
        self.assertGreater(offsets[1], 0.0)
        self.assertTrue(all(abs(offset) >= 0.03 for offset in offsets))
        for wall in confirmed["walls"]:
            self.assertGreaterEqual(wall["support_cells"], 2)
            self.assertGreaterEqual(wall["span_m"], 0.08)
            self.assertGreater(wall["mean_intensity"], 0.30)
            self.assertEqual(len(wall["start"]), 2)
            self.assertEqual(len(wall["end"]), 2)
            self.assertAlmostEqual(wall["start"][0], 0.0, places=6)
            self.assertTrue(all(math.isfinite(value) for value in wall["start"]))
            self.assertTrue(all(math.isfinite(value) for value in wall["end"]))

    def test_short_left_wall_seed_beats_far_center_noise(self):
        estimator = ReflectivityCorridorEstimator()
        point_observations = cells([
            (0.104, -0.119, 0.742),
            (0.140, -0.157, 0.617),
            (0.271, -0.015, 0.475),
            (0.370, -0.015, 0.458),
        ])
        observed = cells(LIVE_CLUTTERED_RIGHT_CORRIDOR)

        for _ in range(3):
            estimator.ingest(
                observed,
                point_observations=point_observations,
            )
        payload = estimator.payload()

        self.assertEqual(payload["status"], "confirmed")
        self.assertLess(payload["guidance_heading_deg"], -30.0)
        self.assertGreater(payload["width_m"], 0.20)
        self.assertLess(payload["width_m"], 0.35)

    def test_input_order_cannot_change_the_confirmed_payload(self):
        forward = ReflectivityCorridorEstimator()
        reverse = ReflectivityCorridorEstimator()
        observed = cells(PHOTO_LAYOUT_HEATMAP_WALL + PHOTO_LAYOUT_CENTER_ARC)
        point_observations = cells(PHOTO_LAYOUT_POINT_WALL)

        for _ in range(3):
            forward.ingest(observed, point_observations=point_observations)
            reverse.ingest(
                list(reversed(observed)),
                point_observations=list(reversed(point_observations)),
            )

        self.assertEqual(forward.payload(), reverse.payload())

    def test_temporal_medians_keep_serialized_wall_geometry_consistent(self):
        estimator = ReflectivityCorridorEstimator()
        for left_clearance, right_clearance in (
            (0.03, 0.09),
            (0.08, 0.08),
            (0.09, 0.03),
        ):
            walls = []
            for offset in (-left_clearance, right_clearance):
                walls.append({
                    "heading_deg": -20.0,
                    "normal_offset_m": offset,
                    "support_cells": 2,
                    "span_m": 0.10,
                    "mean_intensity": 0.60,
                    "start": [0.0, offset / math.cos(math.radians(-20.0))],
                    "end": [0.4, 0.4 * math.tan(math.radians(-20.0))],
                })
            estimator._history.append({
                "measured_heading_deg": -20.0,
                "width_m": left_clearance + right_clearance,
                "confidence": 0.60,
                "walls": walls,
            })

        payload = estimator.payload()
        wall_one, wall_two = payload["walls"]

        self.assertEqual(
            payload["width_m"],
            round(
                abs(
                    wall_one["normal_offset_m"]
                    - wall_two["normal_offset_m"]
                ),
                6,
            ),
        )

    def test_dropped_windows_remove_confirmation_and_reset_removes_geometry(self):
        estimator = ReflectivityCorridorEstimator()
        observed = cells(PHOTO_LAYOUT_HEATMAP_WALL + PHOTO_LAYOUT_CENTER_ARC)
        point_observations = cells(PHOTO_LAYOUT_POINT_WALL)
        for _ in range(3):
            estimator.ingest(observed, point_observations=point_observations)
        self.assertEqual(estimator.payload()["status"], "confirmed")

        estimator.ingest([], elapsed_frame_windows=3)
        retained = estimator.payload()
        self.assertEqual(retained["status"], "candidate")
        self.assertEqual(retained["hits"], 2)
        estimator.ingest([], elapsed_frame_windows=2)
        self.assertEqual(estimator.payload()["status"], "unavailable")

        estimator.ingest(observed, point_observations=point_observations)
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
            estimator.ingest(
                cells(PHOTO_LAYOUT_HEATMAP_WALL),
                point_observations=[],
            )
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

    def test_outer_range_arc_without_point_support_is_not_a_wall(self):
        estimator = ReflectivityCorridorEstimator()
        outer_arc = cells([
            (
                0.49 * math.cos(math.radians(angle)),
                0.49 * math.sin(math.radians(angle)),
                0.9,
            )
            for angle in range(-55, 56, 10)
        ])

        for _ in range(3):
            estimator.ingest(outer_arc)

        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_elapsed_frame_windows_must_be_a_positive_integer(self):
        estimator = ReflectivityCorridorEstimator()
        for invalid in (True, 0, -1, 1.5):
            with self.subTest(invalid=invalid):
                with self.assertRaises((TypeError, ValueError)):
                    estimator.ingest([], elapsed_frame_windows=invalid)


if __name__ == "__main__":
    unittest.main()
