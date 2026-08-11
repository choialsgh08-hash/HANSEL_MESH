import math
import random
import time
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

STRONG_STRAIGHT_HEATMAP_CORRIDOR = [
    (0.150, -0.155, 0.64),
    (0.225, -0.155, 0.68),
    (0.300, -0.155, 0.65),
    (0.375, -0.155, 0.69),
    (0.450, -0.155, 0.66),
    (0.150, 0.155, 0.67),
    (0.225, 0.155, 0.71),
    (0.300, 0.155, 0.66),
    (0.375, 0.155, 0.68),
    (0.450, 0.155, 0.69),
]

STRONG_CONVERGING_HEATMAP_CORRIDOR = [
    (forward_m, lateral_m, 0.82)
    for forward_m, half_width_m in (
        (0.200, 0.250),
        (0.250, 0.200),
        (0.300, 0.150),
        (0.350, 0.100),
        (0.400, 0.050),
        # The lobe beyond the bottleneck is multipath, not a wall reopening.
        (0.425, 0.250),
        (0.450, 0.200),
    )
    for lateral_m in (-half_width_m, half_width_m)
]

# Quantized cells observed from the live right-hand cardboard wall on
# 2026-08-11.  The first sampling gap is about 9 cm even though all five cells
# lie on one strong physical boundary.
LIVE_SINGLE_RIGHT_DIAGONAL = [
    (0.325, 0.375, 1.0),
    (0.375, 0.300, 1.0),
    (0.425, 0.250, 0.9999),
    (0.450, 0.175, 1.0),
    (0.475, 0.125, 1.0),
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


def varied_quantized_scatter(seed, points_per_side):
    left = [
        (forward_index * 0.025, lateral_index * 0.025)
        for forward_index in range(1, 21)
        for lateral_index in range(-20, -1)
        if math.hypot(
            forward_index * 0.025,
            lateral_index * 0.025,
        ) <= 0.5 + 1e-12
    ]
    right = [(forward, -lateral) for forward, lateral in left]
    rng = random.Random(seed)
    return [
        (forward, lateral, rng.uniform(0.60, 1.0))
        for forward, lateral in rng.sample(left, points_per_side)
    ] + [
        (forward, lateral, rng.uniform(0.60, 1.0))
        for forward, lateral in rng.sample(right, points_per_side)
    ]


class ReflectivityCorridorEstimatorTests(unittest.TestCase):
    def test_heatmap_edges_confirm_a_parallel_left_turn_without_distance_hardcoding(self):
        estimator = ReflectivityCorridorEstimator()
        edge_observations = cells([
            (0.20, -0.23, 0.82),
            (0.30, -0.24, 0.84),
            (0.40, -0.25, 0.83),
            (0.50, -0.26, 0.81),
            (0.20, 0.30, 0.86),
            (0.30, 0.29, 0.85),
            (0.40, 0.28, 0.87),
            (0.50, 0.27, 0.84),
        ])
        for index, observation in enumerate(edge_observations):
            observation["range_index"] = 4 + index % 4
        radial_sidelobes = cells([
            (0.20, -0.115, 1.0),
            (0.30, -0.173, 1.0),
            (0.40, -0.231, 1.0),
            (0.50, -0.289, 1.0),
            (0.20, 0.115, 1.0),
            (0.30, 0.173, 1.0),
            (0.40, 0.231, 1.0),
            (0.50, 0.289, 1.0),
        ])
        for index, observation in enumerate(radial_sidelobes):
            observation["range_index"] = 4 + index % 4

        for _ in range(3):
            estimator.ingest(
                [],
                edge_observations=edge_observations + radial_sidelobes,
            )

        corridor = estimator.payload()
        self.assertEqual(corridor["status"], "confirmed")
        self.assertEqual(corridor["geometry"], "parallel")
        self.assertAlmostEqual(corridor["measured_heading_deg"], -5.71, delta=1.0)
        self.assertAlmostEqual(corridor["width_m"], 0.53, delta=0.03)
        boundaries = estimator.boundaries_payload()
        self.assertEqual(boundaries["status"], "confirmed")
        self.assertEqual(
            [wall["evidence_kind"] for wall in boundaries["walls"]],
            ["heatmap_edge", "heatmap_edge"],
        )

    def test_two_heatmap_edge_rows_cannot_invent_a_wall(self):
        estimator = ReflectivityCorridorEstimator()
        observations = cells([
            (0.30, -0.24, 0.9),
            (0.40, -0.25, 0.9),
            (0.30, 0.29, 0.9),
            (0.40, 0.28, 0.9),
        ])
        for index, observation in enumerate(observations):
            observation["range_index"] = 5 + index % 2
        for _ in range(5):
            estimator.ingest([], edge_observations=observations)

        self.assertEqual(estimator.payload()["status"], "unavailable")
        self.assertEqual(
            estimator.boundaries_payload()["status"],
            "unavailable",
        )

    def test_three_current_cfar_returns_confirm_a_short_near_wall(self):
        estimator = ReflectivityCorridorEstimator()
        point_observations = cells([
            (0.223, 0.230, 0.525),
            (0.351, 0.185, 0.625),
            (0.413, 0.130, 0.567),
        ])

        for _ in range(3):
            estimator.ingest([], point_observations=point_observations)

        payload = estimator.boundaries_payload()
        self.assertEqual(payload["status"], "confirmed")
        self.assertEqual(len(payload["walls"]), 1)
        wall = payload["walls"][0]
        self.assertEqual(wall["side"], "right")
        self.assertEqual(wall["evidence_kind"], "point")
        self.assertEqual(wall["support_cells"], 3)
        self.assertGreater(wall["span_m"], 0.20)
        self.assertLess(wall["heading_deg"], -20.0)
        self.assertGreater(wall["heading_deg"], -35.0)

    def test_two_cfar_returns_cannot_invent_a_wall(self):
        estimator = ReflectivityCorridorEstimator()
        point_observations = cells([
            (0.223, 0.230, 0.95),
            (0.413, 0.130, 0.95),
        ])

        for _ in range(5):
            estimator.ingest([], point_observations=point_observations)

        self.assertEqual(
            estimator.boundaries_payload()["status"],
            "unavailable",
        )

    def test_point_boundary_is_mirror_equivariant(self):
        points = [
            (0.223, 0.230, 0.65),
            (0.351, 0.185, 0.65),
            (0.413, 0.130, 0.65),
        ]
        original = ReflectivityCorridorEstimator()
        mirrored = ReflectivityCorridorEstimator()
        for _ in range(3):
            original.ingest([], point_observations=cells(points))
            mirrored.ingest([], point_observations=cells([
                (forward, -lateral, intensity)
                for forward, lateral, intensity in points
            ]))

        right = original.boundaries_payload()["walls"][0]
        left = mirrored.boundaries_payload()["walls"][0]
        self.assertEqual(right["side"], "right")
        self.assertEqual(left["side"], "left")
        self.assertAlmostEqual(
            right["heading_deg"],
            -left["heading_deg"],
            places=6,
        )

    def test_live_single_diagonal_confirms_as_boundary_not_corridor(self):
        estimator = ReflectivityCorridorEstimator()
        observed = cells(LIVE_SINGLE_RIGHT_DIAGONAL)

        estimator.ingest(observed)
        self.assertEqual(estimator.boundaries_payload()["status"], "unavailable")
        estimator.ingest(observed)
        self.assertEqual(estimator.boundaries_payload()["status"], "unavailable")
        estimator.ingest(observed)

        boundary = estimator.boundaries_payload()
        self.assertEqual(estimator.payload()["status"], "unavailable")
        self.assertEqual(boundary["status"], "confirmed")
        self.assertEqual(boundary["method"], "independent_boundary_fit")
        self.assertEqual(len(boundary["walls"]), 1)
        wall = boundary["walls"][0]
        self.assertEqual(wall["side"], "right")
        self.assertEqual(wall["evidence_kind"], "heatmap")
        self.assertEqual(wall["hits"], 3)
        self.assertGreater(wall["heading_deg"], -60.0)
        self.assertLess(wall["heading_deg"], -55.0)
        self.assertGreater(wall["span_m"], 0.28)
        self.assertGreater(wall["normal_offset_m"], 0.0)

    def test_single_boundary_is_mirror_equivariant(self):
        original = ReflectivityCorridorEstimator()
        mirrored = ReflectivityCorridorEstimator()
        for _ in range(3):
            original.ingest(cells(LIVE_SINGLE_RIGHT_DIAGONAL))
            mirrored.ingest(cells([
                (forward, -lateral, intensity)
                for forward, lateral, intensity in LIVE_SINGLE_RIGHT_DIAGONAL
            ]))

        original_wall = original.boundaries_payload()["walls"][0]
        mirrored_wall = mirrored.boundaries_payload()["walls"][0]
        self.assertEqual(original_wall["side"], "right")
        self.assertEqual(mirrored_wall["side"], "left")
        self.assertAlmostEqual(
            original_wall["heading_deg"],
            -mirrored_wall["heading_deg"],
            places=6,
        )
        self.assertAlmostEqual(
            original_wall["normal_offset_m"],
            -mirrored_wall["normal_offset_m"],
            places=6,
        )

    def test_dense_varied_scatter_never_confirms_a_single_boundary(self):
        for seed, points_per_side in ((200, 30), (62, 80)):
            with self.subTest(seed=seed, points_per_side=points_per_side):
                estimator = ReflectivityCorridorEstimator()
                observed = cells(varied_quantized_scatter(seed, points_per_side))
                for _ in range(3):
                    estimator.ingest(observed)
                self.assertEqual(
                    estimator.boundaries_payload()["status"],
                    "unavailable",
                )

    def test_heatmap_only_live_reflections_cannot_invent_a_far_diagonal_pair(self):
        estimator = ReflectivityCorridorEstimator()

        estimator.ingest(cells(LIVE_CLUTTERED_RIGHT_CORRIDOR))
        payload = estimator.payload()

        self.assertEqual(payload["status"], "unavailable")
        self.assertEqual(payload["walls"], [])

    def test_strong_bilateral_heatmap_walls_confirm_without_point_returns(self):
        estimator = ReflectivityCorridorEstimator()
        observed = cells(STRONG_STRAIGHT_HEATMAP_CORRIDOR)

        for _ in range(3):
            estimator.ingest(observed)
        payload = estimator.payload()

        self.assertEqual(payload["status"], "confirmed")
        self.assertEqual(payload["geometry"], "parallel")
        self.assertEqual(payload["hits"], 3)
        self.assertAlmostEqual(payload["measured_heading_deg"], 0.0, places=6)
        self.assertAlmostEqual(payload["width_m"], 0.31, places=6)
        self.assertEqual(len(payload["walls"]), 2)
        self.assertLess(payload["walls"][0]["normal_offset_m"], 0.0)
        self.assertGreater(payload["walls"][1]["normal_offset_m"], 0.0)
        self.assertTrue(all(
            wall["support_cells"] >= 5 and wall["span_m"] >= 0.20
            for wall in payload["walls"]
        ))

    def test_converging_boundaries_are_fitted_independently_until_bottleneck(self):
        estimator = ReflectivityCorridorEstimator()
        observed = cells(STRONG_CONVERGING_HEATMAP_CORRIDOR)

        for _ in range(3):
            estimator.ingest(observed)
        payload = estimator.payload()

        self.assertEqual(payload["status"], "confirmed")
        self.assertEqual(payload["method"], "deterministic_boundary_fit")
        self.assertEqual(payload["geometry"], "converging")
        self.assertAlmostEqual(payload["measured_heading_deg"], 0.0, places=6)
        self.assertAlmostEqual(payload["guidance_heading_deg"], 0.0, places=6)
        self.assertAlmostEqual(payload["width_m"], 0.10, places=6)
        left_wall, right_wall = payload["walls"]
        self.assertGreater(left_wall["heading_deg"], 30.0)
        self.assertLess(right_wall["heading_deg"], -30.0)
        self.assertAlmostEqual(left_wall["start"][0], 0.20, places=6)
        self.assertAlmostEqual(right_wall["start"][0], 0.20, places=6)
        self.assertAlmostEqual(left_wall["end"][0], 0.40, places=6)
        self.assertAlmostEqual(right_wall["end"][0], 0.40, places=6)
        near_gap = right_wall["start"][1] - left_wall["start"][1]
        bottleneck_gap = right_wall["end"][1] - left_wall["end"][1]
        self.assertGreater(near_gap, bottleneck_gap)
        self.assertAlmostEqual(bottleneck_gap, payload["width_m"], places=6)

    def test_converging_fit_is_mirror_equivariant_not_direction_hardcoded(self):
        asymmetric = [
            (forward_m, lateral_m, 0.84)
            for forward_m, left_m, right_m in (
                (0.150, -0.250, 0.250),
                (0.200, -0.220, 0.190),
                (0.250, -0.190, 0.130),
                (0.300, -0.160, 0.070),
                (0.350, -0.130, 0.030),
            )
            for lateral_m in (left_m, right_m)
        ]
        original = ReflectivityCorridorEstimator()
        mirrored = ReflectivityCorridorEstimator()
        for _ in range(3):
            original.ingest(cells(asymmetric))
            mirrored.ingest(cells([
                (forward_m, -lateral_m, intensity)
                for forward_m, lateral_m, intensity in asymmetric
            ]))

        original_payload = original.payload()
        mirrored_payload = mirrored.payload()

        self.assertEqual(original_payload["geometry"], "converging")
        self.assertEqual(mirrored_payload["geometry"], "converging")
        self.assertAlmostEqual(
            original_payload["measured_heading_deg"],
            -mirrored_payload["measured_heading_deg"],
            places=6,
        )
        self.assertAlmostEqual(
            original_payload["width_m"],
            mirrored_payload["width_m"],
            places=6,
        )
        self.assertAlmostEqual(
            original_payload["walls"][0]["heading_deg"],
            -mirrored_payload["walls"][1]["heading_deg"],
            places=6,
        )

    def test_distributed_reopen_is_cut_at_the_measured_bottleneck(self):
        widths = (0.55, 0.45, 0.35, 0.25, 0.10, 0.15, 0.20, 0.25, 0.30)
        observed = cells([
            (0.200 + index * 0.025, lateral_m, 0.86)
            for index, width_m in enumerate(widths)
            for lateral_m in (-width_m / 2.0, width_m / 2.0)
        ])
        estimator = ReflectivityCorridorEstimator()
        for _ in range(3):
            estimator.ingest(observed)

        payload = estimator.payload()

        self.assertEqual(payload["geometry"], "converging")
        self.assertAlmostEqual(payload["width_m"], 0.10, places=6)
        self.assertTrue(all(
            math.isclose(float(wall["end"][0]), 0.30, abs_tol=1e-6)
            for wall in payload["walls"]
        ))

    def test_later_wider_lobe_cannot_replace_the_first_bottleneck(self):
        observed = cells([
            (forward_m, lateral_m, 0.86)
            for forward_m, width_m in (
                (0.200, 0.40),
                (0.225, 0.30),
                (0.250, 0.20),
                (0.275, 0.10),
                (0.375, 0.55),
                (0.400, 0.4875),
                (0.425, 0.425),
                (0.450, 0.3625),
                (0.475, 0.30),
            )
            for lateral_m in (-width_m / 2.0, width_m / 2.0)
        ])
        estimator = ReflectivityCorridorEstimator()
        for _ in range(3):
            estimator.ingest(observed)

        payload = estimator.payload()

        self.assertEqual(payload["geometry"], "converging")
        self.assertAlmostEqual(payload["width_m"], 0.10, places=6)
        self.assertTrue(all(
            math.isclose(float(wall["end"][0]), 0.275, abs_tol=1e-6)
            for wall in payload["walls"]
        ))

    def test_alternating_forward_ranges_cannot_combine_into_confirmation(self):
        near = [
            (forward_m, lateral_m, 0.83)
            for forward_m, half_width_m in (
                (0.150, 0.250),
                (0.200, 0.200),
                (0.250, 0.150),
                (0.300, 0.100),
                (0.350, 0.050),
            )
            for lateral_m in (-half_width_m, half_width_m)
        ]
        far = [
            (forward_m + 0.150, lateral_m, intensity)
            for forward_m, lateral_m, intensity in near
        ]
        estimator = ReflectivityCorridorEstimator()
        for observed in (near, far, near):
            estimator.ingest(cells(observed))

        payload = estimator.payload()

        self.assertEqual(payload["status"], "candidate")
        self.assertEqual(payload["hits"], 2)

    def test_zigzag_inner_returns_do_not_form_straight_measured_walls(self):
        observed = cells([
            (forward_m, lateral_m, 0.85)
            for forward_m, left_m, right_m in (
                (0.200, -0.250, 0.250),
                (0.250, -0.100, 0.300),
                (0.300, -0.200, 0.100),
                (0.350, -0.050, 0.150),
                (0.400, -0.050, 0.050),
            )
            for lateral_m in (left_m, right_m)
        ])
        estimator = ReflectivityCorridorEstimator()
        estimator.ingest(observed)

        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_independent_wall_headings_survive_inner_reflection_clutter(self):
        forward_rows = (0.200, 0.250, 0.300, 0.350, 0.400)
        measured_walls = [
            (forward_m, -0.250, 0.92)
            for forward_m in forward_rows
        ] + [
            (forward_m, 0.450 - forward_m, 0.91)
            for forward_m in forward_rows
        ]
        inner_clutter = [
            (forward_m, lateral_m, 0.64)
            for forward_m, lateral_m in zip(
                forward_rows,
                (-0.050, -0.150, -0.050, -0.150, -0.050),
            )
        ]
        estimator = ReflectivityCorridorEstimator()
        for _ in range(3):
            estimator.ingest(cells(measured_walls + inner_clutter))

        payload = estimator.payload()

        self.assertEqual(payload["status"], "confirmed")
        self.assertEqual(payload["geometry"], "converging")
        self.assertAlmostEqual(payload["walls"][0]["heading_deg"], 0.0)
        self.assertAlmostEqual(payload["walls"][1]["heading_deg"], -45.0)
        self.assertAlmostEqual(payload["walls"][0]["start"][0], 0.20)
        self.assertAlmostEqual(payload["walls"][1]["start"][0], 0.20)

    def test_dense_bilateral_scatter_cannot_invent_wall_boundaries(self):
        rng = random.Random(7)
        scatter = []
        for _ in range(20):
            scatter.append((
                rng.uniform(0.05, 0.55),
                rng.uniform(-0.32, -0.04),
                0.80,
            ))
            scatter.append((
                rng.uniform(0.05, 0.55),
                rng.uniform(0.04, 0.32),
                0.80,
            ))
        estimator = ReflectivityCorridorEstimator()
        for _ in range(3):
            estimator.ingest(cells(scatter))

        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_dense_quantized_scatter_cannot_invent_wall_boundaries(self):
        rng = random.Random(19)
        scatter = set()
        while len(scatter) < 40:
            side = -1 if len(scatter) % 2 == 0 else 1
            scatter.add((
                rng.randint(2, 20) * 0.025,
                side * rng.randint(2, 12) * 0.025,
                0.80,
            ))
        estimator = ReflectivityCorridorEstimator()
        for _ in range(3):
            estimator.ingest(cells(scatter))

        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_varied_bright_quantized_scatter_cannot_invent_boundaries(self):
        for seed in range(100):
            with self.subTest(seed=seed):
                scatter = varied_quantized_scatter(seed, 20)
                estimator = ReflectivityCorridorEstimator()
                for _ in range(3):
                    estimator.ingest(cells(scatter))
                self.assertEqual(
                    estimator.payload()["status"],
                    "unavailable",
                )

    def test_denser_scatter_cannot_form_accidental_independent_walls(self):
        estimator = ReflectivityCorridorEstimator()
        scatter = varied_quantized_scatter(200, 30)
        for _ in range(3):
            estimator.ingest(cells(scatter))

        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_denser_scatter_cannot_form_a_four_row_bottleneck(self):
        estimator = ReflectivityCorridorEstimator()
        scatter = varied_quantized_scatter(62, 80)
        for _ in range(3):
            estimator.ingest(cells(scatter))

        self.assertEqual(estimator.payload()["status"], "unavailable")

    def test_independent_fit_stays_inside_one_eight_hz_frame_budget(self):
        occupied_semicircle = [
            (forward_index * 0.025, lateral_index * 0.025, 0.80)
            for forward_index in range(21)
            for lateral_index in range(-20, 21)
            if math.hypot(
                forward_index * 0.025,
                lateral_index * 0.025,
            ) <= 0.5 + 1e-12
        ]
        self.assertEqual(len(occupied_semicircle), 649)

        estimator = ReflectivityCorridorEstimator()
        started = time.perf_counter()
        estimator.ingest(cells(occupied_semicircle))
        elapsed = time.perf_counter() - started

        self.assertLess(elapsed, 0.125)

    def test_point_seeded_geometry_precedes_heatmap_only_geometry(self):
        heatmap = [
            (forward_m, -0.25, 0.90)
            for forward_m in (0.10, 0.15, 0.20, 0.25, 0.30)
        ] + [
            (forward_m, 0.355 - 0.5 * forward_m, 0.90)
            for forward_m in (0.11, 0.16, 0.21, 0.26, 0.31)
        ]
        point_returns = [
            (0.10, -0.10, 0.95),
            (0.30, -0.20, 0.95),
        ]

        fit = ReflectivityCorridorEstimator._fit_pair(
            cells(heatmap),
            cells(point_returns),
        )

        self.assertIsNotNone(fit)
        self.assertEqual(fit["geometry"], "parallel")
        self.assertAlmostEqual(
            fit["measured_heading_deg"],
            -26.565051,
            places=6,
        )

    def test_independent_walls_keep_their_full_measured_extents(self):
        left = [
            (0.10 + index * 0.075, -0.20, 0.90)
            for index in range(5)
        ]
        right = [
            (0.20 + index * 0.075, 0.20, 0.90)
            for index in range(5)
        ]

        fit = ReflectivityCorridorEstimator._independent_heatmap_candidate(
            sorted(left + right),
        )

        self.assertIsNotNone(fit)
        self.assertEqual(fit["walls"][0]["start"][0], 0.10)
        self.assertEqual(fit["walls"][0]["end"][0], 0.40)
        self.assertEqual(fit["walls"][1]["start"][0], 0.20)
        self.assertEqual(fit["walls"][1]["end"][0], 0.50)

    def test_near_narrow_walls_beat_a_brighter_later_lobe(self):
        near = []
        far = []
        for index in range(5):
            near.append((0.075 + index * 0.050, -0.10, 0.72))
            near.append((0.100 + index * 0.050, 0.10, 0.72))
            far.append((0.325 + index * 0.050, -0.20, 0.96))
            far.append((0.350 + index * 0.050, 0.20, 0.96))

        fit = ReflectivityCorridorEstimator._independent_heatmap_candidate(
            sorted(near + far),
        )

        self.assertIsNotNone(fit)
        self.assertAlmostEqual(fit["width_m"], 0.20)
        self.assertLessEqual(max(
            float(wall["end"][0]) for wall in fit["walls"]
        ), 0.300001)

    def test_shallow_independent_narrowing_is_not_a_converging_corridor(self):
        left = [
            (forward, -0.20, 0.90)
            for forward in (0.10, 0.15, 0.20, 0.25, 0.30)
        ]
        right = [
            (forward, 0.30 - 0.20 * forward, 0.90)
            for forward in (0.11, 0.16, 0.21, 0.26, 0.31)
        ]
        estimator = ReflectivityCorridorEstimator()
        for _ in range(3):
            estimator.ingest(cells(left + right))

        self.assertEqual(estimator.payload()["status"], "unavailable")

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
                "geometry": "parallel",
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
            "method": "deterministic_boundary_fit",
            "geometry": None,
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
