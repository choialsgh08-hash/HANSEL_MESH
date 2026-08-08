import base64
from dataclasses import replace
import math
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from common.radar_geometry import RadarAxes
from common.sensor_contract import (
    RadarFrame,
    RadarHeatmap,
    RadarPoint,
    SensorHeader,
)
from monitor.radar_scene import RadarHazardEvaluator, RadarSceneEstimator
from sensors.mission_log import iter_replay
from sensors.radar_calibration import (
    ClutterPointCluster,
    RadarClutterModel,
    build_clutter_model,
)


PROFILE_ID = "radar-scene-test-profile"
CALIBRATION_ID = "radar-clutter-scene-test"
RANGE_BINS = 32
AZIMUTH_BINS = 16
RANGE_STEP_M = 0.09765625
REAL_FIXTURE = (
    Path(__file__).parent
    / "fixtures"
    / "radar_clutter_20260728_f2286_f2291.jsonl"
)


def load_real_fixture() -> tuple[RadarFrame, ...]:
    return tuple(
        entry.record
        for entry in iter_replay(REAL_FIXTURE)
        if isinstance(entry.record, RadarFrame)
    )


def bind_calibration(
    frames: tuple[RadarFrame, ...],
    calibration_id: str,
) -> tuple[RadarFrame, ...]:
    return tuple(
        replace(
            frame,
            header=replace(frame.header, calibration_id=calibration_id),
        )
        for frame in frames
    )


def clutter_model() -> RadarClutterModel:
    cells = RANGE_BINS * AZIMUTH_BINS
    return RadarClutterModel(
        schema_version=1,
        calibration_id=CALIBRATION_ID,
        profile_id=PROFILE_ID,
        axes=RadarAxes(),
        range_bins=RANGE_BINS,
        azimuth_bins=AZIMUTH_BINS,
        range_step_m=RANGE_STEP_M,
        motion_mode="major",
        point_clusters=(),
        heatmap_median_db=(20.0,) * cells,
        heatmap_mad_db=(1.0,) * cells,
    )


def calibrated_estimator() -> RadarSceneEstimator:
    return RadarSceneEstimator(RadarAxes(), clutter_model())


def uncalibrated_estimator() -> RadarSceneEstimator:
    return RadarSceneEstimator(RadarAxes())


def mismatched_estimator() -> RadarSceneEstimator:
    model = RadarClutterModel(
        **{
            **clutter_model().__dict__,
            "profile_id": "different-profile",
        }
    )
    return RadarSceneEstimator(RadarAxes(), model)


def radar_frame(
    points=(),
    heatmap=None,
    *,
    producer_id="radar-scene-producer",
    frame_number=1,
    frame_transition="consecutive",
    profile_id=PROFILE_ID,
    calibration_id=CALIBRATION_ID,
    dropped_frames_since_previous=0,
) -> RadarFrame:
    return RadarFrame(
        header=SensorHeader(
            mission_id="radar-scene-test",
            unit_id="head",
            boot_id="boot-1",
            producer_id=producer_id,
            stream_id="radar/front",
            seq=frame_number,
            monotonic_ns=frame_number * 100_000_000,
            frame_id="radar_native",
            calibration_id=calibration_id,
        ),
        frame_number=frame_number,
        subframe_number=0,
        complete=True,
        dropped_frames_since_previous=dropped_frames_since_previous,
        points=tuple(points),
        source_format="synthetic",
        frame_transition=frame_transition,
        profile_id=profile_id,
        heatmap=heatmap,
    )


def point_scan(count: int, offset: float, frame_number: int) -> RadarFrame:
    positions = (
        (0.28, -0.30, -0.12),
        (0.34, -0.15, -0.04),
        (0.42, 0.00, 0.02),
        (0.36, 0.16, 0.08),
        (0.25, 0.28, 0.14),
    )
    return radar_frame(tuple(
        RadarPoint(lateral, forward + offset, height, 0.0, 24.0)
        for forward, lateral, height in positions[:count]
    ), frame_number=frame_number)


def five_point_scan(offset: float, frame_number: int) -> RadarFrame:
    return point_scan(5, offset, frame_number)


def seed_probability_cell(
    estimator: RadarSceneEstimator,
    producer_id: str = "radar-scene-producer",
    start: float = 1.0,
) -> None:
    for frame_number in range(1, 4):
        estimator.ingest(
            point_frame(
                0.30,
                frame_number=frame_number,
                snr_db=24.0,
                producer_id=producer_id,
            ),
            received_at=start + frame_number * 0.125,
        )


def assert_probability_maps_empty(
    test_case: unittest.TestCase,
    scene: dict[str, object],
) -> None:
    test_case.assertEqual(scene["probabilistic_local_map"]["cells"], [])
    test_case.assertEqual(scene["rolling_local_map"]["cells"], [])


def confirmed_probability_scene_fixture() -> dict[str, object]:
    estimator = calibrated_estimator()
    seed_probability_cell(estimator)
    return estimator.snapshot(now=1.375)


def frame_with_forward_ranges(*ranges_m: float) -> RadarFrame:
    return radar_frame(
        RadarPoint(
            x_m=0.0,
            y_m=range_m,
            z_m=0.0,
            radial_velocity_mps=0.0,
            snr_db=20.0,
        )
        for range_m in ranges_m
    )


def heatmap_only_frame(
    range_index: int,
    azimuth_index: int,
    *,
    frame_number: int = 1,
) -> RadarFrame:
    data = bytearray(RANGE_BINS * AZIMUTH_BINS)
    data[range_index * AZIMUTH_BINS + azimuth_index] = 255
    return radar_frame(
        heatmap=RadarHeatmap(
            data=bytes(data),
            range_bins=RANGE_BINS,
            azimuth_bins=AZIMUTH_BINS,
            range_step_m=RANGE_STEP_M,
            tlv_type=304,
            motion_mode="major",
            floor_db=10.0,
            ceiling_db=110.0,
        ),
        frame_number=frame_number,
    )


def clustered_heatmap_frame(
    range_index: int,
    *,
    frame_number: int,
    with_point: bool,
    value: int = 255,
) -> RadarFrame:
    data = bytearray(RANGE_BINS * AZIMUTH_BINS)
    for azimuth_index in (8, 9):
        data[range_index * AZIMUTH_BINS + azimuth_index] = value
    points = (
        (
            RadarPoint(
                x_m=0.0,
                y_m=range_index * RANGE_STEP_M,
                z_m=0.01,
                radial_velocity_mps=0.0,
                snr_db=24.0,
            ),
        )
        if with_point
        else ()
    )
    return radar_frame(
        points,
        heatmap=RadarHeatmap(
            data=bytes(data),
            range_bins=RANGE_BINS,
            azimuth_bins=AZIMUTH_BINS,
            range_step_m=RANGE_STEP_M,
            tlv_type=304,
            motion_mode="major",
            floor_db=10.0,
            ceiling_db=110.0,
        ),
        frame_number=frame_number,
    )


def split_heatmap_frame(*, frame_number: int) -> RadarFrame:
    range_index = 4
    data = bytearray(RANGE_BINS * AZIMUTH_BINS)
    for azimuth_index in (6, 9):
        data[range_index * AZIMUTH_BINS + azimuth_index] = 255
    return radar_frame(
        (
            RadarPoint(
                x_m=0.0,
                y_m=range_index * RANGE_STEP_M,
                z_m=0.01,
                radial_velocity_mps=0.0,
                snr_db=24.0,
            ),
        ),
        heatmap=RadarHeatmap(
            data=bytes(data),
            range_bins=RANGE_BINS,
            azimuth_bins=AZIMUTH_BINS,
            range_step_m=RANGE_STEP_M,
            tlv_type=304,
            motion_mode="major",
            floor_db=10.0,
            ceiling_db=110.0,
        ),
        frame_number=frame_number,
    )


def point_frame(
    forward_m: float,
    *,
    frame_number: int = 1,
    snr_db: float = 20.0,
    **frame_kwargs,
) -> RadarFrame:
    return radar_frame(
        (
            RadarPoint(
                x_m=0.0,
                y_m=forward_m,
                z_m=0.01,
                radial_velocity_mps=0.0,
                snr_db=snr_db,
            ),
        ),
        frame_number=frame_number,
        **frame_kwargs,
    )


def empty_frame(*, frame_number: int = 1) -> RadarFrame:
    return radar_frame(frame_number=frame_number)


def saturated_near_heatmap_frame(frame_index: int) -> RadarFrame:
    data = bytearray(RANGE_BINS * AZIMUTH_BINS)
    for range_index in (0, 1):
        start = range_index * AZIMUTH_BINS
        data[start : start + AZIMUTH_BINS] = b"\xff" * AZIMUTH_BINS
    return radar_frame(
        heatmap=RadarHeatmap(
            data=bytes(data),
            range_bins=RANGE_BINS,
            azimuth_bins=AZIMUTH_BINS,
            range_step_m=RANGE_STEP_M,
            tlv_type=304,
            motion_mode="major",
            floor_db=10.0,
            ceiling_db=110.0,
        ),
        frame_number=frame_index + 1,
    )


class RadarSceneEstimatorTests(unittest.TestCase):
    def test_existing_positional_clock_argument_remains_compatible(self):
        clock = lambda: 4.0
        estimator = RadarSceneEstimator(RadarAxes(), None, True, clock)
        estimator.ingest(point_frame(0.30))

        scene = estimator.snapshot()

        self.assertEqual(
            scene["calibration_status"],
            "calibration_required",
        )
        self.assertFalse(scene["tracks"])

    def test_untrusted_profile_frames_cannot_seed_next_trusted_snapshot(self):
        estimator = calibrated_estimator()
        wrong_profile_point = lambda frame_number: radar_frame(
            (
                RadarPoint(
                    x_m=0.0,
                    y_m=0.20,
                    z_m=0.01,
                    radial_velocity_mps=0.0,
                    snr_db=20.0,
                ),
            ),
            frame_number=frame_number,
            profile_id="wrong-profile",
        )
        estimator.ingest(wrong_profile_point(1), received_at=4.0)
        estimator.ingest(wrong_profile_point(2), received_at=4.1)
        estimator.ingest(empty_frame(frame_number=3), received_at=4.2)

        scene = estimator.snapshot(now=4.2)

        self.assertEqual(scene["calibration_status"], "ok")
        self.assertFalse(scene["tracks"])
        self.assertEqual(scene["hazard"]["level"], "UNKNOWN")
        self.assertEqual(scene["diagnostics"]["scene_point_count"], 0)

    def test_untrusted_binding_clears_existing_trusted_evidence(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.20, frame_number=1), received_at=4.0)
        estimator.ingest(point_frame(0.20, frame_number=2), received_at=4.1)
        self.assertEqual(
            estimator.snapshot(now=4.1)["hazard"]["level"],
            "NORMAL",
        )

        estimator.ingest(
            radar_frame(
                (
                    RadarPoint(
                        x_m=0.0,
                        y_m=0.20,
                        z_m=0.01,
                        radial_velocity_mps=0.0,
                        snr_db=20.0,
                    ),
                ),
                frame_number=3,
                profile_id="wrong-profile",
            ),
            received_at=4.2,
        )

        scene = estimator.snapshot(now=4.2)
        self.assertEqual(scene["calibration_status"], "profile_mismatch")
        self.assertFalse(scene["tracks"])
        self.assertEqual(scene["hazard"]["level"], "UNKNOWN")
        self.assertEqual(scene["diagnostics"]["scene_point_count"], 0)

    def test_synthetic_mode_authorizes_confirmed_demo_point_hazard(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator.ingest(point_frame(0.09, frame_number=1), received_at=5.0)
        estimator.ingest(point_frame(0.09, frame_number=2), received_at=5.1)

        scene = estimator.snapshot(now=5.1)

        self.assertEqual(scene["calibration_status"], "synthetic")
        self.assertTrue(scene["tracks"][0]["point_confirmed"])
        self.assertEqual(scene["hazard"]["level"], "DANGER")

    def test_hazard_evaluator_releases_latch_at_track_age_300ms(self):
        evaluator = RadarHazardEvaluator()
        track = {
            "track_id": 7,
            "forward_m": 0.09,
            "lateral_m": 0.0,
            "height_m": 0.0,
            "distance_m": 0.09,
            "range_uncertainty_m": 0.0,
            "confidence": 1.0,
            "source": "point",
            "point_confirmed": True,
            "age_ms": 299,
        }
        entered = evaluator.update([track], "ok", "live", now=90.299)
        self.assertEqual(entered["level"], "DANGER")
        expired = evaluator.update(
            [{**track, "age_ms": 300}],
            "ok",
            "live",
            now=90.300,
        )
        self.assertEqual(expired["level"], "UNKNOWN")
        self.assertIsNone(expired["nearest_confirmed_m"])

    def test_hazard_evaluator_rejects_confirmed_heatmap_track(self):
        evaluator = RadarHazardEvaluator()
        heatmap_track = {
            "track_id": 8,
            "forward_m": 0.09,
            "lateral_m": 0.0,
            "height_m": None,
            "distance_m": 0.09,
            "range_uncertainty_m": RANGE_STEP_M,
            "confidence": 1.0,
            "source": "heatmap",
            "point_confirmed": True,
            "age_ms": 0,
        }
        hazard = evaluator.update(
            [heatmap_track],
            "ok",
            "live",
            now=91.0,
        )
        self.assertEqual(hazard["level"], "UNKNOWN")
        self.assertIsNone(hazard["nearest_confirmed_m"])

    def test_three_metre_grid_retains_049_and_299_but_rejects_301(self):
        estimator = calibrated_estimator()
        estimator.ingest(
            frame_with_forward_ranges(0.49, 2.99, 3.01),
            received_at=1.0,
        )
        estimator.ingest(
            frame_with_forward_ranges(0.49, 2.99, 3.01),
            received_at=1.1,
        )
        scene = estimator.snapshot(now=1.1)
        distances = [track["distance_m"] for track in scene["tracks"]]
        self.assertTrue(any(abs(value - 0.49) < 0.01 for value in distances))
        self.assertTrue(any(abs(value - 2.99) < 0.01 for value in distances))
        self.assertFalse(any(value > 3.0 for value in distances))
        grid = base64.b64decode(scene["grid"]["data_base64"])
        self.assertEqual(len(grid), 60 * 60)
        self.assertGreater(grid[59 * 60 + 30], 0)

    def test_exact_three_metres_maps_to_final_forward_row(self):
        estimator = calibrated_estimator()
        estimator.ingest(frame_with_forward_ranges(3.0), received_at=2.0)
        estimator.ingest(frame_with_forward_ranges(3.0), received_at=2.1)
        scene = estimator.snapshot(now=2.1)
        self.assertAlmostEqual(scene["tracks"][0]["forward_m"], 3.0)
        grid = base64.b64decode(scene["grid"]["data_base64"])
        self.assertGreater(grid[59 * 60 + 30], 0)

    def test_grid_zero_is_unknown_and_layout_is_explicit(self):
        scene = calibrated_estimator().snapshot()
        self.assertEqual(scene["grid"]["unknown_value"], 0)
        self.assertEqual(
            scene["grid"]["layout"],
            "forward-major_lateral-minor",
        )
        self.assertEqual(set(base64.b64decode(scene["grid"]["data_base64"])), {0})

    def test_scene_v1_declares_robot_relative_pose_and_exact_grid_geometry(self):
        scene = calibrated_estimator().snapshot()

        self.assertEqual(scene["schema_version"], 1)
        self.assertEqual(scene["pose_mode"], "robot_relative_short_memory")
        self.assertEqual(
            {
                key: scene["grid"][key]
                for key in (
                    "resolution_m",
                    "forward_cells",
                    "lateral_cells",
                    "origin_forward_cell",
                    "origin_lateral_cell",
                )
            },
            {
                "resolution_m": 0.05,
                "forward_cells": 60,
                "lateral_cells": 60,
                "origin_forward_cell": 0,
                "origin_lateral_cell": 30,
            },
        )

    def test_two_hit_point_ghost_never_enters_probability_map(self):
        estimator = calibrated_estimator()
        estimator.ingest(
            point_frame(0.30, frame_number=1, snr_db=24.0),
            received_at=1.000,
        )
        estimator.ingest(
            point_frame(0.30, frame_number=2, snr_db=24.0),
            received_at=1.125,
        )

        self.assertEqual(
            estimator.snapshot(now=1.125)["probabilistic_local_map"]["cells"],
            [],
        )

    def test_two_weak_hits_plus_one_strong_hit_never_enter_probability_map(self):
        estimator = calibrated_estimator()
        for frame_number, snr_db in ((1, 15.0), (2, 15.0), (3, 24.0)):
            estimator.ingest(
                point_frame(0.30, frame_number=frame_number, snr_db=snr_db),
                received_at=1.0 + frame_number * 0.125,
            )

        scene = estimator.snapshot(now=1.375)

        self.assertTrue(scene["tracks"][0]["point_confirmed"])
        self.assertFalse(scene["tracks"][0]["map_confirmed"])
        self.assertEqual(scene["probabilistic_local_map"]["cells"], [])

    def test_three_of_five_point_enters_probability_map(self):
        local_map = confirmed_probability_scene_fixture()[
            "probabilistic_local_map"
        ]

        self.assertEqual(local_map["resolution_m"], 0.05)
        self.assertEqual(local_map["occupied_threshold"], 0.75)
        occupied = [cell for cell in local_map["cells"] if cell["occupied"]]
        self.assertEqual(len(occupied), 1)
        self.assertAlmostEqual(occupied[0]["probability"], 0.80, places=3)
        self.assertEqual(occupied[0]["hits"], 1)

    def test_confirmed_heatmap_enters_probability_map_without_point_map_confirmation(self):
        estimator = calibrated_estimator()

        for frame_number in range(1, 4):
            frame = clustered_heatmap_frame(
                4,
                frame_number=frame_number,
                with_point=True,
            )
            weak_point = replace(frame.points[0], snr_db=18.0)
            estimator.ingest(
                replace(frame, points=(weak_point,)),
                received_at=6.0 + frame_number * 0.1,
            )

        scene = estimator.snapshot(now=6.3)
        point_track = next(
            track for track in scene["tracks"] if track["source"] == "point"
        )
        heatmap_track = next(
            track for track in scene["tracks"] if track["source"] == "heatmap"
        )

        self.assertEqual(point_track["quality_score"], 0.60)
        self.assertFalse(point_track["map_confirmed"])
        self.assertTrue(heatmap_track["heatmap_confirmed"])
        self.assertEqual(
            heatmap_track["classification"],
            "confirmed_obstacle",
        )

        cells = scene["probabilistic_local_map"]["cells"]
        self.assertEqual(len(cells), 1)
        self.assertEqual(cells[0]["source"], "heatmap")
        self.assertAlmostEqual(cells[0]["probability"], 0.80, places=3)
        self.assertEqual(cells[0]["hits"], 1)

    def test_probability_map_decays_and_expires_without_new_ingest(self):
        estimator = calibrated_estimator()
        seed_probability_cell(estimator)

        self.assertFalse(
            estimator.snapshot(now=2.375)["probabilistic_local_map"]["cells"][0][
                "occupied"
            ]
        )
        self.assertEqual(
            estimator.snapshot(now=3.875)["probabilistic_local_map"]["cells"],
            [],
        )

    def test_rolling_map_is_exact_occupied_alias(self):
        scene = confirmed_probability_scene_fixture()
        occupied = [
            cell
            for cell in scene["probabilistic_local_map"]["cells"]
            if cell["occupied"]
        ]

        self.assertEqual(scene["rolling_local_map"]["cells"], occupied)
        self.assertEqual(
            scene["rolling_local_map"]["retention"],
            "probability_decay_with_ttl",
        )

    def test_raw_and_filter_stage_counts_have_distinct_semantics(self):
        model = RadarClutterModel(
            **{
                **clutter_model().__dict__,
                "point_clusters": (
                    ClutterPointCluster(
                        forward_m=0.10,
                        lateral_m=0.0,
                        height_m=0.0,
                        radius_m=0.03,
                        observation_fraction=1.0,
                    ),
                ),
            }
        )
        estimator = RadarSceneEstimator(RadarAxes(), model)
        estimator.ingest(
            radar_frame(
                (
                    RadarPoint(0.0, 0.10, 0.0, 0.0, 24.0),
                    RadarPoint(0.0, 0.30, 0.0, 0.0, 3.0),
                    RadarPoint(0.0, 0.40, 0.0, 0.0, 24.0),
                )
            ),
            received_at=1.0,
        )

        diagnostics = estimator.snapshot(now=1.0)["diagnostics"]

        self.assertEqual(diagnostics["raw_point_count"], 3)
        self.assertEqual(diagnostics["self_clutter_rejected_count"], 1)
        self.assertEqual(diagnostics["quality_gate_rejected_count"], 1)
        self.assertEqual(diagnostics["scene_point_count"], 1)

    def test_calibration_rejected_frame_still_reports_raw_input_count(self):
        estimator = mismatched_estimator()
        estimator.ingest(
            radar_frame(
                (
                    RadarPoint(0.0, 0.30, 0.0, 0.0, 24.0),
                    RadarPoint(0.0, 0.40, 0.0, 0.0, 24.0),
                )
            ),
            received_at=1.0,
        )

        scene = estimator.snapshot(now=1.0)

        self.assertEqual(scene["calibration_status"], "profile_mismatch")
        self.assertEqual(scene["diagnostics"]["raw_point_count"], 2)
        self.assertEqual(
            scene["diagnostics"]["self_clutter_rejected_count"],
            0,
        )
        self.assertEqual(
            scene["diagnostics"]["quality_gate_rejected_count"],
            0,
        )

    def test_heatmap_never_invents_height(self):
        estimator = calibrated_estimator()
        estimator.ingest(heatmap_only_frame(range_index=4, azimuth_index=8))
        track = estimator.snapshot()["tracks"][0]
        self.assertEqual(track["source"], "heatmap")
        self.assertIsNone(track["height_m"])
        self.assertAlmostEqual(track["range_uncertainty_m"], 0.09765625)
        self.assertGreaterEqual(track["lateral_uncertainty_m"], 0.05)

    def test_point_elevation_is_preserved_but_not_used_for_2d_projection(self):
        estimator = calibrated_estimator()
        frame = radar_frame(
            (
                RadarPoint(
                    x_m=0.0,
                    y_m=0.40,
                    z_m=0.75,
                    radial_velocity_mps=0.0,
                    snr_db=20.0,
                ),
            )
        )
        estimator.ingest(frame, received_at=3.0)

        scene = estimator.snapshot(now=3.0)

        self.assertEqual(scene["projection_mode"], "range_azimuth_2d")
        self.assertFalse(scene["elevation_used"])
        self.assertEqual(scene["tracks"][0]["height_m"], 0.75)
        self.assertEqual(scene["tracks"][0]["height_band"], "above_sensor")
        self.assertEqual(
            scene["tracks"][0]["height_reference"],
            "sensor_relative_uncalibrated",
        )

    def test_point_map_confirmation_is_three_of_five_while_hazard_remains_two_of_three(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.09, frame_number=1, snr_db=24.0), received_at=1.000)
        estimator.ingest(point_frame(0.09, frame_number=2, snr_db=24.0), received_at=1.125)
        track = estimator.snapshot(now=1.125)["tracks"][0]
        self.assertTrue(track["point_confirmed"])
        self.assertFalse(track["map_confirmed"])
        self.assertEqual(estimator.snapshot(now=1.125)["hazard"]["level"], "DANGER")
        estimator.ingest(point_frame(0.09, frame_number=3, snr_db=24.0), received_at=1.250)
        self.assertTrue(estimator.snapshot(now=1.250)["tracks"][0]["map_confirmed"])

    def test_weak_hits_confirm_hazard_history_but_not_strong_map_history(self):
        estimator = calibrated_estimator()
        for frame_number, snr_db in ((1, 15.0), (2, 15.0), (3, 24.0)):
            estimator.ingest(
                point_frame(0.30, frame_number=frame_number, snr_db=snr_db),
                received_at=1.0 + frame_number * 0.125,
            )
        track = estimator.snapshot(now=1.375)["tracks"][0]
        self.assertTrue(track["point_confirmed"])
        self.assertFalse(track["map_confirmed"])
        for frame_number in (4, 5):
            estimator.ingest(
                point_frame(0.30, frame_number=frame_number, snr_db=24.0),
                received_at=1.0 + frame_number * 0.125,
            )
        self.assertTrue(estimator.snapshot(now=1.625)["tracks"][0]["map_confirmed"])

    def test_filtered_point_preserves_measured_height_and_band(self):
        estimator = calibrated_estimator()
        point = RadarPoint(0.0, 0.40, 0.12, 0.0, 24.0, 81.0)
        estimator.ingest(radar_frame((point,)), received_at=3.0)
        track = estimator.snapshot(now=3.0)["tracks"][0]
        self.assertEqual(track["height_m"], 0.12)
        self.assertEqual(track["height_band"], "above_sensor")
        self.assertEqual(track["height_reference"], "sensor_relative_uncalibrated")
        self.assertEqual((track["snr_db"], track["noise_db"]), (24.0, 81.0))

    def test_one_frame_position_spike_does_not_move_filtered_track(self):
        estimator = calibrated_estimator()
        for index, forward in enumerate((0.40, 0.41, 0.39, 0.40), 1):
            estimator.ingest(point_frame(forward, frame_number=index), received_at=1.0 + index * 0.1)
        received_before_outlier = estimator._tracks[0]["_received_at_ns"]
        estimator.ingest(radar_frame((
            RadarPoint(0.13, 0.45, 0.40, 0.0, 24.0),
        ), frame_number=5), received_at=1.5)
        track = estimator.snapshot(now=1.5)["tracks"][0]
        self.assertAlmostEqual(track["forward_m"], 0.40, delta=0.02)
        self.assertGreaterEqual(track["outlier_rejections"], 1)
        self.assertEqual(estimator._tracks[0]["_received_at_ns"], received_before_outlier)
        self.assertFalse(estimator._tracks[0]["_updated_by_point"])
        self.assertGreater(track["age_ms"], 0)

    def test_motion_transform_moves_point_history_before_aligned_observation(self):
        estimator = calibrated_estimator()
        for frame_number in range(1, 5):
            estimator.ingest(
                point_frame(0.40, frame_number=frame_number),
                received_at=1.0 + frame_number * 0.1,
            )
        estimator._transform_tracks(0.10, 0.0, 0.0)
        estimator.ingest(
            point_frame(0.50, frame_number=5), received_at=1.5,
        )
        track = estimator.snapshot(now=1.5)["tracks"][0]
        self.assertAlmostEqual(track["forward_m"], 0.50, places=3)

    def test_motion_transform_releases_danger_during_missed_window(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.09, frame_number=1), received_at=2.0)
        estimator.ingest(point_frame(0.09, frame_number=2), received_at=2.1)
        self.assertEqual(estimator.snapshot(now=2.1)["hazard"]["level"], "DANGER")
        estimator._transform_tracks(0.10, 0.0, 0.0)
        estimator.ingest(empty_frame(frame_number=3), received_at=2.2)
        hazard = estimator.snapshot(now=2.2)["hazard"]
        self.assertEqual(hazard["level"], "NORMAL")
        self.assertAlmostEqual(hazard["nearest_confirmed_m"], 0.19)

    def test_four_dropped_windows_clear_five_frame_map_confirmation(self):
        estimator = calibrated_estimator()
        for frame_number in range(1, 4):
            estimator.ingest(point_frame(0.30, frame_number=frame_number, snr_db=24.0), received_at=1.0 + frame_number * 0.125)
        estimator.ingest(radar_frame((), frame_number=8, dropped_frames_since_previous=4), received_at=1.5)
        track = estimator.snapshot(now=1.5)["tracks"][0]
        self.assertFalse(track["map_confirmed"])
        self.assertFalse(track["point_confirmed"])

    def test_trusted_scene_without_verified_range_step_rejects_point(self):
        class TrustedWithoutRangeStep(RadarSceneEstimator):
            def _binding_status(self, frame):
                return "ok"

        estimator = TrustedWithoutRangeStep(RadarAxes(), require_calibration=False, synthetic=False)
        estimator.ingest(radar_frame(
            (RadarPoint(0.0, 0.30, 0.0, 0.0, 24.0),),
            frame_number=1, profile_id="unknown-profile",
        ), received_at=1.0)
        scene = estimator.snapshot(now=1.0)
        self.assertEqual(scene["tracks"], [])
        self.assertEqual(scene["diagnostics"]["quality_gate_rejected_count"], 1)

    def test_tentative_raw_points_cannot_move_probability_map(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator.ingest(five_point_scan(0.0, 1), received_at=1.000)
        estimator.ingest(five_point_scan(-0.02, 2), received_at=1.125)
        motion = estimator.snapshot(now=1.125)["motion_compensation"]
        self.assertFalse(motion["applied"])
        self.assertEqual(motion["matched_points"], 0)

    def test_rejected_four_anchor_motion_never_calls_map_transform(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        for frame_number in range(1, 4):
            estimator.ingest(point_scan(4, 0.0, frame_number),
                             received_at=1.0 + frame_number * 0.125)
        before = sorted(
            (cell["forward_m"], cell["lateral_m"])
            for cell in estimator.snapshot(now=1.375)["rolling_local_map"]["cells"]
        )
        real_transform = estimator._probability_grid.transform
        estimator._probability_grid.transform = Mock(wraps=real_transform)
        estimator.ingest(point_scan(4, -0.02, 4), received_at=1.500)
        scene = estimator.snapshot(now=1.500)
        after = sorted(
            (cell["forward_m"], cell["lateral_m"])
            for cell in scene["rolling_local_map"]["cells"]
        )
        self.assertEqual(scene["motion_compensation"]["status"], "insufficient_points")
        estimator._probability_grid.transform.assert_not_called()
        self.assertEqual(after, before)

    def test_only_stable_five_match_motion_transforms_short_memory_map(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        for frame_number in range(1, 4):
            estimator.ingest(five_point_scan(0.0, frame_number),
                             received_at=1.0 + frame_number * 0.125)
        before = sorted(cell["forward_m"] for cell in
                        estimator.snapshot(now=1.375)["rolling_local_map"]["cells"])
        estimator.ingest(five_point_scan(-0.02, 4), received_at=1.500)
        estimator.ingest(five_point_scan(-0.04, 5), received_at=1.625)
        scene = estimator.snapshot(now=1.625)
        after = sorted(cell["forward_m"] for cell in scene["rolling_local_map"]["cells"])
        self.assertTrue(scene["motion_compensation"]["applied"])
        self.assertEqual(scene["motion_compensation"]["matched_points"], 5)
        self.assertLess(min(after), min(before))
        self.assertEqual(
            sorted(round(track["height_m"], 2) for track in scene["tracks"]),
            [-0.12, -0.04, 0.02, 0.08, 0.14],
        )

    def test_near_cell_transform_collision_merges_uncertainty_and_source(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator._near_cells = {
            (1, -8): {
                "forward_m": 0.025, "lateral_m": -0.200,
                "range_uncertainty_m": 0.08, "lateral_uncertainty_m": 0.03,
                "confidence": 0.90, "source": "point", "hits": 2,
                "reactive": True, "_last_seen_ns": 1_000_000_000,
            },
            (1, -7): {
                "forward_m": 0.025, "lateral_m": -0.175,
                "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.09,
                "confidence": 0.95, "source": "heatmap", "hits": 5,
                "reactive": True, "_last_seen_ns": 1_100_000_000,
            },
        }
        estimator._transform_tracks(0.0, 0.0, math.radians(15.0))
        cell = estimator._near_obstacle_payload(1.1)[0]
        self.assertEqual(len(estimator._near_cells), 1)
        self.assertEqual(cell["range_uncertainty_m"], 0.08)
        self.assertEqual(cell["lateral_uncertainty_m"], 0.09)
        self.assertEqual(cell["confidence"], 0.95)
        self.assertEqual(cell["source"], "mixed")
        self.assertEqual(cell["hits"], 5)

    def test_travel_trail_moves_behind_robot_and_keeps_current_center(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)

        estimator._transform_tracks(-0.10, 0.0, 0.0)
        trail = estimator.snapshot(now=1.0)["travel_trail"]

        self.assertEqual(trail["source"], "radar_ego_motion")
        self.assertEqual(trail["radius_m"], 0.50)
        self.assertEqual(trail["resolution_m"], 0.01)
        self.assertEqual(trail["points"], [[-0.1, 0.0], [0.0, 0.0]])

    def test_travel_trail_rotates_and_drops_points_outside_circle(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator._travel_trail = [(-0.45, 0.0), (-0.20, 0.0), (0.0, 0.0)]

        estimator._transform_tracks(0.0, 0.0, math.radians(90.0))
        rotated = estimator._travel_trail_payload()

        self.assertAlmostEqual(rotated[0][0], 0.0, delta=0.0001)
        self.assertAlmostEqual(rotated[0][1], -0.45, delta=0.0001)
        estimator._transform_tracks(-0.30, 0.0, 0.0)
        retained = estimator._travel_trail_payload()
        self.assertTrue(
            all(math.hypot(point[0], point[1]) <= 0.50 for point in retained)
        )
        self.assertEqual(retained[-1], [0.0, 0.0])

    def test_single_high_confidence_point_does_not_enter_path(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator.ingest(radar_frame((RadarPoint(0.0, 0.35, 0.0, 0.0, 30.0),),
                                     frame_number=1), received_at=1.0)
        scene = estimator.snapshot(now=1.0)

        self.assertEqual(scene["near_occupancy"]["cells"], [])
        self.assertEqual(scene["planned_path"]["curvature_per_m"], 0.0)

    def test_path_uncertainty_is_added_to_robot_width_margin(self):
        obstacle = {
            "forward_m": 0.30, "lateral_m": 0.18,
            "range_uncertainty_m": 0.04, "lateral_uncertainty_m": 0.08,
            "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
        }
        path = RadarSceneEstimator._plan_path([obstacle])
        self.assertEqual(path["robot_width_m"], 0.15)
        self.assertEqual(path["safety_margin_m"], 0.03)
        self.assertEqual(path["minimum_passage_width_m"], 0.21)
        self.assertEqual(path["range_uncertainty_lateral_inflation_m"], 0.08)
        self.assertEqual(path["obstacle_support_radius_m"], 0.0)
        self.assertAlmostEqual(
            path["collision_grid_padding_m"], 0.025 / math.sqrt(2),
        )
        self.assertAlmostEqual(
            path["maximum_collision_radius_m"],
            0.15 / 2 + 0.03 + 0.08 + 0.025 / math.sqrt(2),
        )

    def test_probability_cell_adds_its_half_diagonal_support(self):
        obstacle = {
            "forward_m": 0.30, "lateral_m": 0.18,
            "range_uncertainty_m": 0.04, "lateral_uncertainty_m": 0.08,
            "support_radius_m": 0.05 / math.sqrt(2),
            "evidence_kind": "probability", "source": "point",
        }
        path = RadarSceneEstimator._plan_path([obstacle])
        self.assertAlmostEqual(
            path["maximum_collision_radius_m"],
            0.15 / 2 + 0.03 + 0.08
            + 0.05 / math.sqrt(2) + 0.025 / math.sqrt(2),
        )

    def test_maximum_collision_radius_uses_one_obstacles_combined_terms(self):
        obstacles = [
            {"forward_m": 0.30, "lateral_m": -0.20,
             "range_uncertainty_m": 0.10, "lateral_uncertainty_m": 0.02,
             "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point"},
            {"forward_m": 0.30, "lateral_m": 0.20,
             "range_uncertainty_m": 0.01, "lateral_uncertainty_m": 0.01,
             "support_radius_m": 0.07, "evidence_kind": "direct", "source": "point"},
        ]
        path = RadarSceneEstimator._plan_path(obstacles)
        self.assertEqual(path["range_uncertainty_lateral_inflation_m"], 0.10)
        self.assertEqual(path["obstacle_support_radius_m"], 0.07)
        self.assertAlmostEqual(
            path["maximum_collision_radius_m"],
            0.15 / 2 + 0.03 + 0.10 + 0.025 / math.sqrt(2),
        )

    def test_planner_rejects_invalid_uncertainty_and_support_values(self):
        base = {
            "forward_m": 0.30, "lateral_m": 0.18,
            "range_uncertainty_m": 0.01, "lateral_uncertainty_m": 0.01,
            "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
        }
        for field in (
            "range_uncertainty_m", "lateral_uncertainty_m", "support_radius_m",
        ):
            for invalid in (-0.01, float("nan"), float("inf"), float("-inf")):
                with self.subTest(field=field, invalid=invalid), self.assertRaises(ValueError):
                    RadarSceneEstimator._plan_path([dict(base, **{field: invalid})])

    def test_wall_interpolation_conservatively_carries_endpoint_inflation(self):
        endpoints = [
            {
                "forward_m": 0.20, "lateral_m": 0.15,
                "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.07,
                "support_radius_m": 0.01, "confidence": 0.8,
                "evidence_kind": "direct", "source": "point",
            },
            {
                "forward_m": 0.27, "lateral_m": 0.15,
                "range_uncertainty_m": 0.08, "lateral_uncertainty_m": 0.03,
                "support_radius_m": 0.04, "confidence": 0.9,
                "evidence_kind": "probability", "source": "heatmap",
            },
        ]
        interpolated, segment_count = RadarSceneEstimator._interpolate_wall_obstacles(
            endpoints,
        )
        middle = min(
            interpolated,
            key=lambda obstacle: abs(obstacle["forward_m"] - 0.25),
        )
        self.assertEqual(segment_count, 1)
        self.assertAlmostEqual(middle["forward_m"], 0.25, delta=0.013)
        self.assertEqual(middle["range_uncertainty_m"], 0.08)
        self.assertEqual(middle["lateral_uncertainty_m"], 0.07)
        self.assertEqual(middle["support_radius_m"], 0.04)
        self.assertEqual(middle["confidence"], 0.9)
        self.assertEqual(middle["evidence_kind"], "mixed")
        self.assertEqual(middle["source"], "mixed")

    def test_per_obstacle_inflation_widens_goal_exclusion_and_clearance(self):
        base = {
            "forward_m": 0.35, "lateral_m": 0.0,
            "range_uncertainty_m": 0.0, "lateral_uncertainty_m": 0.0,
            "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
        }
        inflated = dict(
            base, range_uncertainty_m=0.08, support_radius_m=0.02,
        )
        base_path = RadarSceneEstimator._plan_path([base])
        inflated_path = RadarSceneEstimator._plan_path([inflated])
        self.assertLess(
            inflated_path["goal_corridor_width_deg"],
            base_path["goal_corridor_width_deg"],
        )
        self.assertGreater(
            abs(inflated_path["goal_angle_deg"]), abs(base_path["goal_angle_deg"]),
        )
        expected_clearance = min(
            math.hypot(point[0] - 0.35, point[1]) - (0.075 + 0.03 + 0.08 + 0.02)
            for point in inflated_path["points"]
        )
        self.assertAlmostEqual(
            inflated_path["minimum_clearance_m"], round(expected_clearance, 3),
        )

    def test_current_path_collision_can_be_caused_only_by_uncertainty(self):
        selected = {
            "points": [[0.0, 0.0], [0.10, 0.0], [0.20, 0.0], [0.30, 0.0]],
        }
        base = {
            "forward_m": 0.20, "lateral_m": 0.14,
            "range_uncertainty_m": 0.0, "lateral_uncertainty_m": 0.0,
            "support_radius_m": 0.0, "evidence_kind": "direct", "source": "point",
        }
        self.assertEqual(
            RadarSceneEstimator._path_collision_segments(selected, [base]), [],
        )
        inflated = dict(base, lateral_uncertainty_m=0.04)
        segments = RadarSceneEstimator._path_collision_segments(selected, [inflated])
        self.assertTrue(segments)
        self.assertEqual(segments[0]["reason"], "confirmed_obstacle")

    def test_probability_and_reactive_evidence_merge_conservatively(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator._probability_grid.add_hit(
            0.30, 0.10, now=1.0, confidence=1.0,
            source="point",
            range_uncertainty_m=0.08, lateral_uncertainty_m=0.03,
        )
        estimator._near_cells[(12, 4)] = {
            "forward_m": 0.30, "lateral_m": 0.10,
            "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.09,
            "confidence": 0.95, "source": "point", "hits": 2,
            "reactive": True, "_last_seen_ns": 1_000_000_000,
        }
        obstacles = estimator._planning_obstacle_payload(1.0)
        merged = next(item for item in obstacles if item["evidence_kind"] == "mixed")
        self.assertEqual(merged["range_uncertainty_m"], 0.08)
        self.assertEqual(merged["lateral_uncertainty_m"], 0.09)
        self.assertAlmostEqual(merged["support_radius_m"], 0.05 / math.sqrt(2))

    def test_confirmed_point_propagates_lateral_uncertainty_to_reactive_cell(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        point = RadarPoint(0.10, 0.30, 0.0, 0.0, 30.0)
        estimator.ingest(radar_frame((point,), frame_number=1), received_at=1.0)
        estimator.ingest(radar_frame((point,), frame_number=2), received_at=1.125)
        cell = estimator.snapshot(now=1.125)["near_occupancy"]["cells"][0]
        self.assertGreater(cell["lateral_uncertainty_m"], 0.0)

    def test_above_sensor_confirmed_point_still_blocks_the_2d_path(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        elevated = RadarPoint(0.0, 0.20, 0.50, 0.0, 30.0)
        estimator.ingest(radar_frame((elevated,), frame_number=1), received_at=1.000)
        estimator.ingest(radar_frame((elevated,), frame_number=2), received_at=1.125)
        scene = estimator.snapshot(now=1.125)
        self.assertEqual(scene["tracks"][0]["height_band"], "above_sensor")
        self.assertTrue(scene["near_occupancy"]["cells"])
        self.assertNotEqual(scene["planned_path"]["status"], "clear")

    def test_same_key_point_and_heatmap_reactive_hits_merge_conservatively(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator._tracks = [
            {
                "source": "point", "forward_m": 0.30, "lateral_m": 0.10,
                "confidence": 0.90, "point_confirmed": True,
                "range_uncertainty_m": 0.08, "lateral_uncertainty_m": 0.03,
                "_updated_by_point": True,
            },
            {
                "source": "heatmap", "forward_m": 0.30, "lateral_m": 0.10,
                "confidence": 0.95, "heatmap_confirmed": True,
                "range_uncertainty_m": 0.02, "lateral_uncertainty_m": 0.09,
                "_updated_by_heatmap": True,
            },
        ]
        estimator._update_near_occupancy(1.0)
        cell = estimator._near_obstacle_payload(1.0)[0]
        self.assertEqual(cell["range_uncertainty_m"], 0.08)
        self.assertEqual(cell["lateral_uncertainty_m"], 0.09)
        self.assertEqual(cell["confidence"], 0.95)
        self.assertEqual(cell["source"], "mixed")
        self.assertEqual(cell["hits"], 2)

    def test_unblocked_candidate_is_accepted_on_third_ingested_frame(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        straight = estimator._plan_path([])
        alternate = dict(straight, curvature_per_m=1.0)
        with patch.object(estimator, "_plan_path",
                          side_effect=[straight, alternate, alternate, alternate]):
            scenes = []
            for number, when in enumerate((1.0, 1.125, 1.250, 1.375), start=1):
                estimator.ingest(radar_frame((), frame_number=number), received_at=when)
                scenes.append(estimator.snapshot(now=when))
        self.assertEqual(scenes[1]["planned_path"]["curvature_per_m"], 0.0)
        self.assertEqual(scenes[2]["planned_path"]["pending_switch_frames"], 2)
        self.assertEqual(scenes[3]["planned_path"]["curvature_per_m"], 1.0)

    def test_confirmed_obstacle_switches_blocked_path_immediately(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator.ingest(radar_frame((), frame_number=1), received_at=1.0)
        original = estimator.snapshot(now=1.0)["planned_path"]["points"]
        point = RadarPoint(0.0, 0.35, 0.0, 0.0, 30.0)
        estimator.ingest(radar_frame((point,), frame_number=2), received_at=1.125)
        estimator.ingest(radar_frame((point,), frame_number=3), received_at=1.250)
        path = estimator.snapshot(now=1.250)["planned_path"]
        self.assertEqual(path["stability_status"], "route_changed_blocked")
        self.assertIn(path["status"], {"clear", "limited"})
        self.assertNotEqual(path["curvature_per_m"], 0.0)
        self.assertEqual(path["blocked_previous_path"]["points"], original)
        self.assertTrue(path["blocked_previous_path"]["segments"])
        self.assertTrue(path["blocked_previous_path"]["segments"][0]["reason"])

    def test_confirmed_eleven_centimeter_obstacle_stops_path_before_danger(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        point = RadarPoint(0.0, 0.11, 0.0, 0.0, 30.0)
        estimator.ingest(radar_frame((point,), frame_number=1), received_at=1.000)
        estimator.ingest(radar_frame((point,), frame_number=2), received_at=1.125)
        scene = estimator.snapshot(now=1.125)
        self.assertEqual(scene["hazard"]["level"], "NORMAL")
        self.assertEqual(scene["planned_path"]["status"], "blocked_near_field")
        self.assertEqual(scene["planned_path"]["length_m"], 0.0)
        self.assertEqual(scene["planned_path"]["points"], [[0.0, 0.0]])

    def test_repeated_eleven_centimeter_stop_does_not_count_switch_each_frame(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        point = RadarPoint(0.0, 0.11, 0.0, 0.0, 30.0)
        for frame_number in range(1, 6):
            estimator.ingest(
                radar_frame((point,), frame_number=frame_number),
                received_at=1.0 + frame_number * 0.125,
            )
        scene = estimator.snapshot(now=1.625)
        self.assertEqual(scene["planned_path"]["status"], "blocked_near_field")
        self.assertEqual(scene["diagnostics"]["path_switch_accepted_count"], 1)

    def test_snapshot_after_init_or_reset_never_plans_without_ingest(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        with patch.object(estimator, "_plan_path",
                          side_effect=AssertionError("snapshot planned")):
            initial = estimator.snapshot(now=1.0)
            estimator.reset("explicit_test")
            reset = estimator.snapshot(now=1.1)
        for scene in (initial, reset):
            self.assertEqual(scene["planned_path"]["points"], [[0.0, 0.0]])
            self.assertEqual(scene["planned_path"]["robot_width_m"], 0.15)
            self.assertEqual(scene["planned_path"]["safety_margin_m"], 0.03)
            self.assertEqual(scene["planned_path"]["minimum_passage_width_m"], 0.21)
            self.assertEqual(scene["planned_path"]["minimum_hold_ms"], 0)
            self.assertEqual(scene["planned_path"]["switch_confirm_frames"], 3)
            self.assertFalse(scene["planned_path"]["switch_accepted"])
            self.assertTrue(scene["planned_path"]["replan_every_frame"])
            self.assertEqual(scene["planned_path"]["pending_switch_frames"], 0)
            self.assertIsNone(scene["planned_path"].get("blocked_previous_path"))
            self.assertEqual(scene["diagnostics"]["path_switch_pending_count"], 0)
            self.assertEqual(scene["diagnostics"]["path_switch_accepted_count"], 0)

    def test_planned_path_uses_one_centimeter_spacing(self):
        path = RadarSceneEstimator._plan_path([])

        self.assertEqual(path["status"], "clear")
        self.assertEqual(path["resolution_m"], 0.01)
        self.assertEqual(path["length_m"], 0.50)
        self.assertEqual(len(path["points"]), 51)
        self.assertEqual(path["points"][1], [0.01, 0.0])
        self.assertEqual(
            path["planner_mode"],
            "dynamic_corridor_hybrid_lattice",
        )

    def test_planned_path_bends_around_confirmed_obstacle(self):
        path = RadarSceneEstimator._plan_path(
            [
                {
                    "forward_m": 0.35,
                    "lateral_m": 0.0,
                    "range_uncertainty_m": 0.0,
                }
            ]
        )

        self.assertGreater(path["length_m"], 0.0)
        self.assertNotEqual(path["curvature_per_m"], 0.0)
        self.assertTrue(any(abs(point[1]) > 0.01 for point in path["points"]))
        self.assertGreater(abs(path["points"][-1][1]), 0.0)

    def test_three_centimetre_margin_blocks_fifteen_centimeter_corridor(self):
        walls = []
        for index in range(20, 251, 5):
            forward = index / 100.0
            walls.extend(
                [
                    {
                        "forward_m": forward,
                        "lateral_m": -0.075,
                        "range_uncertainty_m": 0.0,
                    },
                    {
                        "forward_m": forward,
                        "lateral_m": 0.075,
                        "range_uncertainty_m": 0.0,
                    },
                ]
            )

        path = RadarSceneEstimator._plan_path(walls)

        self.assertEqual(path["minimum_passage_width_m"], 0.21)
        self.assertEqual(path["range_uncertainty_lateral_inflation_m"], 0.0)
        self.assertLess(path["length_m"], 3.0)

    def test_planner_has_no_right_goal_bias(self):
        path = RadarSceneEstimator._plan_path([])

        self.assertEqual(path["goal_lateral_m"], 0.0)
        self.assertEqual(path["curvature_per_m"], 0.0)
        self.assertEqual(path["points"][-1][1], 0.0)

    def test_nearby_obstacle_samples_are_linked_as_wall(self):
        path = RadarSceneEstimator._plan_path(
            [
                {
                    "forward_m": 0.50,
                    "lateral_m": -0.025,
                    "source": "point",
                },
                {
                    "forward_m": 0.50,
                    "lateral_m": 0.025,
                    "source": "point",
                },
            ]
        )

        self.assertEqual(path["obstacle_wall_link_m"], 0.10)
        self.assertEqual(path["obstacle_wall_segments"], 1)
        self.assertEqual(path["obstacle_object_clusters"], 1)
        self.assertNotEqual(path["curvature_per_m"], 0.0)

    def test_dynamic_goal_selects_right_free_corridor(self):
        path = RadarSceneEstimator._plan_path(
            [
                {
                    "forward_m": 0.275,
                    "lateral_m": -0.275,
                    "source": "point",
                },
                {
                    "forward_m": 0.275,
                    "lateral_m": -0.375,
                    "source": "heatmap",
                },
                {
                    "forward_m": 0.45,
                    "lateral_m": 0.05,
                    "source": "heatmap",
                },
                {
                    "forward_m": 0.50,
                    "lateral_m": 0.025,
                    "source": "point",
                },
                {
                    "forward_m": 0.55,
                    "lateral_m": 0.05,
                    "source": "point",
                },
            ]
        )

        self.assertGreater(path["goal_angle_deg"], 20.0)
        self.assertGreater(path["goal_lateral_m"], 0.20)
        self.assertGreater(path["points"][-1][1], 0.20)
        self.assertEqual(
            path["goal_selection_mode"],
            "widest_motion_relative_corridor",
        )

    def test_planner_uses_curved_local_avoidance(self):
        path = RadarSceneEstimator._plan_path(
            [{"forward_m": 0.30, "lateral_m": 0.0}]
        )

        self.assertEqual(path["status"], "clear")
        self.assertGreater(path["turn_count"], 0)
        self.assertTrue(any(abs(point[1]) > 0.05 for point in path["points"]))
        self.assertEqual(path["search_resolution_m"], 0.025)
        self.assertTrue(path["smoothed"])
        self.assertGreaterEqual(path["minimum_clearance_m"], 0.0)
        self.assertGreater(path["geometry_confidence"], 0.5)
        for left, right in zip(path["points"], path["points"][1:]):
            self.assertAlmostEqual(
                math.hypot(right[0] - left[0], right[1] - left[1]),
                0.01,
                delta=0.00015,
            )

    def test_blocked_scene_returns_best_partial_route(self):
        wall = [
            {"forward_m": 0.35, "lateral_m": index / 100.0}
            for index in range(-60, 61, 5)
        ]

        path = RadarSceneEstimator._plan_path(wall)

        self.assertEqual(path["status"], "limited")
        self.assertFalse(path["goal_reached"])
        self.assertGreater(path["length_m"], 0.0)
        self.assertLess(path["length_m"], 0.35)

    def test_only_strong_confirmed_point_is_retained_for_five_seconds(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)

        def strong_frame(frame_number: int) -> RadarFrame:
            return radar_frame(
                (
                    RadarPoint(
                        x_m=0.0,
                        y_m=0.60,
                        z_m=0.0,
                        radial_velocity_mps=0.0,
                        snr_db=30.0,
                    ),
                ),
                frame_number=frame_number,
            )

        estimator.ingest(strong_frame(1), received_at=1.0)
        estimator.ingest(strong_frame(2), received_at=1.1)
        estimator.ingest(strong_frame(3), received_at=1.2)

        retained = estimator.snapshot(now=6.199)
        expired = estimator.snapshot(now=6.2)

        self.assertEqual(len(retained["accumulated_obstacles"]), 1)
        self.assertEqual(retained["obstacle_memory_window_ms"], 5000)
        self.assertEqual(retained["accumulated_obstacles"][0]["remaining_ms"], 1)
        self.assertEqual(expired["accumulated_obstacles"], [])

    def test_weak_confirmed_point_is_not_accumulated(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator.ingest(point_frame(0.60, frame_number=1), received_at=1.0)
        estimator.ingest(point_frame(0.60, frame_number=2), received_at=1.1)

        scene = estimator.snapshot(now=1.1)

        self.assertTrue(scene["obstacle_tracks"])
        self.assertEqual(scene["accumulated_obstacles"], [])

    def test_near_obstacle_expires_from_path_after_one_second(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        for frame_number, received_at in (
            (1, 1.0),
            (2, 1.1),
            (3, 1.2),
        ):
            estimator.ingest(
                radar_frame(
                    (
                        RadarPoint(
                            x_m=0.0,
                            y_m=0.35,
                            z_m=0.0,
                            radial_velocity_mps=0.0,
                            snr_db=30.0,
                        ),
                    ),
                    frame_number=frame_number,
                ),
                received_at=received_at,
            )

        self.assertTrue(estimator.snapshot(now=1.2)["near_occupancy"]["cells"])
        scene = estimator.snapshot(now=3.0)

        self.assertEqual(scene["obstacle_tracks"], [])
        self.assertEqual(len(scene["accumulated_obstacles"]), 1)
        self.assertEqual(scene["near_occupancy"]["cells"], [])
        self.assertTrue(scene["persistent_near_obstacles"])
        self.assertEqual(
            scene["persistent_near_obstacle_memory"]["window_ms"],
            5000,
        )
        self.assertEqual(
            set(base64.b64decode(scene["grid"]["data_base64"])),
            {0},
        )

    def test_rejected_motion_does_not_transform_map_or_freeze_blocked_path(self):
        estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
        estimator.ingest(radar_frame((), frame_number=1), received_at=1.0)
        real_transform = estimator._probability_grid.transform
        estimator._probability_grid.transform = Mock(wraps=real_transform)
        obstacle = RadarPoint(0.0, 0.35, 0.0, 0.0, 30.0)
        estimator.ingest(
            radar_frame((obstacle,), frame_number=2), received_at=1.125,
        )
        estimator.ingest(
            radar_frame((obstacle,), frame_number=3), received_at=1.250,
        )

        scene = estimator.snapshot(now=1.250)
        estimator._probability_grid.transform.assert_not_called()
        self.assertIn(
            scene["motion_compensation"]["status"],
            {"insufficient_points", "stabilizing"},
        )
        self.assertEqual(
            scene["planned_path"]["stability_status"],
            "route_changed_blocked",
        )

    def test_unconfirmed_point_track_does_not_enter_occupancy_grid(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.40), received_at=3.0)

        scene = estimator.snapshot(now=3.0)

        self.assertEqual(len(scene["tracks"]), 1)
        self.assertEqual(scene["tracks"][0]["classification"], "point_candidate")
        self.assertFalse(scene["obstacle_tracks"])
        self.assertEqual(
            set(base64.b64decode(scene["grid"]["data_base64"])),
            {0},
        )

    def test_adjacent_heatmap_cells_form_one_reflection_cluster(self):
        estimator = calibrated_estimator()
        estimator.ingest(
            clustered_heatmap_frame(
                4,
                frame_number=1,
                with_point=False,
            ),
            received_at=4.0,
        )

        scene = estimator.snapshot(now=4.0)

        self.assertEqual(scene["diagnostics"]["heatmap_candidate_cells"], 2)
        self.assertEqual(scene["diagnostics"]["heatmap_cluster_count"], 1)
        self.assertEqual(len(scene["reference_tracks"]), 1)
        self.assertEqual(
            scene["reference_tracks"][0]["classification"],
            "reflection_evidence",
        )
        self.assertFalse(scene["obstacle_tracks"])

    def test_repeated_stable_heatmap_without_point_stays_reflection(self):
        estimator = calibrated_estimator()
        for frame_number in range(1, 5):
            estimator.ingest(
                clustered_heatmap_frame(
                    4,
                    frame_number=frame_number,
                    with_point=False,
                ),
                received_at=5.0 + frame_number * 0.1,
            )

        scene = estimator.snapshot(now=5.4)
        heatmap_track = next(
            track for track in scene["tracks"] if track["source"] == "heatmap"
        )

        self.assertTrue(heatmap_track["position_stable"])
        self.assertFalse(heatmap_track["point_correlated"])
        self.assertFalse(heatmap_track["heatmap_confirmed"])
        self.assertEqual(heatmap_track["classification"], "reflection_evidence")
        self.assertFalse(scene["obstacle_tracks"])

    def test_heatmap_exit_threshold_holds_existing_cluster_without_flicker(self):
        estimator = calibrated_estimator()
        estimator.ingest(
            clustered_heatmap_frame(
                4,
                frame_number=1,
                with_point=False,
                value=255,
            ),
            received_at=5.5,
        )
        first_track_id = estimator.snapshot(now=5.5)["tracks"][0]["track_id"]
        for frame_number in (2, 3):
            estimator.ingest(
                clustered_heatmap_frame(
                    4,
                    frame_number=frame_number,
                    with_point=False,
                    value=41,
                ),
                received_at=5.5 + frame_number * 0.1,
            )

        scene = estimator.snapshot(now=5.8)
        heatmap_tracks = [
            track for track in scene["tracks"] if track["source"] == "heatmap"
        ]

        self.assertEqual(len(heatmap_tracks), 1)
        self.assertEqual(heatmap_tracks[0]["track_id"], first_track_id)
        self.assertEqual(heatmap_tracks[0]["classification"], "reflection_evidence")

    def test_repeated_stable_point_correlated_cluster_becomes_obstacle(self):
        estimator = calibrated_estimator()
        for frame_number in range(1, 4):
            estimator.ingest(
                clustered_heatmap_frame(
                    4,
                    frame_number=frame_number,
                    with_point=True,
                ),
                received_at=6.0 + frame_number * 0.1,
            )

        scene = estimator.snapshot(now=6.3)
        heatmap_track = next(
            track for track in scene["tracks"] if track["source"] == "heatmap"
        )

        self.assertTrue(heatmap_track["position_stable"])
        self.assertTrue(heatmap_track["point_correlated"])
        self.assertTrue(heatmap_track["heatmap_confirmed"])
        self.assertEqual(heatmap_track["classification"], "confirmed_obstacle")
        self.assertIn(heatmap_track, scene["obstacle_tracks"])
        self.assertGreater(
            max(base64.b64decode(scene["grid"]["data_base64"])),
            0,
        )

    def test_one_point_supports_only_its_nearest_heatmap_cluster(self):
        estimator = calibrated_estimator()
        for frame_number in range(1, 4):
            estimator.ingest(
                split_heatmap_frame(frame_number=frame_number),
                received_at=7.0 + frame_number * 0.1,
            )

        scene = estimator.snapshot(now=7.3)
        heatmap_tracks = [
            track for track in scene["tracks"] if track["source"] == "heatmap"
        ]

        self.assertEqual(len(heatmap_tracks), 2)
        self.assertEqual(
            sum(track["heatmap_confirmed"] for track in heatmap_tracks),
            1,
        )
        self.assertEqual(
            sum(
                track["classification"] == "reflection_evidence"
                for track in heatmap_tracks
            ),
            1,
        )

    def test_hazard_enters_at_10cm_after_two_of_three_and_releases_at_13cm(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.099, frame_number=1), received_at=10.0)
        self.assertEqual(
            estimator.snapshot(now=10.0)["hazard"]["level"],
            "UNKNOWN",
        )
        estimator.ingest(point_frame(0.098, frame_number=2), received_at=10.1)
        self.assertEqual(
            estimator.snapshot(now=10.1)["hazard"]["level"],
            "DANGER",
        )
        estimator.ingest(point_frame(0.129, frame_number=3), received_at=10.2)
        self.assertEqual(
            estimator.snapshot(now=10.2)["hazard"]["level"],
            "DANGER",
        )
        estimator.ingest(point_frame(0.131, frame_number=4), received_at=10.3)
        self.assertEqual(
            estimator.snapshot(now=10.3)["hazard"]["level"],
            "NORMAL",
        )

    def test_second_point_in_frame_cannot_overwrite_latched_near_track(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.09, frame_number=1), received_at=15.0)
        estimator.ingest(point_frame(0.09, frame_number=2), received_at=15.1)
        self.assertEqual(
            estimator.snapshot(now=15.1)["hazard"]["level"],
            "DANGER",
        )
        estimator.ingest(
            radar_frame(
                (
                    RadarPoint(0.0, 0.09, 0.0, 0.0),
                    RadarPoint(0.0, 0.20, 0.0, 0.0),
                ),
                frame_number=3,
            ),
            received_at=15.2,
        )
        scene = estimator.snapshot(now=15.2)
        self.assertEqual(scene["hazard"]["level"], "DANGER")
        self.assertAlmostEqual(scene["hazard"]["nearest_confirmed_m"], 0.09)
        self.assertEqual(len(scene["tracks"]), 2)

    def test_track_is_present_before_but_not_at_300ms(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.40), received_at=2.0)
        tracks = estimator.snapshot(now=2.299999999)["tracks"]
        self.assertTrue(tracks)
        self.assertEqual(tracks[0]["age_ms"], 299)
        self.assertFalse(estimator.snapshot(now=2.300000000)["tracks"])

    def test_two_point_hits_with_one_missed_frame_enter_danger(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.09, frame_number=1), received_at=25.0)
        estimator.ingest(empty_frame(frame_number=2), received_at=25.1)
        self.assertEqual(
            estimator.snapshot(now=25.1)["hazard"]["level"],
            "UNKNOWN",
        )
        estimator.ingest(point_frame(0.09, frame_number=3), received_at=25.2)
        self.assertEqual(
            estimator.snapshot(now=25.2)["hazard"]["level"],
            "DANGER",
        )

    def test_reported_dropped_frames_age_point_confirmation_window(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.20, frame_number=1), received_at=27.0)
        estimator.ingest(point_frame(0.20, frame_number=2), received_at=27.1)
        self.assertEqual(
            estimator.snapshot(now=27.1)["hazard"]["level"],
            "NORMAL",
        )
        estimator.ingest(
            radar_frame(
                frame_number=10,
                frame_transition="gap",
                dropped_frames_since_previous=7,
            ),
            received_at=27.2,
        )
        self.assertEqual(
            estimator.snapshot(now=27.2)["hazard"]["level"],
            "UNKNOWN",
        )

    def test_saturated_heatmap_bins_zero_and_one_never_enter_danger(self):
        estimator = calibrated_estimator()
        for index in range(3):
            estimator.ingest(
                saturated_near_heatmap_frame(index),
                received_at=30.0 + index * 0.1,
            )
        hazard = estimator.snapshot(now=30.2)["hazard"]
        self.assertNotEqual(hazard["level"], "DANGER")
        self.assertIsNone(hazard["nearest_confirmed_m"])

    def test_bin_one_heatmap_cannot_move_confirmed_point_track_into_danger(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.20, frame_number=1), received_at=35.0)
        estimator.ingest(point_frame(0.20, frame_number=2), received_at=35.1)
        self.assertEqual(
            estimator.snapshot(now=35.1)["hazard"]["level"],
            "NORMAL",
        )
        estimator.ingest(
            heatmap_only_frame(1, 8, frame_number=3),
            received_at=35.2,
        )
        scene = estimator.snapshot(now=35.2)
        self.assertEqual(scene["hazard"]["level"], "NORMAL")
        self.assertAlmostEqual(scene["hazard"]["nearest_confirmed_m"], 0.20)
        self.assertEqual(
            {track["source"] for track in scene["tracks"]},
            {"point", "heatmap"},
        )

    def test_missing_and_mismatched_calibration_are_unknown(self):
        for estimator in (uncalibrated_estimator(), mismatched_estimator()):
            estimator.ingest(point_frame(0.30))
            scene = estimator.snapshot()
            self.assertIn(
                scene["calibration_status"],
                {"calibration_required", "profile_mismatch"},
            )
            self.assertEqual(scene["hazard"]["level"], "UNKNOWN")
            self.assertFalse(scene["tracks"])

    def test_calibration_mismatch_clears_all_evidence(self):
        estimator = calibrated_estimator()
        seed_probability_cell(estimator)
        before = estimator.snapshot(now=1.375)
        self.assertEqual(len(before["rolling_local_map"]["cells"]), 1)

        estimator.ingest(
            point_frame(
                0.30,
                frame_number=4,
                snr_db=24.0,
                calibration_id="different-calibration",
            ),
            received_at=1.5,
        )
        scene = estimator.snapshot(now=1.5)

        self.assertEqual(scene["calibration_status"], "calibration_mismatch")
        assert_probability_maps_empty(self, scene)
        self.assertEqual(scene["tracks"], [])
        self.assertEqual(scene["hazard"]["level"], "UNKNOWN")
        self.assertGreater(
            scene["diagnostics"]["scene_epoch"],
            before["diagnostics"]["scene_epoch"],
        )

    def test_scene_epoch_is_stable_until_a_reset_boundary(self):
        estimator = calibrated_estimator()
        self.assertEqual(estimator.snapshot(now=1.0)["diagnostics"]["scene_epoch"], 0)

        estimator.ingest(point_frame(0.30), received_at=1.0)
        self.assertEqual(estimator.snapshot(now=1.0)["diagnostics"]["scene_epoch"], 0)
        self.assertEqual(estimator.snapshot(now=1.1)["diagnostics"]["scene_epoch"], 0)

        estimator.reset("operator_reset")
        self.assertEqual(estimator.snapshot(now=1.1)["diagnostics"]["scene_epoch"], 1)
        estimator.ingest(point_frame(0.30, frame_number=2), received_at=1.2)
        self.assertEqual(estimator.snapshot(now=1.2)["diagnostics"]["scene_epoch"], 1)

    def test_explicit_reset_clears_probability_maps_and_advances_epoch(self):
        estimator = calibrated_estimator()
        seed_probability_cell(estimator)
        before = estimator.snapshot(now=1.375)
        self.assertEqual(len(before["rolling_local_map"]["cells"]), 1)

        estimator.reset("operator_reset")
        scene = estimator.snapshot(now=1.375)

        assert_probability_maps_empty(self, scene)
        self.assertGreater(
            scene["diagnostics"]["scene_epoch"],
            before["diagnostics"]["scene_epoch"],
        )

    def test_every_scene_boundary_clears_path_selector_epoch_state(self):
        boundary_names = (
            "explicit_reset",
            "producer_change",
            "duplicate",
            "reset_or_out_of_order",
            "profile_mismatch",
            "calibration_mismatch",
            "source_fault",
            "replay_restart",
        )
        for boundary_name in boundary_names:
            with self.subTest(boundary=boundary_name):
                estimator = calibrated_estimator()
                straight = RadarSceneEstimator._plan_path([])
                alternate = dict(straight, curvature_per_m=1.0)
                with patch.object(
                    estimator,
                    "_plan_path",
                    side_effect=[
                        straight, alternate, alternate, alternate, straight,
                    ],
                ):
                    for frame_number in range(1, 6):
                        estimator.ingest(
                            empty_frame(frame_number=frame_number),
                            received_at=1.0 + frame_number * 0.125,
                        )
                seeded = estimator.snapshot(now=1.625)
                self.assertEqual(
                    seeded["diagnostics"]["path_switch_accepted_count"], 1,
                )
                self.assertEqual(
                    seeded["planned_path"]["pending_switch_frames"], 1,
                )

                if boundary_name == "explicit_reset":
                    estimator.reset("operator_reset")
                elif boundary_name == "producer_change":
                    estimator.ingest(
                        radar_frame((), producer_id="new-producer", frame_number=6),
                        received_at=2.0,
                    )
                elif boundary_name in {"duplicate", "reset_or_out_of_order"}:
                    estimator.ingest(
                        radar_frame(
                            (), frame_number=6, frame_transition=boundary_name,
                        ),
                        received_at=2.0,
                    )
                elif boundary_name == "profile_mismatch":
                    estimator.ingest(
                        radar_frame((), frame_number=6,
                                    profile_id="different-profile"),
                        received_at=2.0,
                    )
                elif boundary_name == "calibration_mismatch":
                    estimator.ingest(
                        radar_frame((), frame_number=6,
                                    calibration_id="different-calibration"),
                        received_at=2.0,
                    )
                elif boundary_name == "source_fault":
                    estimator.snapshot(source_status="fault", now=2.0)
                else:
                    estimator.reset("replay_restart")

                scene = estimator.snapshot(now=2.0)
                self.assertEqual(
                    scene["planned_path"]["pending_switch_frames"], 0,
                )
                self.assertEqual(
                    scene["diagnostics"]["path_switch_pending_count"], 0,
                )
                self.assertEqual(
                    scene["diagnostics"]["path_switch_accepted_count"], 0,
                )
                self.assertIsNone(
                    scene["planned_path"].get("blocked_previous_path"),
                )

    def test_producer_change_clears_probability_maps_and_advances_epoch(self):
        estimator = calibrated_estimator()
        seed_probability_cell(estimator)
        before = estimator.snapshot(now=1.375)
        self.assertEqual(len(before["rolling_local_map"]["cells"]), 1)

        estimator.ingest(
            point_frame(
                0.30,
                frame_number=1,
                producer_id="new-producer",
            ),
            received_at=1.5,
        )
        scene = estimator.snapshot(now=1.5)

        assert_probability_maps_empty(self, scene)
        self.assertGreater(
            scene["diagnostics"]["scene_epoch"],
            before["diagnostics"]["scene_epoch"],
        )

    def test_duplicate_and_out_of_order_clear_probability_maps(self):
        for transition in ("duplicate", "reset_or_out_of_order"):
            with self.subTest(transition=transition):
                estimator = calibrated_estimator()
                seed_probability_cell(estimator)
                before = estimator.snapshot(now=1.375)
                self.assertEqual(len(before["rolling_local_map"]["cells"]), 1)

                estimator.ingest(
                    point_frame(
                        0.30,
                        frame_number=3,
                        frame_transition=transition,
                    ),
                    received_at=1.5,
                )
                scene = estimator.snapshot(now=1.5)

                assert_probability_maps_empty(self, scene)
                self.assertGreater(
                    scene["diagnostics"]["scene_epoch"],
                    before["diagnostics"]["scene_epoch"],
                )

    def test_profile_mismatch_clears_probability_maps_and_advances_epoch(self):
        estimator = calibrated_estimator()
        seed_probability_cell(estimator)
        before = estimator.snapshot(now=1.375)
        self.assertEqual(len(before["rolling_local_map"]["cells"]), 1)

        estimator.ingest(
            point_frame(
                0.30,
                frame_number=4,
                profile_id="different-profile",
            ),
            received_at=1.5,
        )
        scene = estimator.snapshot(now=1.5)

        self.assertEqual(scene["calibration_status"], "profile_mismatch")
        assert_probability_maps_empty(self, scene)
        self.assertGreater(
            scene["diagnostics"]["scene_epoch"],
            before["diagnostics"]["scene_epoch"],
        )

    def test_source_fault_clears_probability_maps_and_advances_epoch(self):
        estimator = calibrated_estimator()
        seed_probability_cell(estimator)
        before = estimator.snapshot(now=1.375)
        self.assertEqual(len(before["rolling_local_map"]["cells"]), 1)

        scene = estimator.snapshot(source_status="fault", now=1.5)

        assert_probability_maps_empty(self, scene)
        self.assertGreater(
            scene["diagnostics"]["scene_epoch"],
            before["diagnostics"]["scene_epoch"],
        )

    def test_zero_returns_and_heatmap_only_evidence_remain_unknown(self):
        estimator = calibrated_estimator()
        estimator.ingest(empty_frame(), received_at=40.0)
        self.assertEqual(
            estimator.snapshot(now=40.0)["hazard"]["level"],
            "UNKNOWN",
        )
        estimator.ingest(
            heatmap_only_frame(4, 8, frame_number=2),
            received_at=40.1,
        )
        self.assertEqual(
            estimator.snapshot(now=40.1)["hazard"]["level"],
            "UNKNOWN",
        )

    def test_exact_enter_and_release_thresholds_are_inclusive(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.10, frame_number=1), received_at=50.0)
        estimator.ingest(point_frame(0.10, frame_number=2), received_at=50.1)
        self.assertEqual(
            estimator.snapshot(now=50.1)["hazard"]["level"],
            "DANGER",
        )
        estimator.ingest(point_frame(0.13, frame_number=3), received_at=50.2)
        self.assertEqual(
            estimator.snapshot(now=50.2)["hazard"]["level"],
            "NORMAL",
        )

    def test_source_fault_states_clear_scene_and_report_sensor_fault(self):
        for source_status in ("waiting", "stale", "fault", "replay_end"):
            with self.subTest(source_status=source_status):
                estimator = calibrated_estimator()
                estimator.ingest(
                    point_frame(0.30, frame_number=1),
                    received_at=60.0,
                )
                scene = estimator.snapshot(
                    source_status=source_status,
                    now=60.0,
                )
                self.assertEqual(scene["hazard"]["level"], "SENSOR_FAULT")
                self.assertFalse(scene["tracks"])
                self.assertEqual(
                    set(base64.b64decode(scene["grid"]["data_base64"])),
                    {0},
                )

    def test_nondefault_axes_ignore_heatmap_but_keep_mapped_points(self):
        axes = RadarAxes(forward_axis="x", lateral_axis="y")
        model = RadarClutterModel(
            **{
                **clutter_model().__dict__,
                "axes": axes,
            }
        )
        data = bytearray(RANGE_BINS * AZIMUTH_BINS)
        data[4 * AZIMUTH_BINS + 8] = 255
        estimator = RadarSceneEstimator(axes, model)
        estimator.ingest(
            radar_frame(
                (
                    RadarPoint(
                        x_m=0.40,
                        y_m=0.0,
                        z_m=0.02,
                        radial_velocity_mps=0.0,
                    ),
                ),
                heatmap=RadarHeatmap(
                    data=bytes(data),
                    range_bins=RANGE_BINS,
                    azimuth_bins=AZIMUTH_BINS,
                    range_step_m=RANGE_STEP_M,
                    tlv_type=304,
                    motion_mode="major",
                    floor_db=10.0,
                    ceiling_db=110.0,
                ),
            )
        )
        tracks = estimator.snapshot()["tracks"]
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]["source"], "point")
        self.assertAlmostEqual(tracks[0]["forward_m"], 0.40)

    def test_optional_calibration_degrades_heatmap_to_point_only(self):
        estimator = RadarSceneEstimator(
            RadarAxes(),
            require_calibration=False,
        )
        estimator.ingest(heatmap_only_frame(4, 8))
        scene = estimator.snapshot()
        self.assertEqual(
            scene["calibration_status"],
            "calibration_unavailable",
        )
        self.assertFalse(scene["tracks"])
        self.assertEqual(
            scene["diagnostics"]["heatmap_cells_accepted"],
            0,
        )

    def test_optional_missing_calibration_never_authorizes_point_hazard(self):
        estimator = RadarSceneEstimator(
            RadarAxes(),
            require_calibration=False,
        )
        estimator.ingest(point_frame(0.09, frame_number=1), received_at=65.0)
        estimator.ingest(point_frame(0.09, frame_number=2), received_at=65.1)
        scene = estimator.snapshot(now=65.1)
        self.assertEqual(
            scene["calibration_status"],
            "calibration_unavailable",
        )
        self.assertFalse(scene["tracks"])
        self.assertEqual(scene["hazard"]["level"], "UNKNOWN")

    def test_reset_and_producer_change_clear_confirmation_and_latch(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.09, frame_number=1), received_at=70.0)
        estimator.ingest(point_frame(0.09, frame_number=2), received_at=70.1)
        self.assertEqual(
            estimator.snapshot(now=70.1)["hazard"]["level"],
            "DANGER",
        )
        estimator.reset("operator_reset")
        self.assertEqual(
            estimator.snapshot(now=70.1)["hazard"]["level"],
            "UNKNOWN",
        )
        self.assertFalse(estimator.snapshot(now=70.1)["tracks"])

        estimator.ingest(
            radar_frame(
                (
                    RadarPoint(0.0, 0.09, 0.0, 0.0),
                ),
                producer_id="new-producer",
                frame_number=1,
            ),
            received_at=70.2,
        )
        scene = estimator.snapshot(now=70.2)
        self.assertEqual(scene["hazard"]["level"], "UNKNOWN")
        self.assertEqual(len(scene["tracks"]), 1)

    def test_duplicate_transition_resets_frame_hit_window_and_latch(self):
        estimator = calibrated_estimator()
        estimator.ingest(point_frame(0.09, frame_number=1), received_at=80.0)
        estimator.ingest(point_frame(0.09, frame_number=2), received_at=80.1)
        self.assertEqual(
            estimator.snapshot(now=80.1)["hazard"]["level"],
            "DANGER",
        )
        duplicate = radar_frame(
            (RadarPoint(0.0, 0.09, 0.0, 0.0),),
            frame_number=2,
            frame_transition="duplicate",
        )
        estimator.ingest(duplicate, received_at=80.2)
        scene = estimator.snapshot(now=80.2)
        self.assertEqual(scene["hazard"]["level"], "UNKNOWN")
        self.assertEqual(
            scene["diagnostics"]["last_reset_reason"],
            "duplicate",
        )

    def test_real_977cm_clutter_is_rejected_without_erasing_transient_points(self):
        fixture = load_real_fixture()
        model = build_clutter_model(
            fixture,
            RadarAxes(),
            min_frames=6,
        )
        estimator = RadarSceneEstimator(RadarAxes(), model)
        kept = rejected = danger_frames = 0
        for frame in bind_calibration(fixture, model.calibration_id):
            estimator.ingest(frame)
            scene = estimator.snapshot()
            kept += scene["diagnostics"]["scene_point_count"]
            rejected += scene["diagnostics"]["clutter_points_rejected"]
            danger_frames += scene["hazard"]["level"] == "DANGER"
        self.assertGreater(rejected, 0)
        self.assertEqual(danger_frames, 0)
        self.assertGreater(kept, 0)


if __name__ == "__main__":
    unittest.main()
