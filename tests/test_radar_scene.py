import base64
from pathlib import Path
import unittest

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
    RadarClutterModel,
    build_clutter_model,
)


PROFILE_ID = "radar-scene-test-profile"
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


def clutter_model() -> RadarClutterModel:
    cells = RANGE_BINS * AZIMUTH_BINS
    return RadarClutterModel(
        schema_version=1,
        calibration_id="radar-clutter-scene-test",
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


def point_frame(
    forward_m: float,
    *,
    frame_number: int = 1,
) -> RadarFrame:
    return radar_frame(
        (
            RadarPoint(
                x_m=0.0,
                y_m=forward_m,
                z_m=0.01,
                radial_velocity_mps=0.0,
                snr_db=20.0,
            ),
        ),
        frame_number=frame_number,
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

    def test_three_metre_grid_retains_049_and_299_but_rejects_301(self):
        estimator = calibrated_estimator()
        estimator.ingest(frame_with_forward_ranges(0.49, 2.99, 3.01))
        scene = estimator.snapshot()
        distances = [track["distance_m"] for track in scene["tracks"]]
        self.assertTrue(any(abs(value - 0.49) < 0.01 for value in distances))
        self.assertTrue(any(abs(value - 2.99) < 0.01 for value in distances))
        self.assertFalse(any(value > 3.0 for value in distances))
        grid = base64.b64decode(scene["grid"]["data_base64"])
        self.assertEqual(len(grid), 60 * 60)
        self.assertGreater(grid[59 * 60 + 30], 0)

    def test_exact_three_metres_maps_to_final_forward_row(self):
        estimator = calibrated_estimator()
        estimator.ingest(frame_with_forward_ranges(3.0))
        scene = estimator.snapshot()
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

    def test_heatmap_never_invents_height(self):
        estimator = calibrated_estimator()
        estimator.ingest(heatmap_only_frame(range_index=4, azimuth_index=8))
        track = estimator.snapshot()["tracks"][0]
        self.assertEqual(track["source"], "heatmap")
        self.assertIsNone(track["height_m"])
        self.assertAlmostEqual(track["range_uncertainty_m"], 0.09765625)

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
        estimator.ingest(point_frame(0.40), received_at=20.0)
        tracks = estimator.snapshot(now=20.299999999)["tracks"]
        self.assertTrue(tracks)
        self.assertEqual(tracks[0]["age_ms"], 299)
        self.assertFalse(estimator.snapshot(now=20.300000000)["tracks"])

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
            self.assertTrue(scene["tracks"])

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
        self.assertEqual(scene["calibration_status"], "ok")
        self.assertFalse(scene["tracks"])
        self.assertEqual(
            scene["diagnostics"]["heatmap_cells_accepted"],
            0,
        )

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
        model = build_clutter_model(
            load_real_fixture(),
            RadarAxes(),
            min_frames=6,
        )
        estimator = RadarSceneEstimator(RadarAxes(), model)
        kept = rejected = danger_frames = 0
        for frame in load_real_fixture():
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
