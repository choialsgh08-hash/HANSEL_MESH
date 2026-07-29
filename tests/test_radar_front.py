import json
import base64
import contextlib
import io
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.request import urlopen

from common.sensor_contract import (
    RadarFrame,
    RadarHeatmap,
    RadarPoint,
    SensorHeader,
    SensorHealth,
)
from common.radar_geometry import RadarAxes as CommonRadarAxes
from sensors.mission_log import encode_log_entry
from sensors.radar_calibration import RadarClutterModel
from monitor.radar_front import (
    MissionLogFollower,
    RadarAxes,
    RadarFrontState,
    build_handler,
    main,
    make_demo_frame,
    parse_args,
)
from monitor.radar_scene import RadarSceneEstimator
from http.server import ThreadingHTTPServer


REPO_ROOT = Path(__file__).resolve().parents[1]


def radar_frame(
    points,
    frame_number=7,
    complete=True,
    dropped=0,
    transition="consecutive",
    heatmap=None,
    producer_id="test-radar",
):
    return RadarFrame(
        header=SensorHeader(
            mission_id="test-mission",
            unit_id="head",
            boot_id="test-boot",
            producer_id=producer_id,
            stream_id="radar/front",
            seq=frame_number + 1,
            monotonic_ns=1_000_000_000 + frame_number,
            frame_id="radar_native",
            calibration_id="uncalibrated",
        ),
        frame_number=frame_number,
        subframe_number=0,
        complete=complete,
        dropped_frames_since_previous=dropped,
        points=tuple(points),
        source_format="test",
        sdk_version="5.5.0.2",
        frame_transition=transition,
        profile_id="test-profile",
        capture_baudrate=115200,
        heatmap=heatmap,
    )


def calibrated_scene_estimator(
    profile_id="test-profile",
) -> RadarSceneEstimator:
    return RadarSceneEstimator(
        RadarAxes(),
        RadarClutterModel(
            schema_version=1,
            calibration_id="radar-clutter-front-test",
            profile_id=profile_id,
            axes=RadarAxes(),
            range_bins=2,
            azimuth_bins=2,
            range_step_m=0.05,
            motion_mode="major",
            point_clusters=(),
            heatmap_median_db=(0.0, 0.0, 0.0, 0.0),
            heatmap_mad_db=(0.0, 0.0, 0.0, 0.0),
        ),
    )


class FakeClock:
    def __init__(self, value=10.0):
        self.value = value

    def __call__(self):
        return self.value


class RadarFrontStateTests(unittest.TestCase):
    def test_radar_axes_is_reexported_from_common_geometry(self):
        self.assertIs(RadarAxes, CommonRadarAxes)

    def test_snapshot_exposes_scene_and_retains_three_metre_point(self):
        state = RadarFrontState(
            "test",
            max_range_m=3.0,
            scene_estimator=calibrated_scene_estimator(),
            clock=FakeClock(),
        )
        state.ingest(radar_frame([RadarPoint(0.0, 2.99, 0.0, 0.0)]))

        snapshot = state.snapshot()

        self.assertEqual(snapshot["scene"]["schema_version"], 1)
        self.assertTrue(
            any(
                track["distance_m"] > 2.9
                for track in snapshot["scene"]["tracks"]
            )
        )

    def test_scene_ingests_complete_frame_before_display_limits(self):
        state = RadarFrontState(
            "test",
            max_points=1,
            max_range_m=1.0,
            scene_estimator=calibrated_scene_estimator(),
            clock=FakeClock(),
        )
        state.ingest(
            radar_frame(
                [
                    RadarPoint(0.0, 0.5, 0.0, 0.0),
                    RadarPoint(0.0, 2.99, 0.0, 0.0),
                ]
            )
        )

        snapshot = state.snapshot()

        self.assertEqual(snapshot["frame"]["display_point_count"], 1)
        self.assertEqual(len(snapshot["scene"]["tracks"]), 2)
        self.assertTrue(
            any(
                track["distance_m"] > 2.9
                for track in snapshot["scene"]["tracks"]
            )
        )

    def test_state_status_fault_overrides_scene_hazard(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                frame_number=2,
            )
        )
        self.assertEqual(
            state.snapshot()["scene"]["hazard"]["level"],
            "DANGER",
        )

        clock.value += 2.1
        scene = state.snapshot()["scene"]

        self.assertEqual(scene["hazard"]["level"], "SENSOR_FAULT")
        self.assertFalse(scene["tracks"])

    def test_demo_scene_is_explicitly_synthetic_not_calibration_required(self):
        state = RadarFrontState("demo", clock=FakeClock())
        state.ingest(make_demo_frame(1, 1, seed=6432))

        self.assertEqual(
            state.snapshot()["scene"]["calibration_status"],
            "synthetic",
        )

    def test_follow_and_replay_without_model_remain_untrusted(self):
        for source_mode in ("follow", "replay"):
            with self.subTest(source_mode=source_mode):
                clock = FakeClock()
                state = RadarFrontState(source_mode, clock=clock)
                state.ingest(
                    radar_frame(
                        [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                        frame_number=1,
                    )
                )
                clock.value += 0.1
                state.ingest(
                    radar_frame(
                        [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                        frame_number=2,
                    )
                )

                scene = state.snapshot()["scene"]

                self.assertEqual(
                    scene["calibration_status"],
                    "calibration_required",
                )
                self.assertEqual(scene["hazard"]["level"], "UNKNOWN")

    def test_profile_mismatch_stays_running_and_fails_closed(self):
        clock = FakeClock()
        state = RadarFrontState(
            "follow",
            scene_estimator=calibrated_scene_estimator(
                profile_id="different-profile"
            ),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                frame_number=2,
            )
        )

        snapshot = state.snapshot()

        self.assertEqual(snapshot["status"], "live")
        self.assertEqual(
            snapshot["scene"]["calibration_status"],
            "profile_mismatch",
        )
        self.assertEqual(snapshot["scene"]["hazard"]["level"], "UNKNOWN")

    def test_replay_loop_restart_clears_scene_once_then_accepts_new_frame(self):
        clock = FakeClock()
        state = RadarFrontState(
            "replay",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.09, 0.0, 0.0)],
                frame_number=2,
            )
        )
        self.assertEqual(
            state.snapshot()["scene"]["hazard"]["level"],
            "DANGER",
        )

        state.reset_sensor_sequence_tracking()
        reset_scene = state.snapshot()["scene"]

        self.assertFalse(reset_scene["tracks"])
        self.assertEqual(
            reset_scene["diagnostics"]["last_reset_reason"],
            "replay_loop_restart",
        )

        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        restarted_scene = state.snapshot()["scene"]

        self.assertEqual(len(restarted_scene["tracks"]), 1)
        self.assertAlmostEqual(restarted_scene["tracks"][0]["distance_m"], 0.20)

    def test_scene_extension_retains_legacy_snapshot_fields(self):
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=FakeClock(),
        )
        state.ingest(radar_frame([RadarPoint(0.0, 0.5, 0.0, 0.0)]))

        snapshot = state.snapshot()

        for field in (
            "version",
            "ui_build_id",
            "status",
            "warning",
            "source",
            "age_ms",
            "fps",
            "axes",
            "limits",
            "frame",
            "occupancy",
            "counters",
            "health",
        ):
            self.assertIn(field, snapshot)
        self.assertIn("scene", snapshot)

    def test_default_ti_axes_map_y_forward_and_x_right(self):
        clock = FakeClock()
        state = RadarFrontState("test", clock=clock)
        point = RadarPoint(
            x_m=1.0,
            y_m=2.0,
            z_m=0.3,
            radial_velocity_mps=-0.4,
            snr_db=17.0,
        )
        self.assertTrue(state.ingest(radar_frame([point])))
        frame = state.snapshot()["frame"]
        self.assertEqual(
            frame["point_fields"],
            [
                "forward_m",
                "lateral_m",
                "height_m",
                "radial_velocity_mps",
                "snr_db",
            ],
        )
        mapped = frame["points"][0]
        self.assertEqual(mapped[0], 2.0)
        self.assertEqual(mapped[1], 1.0)
        self.assertEqual(mapped[2], 0.3)

    def test_axis_override_supports_reversed_mounting(self):
        axes = RadarAxes(
            forward_axis="x",
            forward_sign=-1,
            lateral_axis="y",
            lateral_sign=-1,
        )
        forward, lateral = axes.map_point(
            RadarPoint(2.0, -3.0, 0.0, 0.0)
        )
        self.assertEqual((forward, lateral), (-2.0, 3.0))

    def test_points_behind_robot_are_not_displayed(self):
        clock = FakeClock()
        state = RadarFrontState("test", clock=clock)
        state.ingest(
            radar_frame(
                [
                    RadarPoint(0.0, -1.0, 0.0, 0.0),
                    RadarPoint(0.0, 1.0, 0.0, 0.0),
                ]
            )
        )
        frame = state.snapshot()["frame"]
        self.assertEqual(frame["source_point_count"], 2)
        self.assertEqual(frame["display_point_count"], 1)

    def test_close_forward_points_are_not_filtered_by_default(self):
        clock = FakeClock()
        state = RadarFrontState("test", clock=clock)
        state.ingest(
            radar_frame(
                [
                    RadarPoint(0.0, 0.049, 0.0, 0.0),
                    RadarPoint(0.0, 0.098, 0.0, 0.0),
                ]
            )
        )
        frame = state.snapshot()["frame"]
        self.assertEqual(frame["source_point_count"], 2)
        self.assertEqual(frame["display_point_count"], 2)
        self.assertEqual(
            [point[0] for point in frame["points"]],
            [0.049, 0.098],
        )

    def test_point_cap_preserves_nearest_returns(self):
        clock = FakeClock()
        state = RadarFrontState("test", max_points=2, clock=clock)
        state.ingest(
            radar_frame(
                [
                    RadarPoint(0.0, 3.0, 0.0, 0.0),
                    RadarPoint(0.0, 1.0, 0.0, 0.0),
                    RadarPoint(0.0, 2.0, 0.0, 0.0),
                ]
            )
        )
        frame = state.snapshot()["frame"]
        self.assertTrue(frame["truncated"])
        self.assertEqual(
            [point[0] for point in frame["points"]],
            [1.0, 2.0],
        )

    def test_incomplete_frame_preserves_last_complete_frame(self):
        clock = FakeClock()
        state = RadarFrontState("test", clock=clock)
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.0, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        self.assertFalse(
            state.ingest(
                radar_frame(
                    [RadarPoint(0.0, 6.0, 0.0, 0.0)],
                    frame_number=2,
                    complete=False,
                )
            )
        )
        snapshot = state.snapshot()
        self.assertEqual(snapshot["status"], "degraded")
        self.assertEqual(snapshot["frame"]["number"], 1)
        self.assertEqual(snapshot["counters"]["incomplete_frames"], 1)

    def test_incomplete_duplicate_immediately_clears_scene_evidence(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=2,
            )
        )
        self.assertEqual(
            state.snapshot()["scene"]["hazard"]["level"],
            "NORMAL",
        )

        clock.value += 0.1
        self.assertFalse(
            state.ingest(
                radar_frame(
                    [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                    frame_number=2,
                    complete=False,
                    transition="duplicate",
                )
            )
        )
        snapshot = state.snapshot()

        self.assertEqual(snapshot["status"], "degraded")
        self.assertFalse(snapshot["scene"]["tracks"])
        self.assertEqual(snapshot["scene"]["hazard"]["level"], "UNKNOWN")
        self.assertEqual(
            snapshot["scene"]["diagnostics"]["last_reset_reason"],
            "duplicate",
        )

    def test_first_complete_frame_after_incomplete_reset_is_retained(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
                complete=False,
                transition="reset_or_out_of_order",
            )
        )

        clock.value += 0.1
        self.assertTrue(
            state.ingest(
                radar_frame(
                    [RadarPoint(0.0, 0.25, 0.0, 0.0)],
                    frame_number=2,
                )
            )
        )
        scene = state.snapshot()["scene"]

        self.assertEqual(len(scene["tracks"]), 1)
        self.assertAlmostEqual(scene["tracks"][0]["distance_m"], 0.25)
        self.assertFalse(scene["tracks"][0]["point_confirmed"])

    def test_incomplete_first_frame_from_new_producer_clears_scene(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=2,
            )
        )
        self.assertEqual(
            state.snapshot()["scene"]["hazard"]["level"],
            "NORMAL",
        )

        clock.value += 0.1
        self.assertFalse(
            state.ingest(
                radar_frame(
                    [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                    frame_number=1,
                    complete=False,
                    transition="first",
                    producer_id="replacement-radar",
                )
            )
        )
        snapshot = state.snapshot()

        self.assertEqual(snapshot["status"], "degraded")
        self.assertFalse(snapshot["scene"]["tracks"])
        self.assertEqual(snapshot["scene"]["hazard"]["level"], "UNKNOWN")
        self.assertEqual(
            snapshot["scene"]["diagnostics"]["last_reset_reason"],
            "producer_change",
        )

    def test_complete_frame_after_incomplete_producer_change_is_fresh(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
                complete=False,
                transition="first",
                producer_id="replacement-radar",
            )
        )

        clock.value += 0.1
        self.assertTrue(
            state.ingest(
                radar_frame(
                    [RadarPoint(0.0, 0.25, 0.0, 0.0)],
                    frame_number=2,
                    producer_id="replacement-radar",
                )
            )
        )
        scene = state.snapshot()["scene"]

        self.assertEqual(len(scene["tracks"]), 1)
        self.assertAlmostEqual(scene["tracks"][0]["distance_m"], 0.25)
        self.assertFalse(scene["tracks"][0]["point_confirmed"])

    def test_complete_producer_change_starts_fresh_scene_evidence(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.25, 0.0, 0.0)],
                frame_number=1,
                transition="first",
                producer_id="replacement-radar",
            )
        )
        scene = state.snapshot()["scene"]

        self.assertEqual(len(scene["tracks"]), 1)
        self.assertAlmostEqual(scene["tracks"][0]["distance_m"], 0.25)
        self.assertFalse(scene["tracks"][0]["point_confirmed"])
        self.assertEqual(
            scene["diagnostics"]["last_reset_reason"],
            "producer_change",
        )

    def test_replay_loop_reset_forgets_state_producer_boundary(self):
        clock = FakeClock()
        state = RadarFrontState(
            "replay",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        state.reset_sensor_sequence_tracking()

        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.25, 0.0, 0.0)],
                frame_number=1,
                transition="first",
                producer_id="replacement-radar",
            )
        )
        scene = state.snapshot()["scene"]

        self.assertEqual(len(scene["tracks"]), 1)
        self.assertEqual(
            scene["diagnostics"]["last_reset_reason"],
            "replay_loop_restart",
        )

    def test_source_fault_reset_forgets_state_producer_boundary(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            scene_estimator=calibrated_scene_estimator(),
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.20, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 2.1
        self.assertEqual(
            state.snapshot()["scene"]["hazard"]["level"],
            "SENSOR_FAULT",
        )

        clock.value += 0.1
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 0.25, 0.0, 0.0)],
                frame_number=1,
                transition="first",
                producer_id="replacement-radar",
            )
        )
        scene = state.snapshot()["scene"]

        self.assertEqual(len(scene["tracks"]), 1)
        self.assertEqual(
            scene["diagnostics"]["last_reset_reason"],
            "fault",
        )

    def test_sensor_sequence_gap_marks_live_view_degraded(self):
        clock = FakeClock()
        state = RadarFrontState("follow", clock=clock)
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.0, 0.0, 0.0)],
                frame_number=1,
                dropped=1,
            )
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.1, 0.0, 0.0)],
                frame_number=3,
            )
        )
        snapshot = state.snapshot()
        self.assertEqual(snapshot["status"], "degraded")
        self.assertEqual(
            snapshot["health"]["degraded_reason"],
            "sensor_sequence_gap",
        )
        self.assertEqual(
            snapshot["counters"]["sensor_sequence_gaps_total"],
            1,
        )
        self.assertEqual(snapshot["counters"]["frame_gaps_total"], 1)
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.2, 0.0, 0.0)],
                frame_number=4,
            )
        )
        self.assertEqual(state.snapshot()["status"], "degraded")
        state.ingest(
            SensorHealth(
                header=SensorHeader(
                    mission_id="test-mission",
                    unit_id="head",
                    boot_id="test-boot",
                    producer_id="health-producer",
                    stream_id="health/radar",
                    seq=1,
                    monotonic_ns=2_000_000_000,
                ),
                subject_stream_id="radar/front",
                status="ok",
                seq_gaps_total=1,
                detail="quiet interval after one gap",
            )
        )
        snapshot = state.snapshot()
        self.assertEqual(snapshot["status"], "live")
        self.assertEqual(snapshot["counters"]["frame_gaps_total"], 1)

    def test_ok_health_does_not_clear_viewer_integrity_failures(self):
        for error_method, detail, counter, reason in (
            (
                "note_parse_error",
                "invalid JSON log line",
                "parse_errors_total",
                "invalid_log_record",
            ),
            (
                "note_log_sequence_error",
                "non-contiguous mission log sequence",
                "log_sequence_errors_total",
                "log_sequence_discontinuity",
            ),
        ):
            with self.subTest(error_method=error_method):
                state = RadarFrontState("follow", clock=FakeClock())
                state.ingest(
                    radar_frame([RadarPoint(0.0, 2.0, 0.0, 0.0)])
                )
                getattr(state, error_method)(detail)
                state.ingest(
                    SensorHealth(
                        header=SensorHeader(
                            mission_id="test-mission",
                            unit_id="head",
                            boot_id="test-boot",
                            producer_id="health-producer",
                            stream_id="health/radar",
                            seq=1,
                            monotonic_ns=2_000_000_000,
                        ),
                        subject_stream_id="radar/front",
                        status="ok",
                    )
                )

                snapshot = state.snapshot()
                self.assertEqual(snapshot["status"], "degraded")
                self.assertEqual(snapshot["health"]["degraded_reason"], reason)
                self.assertEqual(snapshot["counters"][counter], 1)

    def test_integrity_reason_precedes_simultaneous_capture_degradation(self):
        state = RadarFrontState("follow", clock=FakeClock())
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.0, 0.0, 0.0)],
                frame_number=1,
            )
        )
        state.note_parse_error("invalid JSON log line")
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.1, 0.0, 0.0)],
                frame_number=3,
                dropped=1,
            )
        )

        snapshot = state.snapshot()

        self.assertEqual(snapshot["status"], "degraded")
        self.assertEqual(
            snapshot["health"]["degraded_reason"],
            "invalid_log_record",
        )
        self.assertEqual(snapshot["counters"]["parse_errors_total"], 1)
        self.assertEqual(snapshot["counters"]["frame_gaps_total"], 1)

    def test_new_producer_starts_a_fresh_sensor_sequence(self):
        clock = FakeClock()
        state = RadarFrontState("follow", clock=clock)
        first = radar_frame([], frame_number=10)
        second = radar_frame([], frame_number=0)
        second = RadarFrame(
            header=SensorHeader(
                mission_id=second.header.mission_id,
                unit_id=second.header.unit_id,
                boot_id="new-boot",
                producer_id="new-producer",
                stream_id=second.header.stream_id,
                seq=1,
                monotonic_ns=second.header.monotonic_ns,
                frame_id=second.header.frame_id,
                calibration_id=second.header.calibration_id,
            ),
            frame_number=second.frame_number,
            subframe_number=second.subframe_number,
            complete=second.complete,
            dropped_frames_since_previous=(
                second.dropped_frames_since_previous
            ),
            points=second.points,
            source_format=second.source_format,
            sdk_version=second.sdk_version,
            frame_transition="first",
            profile_id=second.profile_id,
            capture_baudrate=second.capture_baudrate,
        )
        state.ingest(first)
        state.ingest(second)
        snapshot = state.snapshot()
        self.assertEqual(snapshot["status"], "live")
        self.assertEqual(
            snapshot["counters"]["sensor_sequence_gaps_total"],
            0,
        )

    def test_live_stale_and_fault_transitions_use_viewer_clock(self):
        clock = FakeClock()
        state = RadarFrontState(
            "test",
            stale_after_s=0.5,
            fault_after_s=1.5,
            clock=clock,
        )
        state.ingest(radar_frame([]))
        self.assertEqual(state.snapshot()["status"], "live")
        clock.value += 0.6
        self.assertEqual(state.snapshot()["status"], "stale")
        clock.value += 1.0
        self.assertEqual(state.snapshot()["status"], "fault")

    def test_zero_point_frame_is_live_but_not_declared_clear(self):
        clock = FakeClock()
        state = RadarFrontState("test", clock=clock)
        state.ingest(radar_frame([]))
        snapshot = state.snapshot()
        self.assertEqual(snapshot["status"], "live")
        self.assertIn("빈 공간", snapshot["warning"])

    def test_native_heatmap_is_exposed_as_compact_base64(self):
        clock = FakeClock()
        state = RadarFrontState("follow", clock=clock)
        heatmap = RadarHeatmap(
            data=bytes([0, 32, 128, 255]),
            range_bins=2,
            azimuth_bins=2,
            range_step_m=0.05,
            tlv_type=304,
            motion_mode="major",
            floor_db=-36.0,
            ceiling_db=27.0,
        )
        state.ingest(radar_frame([], heatmap=heatmap))
        payload = state.snapshot()["frame"]["heatmap"]
        self.assertEqual(payload["range_bins"], 2)
        self.assertEqual(payload["azimuth_bins"], 2)
        self.assertEqual(payload["encoding"], "log-u8")
        self.assertEqual(payload["motion_mode"], "major")
        self.assertEqual(payload["tlv_type"], 304)
        self.assertEqual(
            payload["azimuth_layout"],
            "fft-shifted-spatial-frequency",
        )
        self.assertEqual(payload["lambda_over_d_x"], 2.0)
        self.assertEqual(payload["azimuth_min_deg"], -70.0)
        self.assertEqual(payload["azimuth_max_deg"], 70.0)
        self.assertEqual(payload["valid_min_range_m"], 0.07)
        self.assertEqual(payload["valid_max_range_m"], 7.5)
        self.assertEqual(
            base64.b64decode(payload["data_base64"]),
            heatmap.data,
        )

    def test_native_heatmap_is_disabled_for_nondefault_axes(self):
        clock = FakeClock()
        state = RadarFrontState(
            "follow",
            axes=RadarAxes(lateral_sign=-1),
            clock=clock,
        )
        heatmap = RadarHeatmap(
            data=bytes([0, 32, 128, 255]),
            range_bins=2,
            azimuth_bins=2,
            range_step_m=0.05,
            tlv_type=304,
            motion_mode="major",
            floor_db=-36.0,
            ceiling_db=27.0,
        )
        state.ingest(radar_frame([], heatmap=heatmap))
        frame = state.snapshot()["frame"]
        self.assertIsNone(frame["heatmap"])
        self.assertEqual(
            frame["heatmap_status"],
            "disabled_nondefault_axes",
        )

    def test_point_history_is_time_bounded_return_evidence(self):
        clock = FakeClock()
        state = RadarFrontState(
            "follow",
            history_window_s=1.0,
            stale_after_s=0.5,
            fault_after_s=2.0,
            clock=clock,
        )
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 1.0, 0.0, 0.0)],
                frame_number=1,
            )
        )
        clock.value += 0.6
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.0, 0.0, 0.0)],
                frame_number=2,
            )
        )
        occupancy = state.snapshot()["occupancy"]
        self.assertEqual(occupancy["frames"], 2)
        self.assertEqual(occupancy["history_window_ms"], 1000)
        self.assertEqual(occupancy["semantics"], "return_evidence_only_unknown_elsewhere")
        self.assertEqual(
            sorted(point[5] for point in occupancy["points"]),
            [0, 600],
        )

        clock.value += 0.5
        occupancy = state.snapshot()["occupancy"]
        self.assertEqual(occupancy["frames"], 1)
        self.assertEqual(len(occupancy["points"]), 1)

    def test_demo_frame_produces_intensity_fan_payload(self):
        clock = FakeClock()
        state = RadarFrontState("demo", clock=clock)
        state.ingest(make_demo_frame(4, 123, seed=6432))
        heatmap = state.snapshot()["frame"]["heatmap"]
        self.assertEqual(heatmap["source"], "synthetic-point-derived")
        self.assertEqual(heatmap["azimuth_layout"], "linear-degrees")
        self.assertEqual(heatmap["encoding"], "log-u8")
        self.assertEqual(
            len(base64.b64decode(heatmap["data_base64"])),
            heatmap["range_bins"] * heatmap["azimuth_bins"],
        )

    def test_missing_ti_point_cloud_tlv_is_never_live(self):
        clock = FakeClock()
        state = RadarFrontState("follow", clock=clock)
        record = radar_frame([])
        record = RadarFrame(
            header=record.header,
            frame_number=record.frame_number,
            subframe_number=record.subframe_number,
            complete=True,
            dropped_frames_since_previous=0,
            points=(),
            source_format="ti-mmwave-none",
            sdk_version=record.sdk_version,
            frame_transition=record.frame_transition,
            profile_id=record.profile_id,
            capture_baudrate=record.capture_baudrate,
        )
        state.ingest(record)
        snapshot = state.snapshot()
        self.assertEqual(snapshot["status"], "degraded")
        self.assertIn("TLV", snapshot["warning"])

    def test_periodic_health_drop_counters_latch_degraded(self):
        clock = FakeClock()
        state = RadarFrontState("follow", clock=clock)
        health = SensorHealth(
            header=SensorHeader(
                mission_id="test-mission",
                unit_id="head",
                boot_id="test-boot",
                producer_id="health-producer",
                stream_id="health/radar",
                seq=1,
                monotonic_ns=1_000_000_000,
            ),
            subject_stream_id="radar/front",
            status="degraded",
            writer_drops_total=3,
            parse_errors_total=1,
            detail="periodic diagnostics",
        )
        self.assertTrue(state.ingest(health))
        state.ingest(radar_frame([]))
        snapshot = state.snapshot()
        self.assertEqual(snapshot["status"], "degraded")
        self.assertEqual(snapshot["counters"]["writer_drops_total"], 3)
        self.assertEqual(snapshot["counters"]["parse_errors_total"], 1)

    def test_demo_is_deterministic_and_has_native_front_points(self):
        first = make_demo_frame(4, 123, seed=6432)
        second = make_demo_frame(4, 123, seed=6432)
        self.assertEqual(first, second)
        self.assertGreater(len(first.points), 50)
        self.assertTrue(all(point.y_m > 0 for point in first.points))


class MissionLogFollowerTests(unittest.TestCase):
    def test_partial_line_is_not_published_until_newline_arrives(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            record = radar_frame([RadarPoint(0.0, 2.0, 0.0, 0.0)])
            encoded = encode_log_entry(1, record)
            path.write_bytes(encoded)
            state = RadarFrontState("follow")
            stop_event = threading.Event()
            follower = MissionLogFollower(path, state, stop_event, poll_s=0.01)
            thread = threading.Thread(target=follower.run, daemon=True)
            thread.start()
            time.sleep(0.04)
            self.assertIsNone(state.snapshot()["frame"])
            with path.open("ab") as handle:
                handle.write(b"\n")
                handle.flush()
            deadline = time.monotonic() + 1.0
            while state.snapshot()["frame"] is None and time.monotonic() < deadline:
                time.sleep(0.01)
            stop_event.set()
            thread.join(1.0)
            self.assertIsNotNone(state.snapshot()["frame"])

    def test_completed_backlog_is_skipped_before_following_live_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            old_record = radar_frame(
                [RadarPoint(0.0, 9.0, 0.0, 0.0)],
                frame_number=1,
            )
            path.write_bytes(encode_log_entry(1, old_record) + b"\n")
            state = RadarFrontState("follow")
            stop_event = threading.Event()
            follower = MissionLogFollower(path, state, stop_event, poll_s=0.01)
            thread = threading.Thread(target=follower.run, daemon=True)
            thread.start()
            deadline = time.monotonic() + 1.0
            while (
                not state.snapshot()["source"]["note"].startswith("following")
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            self.assertIsNone(state.snapshot()["frame"])

            live_record = radar_frame(
                [RadarPoint(0.0, 2.0, 0.0, 0.0)],
                frame_number=2,
            )
            with path.open("ab") as handle:
                handle.write(encode_log_entry(2, live_record) + b"\n")
                handle.flush()

            deadline = time.monotonic() + 1.0
            while (
                state.snapshot()["frame"] is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            stop_event.set()
            thread.join(1.0)
            self.assertEqual(state.snapshot()["frame"]["number"], 2)


class RadarFrontDocumentationTests(unittest.TestCase):
    CALIBRATE_COMMAND = (
        "python -m sensors radar-calibrate "
        "missions\\radar-empty-scene.jsonl `\n"
        "  --output configs\\radar\\calibrations\\head-near-8hz.json `\n"
        "  --min-frames 50"
    )
    VIEWER_COMMAND = (
        "python monitor\\radar_front.py `\n"
        "  --follow missions\\radar-board-live.jsonl `\n"
        "  --clutter-calibration "
        "configs\\radar\\calibrations\\head-near-8hz.json `\n"
        "  --max-range-m 3 `\n"
        "  --history-window 0.3"
    )
    CALIBRATION_DIRECTORY_COMMAND = (
        "New-Item -ItemType Directory -Force "
        "-Path configs\\radar\\calibrations"
    )

    def read_guides(self):
        return {
            name: (REPO_ROOT / "docs" / name).read_text(
                encoding="utf-8"
            )
            for name in (
                "radar_front_view.md",
                "radar_reconnect_windows.md",
            )
        }

    def test_both_guides_document_the_calibrated_r9_contract(self):
        for name, docs in self.read_guides().items():
            with self.subTest(name=name):
                self.assertIn(self.CALIBRATE_COMMAND, docs)
                self.assertIn(self.VIEWER_COMMAND, docs)
                self.assertIn("0~3m", docs)
                self.assertIn("0~50cm", docs)
                self.assertIn("10cm", docs)
                self.assertIn("UNKNOWN", docs)
                self.assertNotIn("흑백 벽면", docs)

    def test_both_guides_create_calibration_directory_before_calibrate(
        self,
    ):
        for name, docs in self.read_guides().items():
            with self.subTest(name=name):
                self.assertIn(self.CALIBRATION_DIRECTORY_COMMAND, docs)
                self.assertLess(
                    docs.index(self.CALIBRATION_DIRECTORY_COMMAND),
                    docs.index(self.CALIBRATE_COMMAND),
                )

    def test_both_guides_use_prompt_observed_as_baud_success_signal(self):
        for name, docs in self.read_guides().items():
            with self.subTest(name=name):
                self.assertIn("`new_baud_prompt_observed=true`", docs)
                self.assertNotIn(
                    "`new_baud_verified_by_version=true`",
                    docs,
                )


class RadarFrontHttpTests(unittest.TestCase):
    def test_non_running_supervisor_state_blocks_the_very_next_api_response(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / "radar-supervisor-run.json"
            manifest.write_text(
                json.dumps({"state": "RUNNING"}),
                encoding="utf-8",
            )
            state = RadarFrontState(
                "test",
                supervisor_manifest_path=manifest,
            )
            state.ingest(
                radar_frame([RadarPoint(0.0, 0.25, 0.0, 0.0)])
            )
            server = ThreadingHTTPServer(
                ("127.0.0.1", 0),
                build_handler(state, quiet=True),
            )
            thread = threading.Thread(
                target=server.serve_forever,
                daemon=True,
            )
            thread.start()
            try:
                url = f"http://127.0.0.1:{server.server_port}/api/radar"
                with urlopen(url, timeout=2) as response:
                    running = json.load(response)
                self.assertEqual(running["status"], "live")
                self.assertIsNotNone(running["frame"])

                manifest.write_text(
                    json.dumps(
                        {
                            "state": "RECOVERING",
                            "last_reason": "radar_frame_timeout",
                        }
                    ),
                    encoding="utf-8",
                )
                with urlopen(url, timeout=2) as response:
                    blocked = json.load(response)

                self.assertEqual(blocked["status"], "fault")
                self.assertEqual(
                    blocked["warning"],
                    "RADAR RECONNECTING · DRIVE STOP",
                )
                self.assertEqual(blocked["supervisor_state"], "RECOVERING")
                self.assertIsNone(blocked["frame"])
                self.assertIsNone(blocked["age_ms"])
                self.assertEqual(blocked["fps"], 0.0)
                self.assertEqual(blocked["occupancy"]["points"], [])
                self.assertEqual(blocked["scene"]["tracks"], [])
                self.assertEqual(
                    blocked["scene"]["hazard"]["level"],
                    "SENSOR_FAULT",
                )

                manifest.write_text("[]", encoding="utf-8")
                with urlopen(url, timeout=2) as response:
                    malformed = json.load(response)

                self.assertEqual(malformed["status"], "fault")
                self.assertEqual(
                    malformed["warning"],
                    "RADAR RECONNECTING · DRIVE STOP",
                )
                self.assertEqual(
                    malformed["supervisor_state"],
                    "UNAVAILABLE",
                )
                self.assertIsNone(malformed["frame"])
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)

    def test_api_and_offline_assets_are_served(self):
        state = RadarFrontState("test")
        state.ingest(radar_frame([]))
        server = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            build_handler(state, quiet=True),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            with urlopen(base + "/api/radar", timeout=2) as response:
                payload = json.load(response)
            self.assertEqual(payload["version"], 1)
            self.assertEqual(
                payload["ui_build_id"],
                "20260729-lidar-operator-r10",
            )
            self.assertEqual(payload["frame"]["number"], 7)
            self.assertEqual(payload["scene"]["schema_version"], 1)
            self.assertEqual(
                payload["scene"]["pose_mode"],
                "robot_relative",
            )
            with urlopen(base + "/", timeout=2) as response:
                html = response.read().decode("utf-8")
            with urlopen(base + "/radar_panel.js", timeout=2) as response:
                javascript = response.read().decode("utf-8")
            with urlopen(base + "/radar_scene.js", timeout=2) as response:
                scene_javascript = response.read().decode("utf-8")
            self.assertIn(
                "/radar_scene.js?v=20260729-lidar-operator-r10",
                html,
            )
            self.assertIn(
                "/radar_panel.js?v=20260729-lidar-operator-r10",
                html,
            )
            self.assertIn(
                '#radar-status[data-status="sensor_fault"]',
                html,
            )
            self.assertIn(
                '#radar-status[data-status="http_lost"]',
                html,
            )
            self.assertIn('id="build-tag">UI R10</span>', html)
            self.assertIn(
                '#collision-inset[data-hazard="SENSOR_FAULT"] {',
                html,
            )
            self.assertIn(
                "border-color: rgba(255, 81, 81, 0.92);",
                html,
            )
            self.assertIn(
                'const UI_BUILD_ID = "20260729-lidar-operator-r10";',
                javascript,
            )
            self.assertIn(
                '"RADAR STARTING · DRIVE STOP"',
                javascript,
            )
            self.assertIn(
                '"레이더 준비 중 · 주행을 정지하세요"',
                javascript,
            )
            self.assertIn(
                '"RADAR RECONNECTING · DRIVE STOP"',
                javascript,
            )
            self.assertIn(
                '"레이더 재연결 중 · 주행을 정지하세요"',
                javascript,
            )
            self.assertIn("sensor_fault: [", javascript)
            self.assertIn('id="radar-main-canvas"', html)
            self.assertIn('id="collision-canvas"', html)
            self.assertIn("0~3m LiDAR형 전방 점유 지도", html)
            self.assertIn("0~50cm 충돌 확대", html)
            self.assertIn("UNKNOWN ≠ FREE", html)
            self.assertIn("ROBOT RELATIVE", html)
            self.assertIn("/radar_scene.js", html)
            self.assertIn("fullscreen-button", html)
            self.assertIn("HanselRadarPanel", javascript)
            self.assertIn("HanselRadarScene.parseRadarScene", javascript)
            self.assertIn("drawLidarTopView", javascript)
            self.assertIn("drawCollisionInset", javascript)
            self.assertIn("drawRangeGuideArc", javascript)
            self.assertIn("clipToMapBoundary", javascript)
            self.assertIn("cellIntersectsRadialLimit", javascript)
            self.assertIn('clipShape: "rectangular"', javascript)
            self.assertIn('clipShape: "radial"', javascript)
            self.assertIn(
                "nearestConfirmedPoint("
                "presentation, CLOSE_MAX_RANGE_M",
                javascript,
            )
            self.assertIn("trackVisualAlpha", javascript)
            self.assertIn(
                "1 - ageMs / TRACK_MAX_AGE_MS",
                javascript,
            )
            self.assertNotIn(
                "Math.hypot(track.forward_m, track.lateral_m) "
                "<= options.maxRangeM",
                javascript,
            )
            self.assertIn("DANGER_RANGE_M = 0.1", javascript)
            self.assertIn("UI_BUILD_ID", javascript)
            self.assertIn("requestFullscreen", javascript)
            for forbidden in (
                "drawDepthCamera",
                "drawPerspectiveSurfaceMesh",
                "wallHeight",
                "depthContourCandidates",
                "WALL_TRACK_HOLD_MS",
                "CAUTION_RANGE_M",
            ):
                self.assertNotIn(forbidden, javascript)
            for forbidden in ("view-select", "outline-toggle"):
                self.assertNotIn(forbidden, html)
            self.assertNotIn("https://", html)
            self.assertNotIn("https://", javascript)
            self.assertNotIn("https://", scene_javascript)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(1.0)

    def test_cli_rejects_same_forward_and_lateral_axis(self):
        with self.assertRaises(SystemExit):
            parse_args(
                [
                    "--demo",
                    "--forward-axis",
                    "x",
                    "--lateral-axis",
                    "x",
                ]
            )

    def test_cli_limits_temporal_history_to_safety_window(self):
        with self.assertRaises(SystemExit):
            parse_args(["--demo", "--history-window", "1.3"])

    def test_cli_accepts_short_history_for_sharp_mapping(self):
        args = parse_args(["--demo", "--history-window", "0.2"])
        self.assertEqual(args.history_window, 0.2)

    def test_cli_accepts_clutter_calibration_path(self):
        args = parse_args(
            ["--demo", "--clutter-calibration", "model.json"]
        )
        self.assertEqual(args.clutter_calibration, "model.json")

    def test_cli_accepts_owned_parent_death_lease(self):
        args = parse_args(
            [
                "--demo",
                "--supervisor-parent-lease",
                "runtime/viewer-parent.lease",
            ]
        )
        self.assertEqual(
            args.supervisor_parent_lease,
            Path("runtime/viewer-parent.lease"),
        )

    def test_cli_defaults_main_range_to_three_metres(self):
        args = parse_args(["--demo"])
        self.assertEqual(args.max_range_m, 3.0)

    def test_corrupt_clutter_model_fails_through_cli_error_path(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "corrupt.json"
            path.write_text("{not-json", encoding="utf-8")
            stderr = io.StringIO()

            with contextlib.redirect_stderr(stderr):
                result = main(
                    [
                        "--demo",
                        "--clutter-calibration",
                        str(path),
                    ]
                )

        self.assertEqual(result, 1)
        self.assertIn("invalid clutter model JSON", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
