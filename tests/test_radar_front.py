import json
import base64
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
from sensors.mission_log import encode_log_entry
from monitor.radar_front import (
    MissionLogFollower,
    RadarAxes,
    RadarFrontState,
    build_handler,
    make_demo_frame,
    parse_args,
)
from http.server import ThreadingHTTPServer


def radar_frame(
    points,
    frame_number=7,
    complete=True,
    dropped=0,
    transition="consecutive",
    heatmap=None,
):
    return RadarFrame(
        header=SensorHeader(
            mission_id="test-mission",
            unit_id="head",
            boot_id="test-boot",
            producer_id="test-radar",
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


class FakeClock:
    def __init__(self, value=10.0):
        self.value = value

    def __call__(self):
        return self.value


class RadarFrontStateTests(unittest.TestCase):
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

    def test_sensor_sequence_gap_marks_live_view_degraded(self):
        clock = FakeClock()
        state = RadarFrontState("follow", clock=clock)
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.0, 0.0, 0.0)],
                frame_number=1,
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
        state.ingest(
            radar_frame(
                [RadarPoint(0.0, 2.2, 0.0, 0.0)],
                frame_number=4,
            )
        )
        self.assertEqual(state.snapshot()["status"], "degraded")

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


class RadarFrontHttpTests(unittest.TestCase):
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
            self.assertEqual(payload["ui_build_id"], "20260726-depth-camera-r8")
            self.assertEqual(payload["frame"]["number"], 7)
            with urlopen(base + "/", timeout=2) as response:
                html = response.read().decode("utf-8")
            with urlopen(base + "/radar_panel.js", timeout=2) as response:
                javascript = response.read().decode("utf-8")
            self.assertIn("전방 레이더", html)
            self.assertIn("UNKNOWN ≠ FREE", html)
            self.assertIn('option value="0.5" selected', html)
            self.assertNotIn('option value="1"', html)
            self.assertNotIn('option value="3"', html)
            self.assertIn('option value="camera" selected', html)
            self.assertIn('option value="perspective"', html)
            self.assertIn("0~50cm 흑백 깊이 영상 + 실측 거리/높이", html)
            self.assertIn("outline-toggle", html)
            self.assertIn("안전 경계 빨강: 10cm 이하", html)
            self.assertIn("UI R8", html)
            self.assertIn("3D 반구 포인트 맵", html)
            self.assertIn("fullscreen-button", html)
            self.assertIn("HanselRadarPanel", javascript)
            self.assertIn("data_base64", javascript)
            self.assertIn("Math.asin", javascript)
            self.assertIn("drawPerspectiveHeatmap", javascript)
            self.assertIn("drawPerspectivePoints", javascript)
            self.assertIn("drawPerspectiveOutline", javascript)
            self.assertIn("strokeHemisphereLatitude", javascript)
            self.assertIn("strokeHemisphereMeridian", javascript)
            self.assertIn("worldSpherical", javascript)
            self.assertIn("DANGER_RANGE_M = 0.1", javascript)
            self.assertIn("CAUTION_RANGE_M = 0.25", javascript)
            self.assertIn("drawPerspectiveSafetyZones", javascript)
            self.assertIn("drawPerspectiveSurfaceMesh", javascript)
            self.assertIn("heatmapColor", javascript)
            self.assertIn("UI_BUILD_ID", javascript)
            self.assertIn("drawDepthCamera", javascript)
            self.assertIn("depthContourCandidates", javascript)
            self.assertIn("stableDepthContour", javascript)
            self.assertIn("WALL_TRACK_HOLD_MS = 900", javascript)
            self.assertIn("requestFullscreen", javascript)
            self.assertIn("project3d", javascript)
            self.assertIn("height: Number(point[2])", javascript)
            self.assertIn("disabled_nondefault_axes", javascript)
            self.assertIn("floorDb +", javascript)
            self.assertNotIn("https://", html)
            self.assertNotIn("https://", javascript)
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


if __name__ == "__main__":
    unittest.main()
