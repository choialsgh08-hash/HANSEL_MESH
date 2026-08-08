from __future__ import annotations

import hashlib
import io
import json
import math
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from common.radar_geometry import RadarAxes
from common.sensor_contract import RadarPoint
from monitor.radar_scene import RadarSceneEstimator
from scripts.analyze_radar_scene import (
    BENCH_FIXED_TARGETS,
    analyze_records,
    load_bench_annotations,
    main,
)
from sensors.mission_log import encode_log_entry
from sensors.radar_calibration import build_clutter_model
from tests.test_radar_scene import (
    bind_calibration,
    clutter_model,
    five_point_scan,
    load_real_fixture,
    point_frame,
    radar_frame,
)


class RadarReplayReportTests(unittest.TestCase):
    def test_six_frame_fixture_reports_each_filter_stage(self):
        frames = load_real_fixture()
        model = build_clutter_model(frames, RadarAxes(), min_frames=6)
        frames = bind_calibration(frames, model.calibration_id)
        report = analyze_records(
            frames,
            RadarSceneEstimator(RadarAxes(), clutter_model=model),
            max_frames=6,
        )
        self.assertEqual(report["frames_processed"], 6)
        self.assertGreater(report["raw_points_total"], 0)
        self.assertGreater(report["self_clutter_rejected_total"], 0)
        self.assertIn("quality_gate_rejected_total", report)
        self.assertIn("tentative_point_tracks_max", report)
        self.assertIn("confirmed_point_tracks_max", report)
        self.assertIn("probability_cells_max", report)
        self.assertIn("occupied_cells_max", report)
        self.assertIn("path_switches", report)
        self.assertIn("calibration_status_counts", report)
        self.assertIn("motion_status_counts", report)
        self.assertGreaterEqual(report["producer_reported_drops_total"], 0)
        self.assertEqual(report["confirmed_tracks_in_self_clutter_zone_max"], 0)
        self.assertEqual(report["reactive_cells_in_self_clutter_zone_max"], 0)
        self.assertEqual(report["occupied_cells_in_self_clutter_zone_max"], 0)
        self.assertEqual(report["danger_frames"], 0)
        self.assertGreaterEqual(report["processing_ms_p95"], 0.0)
        self.assertIsNone(report["bench"])

    def test_empty_input_has_finite_zero_report(self):
        report = analyze_records(
            [], RadarSceneEstimator(RadarAxes(), synthetic=True), max_frames=10,
        )
        self.assertEqual(report["frames_processed"], 0)
        json.dumps(report, allow_nan=False)

    def test_counter_deltas_survive_scene_counter_reset(self):
        estimator = _ScriptedEstimator(
            [_scene(count, epoch) for count, epoch in
             zip((0, 1, 2, 0, 1), (0, 0, 0, 1, 1))]
        )
        ticks = iter((0.000, 0.001, 0.001, 0.002, 0.002,
                      0.003, 0.003, 0.004, 0.004, 0.005))
        report = analyze_records(
            [five_point_scan(0.0, index) for index in range(1, 6)],
            estimator,
            max_frames=5,
            timer=ticks.__next__,
        )
        self.assertEqual(report["probability_decayed_total"], 3)
        self.assertEqual(report["probability_ttl_expired_total"], 3)

    def test_counter_delta_uses_scene_epoch_when_post_reset_count_equals_previous(self):
        estimator = _ScriptedEstimator(
            [_scene(count, epoch) for count, epoch in ((0, 0), (1, 0), (1, 1))]
        )
        report = analyze_records(
            [five_point_scan(0.0, index) for index in range(1, 4)], estimator,
        )
        self.assertEqual(report["probability_decayed_total"], 2)
        self.assertEqual(report["probability_ttl_expired_total"], 2)

    def test_latency_summary_uses_nearest_rank_p95_and_max(self):
        ticks = iter((0.000, 0.001, 0.010, 0.015, 0.020, 0.023))
        report = analyze_records(
            [five_point_scan(0.0, index) for index in range(1, 4)],
            RadarSceneEstimator(RadarAxes(), synthetic=True),
            max_frames=3,
            timer=ticks.__next__,
        )
        self.assertAlmostEqual(report["processing_ms_mean"], 3.0)
        self.assertAlmostEqual(report["processing_ms_p95"], 5.0)
        self.assertAlmostEqual(report["processing_ms_max"], 5.0)

    def test_max_frames_must_be_positive_when_supplied(self):
        for value in (0, -1, True, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                analyze_records([], RadarSceneEstimator(RadarAxes(), synthetic=True),
                                max_frames=value)

    def test_max_frames_truncates_a_longer_input_exactly(self):
        consumed = []

        def records():
            for index in range(1, 6):
                consumed.append(index)
                yield five_point_scan(0.0, index)

        report = analyze_records(
            records(),
            RadarSceneEstimator(RadarAxes(), synthetic=True), max_frames=2,
        )
        self.assertEqual(report["records_seen"], 2)
        self.assertEqual(report["frames_processed"], 2)
        self.assertEqual(consumed, [1, 2])

    def test_repeated_scene_keeps_map_and_path_state_bounded(self):
        records = [five_point_scan(0.0, index) for index in range(1, 241)]
        report = analyze_records(
            records, RadarSceneEstimator(RadarAxes(), synthetic=True), max_frames=240,
        )
        self.assertEqual(report["frames_processed"], 240)
        self.assertLessEqual(report["probability_cells_max"], 25)
        self.assertLessEqual(report["occupied_cells_max"], 25)
        self.assertLessEqual(report["path_switches"], 3)
        json.dumps(report, allow_nan=False)

    def test_confirmed_ghost_expires_and_path_returns_straight(self):
        records = [
            point_frame(0.30, frame_number=index, snr_db=24.0)
            for index in range(1, 4)
        ] + [radar_frame((), frame_number=index) for index in range(4, 31)]
        report = analyze_records(
            records, RadarSceneEstimator(RadarAxes(), synthetic=True), max_frames=30,
        )
        self.assertGreaterEqual(report["occupied_cells_max"], 1)
        self.assertEqual(report["probability_cells_final"], 0)
        self.assertEqual(report["occupied_cells_final"], 0)
        self.assertAlmostEqual(report["planned_path_curvature_final"], 0.0)

    def test_controlled_mixed_records_report_exact_stage_counts_and_final_state(self):
        first = replace(radar_frame(frame_number=1), dropped_frames_since_previous=2)
        second = replace(radar_frame(frame_number=2), dropped_frames_since_previous=1)
        scenes = [
            _scene(0, 0, raw=3, clutter=1, quality=1, reflections=2,
                   tentative=1, confirmed_point=0, confirmed_heatmap=1),
            _scene(1, 0, raw=2, clutter=0, quality=1, reflections=1,
                   tentative=0, confirmed_point=2, confirmed_heatmap=1,
                   status="blocked_near_field", stability="route_changed_blocked",
                   curvature=0.5, switch=True, calibration="ok", motion="applied"),
        ]
        report = analyze_records(
            [object(), replace(first, complete=False), first, second],
            _ScriptedEstimator(scenes),
        )
        self.assertEqual(report["records_seen"], 4)
        self.assertEqual(report["non_radar_records_skipped"], 1)
        self.assertEqual(report["incomplete_frames_skipped"], 1)
        self.assertEqual(report["frames_processed"], 2)
        self.assertEqual(report["producer_reported_drops_total"], 3)
        self.assertEqual(report["raw_points_total"], 5)
        self.assertEqual(report["self_clutter_rejected_total"], 1)
        self.assertEqual(report["quality_gate_rejected_total"], 2)
        self.assertEqual(report["heatmap_reflections_total"], 3)
        self.assertEqual(report["tentative_point_tracks_max"], 1)
        self.assertEqual(report["confirmed_point_tracks_max"], 2)
        self.assertEqual(report["confirmed_heatmap_tracks_max"], 1)
        self.assertEqual(report["blocked_path_replans"], 1)
        self.assertEqual(report["path_switches"], 1)
        self.assertEqual(report["planned_path_status_final"], "blocked_near_field")
        self.assertEqual(report["planned_path_stability_status_final"],
                         "route_changed_blocked")
        self.assertEqual(report["planned_path_curvature_final"], 0.5)
        self.assertEqual(report["calibration_status_counts"], {"synthetic": 1, "ok": 1})
        self.assertEqual(report["motion_status_counts"],
                         {"insufficient_points": 1, "applied": 1})

    def test_state_bounds_fail_on_too_many_probability_cells_or_path_points(self):
        valid = _scene(0, 0, cells=317, occupied=317, path_points=121)
        self.assertTrue(analyze_records([radar_frame()],
                                        _ScriptedEstimator([valid]))["state_bounds_passed"])
        cases = (
            (_scene(0, 0, cells=318), "probability_cells_exceed_317"),
            (_scene(0, 0, cells=2, occupied=3), "occupied_cells_exceed_active_cells"),
            (_scene(0, 0, path_points=122), "planned_path_points_exceed_121"),
        )
        for scene, failure in cases:
            with self.subTest(failure=failure):
                report = analyze_records([radar_frame()], _ScriptedEstimator([scene]))
                self.assertFalse(report["state_bounds_passed"])
                self.assertIn(failure, report["state_bound_failures"])

    def test_cli_prints_one_json_report_and_returns_zero(self):
        with TemporaryDirectory() as directory:
            log_path = Path(directory) / "three-frames.jsonl"
            payload = b"".join(
                encode_log_entry(index, frame) + b"\n"
                for index, frame in enumerate(load_real_fixture()[:3], start=1)
            )
            log_path.write_bytes(payload)
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                exit_code = main([
                    "--input", str(log_path), "--synthetic", "--max-frames", "3",
                ])
        self.assertEqual(exit_code, 0)
        self.assertEqual(json.loads(stdout.getvalue())["frames_processed"], 3)

    def test_cli_uses_calibration_axes_and_rejects_conflicting_modes(self):
        frames = load_real_fixture()
        axes = RadarAxes(forward_axis="x", forward_sign=-1,
                         lateral_axis="y", lateral_sign=1)
        model = build_clutter_model(frames, axes, min_frames=6)
        with patch("scripts.analyze_radar_scene.load_clutter_model", return_value=model), \
             patch("scripts.analyze_radar_scene.iter_replay", return_value=()), \
             patch("scripts.analyze_radar_scene.RadarSceneEstimator") as constructor:
            constructor.return_value = RadarSceneEstimator(model.axes, synthetic=True)
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--input", "unused.jsonl",
                                       "--clutter-calibration", "model.json"]), 0)
            constructor.assert_called_once_with(model.axes, clutter_model=model)
        for argv in (
            ["--input", "unused.jsonl", "--synthetic",
             "--clutter-calibration", "model.json"],
            ["--input", "unused.jsonl"],
        ):
            with self.assertRaises(SystemExit) as caught:
                main(argv)
            self.assertEqual(caught.exception.code, 2)

    def test_cli_rejects_invalid_limits_and_returns_two_on_threshold_failure(self):
        for option, value in (("--max-frames", "0"), ("--max-p95-ms", "nan"),
                              ("--max-p95-ms", "inf"), ("--max-frame-ms", "-1")):
            with self.subTest(option=option), self.assertRaises(SystemExit) as caught:
                main(["--input", "unused.jsonl", "--synthetic", option, value])
            self.assertEqual(caught.exception.code, 2)
        failing = {"frames_processed": 1, "processing_ms_mean": 6.0,
                   "processing_ms_p95": 6.0, "processing_ms_max": 8.0,
                   "state_bounds_passed": True, "bench": None}
        with patch("scripts.analyze_radar_scene.iter_replay", return_value=()), \
             patch("scripts.analyze_radar_scene.analyze_records", return_value=failing), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--input", "unused.jsonl", "--synthetic",
                                   "--max-p95-ms", "5", "--max-frame-ms", "7"]), 2)

    def test_cli_rejects_synthetic_or_max_frames_with_annotations_and_require_without_annotations(self):
        cases = (
            ["--input", "x", "--synthetic", "--annotations", "x.json"],
            ["--input", "x", "--synthetic", "--max-frames", "1",
             "--annotations", "x.json"],
            ["--input", "x", "--synthetic", "--require-bench-pass"],
        )
        for argv in cases:
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                main(argv)

    def test_cli_prints_failed_bench_report_before_returning_two(self):
        failed = {"state_bounds_passed": True, "processing_ms_p95": 1.0,
                  "processing_ms_max": 2.0, "bench": {"passed": False}}
        stdout = io.StringIO()
        with patch("scripts.analyze_radar_scene.load_clutter_model",
                   return_value=clutter_model()), \
             patch("scripts.analyze_radar_scene.load_bench_annotations",
                   return_value={}), \
             patch("scripts.analyze_radar_scene.iter_replay", return_value=()), \
             patch("scripts.analyze_radar_scene.analyze_records",
                   return_value=failed), redirect_stdout(stdout):
            exit_code = main(["--input", "log.jsonl",
                              "--clutter-calibration", "model.json",
                              "--annotations", "bench.json",
                              "--require-bench-pass"])
        self.assertEqual(exit_code, 2)
        self.assertEqual(json.loads(stdout.getvalue()), failed)


class RadarBenchAnnotationTests(unittest.TestCase):
    def test_annotations_reject_wrong_sha_domain_profile_calibration_provenance_unknown_and_duplicate_keys(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            log_path, sidecar_path, sidecar = _write_empty_bench(root)
            self.assertEqual(
                load_bench_annotations(sidecar_path, log_path, clutter_model())["recording"]["id"],
                "empty-1",
            )
            mutations = (
                ("sha", lambda value: value.__setitem__("log_sha256", "0" * 64)),
                ("domain", lambda value: value["stream"].__setitem__("unit_id", "other")),
                ("profile", lambda value: value.__setitem__("profile_id", "wrong")),
                ("calibration", lambda value: value.__setitem__("calibration_id", "wrong")),
                ("provenance", lambda value: value["provenance"].__setitem__("operator", " ")),
                ("unknown", lambda value: value.__setitem__("extra", 1)),
            )
            for label, mutate in mutations:
                candidate = json.loads(json.dumps(sidecar))
                mutate(candidate)
                sidecar_path.write_text(json.dumps(candidate), encoding="utf-8")
                with self.subTest(label=label), self.assertRaises(ValueError):
                    load_bench_annotations(sidecar_path, log_path, clutter_model())
            duplicate = json.dumps(sidecar)[:-1] + ',"schema_version":1}'
            sidecar_path.write_text(duplicate, encoding="utf-8")
            with self.assertRaises(ValueError):
                load_bench_annotations(sidecar_path, log_path, clutter_model())

    def test_annotations_reject_empty_or_incomplete_recording_kind_coverage(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            log_path, sidecar_path, sidecar = _write_empty_bench(root)
            for windows in ([], [{"id": "wrong", "kind": "fixed_target",
                                   "start_seq": 1, "end_seq": 2,
                                   "coverage_key": "front_030_metal"}]):
                sidecar["windows"] = windows
                sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
                with self.subTest(windows=windows), self.assertRaises(ValueError):
                    load_bench_annotations(sidecar_path, log_path, clutter_model())

    def test_recording_duration_accepts_exact_sixty_seconds_and_rejects_one_ns_less(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            log_path, sidecar_path, _ = _write_empty_bench(root, 60_000_000_000)
            self.assertEqual(load_bench_annotations(
                sidecar_path, log_path, clutter_model())["_duration_s"], 60.0)
            log_path, sidecar_path, _ = _write_empty_bench(root, 59_999_999_999)
            with self.assertRaisesRegex(ValueError, "at least 60"):
                load_bench_annotations(sidecar_path, log_path, clutter_model())

    def test_annotations_reject_equal_or_regressing_monotonic_timestamps(self):
        with TemporaryDirectory() as directory:
            for duration in (0, -1):
                root = Path(directory)
                log_path, sidecar_path, _ = _write_empty_bench(root, duration)
                with self.subTest(duration=duration), self.assertRaisesRegex(
                        ValueError, "strictly increasing"):
                    load_bench_annotations(sidecar_path, log_path, clutter_model())

    def test_event_sequences_reject_duplicates_reversal_gaps_and_reported_drops(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            log_path, sidecar_path, sidecar = _write_frontal_bench(root)
            load_bench_annotations(sidecar_path, log_path, clutter_model())
            ghost = next(window for window in sidecar["windows"]
                         if window["kind"] == "ghost")
            for evidence in ([4, 4], [5, 4], [4, 6]):
                ghost["evidence_seqs"] = evidence
                sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
                with self.subTest(evidence=evidence), self.assertRaises(ValueError):
                    load_bench_annotations(sidecar_path, log_path, clutter_model())
            _write_frontal_bench(root, dropped_seq=5)
            with self.assertRaisesRegex(ValueError, "zero drops"):
                load_bench_annotations(sidecar_path, log_path, clutter_model())

    def test_one_and_two_frame_ghosts_never_enter_map_but_three_frame_annotation_is_invalid(self):
        target = {"forward_m": 0.30, "lateral_m": 0.0}
        for count in (1, 2):
            frames = [point_frame(0.30, frame_number=index) for index in range(1, count + 1)]
            scenes = [_scene_with(epoch=0, calibration="ok") for _ in frames]
            annotations = _internal_annotations(
                frames, [{"id": "ghost", "kind": "ghost", "start_seq": 1,
                          "end_seq": count, "target": target,
                          "evidence_seqs": list(range(1, count + 1))}])
            report = analyze_records(frames, _ScriptedEstimator(scenes),
                                     bench_annotations=annotations)
            self.assertTrue(report["bench"]["windows"][0]["passed"])
        with TemporaryDirectory() as directory:
            root = Path(directory)
            log_path, sidecar_path, sidecar = _write_frontal_bench(root)
            next(window for window in sidecar["windows"]
                 if window["kind"] == "ghost")["evidence_seqs"] = [4, 5, 6]
            sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_bench_annotations(sidecar_path, log_path, clutter_model())

    def test_fixed_target_pairs_same_frame_current_samples_for_population_2d_std(self):
        frames = []
        scenes = []
        for index in range(1, 31):
            raw_forward = 0.28 if index % 2 else 0.32
            filtered_forward = 0.295 if index % 2 else 0.305
            frames.append(point_frame(raw_forward, frame_number=index))
            scenes.append(_scene_with(
                calibration="ok",
                tracks=[_track(filtered_forward, map_confirmed=True, age_ms=0)],
            ))
        window = {"id": "fixed", "kind": "fixed_target", "start_seq": 1,
                  "end_seq": 30, "coverage_key": "front_030_metal"}
        report = analyze_records(frames, _ScriptedEstimator(scenes),
                                 bench_annotations=_internal_annotations(frames, [window]))
        fixed = report["bench"]["windows"][0]
        self.assertEqual(fixed["paired_target_samples"], 30)
        self.assertAlmostEqual(fixed["raw_position_std_m"], 0.02)
        self.assertAlmostEqual(fixed["filtered_position_std_m"], 0.005)
        self.assertTrue(fixed["passed"])
        short = analyze_records(frames[:29], _ScriptedEstimator(scenes[:29]),
                                bench_annotations=_internal_annotations(
                                    frames[:29], [{**window, "end_seq": 29}]))
        self.assertFalse(short["bench"]["windows"][0]["passed"])
        equal_scenes = [_scene_with(
            calibration="ok",
            tracks=[_track(0.28 if index % 2 else 0.32,
                           map_confirmed=True, age_ms=0)],
        ) for index in range(1, 31)]
        equal = analyze_records(frames, _ScriptedEstimator(equal_scenes),
                                bench_annotations=_internal_annotations(frames, [window]))
        self.assertFalse(equal["bench"]["windows"][0]["passed"])
        retained = [_scene_with(calibration="ok", tracks=[
            _track(0.30, map_confirmed=True, age_ms=125)]) for _ in frames]
        failed = analyze_records(frames, _ScriptedEstimator(retained),
                                 bench_annotations=_internal_annotations(frames, [window]))
        self.assertEqual(failed["bench"]["windows"][0]["paired_target_samples"], 0)

    def test_empty_metrics_report_exact_false_rates_switches_per_min_and_block_run(self):
        point_counts = (2, 0, 1, 1)
        frames = [_timed_frame(index, index * 20_000_000_000,
                               forward_points=point_counts[index - 1])
                  for index in range(1, 5)]
        scenes = []
        for index, confirmed in enumerate((0, 1, 1, 0)):
            scenes.append(_scene_with(
                calibration="ok", switch=index in (1, 3),
                tracks=[_track(0.30)] * confirmed,
                status="blocked_near_field" if confirmed else "clear",
            ))
        window = {"id": "empty", "kind": "empty", "start_seq": 1, "end_seq": 4}
        report = analyze_records(frames, _ScriptedEstimator(scenes),
                                 bench_annotations=_internal_annotations(frames, [window]))
        empty = report["bench"]["windows"][0]
        self.assertEqual(empty["raw_false_points_per_frame"], 1.0)
        self.assertEqual(empty["confirmed_false_obstacles_per_frame"], 0.5)
        self.assertEqual(report["bench"]["path_switches_per_min"], 2.0)
        self.assertEqual(empty["longest_confirmed_route_blocked_ms"], 40_000.0)

    def test_event_windows_require_target_roi_evidence_and_removal_presence(self):
        target = {"forward_m": 0.30, "lateral_m": 0.0}
        frames = [radar_frame(frame_number=index) for index in range(1, 3)]
        scenes = [_scene_with(calibration="ok") for _ in frames]
        ghost = {"id": "ghost", "kind": "ghost", "start_seq": 1, "end_seq": 1,
                 "target": target, "evidence_seqs": [1]}
        report = analyze_records(frames, _ScriptedEstimator(scenes),
                                 bench_annotations=_internal_annotations(frames, [ghost]))
        self.assertIn("missing_raw_target_evidence",
                      report["bench"]["windows"][0]["failures"])
        removal = {"id": "remove", "kind": "removal", "start_seq": 1,
                   "end_seq": 2, "target": target, "last_present_seq": 1}
        report = analyze_records(frames, _ScriptedEstimator(scenes),
                                 bench_annotations=_internal_annotations(frames, [removal]))
        self.assertIn("missing_removal_presence",
                      report["bench"]["windows"][0]["failures"])

    def test_removal_is_clear_on_first_observation_at_or_after_deadline(self):
        target = {"forward_m": 0.30, "lateral_m": 0.0}
        window = {"id": "remove", "kind": "removal", "start_seq": 1,
                  "end_seq": 2, "target": target, "last_present_seq": 1}
        for delay_ms, present, expected in ((2500, False, True), (2500, True, False),
                                             (2625, False, True)):
            frames = [_timed_frame(1, 1_000_000_000),
                      _timed_frame(2, 1_000_000_000 + delay_ms * 1_000_000)]
            present_scene = _scene_with(calibration="ok", tracks=[_track(0.30)])
            later = (_scene_with(calibration="ok", cells=[_cell(0.30)]) if present
                     else _scene_with(calibration="ok"))
            report = analyze_records(frames, _ScriptedEstimator([present_scene, later]),
                                     bench_annotations=_internal_annotations(frames, [window]))
            self.assertEqual(report["bench"]["windows"][0]["passed"], expected)

    def test_transient_path_holds_route_and_confirmed_collision_has_zero_frame_delay(self):
        target = {"forward_m": 0.30, "lateral_m": 0.0}
        frames = [point_frame(0.30, frame_number=index) for index in range(1, 6)]
        scenes = [_scene_with(calibration="ok") for _ in frames]
        scenes[4] = _scene_with(
            calibration="ok", tracks=[_track(0.30)], switch=True,
            status="blocked_near_field", stability="route_changed_blocked")
        windows = [
            {"id": "transient", "kind": "transient_path", "start_seq": 1,
             "end_seq": 3, "target": target, "event_seq": 1},
            {"id": "collision", "kind": "confirmed_collision", "start_seq": 4,
             "end_seq": 5, "target": target, "evidence_seqs": [4, 5],
             "confirmation_seq": 5},
        ]
        report = analyze_records(frames, _ScriptedEstimator(scenes),
                                 bench_annotations=_internal_annotations(frames, windows))
        transient, collision = report["bench"]["windows"]
        self.assertTrue(transient["passed"])
        self.assertEqual(collision["response_seq"], 5)
        self.assertEqual(collision["response_delay_frames"], 0)
        self.assertTrue(collision["passed"])

    def test_hazard_boundaries_accept_exact_010_and_013_and_reject_outside_values(self):
        frames = [radar_frame(frame_number=index) for index in range(1, 4)]
        window = {"id": "hazard", "kind": "approach_retreat", "start_seq": 1,
                  "end_seq": 3}
        scenes = [_scene_with(calibration="ok", hazard=("NORMAL", 0.11)),
                  _scene_with(calibration="ok", hazard=("DANGER", 0.10)),
                  _scene_with(calibration="ok", hazard=("NORMAL", 0.13))]
        report = analyze_records(frames, _ScriptedEstimator(scenes),
                                 bench_annotations=_internal_annotations(frames, [window]))
        self.assertTrue(report["bench"]["windows"][0]["passed"])
        scenes[1] = _scene_with(calibration="ok", hazard=("DANGER", 0.100001))
        scenes[2] = _scene_with(calibration="ok", hazard=("NORMAL", 0.129999))
        report = analyze_records(frames, _ScriptedEstimator(scenes),
                                 bench_annotations=_internal_annotations(frames, [window]))
        self.assertFalse(report["bench"]["windows"][0]["passed"])

    def test_blocked_replan_counts_only_switch_accepted_and_reports_blocked_near_run(self):
        frames = [_timed_frame(index, index * 1_000_000_000)
                  for index in range(1, 4)]
        scenes = [
            _scene_with(status="blocked_near_field", stability="route_changed_blocked"),
            _scene_with(status="blocked_near_field", stability="route_changed_blocked",
                        switch=True),
            _scene_with(status="clear"),
        ]
        report = analyze_records(frames, _ScriptedEstimator(scenes))
        self.assertEqual(report["blocked_path_replans"], 1)
        self.assertEqual(report["blocked_near_field_longest_ms"], 2000.0)


def _timed_frame(seq, monotonic_ns, forward_points=0, dropped=0):
    points = tuple(RadarPoint(0.0, 0.30, 0.01, 0.0, 24.0)
                   for _ in range(forward_points))
    frame = radar_frame(points, frame_number=seq,
                        dropped_frames_since_previous=dropped)
    return replace(frame, header=replace(frame.header, monotonic_ns=monotonic_ns))


def _track(forward, *, map_confirmed=False, age_ms=0):
    return {"forward_m": forward, "lateral_m": 0.0, "source": "point",
            "age_ms": age_ms, "point_confirmed": True,
            "heatmap_confirmed": False, "map_confirmed": map_confirmed}


def _cell(forward, *, occupied=True):
    return {"forward_m": forward, "lateral_m": 0.0, "occupied": occupied}


def _scene_with(*, epoch=0, calibration="synthetic", tracks=None, cells=None,
                switch=False, status="clear", stability="replanned",
                curvature=0.0, hazard=("NORMAL", None)):
    result = _scene(epoch=epoch, calibration=calibration, switch=switch,
                    status=status, stability=stability, curvature=curvature)
    result["tracks"] = list(tracks or [])
    result["probabilistic_local_map"]["cells"] = list(cells or [])
    result["diagnostics"]["probability_active_cell_count"] = len(cells or [])
    result["diagnostics"]["probability_occupied_cell_count"] = sum(
        cell.get("occupied") is True for cell in (cells or []))
    result["hazard"] = {"level": hazard[0], "nearest_confirmed_m": hazard[1]}
    return result


def _internal_annotations(frames, windows):
    first = frames[0]
    last = frames[-1]
    duration = max(60.0, (last.header.monotonic_ns
                          - first.header.monotonic_ns) / 1e9)
    return {
        "schema_version": 1,
        "recording": {"id": "scripted", "kind": "frontal_targets",
                      "start_seq": first.header.seq, "end_seq": last.header.seq},
        "provenance": {"operator": "test"},
        "windows": windows,
        "_annotations_sha256": "a" * 64,
        "_log_sha256": "b" * 64,
        "_duration_s": duration,
        "_domain": tuple(getattr(first.header, field) for field in
                         ("mission_id", "unit_id", "boot_id", "producer_id", "stream_id")),
    }


def _provenance():
    return {
        "board_serial": "BOARD-1",
        "robot_mount_id": "head-final-mount-v1",
        "mount_orientation": "forward level, connector down",
        "usb_cable_adapter": "USB-C adapter A",
        "power_state": "bench supply",
        "physical_location": "lab-a",
        "ambient_temperature_c": 24.5,
        "created_utc": "2026-08-07T08:00:00Z",
        "operator": "test operator",
    }


def _base_sidecar(log_path, frames, kind, windows):
    first, last = frames[0], frames[-1]
    return {
        "schema_version": 1,
        "log_file": log_path.name,
        "log_sha256": hashlib.sha256(log_path.read_bytes()).hexdigest(),
        "profile_id": clutter_model().profile_id,
        "calibration_id": clutter_model().calibration_id,
        "provenance": _provenance(),
        "stream": {field: getattr(first.header, field) for field in
                   ("mission_id", "unit_id", "boot_id", "producer_id", "stream_id")},
        "recording": {"id": "empty-1" if kind == "empty_mounted" else "front-1",
                      "kind": kind, "start_seq": first.header.seq,
                      "end_seq": last.header.seq},
        "windows": windows,
    }


def _write_log(path, frames):
    path.write_bytes(b"".join(encode_log_entry(index, frame) + b"\n"
                              for index, frame in enumerate(frames, start=1)))


def _write_empty_bench(root, duration_ns=60_000_000_000):
    start_ns = 1_000_000_000
    frames = [_timed_frame(1, start_ns), _timed_frame(2, start_ns + duration_ns)]
    log_path = root / "empty.jsonl"
    sidecar_path = root / "empty.radar-bench.json"
    _write_log(log_path, frames)
    windows = [{"id": "empty", "kind": "empty", "start_seq": 1, "end_seq": 2}]
    sidecar = _base_sidecar(log_path, frames, "empty_mounted", windows)
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
    return log_path, sidecar_path, sidecar


def _write_frontal_bench(root, dropped_seq=None):
    timestamps = [index * 1_000_000_000 for index in range(1, 14)]
    timestamps[-1] = 61_000_000_000
    frames = [_timed_frame(index, timestamps[index - 1],
                           dropped=1 if index == dropped_seq else 0)
              for index in range(1, 14)]
    log_path = root / "front.jsonl"
    sidecar_path = root / "front.radar-bench.json"
    _write_log(log_path, frames)
    target = {"forward_m": 0.30, "lateral_m": 0.0}
    windows = [
        {"id": "f030", "kind": "fixed_target", "start_seq": 1, "end_seq": 1,
         "coverage_key": "front_030_metal"},
        {"id": "f050", "kind": "fixed_target", "start_seq": 2, "end_seq": 2,
         "coverage_key": "front_050_metal"},
        {"id": "f100", "kind": "fixed_target", "start_seq": 3, "end_seq": 3,
         "coverage_key": "front_100_metal"},
        {"id": "ghost", "kind": "ghost", "start_seq": 4, "end_seq": 5,
         "target": target, "evidence_seqs": [4, 5]},
        {"id": "remove", "kind": "removal", "start_seq": 6, "end_seq": 9,
         "target": target, "last_present_seq": 6},
        {"id": "transient", "kind": "transient_path", "start_seq": 9,
         "end_seq": 11, "target": target, "event_seq": 9},
        {"id": "collision", "kind": "confirmed_collision", "start_seq": 12,
         "end_seq": 13, "target": target, "evidence_seqs": [12, 13],
         "confirmation_seq": 13},
    ]
    sidecar = _base_sidecar(log_path, frames, "frontal_targets", windows)
    sidecar_path.write_text(json.dumps(sidecar), encoding="utf-8")
    return log_path, sidecar_path, sidecar


class _ScriptedEstimator:
    axes = RadarAxes()

    def __init__(self, scenes):
        self._scenes = iter(scenes)

    def ingest(self, _record, received_at):
        self.received_at = received_at

    def snapshot(self, now):
        return next(self._scenes)


def _scene(
    cumulative=0,
    epoch=0,
    *,
    raw=0,
    clutter=0,
    quality=0,
    reflections=0,
    tentative=0,
    confirmed_point=0,
    confirmed_heatmap=0,
    cells=0,
    occupied=0,
    path_points=0,
    status="clear",
    stability="replanned",
    curvature=0.0,
    switch=False,
    calibration="synthetic",
    motion="insufficient_points",
):
    probability_cells = [
        {"forward_m": 0.20, "lateral_m": 0.0, "occupied": index < occupied}
        for index in range(cells)
    ]
    if occupied > cells:
        probability_cells.extend(
            {"forward_m": 0.20, "lateral_m": 0.0, "occupied": True}
            for _ in range(occupied - cells)
        )
    return {
        "calibration_status": calibration,
        "motion_compensation": {"status": motion},
        "probabilistic_local_map": {"cells": probability_cells[:cells]},
        "tracks": [],
        "near_occupancy": {"cells": []},
        "planned_path": {
            "status": status,
            "stability_status": stability,
            "curvature_per_m": curvature,
            "switch_accepted": switch,
            "points": [[0.0, 0.0]] * path_points,
        },
        "hazard": {"level": "NORMAL", "nearest_confirmed_m": None},
        "diagnostics": {
            "raw_point_count": raw,
            "self_clutter_rejected_count": clutter,
            "quality_gate_rejected_count": quality,
            "tentative_point_track_count": tentative,
            "confirmed_point_track_count": confirmed_point,
            "confirmed_heatmap_track_count": confirmed_heatmap,
            "heatmap_reflection_count": reflections,
            "probability_decayed_cell_count": cumulative,
            "probability_ttl_expired_cell_count": cumulative,
            "probability_active_cell_count": cells,
            "probability_occupied_cell_count": occupied,
            "path_switch_accepted_count": cumulative,
            "scene_epoch": epoch,
        },
    }


if __name__ == "__main__":
    unittest.main()
