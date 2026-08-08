"""Read-only replay diagnostics and strict physical-bench evaluation."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import time
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path

from common.radar_geometry import RadarAxes
from common.sensor_contract import RadarFrame
from monitor.radar_scene import RadarSceneEstimator
from sensors.mission_log import iter_replay
from sensors.radar_calibration import load_clutter_model


BENCH_ASSOCIATION_RADIUS_M = 0.15
BENCH_MIN_POSITION_SAMPLES = 30
BENCH_PERSISTENCE_LIMIT_MS = 2500.0
BENCH_FIXED_TARGETS = {
    "front_030_metal": (0.30, 0.0, "metal"),
    "front_050_metal": (0.50, 0.0, "metal"),
    "front_100_metal": (1.00, 0.0, "metal"),
    "azimuth_neg30_metal": (
        0.50 * math.cos(math.radians(30.0)),
        -0.50 * math.sin(math.radians(30.0)),
        "metal",
    ),
    "azimuth_pos30_metal": (
        0.50 * math.cos(math.radians(30.0)),
        0.50 * math.sin(math.radians(30.0)),
        "metal",
    ),
    "front_050_wood": (0.50, 0.0, "wood"),
}

_RECORDING_KINDS = {
    "empty_mounted", "frontal_targets", "angled_targets", "materials",
    "approach_retreat",
}
_WINDOW_FIELDS = {
    "empty": set(),
    "fixed_target": {"coverage_key"},
    "ghost": {"target", "evidence_seqs"},
    "removal": {"target", "last_present_seq"},
    "transient_path": {"target", "event_seq"},
    "confirmed_collision": {"target", "evidence_seqs", "confirmation_seq"},
    "approach_retreat": set(),
}
_DOMAIN_FIELDS = ("mission_id", "unit_id", "boot_id", "producer_id", "stream_id")


def _strict_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _keys(value, expected, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    actual = set(value)
    missing = expected - actual
    unknown = actual - expected
    if missing:
        raise ValueError(f"{label} missing keys: {', '.join(sorted(missing))}")
    if unknown:
        raise ValueError(f"{label} unknown keys: {', '.join(sorted(unknown))}")


def _integer(value, label, minimum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    if minimum is not None and value < minimum:
        raise ValueError(f"{label} must be at least {minimum}")
    return value


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _text(value, label):
    if not isinstance(value, str) or value != value.strip() or not value:
        raise ValueError(f"{label} must be a trimmed non-empty string")
    return value


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _domain(header):
    return tuple(getattr(header, field) for field in _DOMAIN_FIELDS)


def _target(value, label):
    _keys(value, {"forward_m", "lateral_m"}, label)
    forward = _finite(value["forward_m"], f"{label}.forward_m")
    lateral = _finite(value["lateral_m"], f"{label}.lateral_m")
    if forward <= 0 or math.hypot(forward, lateral) > 0.50 + 1e-12:
        raise ValueError(f"{label} must lie inside the forward local map")
    return {"forward_m": forward, "lateral_m": lateral}


def _validate_coverage(kind, windows):
    kinds = [window["kind"] for window in windows]
    fixed = [window.get("coverage_key") for window in windows
             if window["kind"] == "fixed_target"]
    if kind == "empty_mounted":
        if len(windows) != 1 or kinds != ["empty"]:
            raise ValueError("empty_mounted requires one spanning empty window")
    elif kind == "frontal_targets":
        required_fixed = {"front_030_metal", "front_050_metal", "front_100_metal"}
        required_events = {"ghost", "removal", "transient_path", "confirmed_collision"}
        if not required_fixed.issubset(fixed) or not required_events.issubset(kinds):
            raise ValueError("frontal_targets coverage is incomplete")
        if not set(kinds).issubset({"fixed_target"} | required_events):
            raise ValueError("frontal_targets has an incompatible window")
    elif kind == "angled_targets":
        if not {"azimuth_neg30_metal", "azimuth_pos30_metal"}.issubset(fixed):
            raise ValueError("angled_targets coverage is incomplete")
        if set(kinds) != {"fixed_target"}:
            raise ValueError("angled_targets has an incompatible window")
    elif kind == "materials":
        if not {"front_050_metal", "front_050_wood"}.issubset(fixed):
            raise ValueError("materials coverage is incomplete")
        if set(kinds) != {"fixed_target"}:
            raise ValueError("materials has an incompatible window")
    elif kind == "approach_retreat":
        if len(windows) != 1 or kinds != ["approach_retreat"]:
            raise ValueError("approach_retreat requires exactly one matching window")


def load_bench_annotations(path: Path, input_path: Path, clutter_model) -> dict:
    """Load and bind a strict sidecar to one complete canonical radar stream."""
    path = Path(path)
    input_path = Path(input_path)
    try:
        annotations = json.loads(path.read_text(encoding="utf-8"),
                                 object_pairs_hook=_strict_pairs)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid annotation JSON: {exc}") from exc
    _keys(annotations, {
        "schema_version", "log_file", "log_sha256", "profile_id",
        "calibration_id", "provenance", "stream", "recording", "windows",
    }, "annotations")
    if (_integer(annotations["schema_version"], "schema_version", 1) != 1):
        raise ValueError("schema_version must equal 1")
    if annotations["log_file"] != input_path.name:
        raise ValueError("log_file does not match input filename")
    log_hash = annotations["log_sha256"]
    if (not isinstance(log_hash, str) or len(log_hash) != 64
            or any(character not in "0123456789abcdef" for character in log_hash)):
        raise ValueError("log_sha256 must be 64 lowercase hex characters")
    actual_hash = _sha256(input_path)
    if log_hash != actual_hash:
        raise ValueError("log_sha256 does not match input")
    if annotations["profile_id"] != clutter_model.profile_id:
        raise ValueError("profile_id does not match calibration")
    if annotations["calibration_id"] != clutter_model.calibration_id:
        raise ValueError("calibration_id does not match calibration")

    provenance = annotations["provenance"]
    provenance_fields = {
        "board_serial", "robot_mount_id", "mount_orientation",
        "usb_cable_adapter", "power_state", "physical_location",
        "ambient_temperature_c", "created_utc", "operator",
    }
    _keys(provenance, provenance_fields, "provenance")
    for field in provenance_fields - {"ambient_temperature_c", "created_utc"}:
        _text(provenance[field], f"provenance.{field}")
    _finite(provenance["ambient_temperature_c"], "provenance.ambient_temperature_c")
    created = _text(provenance["created_utc"], "provenance.created_utc")
    if not re.fullmatch(
            r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z", created):
        raise ValueError("provenance.created_utc must be UTC RFC 3339 ending in Z")
    try:
        parsed = datetime.fromisoformat(created[:-1] + "+00:00")
    except ValueError as exc:
        raise ValueError("provenance.created_utc must be RFC 3339") from exc
    if parsed.utcoffset() is None or parsed.utcoffset().total_seconds() != 0:
        raise ValueError("provenance.created_utc must be UTC")

    stream = annotations["stream"]
    _keys(stream, set(_DOMAIN_FIELDS), "stream")
    for field in _DOMAIN_FIELDS:
        _text(stream[field], f"stream.{field}")
    expected_domain = tuple(stream[field] for field in _DOMAIN_FIELDS)

    recording = annotations["recording"]
    _keys(recording, {"id", "kind", "start_seq", "end_seq"}, "recording")
    _text(recording["id"], "recording.id")
    recording_kind = recording["kind"]
    if not isinstance(recording_kind, str) or recording_kind not in _RECORDING_KINDS:
        raise ValueError("recording.kind is unsupported")
    start_seq = _integer(recording["start_seq"], "recording.start_seq", 0)
    end_seq = _integer(recording["end_seq"], "recording.end_seq", 0)
    if start_seq > end_seq:
        raise ValueError("recording sequence range is reversed")

    windows = annotations["windows"]
    if not isinstance(windows, list) or not windows:
        raise ValueError("windows must be a non-empty array")
    ids = set()
    by_kind = {}
    for index, window in enumerate(windows):
        label = f"windows[{index}]"
        if not isinstance(window, dict):
            raise ValueError(f"{label} must be an object")
        kind = window.get("kind")
        if not isinstance(kind, str) or kind not in _WINDOW_FIELDS:
            raise ValueError(f"{label}.kind is unsupported")
        _keys(window, {"id", "kind", "start_seq", "end_seq"} | _WINDOW_FIELDS[kind], label)
        window_id = _text(window["id"], f"{label}.id")
        if window_id in ids:
            raise ValueError("window ids must be unique")
        ids.add(window_id)
        window_start = _integer(window["start_seq"], f"{label}.start_seq", 0)
        window_end = _integer(window["end_seq"], f"{label}.end_seq", 0)
        if window_start > window_end or window_start < start_seq or window_end > end_seq:
            raise ValueError(f"{label} range is invalid")
        for other_start, other_end in by_kind.setdefault(kind, []):
            if max(window_start, other_start) <= min(window_end, other_end):
                raise ValueError(f"overlapping {kind} windows")
        by_kind[kind].append((window_start, window_end))
        if kind == "fixed_target":
            coverage_key = window["coverage_key"]
            if (not isinstance(coverage_key, str)
                    or coverage_key not in BENCH_FIXED_TARGETS):
                raise ValueError(f"{label}.coverage_key is unsupported")
        if "target" in window:
            window["target"] = _target(window["target"], f"{label}.target")
        if "evidence_seqs" in window:
            evidence = window["evidence_seqs"]
            required_length = 2 if kind == "confirmed_collision" else None
            if (not isinstance(evidence, list) or
                    (required_length is not None and len(evidence) != required_length) or
                    (kind == "ghost" and len(evidence) not in (1, 2))):
                raise ValueError(f"{label}.evidence_seqs has invalid length")
            for item in evidence:
                _integer(item, f"{label}.evidence_seqs", 0)
            if len(set(evidence)) != len(evidence) or evidence != sorted(evidence):
                raise ValueError(f"{label}.evidence_seqs must be unique and ordered")
            if any(item < window_start or item > window_end for item in evidence):
                raise ValueError(f"{label}.evidence_seqs lies outside its window")
        for field in ("last_present_seq", "event_seq", "confirmation_seq"):
            if field in window:
                seq = _integer(window[field], f"{label}.{field}", 0)
                if seq < window_start or seq > window_end:
                    raise ValueError(f"{label}.{field} lies outside its window")
        if kind == "confirmed_collision" and window["confirmation_seq"] != window["evidence_seqs"][-1]:
            raise ValueError("confirmation_seq must equal final evidence seq")
    _validate_coverage(recording_kind, windows)
    if recording_kind == "empty_mounted":
        only = windows[0]
        if only["start_seq"] != start_seq or only["end_seq"] != end_seq:
            raise ValueError("empty window must span the recording")

    complete = []
    for entry in iter_replay(input_path):
        record = entry.record
        if isinstance(record, RadarFrame) and record.complete:
            if _domain(record.header) != expected_domain:
                raise ValueError("complete radar frame belongs to another stream domain")
            complete.append(record)
    if not complete:
        raise ValueError("bound log contains no complete radar frames")
    if complete[0].header.seq != start_seq or complete[-1].header.seq != end_seq:
        raise ValueError("recording bounds must equal first/last complete frame")
    previous_seq = None
    previous_ns = None
    by_seq = {}
    for record in complete:
        seq = record.header.seq
        timestamp = record.header.monotonic_ns
        if previous_seq is not None and seq <= previous_seq:
            raise ValueError("complete frame seqs must be strictly increasing")
        if previous_ns is not None and timestamp <= previous_ns:
            raise ValueError("complete monotonic timestamps must be strictly increasing")
        if record.profile_id != annotations["profile_id"]:
            raise ValueError("frame profile_id does not match annotations")
        if record.header.calibration_id != annotations["calibration_id"]:
            raise ValueError("frame calibration_id does not match annotations")
        by_seq[seq] = record
        previous_seq = seq
        previous_ns = timestamp
    duration_s = (complete[-1].header.monotonic_ns - complete[0].header.monotonic_ns) / 1e9
    if duration_s < 60.0:
        raise ValueError("recording duration must be at least 60.0 seconds")
    for window in windows:
        seqs = [seq for seq in by_seq if window["start_seq"] <= seq <= window["end_seq"]]
        if not seqs:
            raise ValueError(f"window {window['id']} contains no complete frame")
        event_groups = []
        if "evidence_seqs" in window:
            event_groups.append(window["evidence_seqs"])
        if window["kind"] == "transient_path":
            event_groups.append([window["event_seq"], window["event_seq"] + 1,
                                 window["event_seq"] + 2])
        for group in event_groups:
            for offset, seq in enumerate(group):
                if seq < window["start_seq"] or seq > window["end_seq"]:
                    raise ValueError("event evidence must remain inside its window")
                if seq not in by_seq:
                    raise ValueError("event seq is not a complete processed frame")
                if offset and (seq != group[offset - 1] + 1
                               or by_seq[seq].dropped_frames_since_previous != 0):
                    raise ValueError("event evidence must be consecutive with zero drops")
        if window["kind"] == "removal":
            last_seq = window["last_present_seq"]
            if last_seq not in by_seq:
                raise ValueError("last_present_seq is not a complete frame")
            deadline = by_seq[last_seq].header.monotonic_ns + 2_500_000_000
            if not any(record.header.monotonic_ns >= deadline
                       and window["start_seq"] <= record.header.seq <= window["end_seq"]
                       for record in complete):
                raise ValueError("removal window lacks an observation at/after TTL")
    result = dict(annotations)
    result["_annotations_sha256"] = _sha256(path)
    result["_log_sha256"] = actual_hash
    result["_duration_s"] = duration_s
    result["_domain"] = expected_domain
    return result


def _initial_report():
    return {
        "records_seen": 0, "frames_processed": 0,
        "non_radar_records_skipped": 0, "incomplete_frames_skipped": 0,
        "producer_reported_drops_total": 0, "raw_points_total": 0,
        "self_clutter_rejected_total": 0, "quality_gate_rejected_total": 0,
        "heatmap_reflections_total": 0, "tentative_point_tracks_max": 0,
        "confirmed_point_tracks_max": 0, "confirmed_heatmap_tracks_max": 0,
        "probability_cells_max": 0, "occupied_cells_max": 0,
        "confirmed_tracks_in_self_clutter_zone_max": 0,
        "reactive_cells_in_self_clutter_zone_max": 0,
        "occupied_cells_in_self_clutter_zone_max": 0,
        "probability_decayed_total": 0, "probability_ttl_expired_total": 0,
        "path_switches": 0, "path_switches_per_min": 0.0,
        "danger_frames": 0, "blocked_path_replans": 0,
        "blocked_near_field_frames": 0, "blocked_near_field_longest_ms": 0.0,
        "duration_s": 0.0, "probability_cells_final": 0,
        "occupied_cells_final": 0, "planned_path_points_max": 0,
        "state_bounds_passed": True, "state_bound_failures": [],
        "planned_path_status_final": "no_frames",
        "planned_path_stability_status_final": "no_frames",
        "planned_path_curvature_final": 0.0,
        "calibration_status_counts": {}, "motion_status_counts": {},
        "processing_ms_mean": 0.0, "processing_ms_p95": 0.0,
        "processing_ms_max": 0.0, "bench": None,
    }


def _nonnegative_int(value, label):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return value


def analyze_records(records, estimator, max_frames=None, timer=time.perf_counter,
                    bench_annotations=None):
    if (max_frames is not None and
            (isinstance(max_frames, bool) or not isinstance(max_frames, int)
             or max_frames <= 0)):
        raise ValueError("max_frames must be a positive integer or None")
    processing_ms = []
    report = _initial_report()
    cumulative_keys = {
        "probability_decayed_cell_count": "probability_decayed_total",
        "probability_ttl_expired_cell_count": "probability_ttl_expired_total",
    }
    previous_cumulative = {key: 0 for key in cumulative_keys}
    previous_scene_epoch = None
    first_timestamp = None
    last_timestamp = None
    blocked_start = None
    bench_frames = []
    iterator = iter(records)
    while max_frames is None or report["frames_processed"] < max_frames:
        try:
            record = next(iterator)
        except StopIteration:
            break
        report["records_seen"] += 1
        if not isinstance(record, RadarFrame):
            report["non_radar_records_skipped"] += 1
            continue
        if not record.complete:
            report["incomplete_frames_skipped"] += 1
            continue
        received_at = record.header.monotonic_ns / 1e9
        started = timer()
        estimator.ingest(record, received_at=received_at)
        scene = estimator.snapshot(now=received_at)
        elapsed = max(0.0, (timer() - started) * 1000.0)
        if not math.isfinite(elapsed):
            raise ValueError("timer produced non-finite processing latency")
        processing_ms.append(elapsed)
        diagnostics = scene["diagnostics"]
        scene_epoch = _nonnegative_int(diagnostics["scene_epoch"], "scene_epoch")
        if previous_scene_epoch is not None and scene_epoch < previous_scene_epoch:
            raise ValueError("scene_epoch must be monotonic")
        probability_cells = scene["probabilistic_local_map"]["cells"]
        tracks = scene["tracks"]
        near_cells = scene["near_occupancy"]["cells"]
        planned_path = scene["planned_path"]
        path_points = planned_path.get("points", [])
        occupied_count = sum(cell.get("occupied") is True for cell in probability_cells)
        active_count = len(probability_cells)
        diagnostic_active = diagnostics.get("probability_active_cell_count")
        diagnostic_occupied = diagnostics.get("probability_occupied_cell_count")
        if diagnostic_active is not None:
            diagnostic_active = _nonnegative_int(
                diagnostic_active, "probability_active_cell_count")
        if diagnostic_occupied is not None:
            diagnostic_occupied = _nonnegative_int(
                diagnostic_occupied, "probability_occupied_cell_count")
        report["frames_processed"] += 1
        report["producer_reported_drops_total"] += record.dropped_frames_since_previous
        for source, target in (
            ("raw_point_count", "raw_points_total"),
            ("self_clutter_rejected_count", "self_clutter_rejected_total"),
            ("quality_gate_rejected_count", "quality_gate_rejected_total"),
            ("heatmap_reflection_count", "heatmap_reflections_total"),
        ):
            report[target] += int(diagnostics[source])
        for source, target in (
            ("tentative_point_track_count", "tentative_point_tracks_max"),
            ("confirmed_point_track_count", "confirmed_point_tracks_max"),
            ("confirmed_heatmap_track_count", "confirmed_heatmap_tracks_max"),
        ):
            report[target] = max(report[target], int(diagnostics[source]))
        report["probability_cells_max"] = max(report["probability_cells_max"], active_count)
        report["occupied_cells_max"] = max(report["occupied_cells_max"], occupied_count)

        def inside(item, radius=0.15):
            return math.hypot(float(item["forward_m"]),
                              float(item["lateral_m"])) <= radius
        report["confirmed_tracks_in_self_clutter_zone_max"] = max(
            report["confirmed_tracks_in_self_clutter_zone_max"],
            sum(inside(track) and bool(track.get("point_confirmed")
                                       or track.get("heatmap_confirmed"))
                for track in tracks),
        )
        report["reactive_cells_in_self_clutter_zone_max"] = max(
            report["reactive_cells_in_self_clutter_zone_max"],
            sum(inside(cell) for cell in near_cells),
        )
        report["occupied_cells_in_self_clutter_zone_max"] = max(
            report["occupied_cells_in_self_clutter_zone_max"],
            sum(cell.get("occupied") is True and inside(cell)
                for cell in probability_cells),
        )
        report["probability_cells_final"] = active_count
        report["occupied_cells_final"] = occupied_count
        report["planned_path_points_max"] = max(report["planned_path_points_max"],
                                                 len(path_points))
        failures = []
        if active_count > 317:
            failures.append("probability_cells_exceed_317")
        if occupied_count > active_count:
            failures.append("occupied_cells_exceed_active_cells")
        if diagnostic_active is not None and diagnostic_active != active_count:
            failures.append("probability_active_diagnostic_mismatch")
        if diagnostic_occupied is not None and diagnostic_occupied != occupied_count:
            failures.append("probability_occupied_diagnostic_mismatch")
        if len(path_points) > 121:
            failures.append("planned_path_points_exceed_121")
        if not all(isinstance(point, (list, tuple)) and len(point) == 2
                   and all(math.isfinite(float(value)) for value in point)
                   and math.hypot(float(point[0]), float(point[1])) <= 0.60 + 1e-9
                   for point in path_points):
            failures.append("planned_path_point_out_of_bounds")
        if failures:
            report["state_bounds_passed"] = False
            for failure in failures:
                if failure not in report["state_bound_failures"]:
                    report["state_bound_failures"].append(failure)
        report["planned_path_status_final"] = str(planned_path["status"])
        report["planned_path_stability_status_final"] = str(planned_path["stability_status"])
        report["planned_path_curvature_final"] = float(planned_path["curvature_per_m"])
        if scene["hazard"]["level"] == "DANGER":
            report["danger_frames"] += 1
        if (planned_path["switch_accepted"] is True
                and planned_path["stability_status"] == "route_changed_blocked"):
            report["blocked_path_replans"] += 1
        if planned_path["switch_accepted"] is True:
            report["path_switches"] += 1
        blocked = planned_path["status"] == "blocked_near_field"
        if blocked:
            report["blocked_near_field_frames"] += 1
            if blocked_start is None:
                blocked_start = received_at
        elif blocked_start is not None:
            report["blocked_near_field_longest_ms"] = max(
                report["blocked_near_field_longest_ms"],
                max(0.0, (received_at - blocked_start) * 1000.0),
            )
            blocked_start = None
        for diagnostic_key, report_key in cumulative_keys.items():
            current = _nonnegative_int(diagnostics[diagnostic_key], diagnostic_key)
            previous = previous_cumulative[diagnostic_key]
            if previous_scene_epoch is None or scene_epoch != previous_scene_epoch:
                report[report_key] += current
            else:
                report[report_key] += max(0, current - previous)
            previous_cumulative[diagnostic_key] = current
        previous_scene_epoch = scene_epoch
        calibration = str(scene.get("calibration_status", "missing"))
        report["calibration_status_counts"][calibration] = (
            report["calibration_status_counts"].get(calibration, 0) + 1)
        motion = str(scene.get("motion_compensation", {}).get("status", "missing"))
        report["motion_status_counts"][motion] = report["motion_status_counts"].get(motion, 0) + 1
        if first_timestamp is None:
            first_timestamp = received_at
        last_timestamp = received_at
        if bench_annotations is not None:
            bench_frames.append((record, scene, elapsed))
    if blocked_start is not None and last_timestamp is not None:
        report["blocked_near_field_longest_ms"] = max(
            report["blocked_near_field_longest_ms"],
            max(0.0, (last_timestamp - blocked_start) * 1000.0),
        )
    if first_timestamp is not None and last_timestamp is not None:
        report["duration_s"] = max(0.0, last_timestamp - first_timestamp)
    if report["duration_s"]:
        report["path_switches_per_min"] = report["path_switches"] * 60.0 / report["duration_s"]
    ordered = sorted(processing_ms)
    if ordered:
        report["processing_ms_mean"] = sum(ordered) / len(ordered)
        report["processing_ms_p95"] = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
        report["processing_ms_max"] = ordered[-1]
    if bench_annotations is not None:
        report["bench"] = _evaluate_bench(bench_annotations, bench_frames, estimator,
                                           report)
    json.dumps(report, allow_nan=False)
    return report


def _distance(item, target):
    return math.hypot(float(item["forward_m"]) - target["forward_m"],
                      float(item["lateral_m"]) - target["lateral_m"])


def _nearest(items, target, predicate=lambda _item: True):
    candidates = [item for item in items
                  if predicate(item) and _distance(item, target) <= BENCH_ASSOCIATION_RADIUS_M]
    return min(candidates, key=lambda item: _distance(item, target)) if candidates else None


def _raw_points(record, axes):
    return [{"forward_m": forward, "lateral_m": lateral}
            for forward, lateral in (axes.map_point(point) for point in record.points)]


def _current_confirmed(scene, target, require_map=False):
    def matches(track):
        age = track.get("age_ms")
        current = (not isinstance(age, bool)
                   and isinstance(age, (int, float))
                   and math.isfinite(float(age)) and float(age) == 0.0)
        return (track.get("source") == "point" and current
                and (track.get("map_confirmed") is True if require_map
                     else track.get("point_confirmed") is True))
    return _nearest(scene.get("tracks", []), target, lambda track: (
        matches(track)
    ))


def _current_confirmed_evidence(scene, target):
    def matches(track):
        age = track.get("age_ms")
        current = (not isinstance(age, bool)
                   and isinstance(age, (int, float))
                   and math.isfinite(float(age)) and float(age) == 0.0)
        if not current:
            return False
        return ((track.get("source") == "point"
                 and track.get("point_confirmed") is True)
                or (track.get("source") == "heatmap"
                    and track.get("heatmap_confirmed") is True))
    return _nearest(scene.get("tracks", []), target, matches)


def _roi_state_present(scene, target):
    if _nearest(scene.get("tracks", []), target, lambda track: bool(
            track.get("point_confirmed") or track.get("heatmap_confirmed"))):
        return True
    return _nearest(scene.get("probabilistic_local_map", {}).get("cells", []),
                    target) is not None


def _route_identity(scene):
    path = scene["planned_path"]
    return (path["status"] == "blocked_near_field", float(path["curvature_per_m"]))


def _route_changed(left, right):
    return left[0] != right[0] or not math.isclose(left[1], right[1], abs_tol=1e-9,
                                                   rel_tol=0.0)


def _scatter(samples):
    if not samples:
        return None
    mean_f = sum(forward for forward, _ in samples) / len(samples)
    mean_l = sum(lateral for _, lateral in samples) / len(samples)
    return math.sqrt(sum((forward - mean_f) ** 2 + (lateral - mean_l) ** 2
                         for forward, lateral in samples) / len(samples))


def _window_base(annotation, frames):
    duration = 0.0
    if frames:
        duration = max(0.0, (frames[-1][0].header.monotonic_ns
                             - frames[0][0].header.monotonic_ns) / 1e9)
    return {"id": annotation["id"], "kind": annotation["kind"],
            "frames_evaluated": len(frames), "duration_s": duration,
            "passed": True, "failures": []}


def _fail(window, message):
    if message not in window["failures"]:
        window["failures"].append(message)
    window["passed"] = False


def _evaluate_bench(annotations, frames, estimator, report):
    expected_domain = tuple(annotations["_domain"])
    indexed = {record.header.seq: (record, scene, elapsed)
               for record, scene, elapsed in frames
               if _domain(record.header) == expected_domain}
    ordered_all = [indexed[seq] for seq in sorted(indexed)]
    windows_report = []
    for annotation in annotations["windows"]:
        selected = [indexed[seq] for seq in sorted(indexed)
                    if annotation["start_seq"] <= seq <= annotation["end_seq"]]
        result = _window_base(annotation, selected)
        kind = annotation["kind"]
        if kind == "empty":
            raw_total = sum(len(record.points) for record, _, _ in selected)
            confirmed_total = sum(sum(bool(track.get("point_confirmed")
                                                   or track.get("heatmap_confirmed"))
                                      for track in scene.get("tracks", []))
                                  for _, scene, _ in selected)
            count = len(selected)
            result["raw_false_points_per_frame"] = raw_total / count if count else 0.0
            result["confirmed_false_obstacles_per_frame"] = (
                confirmed_total / count if count else 0.0)
            longest = 0.0
            run_start = None
            for record, scene, _ in selected:
                path = scene["planned_path"]
                evidence = (any(bool(track.get("point_confirmed")
                                          or track.get("heatmap_confirmed"))
                                for track in scene.get("tracks", []))
                            or any(cell.get("occupied") is True
                                   for cell in scene.get("probabilistic_local_map", {})
                                   .get("cells", [])))
                blocked = evidence and (path["status"] == "blocked_near_field"
                                        or path["stability_status"] == "route_changed_blocked"
                                        or path.get("blocked_previous_path") is not None)
                timestamp = record.header.monotonic_ns
                if blocked and run_start is None:
                    run_start = timestamp
                elif not blocked and run_start is not None:
                    longest = max(longest, max(0.0, (timestamp - run_start) / 1e6))
                    run_start = None
            if run_start is not None and selected:
                longest = max(longest, max(0.0,
                    (selected[-1][0].header.monotonic_ns - run_start) / 1e6))
            result["longest_confirmed_route_blocked_ms"] = longest
            if longest > BENCH_PERSISTENCE_LIMIT_MS:
                _fail(result, "confirmed_route_blocked_run_exceeds_2500_ms")
        elif kind == "fixed_target":
            forward, lateral, _material = BENCH_FIXED_TARGETS[annotation["coverage_key"]]
            target = {"forward_m": forward, "lateral_m": lateral}
            raw_samples = []
            filtered_samples = []
            paired_raw = []
            paired_filtered = []
            detections = 0
            for record, scene, _ in selected:
                raw = _nearest(_raw_points(record, estimator.axes), target)
                filtered = _current_confirmed(scene, target, require_map=True)
                if raw is not None:
                    raw_samples.append((raw["forward_m"], raw["lateral_m"]))
                if filtered is not None:
                    sample = (float(filtered["forward_m"]), float(filtered["lateral_m"]))
                    filtered_samples.append(sample)
                    detections += 1
                if raw is not None and filtered is not None:
                    paired_raw.append((raw["forward_m"], raw["lateral_m"]))
                    paired_filtered.append(sample)
            raw_std = _scatter(paired_raw)
            filtered_std = _scatter(paired_filtered)
            result.update({
                "coverage_key": annotation["coverage_key"],
                "raw_target_samples": len(raw_samples),
                "filtered_target_samples": len(filtered_samples),
                "paired_target_samples": len(paired_raw),
                "target_detection_rate": detections / len(selected) if selected else 0.0,
                "raw_position_std_m": raw_std,
                "filtered_position_std_m": filtered_std,
            })
            if len(paired_raw) < BENCH_MIN_POSITION_SAMPLES:
                _fail(result, "paired_target_samples_below_30")
            if raw_std is None or filtered_std is None or filtered_std >= raw_std:
                _fail(result, "filtered_position_std_not_strictly_lower")
        elif kind == "ghost":
            target = annotation["target"]
            result["evidence_seqs"] = list(annotation["evidence_seqs"])
            entries = sum(_nearest(scene.get("probabilistic_local_map", {}).get("cells", []),
                                   target) is not None for _, scene, _ in selected)
            result["probability_map_entries"] = entries
            for seq in annotation["evidence_seqs"]:
                record, _scene_value, _elapsed = indexed[seq]
                if _nearest(_raw_points(record, estimator.axes), target) is None:
                    _fail(result, "missing_raw_target_evidence")
            if entries:
                _fail(result, "ghost_entered_probability_map")
        elif kind == "removal":
            target = annotation["target"]
            last_seq = annotation["last_present_seq"]
            result["last_present_seq"] = last_seq
            first_clear_seq = None
            last_record, last_scene, _ = indexed[last_seq]
            if not (_current_confirmed_evidence(last_scene, target) is not None
                    or _nearest(last_scene.get("probabilistic_local_map", {}).get("cells", []),
                                target) is not None):
                _fail(result, "missing_removal_presence")
            for record, scene, _ in selected:
                if record.header.seq >= last_seq and not _roi_state_present(scene, target):
                    first_clear_seq = record.header.seq
                    break
            persistence = None
            if first_clear_seq is not None:
                persistence = (indexed[first_clear_seq][0].header.monotonic_ns
                               - last_record.header.monotonic_ns) / 1e6
            deadline = last_record.header.monotonic_ns + 2_500_000_000
            first_at_deadline = next(((record, scene) for record, scene, _ in selected
                                      if record.header.monotonic_ns >= deadline), None)
            if first_at_deadline is None or _roi_state_present(first_at_deadline[1], target):
                _fail(result, "target_present_at_first_observation_after_ttl")
            result["first_clear_seq"] = first_clear_seq
            result["ghost_persistence_ms"] = persistence
        elif kind == "transient_path":
            target = annotation["target"]
            event = annotation["event_seq"]
            event_frames = [indexed[event + offset] for offset in range(3)]
            event_index = sorted(indexed).index(event)
            prior_seq = sorted(indexed)[event_index - 1] if event_index else event
            baseline = _route_identity(indexed[prior_seq][1])
            switches = sum(scene["planned_path"]["switch_accepted"] is True
                           for _, scene, _ in event_frames)
            changed = any(_route_changed(baseline, _route_identity(scene))
                          for _, scene, _ in event_frames)
            if _nearest(_raw_points(indexed[event][0], estimator.axes), target) is None:
                _fail(result, "missing_raw_target_evidence")
            if switches:
                _fail(result, "transient_path_switch_accepted")
            if changed:
                _fail(result, "transient_route_identity_changed")
            result.update({"event_seq": event, "path_switches_during_event": switches,
                           "route_identity_changed": changed})
        elif kind == "confirmed_collision":
            target = annotation["target"]
            evidence = annotation["evidence_seqs"]
            confirmation = annotation["confirmation_seq"]
            for seq in evidence:
                if _nearest(_raw_points(indexed[seq][0], estimator.axes), target) is None:
                    _fail(result, "missing_raw_target_evidence")
            if _current_confirmed(indexed[evidence[0]][1], target) is not None:
                _fail(result, "collision_confirmed_before_second_evidence_frame")
            if _current_confirmed(indexed[confirmation][1], target) is None:
                _fail(result, "collision_not_confirmed_on_second_evidence_frame")
            response = next((record.header.seq for record, scene, _ in selected
                             if scene["planned_path"]["switch_accepted"] is True
                             and scene["planned_path"]["stability_status"]
                             == "route_changed_blocked"), None)
            delay = response - confirmation if response is not None else None
            if response != confirmation:
                _fail(result, "collision_response_not_on_confirmation_frame")
            result.update({"confirmation_seq": confirmation, "response_seq": response,
                           "response_delay_frames": delay})
        elif kind == "approach_retreat":
            entry_seq = entry_distance = release_seq = release_distance = None
            previous = None
            for record, scene, _ in selected:
                level = scene["hazard"]["level"]
                distance = scene["hazard"].get("nearest_confirmed_m")
                if previous == "NORMAL" and level == "DANGER" and entry_seq is None:
                    entry_seq, entry_distance = record.header.seq, distance
                elif previous == "DANGER" and level == "NORMAL" and entry_seq is not None:
                    release_seq, release_distance = record.header.seq, distance
                    break
                previous = level
            if entry_seq is None or entry_distance is None or not math.isfinite(float(entry_distance)):
                _fail(result, "missing_danger_entry")
            elif float(entry_distance) > 0.10:
                _fail(result, "danger_entry_above_010_m")
            if release_seq is None or release_distance is None or not math.isfinite(float(release_distance)):
                _fail(result, "missing_normal_release")
            elif float(release_distance) < 0.13:
                _fail(result, "normal_release_below_013_m")
            result.update({"danger_entry_seq": entry_seq,
                           "danger_entry_distance_m": entry_distance,
                           "normal_release_seq": release_seq,
                           "normal_release_distance_m": release_distance})
        windows_report.append(result)
    failures = []
    if not report["state_bounds_passed"]:
        failures.append("state_bounds_failed")
    for status, count in report["calibration_status_counts"].items():
        if status != "ok" and count:
            failures.append(f"calibration_status_not_ok:{status}")
    if report["processing_ms_p95"] > 125.0:
        failures.append("processing_ms_p95_exceeds_125")
    if report["processing_ms_max"] > 125.0:
        failures.append("processing_ms_max_exceeds_125")
    for window in windows_report:
        if not window["passed"]:
            failures.append(f"window_failed:{window['id']}")
    duration = annotations["_duration_s"]
    path_switches = sum(scene["planned_path"]["switch_accepted"] is True
                        for _, scene, _ in ordered_all)
    return {
        "schema_version": 1,
        "annotations_sha256": annotations["_annotations_sha256"],
        "log_sha256": annotations["_log_sha256"],
        "recording_id": annotations["recording"]["id"],
        "recording_kind": annotations["recording"]["kind"],
        "provenance": dict(annotations["provenance"]),
        "duration_s": duration,
        "producer_reported_drops_total": report["producer_reported_drops_total"],
        "path_switches": path_switches,
        "path_switches_per_min": path_switches * 60.0 / duration if duration else 0.0,
        "processing_ms_p95": report["processing_ms_p95"],
        "processing_ms_max": report["processing_ms_max"],
        "passed": not failures,
        "failures": failures,
        "windows": windows_report,
    }


def _positive_integer(value):
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if str(result) != value or result <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return result


def _nonnegative_finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("must be a finite non-negative number") from exc
    if not math.isfinite(result) or result < 0:
        raise argparse.ArgumentTypeError("must be a finite non-negative number")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--clutter-calibration", type=Path)
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--max-frames", type=_positive_integer)
    parser.add_argument("--max-p95-ms", type=_nonnegative_finite)
    parser.add_argument("--max-frame-ms", type=_nonnegative_finite)
    parser.add_argument("--annotations", type=Path)
    parser.add_argument("--require-bench-pass", action="store_true")
    args = parser.parse_args(argv)
    if args.synthetic and args.clutter_calibration is not None:
        parser.error("--synthetic conflicts with --clutter-calibration")
    if not args.synthetic and args.clutter_calibration is None:
        parser.error("--clutter-calibration is required unless --synthetic")
    if args.annotations is not None and args.synthetic:
        parser.error("--annotations conflicts with --synthetic")
    if args.annotations is not None and args.max_frames is not None:
        parser.error("--annotations conflicts with --max-frames")
    if args.require_bench_pass and args.annotations is None:
        parser.error("--require-bench-pass requires --annotations")
    try:
        if args.synthetic:
            estimator = RadarSceneEstimator(RadarAxes(), synthetic=True)
            annotations = None
        else:
            model = load_clutter_model(args.clutter_calibration)
            estimator = RadarSceneEstimator(model.axes, clutter_model=model)
            annotations = (load_bench_annotations(args.annotations, args.input, model)
                           if args.annotations is not None else None)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    report = analyze_records((entry.record for entry in iter_replay(args.input)),
                             estimator, max_frames=args.max_frames,
                             bench_annotations=annotations)
    print(json.dumps(report, sort_keys=True, ensure_ascii=False, allow_nan=False))
    failed = not report["state_bounds_passed"]
    if args.max_p95_ms is not None and report["processing_ms_p95"] > args.max_p95_ms:
        failed = True
    if args.max_frame_ms is not None and report["processing_ms_max"] > args.max_frame_ms:
        failed = True
    if args.require_bench_pass and not report["bench"]["passed"]:
        failed = True
    return 2 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
