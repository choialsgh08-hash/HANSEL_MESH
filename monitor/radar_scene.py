"""Calibrated, robot-relative radar scene evidence."""

from __future__ import annotations

import base64
from collections import deque
import heapq
import math
import statistics
import time
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

from common.radar_geometry import RadarAxes
from common.sensor_contract import RadarFrame
from sensors.radar_calibration import RadarClutterModel
from monitor.radar_local_map import ProbabilisticRollingGrid
from monitor.radar_motion import RadarMotionEstimator
from monitor.radar_path import PathSelectionHysteresis
from monitor.radar_ground import RadarMountGeometry
from monitor.radar_inverse_sensor import first_surface_measurements
from monitor.radar_adc import (
    RadarAdcAngularSurface,
)
from monitor.radar_corridor import ReflectivityCorridorEstimator
from monitor.radar_contour import CalibratedContourEstimator
from monitor.radar_reflectivity import RadarReflectivitySurface
from monitor.radar_tracking import (
    DEFAULT_RANGE_STEP_M,
    POINT_HAZARD_CONFIRM_HITS,
    POINT_HAZARD_WINDOW_FRAMES,
    POINT_HISTORY_FRAMES,
    POINT_POSITION_RESIDUAL_LIMIT_M,
    POINT_STRONG_QUALITY,
    height_band,
    lateral_uncertainty_m,
    point_association_gate,
    point_quality_score,
    resolve_range_step_m,
    robust_point_estimate,
)


SCENE_SCHEMA_VERSION = 1
SCENE_PROJECTION_MODE = "range_azimuth_2d"
GRID_RESOLUTION_M = 0.05
GRID_FORWARD_CELLS = 60
GRID_LATERAL_CELLS = 60
GRID_ORIGIN_FORWARD_CELL = 0
GRID_ORIGIN_LATERAL_CELL = 30
SCENE_MAX_FORWARD_M = 3.0
SCENE_HALF_WIDTH_M = 1.5
PATH_RESOLUTION_M = 0.01
PATH_MAX_LENGTH_M = 3.0
PATH_ROBOT_WIDTH_M = 0.15
PATH_SAFETY_MARGIN_M = 0.03
PATH_GOAL_LATERAL_M = 0.0
PATH_GOAL_WEIGHT = 0.8
PATH_SEARCH_STEP_M = 0.05
PATH_SEARCH_GRID_M = 0.025
PATH_COLLISION_GRID_M = 0.025
PATH_HEADING_BINS = 36
PATH_HEADING_STEP_RAD = 2.0 * math.pi / PATH_HEADING_BINS
PATH_CURVATURES = (
    -2.0 * PATH_HEADING_STEP_RAD / PATH_SEARCH_STEP_M,
    0.0,
    2.0 * PATH_HEADING_STEP_RAD / PATH_SEARCH_STEP_M,
)
PATH_LOCAL_MAX_RANGE_M = 0.30
SPI_REFLECTIVITY_VALID_MIN_RANGE_M = 0.12
REFLECTIVITY_STEER_MIN_RANGE_M = 0.16
REFLECTIVITY_NEAR_STEER_MIN_RANGE_M = 0.12
REFLECTIVITY_NEAR_STEER_MAX_RANGE_M = 0.16
REFLECTIVITY_NEAR_BLOCK_MIN_GAIN = 0.10
# Enough to make a close bend influence the route without allowing one broad
# 29-degree near-field beam to command an unrealistic +/-40 degree turn.
REFLECTIVITY_NEAR_BLOCK_COST_WEIGHT = 0.18
# A side is advisory only when its accumulated reflectivity cost separates
# from the opposite side by more than the measured frame-to-frame ambiguity.
# Below this floor the radar proves a bend, but not which branch is open.
REFLECTIVITY_SIDE_RESOLUTION_MIN_GAP = 0.04
REFLECTIVITY_STEER_MIN_GAIN = 0.055
REFLECTIVITY_STEER_MIN_RELATIVE_GAIN = 0.09
REFLECTIVITY_POINT_STEER_WEIGHT = 0.015
REFLECTIVITY_PROFILE_MEAN_WEIGHT = 0.25
REFLECTIVITY_PROFILE_UPPER_WEIGHT = 0.75
REFLECTIVITY_SCORE_HISTORY_FRAMES = 7
REFLECTIVITY_FORWARD_BLOCK_UPPER = 0.58
REFLECTIVITY_FORWARD_BLOCK_EXCESS = 0.07
EVIDENCE_CONTEXT_RADIUS_M = 0.50
PATH_FOV_AZIMUTH_DEG = 60.0
PATH_PREFERRED_GOAL_MAX_DEG = 55.0
PATH_SEARCH_MAX_ARC_M = 1.20
PATH_SEARCH_MAX_EXPANSIONS = 12_000
PATH_GOAL_FORWARD_M = 0.55
PATH_DYNAMIC_GOAL_RANGE_M = 0.28
PATH_DYNAMIC_GOAL_TOLERANCE_M = 0.04
NEAR_OCCUPANCY_RESOLUTION_M = 0.025
NEAR_OCCUPANCY_TTL_NS = 1_000_000_000
NEAR_POINT_REACTIVE_MIN_CONFIDENCE = 0.85
ADC_NEAR_WALL_MIN_RANGE_M = 0.10
ADC_NEAR_WALL_MAX_RANGE_M = 0.30
ADC_NEAR_WALL_MIN_CONFIDENCE = 0.75
ADC_NEAR_WALL_MIN_COHERENCE = 0.70
ADC_NEAR_WALL_MIN_RELATIVE_DB = 6.0
ADC_NEAR_WALL_MIN_PEAKS = 2
ADC_NEAR_WALL_MAX_AZIMUTH_DELTA_DEG = 20.0
STABLE_OCCUPANCY_RESOLUTION_M = 0.05
STABLE_OCCUPANCY_WALL_LINK_M = 0.18
OBSTACLE_CLUSTER_LINK_M = 0.10
OBSTACLE_CLUSTER_CROSS_SOURCE_LINK_M = 0.075
OBSTACLE_CLUSTER_MAX_AZIMUTH_DELTA_DEG = 25.0
OBSTACLE_CLUSTER_MAX_RANGE_DELTA_M = 0.10
STABLE_OCCUPANCY_MIN_HITS = 2
STABLE_OCCUPANCY_TTL_NS = 5_000_000_000
SHORT_MEMORY_RADIUS_M = 0.50
TRAVEL_TRAIL_RESOLUTION_M = 0.01
ACCUMULATED_OBSTACLE_TTL_S = 5.0
ACCUMULATED_OBSTACLE_TTL_NS = 5_000_000_000
ACCUMULATED_POINT_MIN_CONFIDENCE = 0.70
ACCUMULATED_HEATMAP_MIN_CONFIDENCE = 0.85
ACCUMULATED_HEATMAP_MIN_PEAK_DB = 12.0
ACCUMULATED_OBSTACLE_ASSOCIATION_M = 0.22
TRACK_ASSOCIATION_M = 0.18
TRACK_TTL_S = 0.300
TRACK_TTL_NS = 300_000_000
HEATMAP_ENTER_MIN_RESIDUAL_DB = 8.0
HEATMAP_EXIT_MIN_RESIDUAL_DB = 5.0
HEATMAP_ENTER_MAD_MULTIPLIER = 5.0
HEATMAP_EXIT_MAD_MULTIPLIER = 3.0
HEATMAP_SINGLE_CELL_PEAK_DB = 16.0
HEATMAP_TRACK_ASSOCIATION_M = 0.12
HEATMAP_CONFIRM_HITS = 3
HEATMAP_POINT_CONFIRM_HITS = 2
HEATMAP_HISTORY_FRAMES = 5
HEATMAP_MAX_POSITION_JITTER_M = 0.12
HEATMAP_MAX_CLUSTER_RADIUS_M = 0.15
HEATMAP_MAX_CLUSTERS_PER_FRAME = 64
DANGER_ENTER_M = 0.10
DANGER_EXIT_M = 0.13
FAULT_SOURCE_STATUSES = {
    "waiting",
    "stale",
    "fault",
    "replay_end",
    "replay_loop_restart",
}
TRUSTED_CALIBRATION_STATUSES = {"ok", "synthetic", "spi_raw"}


class RadarHazardEvaluator:
    """Evaluate confirmed point tracks with entry and release hysteresis."""

    def __init__(self) -> None:
        self._danger_track_id: Optional[int] = None

    def reset(self) -> None:
        self._danger_track_id = None

    def update(
        self,
        tracks: List[Dict[str, object]],
        calibration_status: str,
        source_status: str,
        now: float,
    ) -> Dict[str, object]:
        del now
        if source_status in FAULT_SOURCE_STATUSES:
            self.reset()
            return self._result(
                "SENSOR_FAULT",
                None,
                f"source_{source_status}",
            )
        if calibration_status not in TRUSTED_CALIBRATION_STATUSES:
            self.reset()
            return self._result(
                "UNKNOWN",
                None,
                calibration_status,
            )

        def hazard_distance(track: Dict[str, object]) -> float:
            return float(track.get("hazard_distance_m", track["distance_m"]))

        fresh_tracks = [
            track
            for track in tracks
            if int(track.get("age_ms", 0)) < TRACK_TTL_S * 1000
        ]
        point_source_tracks = [
            track
            for track in fresh_tracks
            if track.get("source") == "point"
        ]
        by_id = {
            int(track["track_id"]): track
            for track in point_source_tracks
        }
        if self._danger_track_id is not None:
            latched = by_id.get(self._danger_track_id)
            if (
                latched is not None
                and hazard_distance(latched) < DANGER_EXIT_M
            ):
                return self._result(
                    "DANGER",
                    hazard_distance(latched),
                    "confirmed_point_inside_threshold",
                )
            self._danger_track_id = None

        confirmed = [
            track
            for track in point_source_tracks
            if bool(track["point_confirmed"])
        ]
        entering = [
            track
            for track in confirmed
            if hazard_distance(track) <= DANGER_ENTER_M
        ]
        if entering:
            nearest = min(
                entering,
                key=hazard_distance,
            )
            self._danger_track_id = int(nearest["track_id"])
            return self._result(
                "DANGER",
                hazard_distance(nearest),
                "confirmed_point_inside_threshold",
            )
        if confirmed:
            nearest_m = min(
                hazard_distance(track) for track in confirmed
            )
            return self._result(
                "NORMAL",
                nearest_m,
                "confirmed_points_clear",
            )
        return self._result(
            "UNKNOWN",
            None,
            "insufficient_confirmed_points",
        )

    @staticmethod
    def _result(
        level: str,
        nearest_confirmed_m: Optional[float],
        reason: str,
    ) -> Dict[str, object]:
        return {
            "level": level,
            "nearest_confirmed_m": nearest_confirmed_m,
            "threshold_m": DANGER_ENTER_M,
            "release_m": DANGER_EXIT_M,
            "reason": reason,
        }


class RadarSceneEstimator:
    """Turn canonical radar frames into short-lived robot-relative evidence."""

    def __init__(
        self,
        axes: RadarAxes,
        clutter_model: Optional[RadarClutterModel] = None,
        require_calibration: bool = True,
        clock: Callable[[], float] = time.monotonic,
        synthetic: bool = False,
        mount_geometry: Optional[RadarMountGeometry] = None,
    ) -> None:
        self.axes = axes
        self.clutter_model = clutter_model
        self.require_calibration = require_calibration
        self.synthetic = synthetic
        self.mount_geometry = mount_geometry or RadarMountGeometry()
        self._clock = clock
        self._tracks: List[Dict[str, object]] = []
        self._next_track_id = 1
        if synthetic:
            self._calibration_status = "synthetic"
        elif clutter_model is None:
            self._calibration_status = (
                "calibration_required"
                if require_calibration
                else "calibration_unavailable"
            )
        else:
            self._calibration_status = "ok"
        self._scene_point_count = 0
        self._raw_point_count = 0
        self._clutter_points_rejected = 0
        self._quality_gate_rejected_count = 0
        self._ground_rejected_count = 0
        self._ground_fringe_rejected_count = 0
        self._heatmap_cells_accepted = 0
        self._heatmap_cells_observed = 0
        self._heatmap_candidate_cells = 0
        self._heatmap_cluster_count = 0
        self._heatmap_reflection_count = 0
        self._heatmap_confirmed_count = 0
        self._hazard_evaluator = RadarHazardEvaluator()
        self._producer_id: Optional[str] = None
        self._last_reset_reason: Optional[str] = None
        self._motion_estimator = RadarMotionEstimator()
        self._motion: Dict[str, object] = self._empty_motion("initializing")
        self._accumulated_obstacles: List[Dict[str, object]] = []
        self._next_accumulated_obstacle_id = 1
        self._stable_cells: Dict[Tuple[int, int], Dict[str, object]] = {}
        self._near_cells: Dict[Tuple[int, int], Dict[str, object]] = {}
        self._probability_grid = ProbabilisticRollingGrid(
            radius_m=EVIDENCE_CONTEXT_RADIUS_M,
            resolution_m=0.05,
            occupied_threshold=0.75,
            decay_half_life_s=1.0,
            ttl_s=2.5,
            hit_probability=0.80,
            maximum_probability=0.97,
        )
        self._reflectivity_surface = RadarReflectivitySurface(
            radius_m=EVIDENCE_CONTEXT_RADIUS_M,
            clutter_model=clutter_model,
            # The compact SPI chirp's first complete range-resolution cell is
            # centred at 11.9 cm.  Publishing the zero-filled 10 cm display
            # row let median polish turn blind data into artificial ridges.
            valid_min_range_m=SPI_REFLECTIVITY_VALID_MIN_RANGE_M,
        )
        self._adc_angular_surface = RadarAdcAngularSurface()
        self._reflectivity_corridor = ReflectivityCorridorEstimator()
        self._reflectivity_contours = CalibratedContourEstimator()
        self._scene_epoch = 0
        self._travel_trail: List[Tuple[float, float]] = [(0.0, 0.0)]
        self._path_selector = PathSelectionHysteresis(confirm_frames=3)
        self._opening_score_history: Deque[Dict[float, float]] = deque(
            maxlen=REFLECTIVITY_SCORE_HISTORY_FRAMES - 1,
        )
        self._planned_path = self._neutral_planned_path("initializing")
        self._path_stability_status = "initializing"

    def reset(self, reason: str) -> None:
        self._clear_evidence(reason)
        self._producer_id = None

    def _clear_evidence(self, reason: str) -> None:
        self._scene_epoch += 1
        self._tracks.clear()
        self._next_track_id = 1
        self._hazard_evaluator.reset()
        self._scene_point_count = 0
        self._raw_point_count = 0
        self._clutter_points_rejected = 0
        self._quality_gate_rejected_count = 0
        self._ground_rejected_count = 0
        self._ground_fringe_rejected_count = 0
        self._heatmap_cells_accepted = 0
        self._heatmap_cells_observed = 0
        self._heatmap_candidate_cells = 0
        self._heatmap_cluster_count = 0
        self._heatmap_reflection_count = 0
        self._heatmap_confirmed_count = 0
        self._motion_estimator.reset()
        self._motion = self._empty_motion("reset")
        self._accumulated_obstacles = []
        self._next_accumulated_obstacle_id = 1
        self._stable_cells = {}
        self._near_cells = {}
        self._probability_grid.clear()
        self._reflectivity_surface.reset()
        self._adc_angular_surface.reset()
        self._reflectivity_corridor.reset()
        self._reflectivity_contours.reset()
        self._travel_trail = [(0.0, 0.0)]
        self._path_selector.reset()
        self._opening_score_history.clear()
        self._planned_path = self._neutral_planned_path("reset")
        self._path_stability_status = "reset"
        self._last_reset_reason = reason

    def ingest(
        self,
        frame: RadarFrame,
        received_at: Optional[float] = None,
    ) -> None:
        now = self._clock() if received_at is None else received_at
        if not isinstance(frame, RadarFrame):
            raise TypeError("frame must be a RadarFrame")
        producer_id = frame.header.producer_id
        if (
            self._producer_id is not None
            and producer_id != self._producer_id
        ):
            self.reset("producer_change")
        if frame.frame_transition in {
            "duplicate",
            "reset_or_out_of_order",
        }:
            self.reset(frame.frame_transition)
        self._producer_id = producer_id
        self._raw_point_count = len(frame.points)
        self._scene_point_count = 0
        self._clutter_points_rejected = 0
        self._quality_gate_rejected_count = 0
        self._ground_rejected_count = 0
        self._ground_fringe_rejected_count = 0
        previous_calibration_status = self._calibration_status
        self._calibration_status = self._binding_status(frame)
        heatmap_elapsed_frame_windows = min(
            HEATMAP_HISTORY_FRAMES,
            frame.dropped_frames_since_previous + 1,
        )
        if self._calibration_status not in TRUSTED_CALIBRATION_STATUSES:
            # Clear prior driving state once when trust changes, but retain
            # consecutive full heatmaps while the same untrusted status
            # persists.  Calibration controls safety authorization; it must
            # not make the operator evidence flicker or remain forever new.
            if previous_calibration_status != self._calibration_status:
                calibration_status = self._calibration_status
                self._clear_evidence(calibration_status)
                self._raw_point_count = len(frame.points)
                self._calibration_status = calibration_status
            if (
                frame.heatmap is not None
                and self.axes.supports_native_heatmap()
            ):
                self._reflectivity_surface.ingest_heatmap(
                    frame.heatmap,
                    now,
                    lateral_sign=self.axes.lateral_sign,
                    elapsed_frame_windows=heatmap_elapsed_frame_windows,
                )
                self._heatmap_cells_observed = len(
                    self._reflectivity_surface.payload(now)["cells"]
                )
                if frame.adc_snapshot is not None:
                    self._adc_angular_surface.ingest(
                        frame.adc_snapshot,
                        range_step_m=frame.heatmap.range_step_m,
                        received_at=now,
                        elapsed_frame_windows=heatmap_elapsed_frame_windows,
                        lateral_sign=self.axes.lateral_sign,
                    )
            return
        self._expire_tracks(now)
        heatmap_range_step_m = (
            frame.heatmap.range_step_m if frame.heatmap is not None else None
        )
        calibration_profile_matches = bool(
            self.clutter_model is not None
            and frame.profile_id == self.clutter_model.profile_id
        )
        calibrated_range_step_m = (
            self.clutter_model.range_step_m
            if self.clutter_model is not None
            else None
        )
        range_step_m = resolve_range_step_m(
            heatmap_range_step_m=heatmap_range_step_m,
            calibrated_range_step_m=calibrated_range_step_m,
            calibration_profile_matches=calibration_profile_matches,
            frame_profile_id=frame.profile_id,
            synthetic=self.synthetic,
        )
        accepted_points: List[Tuple[object, float, float, float, float]] = []
        ground_positions: List[Tuple[float, float]] = []
        rejected = 0
        for point in frame.points:
            forward_m, lateral_m = self.axes.map_point(point)
            if not self._in_scene(forward_m, lateral_m):
                continue
            calibrated_clutter = bool(
                self.clutter_model is not None
                and self._calibration_status == "ok"
                and self.clutter_model.matches_point(
                    forward_m,
                    lateral_m,
                    point.z_m,
                )
            )
            quality_score = point_quality_score(point.snr_db, point.noise_db)
            ground_classification = self.mount_geometry.ground_classification(
                forward_m,
                lateral_m,
                point.z_m,
            )
            if ground_classification is not None:
                ground_positions.append((forward_m, lateral_m))
                if ground_classification == "ground":
                    self._ground_rejected_count += 1
                else:
                    self._ground_fringe_rejected_count += 1
                continue
            if calibrated_clutter:
                rejected += 1
                continue
            if range_step_m is None:
                self._quality_gate_rejected_count += 1
                continue
            accepted_points.append(
                (point, forward_m, lateral_m, range_step_m, quality_score)
            )

        now_ns = self._seconds_to_ns(now)
        prior_anchors = [
            track
            for track in self._tracks
            if track.get("source") == "point"
            and track.get("map_confirmed") is True
            and float(track.get("quality_score", 0.0)) >= POINT_STRONG_QUALITY
            and now_ns - int(track["_received_at_ns"]) < TRACK_TTL_NS
        ]
        association_candidates = []
        for anchor_index, anchor in enumerate(prior_anchors):
            for point_index, accepted_point in enumerate(accepted_points):
                _, forward_m, lateral_m, range_step_m, quality_score = (
                    accepted_point
                )
                if quality_score < POINT_STRONG_QUALITY:
                    continue
                if not point_association_gate(
                    float(anchor["forward_m"]),
                    float(anchor["lateral_m"]),
                    forward_m,
                    lateral_m,
                    range_step_m,
                ):
                    continue
                association_candidates.append((
                    math.hypot(
                        float(anchor["forward_m"]) - forward_m,
                        float(anchor["lateral_m"]) - lateral_m,
                    ),
                    anchor_index,
                    point_index,
                ))
        used_anchors = set()
        used_points = set()
        correspondences = []
        for _, anchor_index, point_index in sorted(association_candidates):
            if anchor_index in used_anchors or point_index in used_points:
                continue
            used_anchors.add(anchor_index)
            used_points.add(point_index)
            anchor = prior_anchors[anchor_index]
            accepted_point = accepted_points[point_index]
            correspondences.append((
                (
                    float(anchor["forward_m"]),
                    float(anchor["lateral_m"]),
                ),
                (float(accepted_point[1]), float(accepted_point[2])),
            ))
        self._motion = self._motion_estimator.update(
            correspondences,
            len(prior_anchors),
        )
        if bool(self._motion["applied"]):
            transform_forward_m = float(self._motion["transform_forward_m"])
            transform_lateral_m = float(self._motion["transform_lateral_m"])
            transform_yaw_rad = math.radians(
                float(self._motion["transform_yaw_deg"])
            )
            self._transform_tracks(
                transform_forward_m,
                transform_lateral_m,
                transform_yaw_rad,
            )
            self._probability_grid.transform(
                transform_forward_m,
                transform_lateral_m,
                transform_yaw_rad,
                now,
            )

        point_elapsed_frame_windows = min(
            POINT_HISTORY_FRAMES,
            frame.dropped_frames_since_previous + 1,
        )
        for track in self._tracks:
            hits = track["_point_hits"]
            assert isinstance(hits, list)
            hits.extend([False] * point_elapsed_frame_windows)
            del hits[:-POINT_HAZARD_WINDOW_FRAMES]
            track["point_confirmed"] = (
                sum(hits[-POINT_HAZARD_WINDOW_FRAMES:])
                >= POINT_HAZARD_CONFIRM_HITS
            )
            strong_hits = track.get("_point_strong_hits")
            if isinstance(strong_hits, list):
                strong_hits.extend([False] * point_elapsed_frame_windows)
                del strong_hits[:-POINT_HISTORY_FRAMES]
                track["map_confirmed"] = (
                    sum(strong_hits[-POINT_HISTORY_FRAMES:]) >= 3
                )
            track["_updated_by_point"] = False
            if track["source"] == "point":
                track["classification"] = (
                    "confirmed_obstacle"
                    if track["point_confirmed"]
                    else "point_candidate"
                )
            heatmap_hits = track.get("_heatmap_hits")
            if isinstance(heatmap_hits, list):
                heatmap_hits.extend([False] * heatmap_elapsed_frame_windows)
                del heatmap_hits[:-HEATMAP_HISTORY_FRAMES]
            point_association_hits = track.get("_point_association_hits")
            if isinstance(point_association_hits, list):
                point_association_hits.extend(
                    [False] * heatmap_elapsed_frame_windows
                )
                del point_association_hits[:-HEATMAP_HISTORY_FRAMES]
            track["_updated_by_heatmap"] = False
        point_count = 0
        accepted_point_positions: List[Tuple[float, float]] = []

        for point, forward_m, lateral_m, point_range_step_m, quality_score in accepted_points:
            self._upsert_track(
                forward_m=forward_m,
                lateral_m=lateral_m,
                height_m=point.z_m,
                range_uncertainty_m=point_range_step_m / 2.0,
                lateral_uncertainty_m=lateral_uncertainty_m(
                    math.hypot(forward_m, lateral_m)
                ),
                confidence=quality_score,
                source="point",
                received_at=now,
                snr_db=point.snr_db,
                noise_db=point.noise_db,
                quality_score=quality_score,
            )
            accepted_point_positions.append((forward_m, lateral_m))
            point_count += 1

        corridor_point_observations = [
            {
                "forward_m": forward_m,
                "lateral_m": lateral_m,
                "intensity": quality_score,
            }
            for _, forward_m, lateral_m, _, quality_score in accepted_points
            if math.hypot(forward_m, lateral_m) <= SHORT_MEMORY_RADIUS_M
        ]

        if (
            frame.heatmap is not None
            and self.axes.supports_native_heatmap()
        ):
            self._reflectivity_surface.ingest_heatmap(
                frame.heatmap,
                now,
                lateral_sign=self.axes.lateral_sign,
                elapsed_frame_windows=heatmap_elapsed_frame_windows,
            )
        if (
            frame.adc_snapshot is not None
            and frame.heatmap is not None
            and self.axes.supports_native_heatmap()
        ):
            self._adc_angular_surface.ingest(
                frame.adc_snapshot,
                range_step_m=frame.heatmap.range_step_m,
                received_at=now,
                elapsed_frame_windows=heatmap_elapsed_frame_windows,
                lateral_sign=self.axes.lateral_sign,
            )
        reflectivity_surface = self._reflectivity_surface.payload(now)
        adc_range_angle_surface = self._adc_angular_surface.payload(now)
        self._heatmap_cells_observed = len(reflectivity_surface["cells"])
        if (
            frame.heatmap is not None
            and self.clutter_model is not None
            and self._calibration_status == "ok"
            and self.axes.supports_native_heatmap()
        ):
            (
                clusters,
                candidate_cell_count,
                _,
            ) = self._heatmap_clusters(frame)
            point_supported_clusters = set()
            for point_forward, point_lateral in accepted_point_positions:
                nearest_cluster_index = min(
                    range(len(clusters)),
                    key=lambda index: math.hypot(
                        point_forward - float(clusters[index]["forward_m"]),
                        point_lateral - float(clusters[index]["lateral_m"]),
                    ),
                    default=None,
                )
                if nearest_cluster_index is None:
                    continue
                cluster = clusters[nearest_cluster_index]
                if self._heatmap_point_matches(
                    float(cluster["forward_m"]),
                    float(cluster["lateral_m"]),
                    point_forward,
                    point_lateral,
                    frame.heatmap.range_step_m,
                ):
                    point_supported_clusters.add(nearest_cluster_index)
            for cluster_index, cluster in enumerate(clusters):
                forward_m = float(cluster["forward_m"])
                lateral_m = float(cluster["lateral_m"])
                point_correlated = (
                    cluster_index in point_supported_clusters
                )
                self._upsert_track(
                    forward_m=forward_m,
                    lateral_m=lateral_m,
                    height_m=None,
                    range_uncertainty_m=float(
                        cluster["range_uncertainty_m"]
                    ),
                    confidence=float(cluster["strength_confidence"]),
                    source="heatmap",
                    received_at=now,
                    lateral_uncertainty_m=lateral_uncertainty_m(
                        math.hypot(forward_m, lateral_m)
                    ),
                    point_correlated=point_correlated,
                    cluster_cells=int(cluster["cell_count"]),
                    peak_residual_db=float(cluster["peak_residual_db"]),
                )
            self._heatmap_candidate_cells = candidate_cell_count
            self._heatmap_cluster_count = len(clusters)
        else:
            self._heatmap_candidate_cells = 0
            self._heatmap_cluster_count = 0

        # Retain the complete heatmap as a soft operator surface, but fit
        # geometry only to its range ridge.  Feeding every measured cell into
        # a line fitter makes the rectangular sampling lattice look like
        # walls even when no reflector occupies those cells.
        surface_ridge = (
            self._surface_contour_observations(
                reflectivity_surface["cells"]
            )
            if self._calibration_status == "spi_raw"
            else []
        )
        if self._calibration_status == "spi_raw":
            # Full-frame SPI mode guides on the complete reflectivity trend.
            # Fitting short bright fragments into walls made a scene-specific
            # geometric hypothesis look more certain than the measurements.
            self._reflectivity_corridor.reset()
            self._reflectivity_contours.reset()
        else:
            self._reflectivity_corridor.ingest(
                surface_ridge,
                elapsed_frame_windows=heatmap_elapsed_frame_windows,
                point_observations=corridor_point_observations,
            )
            self._reflectivity_contours.ingest(
                surface_ridge,
                elapsed_frame_windows=heatmap_elapsed_frame_windows,
            )

        self._update_probability_map(
            now,
            reflectivity_surface["cells"],
            ground_positions,
            adc_range_angle_surface,
        )
        self._scene_point_count = point_count
        self._clutter_points_rejected = rejected
        self._heatmap_cells_accepted = self._heatmap_candidate_cells
        heatmap_tracks = [
            track for track in self._tracks if track["source"] == "heatmap"
        ]
        self._heatmap_confirmed_count = sum(
            track.get("classification") == "confirmed_obstacle"
            for track in heatmap_tracks
        )
        self._heatmap_reflection_count = (
            len(heatmap_tracks) - self._heatmap_confirmed_count
        )
        self._update_accumulated_obstacles(now)
        self._update_near_occupancy(now)
        if self._motion["status"] not in {"unstable", "low_confidence"}:
            self._update_stable_occupancy(now)
        self._update_stable_path(now)

    def snapshot(
        self,
        source_status: str = "live",
        now: Optional[float] = None,
    ) -> Dict[str, object]:
        observed_at = self._clock() if now is None else now
        observed_at_ns = self._seconds_to_ns(observed_at)
        if source_status in FAULT_SOURCE_STATUSES:
            self.reset(source_status)
        self._expire_tracks(observed_at)
        self._expire_accumulated_obstacles(observed_at)
        self._expire_stable_cells(observed_at)
        self._expire_near_cells(observed_at)
        tracks = [
            {
                key: value
                for key, value in track.items()
                if not key.startswith("_")
            }
            | {
                "age_ms": max(
                    0,
                    (
                        observed_at_ns
                        - int(track["_received_at_ns"])
                    )
                    // 1_000_000,
                )
            }
            for track in self._tracks
        ]
        obstacle_tracks = [
            track
            for track in tracks
            if track.get("classification") == "confirmed_obstacle"
        ]
        reference_tracks = [
            track
            for track in tracks
            if track.get("classification") != "confirmed_obstacle"
        ]
        accumulated_obstacles = [
            {
                key: value
                for key, value in obstacle.items()
                if not key.startswith("_")
            }
            | {
                "age_ms": max(
                    0,
                    (
                        observed_at_ns
                        - int(obstacle["_last_seen_ns"])
                    )
                    // 1_000_000,
                ),
                "remaining_ms": max(
                    0,
                    (
                        ACCUMULATED_OBSTACLE_TTL_NS
                        - (
                            observed_at_ns
                            - int(obstacle["_last_seen_ns"])
                        )
                    )
                    // 1_000_000,
                ),
            }
            for obstacle in self._accumulated_obstacles
        ]
        stable_obstacles = self._stable_obstacle_payload(observed_at)
        reflectivity_surface = self._reflectivity_surface.payload(observed_at)
        adc_range_angle_surface = self._adc_angular_surface.payload(observed_at)
        reflectivity_corridor = self._reflectivity_corridor.payload()
        reflectivity_boundaries = (
            self._reflectivity_corridor.boundaries_payload()
        )
        reflectivity_contours = self._reflectivity_contours.payload()
        probability_map = self._probability_grid.payload(observed_at)
        occupied_probability_cells = self._probability_grid.occupied_cells(
            observed_at
        )
        probability_diagnostics = self._probability_grid.diagnostics()
        persistent_near_obstacles = [
            obstacle
            for obstacle in stable_obstacles
            if math.hypot(
                float(obstacle["forward_m"]),
                float(obstacle["lateral_m"]),
            ) <= PATH_LOCAL_MAX_RANGE_M
        ]
        near_obstacles = self._near_obstacle_payload(observed_at)
        planned_path = dict(self._planned_path)
        grid = bytearray(GRID_FORWARD_CELLS * GRID_LATERAL_CELLS)
        # Keep newly confirmed returns visible immediately while retaining the
        # multi-frame stable cells that the path planner trusts.  Duplicate
        # cells are harmless because the grid stores the maximum confidence.
        for track in [
            *obstacle_tracks,
            *near_obstacles,
            *occupied_probability_cells,
        ]:
            forward_cell = min(
                GRID_FORWARD_CELLS - 1,
                int(float(track["forward_m"]) / GRID_RESOLUTION_M),
            )
            lateral_cell = (
                math.floor(
                    float(track["lateral_m"]) / GRID_RESOLUTION_M
                )
                + GRID_ORIGIN_LATERAL_CELL
            )
            if (
                0 <= forward_cell < GRID_FORWARD_CELLS
                and 0 <= lateral_cell < GRID_LATERAL_CELLS
            ):
                index = (
                    forward_cell * GRID_LATERAL_CELLS + lateral_cell
                )
                grid[index] = max(
                    grid[index],
                    max(
                        1,
                        min(
                            255,
                            round(float(track["confidence"]) * 255.0),
                        ),
                    ),
                )
        return {
            "schema_version": SCENE_SCHEMA_VERSION,
            "pose_mode": "robot_relative_short_memory",
            "projection_mode": SCENE_PROJECTION_MODE,
            "elevation_used": False,
            "calibration_status": self._calibration_status,
            "ground_filter": self.mount_geometry.payload(),
            "grid": {
                "resolution_m": GRID_RESOLUTION_M,
                "forward_cells": GRID_FORWARD_CELLS,
                "lateral_cells": GRID_LATERAL_CELLS,
                "origin_forward_cell": GRID_ORIGIN_FORWARD_CELL,
                "origin_lateral_cell": GRID_ORIGIN_LATERAL_CELL,
                "encoding": "occupancy-u8-base64",
                "layout": "forward-major_lateral-minor",
                "unknown_value": 0,
                "data_base64": base64.b64encode(bytes(grid)).decode("ascii"),
            },
            "tracks": tracks,
            "obstacle_tracks": obstacle_tracks,
            "reference_tracks": reference_tracks,
            "accumulated_obstacles": accumulated_obstacles,
            "persistent_near_obstacles": persistent_near_obstacles,
            "reflectivity_surface": reflectivity_surface,
            # The UART last-chirp ADC overlay is optional.  Full-frame SPI
            # mode already supplies its own range/angle surface and never
            # populates this legacy accumulator, so do not serialize a
            # deliberately incomplete placeholder as if it were evidence.
            **(
                {"adc_range_angle_surface": adc_range_angle_surface}
                if adc_range_angle_surface.get("range_step_m") is not None
                else {}
            ),
            "reflectivity_corridor": reflectivity_corridor,
            "reflectivity_boundaries": reflectivity_boundaries,
            "reflectivity_contours": reflectivity_contours,
            "probabilistic_local_map": probability_map,
            "rolling_local_map": {
                "radius_m": 0.50,
                "resolution_m": 0.05,
                "minimum_hits": 1,
                "motion_relative": True,
                "retention": "probability_decay_with_ttl",
                "cells": occupied_probability_cells,
            },
            "travel_trail": {
                "radius_m": SHORT_MEMORY_RADIUS_M,
                "resolution_m": TRAVEL_TRAIL_RESOLUTION_M,
                "motion_relative": True,
                "source": "radar_ego_motion",
                "points": self._travel_trail_payload(),
            },
            "persistent_near_obstacle_memory": {
                "window_ms": STABLE_OCCUPANCY_TTL_NS // 1_000_000,
                "point_min_confidence": ACCUMULATED_POINT_MIN_CONFIDENCE,
                "heatmap_min_confidence": (
                    ACCUMULATED_HEATMAP_MIN_CONFIDENCE
                ),
                "heatmap_min_peak_db": ACCUMULATED_HEATMAP_MIN_PEAK_DB,
                "motion_relative": True,
            },
            "obstacle_memory_window_ms": round(
                ACCUMULATED_OBSTACLE_TTL_S * 1000
            ),
            "stable_occupancy": {
                "resolution_m": STABLE_OCCUPANCY_RESOLUTION_M,
                "minimum_hits": STABLE_OCCUPANCY_MIN_HITS,
                "window_ms": STABLE_OCCUPANCY_TTL_NS // 1_000_000,
                "cells": stable_obstacles,
            },
            "near_occupancy": {
                "resolution_m": NEAR_OCCUPANCY_RESOLUTION_M,
                "window_ms": NEAR_OCCUPANCY_TTL_NS // 1_000_000,
                "cells": near_obstacles,
            },
            "motion_compensation": dict(self._motion),
            "planned_path": planned_path,
            "hazard": self._hazard_evaluator.update(
                tracks,
                self._calibration_status,
                source_status,
                observed_at,
            ),
            "diagnostics": {
                "raw_point_count": self._raw_point_count,
                "scene_point_count": self._scene_point_count,
                "clutter_points_rejected": self._clutter_points_rejected,
                "self_clutter_rejected_count": (
                    self._clutter_points_rejected
                ),
                "quality_gate_rejected_count": (
                    self._quality_gate_rejected_count
                ),
                "ground_rejected_count": self._ground_rejected_count,
                "ground_fringe_rejected_count": (
                    self._ground_fringe_rejected_count
                ),
                "reflectivity_surface_cell_count": len(
                    reflectivity_surface["cells"]
                ),
                "heatmap_cells_accepted": self._heatmap_cells_accepted,
                "heatmap_cells_observed": self._heatmap_cells_observed,
                "heatmap_candidate_cells": self._heatmap_candidate_cells,
                "heatmap_cluster_count": self._heatmap_cluster_count,
                "heatmap_confirmed_obstacles": (
                    self._heatmap_confirmed_count
                ),
                "heatmap_reflection_evidence": (
                    self._heatmap_reflection_count
                ),
                "tentative_point_track_count": sum(
                    track.get("source") == "point"
                    and not bool(track.get("point_confirmed"))
                    for track in tracks
                ),
                "confirmed_point_track_count": sum(
                    track.get("source") == "point"
                    and bool(track.get("point_confirmed"))
                    for track in tracks
                ),
                "heatmap_reflection_count": self._heatmap_reflection_count,
                "confirmed_heatmap_track_count": (
                    self._heatmap_confirmed_count
                ),
                "probability_active_cell_count": probability_diagnostics[
                    "active_cell_count"
                ],
                "probability_occupied_cell_count": probability_diagnostics[
                    "occupied_cell_count"
                ],
                "probability_decayed_cell_count": probability_diagnostics[
                    "decayed_cell_count"
                ],
                "probability_ttl_expired_cell_count": (
                    probability_diagnostics["ttl_expired_cell_count"]
                ),
                "occupancy_track_count": len(obstacle_tracks),
                "accumulated_obstacle_count": len(accumulated_obstacles),
                "stable_occupancy_cell_count": len(stable_obstacles),
                "near_occupancy_cell_count": len(near_obstacles),
                "persistent_near_obstacle_count": len(
                    persistent_near_obstacles
                ),
                "rolling_local_map_cell_count": len(
                    occupied_probability_cells
                ),
                "travel_trail_point_count": len(
                    self._travel_trail_payload()
                ),
                "motion_status": self._motion["status"],
                "motion_matches": self._motion["matched_points"],
                "path_length_m": planned_path["length_m"],
                "path_stability_status": self._path_stability_status,
                "path_pending_frames": planned_path[
                    "pending_switch_frames"
                ],
                **self._path_selector.diagnostics(),
                "scene_epoch": self._scene_epoch,
                "last_reset_reason": self._last_reset_reason,
            },
        }

    @staticmethod
    def _empty_motion(status: str) -> Dict[str, object]:
        return {
            "status": status,
            "applied": False,
            "confidence": 0.0,
            "matched_points": 0,
            "match_ratio": 0.0,
            "rmse_m": None,
            "ego_forward_m": 0.0,
            "ego_lateral_m": 0.0,
            "ego_yaw_deg": 0.0,
            "transform_forward_m": 0.0,
            "transform_lateral_m": 0.0,
            "transform_yaw_deg": 0.0,
        }

    def _transform_tracks(
        self,
        delta_forward: float,
        delta_lateral: float,
        yaw_rad: float,
    ) -> None:
        cosine = math.cos(yaw_rad)
        sine = math.sin(yaw_rad)

        def transform(position: Tuple[float, float]) -> Tuple[float, float]:
            forward, lateral = position
            return (
                cosine * forward - sine * lateral + delta_forward,
                sine * forward + cosine * lateral + delta_lateral,
            )

        retained: List[Dict[str, object]] = []
        for track in self._tracks:
            forward, lateral = transform(
                (float(track["forward_m"]), float(track["lateral_m"]))
            )
            if not self._in_scene(forward, lateral):
                continue
            track["forward_m"] = forward
            track["lateral_m"] = lateral
            track["distance_m"] = math.hypot(forward, lateral)
            if track.get("_hazard_forward_m") is not None:
                hazard_forward, hazard_lateral = transform(
                    (
                        float(track["_hazard_forward_m"]),
                        float(track["_hazard_lateral_m"]),
                    )
                )
                track["_hazard_forward_m"] = hazard_forward
                track["_hazard_lateral_m"] = hazard_lateral
                track["hazard_distance_m"] = math.hypot(
                    hazard_forward, hazard_lateral
                )
            history = track.get("_position_history")
            if isinstance(history, list):
                track["_position_history"] = [
                    transform((float(point[0]), float(point[1])))
                    for point in history
                ]
            point_samples = track.get("_point_samples")
            if isinstance(point_samples, list):
                track["_point_samples"] = [
                    (*transform((float(point[0]), float(point[1]))), point[2])
                    for point in point_samples
                ]
            retained.append(track)
        self._tracks = retained

        retained_obstacles: List[Dict[str, object]] = []
        for obstacle in self._accumulated_obstacles:
            forward, lateral = transform(
                (
                    float(obstacle["forward_m"]),
                    float(obstacle["lateral_m"]),
                )
            )
            if not self._in_scene(forward, lateral):
                continue
            obstacle["forward_m"] = forward
            obstacle["lateral_m"] = lateral
            obstacle["distance_m"] = math.hypot(forward, lateral)
            retained_obstacles.append(obstacle)
        self._accumulated_obstacles = retained_obstacles

        transformed_cells: Dict[Tuple[int, int], Dict[str, object]] = {}
        for cell in self._stable_cells.values():
            forward, lateral = transform(
                (float(cell["forward_m"]), float(cell["lateral_m"]))
            )
            if not self._in_scene(forward, lateral):
                continue
            key = (
                round(forward / STABLE_OCCUPANCY_RESOLUTION_M),
                round(lateral / STABLE_OCCUPANCY_RESOLUTION_M),
            )
            existing = transformed_cells.get(key)
            transformed = {
                **cell,
                "forward_m": key[0] * STABLE_OCCUPANCY_RESOLUTION_M,
                "lateral_m": key[1] * STABLE_OCCUPANCY_RESOLUTION_M,
            }
            if existing is None:
                transformed_cells[key] = transformed
            else:
                existing["hits"] = min(
                    255, int(existing["hits"]) + int(cell["hits"])
                )
                existing["confidence"] = max(
                    float(existing["confidence"]),
                    float(cell["confidence"]),
                )
                existing["_last_seen_ns"] = max(
                    int(existing["_last_seen_ns"]),
                    int(cell["_last_seen_ns"]),
                )
        self._stable_cells = transformed_cells

        transformed_near_cells: Dict[Tuple[int, int], Dict[str, object]] = {}
        for cell in self._near_cells.values():
            forward, lateral = transform(
                (float(cell["forward_m"]), float(cell["lateral_m"]))
            )
            distance = math.hypot(forward, lateral)
            if distance > PATH_LOCAL_MAX_RANGE_M or not self._in_scene(
                forward, lateral
            ):
                continue
            key = (
                round(forward / NEAR_OCCUPANCY_RESOLUTION_M),
                round(lateral / NEAR_OCCUPANCY_RESOLUTION_M),
            )
            existing = transformed_near_cells.get(key)
            transformed = {
                **cell,
                "forward_m": key[0] * NEAR_OCCUPANCY_RESOLUTION_M,
                "lateral_m": key[1] * NEAR_OCCUPANCY_RESOLUTION_M,
            }
            if existing is None:
                transformed_near_cells[key] = transformed
                continue
            existing["confidence"] = max(
                float(existing["confidence"]),
                float(transformed["confidence"]),
            )
            existing["range_uncertainty_m"] = max(
                float(existing.get("range_uncertainty_m", 0.0)),
                float(transformed.get("range_uncertainty_m", 0.0)),
            )
            existing["lateral_uncertainty_m"] = max(
                float(existing.get("lateral_uncertainty_m", 0.0)),
                float(transformed.get("lateral_uncertainty_m", 0.0)),
            )
            existing["hits"] = min(
                255,
                max(int(existing["hits"]), int(transformed["hits"])),
            )
            existing["_last_seen_ns"] = max(
                int(existing["_last_seen_ns"]),
                int(transformed["_last_seen_ns"]),
            )
            if existing["source"] != transformed["source"]:
                existing["source"] = "mixed"
        self._near_cells = transformed_near_cells

        transformed_trail = [
            transformed
            for point in self._travel_trail
            for transformed in [transform(point)]
            if math.hypot(transformed[0], transformed[1])
            <= SHORT_MEMORY_RADIUS_M
        ]
        if not transformed_trail:
            transformed_trail = [(0.0, 0.0)]
        if math.hypot(
            transformed_trail[-1][0], transformed_trail[-1][1]
        ) >= TRAVEL_TRAIL_RESOLUTION_M:
            transformed_trail.append((0.0, 0.0))
        self._travel_trail = transformed_trail

    def _update_accumulated_obstacles(self, received_at: float) -> None:
        self._expire_accumulated_obstacles(received_at)
        for track in self._tracks:
            if track.get("classification") != "confirmed_obstacle":
                continue
            source = str(track["source"])
            updated = (
                bool(track.get("_updated_by_point"))
                if source == "point"
                else bool(track.get("_updated_by_heatmap"))
            )
            if not updated:
                continue
            confidence = float(track["confidence"])
            if source == "point":
                if confidence < ACCUMULATED_POINT_MIN_CONFIDENCE:
                    continue
            elif (
                confidence < ACCUMULATED_HEATMAP_MIN_CONFIDENCE
                or float(track.get("peak_residual_db", 0.0))
                < ACCUMULATED_HEATMAP_MIN_PEAK_DB
            ):
                continue
            forward = float(track["forward_m"])
            lateral = float(track["lateral_m"])
            nearest = min(
                self._accumulated_obstacles,
                key=lambda obstacle: math.hypot(
                    float(obstacle["forward_m"]) - forward,
                    float(obstacle["lateral_m"]) - lateral,
                ),
                default=None,
            )
            if (
                nearest is not None
                and math.hypot(
                    float(nearest["forward_m"]) - forward,
                    float(nearest["lateral_m"]) - lateral,
                ) <= ACCUMULATED_OBSTACLE_ASSOCIATION_M
            ):
                smoothing = 0.30
                nearest["forward_m"] = (
                    float(nearest["forward_m"]) * (1.0 - smoothing)
                    + forward * smoothing
                )
                nearest["lateral_m"] = (
                    float(nearest["lateral_m"]) * (1.0 - smoothing)
                    + lateral * smoothing
                )
                nearest["distance_m"] = math.hypot(
                    float(nearest["forward_m"]),
                    float(nearest["lateral_m"]),
                )
                nearest["confidence"] = max(
                    float(nearest["confidence"]), confidence
                )
                nearest["range_uncertainty_m"] = max(
                    float(nearest["range_uncertainty_m"]),
                    float(track.get("range_uncertainty_m", 0.0)),
                )
                nearest["hits"] = int(nearest["hits"]) + 1
                nearest["source"] = source
                nearest["_last_seen_ns"] = self._seconds_to_ns(received_at)
                continue
            self._accumulated_obstacles.append(
                {
                    "obstacle_id": self._next_accumulated_obstacle_id,
                    "forward_m": forward,
                    "lateral_m": lateral,
                    "distance_m": math.hypot(forward, lateral),
                    "range_uncertainty_m": float(
                        track.get("range_uncertainty_m", 0.0)
                    ),
                    "confidence": confidence,
                    "source": source,
                    "hits": 1,
                    "_last_seen_ns": self._seconds_to_ns(received_at),
                }
            )
            self._next_accumulated_obstacle_id += 1

    def _expire_accumulated_obstacles(self, now: float) -> None:
        now_ns = self._seconds_to_ns(now)
        self._accumulated_obstacles = [
            obstacle
            for obstacle in self._accumulated_obstacles
            if now_ns - int(obstacle["_last_seen_ns"])
            < ACCUMULATED_OBSTACLE_TTL_NS
        ]

    def _strong_updated_tracks(
        self,
        *,
        require_point_map_confirmation: bool = True,
        include_heatmap: bool = True,
    ) -> List[Dict[str, object]]:
        strong: List[Dict[str, object]] = []
        for track in self._tracks:
            if track.get("classification") != "confirmed_obstacle":
                continue
            source = str(track["source"])
            updated = (
                bool(track.get("_updated_by_point"))
                if source == "point"
                else bool(track.get("_updated_by_heatmap"))
            )
            if not updated:
                continue
            confidence = float(track["confidence"])
            if source == "point":
                if (
                    (
                        require_point_map_confirmation
                        and track.get("map_confirmed") is not True
                    )
                    or confidence < ACCUMULATED_POINT_MIN_CONFIDENCE
                ):
                    continue
            elif (
                not include_heatmap
                or
                confidence < ACCUMULATED_HEATMAP_MIN_CONFIDENCE
                or float(track.get("peak_residual_db", 0.0))
                < ACCUMULATED_HEATMAP_MIN_PEAK_DB
            ):
                continue
            strong.append(track)
        return strong

    def _update_stable_occupancy(self, received_at: float) -> None:
        self._expire_stable_cells(received_at)
        received_at_ns = self._seconds_to_ns(received_at)
        for track in self._strong_updated_tracks(
            require_point_map_confirmation=False
        ):
            forward = float(track["forward_m"])
            lateral = float(track["lateral_m"])
            key = (
                round(forward / STABLE_OCCUPANCY_RESOLUTION_M),
                round(lateral / STABLE_OCCUPANCY_RESOLUTION_M),
            )
            cell = self._stable_cells.get(key)
            if cell is None:
                self._stable_cells[key] = {
                    "forward_m": key[0] * STABLE_OCCUPANCY_RESOLUTION_M,
                    "lateral_m": key[1] * STABLE_OCCUPANCY_RESOLUTION_M,
                    "range_uncertainty_m": 0.0,
                    "confidence": float(track["confidence"]),
                    "source": str(track["source"]),
                    "hits": 1,
                    "_last_seen_ns": received_at_ns,
                }
                continue
            cell["hits"] = min(255, int(cell["hits"]) + 1)
            cell["confidence"] = max(
                float(cell["confidence"]), float(track["confidence"])
            )
            cell["source"] = str(track["source"])
            cell["_last_seen_ns"] = received_at_ns

    def _update_probability_map(
        self,
        received_at: float,
        reflectivity_cells: Sequence[Dict[str, object]] = (),
        ground_positions: Sequence[Tuple[float, float]] = (),
        adc_range_angle_surface: Optional[Dict[str, object]] = None,
    ) -> None:
        # Heatmap tracks come from the old CFAR/clustering compatibility
        # path.  They must not bypass the range-ordered inverse model and add
        # a stronger multipath lobe as a hard map hit.  Point tracks remain an
        # independent corroborating sensor source.
        for track in self._strong_updated_tracks(include_heatmap=False):
            self._probability_grid.add_hit(
                float(track["forward_m"]),
                float(track["lateral_m"]),
                float(track["confidence"]),
                str(track["source"]),
                float(track.get("range_uncertainty_m", 0.0)),
                float(track.get("lateral_uncertainty_m", 0.0)),
                received_at,
            )

        measurements: List[Dict[str, object]] = []
        for surface in first_surface_measurements(
            reflectivity_cells,
            minimum_range_m=self._reflectivity_surface.valid_min_range_m,
            maximum_range_m=EVIDENCE_CONTEXT_RADIUS_M,
            corroborating_peaks=(
                adc_range_angle_surface.get("peaks", ())
                if isinstance(adc_range_angle_surface, dict)
                else ()
            ),
            exclude_near_boundary=self._calibration_status == "spi_raw",
        ):
            forward_m = float(surface["forward_m"])
            lateral_m = float(surface["lateral_m"])
            distance_m = float(surface["range_m"])
            range_uncertainty_m = float(surface["range_uncertainty_m"])
            lateral_uncertainty_m = max(
                GRID_RESOLUTION_M / 2.0,
                distance_m * math.sin(math.radians(
                    float(surface["angular_uncertainty_deg"])
                )),
            )
            if self.clutter_model is not None and any(
                math.hypot(
                    forward_m - cluster.forward_m,
                    lateral_m - cluster.lateral_m,
                ) <= cluster.radius_m + max(
                    range_uncertainty_m,
                    lateral_uncertainty_m,
                ) + GRID_RESOLUTION_M / math.sqrt(2.0)
                for cluster in self.clutter_model.point_clusters
            ):
                continue
            if any(
                math.hypot(
                    forward_m - ground_forward,
                    lateral_m - ground_lateral,
                ) <= max(
                    GRID_RESOLUTION_M,
                    range_uncertainty_m * 2.0,
                    lateral_uncertainty_m + GRID_RESOLUTION_M / 2.0,
                )
                for ground_forward, ground_lateral in ground_positions
            ):
                continue
            measurements.append({
                "forward_m": forward_m,
                "lateral_m": lateral_m,
                "measurement_probability": float(
                    surface["measurement_probability"]
                ),
                "confidence": float(surface["confidence"]),
                "source": "heatmap",
                "range_uncertainty_m": range_uncertainty_m,
                "lateral_uncertainty_m": lateral_uncertainty_m,
            })
        # The UART ADC TLV contains only the final chirp pair from the same
        # frame that produced the range-azimuth heatmap. Its phase surface is
        # display diagnostics, not an independent occupancy observation.
        self._probability_grid.observe_frame(measurements, received_at)

    def _travel_trail_payload(self) -> List[List[float]]:
        points = list(self._travel_trail)
        if not points or math.hypot(points[-1][0], points[-1][1]) > 1e-9:
            points.append((0.0, 0.0))
        return [
            [round(forward, 4), round(lateral, 4)]
            for forward, lateral in points
        ]

    def _update_near_occupancy(self, received_at: float) -> None:
        """Keep recent, confirmed near-field evidence for reactive planning."""
        self._expire_near_cells(received_at)
        received_at_ns = self._seconds_to_ns(received_at)
        for track in self._tracks:
            source = str(track["source"])
            if source == "point":
                confirmed = track.get("point_confirmed") is True
                updated = bool(track.get("_updated_by_point"))
            else:
                continue
            if not confirmed or not updated:
                continue
            confidence = float(track["confidence"])
            if source == "point":
                # A two-frame point is useful visual evidence at 0.50, but
                # reactive collision geometry needs the stronger directional
                # evidence threshold.  This prevents low-directionality SPI
                # coupling lobes from stopping the path at the sensor.
                if confidence < NEAR_POINT_REACTIVE_MIN_CONFIDENCE:
                    continue
            forward = float(track["forward_m"])
            lateral = float(track["lateral_m"])
            distance = math.hypot(forward, lateral)
            if not (
                math.isfinite(forward)
                and math.isfinite(lateral)
                and distance <= PATH_LOCAL_MAX_RANGE_M
            ):
                continue
            if abs(math.degrees(math.atan2(lateral, forward))) > (
                PATH_FOV_AZIMUTH_DEG
            ):
                continue
            key = (
                round(forward / NEAR_OCCUPANCY_RESOLUTION_M),
                round(lateral / NEAR_OCCUPANCY_RESOLUTION_M),
            )
            cell = self._near_cells.get(key)
            if cell is None:
                self._near_cells[key] = {
                    "forward_m": key[0] * NEAR_OCCUPANCY_RESOLUTION_M,
                    "lateral_m": key[1] * NEAR_OCCUPANCY_RESOLUTION_M,
                    "range_uncertainty_m": float(
                        track.get("range_uncertainty_m", 0.0)
                    ),
                    "lateral_uncertainty_m": float(
                        track.get("lateral_uncertainty_m", 0.0)
                    ),
                    "confidence": confidence,
                    "source": source,
                    "hits": 1,
                    "reactive": True,
                    "_last_seen_ns": received_at_ns,
                }
                continue
            cell["hits"] = min(255, int(cell["hits"]) + 1)
            cell["confidence"] = max(float(cell["confidence"]), confidence)
            cell["range_uncertainty_m"] = max(
                float(cell.get("range_uncertainty_m", 0.0)),
                float(track.get("range_uncertainty_m", 0.0)),
            )
            cell["lateral_uncertainty_m"] = max(
                float(cell.get("lateral_uncertainty_m", 0.0)),
                float(track.get("lateral_uncertainty_m", 0.0)),
            )
            if cell["source"] != source:
                cell["source"] = "mixed"
            cell["_last_seen_ns"] = received_at_ns

    def _expire_near_cells(self, now: float) -> None:
        now_ns = self._seconds_to_ns(now)
        self._near_cells = {
            key: cell
            for key, cell in self._near_cells.items()
            if now_ns - int(cell["_last_seen_ns"]) < NEAR_OCCUPANCY_TTL_NS
        }

    def _near_obstacle_payload(self, now: float) -> List[Dict[str, object]]:
        now_ns = self._seconds_to_ns(now)
        return [
            {
                key: value
                for key, value in cell.items()
                if not key.startswith("_")
            }
            | {
                "age_ms": max(
                    0,
                    (now_ns - int(cell["_last_seen_ns"])) // 1_000_000,
                )
            }
            for cell in self._near_cells.values()
        ]

    def _expire_stable_cells(self, now: float) -> None:
        now_ns = self._seconds_to_ns(now)
        self._stable_cells = {
            key: cell
            for key, cell in self._stable_cells.items()
            if now_ns - int(cell["_last_seen_ns"])
            < STABLE_OCCUPANCY_TTL_NS
        }

    def _stable_obstacle_payload(
        self,
        now: float,
    ) -> List[Dict[str, object]]:
        now_ns = self._seconds_to_ns(now)
        return [
            {
                "forward_m": float(cell["forward_m"]),
                "lateral_m": float(cell["lateral_m"]),
                "range_uncertainty_m": 0.0,
                "confidence": float(cell["confidence"]),
                "source": str(cell["source"]),
                "hits": int(cell["hits"]),
                "age_ms": max(
                    0,
                    (now_ns - int(cell["_last_seen_ns"])) // 1_000_000,
                ),
                "remaining_ms": max(
                    0,
                    (
                        STABLE_OCCUPANCY_TTL_NS
                        - (now_ns - int(cell["_last_seen_ns"]))
                    )
                    // 1_000_000,
                ),
            }
            for cell in self._stable_cells.values()
            if int(cell["hits"]) >= STABLE_OCCUPANCY_MIN_HITS
        ]

    def _update_stable_path(self, now: float) -> None:
        obstacles = self._planning_obstacle_payload(now)
        now_ns = self._seconds_to_ns(now)
        confirmed_stop = any(
            track.get("source") == "point"
            and track.get("point_confirmed") is True
            and now_ns - int(track["_received_at_ns"]) < TRACK_TTL_NS
            and float(track.get("hazard_distance_m", track["distance_m"]))
            <= DANGER_ENTER_M
            for track in self._tracks
        )
        if self._calibration_status == "spi_raw":
            reflectivity = self._reflectivity_surface.payload(now)
            candidate = self._plan_reflectivity_opening(
                reflectivity.get("cells", ()),
                obstacles,
                confirmed_stop=confirmed_stop,
                score_history=tuple(self._opening_score_history),
            )
            self._opening_score_history.append({
                float(item["angle_deg"]): float(item["cost"])
                for item in candidate["instant_opening_scores"]
            })
        else:
            candidate = self._plan_path(obstacles)
        motion_factor = {
            "ok": 1.0,
            "stabilizing": 0.85,
            "insufficient_points": 0.70,
            "initializing": 0.70,
            "unstable": 0.60,
            "low_confidence": 0.55,
        }.get(str(self._motion["status"]), 0.65)
        evidence_factor = 0.72 + 0.28 * min(1.0, len(obstacles) / 5.0)
        confidence = (
            float(candidate["geometry_confidence"])
            * motion_factor
            * evidence_factor
        )
        candidate["confidence"] = round(confidence, 3)
        candidate["confidence_level"] = (
            "high" if confidence >= 0.80
            else "medium" if confidence >= 0.55
            else "low"
        )
        if self._calibration_status == "spi_raw":
            blocked_segments = (
                [{
                    "start_index": 0,
                    "end_index": max(
                        0,
                        len(self._planned_path.get("points", ())) - 1,
                    ),
                    "reason": "confirmed_point_inside_10cm",
                }]
                if confirmed_stop
                else [{
                    "start_index": 0,
                    "end_index": max(
                        0,
                        len(self._planned_path.get("points", ())) - 1,
                    ),
                    "reason": "turn_side_unresolved",
                }]
                if candidate["status"] == "turn_side_unresolved"
                else []
            )
        else:
            blocked_segments = self._path_collision_segments(
                self._planned_path,
                obstacles,
            )
        selected = self._path_selector.select(
            candidate,
            bool(blocked_segments),
            blocked_segments,
            now_ns,
        )
        self._set_planned_path(selected, now_ns)

    def _planning_obstacle_payload(
        self,
        now: float,
    ) -> List[Dict[str, object]]:
        """Fuse immediate confirmed returns with motion-relative strong memory."""
        combined: Dict[Tuple[int, int], Dict[str, object]] = {}
        adc_near_walls = self._verified_adc_wall_obstacles(
            self._adc_angular_surface.payload(now)
        )
        containers = (
            (
                self._probability_grid.active_cells(now),
                "probability",
                GRID_RESOLUTION_M / math.sqrt(2.0),
            ),
            (
                self._near_obstacle_payload(now),
                "reactive",
                NEAR_OCCUPANCY_RESOLUTION_M / math.sqrt(2.0),
            ),
            (
                adc_near_walls,
                "adc_near_wall",
                None,
            ),
        )
        for obstacles, evidence_kind, support_radius_m in containers:
            for obstacle in obstacles:
                # Only heatmap evidence remains meaningful as a continuous
                # sub-threshold cost.  A decayed point/mixed cell must not be
                # resurrected as hard collision geometry.
                if (
                    evidence_kind == "probability"
                    and
                    str(obstacle.get("source", "unknown")) != "heatmap"
                    and obstacle.get("occupied") is not True
                ):
                    continue
                normalized = self._normalize_planning_obstacle({
                    **obstacle,
                    "evidence_kind": evidence_kind,
                    "support_radius_m": (
                        obstacle.get("support_radius_m", 0.0)
                        if support_radius_m is None
                        else support_radius_m
                    ),
                })
                forward = float(normalized["forward_m"])
                lateral = float(normalized["lateral_m"])
                if (
                    not math.isfinite(forward)
                    or not math.isfinite(lateral)
                    or forward < 0.0
                    or forward > PATH_LOCAL_MAX_RANGE_M
                    or abs(lateral) > PATH_LOCAL_MAX_RANGE_M
                    or abs(math.degrees(math.atan2(lateral, forward)))
                    > PATH_FOV_AZIMUTH_DEG
                ):
                    continue
                key = (
                    round(forward / NEAR_OCCUPANCY_RESOLUTION_M),
                    round(lateral / NEAR_OCCUPANCY_RESOLUTION_M),
                )
                normalized["forward_m"] = (
                    key[0] * NEAR_OCCUPANCY_RESOLUTION_M
                )
                normalized["lateral_m"] = (
                    key[1] * NEAR_OCCUPANCY_RESOLUTION_M
                )
                existing = combined.get(key)
                combined[key] = (
                    normalized
                    if existing is None
                    else self._merge_planning_obstacles(existing, normalized)
                )
        return list(combined.values())

    @staticmethod
    def _verified_adc_wall_obstacles(
        adc_surface: Dict[str, object],
    ) -> List[Dict[str, object]]:
        """Promote only a persistent, range-continuous ADC ridge to a wall.

        One coarse UART ADC angle peak is too ambiguous to be collision
        geometry.  Two or more adjacent range bins on the same side, each
        verified over time with coherent antenna phase, are substantially
        stronger evidence that a near wall occupies that side of the robot.
        """

        range_step_m = adc_surface.get("range_step_m")
        if not isinstance(range_step_m, (int, float)) or isinstance(
            range_step_m, bool
        ):
            return []
        range_step_m = float(range_step_m)
        if not math.isfinite(range_step_m) or range_step_m <= 0.0:
            return []
        required_hits = max(3, int(adc_surface.get("required_hits", 3)))
        candidates: List[Dict[str, object]] = []
        for raw_peak in adc_surface.get("peaks", ()):
            if not isinstance(raw_peak, dict):
                continue
            try:
                peak = dict(raw_peak)
                range_m = float(peak["range_m"])
                azimuth_deg = float(peak["azimuth_deg"])
                confidence = float(peak["confidence"])
                coherence = float(peak["coherence"])
                relative_db = float(peak["relative_db"])
                hits = int(peak["hits"])
                range_index = int(peak["range_index"])
            except (KeyError, TypeError, ValueError):
                continue
            values = (
                range_m,
                azimuth_deg,
                confidence,
                coherence,
                relative_db,
            )
            if (
                any(not math.isfinite(value) for value in values)
                or peak.get("verified") is not True
                or not ADC_NEAR_WALL_MIN_RANGE_M <= range_m
                <= ADC_NEAR_WALL_MAX_RANGE_M
                or confidence < ADC_NEAR_WALL_MIN_CONFIDENCE
                or coherence < ADC_NEAR_WALL_MIN_COHERENCE
                or relative_db < ADC_NEAR_WALL_MIN_RELATIVE_DB
                or hits < required_hits
            ):
                continue
            peak.update({
                "range_m": range_m,
                "azimuth_deg": azimuth_deg,
                "confidence": confidence,
                "coherence": coherence,
                "relative_db": relative_db,
                "hits": hits,
                "range_index": range_index,
            })
            candidates.append(peak)

        # Split left, centre, and right so opposite-side lobes cannot combine
        # into an invented wall.  The centre band still detects a frontal wall.
        side_groups: Dict[int, List[Dict[str, object]]] = {-1: [], 0: [], 1: []}
        for peak in candidates:
            azimuth_deg = float(peak["azimuth_deg"])
            side = -1 if azimuth_deg < -10.0 else 1 if azimuth_deg > 10.0 else 0
            side_groups[side].append(peak)

        qualifying: List[Dict[str, object]] = []
        for side_peaks in side_groups.values():
            ordered = sorted(side_peaks, key=lambda item: int(item["range_index"]))
            runs: List[List[Dict[str, object]]] = []
            for peak in ordered:
                if not runs:
                    runs.append([peak])
                    continue
                previous = runs[-1][-1]
                adjacent_range = (
                    int(peak["range_index"]) - int(previous["range_index"]) == 1
                )
                consistent_angle = abs(
                    float(peak["azimuth_deg"])
                    - float(previous["azimuth_deg"])
                ) <= ADC_NEAR_WALL_MAX_AZIMUTH_DELTA_DEG
                if adjacent_range and consistent_angle:
                    runs[-1].append(peak)
                else:
                    runs.append([peak])
            for run in runs:
                if len(run) >= ADC_NEAR_WALL_MIN_PEAKS:
                    qualifying.extend(run)

        obstacles: List[Dict[str, object]] = []
        for peak in qualifying:
            range_m = float(peak["range_m"])
            azimuth_deg = float(peak["azimuth_deg"])
            angle_rad = math.radians(azimuth_deg)
            angular_uncertainty_deg = float(
                peak.get("angular_uncertainty_deg", 14.5)
            )
            obstacles.append({
                "forward_m": range_m * math.cos(angle_rad),
                "lateral_m": range_m * math.sin(angle_rad),
                "range_uncertainty_m": range_step_m / 2.0,
                "lateral_uncertainty_m": max(
                    0.025,
                    range_m * math.sin(math.radians(angular_uncertainty_deg)),
                ),
                "support_radius_m": range_step_m / 2.0,
                "confidence": float(peak["confidence"]),
                "source": "adc_near_wall",
                "hits": int(peak["hits"]),
                "verified": True,
                "range_index": int(peak["range_index"]),
                "azimuth_deg": azimuth_deg,
                "relative_db": float(peak["relative_db"]),
                "coherence": float(peak["coherence"]),
            })
        return obstacles

    @staticmethod
    def _neutral_planned_path(status: str) -> Dict[str, object]:
        return {
            "status": status,
            "stability_status": status,
            "resolution_m": PATH_RESOLUTION_M,
            "length_m": 0.0,
            "curvature_per_m": 0.0,
            "points": [[0.0, 0.0]],
            "robot_width_m": PATH_ROBOT_WIDTH_M,
            "safety_margin_m": PATH_SAFETY_MARGIN_M,
            "minimum_passage_width_m": (
                PATH_ROBOT_WIDTH_M + PATH_SAFETY_MARGIN_M * 2.0
            ),
            "minimum_hold_ms": 0,
            "switch_confirm_frames": 3,
            "pending_switch_frames": 0,
            "switch_accepted": False,
            "replan_every_frame": True,
            "selected_for_ms": 0,
            "blocked_previous_path": None,
        }

    @staticmethod
    def _normalize_planning_obstacle(
        obstacle: Dict[str, object],
    ) -> Dict[str, object]:
        normalized = dict(obstacle)
        normalized["forward_m"] = float(obstacle["forward_m"])
        normalized["lateral_m"] = float(obstacle["lateral_m"])
        for field in (
            "range_uncertainty_m",
            "lateral_uncertainty_m",
            "support_radius_m",
        ):
            value = float(obstacle.get(field, 0.0))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field} must be finite and non-negative")
            normalized[field] = value
        normalized["confidence"] = float(obstacle.get("confidence", 0.0))
        normalized["hits"] = min(255, max(0, int(obstacle.get("hits", 0))))
        normalized["evidence_kind"] = str(
            obstacle.get("evidence_kind", "direct")
        )
        normalized["source"] = str(obstacle.get("source", "unknown"))
        return normalized

    @staticmethod
    def _is_hard_obstacle(obstacle: Dict[str, object]) -> bool:
        """Use point-backed or verified ADC-ridge evidence as geometry.

        A range-azimuth heatmap lobe has no elevation and a 29-degree angular
        footprint on this board.  It remains useful as a continuous steering
        cost, but treating its whole footprint as a solid wall creates false
        near-field stops.  A verified ADC ridge is emitted under a distinct
        source after temporal and range-continuity checks.  A mixed cell is
        point- or ADC-backed and stays hard.
        """

        return str(obstacle.get("source", "unknown")) != "heatmap"

    @staticmethod
    def _heatmap_point_matches(
        heatmap_forward_m: float,
        heatmap_lateral_m: float,
        point_forward_m: float,
        point_lateral_m: float,
        range_step_m: float,
    ) -> bool:
        values = (
            heatmap_forward_m,
            heatmap_lateral_m,
            point_forward_m,
            point_lateral_m,
            range_step_m,
        )
        if any(not math.isfinite(float(value)) for value in values):
            return False
        if range_step_m <= 0.0:
            return False
        heatmap_range = math.hypot(heatmap_forward_m, heatmap_lateral_m)
        point_range = math.hypot(point_forward_m, point_lateral_m)
        radial_match = abs(heatmap_range - point_range) <= (
            range_step_m * 0.75 + 1e-9
        )
        if heatmap_range <= 1e-9 or point_range <= 1e-9:
            return radial_match
        angular_delta = abs(math.degrees(math.atan2(
            math.sin(
                math.atan2(heatmap_lateral_m, heatmap_forward_m)
                - math.atan2(point_lateral_m, point_forward_m)
            ),
            math.cos(
                math.atan2(heatmap_lateral_m, heatmap_forward_m)
                - math.atan2(point_lateral_m, point_forward_m)
            ),
        )))
        return radial_match and angular_delta <= 14.5 + 1e-9

    @staticmethod
    def _merge_planning_obstacles(
        left: Dict[str, object],
        right: Dict[str, object],
    ) -> Dict[str, object]:
        merged = dict(left)
        for field in (
            "range_uncertainty_m",
            "lateral_uncertainty_m",
            "support_radius_m",
            "confidence",
        ):
            merged[field] = max(float(left[field]), float(right[field]))
        merged["hits"] = min(255, max(int(left["hits"]), int(right["hits"])))
        if left["evidence_kind"] != right["evidence_kind"]:
            merged["evidence_kind"] = "mixed"
        if left["source"] != right["source"]:
            merged["source"] = "mixed"
        return merged

    @staticmethod
    def _wall_obstacles_linked(
        left: Dict[str, object],
        right: Dict[str, object],
    ) -> bool:
        left_forward = float(left["forward_m"])
        left_lateral = float(left["lateral_m"])
        right_forward = float(right["forward_m"])
        right_lateral = float(right["lateral_m"])
        distance = math.hypot(
            right_forward - left_forward,
            right_lateral - left_lateral,
        )
        link_limit = (
            OBSTACLE_CLUSTER_LINK_M
            if left["source"] == right["source"]
            else OBSTACLE_CLUSTER_CROSS_SOURCE_LINK_M
        )
        return (
            distance <= link_limit
            and abs(
                math.hypot(right_forward, right_lateral)
                - math.hypot(left_forward, left_lateral)
            ) <= OBSTACLE_CLUSTER_MAX_RANGE_DELTA_M
            and abs(
                math.degrees(math.atan2(right_lateral, right_forward))
                - math.degrees(math.atan2(left_lateral, left_forward))
            ) <= OBSTACLE_CLUSTER_MAX_AZIMUTH_DELTA_DEG
        )

    @staticmethod
    def _interpolate_wall_obstacles(
        obstacles: Sequence[Dict[str, object]],
    ) -> Tuple[List[Dict[str, object]], int]:
        normalized = [
            RadarSceneEstimator._normalize_planning_obstacle(obstacle)
            for obstacle in obstacles
        ]
        positions: Dict[Tuple[float, float], Dict[str, object]] = {}
        for obstacle in normalized:
            key = (
                float(obstacle["forward_m"]),
                float(obstacle["lateral_m"]),
            )
            existing = positions.get(key)
            positions[key] = (
                obstacle
                if existing is None
                else RadarSceneEstimator._merge_planning_obstacles(
                    existing, obstacle
                )
            )
        segment_count = 0
        for left_index, left in enumerate(normalized):
            for right in normalized[left_index + 1:]:
                if not RadarSceneEstimator._wall_obstacles_linked(left, right):
                    continue
                segment_count += 1
                distance = math.hypot(
                    float(right["forward_m"]) - float(left["forward_m"]),
                    float(right["lateral_m"]) - float(left["lateral_m"]),
                )
                samples = max(
                    1,
                    math.ceil(distance / NEAR_OCCUPANCY_RESOLUTION_M),
                )
                for sample_index in range(1, samples):
                    ratio = sample_index / samples
                    sample = RadarSceneEstimator._merge_planning_obstacles(
                        left, right
                    )
                    sample["forward_m"] = round(
                        float(left["forward_m"])
                        + (
                            float(right["forward_m"])
                            - float(left["forward_m"])
                        ) * ratio,
                        4,
                    )
                    sample["lateral_m"] = round(
                        float(left["lateral_m"])
                        + (
                            float(right["lateral_m"])
                            - float(left["lateral_m"])
                        ) * ratio,
                        4,
                    )
                    key = (sample["forward_m"], sample["lateral_m"])
                    existing = positions.get(key)
                    positions[key] = (
                        sample
                        if existing is None
                        else RadarSceneEstimator._merge_planning_obstacles(
                            existing, sample
                        )
                    )
        return list(positions.values()), segment_count

    @staticmethod
    def _obstacle_collision_radii(
        obstacle: Dict[str, object],
    ) -> Tuple[float, float]:
        physical_axes = RadarSceneEstimator._obstacle_collision_axes(
            obstacle,
            include_grid_padding=False,
        )
        effective_axes = RadarSceneEstimator._obstacle_collision_axes(
            obstacle,
            include_grid_padding=True,
        )
        return max(physical_axes), max(effective_axes)

    @staticmethod
    def _obstacle_collision_axes(
        obstacle: Dict[str, object],
        *,
        include_grid_padding: bool,
    ) -> Tuple[float, float]:
        normalized = RadarSceneEstimator._normalize_planning_obstacle(obstacle)
        support_radius = float(normalized["support_radius_m"])
        radial_base = PATH_SAFETY_MARGIN_M + support_radius
        tangential_base = (
            PATH_ROBOT_WIDTH_M / 2.0
            + PATH_SAFETY_MARGIN_M
            + support_radius
        )
        padding = (
            PATH_COLLISION_GRID_M / math.sqrt(2.0)
            if include_grid_padding
            else 0.0
        )
        return (
            radial_base + float(normalized["range_uncertainty_m"]) + padding,
            tangential_base
            + float(normalized["lateral_uncertainty_m"])
            + padding,
        )

    @staticmethod
    def _inside_collision_ellipse(
        forward_m: float,
        lateral_m: float,
        obstacle: Dict[str, object],
        radial_radius_m: float,
        tangential_radius_m: float,
    ) -> bool:
        obstacle_forward = float(obstacle["forward_m"])
        obstacle_lateral = float(obstacle["lateral_m"])
        obstacle_distance = math.hypot(obstacle_forward, obstacle_lateral)
        if obstacle_distance <= 1e-12:
            radial_forward, radial_lateral = 1.0, 0.0
        else:
            radial_forward = obstacle_forward / obstacle_distance
            radial_lateral = obstacle_lateral / obstacle_distance
        delta_forward = forward_m - obstacle_forward
        delta_lateral = lateral_m - obstacle_lateral
        radial_delta = (
            delta_forward * radial_forward
            + delta_lateral * radial_lateral
        )
        tangential_delta = (
            -delta_forward * radial_lateral
            + delta_lateral * radial_forward
        )
        normalized_distance = (
            (radial_delta / radial_radius_m) ** 2
            + (tangential_delta / tangential_radius_m) ** 2
        )
        return normalized_distance <= 1.0 + 1e-12

    @staticmethod
    def _path_collision_segments(
        path: Dict[str, object],
        obstacles: Sequence[Dict[str, object]],
    ) -> List[Dict[str, object]]:
        interpolated, _ = RadarSceneEstimator._interpolate_wall_obstacles(
            tuple(
                obstacle
                for obstacle in obstacles
                if RadarSceneEstimator._is_hard_obstacle(obstacle)
            )
        )
        blocked_indices: List[Tuple[int, str]] = []
        for index, point in enumerate(path.get("points", [])):
            reason = None
            for obstacle in interpolated:
                radial_radius, tangential_radius = (
                    RadarSceneEstimator._obstacle_collision_axes(
                        obstacle,
                        include_grid_padding=True,
                    )
                )
                if RadarSceneEstimator._inside_collision_ellipse(
                    float(point[0]),
                    float(point[1]),
                    obstacle,
                    radial_radius,
                    tangential_radius,
                ):
                    reason = (
                        "near_field_overlap"
                        if math.hypot(
                            float(obstacle["forward_m"]),
                            float(obstacle["lateral_m"]),
                        ) <= max(radial_radius, tangential_radius)
                        else "confirmed_obstacle"
                    )
                    break
            if reason is not None:
                blocked_indices.append((index, reason))
        segments: List[Dict[str, object]] = []
        for index, reason in blocked_indices:
            if (
                segments
                and index == int(segments[-1]["end_index"]) + 1
                and reason == segments[-1]["reason"]
            ):
                segments[-1]["end_index"] = index
            else:
                segments.append({
                    "start_index": index,
                    "end_index": index,
                    "reason": reason,
                })
        return segments

    def _set_planned_path(
        self,
        selection: Dict[str, object],
        now_ns: int,
    ) -> None:
        path = dict(selection["path"])
        status = str(selection["status"])
        self._path_stability_status = status
        selected_since_ns = int(selection["selected_since_ns"])
        held_ms = max(0, (now_ns - selected_since_ns) // 1_000_000)
        self._planned_path = {
            **path,
            "stability_status": status,
            "selected_for_ms": held_ms,
            "minimum_hold_ms": 0,
            "switch_confirm_frames": 3,
            "pending_switch_frames": int(
                selection["pending_switch_frames"]
            ),
            "switch_accepted": bool(selection["switch_accepted"]),
            "replan_every_frame": True,
            "blocked_previous_path": selection["blocked_previous_path"],
        }

    @staticmethod
    def _plan_reflectivity_opening(
        reflectivity_cells: Sequence[Dict[str, object]],
        obstacle_tracks: Sequence[Dict[str, object]],
        *,
        confirmed_stop: bool = False,
        score_history: Sequence[Dict[float, float]] = (),
    ) -> Dict[str, object]:
        """Build an advisory centreline through the lowest-reflection sector.

        Every valid range/azimuth cell contributes continuously.  No cell is
        promoted into a wall and no absolute signal threshold declares free
        space.  The result is therefore a relative steering aid, while the
        independent confirmed-point 10 cm guard remains the stop authority.
        """
        candidates = tuple(float(angle) for angle in range(-40, 41, 5))
        cells: List[Tuple[float, float, float, float]] = []
        for cell in reflectivity_cells:
            try:
                range_m = float(cell["range_m"])
                azimuth_deg = float(cell["azimuth_deg"])
                intensity = float(cell["intensity"])
                confidence = float(cell.get("confidence", 1.0))
            except (KeyError, TypeError, ValueError):
                continue
            if (
                not all(math.isfinite(value) for value in (
                    range_m, azimuth_deg, intensity, confidence,
                ))
                or not SPI_REFLECTIVITY_VALID_MIN_RANGE_M
                <= range_m <= PATH_LOCAL_MAX_RANGE_M
                or abs(azimuth_deg) > PATH_FOV_AZIMUTH_DEG
            ):
                continue
            cells.append((
                range_m,
                azimuth_deg,
                max(0.0, min(1.0, intensity)),
                max(0.0, min(1.0, confidence)),
            ))

        normalized_obstacles = [
            RadarSceneEstimator._normalize_planning_obstacle(obstacle)
            for obstacle in obstacle_tracks
        ]

        def angular_difference(left: float, right: float) -> float:
            return abs((left - right + 180.0) % 360.0 - 180.0)

        def directional_profile_metrics(
            candidate_deg: float,
            minimum_range_m: float,
            maximum_range_m: float,
        ) -> Tuple[float, float, int]:
            range_profiles: Dict[float, List[float]] = {}
            for range_m, azimuth_deg, intensity, confidence in cells:
                if not minimum_range_m <= range_m <= maximum_range_m:
                    continue
                angle_weight = math.exp(
                    -0.5
                    * (
                        angular_difference(azimuth_deg, candidate_deg) / 15.0
                    )
                    ** 2
                )
                weight = angle_weight * (0.55 + 0.45 * confidence)
                bucket = range_profiles.setdefault(range_m, [0.0, 0.0])
                bucket[0] += weight * intensity
                bucket[1] += weight
            directional_profile = [
                energy / weight
                for energy, weight in range_profiles.values()
                if weight > 1e-9
            ]
            profile_mean = (
                statistics.mean(directional_profile)
                if directional_profile
                else 1.0
            )
            profile_upper = (
                statistics.quantiles(
                    directional_profile,
                    n=5,
                    method="inclusive",
                )[3]
                if len(directional_profile) >= 2
                else profile_mean
            )
            return profile_mean, profile_upper, len(directional_profile)

        scored: List[Tuple[float, float, float, float]] = []
        profile_metrics: Dict[float, Tuple[float, float]] = {}
        near_profile_metrics: Dict[float, Tuple[float, float, int]] = {}
        for candidate_deg in candidates:
            profile_mean, profile_upper, _ = directional_profile_metrics(
                candidate_deg,
                REFLECTIVITY_STEER_MIN_RANGE_M,
                PATH_LOCAL_MAX_RANGE_M,
            )
            # Preserve the range profile instead of averaging a wall away.
            # A far wall can occupy only the last few 2 cm bins; the upper
            # profile statistic keeps it visible while a single noisy bin
            # still cannot dominate the route.
            surface_cost = (
                profile_mean * REFLECTIVITY_PROFILE_MEAN_WEIGHT
                + profile_upper * REFLECTIVITY_PROFILE_UPPER_WEIGHT
            )
            profile_metrics[candidate_deg] = (profile_mean, profile_upper)

            near_profile_metrics[candidate_deg] = directional_profile_metrics(
                candidate_deg,
                REFLECTIVITY_NEAR_STEER_MIN_RANGE_M,
                REFLECTIVITY_NEAR_STEER_MAX_RANGE_M,
            )

        near_costs = {
            angle: (
                metrics[0] * REFLECTIVITY_PROFILE_MEAN_WEIGHT
                + metrics[1] * REFLECTIVITY_PROFILE_UPPER_WEIGHT
            )
            for angle, metrics in near_profile_metrics.items()
        }
        near_straight_cost = near_costs.get(0.0, 1.0)
        near_side_cost = min(
            (
                cost
                for angle, cost in near_costs.items()
                if abs(angle) >= 20.0
            ),
            default=near_straight_cost,
        )
        near_profile_range_count = near_profile_metrics.get(
            0.0,
            (1.0, 1.0, 0),
        )[2]
        near_steering_gain = max(
            0.0,
            near_straight_cost - near_side_cost,
        )
        near_field_blockage_evidence = (
            near_profile_range_count >= 2
            and near_steering_gain >= REFLECTIVITY_NEAR_BLOCK_MIN_GAIN
        )

        for candidate_deg in candidates:
            profile_mean, profile_upper = profile_metrics[candidate_deg]
            surface_cost = (
                profile_mean * REFLECTIVITY_PROFILE_MEAN_WEIGHT
                + profile_upper * REFLECTIVITY_PROFILE_UPPER_WEIGHT
            )
            near_cost = near_costs.get(candidate_deg, 1.0)
            if near_field_blockage_evidence:
                surface_cost += (
                    near_cost * REFLECTIVITY_NEAR_BLOCK_COST_WEIGHT
                )

            point_cost = 0.0
            for obstacle in normalized_obstacles:
                forward_m = float(obstacle["forward_m"])
                lateral_m = float(obstacle["lateral_m"])
                distance_m = math.hypot(forward_m, lateral_m)
                if distance_m <= 1e-9 or distance_m > PATH_LOCAL_MAX_RANGE_M:
                    continue
                obstacle_angle = math.degrees(
                    math.atan2(lateral_m, forward_m)
                )
                angle_weight = math.exp(
                    -0.5
                    * (
                        angular_difference(obstacle_angle, candidate_deg)
                        / 14.0
                    )
                    ** 2
                )
                distance_weight = math.exp(-distance_m / 0.20)
                point_cost += (
                    max(0.0, min(1.0, float(obstacle["confidence"])))
                    * angle_weight
                    * distance_weight
                )
            point_cost = min(1.5, point_cost)
            turn_cost = abs(candidate_deg) / 40.0 * 0.045
            # Sparse Cartesian points are valuable for the independent 10 cm
            # stop guard, but their azimuth jumps between frames in a static
            # near-field scene.  Above the stop distance they provide only a
            # small tie-breaker; the five-frame full reflectivity surface
            # remains the steering authority.
            total_cost = (
                surface_cost
                + point_cost * REFLECTIVITY_POINT_STEER_WEIGHT
                + turn_cost
            )
            scored.append((
                total_cost,
                candidate_deg,
                surface_cost,
                point_cost,
            ))

        if not cells:
            scored = [(1.0, 0.0, 1.0, 0.0)]
        instant_scored = list(scored)
        instant_best = min(
            instant_scored,
            key=lambda item: (item[0], abs(item[1])),
        )
        if score_history:
            scored = [
                (
                    statistics.median([
                        cost,
                        *(
                            float(history[angle])
                            for history in score_history
                            if angle in history
                        ),
                    ]),
                    angle,
                    surface_cost,
                    point_cost,
                )
                for cost, angle, surface_cost, point_cost in instant_scored
            ]
        raw_best = min(scored, key=lambda item: (item[0], abs(item[1])))
        left_best = min(
            (item for item in scored if item[1] <= -15.0),
            key=lambda item: (item[0], abs(item[1])),
            default=None,
        )
        right_best = min(
            (item for item in scored if item[1] >= 15.0),
            key=lambda item: (item[0], abs(item[1])),
            default=None,
        )
        side_cost_gap = (
            abs(left_best[0] - right_best[0])
            if left_best is not None and right_best is not None
            else math.inf
        )
        turn_side_unresolved = (
            near_field_blockage_evidence
            and left_best is not None
            and right_best is not None
            and side_cost_gap < REFLECTIVITY_SIDE_RESOLUTION_MIN_GAP
        )
        if near_field_blockage_evidence and abs(raw_best[1]) < 15.0:
            # The close range profile already proves that the centre ray is
            # not the continuation of the opening.  Do not let the generic
            # turn-effort prior pull the advisory line back through that
            # bend; choose either side exclusively from measured cost.
            raw_best = min(
                (
                    item for item in scored
                    if abs(item[1]) >= 15.0
                ),
                key=lambda item: (item[0], abs(item[1])),
            )
        straight = min(scored, key=lambda item: abs(item[1]))
        steering_gain = max(0.0, straight[0] - raw_best[0])
        steering_relative_gain = steering_gain / max(straight[0], 0.05)
        straight_profile_mean, straight_profile_upper = profile_metrics.get(
            straight[1],
            (straight[2], straight[2]),
        )
        straight_profile_excess = max(
            0.0,
            straight_profile_upper - straight_profile_mean,
        )
        straight_blockage_evidence = (
            near_field_blockage_evidence
            or (
                straight_profile_upper >= REFLECTIVITY_FORWARD_BLOCK_UPPER
                and straight_profile_excess
                >= REFLECTIVITY_FORWARD_BLOCK_EXCESS
            )
        )
        straight_preferred = (
            abs(raw_best[1]) > 1e-9
            and (
                steering_gain < REFLECTIVITY_STEER_MIN_GAIN
                or steering_relative_gain
                < REFLECTIVITY_STEER_MIN_RELATIVE_GAIN
            )
            and not straight_blockage_evidence
        )
        best = straight if straight_preferred else raw_best
        best_cost, goal_angle_deg, surface_cost, point_cost = best
        profile_mean, profile_upper = profile_metrics.get(
            goal_angle_deg,
            (surface_cost, surface_cost),
        )
        median_cost = statistics.median(item[0] for item in scored)
        tolerance = max(0.015, (median_cost - best_cost) * 0.30)
        open_angles = sorted(
            item[1] for item in scored
            if item[0] <= best_cost + tolerance
        )
        goal_corridor_width_deg = 5.0
        if open_angles:
            connected = [goal_angle_deg]
            cursor = goal_angle_deg - 5.0
            while cursor in open_angles:
                connected.insert(0, cursor)
                cursor -= 5.0
            cursor = goal_angle_deg + 5.0
            while cursor in open_angles:
                connected.append(cursor)
                cursor += 5.0
            goal_corridor_width_deg = max(
                5.0,
                connected[-1] - connected[0] + 5.0,
            )

        goal_range_m = 0.26
        # A near bend must become visible before the old 7 cm straight
        # tangent.  Far-field guidance keeps the gentler curve.
        tangent_m = 0.025 if near_field_blockage_evidence else 0.07

        def sampled_curve(angle_deg: float) -> Tuple[
            Tuple[float, float], List[List[float]]
        ]:
            angle_rad = math.radians(angle_deg)
            curve_goal = (
                goal_range_m * math.cos(angle_rad),
                goal_range_m * math.sin(angle_rad),
            )
            control = (
                (0.0, 0.0),
                (tangent_m, 0.0),
                (
                    curve_goal[0] - tangent_m * math.cos(angle_rad),
                    curve_goal[1] - tangent_m * math.sin(angle_rad),
                ),
                curve_goal,
            )

            def bezier(t: float) -> Tuple[float, float]:
                u = 1.0 - t
                return (
                    u**3 * control[0][0]
                    + 3.0 * u * u * t * control[1][0]
                    + 3.0 * u * t * t * control[2][0]
                    + t**3 * control[3][0],
                    u**3 * control[0][1]
                    + 3.0 * u * u * t * control[1][1]
                    + 3.0 * u * t * t * control[2][1]
                    + t**3 * control[3][1],
                )

            dense = [bezier(index / 200.0) for index in range(201)]
            cumulative = [0.0]
            for curve_left, curve_right in zip(dense, dense[1:]):
                cumulative.append(
                    cumulative[-1]
                    + math.hypot(
                        curve_right[0] - curve_left[0],
                        curve_right[1] - curve_left[1],
                    )
                )
            sample_count = math.floor(
                (cumulative[-1] + 1e-9) / PATH_RESOLUTION_M
            )
            curve_points: List[List[float]] = [[0.0, 0.0]]
            dense_index = 0
            for sample_index in range(1, sample_count + 1):
                target = sample_index * PATH_RESOLUTION_M
                while (
                    dense_index + 1 < len(cumulative)
                    and cumulative[dense_index + 1] < target
                ):
                    dense_index += 1
                if dense_index + 1 >= len(dense):
                    break
                span = cumulative[dense_index + 1] - cumulative[dense_index]
                ratio = 0.0 if span <= 1e-12 else (
                    target - cumulative[dense_index]
                ) / span
                curve_left = dense[dense_index]
                curve_right = dense[dense_index + 1]
                curve_points.append([
                    round(
                        curve_left[0]
                        + (curve_right[0] - curve_left[0]) * ratio,
                        4,
                    ),
                    round(
                        curve_left[1]
                        + (curve_right[1] - curve_left[1]) * ratio,
                        4,
                    ),
                ])
            return curve_goal, curve_points

        goal, points = sampled_curve(goal_angle_deg)
        turn_candidates: List[Dict[str, object]] = []
        if turn_side_unresolved:
            for direction, candidate in (
                ("left", left_best),
                ("right", right_best),
            ):
                candidate_goal, candidate_points = sampled_curve(candidate[1])
                turn_candidates.append({
                    "direction": direction,
                    "angle_deg": round(candidate[1], 1),
                    "cost": round(candidate[0], 4),
                    "goal_forward_m": round(candidate_goal[0], 3),
                    "goal_lateral_m": round(candidate_goal[1], 3),
                    "points": candidate_points,
                })

        if confirmed_stop or turn_side_unresolved:
            points = [[0.0, 0.0]]
        length_m = (len(points) - 1) * PATH_RESOLUTION_M
        contrast = max(
            0.0,
            min(1.0, (median_cost - best_cost) / max(median_cost, 0.05)),
        )
        persistence = (
            statistics.mean(cell[3] for cell in cells) if cells else 0.0
        )
        geometry_confidence = min(
            0.92,
            0.40 + contrast * 1.8 + persistence * 0.18,
        )
        curvature = (
            0.0
            if confirmed_stop or turn_side_unresolved
            else 2.0 * math.radians(goal_angle_deg) / goal_range_m
        )
        path_status = (
            "blocked_near_field"
            if confirmed_stop
            else "turn_side_unresolved"
            if turn_side_unresolved
            else "clear"
        )
        return {
            "status": path_status,
            "resolution_m": PATH_RESOLUTION_M,
            "length_m": round(length_m, 2),
            "curvature_per_m": round(curvature, 3),
            "planner_mode": "full_reflectivity_opening",
            "search_resolution_m": 5.0,
            "heading_resolution_deg": 5.0,
            "minimum_turn_radius_m": round(
                1.0 / max(abs(curvature), 1e-9), 3
            ) if abs(curvature) > 1e-9 else None,
            "goal_reached": not confirmed_stop and not turn_side_unresolved,
            "search_expansions": len(scored),
            "turn_count": (
                0
                if turn_side_unresolved or abs(goal_angle_deg) < 2.5
                else 1
            ),
            "direction_changes": 0,
            "receding_horizon_m": goal_range_m,
            "local_max_range_m": PATH_LOCAL_MAX_RANGE_M,
            "fov_azimuth_deg": PATH_FOV_AZIMUTH_DEG,
            "smoothed": not confirmed_stop and not turn_side_unresolved,
            "minimum_clearance_m": None,
            "geometry_confidence": round(geometry_confidence, 3),
            "maximum_heading_step_deg": 5.0,
            "confidence_scope": "relative_reflectivity_opening_not_free_space_proof",
            "robot_width_m": PATH_ROBOT_WIDTH_M,
            "safety_margin_m": PATH_SAFETY_MARGIN_M,
            "minimum_passage_width_m": (
                PATH_ROBOT_WIDTH_M + PATH_SAFETY_MARGIN_M * 2.0
            ),
            "obstacle_wall_link_m": 0.0,
            "obstacle_wall_segments": 0,
            "obstacle_object_clusters": 0,
            "largest_object_cluster_cells": 0,
            "obstacle_cross_source_link_m": 0.0,
            "near_field_exclusion_m": 0.0,
            "near_field_excluded": 0,
            "range_uncertainty_lateral_inflation_m": 0.0,
            "obstacle_support_radius_m": 0.0,
            "collision_grid_padding_m": 0.0,
            "maximum_collision_radius_m": DANGER_ENTER_M,
            "maximum_collision_radial_m": DANGER_ENTER_M,
            "maximum_collision_tangential_m": DANGER_ENTER_M,
            "goal_forward_m": round(goal[0], 3),
            "goal_lateral_m": round(goal[1], 3),
            "goal_angle_deg": round(goal_angle_deg, 1),
            "goal_corridor_width_deg": round(goal_corridor_width_deg, 1),
            "goal_selection_mode": (
                "turn_required_side_unresolved"
                if turn_side_unresolved
                else "minimum_reflection_trend"
            ),
            "instant_best_angle_deg": round(instant_best[1], 1),
            "raw_best_angle_deg": round(raw_best[1], 1),
            "straight_preferred": straight_preferred,
            "straight_blockage_evidence": straight_blockage_evidence,
            "near_field_blockage_evidence": near_field_blockage_evidence,
            "near_profile_range_count": near_profile_range_count,
            "near_straight_cost": round(near_straight_cost, 4),
            "near_side_cost": round(near_side_cost, 4),
            "near_steering_gain": round(near_steering_gain, 4),
            "near_steering_min_gain": REFLECTIVITY_NEAR_BLOCK_MIN_GAIN,
            "near_cost_weight": REFLECTIVITY_NEAR_BLOCK_COST_WEIGHT,
            "turn_side_unresolved": turn_side_unresolved,
            "turn_side_cost_gap": (
                round(side_cost_gap, 4)
                if math.isfinite(side_cost_gap)
                else None
            ),
            "turn_side_resolution_min_gap": (
                REFLECTIVITY_SIDE_RESOLUTION_MIN_GAP
            ),
            "turn_candidates": turn_candidates,
            "path_initial_tangent_m": tangent_m,
            "straight_profile_mean": round(straight_profile_mean, 4),
            "straight_profile_upper": round(straight_profile_upper, 4),
            "straight_profile_excess": round(straight_profile_excess, 4),
            "steering_gain": round(steering_gain, 4),
            "steering_relative_gain": round(steering_relative_gain, 4),
            "steering_min_gain": REFLECTIVITY_STEER_MIN_GAIN,
            "steering_min_relative_gain": (
                REFLECTIVITY_STEER_MIN_RELATIVE_GAIN
            ),
            "hard_obstacles_used": 1 if confirmed_stop else 0,
            "soft_obstacles_used": len(normalized_obstacles),
            "persistent_obstacles_used": len(normalized_obstacles),
            "reflectivity_cells_used": len(cells),
            "opening_surface_cost": round(surface_cost, 4),
            "opening_profile_mean": round(profile_mean, 4),
            "opening_profile_upper": round(profile_upper, 4),
            "opening_profile_mean_weight": REFLECTIVITY_PROFILE_MEAN_WEIGHT,
            "opening_profile_upper_weight": (
                REFLECTIVITY_PROFILE_UPPER_WEIGHT
            ),
            "opening_steer_min_range_m": REFLECTIVITY_STEER_MIN_RANGE_M,
            "opening_point_cost": round(point_cost, 4),
            "opening_point_steer_weight": REFLECTIVITY_POINT_STEER_WEIGHT,
            "opening_contrast": round(contrast, 4),
            "opening_scores": [
                {
                    "angle_deg": angle,
                    "cost": round(cost, 4),
                }
                for cost, angle, _, _ in scored
            ],
            "instant_opening_scores": [
                {
                    "angle_deg": angle,
                    "cost": round(cost, 4),
                }
                for cost, angle, _, _ in instant_scored
            ],
            "opening_score_history_frames": min(
                REFLECTIVITY_SCORE_HISTORY_FRAMES,
                len(score_history) + 1,
            ),
            "points": points,
        }

    @staticmethod
    def _plan_path(
        obstacle_tracks: Sequence[Dict[str, object]],
        curvatures: Sequence[float] = PATH_CURVATURES,
        preferred_goal_angle_deg: Optional[float] = None,
    ) -> Dict[str, object]:
        if preferred_goal_angle_deg is not None:
            if isinstance(preferred_goal_angle_deg, bool) or not isinstance(
                preferred_goal_angle_deg,
                (int, float),
            ):
                raise TypeError("preferred_goal_angle_deg must be numeric")
            preferred_goal_angle_deg = float(preferred_goal_angle_deg)
            if not math.isfinite(preferred_goal_angle_deg):
                raise ValueError("preferred_goal_angle_deg must be finite")
            preferred_goal_angle_deg = max(
                -PATH_PREFERRED_GOAL_MAX_DEG,
                min(PATH_PREFERRED_GOAL_MAX_DEG, preferred_goal_angle_deg),
            )
        base_clearance = PATH_ROBOT_WIDTH_M / 2.0 + PATH_SAFETY_MARGIN_M
        collision_grid_padding = PATH_COLLISION_GRID_M / math.sqrt(2.0)
        normalized_obstacles = [
            RadarSceneEstimator._normalize_planning_obstacle(track)
            for track in obstacle_tracks
        ]
        observed_obstacles = [
            obstacle
            for obstacle in normalized_obstacles
            if math.isfinite(float(obstacle["forward_m"]))
            and math.isfinite(float(obstacle["lateral_m"]))
            and 0.0 <= float(obstacle["forward_m"]) <= PATH_LOCAL_MAX_RANGE_M
            and abs(float(obstacle["lateral_m"])) <= PATH_LOCAL_MAX_RANGE_M
            and abs(math.degrees(math.atan2(
                float(obstacle["lateral_m"]),
                float(obstacle["forward_m"]),
            ))) <= PATH_FOV_AZIMUTH_DEG
        ]
        hard_obstacles = [
            obstacle
            for obstacle in observed_obstacles
            if RadarSceneEstimator._is_hard_obstacle(obstacle)
        ]
        soft_obstacles = [
            obstacle
            for obstacle in observed_obstacles
            if not RadarSceneEstimator._is_hard_obstacle(obstacle)
        ]
        cluster_parents = list(range(len(observed_obstacles)))

        def find_cluster(index: int) -> int:
            while cluster_parents[index] != index:
                cluster_parents[index] = cluster_parents[
                    cluster_parents[index]
                ]
                index = cluster_parents[index]
            return index

        def union_cluster(left_index: int, right_index: int) -> None:
            left_root = find_cluster(left_index)
            right_root = find_cluster(right_index)
            if left_root != right_root:
                cluster_parents[right_root] = left_root

        for left_index, left in enumerate(observed_obstacles):
            for right_index in range(
                left_index + 1, len(observed_obstacles)
            ):
                right = observed_obstacles[right_index]
                if RadarSceneEstimator._wall_obstacles_linked(left, right):
                    union_cluster(left_index, right_index)
        cluster_sizes: Dict[int, int] = {}
        for obstacle_index in range(len(observed_obstacles)):
            root = find_cluster(obstacle_index)
            cluster_sizes[root] = cluster_sizes.get(root, 0) + 1
        object_cluster_count = len(cluster_sizes)
        largest_object_cluster = max(cluster_sizes.values(), default=0)
        obstacles, wall_segment_count = (
            RadarSceneEstimator._interpolate_wall_obstacles(
                hard_obstacles
            )
        )
        blocked_collision_cells: set[Tuple[int, int]] = set()
        for obstacle in obstacles:
            obstacle_forward = float(obstacle["forward_m"])
            obstacle_lateral = float(obstacle["lateral_m"])
            radial_radius, tangential_radius = (
                RadarSceneEstimator._obstacle_collision_axes(
                    obstacle,
                    include_grid_padding=True,
                )
            )
            collision_radius = max(radial_radius, tangential_radius)
            collision_cell_radius = math.ceil(
                collision_radius / PATH_COLLISION_GRID_M
            )
            center_forward = round(
                obstacle_forward / PATH_COLLISION_GRID_M
            )
            center_lateral = round(
                obstacle_lateral / PATH_COLLISION_GRID_M
            )
            for forward_offset in range(
                -collision_cell_radius,
                collision_cell_radius + 1,
            ):
                for lateral_offset in range(
                    -collision_cell_radius,
                    collision_cell_radius + 1,
                ):
                    cell_forward = (
                        center_forward + forward_offset
                    ) * PATH_COLLISION_GRID_M
                    cell_lateral = (
                        center_lateral + lateral_offset
                    ) * PATH_COLLISION_GRID_M
                    if RadarSceneEstimator._inside_collision_ellipse(
                        cell_forward,
                        cell_lateral,
                        obstacle,
                        radial_radius,
                        tangential_radius,
                    ):
                        blocked_collision_cells.add(
                            (
                                center_forward + forward_offset,
                                center_lateral + lateral_offset,
                            )
                        )

        def goal_from_angle(
            angle_deg: float,
            corridor_width_deg: float,
        ) -> Tuple[float, float, float, float]:
            angle_rad = math.radians(angle_deg)
            return (
                PATH_DYNAMIC_GOAL_RANGE_M * math.cos(angle_rad),
                PATH_DYNAMIC_GOAL_RANGE_M * math.sin(angle_rad),
                angle_deg,
                corridor_width_deg,
            )

        def select_dynamic_goal() -> Tuple[float, float, float, float]:
            desired_angle = (
                preferred_goal_angle_deg
                if preferred_goal_angle_deg is not None
                else 0.0
            )
            if not hard_obstacles:
                return goal_from_angle(
                    desired_angle,
                    PATH_FOV_AZIMUTH_DEG * 2.0,
                )
            occupied: List[Tuple[float, float]] = []
            for obstacle in hard_obstacles:
                forward = float(obstacle["forward_m"])
                lateral = float(obstacle["lateral_m"])
                distance = math.hypot(forward, lateral)
                _, physical_tangential_radius = (
                    RadarSceneEstimator._obstacle_collision_axes(
                        obstacle,
                        include_grid_padding=False,
                    )
                )
                azimuth = math.degrees(math.atan2(lateral, forward))
                half_width = math.degrees(
                    math.asin(
                        min(
                            0.95,
                            physical_tangential_radius / max(distance, 1e-9),
                        )
                    )
                )
                occupied.append(
                    (
                        max(-PATH_FOV_AZIMUTH_DEG, azimuth - half_width),
                        min(PATH_FOV_AZIMUTH_DEG, azimuth + half_width),
                    )
                )
            occupied.sort()
            merged: List[List[float]] = []
            for start_angle, end_angle in occupied:
                if start_angle >= end_angle:
                    continue
                if not merged or start_angle > merged[-1][1]:
                    merged.append([start_angle, end_angle])
                else:
                    merged[-1][1] = max(merged[-1][1], end_angle)
            free_intervals: List[Tuple[float, float]] = []
            cursor = -PATH_FOV_AZIMUTH_DEG
            for start_angle, end_angle in merged:
                if start_angle > cursor:
                    free_intervals.append((cursor, start_angle))
                cursor = max(cursor, end_angle)
            if cursor < PATH_FOV_AZIMUTH_DEG:
                free_intervals.append((cursor, PATH_FOV_AZIMUTH_DEG))
            if not free_intervals:
                return goal_from_angle(
                    preferred_goal_angle_deg or 0.0,
                    0.0,
                )
            if preferred_goal_angle_deg is None and not soft_obstacles:
                best_interval = max(
                    free_intervals,
                    key=lambda interval: (
                        (interval[1] - interval[0])
                        - abs((interval[0] + interval[1]) * 0.5) * 0.15,
                        -abs((interval[0] + interval[1]) * 0.5),
                    ),
                )
                goal_angle_deg = (
                    best_interval[0] + best_interval[1]
                ) * 0.5
            else:
                def guided_angle(interval: Tuple[float, float]) -> float:
                    width = interval[1] - interval[0]
                    inset = min(0.5, width * 0.5)
                    lower = interval[0] + inset
                    upper = interval[1] - inset
                    return max(
                        lower,
                        min(upper, desired_angle),
                    )

                best_interval = min(
                    free_intervals,
                    key=lambda interval: (
                        abs(guided_angle(interval) - desired_angle),
                        -(interval[1] - interval[0]),
                        abs(guided_angle(interval)),
                    ),
                )
                goal_angle_deg = guided_angle(best_interval)
            return goal_from_angle(
                goal_angle_deg,
                best_interval[1] - best_interval[0],
            )

        (
            goal_forward_m,
            goal_lateral_m,
            goal_angle_deg,
            goal_corridor_width_deg,
        ) = select_dynamic_goal()
        goal_range_m = math.hypot(goal_forward_m, goal_lateral_m)
        goal_unit_forward = goal_forward_m / max(goal_range_m, 1e-9)
        goal_unit_lateral = goal_lateral_m / max(goal_range_m, 1e-9)
        initial_curvatures = tuple(
            dict.fromkeys(
                min(
                    PATH_CURVATURES,
                    key=lambda primitive: abs(primitive - float(curvature)),
                )
                for curvature in curvatures
            )
        ) or PATH_CURVATURES

        def heading_index(heading: float) -> int:
            raw = round(heading / PATH_HEADING_STEP_RAD)
            return (
                (raw + PATH_HEADING_BINS // 2) % PATH_HEADING_BINS
                - PATH_HEADING_BINS // 2
            )

        def in_search_scene(forward: float, lateral: float) -> bool:
            if not (
                0.0 <= forward <= PATH_LOCAL_MAX_RANGE_M
                and abs(lateral) < PATH_LOCAL_MAX_RANGE_M
            ):
                return False
            if math.hypot(forward, lateral) <= 1e-9:
                return True
            return abs(math.degrees(math.atan2(lateral, forward))) <= (
                PATH_FOV_AZIMUTH_DEG
            )

        def collision_free(
            start: Tuple[float, float],
            end: Tuple[float, float],
        ) -> bool:
            for sample_index in range(1, 5):
                ratio = sample_index / 4.0
                forward = start[0] + (end[0] - start[0]) * ratio
                lateral = start[1] + (end[1] - start[1]) * ratio
                if (
                    round(forward / PATH_COLLISION_GRID_M),
                    round(lateral / PATH_COLLISION_GRID_M),
                ) in blocked_collision_cells:
                    return False
            return True

        def soft_evidence_cost(forward: float, lateral: float) -> float:
            """Continuous path cost from every repeated heatmap return.

            Heatmap cells do not become collision geometry, but a candidate
            path crossing a strong current reflection must cost more than a
            similarly smooth path through the measured opening.
            """
            total = 0.0
            for obstacle in soft_obstacles:
                confidence = max(
                    0.0, min(1.0, float(obstacle["confidence"]))
                )
                influence_m = min(
                    0.11,
                    0.035
                    + max(
                        float(obstacle.get("range_uncertainty_m", 0.0)),
                        float(obstacle.get("lateral_uncertainty_m", 0.0)),
                    ),
                )
                distance_m = math.hypot(
                    forward - float(obstacle["forward_m"]),
                    lateral - float(obstacle["lateral_m"]),
                )
                total += confidence * math.exp(
                    -0.5 * (distance_m / max(0.025, influence_m)) ** 2
                )
            return total

        State = Tuple[int, int, int]
        start: State = (0, 0, 0)
        poses: Dict[State, Tuple[float, float, float]] = {
            start: (0.0, 0.0, 0.0)
        }
        parents: Dict[State, Tuple[State, float]] = {}
        costs: Dict[State, float] = {start: 0.0}
        arc_lengths: Dict[State, float] = {start: 0.0}
        queue: List[Tuple[float, int, State]] = [
            (goal_range_m, 0, start)
        ]
        closed: set[State] = set()
        push_sequence = 0
        expansions = 0
        best_state = start
        best_score = 0.0
        goal_state: Optional[State] = None

        while queue and expansions < PATH_SEARCH_MAX_EXPANSIONS:
            priority, _, state = heapq.heappop(queue)
            del priority
            if state in closed:
                continue
            closed.add(state)
            forward, lateral, heading = poses[state]
            cost = costs[state]
            expansions += 1
            if math.hypot(
                forward - goal_forward_m,
                lateral - goal_lateral_m,
            ) <= PATH_DYNAMIC_GOAL_TOLERANCE_M:
                goal_state = state
                break
            if arc_lengths[state] >= PATH_SEARCH_MAX_ARC_M:
                continue
            primitives = initial_curvatures if state == start else PATH_CURVATURES
            for curvature in primitives:
                next_heading = heading + curvature * PATH_SEARCH_STEP_M
                midpoint_heading = (
                    heading + curvature * PATH_SEARCH_STEP_M * 0.5
                )
                next_forward = (
                    forward + math.cos(midpoint_heading) * PATH_SEARCH_STEP_M
                )
                next_lateral = (
                    lateral + math.sin(midpoint_heading) * PATH_SEARCH_STEP_M
                )
                next_forward = (
                    round(next_forward / PATH_SEARCH_GRID_M)
                    * PATH_SEARCH_GRID_M
                )
                next_lateral = (
                    round(next_lateral / PATH_SEARCH_GRID_M)
                    * PATH_SEARCH_GRID_M
                )
                next_heading_index = heading_index(next_heading)
                next_heading = next_heading_index * PATH_HEADING_STEP_RAD
                if not in_search_scene(next_forward, next_lateral):
                    continue
                if not collision_free(
                    (forward, lateral),
                    (next_forward, next_lateral),
                ):
                    continue
                next_state: State = (
                    round(next_forward / PATH_SEARCH_GRID_M),
                    round(next_lateral / PATH_SEARCH_GRID_M),
                    next_heading_index,
                )
                step_distance = math.hypot(
                    next_forward - forward,
                    next_lateral - lateral,
                )
                if step_distance < PATH_SEARCH_GRID_M * 0.5:
                    continue
                turn_cost = 0.012 if abs(curvature) > 1e-9 else 0.0
                center_recovery_cost = (
                    abs(next_lateral - goal_lateral_m) * 0.003
                )
                next_cost = (
                    cost
                    + step_distance
                    + turn_cost
                    + center_recovery_cost
                    + soft_evidence_cost(
                        next_forward, next_lateral
                    ) * 0.012
                )
                next_arc = arc_lengths[state] + step_distance
                if next_arc > PATH_SEARCH_MAX_ARC_M + 1e-9:
                    continue
                if next_cost >= costs.get(next_state, math.inf) - 1e-9:
                    continue
                costs[next_state] = next_cost
                arc_lengths[next_state] = next_arc
                poses[next_state] = (
                    next_forward,
                    next_lateral,
                    next_heading,
                )
                parents[next_state] = (state, curvature)
                heuristic = math.hypot(
                    next_forward - goal_forward_m,
                    next_lateral - goal_lateral_m,
                )
                push_sequence += 1
                heapq.heappush(
                    queue,
                    (next_cost + heuristic, push_sequence, next_state),
                )
                progress_along_goal = (
                    next_forward * goal_unit_forward
                    + next_lateral * goal_unit_lateral
                )
                cross_track_error = abs(
                    -next_forward * goal_unit_lateral
                    + next_lateral * goal_unit_forward
                )
                progress_score = (
                    progress_along_goal
                    - cross_track_error * 0.05
                    - next_cost * 0.01
                )
                if progress_score > best_score:
                    best_score = progress_score
                    best_state = next_state

        selected_state = goal_state if goal_state is not None else best_state
        states = [selected_state]
        primitive_path: List[float] = []
        while states[-1] != start:
            previous, curvature = parents[states[-1]]
            primitive_path.append(curvature)
            states.append(previous)
        states.reverse()
        primitive_path.reverse()
        lattice_points = [
            (poses[state][0], poses[state][1])
            for state in states
        ]

        def resample_polyline(
            raw_points: Sequence[Tuple[float, float]],
        ) -> List[List[float]]:
            cumulative = [0.0]
            for left, right in zip(raw_points, raw_points[1:]):
                cumulative.append(
                    cumulative[-1]
                    + math.hypot(
                        right[0] - left[0],
                        right[1] - left[1],
                    )
                )
            sampled_length = math.floor(
                (cumulative[-1] + 1e-9) / PATH_RESOLUTION_M
            ) * PATH_RESOLUTION_M
            sampled: List[List[float]] = [[0.0, 0.0]]
            segment_index = 0
            for sample_index in range(
                1,
                round(sampled_length / PATH_RESOLUTION_M) + 1,
            ):
                target = sample_index * PATH_RESOLUTION_M
                while (
                    segment_index + 1 < len(cumulative)
                    and cumulative[segment_index + 1] < target - 1e-9
                ):
                    segment_index += 1
                if segment_index + 1 >= len(raw_points):
                    break
                span = (
                    cumulative[segment_index + 1]
                    - cumulative[segment_index]
                )
                ratio = 0.0 if span <= 1e-9 else (
                    (target - cumulative[segment_index]) / span
                )
                left = raw_points[segment_index]
                right = raw_points[segment_index + 1]
                sampled.append(
                    [
                        round(
                            left[0] + (right[0] - left[0]) * ratio,
                            4,
                        ),
                        round(
                            left[1] + (right[1] - left[1]) * ratio,
                            4,
                        ),
                    ]
                )
            return sampled

        smooth_points: List[Tuple[float, float]] = []
        if len(lattice_points) >= 3:
            for index in range(len(lattice_points) - 1):
                p0 = lattice_points[max(0, index - 1)]
                p1 = lattice_points[index]
                p2 = lattice_points[index + 1]
                p3 = lattice_points[min(len(lattice_points) - 1, index + 2)]
                for subdivision in range(8):
                    if index > 0 and subdivision == 0:
                        continue
                    t = subdivision / 8.0
                    t2 = t * t
                    t3 = t2 * t
                    smooth_points.append(
                        (
                            0.5
                            * (
                                2.0 * p1[0]
                                + (-p0[0] + p2[0]) * t
                                + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                                + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
                            ),
                            0.5
                            * (
                                2.0 * p1[1]
                                + (-p0[1] + p2[1]) * t
                                + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                                + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
                            ),
                        )
                    )
            smooth_points.append(lattice_points[-1])
        smoothing_valid = bool(smooth_points) and all(
            in_search_scene(forward, lateral)
            and (
                round(forward / PATH_COLLISION_GRID_M),
                round(lateral / PATH_COLLISION_GRID_M),
            ) not in blocked_collision_cells
            for forward, lateral in smooth_points
        )
        points = resample_polyline(
            smooth_points if smoothing_valid else lattice_points
        )

        near_field_blocked = any(
            RadarSceneEstimator._inside_collision_ellipse(
                0.0,
                0.0,
                obstacle,
                *RadarSceneEstimator._obstacle_collision_axes(
                    obstacle,
                    include_grid_padding=True,
                ),
            )
            for obstacle in obstacles
        )
        if near_field_blocked:
            points = [[0.0, 0.0]]
            primitive_path = []
            goal_state = None
            smoothing_valid = False
        else:
            postcondition_collisions = (
                RadarSceneEstimator._path_collision_segments(
                    {"points": points},
                    obstacles,
                )
            )
            if postcondition_collisions:
                first_blocked_index = int(
                    postcondition_collisions[0]["start_index"]
                )
                points = points[:first_blocked_index] or [[0.0, 0.0]]
                goal_state = None
                smoothing_valid = False

        length_m = (len(points) - 1) * PATH_RESOLUTION_M
        initial_curvature = next(
            (
                curvature
                for curvature in primitive_path[
                    : max(1, round(0.50 / PATH_SEARCH_STEP_M))
                ]
                if abs(curvature) > 1e-9
            ),
            0.0,
        )
        nonzero_turns = [
            1 if curvature > 0 else -1
            for curvature in primitive_path
            if abs(curvature) > 1e-9
        ]
        direction_changes = sum(
            left != right
            for left, right in zip(nonzero_turns, nonzero_turns[1:])
        )
        minimum_clearance = PATH_LOCAL_MAX_RANGE_M
        if obstacles and points:
            minimum_clearance = min(
                math.hypot(
                    point[0] - float(obstacle["forward_m"]),
                    point[1] - float(obstacle["lateral_m"]),
                )
                - RadarSceneEstimator._obstacle_collision_radii(obstacle)[0]
                for point in points
                for obstacle in obstacles
            )
        completion = min(
            1.0,
            max(
                0.0,
                (
                    points[-1][0] * goal_unit_forward
                    + points[-1][1] * goal_unit_lateral
                )
                / max(goal_range_m, 1e-9),
            ),
        )
        if goal_state is not None:
            completion = 1.0
        # The configured 3 cm safety margin and conservative collision raster
        # are already included in base_clearance.
        clearance_score = min(1.0, max(0.0, minimum_clearance / 0.05))
        search_headroom = max(
            0.0,
            1.0 - expansions / PATH_SEARCH_MAX_EXPANSIONS,
        )
        geometry_confidence = (
            completion * 0.50
            + clearance_score * 0.25
            + search_headroom * 0.15
            + (0.10 if smoothing_valid else 0.04)
        )
        headings = [
            math.atan2(
                right[1] - left[1],
                right[0] - left[0],
            )
            for left, right in zip(points, points[1:])
        ]
        maximum_heading_step = max(
            (
                abs(
                    math.atan2(
                        math.sin(right - left),
                        math.cos(right - left),
                    )
                )
                for left, right in zip(headings, headings[1:])
            ),
            default=0.0,
        )
        maximum_uncertainty = max(
            (
                max(
                    float(obstacle["range_uncertainty_m"]),
                    float(obstacle["lateral_uncertainty_m"]),
                )
                for obstacle in obstacles
            ),
            default=0.0,
        )
        maximum_support = max(
            (float(obstacle["support_radius_m"]) for obstacle in obstacles),
            default=0.0,
        )
        maximum_collision_radius = max(
            (
                RadarSceneEstimator._obstacle_collision_radii(obstacle)[1]
                for obstacle in obstacles
            ),
            default=base_clearance + collision_grid_padding,
        )
        maximum_collision_radial = max(
            (
                RadarSceneEstimator._obstacle_collision_axes(
                    obstacle,
                    include_grid_padding=True,
                )[0]
                for obstacle in obstacles
            ),
            default=base_clearance + collision_grid_padding,
        )
        maximum_collision_tangential = max(
            (
                RadarSceneEstimator._obstacle_collision_axes(
                    obstacle,
                    include_grid_padding=True,
                )[1]
                for obstacle in obstacles
            ),
            default=base_clearance + collision_grid_padding,
        )
        return {
            "status": (
                "blocked_near_field"
                if near_field_blocked
                else "clear" if goal_state is not None
                else "limited"
            ),
            "resolution_m": PATH_RESOLUTION_M,
            "length_m": round(length_m, 2),
            "curvature_per_m": round(initial_curvature, 3),
            "planner_mode": "dynamic_corridor_hybrid_lattice",
            "search_resolution_m": PATH_SEARCH_GRID_M,
            "heading_resolution_deg": round(
                math.degrees(PATH_HEADING_STEP_RAD), 1
            ),
            "minimum_turn_radius_m": round(
                1.0 / max(abs(curvature) for curvature in PATH_CURVATURES),
                3,
            ),
            "goal_reached": goal_state is not None,
            "search_expansions": expansions,
            "turn_count": len(nonzero_turns),
            "direction_changes": direction_changes,
            "receding_horizon_m": 0.25,
            "local_max_range_m": PATH_LOCAL_MAX_RANGE_M,
            "fov_azimuth_deg": PATH_FOV_AZIMUTH_DEG,
            "smoothed": smoothing_valid,
            "minimum_clearance_m": round(minimum_clearance, 3),
            "geometry_confidence": round(geometry_confidence, 3),
            "maximum_heading_step_deg": round(
                math.degrees(maximum_heading_step), 2
            ),
            "confidence_scope": "local_geometry_not_free_space_proof",
            "robot_width_m": PATH_ROBOT_WIDTH_M,
            "safety_margin_m": PATH_SAFETY_MARGIN_M,
            "minimum_passage_width_m": (
                PATH_ROBOT_WIDTH_M + PATH_SAFETY_MARGIN_M * 2.0
            ),
            "obstacle_wall_link_m": OBSTACLE_CLUSTER_LINK_M,
            "obstacle_wall_segments": wall_segment_count,
            "obstacle_object_clusters": object_cluster_count,
            "largest_object_cluster_cells": largest_object_cluster,
            "obstacle_cross_source_link_m": (
                OBSTACLE_CLUSTER_CROSS_SOURCE_LINK_M
            ),
            "near_field_exclusion_m": 0.0,
            "near_field_excluded": 0,
            "range_uncertainty_lateral_inflation_m": maximum_uncertainty,
            "obstacle_support_radius_m": maximum_support,
            "collision_grid_padding_m": collision_grid_padding,
            "maximum_collision_radius_m": maximum_collision_radius,
            "maximum_collision_radial_m": maximum_collision_radial,
            "maximum_collision_tangential_m": maximum_collision_tangential,
            "goal_forward_m": round(goal_forward_m, 3),
            "goal_lateral_m": round(goal_lateral_m, 3),
            "goal_angle_deg": round(goal_angle_deg, 1),
            "goal_corridor_width_deg": round(
                goal_corridor_width_deg, 1
            ),
            "goal_selection_mode": (
                "reflectivity_corridor_guided"
                if preferred_goal_angle_deg is not None
                else "widest_motion_relative_corridor"
            ),
            "hard_obstacles_used": len(hard_obstacles),
            "soft_obstacles_used": len(soft_obstacles),
            "persistent_obstacles_used": len(obstacle_tracks),
            "points": points,
        }

    def _heatmap_clusters(
        self,
        frame: RadarFrame,
    ) -> Tuple[
        List[Dict[str, float]],
        int,
        List[Tuple[float, float, float]],
    ]:
        """Return spatial clusters of calibrated heatmap residual cells."""
        heatmap = frame.heatmap
        model = self.clutter_model
        if heatmap is None or model is None:
            return [], 0, []
        span_db = heatmap.ceiling_db - heatmap.floor_db
        candidates: Dict[
            Tuple[int, int], Dict[str, float]
        ] = {}
        for range_index in range(1, heatmap.range_bins):
            range_m = range_index * heatmap.range_step_m
            if range_m > SCENE_MAX_FORWARD_M:
                break
            for azimuth_index in range(heatmap.azimuth_bins):
                flat_index = (
                    range_index * heatmap.azimuth_bins + azimuth_index
                )
                observed_db = (
                    heatmap.floor_db
                    + heatmap.data[flat_index] / 255.0 * span_db
                )
                residual_db = (
                    observed_db - model.heatmap_median_db[flat_index]
                )
                mad_db = model.heatmap_mad_db[flat_index]
                enter_threshold_db = max(
                    HEATMAP_ENTER_MIN_RESIDUAL_DB,
                    HEATMAP_ENTER_MAD_MULTIPLIER * mad_db,
                )
                exit_threshold_db = max(
                    HEATMAP_EXIT_MIN_RESIDUAL_DB,
                    HEATMAP_EXIT_MAD_MULTIPLIER * mad_db,
                )
                forward_m, lateral_m = self._heatmap_cell_position(
                    range_m,
                    azimuth_index,
                    heatmap.azimuth_bins,
                )
                if not self._in_scene(forward_m, lateral_m):
                    continue
                entering = residual_db >= enter_threshold_db
                held = (
                    residual_db >= exit_threshold_db
                    and self._near_existing_heatmap_track(
                        forward_m,
                        lateral_m,
                    )
                )
                if not entering and not held:
                    continue
                candidates[(range_index, azimuth_index)] = {
                    "forward_m": forward_m,
                    "lateral_m": lateral_m,
                    "residual_db": residual_db,
                    "entering": 1.0 if entering else 0.0,
                }

        remaining = set(candidates)
        clusters: List[Dict[str, float]] = []
        while remaining:
            seed = remaining.pop()
            seed_cell = candidates[seed]
            component = [seed]
            pending = [seed]
            while pending:
                range_index, azimuth_index = pending.pop()
                for range_offset in (-1, 0, 1):
                    for azimuth_offset in (-1, 0, 1):
                        if range_offset == 0 and azimuth_offset == 0:
                            continue
                        neighbor = (
                            range_index + range_offset,
                            azimuth_index + azimuth_offset,
                        )
                        if (
                            neighbor in remaining
                            and math.hypot(
                                candidates[neighbor]["forward_m"]
                                - seed_cell["forward_m"],
                                candidates[neighbor]["lateral_m"]
                                - seed_cell["lateral_m"],
                            ) <= HEATMAP_MAX_CLUSTER_RADIUS_M
                        ):
                            remaining.remove(neighbor)
                            component.append(neighbor)
                            pending.append(neighbor)
            cells = [candidates[index] for index in component]
            peak_residual_db = max(
                cell["residual_db"] for cell in cells
            )
            if (
                len(cells) < 2
                and peak_residual_db < HEATMAP_SINGLE_CELL_PEAK_DB
            ):
                continue
            if not any(cell["entering"] for cell in cells):
                # Hysteresis may retain an existing cluster, but may not
                # create a new cluster from exit-threshold cells alone.
                if not any(
                    self._near_existing_heatmap_track(
                        cell["forward_m"],
                        cell["lateral_m"],
                    )
                    for cell in cells
                ):
                    continue
            weights = [max(0.1, cell["residual_db"]) for cell in cells]
            total_weight = sum(weights)
            forward_m = sum(
                cell["forward_m"] * weight
                for cell, weight in zip(cells, weights)
            ) / total_weight
            lateral_m = sum(
                cell["lateral_m"] * weight
                for cell, weight in zip(cells, weights)
            ) / total_weight
            clusters.append(
                {
                    "forward_m": forward_m,
                    "lateral_m": lateral_m,
                    "range_uncertainty_m": heatmap.range_step_m,
                    "strength_confidence": max(
                        0.05,
                        min(1.0, peak_residual_db / 24.0),
                    ),
                    "cell_count": float(len(cells)),
                    "peak_residual_db": peak_residual_db,
                }
            )
        clusters.sort(
            key=lambda cluster: (
                float(cluster["peak_residual_db"]),
                float(cluster["cell_count"]),
            ),
            reverse=True,
        )
        reflectivity_observations = [
            (
                float(cell["forward_m"]),
                float(cell["lateral_m"]),
                min(1.0, max(0.0, float(cell["residual_db"]) / 24.0)),
            )
            for cell in candidates.values()
        ]
        return (
            clusters[:HEATMAP_MAX_CLUSTERS_PER_FRAME],
            len(candidates),
            reflectivity_observations,
        )

    def _heatmap_cell_position(
        self,
        range_m: float,
        azimuth_index: int,
        azimuth_bins: int,
    ) -> Tuple[float, float]:
        angle_argument = (
            2.0 * (azimuth_index - azimuth_bins / 2.0) / azimuth_bins
        )
        angle_rad = math.asin(
            max(-1.0, min(1.0, angle_argument))
        )
        angle_rad = math.radians(
            max(-70.0, min(70.0, math.degrees(angle_rad)))
        )
        return (
            range_m * math.cos(angle_rad),
            range_m * math.sin(angle_rad) * self.axes.lateral_sign,
        )

    def _near_existing_heatmap_track(
        self,
        forward_m: float,
        lateral_m: float,
    ) -> bool:
        return any(
            track["source"] == "heatmap"
            and math.hypot(
                float(track["forward_m"]) - forward_m,
                float(track["lateral_m"]) - lateral_m,
            ) <= HEATMAP_TRACK_ASSOCIATION_M
            for track in self._tracks
        )

    def _binding_status(self, frame: RadarFrame) -> str:
        if self.synthetic:
            return "synthetic"
        if frame.source_format == "ti-fccsp-spi-raw-bpm-v1":
            return "spi_raw"
        if self.clutter_model is None:
            return (
                "calibration_required"
                if self.require_calibration
                else "calibration_unavailable"
            )
        if (
            frame.profile_id != self.clutter_model.profile_id
            or self.axes.signature()
            != self.clutter_model.axes.signature()
        ):
            return "profile_mismatch"
        if frame.header.calibration_id != self.clutter_model.calibration_id:
            return "calibration_mismatch"
        if frame.heatmap is None:
            return "ok"
        return self.clutter_model.binding_status(
            frame.profile_id,
            frame.heatmap,
            self.axes,
        )

    @staticmethod
    def _surface_contour_observations(
        cells: Sequence[Dict[str, object]],
    ) -> List[Dict[str, object]]:
        """Extract every directional radial ridge from the raw SPI image.

        The old implementation selected the globally strongest cell on each
        azimuth ray.  Range compensation then made the end of the 30 cm view
        win on most rays, and the contour fitter connected that clipped arc
        into two walls.  A radar surface is instead a *local* maximum in
        range.  ``relative_db`` has the per-range common mode removed, so a
        broad coupling ring or range gain cannot become directional geometry.
        All positive local ridges remain available; line continuity and time,
        rather than a scene-specific amplitude cutoff, decide which become a
        displayed wall.
        """

        grouped: Dict[int, List[Dict[str, object]]] = {}
        for cell in cells:
            try:
                azimuth_index = int(cell["azimuth_index"])
                range_m = float(cell["range_m"])
                measured_db = float(cell.get("measured_db", float("nan")))
                relative_db = float(cell["relative_db"])
                intensity = float(cell["intensity"])
                range_index = int(cell["range_index"])
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if not (
                0.10 <= range_m <= PATH_LOCAL_MAX_RANGE_M
                and math.isfinite(intensity)
                and math.isfinite(measured_db)
                and math.isfinite(relative_db)
                and range_index >= 0
            ):
                continue
            grouped.setdefault(azimuth_index, []).append(cell)

        observations: List[Dict[str, object]] = []
        for azimuth_index, ray in sorted(grouped.items()):
            ordered = sorted(ray, key=lambda cell: int(cell["range_index"]))
            for index, ridge in enumerate(ordered):
                # Maxima at either edge are censored: the actual peak can lie
                # inside the rejected coupling band or beyond the operator
                # view.  Without a neighbour on both sides they are not a
                # measurable radial ridge and must not become wall geometry.
                if index == 0 or index + 1 >= len(ordered):
                    continue
                value_db = float(ridge["relative_db"])
                left_db = float(ordered[index - 1]["relative_db"])
                right_db = float(ordered[index + 1]["relative_db"])
                if value_db <= 0.0 or value_db < right_db:
                    continue
                if value_db < left_db:
                    continue
                if not (
                    value_db > right_db
                    or value_db > left_db
                ):
                    continue
                local_floor_db = max(
                    right_db,
                    left_db,
                )
                prominence_db = value_db - local_floor_db
                if prominence_db <= 1e-9:
                    continue
                ridge_intensity = 0.30 + 0.70 * (
                    1.0 - math.exp(-prominence_db / 3.0)
                )
                observations.append(
                    {
                        "forward_m": float(ridge["forward_m"]),
                        "lateral_m": float(ridge["lateral_m"]),
                        "intensity": ridge_intensity,
                        "prominence_db": prominence_db,
                        "residual_db": value_db,
                        "azimuth_index": azimuth_index,
                        "range_index": int(ridge["range_index"]),
                    }
                )
        return observations

    @staticmethod
    def _in_scene(forward_m: float, lateral_m: float) -> bool:
        return (
            0.0 <= forward_m <= SCENE_MAX_FORWARD_M
            and -SCENE_HALF_WIDTH_M
            <= lateral_m
            < SCENE_HALF_WIDTH_M
        )

    @staticmethod
    def _point_confidence(snr_db: Optional[float]) -> float:
        if snr_db is None:
            return 0.75
        return max(0.05, min(1.0, snr_db / 30.0))

    def _upsert_track(
        self,
        *,
        forward_m: float,
        lateral_m: float,
        height_m: Optional[float],
        range_uncertainty_m: float,
        confidence: float,
        source: str,
        received_at: float,
        lateral_uncertainty_m: float = 0.05,
        snr_db: Optional[float] = None,
        noise_db: Optional[float] = None,
        quality_score: Optional[float] = None,
        point_correlated: bool = False,
        cluster_cells: int = 0,
        peak_residual_db: float = 0.0,
    ) -> None:
        association_candidates = [
            track
            for track in self._tracks
            if track["source"] == source
            and (
                not bool(track.get("_updated_by_point"))
                if source == "point"
                else not bool(track.get("_updated_by_heatmap"))
            )
        ]
        if source == "point":
            association_candidates = [
                track for track in association_candidates
                if point_association_gate(
                    float(track["forward_m"]),
                    float(track["lateral_m"]),
                    forward_m,
                    lateral_m,
                    range_uncertainty_m * 2.0,
                )
            ]
        nearest = min(
            association_candidates,
            key=lambda track: math.hypot(
                float(track["forward_m"]) - forward_m,
                float(track["lateral_m"]) - lateral_m,
            ),
            default=None,
        )
        if nearest is not None and (
            source == "point" or math.hypot(
                float(nearest["forward_m"]) - forward_m,
                float(nearest["lateral_m"]) - lateral_m,
            ) <= HEATMAP_TRACK_ASSOCIATION_M
        ):
            if source == "point":
                samples = nearest["_point_samples"]
                assert isinstance(samples, list)
                samples.append((forward_m, lateral_m, height_m))
                del samples[:-POINT_HISTORY_FRAMES]
                estimate = robust_point_estimate(
                    samples,
                    residual_limit_m=POINT_POSITION_RESIDUAL_LIMIT_M,
                )
                nearest["position_jitter_m"] = round(
                    estimate.position_jitter_m, 4
                )
                nearest["outlier_rejections"] = estimate.rejected_count
                if not estimate.latest_inlier:
                    return
                nearest["forward_m"] = estimate.forward_m
                nearest["lateral_m"] = estimate.lateral_m
                nearest["height_m"] = estimate.height_m
                nearest["height_band"] = height_band(estimate.height_m)
                nearest["range_uncertainty_m"] = range_uncertainty_m
                nearest["lateral_uncertainty_m"] = lateral_uncertainty_m
                nearest["confidence"] = confidence
                nearest["snr_db"] = snr_db
                nearest["noise_db"] = noise_db
                nearest["quality_score"] = quality_score
            else:
                smoothing = 0.35
                nearest["forward_m"] = (
                    float(nearest["forward_m"]) * (1.0 - smoothing)
                    + forward_m * smoothing
                )
                nearest["lateral_m"] = (
                    float(nearest["lateral_m"]) * (1.0 - smoothing)
                    + lateral_m * smoothing
                )
                nearest["range_uncertainty_m"] = max(
                    float(nearest["range_uncertainty_m"]),
                    range_uncertainty_m,
                )
                position_history = nearest["_position_history"]
                assert isinstance(position_history, list)
                position_history.append((forward_m, lateral_m))
                del position_history[:-HEATMAP_HISTORY_FRAMES]
                heatmap_hits = nearest["_heatmap_hits"]
                point_hits = nearest["_point_association_hits"]
                assert isinstance(heatmap_hits, list)
                assert isinstance(point_hits, list)
                heatmap_hits[-1] = True
                point_hits[-1] = point_correlated
                nearest["cluster_cells"] = cluster_cells
                nearest["peak_residual_db"] = round(
                    peak_residual_db,
                    3,
                )
                nearest["_strength_confidence"] = confidence
                nearest["_updated_by_heatmap"] = True
                self._update_heatmap_track_state(nearest)
            nearest["distance_m"] = math.hypot(
                float(nearest["forward_m"]),
                float(nearest["lateral_m"]),
            )
            if source == "point":
                nearest["hazard_distance_m"] = math.hypot(
                    forward_m, lateral_m
                )
                nearest["_hazard_forward_m"] = forward_m
                nearest["_hazard_lateral_m"] = lateral_m
            nearest["_received_at_ns"] = self._seconds_to_ns(received_at)
            if source == "point":
                hits = nearest["_point_hits"]
                assert isinstance(hits, list)
                hits[-1] = True
                strong_hits = nearest["_point_strong_hits"]
                assert isinstance(strong_hits, list)
                strong_hits[-1] = bool(
                    quality_score is not None
                    and quality_score >= POINT_STRONG_QUALITY
                )
                nearest["point_confirmed"] = (
                    sum(hits[-POINT_HAZARD_WINDOW_FRAMES:])
                    >= POINT_HAZARD_CONFIRM_HITS
                )
                nearest["map_confirmed"] = (
                    sum(strong_hits[-POINT_HISTORY_FRAMES:]) >= 3
                )
                nearest["classification"] = (
                    "confirmed_obstacle"
                    if nearest["point_confirmed"]
                    else "point_candidate"
                )
                nearest["_updated_by_point"] = True
            return

        point_hit = source == "point"
        estimate = (
            robust_point_estimate(
                [(forward_m, lateral_m, height_m)],
                residual_limit_m=POINT_POSITION_RESIDUAL_LIMIT_M,
            )
            if point_hit
            else None
        )
        track: Dict[str, object] = {
            "track_id": self._next_track_id,
            "forward_m": forward_m,
            "lateral_m": lateral_m,
            "height_m": estimate.height_m if estimate is not None else height_m,
            "height_band": (
                height_band(estimate.height_m)
                if estimate is not None
                else "unknown"
            ),
            "height_reference": (
                "sensor_relative_uncalibrated"
                if point_hit
                else "unavailable"
            ),
            "distance_m": math.hypot(forward_m, lateral_m),
            "range_uncertainty_m": range_uncertainty_m,
            "lateral_uncertainty_m": lateral_uncertainty_m,
            "confidence": confidence,
            "snr_db": snr_db,
            "noise_db": noise_db,
            "quality_score": quality_score,
            "position_jitter_m": (
                round(estimate.position_jitter_m, 4)
                if estimate is not None
                else None
            ),
            "outlier_rejections": 0,
            "source": source,
            "point_confirmed": False,
            "classification": (
                "point_candidate"
                if source == "point"
                else "reflection_evidence"
            ),
            "age_ms": 0,
            "_received_at_ns": self._seconds_to_ns(received_at),
            "_point_hits": [point_hit],
            "_point_strong_hits": [
                bool(
                    point_hit
                    and quality_score is not None
                    and quality_score >= POINT_STRONG_QUALITY
                )
            ],
            "_point_samples": (
                [(forward_m, lateral_m, height_m)] if point_hit else []
            ),
            "_hazard_forward_m": forward_m if point_hit else None,
            "_hazard_lateral_m": lateral_m if point_hit else None,
            "map_confirmed": False,
            "_updated_by_point": point_hit,
            "_updated_by_heatmap": source == "heatmap",
        }
        if point_hit:
            track["hazard_distance_m"] = math.hypot(forward_m, lateral_m)
        if source == "heatmap":
            track.update(
                {
                    "heatmap_confirmed": False,
                    "point_correlated": point_correlated,
                    "position_stable": False,
                    "position_jitter_m": None,
                    "persistence": 1.0 / HEATMAP_HISTORY_FRAMES,
                    "cluster_cells": cluster_cells,
                    "peak_residual_db": round(peak_residual_db, 3),
                    "_heatmap_hits": [True],
                    "_point_association_hits": [point_correlated],
                    "_position_history": [(forward_m, lateral_m)],
                    "_strength_confidence": confidence,
                }
            )
            self._update_heatmap_track_state(track)
        self._tracks.append(track)
        self._next_track_id += 1

    @staticmethod
    def _update_heatmap_track_state(track: Dict[str, object]) -> None:
        heatmap_hits = track["_heatmap_hits"]
        point_hits = track["_point_association_hits"]
        position_history = track["_position_history"]
        assert isinstance(heatmap_hits, list)
        assert isinstance(point_hits, list)
        assert isinstance(position_history, list)
        persistence = sum(bool(hit) for hit in heatmap_hits) / max(
            HEATMAP_HISTORY_FRAMES,
            len(heatmap_hits),
        )
        center_forward = sum(point[0] for point in position_history) / len(
            position_history
        )
        center_lateral = sum(point[1] for point in position_history) / len(
            position_history
        )
        jitter_m = max(
            math.hypot(
                point[0] - center_forward,
                point[1] - center_lateral,
            )
            for point in position_history
        )
        repeated = sum(bool(hit) for hit in heatmap_hits) >= HEATMAP_CONFIRM_HITS
        point_correlated = (
            sum(bool(hit) for hit in point_hits)
            >= HEATMAP_POINT_CONFIRM_HITS
        )
        position_stable = (
            len(position_history) >= HEATMAP_CONFIRM_HITS
            and jitter_m <= HEATMAP_MAX_POSITION_JITTER_M
        )
        confirmed = repeated and point_correlated and position_stable
        strength = float(track.get("_strength_confidence", 0.0))
        stability_score = max(
            0.0,
            1.0 - jitter_m / HEATMAP_MAX_POSITION_JITTER_M,
        )
        confidence = (
            strength * 0.35
            + persistence * 0.25
            + stability_score * 0.15
            + (0.25 if point_correlated else 0.0)
        )
        if not point_correlated:
            confidence = min(confidence, 0.55)
        track["persistence"] = round(persistence, 3)
        track["position_jitter_m"] = round(jitter_m, 4)
        track["position_stable"] = position_stable
        track["point_correlated"] = point_correlated
        track["heatmap_confirmed"] = confirmed
        track["classification"] = (
            "confirmed_obstacle" if confirmed else "reflection_evidence"
        )
        track["confidence"] = round(max(0.05, min(1.0, confidence)), 3)

    def _expire_tracks(self, now: float) -> None:
        now_ns = self._seconds_to_ns(now)
        self._tracks = [
            track
            for track in self._tracks
            if (
                now_ns - int(track["_received_at_ns"])
                < TRACK_TTL_NS
            )
        ]

    @staticmethod
    def _seconds_to_ns(value: float) -> int:
        return int(round(value * 1_000_000_000))
