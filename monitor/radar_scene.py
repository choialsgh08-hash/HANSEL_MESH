"""Calibrated, robot-relative radar scene evidence."""

from __future__ import annotations

import base64
import math
import time
from typing import Callable, Dict, List, Optional

from common.radar_geometry import RadarAxes
from common.sensor_contract import RadarFrame
from sensors.radar_calibration import RadarClutterModel


SCENE_SCHEMA_VERSION = 1
GRID_RESOLUTION_M = 0.05
GRID_FORWARD_CELLS = 60
GRID_LATERAL_CELLS = 60
GRID_ORIGIN_FORWARD_CELL = 0
GRID_ORIGIN_LATERAL_CELL = 30
SCENE_MAX_FORWARD_M = 3.0
SCENE_HALF_WIDTH_M = 1.5
TRACK_ASSOCIATION_M = 0.18
TRACK_TTL_S = 0.300
HEATMAP_MIN_RESIDUAL_DB = 6.0
HEATMAP_MAD_MULTIPLIER = 4.0
DANGER_ENTER_M = 0.10
DANGER_EXIT_M = 0.13
FAULT_SOURCE_STATUSES = {
    "waiting",
    "stale",
    "fault",
    "replay_end",
    "replay_loop_restart",
}


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
        if calibration_status != "ok":
            self.reset()
            return self._result(
                "UNKNOWN",
                None,
                calibration_status,
            )

        fresh_tracks = [
            track
            for track in tracks
            if int(track.get("age_ms", 0)) < TRACK_TTL_S * 1000
        ]
        by_id = {
            int(track["track_id"]): track
            for track in fresh_tracks
        }
        if self._danger_track_id is not None:
            latched = by_id.get(self._danger_track_id)
            if (
                latched is not None
                and float(latched["distance_m"]) < DANGER_EXIT_M
            ):
                return self._result(
                    "DANGER",
                    float(latched["distance_m"]),
                    "confirmed_point_inside_threshold",
                )
            self._danger_track_id = None

        confirmed = [
            track
            for track in fresh_tracks
            if bool(track["point_confirmed"])
        ]
        entering = [
            track
            for track in confirmed
            if float(track["distance_m"]) <= DANGER_ENTER_M
        ]
        if entering:
            nearest = min(
                entering,
                key=lambda track: float(track["distance_m"]),
            )
            self._danger_track_id = int(nearest["track_id"])
            return self._result(
                "DANGER",
                float(nearest["distance_m"]),
                "confirmed_point_inside_threshold",
            )
        if confirmed:
            nearest_m = min(
                float(track["distance_m"]) for track in confirmed
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
    ) -> None:
        self.axes = axes
        self.clutter_model = clutter_model
        self.require_calibration = require_calibration
        self._clock = clock
        self._tracks: List[Dict[str, object]] = []
        self._next_track_id = 1
        self._calibration_status = (
            "calibration_required"
            if require_calibration and clutter_model is None
            else "ok"
        )
        self._scene_point_count = 0
        self._clutter_points_rejected = 0
        self._heatmap_cells_accepted = 0
        self._hazard_evaluator = RadarHazardEvaluator()
        self._producer_id: Optional[str] = None
        self._last_reset_reason: Optional[str] = None

    def reset(self, reason: str) -> None:
        self._tracks.clear()
        self._next_track_id = 1
        self._hazard_evaluator.reset()
        self._producer_id = None
        self._scene_point_count = 0
        self._clutter_points_rejected = 0
        self._heatmap_cells_accepted = 0
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
        self._expire_tracks(now)
        elapsed_frame_windows = min(
            3,
            frame.dropped_frames_since_previous + 1,
        )
        for track in self._tracks:
            hits = track["_point_hits"]
            assert isinstance(hits, list)
            hits.extend([False] * elapsed_frame_windows)
            del hits[:-3]
            track["point_confirmed"] = sum(hits) >= 2
            track["_updated_by_point"] = False
        self._calibration_status = self._binding_status(frame)
        point_count = 0
        rejected = 0
        heatmap_count = 0

        for point in frame.points:
            forward_m, lateral_m = self.axes.map_point(point)
            if not self._in_scene(forward_m, lateral_m):
                continue
            if (
                self.clutter_model is not None
                and self._calibration_status == "ok"
                and self.clutter_model.matches_point(
                    forward_m,
                    lateral_m,
                    point.z_m,
                )
            ):
                rejected += 1
                continue
            self._upsert_track(
                forward_m=forward_m,
                lateral_m=lateral_m,
                height_m=point.z_m,
                range_uncertainty_m=0.0,
                confidence=self._point_confidence(point.snr_db),
                source="point",
                received_at=now,
            )
            point_count += 1

        if (
            frame.heatmap is not None
            and self.clutter_model is not None
            and self._calibration_status == "ok"
            and self.axes.signature() == RadarAxes().signature()
        ):
            heatmap = frame.heatmap
            model = self.clutter_model
            assert model is not None
            span_db = heatmap.ceiling_db - heatmap.floor_db
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
                    threshold_db = max(
                        HEATMAP_MIN_RESIDUAL_DB,
                        HEATMAP_MAD_MULTIPLIER
                        * model.heatmap_mad_db[flat_index],
                    )
                    if residual_db < threshold_db:
                        continue
                    angle_argument = (
                        2.0
                        * (
                            azimuth_index
                            - heatmap.azimuth_bins / 2.0
                        )
                        / heatmap.azimuth_bins
                    )
                    angle_rad = math.asin(
                        max(-1.0, min(1.0, angle_argument))
                    )
                    angle_rad = math.radians(
                        max(-70.0, min(70.0, math.degrees(angle_rad)))
                    )
                    forward_m = range_m * math.cos(angle_rad)
                    lateral_m = range_m * math.sin(angle_rad)
                    if not self._in_scene(forward_m, lateral_m):
                        continue
                    self._upsert_track(
                        forward_m=forward_m,
                        lateral_m=lateral_m,
                        height_m=None,
                        range_uncertainty_m=heatmap.range_step_m,
                        confidence=min(1.0, residual_db / 20.0),
                        source="heatmap",
                        received_at=now,
                    )
                    heatmap_count += 1

        self._scene_point_count = point_count
        self._clutter_points_rejected = rejected
        self._heatmap_cells_accepted = heatmap_count

    def snapshot(
        self,
        source_status: str = "live",
        now: Optional[float] = None,
    ) -> Dict[str, object]:
        observed_at = self._clock() if now is None else now
        if source_status in FAULT_SOURCE_STATUSES:
            self.reset(source_status)
        self._expire_tracks(observed_at)
        tracks = [
            {
                key: value
                for key, value in track.items()
                if not key.startswith("_")
            }
            | {
                "age_ms": max(
                    0,
                    int(
                        (observed_at - float(track["_received_at"]))
                        * 1000.0
                    ),
                )
            }
            for track in self._tracks
        ]
        grid = bytearray(GRID_FORWARD_CELLS * GRID_LATERAL_CELLS)
        for track in tracks:
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
            "calibration_status": self._calibration_status,
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
            "hazard": self._hazard_evaluator.update(
                tracks,
                self._calibration_status,
                source_status,
                observed_at,
            ),
            "diagnostics": {
                "scene_point_count": self._scene_point_count,
                "clutter_points_rejected": self._clutter_points_rejected,
                "heatmap_cells_accepted": self._heatmap_cells_accepted,
                "last_reset_reason": self._last_reset_reason,
            },
        }

    def _binding_status(self, frame: RadarFrame) -> str:
        if self.clutter_model is None:
            return (
                "calibration_required"
                if self.require_calibration
                else "ok"
            )
        if (
            frame.profile_id != self.clutter_model.profile_id
            or self.axes.signature()
            != self.clutter_model.axes.signature()
        ):
            return "profile_mismatch"
        if frame.heatmap is None:
            return "ok"
        return self.clutter_model.binding_status(
            frame.profile_id,
            frame.heatmap,
            self.axes,
        )

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
    ) -> None:
        association_candidates = [
            track
            for track in self._tracks
            if (
                source == "point"
                and not bool(track.get("_updated_by_point"))
            )
            or (
                source == "heatmap"
                and track["source"] == "heatmap"
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
        if (
            nearest is not None
            and math.hypot(
                float(nearest["forward_m"]) - forward_m,
                float(nearest["lateral_m"]) - lateral_m,
            )
            <= TRACK_ASSOCIATION_M
        ):
            if not (
                source == "heatmap"
                and bool(nearest.get("_updated_by_point"))
            ):
                nearest["forward_m"] = forward_m
                nearest["lateral_m"] = lateral_m
                nearest["distance_m"] = math.hypot(
                    forward_m,
                    lateral_m,
                )
                nearest["range_uncertainty_m"] = range_uncertainty_m
                nearest["confidence"] = confidence
                nearest["source"] = source
                if height_m is not None:
                    nearest["height_m"] = height_m
            nearest["_received_at"] = received_at
            if source == "point":
                hits = nearest["_point_hits"]
                assert isinstance(hits, list)
                hits[-1] = True
                nearest["point_confirmed"] = sum(hits) >= 2
                nearest["_updated_by_point"] = True
            return

        point_hit = source == "point"
        self._tracks.append(
            {
                "track_id": self._next_track_id,
                "forward_m": forward_m,
                "lateral_m": lateral_m,
                "height_m": height_m,
                "distance_m": math.hypot(forward_m, lateral_m),
                "range_uncertainty_m": range_uncertainty_m,
                "confidence": confidence,
                "source": source,
                "point_confirmed": False,
                "age_ms": 0,
                "_received_at": received_at,
                "_point_hits": [point_hit],
                "_updated_by_point": point_hit,
            }
        )
        self._next_track_id += 1

    def _expire_tracks(self, now: float) -> None:
        self._tracks = [
            track
            for track in self._tracks
            if now - float(track["_received_at"]) < TRACK_TTL_S
        ]
