"""Continuous evidence from the complete range-azimuth heatmap.

The TI heatmap is an image, not a point cloud.  Every cell inside the trusted
field of view is retained here.  Signal magnitude changes how strongly a cell
is drawn and how much it contributes to occupancy; it never decides whether
the measurement exists.
"""

from __future__ import annotations

from collections import deque
import math
import statistics
from typing import Deque, Dict, Optional, Tuple

from common.sensor_contract import RadarHeatmap
from sensors.radar_calibration import RadarClutterModel


GridKey = Tuple[int, int]
FrameCells = Dict[GridKey, Dict[str, float]]


def _rounded(value: float) -> float:
    return round(float(value), 6)


class RadarReflectivitySurface:
    """Short-memory, threshold-free view of a range-azimuth heatmap."""

    ANGULAR_RESOLUTION_DEG = 29.0
    TRUSTED_AZIMUTH_DEG = 60.0

    def __init__(
        self,
        *,
        radius_m: float = 0.30,
        history_frames: int = 5,
        required_hits: int = 3,
        clutter_model: Optional[RadarClutterModel] = None,
        valid_min_range_m: float = 0.0,
    ) -> None:
        self.radius_m = float(radius_m)
        self.history_frames = int(history_frames)
        self.required_hits = int(required_hits)
        self.clutter_model = clutter_model
        self.valid_min_range_m = float(valid_min_range_m)
        if (
            not math.isfinite(self.radius_m)
            or self.radius_m <= 0.0
            or self.history_frames < 1
            or not 1 <= self.required_hits <= self.history_frames
            or not math.isfinite(self.valid_min_range_m)
            or self.valid_min_range_m < 0.0
            or self.valid_min_range_m >= self.radius_m
        ):
            raise ValueError("invalid reflectivity surface geometry")
        self._history: Deque[Tuple[float, FrameCells]] = deque(
            maxlen=self.history_frames
        )
        self._range_step_m: float | None = None
        self._last_calibrated = False

    def _calibration_matches(self, heatmap: RadarHeatmap) -> bool:
        model = self.clutter_model
        return bool(
            model is not None
            and model.range_bins == heatmap.range_bins
            and model.azimuth_bins == heatmap.azimuth_bins
            and math.isclose(
                model.range_step_m,
                heatmap.range_step_m,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            and model.motion_mode == heatmap.motion_mode
        )

    @classmethod
    def _physical_azimuth_smooth(
        cls,
        values: Dict[int, float],
        azimuth_bins: int,
    ) -> Dict[int, float]:
        """Do not treat zero-padded FFT bins as independent radar beams."""
        sigma_deg = cls.ANGULAR_RESOLUTION_DEG / 2.355
        result: Dict[int, float] = {}
        angles = {
            index: cls._angle_deg(index, azimuth_bins)
            for index in values
        }
        for index, angle_deg in angles.items():
            weighted_sum = 0.0
            total_weight = 0.0
            for other_index, other_angle_deg in angles.items():
                delta = (other_angle_deg - angle_deg) / sigma_deg
                weight = math.exp(-0.5 * delta * delta)
                weighted_sum += values[other_index] * weight
                total_weight += weight
            result[index] = weighted_sum / total_weight
        return result

    @staticmethod
    def _time(value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("time must be a real number")
        result = float(value)
        if not math.isfinite(result):
            raise ValueError("time must be finite")
        return result

    @staticmethod
    def _angle_deg(azimuth_index: int, azimuth_bins: int) -> float:
        argument = 2.0 * (
            azimuth_index - azimuth_bins / 2.0
        ) / azimuth_bins
        return math.degrees(math.asin(max(-1.0, min(1.0, argument))))

    @staticmethod
    def _logistic(value: float) -> float:
        limited = max(-12.0, min(12.0, value))
        return 1.0 / (1.0 + math.exp(-limited))

    def reset(self) -> None:
        self._history.clear()
        self._range_step_m = None
        self._last_calibrated = False

    def ingest_heatmap(
        self,
        heatmap: RadarHeatmap,
        received_at: float,
        *,
        lateral_sign: int,
        elapsed_frame_windows: int = 1,
    ) -> None:
        if not isinstance(heatmap, RadarHeatmap):
            raise TypeError("heatmap must be a RadarHeatmap")
        timestamp = self._time(received_at)
        if lateral_sign not in (-1, 1) or isinstance(lateral_sign, bool):
            raise ValueError("lateral_sign must be -1 or 1")
        if isinstance(elapsed_frame_windows, bool) or not isinstance(
            elapsed_frame_windows, int
        ):
            raise TypeError("elapsed_frame_windows must be an integer")
        if elapsed_frame_windows < 1:
            raise ValueError("elapsed_frame_windows must be positive")
        if self._history and timestamp < self._history[-1][0]:
            raise ValueError("received_at must not move backwards")

        for _ in range(min(
            elapsed_frame_windows - 1,
            self.history_frames,
        )):
            self._history.append((timestamp, {}))

        self._range_step_m = heatmap.range_step_m
        rows: Dict[int, Dict[int, float]] = {}
        maximum_range_index = min(
            heatmap.range_bins - 1,
            int(math.floor(self.radius_m / heatmap.range_step_m + 1e-12)),
        )
        span_db = heatmap.ceiling_db - heatmap.floor_db
        minimum_range_index = max(
            1,
            int(math.ceil(
                self.valid_min_range_m / heatmap.range_step_m - 1e-12
            )),
        )
        for range_index in range(
            minimum_range_index, maximum_range_index + 1
        ):
            row: Dict[int, float] = {}
            for azimuth_index in range(heatmap.azimuth_bins):
                angle_deg = self._angle_deg(
                    azimuth_index, heatmap.azimuth_bins
                )
                if abs(angle_deg) > self.TRUSTED_AZIMUTH_DEG + 1e-12:
                    continue
                flat_index = (
                    range_index * heatmap.azimuth_bins + azimuth_index
                )
                row[azimuth_index] = (
                    heatmap.floor_db
                    + heatmap.data[flat_index] / 255.0 * span_db
                )
            if row:
                rows[range_index] = row

        calibrated = self._calibration_matches(heatmap)
        self._last_calibrated = calibrated
        if calibrated:
            calibrated_rows: Dict[int, Dict[int, float]] = {}
            for range_index, row in rows.items():
                # Use calibration only to bind the board/profile geometry.
                # Never subtract a stored room image: a wall present during
                # calibration would otherwise be erased (or reappear after it
                # moved).  The current row median removes receiver/range gain
                # while retaining every current angular measurement.
                common_mode_db = statistics.median(row.values())
                calibrated_rows[range_index] = self._physical_azimuth_smooth(
                    {
                        azimuth_index: observed_db - common_mode_db
                        for azimuth_index, observed_db in row.items()
                    },
                    heatmap.azimuth_bins,
                )
            relative = {
                (range_index, azimuth_index): residual_db
                for range_index, row in calibrated_rows.items()
                for azimuth_index, residual_db in row.items()
            }
            relative_median = 0.0
            # TI heatmap dB values are relative units.  Use a conservative
            # noise scale instead of tiny calibration MAD values that would
            # turn quantization into certain occupancy.
            robust_scale_db = 1.5
        else:
            # Calibration-free evidence is still shown for diagnostics, but
            # is not authorized by RadarSceneEstimator for path decisions.
            relative = {}

        # Median polish is the compatibility fallback when calibration is
        # absent.  No amplitude threshold removes measurements in either path.
        row_centred: Dict[GridKey, float] = {}
        for range_index, row in rows.items():
            row_median = statistics.median(row.values())
            for azimuth_index, observed_db in row.items():
                row_centred[(range_index, azimuth_index)] = (
                    observed_db - row_median
                )
        angle_bias: Dict[int, float] = {}
        for azimuth_index in {
            index for _, index in row_centred
        }:
            angle_bias[azimuth_index] = statistics.median(
                value
                for (range_index, index), value in row_centred.items()
                if index == azimuth_index
            )
        if not calibrated:
            relative = {
                key: value - angle_bias[key[1]]
                for key, value in row_centred.items()
            }
            relative_median = (
                statistics.median(relative.values()) if relative else 0.0
            )
            relative_mad = (
                statistics.median(
                    abs(value - relative_median)
                    for value in relative.values()
                )
                if relative else 0.0
            )
            robust_scale_db = max(1.0, 1.4826 * relative_mad)

        cells: FrameCells = {}
        for (range_index, azimuth_index), relative_db in relative.items():
            range_m = range_index * heatmap.range_step_m
            azimuth_deg = self._angle_deg(
                azimuth_index, heatmap.azimuth_bins
            )
            azimuth_rad = math.radians(azimuth_deg)
            forward_m = range_m * math.cos(azimuth_rad)
            lateral_m = range_m * math.sin(azimuth_rad) * lateral_sign
            cell = {
                "range_m": range_m,
                "azimuth_deg": azimuth_deg * lateral_sign,
                "forward_m": forward_m,
                "lateral_m": lateral_m,
                "intensity": self._logistic(
                    (relative_db - relative_median) / robust_scale_db
                ),
                "measured_db": rows[range_index][azimuth_index],
                "relative_db": relative_db,
                "calibrated_residual_db": relative_db,
            }
            if calibrated and self.clutter_model is not None:
                flat_index = (
                    range_index * heatmap.azimuth_bins + azimuth_index
                )
                baseline_delta_db = (
                    rows[range_index][azimuth_index]
                    - self.clutter_model.heatmap_median_db[flat_index]
                )
                baseline_scale_db = max(
                    1.5,
                    1.4826
                    * self.clutter_model.heatmap_mad_db[flat_index],
                )
                cell["baseline_delta_db"] = baseline_delta_db
                # The old room image never changes geometry.  Its absolute
                # difference is only a weak reliability cue: both a new
                # reflector and a removed reflector are meaningful changes.
                cell["scene_change_score"] = 1.0 - math.exp(
                    -abs(baseline_delta_db) / baseline_scale_db
                )
            cells[(range_index, azimuth_index)] = cell
        self._history.append((timestamp, cells))

    def payload(self, now: float) -> dict[str, object]:
        timestamp = self._time(now)
        combined: Dict[GridKey, list[Tuple[float, Dict[str, float]]]] = {}
        for observed_at, cells in self._history:
            for key, cell in cells.items():
                combined.setdefault(key, []).append((observed_at, cell))

        cells_payload = []
        for key in sorted(combined):
            observations = combined[key]
            last_seen = max(observed_at for observed_at, _ in observations)
            if timestamp < last_seen:
                raise ValueError("now must not precede observed cells")
            latest = max(observations, key=lambda item: item[0])[1]
            intensities = [cell["intensity"] for _, cell in observations]
            relative_values = [
                cell["relative_db"] for _, cell in observations
            ]
            measured_values = [
                cell["measured_db"] for _, cell in observations
            ]
            scene_change_values = [
                cell.get("scene_change_score", 1.0)
                for _, cell in observations
            ]
            baseline_delta_values = [
                cell["baseline_delta_db"]
                for _, cell in observations
                if "baseline_delta_db" in cell
            ]
            intensity = statistics.median(intensities)
            relative_db = statistics.median(relative_values)
            variation_db = statistics.median(
                abs(value - relative_db) for value in relative_values
            )
            hits = len(observations)
            temporal_coverage = min(1.0, hits / self.required_hits)
            temporal_stability = math.exp(-variation_db / 3.0)
            confidence = temporal_coverage * (
                0.65 + 0.35 * temporal_stability
            )
            range_m = latest["range_m"]
            payload_cell = {
                "range_index": key[0],
                "azimuth_index": key[1],
                "range_m": _rounded(range_m),
                "azimuth_deg": _rounded(latest["azimuth_deg"]),
                "forward_m": _rounded(latest["forward_m"]),
                "lateral_m": _rounded(latest["lateral_m"]),
                "intensity": _rounded(intensity),
                "confidence": _rounded(confidence),
                "persistence": round(hits / self.history_frames, 3),
                "hits": hits,
                "verified": hits >= self.required_hits,
                "age_ms": max(
                    0, round((timestamp - last_seen) * 1000.0)
                ),
                "relative_db": _rounded(relative_db),
                "measured_db": _rounded(
                    statistics.median(measured_values)
                ),
                "calibrated_residual_db": _rounded(relative_db),
                "range_uncertainty_m": _rounded(
                    (self._range_step_m or 0.0) / 2.0
                ),
                "angular_uncertainty_deg": (
                    self.ANGULAR_RESOLUTION_DEG / 2.0
                ),
                "scene_change_score": _rounded(
                    statistics.median(scene_change_values)
                ),
            }
            if baseline_delta_values:
                payload_cell["baseline_delta_db"] = _rounded(
                    statistics.median(baseline_delta_values)
                )
            cells_payload.append(payload_cell)
        return {
            "radius_m": self.radius_m,
            "range_step_m": (
                None if self._range_step_m is None
                else _rounded(self._range_step_m)
            ),
            "angular_resolution_deg": self.ANGULAR_RESOLUTION_DEG,
            "history_frames": self.history_frames,
            "required_hits": self.required_hits,
            "retention": "continuous_five_frame_evidence",
            "pose_mode": "sensor_relative_short_memory",
            "source": "full_range_azimuth_heatmap",
            "normalization": (
                "current_range_common_mode_physical_beam"
                if self._last_calibrated
                else "per_frame_median_polish"
            ),
            "calibrated": self._last_calibrated,
            "room_baseline_used": False,
            "valid_min_range_m": self.valid_min_range_m,
            "independent_beam_count": math.ceil(
                2.0 * self.TRUSTED_AZIMUTH_DEG
                / self.ANGULAR_RESOLUTION_DEG
            ),
            "cells": cells_payload,
        }
