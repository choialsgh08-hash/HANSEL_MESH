"""Temporal line segments from calibrated range-azimuth reflections."""

from __future__ import annotations

import math
import statistics
from collections import deque
from collections.abc import Mapping, Sequence
from typing import Deque, Dict, List, Optional, Tuple


MIN_SUPPORT_CELLS = 3
MIN_SPAN_M = 0.030
MAX_ORTHOGONAL_RESIDUAL_M = 0.015
MAX_ADJACENT_GAP_M = 0.045
COMPATIBLE_MIDPOINT_M = 0.05
COMPATIBLE_HEADING_DEG = 12.0
METHOD = "calibrated_nearest_reflection_segments"
MAX_RANGE_M = 0.30


Observation = Tuple[float, float, float, float, int, int]


def _rounded(value: float) -> float:
    return round(float(value), 6)


def _undirected_heading_delta(left: float, right: float) -> float:
    return abs((left - right + 90.0) % 180.0 - 90.0)


class CalibratedContourEstimator:
    """Fit arbitrary wall fragments without assuming a corridor pair."""

    def __init__(self, history_frames: int = 5, required_hits: int = 3):
        if isinstance(history_frames, bool) or not isinstance(history_frames, int):
            raise TypeError("history_frames must be an integer")
        if isinstance(required_hits, bool) or not isinstance(required_hits, int):
            raise TypeError("required_hits must be an integer")
        if history_frames < 1:
            raise ValueError("history_frames must be positive")
        if required_hits < 1 or required_hits > history_frames:
            raise ValueError("required_hits must fit inside history_frames")
        self._history_frames = history_frames
        self._required_hits = required_hits
        self._history: Deque[Tuple[Dict[str, object], ...]] = deque(
            maxlen=history_frames,
        )

    def reset(self) -> None:
        self._history.clear()

    def ingest(
        self,
        observations: Sequence[Mapping[str, object]],
        elapsed_frame_windows: int = 1,
    ) -> None:
        if (
            isinstance(elapsed_frame_windows, bool)
            or not isinstance(elapsed_frame_windows, int)
        ):
            raise TypeError("elapsed_frame_windows must be an integer")
        if elapsed_frame_windows < 1:
            raise ValueError("elapsed_frame_windows must be positive")
        for _ in range(min(self._history_frames, elapsed_frame_windows) - 1):
            self._history.append(())
        self._history.append(tuple(self._fit_segments(observations)))

    def payload(self) -> Dict[str, object]:
        seeds = [segment for frame in self._history for segment in frame]
        candidates: List[Tuple[Dict[str, object], List[Dict[str, object]]]] = []
        for seed in seeds:
            cluster = []
            for frame in self._history:
                compatible = [
                    segment for segment in frame
                    if self._compatible(segment, seed)
                ]
                if compatible:
                    cluster.append(min(
                        compatible,
                        key=lambda segment: self._midpoint_distance(
                            segment,
                            seed,
                        ),
                    ))
            if len(cluster) >= self._required_hits:
                candidates.append((seed, cluster))

        candidates.sort(key=lambda item: (
            -len(item[1]),
            -sum(float(value["mean_prominence_db"]) for value in item[1]),
            -sum(float(value["span_m"]) for value in item[1]),
        ))
        selected: List[Dict[str, object]] = []
        for seed, cluster in candidates:
            if any(self._compatible(seed, prior) for prior in selected):
                continue
            selected.append(self._serialize(cluster, len(selected) + 1))

        selected.sort(key=lambda segment: (
            min(segment["start"][0], segment["end"][0]),
            min(segment["start"][1], segment["end"][1]),
        ))
        for index, segment in enumerate(selected, start=1):
            segment["segment_id"] = f"contour_{index}"
        return {
            "status": "confirmed" if selected else "unavailable",
            "method": METHOD,
            "history_frames": self._history_frames,
            "required_hits": self._required_hits,
            "max_range_m": MAX_RANGE_M,
            "segments": selected,
        }

    @classmethod
    def _fit_segments(
        cls,
        observations: Sequence[Mapping[str, object]],
    ) -> List[Dict[str, object]]:
        unique: Dict[Tuple[int, int], Observation] = {}
        for value in observations:
            parsed = cls._parse_observation(value)
            if parsed is None:
                continue
            key = (parsed[4], parsed[5])
            existing = unique.get(key)
            if (
                existing is None
                or parsed[2] > existing[2]
            ):
                unique[key] = parsed

        # Keep simultaneous near and far radial ridges separate.  Selecting
        # only the nearest value at each angle erased legitimate wall pieces;
        # selecting only the strongest joined unrelated multipath.  Connected
        # components across adjacent angular samples preserve every coherent
        # surface while isolated bins still cannot form a line.
        points = sorted(
            unique.values(),
            key=lambda item: (item[4], item[5], -item[2]),
        )
        parents = list(range(len(points)))

        def find(index: int) -> int:
            while parents[index] != index:
                parents[index] = parents[parents[index]]
                index = parents[index]
            return index

        def union(left: int, right: int) -> None:
            left_root = find(left)
            right_root = find(right)
            if left_root != right_root:
                parents[right_root] = left_root

        by_azimuth: Dict[int, List[int]] = {}
        for point_index, point in enumerate(points):
            by_azimuth.setdefault(point[4], []).append(point_index)
        for azimuth_index in sorted(by_azimuth):
            next_indices = by_azimuth.get(azimuth_index + 1, ())
            for left_index in by_azimuth[azimuth_index]:
                left = points[left_index]
                for right_index in next_indices:
                    right = points[right_index]
                    if math.hypot(
                        left[0] - right[0],
                        left[1] - right[1],
                    ) <= MAX_ADJACENT_GAP_M + 1e-12:
                        union(left_index, right_index)

        components: Dict[int, List[Observation]] = {}
        for point_index, point in enumerate(points):
            components.setdefault(find(point_index), []).append(point)

        runs: List[List[Observation]] = []
        for component in components.values():
            component_by_azimuth: Dict[int, Observation] = {}
            for point in component:
                existing = component_by_azimuth.get(point[4])
                if (
                    existing is None
                    or point[2] > existing[2]
                    or (point[2] == existing[2] and point[5] < existing[5])
                ):
                    component_by_azimuth[point[4]] = point
            runs.append(sorted(
                component_by_azimuth.values(),
                key=lambda item: item[4],
            ))

        segments: List[Dict[str, object]] = []
        for run in runs:
            segments.extend(cls._split_run(run))
        return segments

    @staticmethod
    def _parse_observation(
        value: Mapping[str, object],
    ) -> Optional[Observation]:
        if not isinstance(value, Mapping):
            return None
        try:
            forward = float(value["forward_m"])
            lateral = float(value["lateral_m"])
            prominence = float(value["prominence_db"])
            residual = float(value["residual_db"])
            azimuth_index = value["azimuth_index"]
            range_index = value["range_index"]
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        if (
            isinstance(azimuth_index, bool)
            or not isinstance(azimuth_index, int)
            or isinstance(range_index, bool)
            or not isinstance(range_index, int)
            or azimuth_index < 0
            or range_index < 0
            or not all(math.isfinite(number) for number in (
                forward, lateral, prominence, residual,
            ))
            or forward < 0.0
            or prominence <= 0.0
        ):
            return None
        return (
            forward,
            lateral,
            prominence,
            residual,
            azimuth_index,
            range_index,
        )

    @classmethod
    def _split_run(cls, run: Sequence[Observation]) -> List[Dict[str, object]]:
        if len(run) < MIN_SUPPORT_CELLS:
            return []
        fitted = cls._fit_line(run)
        if (
            fitted is not None
            and float(fitted["max_residual_m"])
            <= MAX_ORTHOGONAL_RESIDUAL_M
        ):
            return [fitted] if float(fitted["span_m"]) >= MIN_SPAN_M else []
        best: Optional[Tuple[float, int]] = None
        for split in range(MIN_SUPPORT_CELLS, len(run) - MIN_SUPPORT_CELLS + 1):
            left = cls._fit_line(run[:split])
            right = cls._fit_line(run[split:])
            if left is None or right is None:
                continue
            score = max(
                float(left["max_residual_m"]),
                float(right["max_residual_m"]),
            )
            if best is None or score < best[0]:
                best = (score, split)
        if best is None:
            return []
        return [
            *cls._split_run(run[:best[1]]),
            *cls._split_run(run[best[1]:]),
        ]

    @staticmethod
    def _fit_line(run: Sequence[Observation]) -> Optional[Dict[str, object]]:
        if len(run) < MIN_SUPPORT_CELLS:
            return None
        mean_forward = statistics.fmean(point[0] for point in run)
        mean_lateral = statistics.fmean(point[1] for point in run)
        xx = sum((point[0] - mean_forward) ** 2 for point in run)
        yy = sum((point[1] - mean_lateral) ** 2 for point in run)
        xy = sum(
            (point[0] - mean_forward) * (point[1] - mean_lateral)
            for point in run
        )
        if xx + yy <= 1e-12:
            return None
        heading_rad = 0.5 * math.atan2(2.0 * xy, xx - yy)
        unit_forward = math.cos(heading_rad)
        unit_lateral = math.sin(heading_rad)
        projections = [
            (point[0] - mean_forward) * unit_forward
            + (point[1] - mean_lateral) * unit_lateral
            for point in run
        ]
        start_projection = min(projections)
        end_projection = max(projections)
        start = [
            mean_forward + start_projection * unit_forward,
            mean_lateral + start_projection * unit_lateral,
        ]
        end = [
            mean_forward + end_projection * unit_forward,
            mean_lateral + end_projection * unit_lateral,
        ]
        if (start[0], start[1]) > (end[0], end[1]):
            start, end = end, start
        heading_deg = math.degrees(math.atan2(
            end[1] - start[1],
            end[0] - start[0],
        ))
        residuals = [
            abs(
                -unit_lateral * (point[0] - mean_forward)
                + unit_forward * (point[1] - mean_lateral)
            )
            for point in run
        ]
        return {
            "heading_deg": heading_deg,
            "support_cells": len(run),
            "span_m": end_projection - start_projection,
            "max_residual_m": max(residuals),
            "mean_prominence_db": statistics.fmean(point[2] for point in run),
            "start": start,
            "end": end,
        }

    @staticmethod
    def _midpoint(segment: Mapping[str, object]) -> Tuple[float, float]:
        return (
            (float(segment["start"][0]) + float(segment["end"][0])) / 2.0,
            (float(segment["start"][1]) + float(segment["end"][1])) / 2.0,
        )

    @classmethod
    def _midpoint_distance(
        cls,
        left: Mapping[str, object],
        right: Mapping[str, object],
    ) -> float:
        left_midpoint = cls._midpoint(left)
        right_midpoint = cls._midpoint(right)
        return math.hypot(
            left_midpoint[0] - right_midpoint[0],
            left_midpoint[1] - right_midpoint[1],
        )

    @classmethod
    def _compatible(
        cls,
        left: Mapping[str, object],
        right: Mapping[str, object],
    ) -> bool:
        return (
            cls._midpoint_distance(left, right) <= COMPATIBLE_MIDPOINT_M
            and _undirected_heading_delta(
                float(left["heading_deg"]),
                float(right["heading_deg"]),
            ) <= COMPATIBLE_HEADING_DEG
        )

    def _serialize(
        self,
        cluster: Sequence[Mapping[str, object]],
        segment_index: int,
    ) -> Dict[str, object]:
        start = [
            statistics.median(float(value["start"][axis]) for value in cluster)
            for axis in (0, 1)
        ]
        end = [
            statistics.median(float(value["end"][axis]) for value in cluster)
            for axis in (0, 1)
        ]
        heading_deg = math.degrees(math.atan2(
            end[1] - start[1],
            end[0] - start[0],
        ))
        span_m = math.hypot(end[0] - start[0], end[1] - start[1])
        prominence = statistics.median(
            float(value["mean_prominence_db"]) for value in cluster
        )
        hits = len(cluster)
        confidence = min(
            1.0,
            hits / self._history_frames * 0.45
            + min(1.0, span_m / 0.10) * 0.25
            + min(1.0, prominence / 6.0) * 0.30,
        )
        return {
            "segment_id": f"contour_{segment_index}",
            "evidence_kind": "calibrated_heatmap_prominence",
            "hits": hits,
            "confidence": _rounded(confidence),
            "heading_deg": _rounded(heading_deg),
            "support_cells": int(round(statistics.median(
                int(value["support_cells"]) for value in cluster
            ))),
            "span_m": _rounded(span_m),
            "mean_prominence_db": _rounded(prominence),
            "start": [_rounded(number) for number in start],
            "end": [_rounded(number) for number in end],
        }
