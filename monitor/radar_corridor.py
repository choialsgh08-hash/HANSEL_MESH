import math
import statistics
from collections import deque
from typing import Deque, Dict, List, Mapping, Optional, Sequence, Tuple


MIN_CELL_INTENSITY = 0.30
LINE_TOLERANCE_M = 0.026
MIN_POINT_WALL_SUPPORT = 2
MIN_POINT_WALL_SPAN_M = 0.04
MIN_HEATMAP_WALL_SUPPORT = 2
MIN_HEATMAP_WALL_SPAN_M = 0.08
MAX_ABS_WALL_HEADING_DEG = 75.0
MIN_CORRIDOR_WIDTH_M = 0.06
MAX_CORRIDOR_WIDTH_M = 0.55
MIN_POINT_WALL_CLEARANCE_FROM_SENSOR_M = 0.005
MIN_HEATMAP_WALL_CLEARANCE_FROM_SENSOR_M = 0.03
COMPATIBLE_HEADING_DEG = 5.0
COMPATIBLE_WIDTH_M = 0.05
MAX_GUIDANCE_HEADING_DEG = 55.0
METHOD = "deterministic_parallel_ransac"


Point = Tuple[float, float, float]


def _median(values: Sequence[float]) -> float:
    return float(statistics.median(values))


def _rounded(value: float) -> float:
    return round(float(value), 6)


class ReflectivityCorridorEstimator:
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
        self._history: Deque[Optional[Dict[str, object]]] = deque(
            maxlen=history_frames,
        )

    def reset(self) -> None:
        self._history.clear()

    def ingest(
        self,
        cells: Sequence[Mapping[str, object]],
        elapsed_frame_windows: int = 1,
        *,
        point_observations: Sequence[Mapping[str, object]] = (),
    ) -> None:
        if (
            isinstance(elapsed_frame_windows, bool)
            or not isinstance(elapsed_frame_windows, int)
        ):
            raise TypeError("elapsed_frame_windows must be an integer")
        if elapsed_frame_windows < 1:
            raise ValueError("elapsed_frame_windows must be positive")
        for _ in range(min(self._history_frames, elapsed_frame_windows) - 1):
            self._history.append(None)
        self._history.append(self._fit_pair(cells, point_observations))

    def payload(self) -> Dict[str, object]:
        fits = [fit for fit in self._history if fit is not None]
        if not fits:
            return self._empty_payload()
        clusters: List[List[Dict[str, object]]] = []
        for seed in fits:
            compatible = [
                fit
                for fit in fits
                if abs(
                    float(fit["measured_heading_deg"])
                    - float(seed["measured_heading_deg"])
                ) <= COMPATIBLE_HEADING_DEG
                and abs(
                    float(fit["width_m"]) - float(seed["width_m"])
                ) <= COMPATIBLE_WIDTH_M
            ]
            clusters.append(compatible)
        cluster = max(
            clusters,
            key=lambda values: (
                len(values),
                sum(float(value["confidence"]) for value in values),
                -abs(_median([
                    float(value["measured_heading_deg"])
                    for value in values
                ])),
            ),
        )
        hits = len(cluster)
        walls: List[Dict[str, object]] = []
        for wall_index in range(2):
            source_walls = [
                fit["walls"][wall_index]
                for fit in cluster
            ]
            wall_heading = _median([
                float(wall["heading_deg"])
                for wall in source_walls
            ])
            normal_offset = _median([
                float(wall["normal_offset_m"])
                for wall in source_walls
            ])
            start_forward = _median([
                float(wall["start"][0])
                for wall in source_walls
            ])
            end_forward = _median([
                float(wall["end"][0])
                for wall in source_walls
            ])
            heading_rad = math.radians(wall_heading)
            lateral_at_origin = normal_offset / math.cos(heading_rad)
            walls.append({
                "wall_id": f"wall_{wall_index + 1}",
                "heading_deg": _rounded(wall_heading),
                "normal_offset_m": _rounded(normal_offset),
                "support_cells": int(round(_median([
                    float(wall["support_cells"])
                    for wall in source_walls
                ]))),
                "span_m": _rounded(_median([
                    float(wall["span_m"])
                    for wall in source_walls
                ])),
                "mean_intensity": _rounded(_median([
                    float(wall["mean_intensity"])
                    for wall in source_walls
                ])),
                "start": [
                    _rounded(start_forward),
                    _rounded(
                        lateral_at_origin
                        + math.tan(heading_rad) * start_forward
                    ),
                ],
                "end": [
                    _rounded(end_forward),
                    _rounded(
                        lateral_at_origin
                        + math.tan(heading_rad) * end_forward
                    ),
                ],
            })
        measured_heading = _median([
            float(fit["measured_heading_deg"])
            for fit in cluster
        ])
        serialized_width = abs(
            float(walls[0]["normal_offset_m"])
            - float(walls[1]["normal_offset_m"])
        )
        return {
            "status": (
                "confirmed" if hits >= self._required_hits else "candidate"
            ),
            "method": METHOD,
            "history_frames": self._history_frames,
            "required_hits": self._required_hits,
            "hits": hits,
            "measured_heading_deg": _rounded(measured_heading),
            "guidance_heading_deg": _rounded(max(
                -MAX_GUIDANCE_HEADING_DEG,
                min(MAX_GUIDANCE_HEADING_DEG, measured_heading),
            )),
            "width_m": _rounded(serialized_width),
            "confidence": _rounded(_median([
                float(fit["confidence"])
                for fit in cluster
            ])),
            "walls": walls,
        }

    def _empty_payload(self) -> Dict[str, object]:
        return {
            "status": "unavailable",
            "method": METHOD,
            "history_frames": self._history_frames,
            "required_hits": self._required_hits,
            "hits": 0,
            "measured_heading_deg": None,
            "guidance_heading_deg": None,
            "width_m": None,
            "confidence": 0.0,
            "walls": [],
        }

    @staticmethod
    def _points(cells: Sequence[Mapping[str, object]]) -> List[Point]:
        unique: Dict[Tuple[float, float], Point] = {}
        for cell in cells:
            if not isinstance(cell, Mapping):
                continue
            try:
                forward = float(cell["forward_m"])
                lateral = float(cell["lateral_m"])
                intensity = float(cell["intensity"])
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if (
                not math.isfinite(forward)
                or not math.isfinite(lateral)
                or not math.isfinite(intensity)
                or forward < 0.0
                or intensity < MIN_CELL_INTENSITY
                or intensity > 1.0
            ):
                continue
            key = (forward, lateral)
            existing = unique.get(key)
            if existing is None or intensity > existing[2]:
                unique[key] = (forward, lateral, intensity)
        return sorted(unique.values())

    @classmethod
    def _fit_pair(
        cls,
        cells: Sequence[Mapping[str, object]],
        point_observations: Sequence[Mapping[str, object]],
    ) -> Optional[Dict[str, object]]:
        heatmap_points = cls._points(cells)
        point_returns = cls._points(point_observations)
        pairs: List[Tuple[Tuple[object, ...], Dict[str, object]]] = []
        for left_index, left in enumerate(point_returns):
            for right in point_returns[left_index + 1:]:
                delta_forward = right[0] - left[0]
                delta_lateral = right[1] - left[1]
                pair_span = math.hypot(delta_forward, delta_lateral)
                if pair_span < MIN_POINT_WALL_SPAN_M:
                    continue
                heading = math.degrees(math.atan2(
                    delta_lateral,
                    delta_forward,
                ))
                while heading > 90.0:
                    heading -= 180.0
                while heading < -90.0:
                    heading += 180.0
                if abs(heading) > MAX_ABS_WALL_HEADING_DEG:
                    continue
                heading_rad = math.radians(heading)
                unit_forward = math.cos(heading_rad)
                unit_lateral = math.sin(heading_rad)
                normal_forward = -unit_lateral
                normal_lateral = unit_forward
                normal_offset = (
                    normal_forward * left[0]
                    + normal_lateral * left[1]
                )
                point_inliers = tuple(
                    index
                    for index, point in enumerate(point_returns)
                    if abs(
                        normal_forward * point[0]
                        + normal_lateral * point[1]
                        - normal_offset
                    ) <= LINE_TOLERANCE_M + 1e-12
                )
                if len(point_inliers) < MIN_POINT_WALL_SUPPORT:
                    continue
                point_projections = [
                    unit_forward * point_returns[index][0]
                    + unit_lateral * point_returns[index][1]
                    for index in point_inliers
                ]
                point_span = max(point_projections) - min(point_projections)
                if point_span < MIN_POINT_WALL_SPAN_M:
                    continue
                normal_offset = _median([
                    normal_forward * point_returns[index][0]
                    + normal_lateral * point_returns[index][1]
                    for index in point_inliers
                ])
                if (
                    abs(normal_offset)
                    < MIN_POINT_WALL_CLEARANCE_FROM_SENSOR_M
                ):
                    continue
                primary = cls._wall_payload(
                    heading,
                    normal_offset,
                    [point_returns[index] for index in point_inliers],
                )
                heatmap_offsets = [
                    normal_forward * point[0] + normal_lateral * point[1]
                    for point in heatmap_points
                ]
                seen_support = set()
                for seed_offset in heatmap_offsets:
                    heatmap_inliers = tuple(
                        index
                        for index, offset in enumerate(heatmap_offsets)
                        if abs(offset - seed_offset) <= LINE_TOLERANCE_M + 1e-12
                    )
                    if heatmap_inliers in seen_support:
                        continue
                    seen_support.add(heatmap_inliers)
                    if len(heatmap_inliers) < MIN_HEATMAP_WALL_SUPPORT:
                        continue
                    heatmap_projections = [
                        unit_forward * heatmap_points[index][0]
                        + unit_lateral * heatmap_points[index][1]
                        for index in heatmap_inliers
                    ]
                    heatmap_span = max(heatmap_projections) - min(
                        heatmap_projections
                    )
                    if heatmap_span < MIN_HEATMAP_WALL_SPAN_M:
                        continue
                    secondary_offset = _median([
                        heatmap_offsets[index] for index in heatmap_inliers
                    ])
                    if (
                        abs(secondary_offset)
                        < MIN_HEATMAP_WALL_CLEARANCE_FROM_SENSOR_M
                    ):
                        continue
                    offsets = sorted((normal_offset, secondary_offset))
                    if not offsets[0] < 0.0 < offsets[1]:
                        continue
                    width = offsets[1] - offsets[0]
                    if not MIN_CORRIDOR_WIDTH_M <= width <= MAX_CORRIDOR_WIDTH_M:
                        continue
                    secondary = cls._wall_payload(
                        heading,
                        secondary_offset,
                        [heatmap_points[index] for index in heatmap_inliers],
                    )
                    walls = sorted(
                        (primary, secondary),
                        key=lambda wall: float(wall["normal_offset_m"]),
                    )
                    support_total = sum(
                        int(wall["support_cells"]) for wall in walls
                    )
                    minimum_span = min(
                        float(wall["span_m"]) for wall in walls
                    )
                    mean_intensity = sum(
                        float(wall["mean_intensity"]) for wall in walls
                    ) / 2.0
                    primary_mean_intensity = sum(
                        point_returns[index][2]
                        for index in point_inliers
                    ) / len(point_inliers)
                    confidence = min(
                        1.0,
                        min(1.0, support_total / 8.0) * 0.50
                        + min(1.0, minimum_span / 0.30) * 0.25
                        + mean_intensity * 0.25,
                    )
                    pair_key = (
                        -primary_mean_intensity,
                        -confidence,
                        -support_total,
                        -minimum_span,
                        width,
                        abs(heading),
                        offsets[0],
                    )
                    pairs.append((pair_key, {
                        "measured_heading_deg": heading,
                        "width_m": width,
                        "confidence": confidence,
                        "walls": walls,
                    }))
        if not pairs:
            return None
        return min(pairs, key=lambda value: value[0])[1]

    @staticmethod
    def _wall_payload(
        heading_deg: float,
        normal_offset_m: float,
        support: Sequence[Point],
    ) -> Dict[str, object]:
        heading_rad = math.radians(heading_deg)
        normal_lateral = math.cos(heading_rad)
        lateral_at_origin = normal_offset_m / normal_lateral
        end_forward = max(point[0] for point in support)
        end_lateral = (
            lateral_at_origin + math.tan(heading_rad) * end_forward
        )
        unit_forward = math.cos(heading_rad)
        unit_lateral = math.sin(heading_rad)
        projections = [
            unit_forward * point[0] + unit_lateral * point[1]
            for point in support
        ]
        return {
            "heading_deg": heading_deg,
            "normal_offset_m": normal_offset_m,
            "support_cells": len(support),
            "span_m": max(projections) - min(projections),
            "mean_intensity": sum(point[2] for point in support) / len(support),
            "start": [0.0, lateral_at_origin],
            "end": [end_forward, end_lateral],
        }
