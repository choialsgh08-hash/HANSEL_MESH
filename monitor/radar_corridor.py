import math
import statistics
from collections import deque
from typing import Deque, Dict, List, Mapping, Optional, Sequence, Tuple


MIN_CELL_INTENSITY = 0.30
LINE_TOLERANCE_M = 0.026
MIN_WALL_SUPPORT_CELLS = 4
MIN_WALL_SPAN_M = 0.14
MAX_ABS_WALL_HEADING_DEG = 75.0
MAX_PARALLEL_HEADING_DIFFERENCE_DEG = 10.0
MIN_CORRIDOR_WIDTH_M = 0.14
MAX_CORRIDOR_WIDTH_M = 0.55
MAX_SHARED_SUPPORT_CELLS = 1
MAX_LINE_CANDIDATES = 128
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
        self._history.append(self._fit_pair(cells))

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
            walls.append({
                "wall_id": f"wall_{wall_index + 1}",
                "heading_deg": _rounded(_median([
                    float(wall["heading_deg"])
                    for wall in source_walls
                ])),
                "normal_offset_m": _rounded(_median([
                    float(wall["normal_offset_m"])
                    for wall in source_walls
                ])),
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
                    _rounded(_median([
                        float(wall["start"][axis])
                        for wall in source_walls
                    ]))
                    for axis in range(2)
                ],
                "end": [
                    _rounded(_median([
                        float(wall["end"][axis])
                        for wall in source_walls
                    ]))
                    for axis in range(2)
                ],
            })
        measured_heading = _median([
            float(fit["measured_heading_deg"])
            for fit in cluster
        ])
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
            "width_m": _rounded(_median([
                float(fit["width_m"])
                for fit in cluster
            ])),
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
    ) -> Optional[Dict[str, object]]:
        points = cls._points(cells)
        candidates: Dict[frozenset[int], Dict[str, object]] = {}
        for left_index, left in enumerate(points):
            for right in points[left_index + 1:]:
                delta_forward = right[0] - left[0]
                delta_lateral = right[1] - left[1]
                pair_span = math.hypot(delta_forward, delta_lateral)
                if pair_span < 0.10:
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
                inliers = frozenset(
                    index
                    for index, point in enumerate(points)
                    if abs(
                        normal_forward * point[0]
                        + normal_lateral * point[1]
                        - normal_offset
                    ) <= LINE_TOLERANCE_M + 1e-12
                )
                if len(inliers) < MIN_WALL_SUPPORT_CELLS:
                    continue
                projections = [
                    unit_forward * points[index][0]
                    + unit_lateral * points[index][1]
                    for index in inliers
                ]
                span = max(projections) - min(projections)
                if span < MIN_WALL_SPAN_M:
                    continue
                intensity_sum = sum(points[index][2] for index in inliers)
                candidate = {
                    "heading_deg": heading,
                    "normal_offset_m": normal_offset,
                    "support": inliers,
                    "support_cells": len(inliers),
                    "span_m": span,
                    "mean_intensity": intensity_sum / len(inliers),
                    "score": intensity_sum + span * 2.0,
                    "start": [
                        unit_forward * min(projections)
                        + normal_forward * normal_offset,
                        unit_lateral * min(projections)
                        + normal_lateral * normal_offset,
                    ],
                    "end": [
                        unit_forward * max(projections)
                        + normal_forward * normal_offset,
                        unit_lateral * max(projections)
                        + normal_lateral * normal_offset,
                    ],
                }
                existing = candidates.get(inliers)
                if existing is None or cls._line_sort_key(candidate) < (
                    cls._line_sort_key(existing)
                ):
                    candidates[inliers] = candidate
        ordered = sorted(
            candidates.values(),
            key=cls._line_sort_key,
        )[:MAX_LINE_CANDIDATES]
        pairs: List[Tuple[Tuple[object, ...], Dict[str, object], Dict[str, object]]] = []
        for left_index, left in enumerate(ordered):
            for right in ordered[left_index + 1:]:
                if abs(
                    float(left["heading_deg"])
                    - float(right["heading_deg"])
                ) > MAX_PARALLEL_HEADING_DIFFERENCE_DEG:
                    continue
                width = abs(
                    float(left["normal_offset_m"])
                    - float(right["normal_offset_m"])
                )
                if not MIN_CORRIDOR_WIDTH_M <= width <= MAX_CORRIDOR_WIDTH_M:
                    continue
                shared = len(left["support"] & right["support"])
                if shared > MAX_SHARED_SUPPORT_CELLS:
                    continue
                unique_support = left["support"] | right["support"]
                unique_intensity = sum(
                    points[index][2]
                    for index in unique_support
                )
                combined_score = (
                    unique_intensity
                    + float(left["span_m"]) * 2.0
                    + float(right["span_m"]) * 2.0
                )
                pair_key = (
                    -combined_score,
                    -len(unique_support),
                    -min(float(left["span_m"]), float(right["span_m"])),
                    abs((
                        float(left["heading_deg"])
                        + float(right["heading_deg"])
                    ) / 2.0),
                    width,
                    float(left["normal_offset_m"]),
                    float(right["normal_offset_m"]),
                )
                pairs.append((pair_key, left, right))
        if not pairs:
            return None
        _, first, second = min(pairs, key=lambda value: value[0])
        walls = sorted(
            [first, second],
            key=lambda wall: float(wall["normal_offset_m"]),
        )
        measured_heading = (
            float(walls[0]["heading_deg"])
            + float(walls[1]["heading_deg"])
        ) / 2.0
        width = abs(
            float(walls[1]["normal_offset_m"])
            - float(walls[0]["normal_offset_m"])
        )
        support_total = sum(int(wall["support_cells"]) for wall in walls)
        minimum_span = min(float(wall["span_m"]) for wall in walls)
        mean_intensity = sum(
            float(wall["mean_intensity"]) for wall in walls
        ) / 2.0
        confidence = min(
            1.0,
            min(1.0, support_total / 12.0) * 0.50
            + min(1.0, minimum_span / 0.40) * 0.25
            + mean_intensity * 0.25,
        )
        return {
            "measured_heading_deg": measured_heading,
            "width_m": width,
            "confidence": confidence,
            "walls": [
                {
                    key: value
                    for key, value in wall.items()
                    if key not in {"support", "score"}
                }
                for wall in walls
            ],
        }

    @staticmethod
    def _line_sort_key(candidate: Mapping[str, object]) -> Tuple[object, ...]:
        return (
            -float(candidate["score"]),
            -int(candidate["support_cells"]),
            -float(candidate["span_m"]),
            abs(float(candidate["heading_deg"])),
            float(candidate["heading_deg"]),
            float(candidate["normal_offset_m"]),
        )
