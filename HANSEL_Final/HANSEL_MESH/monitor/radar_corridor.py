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
HEATMAP_ONLY_MIN_WALL_SUPPORT = 5
HEATMAP_ONLY_MIN_MEAN_INTENSITY = 0.60
INDEPENDENT_MIN_WALL_SPAN_M = 0.18
INDEPENDENT_MIN_OVERLAP_M = 0.12
INDEPENDENT_PARALLEL_WIDTH_DELTA_M = 0.025
INDEPENDENT_HEADING_STEP_DEG = 5
INDEPENDENT_OFFSET_BIN_M = 0.025
INDEPENDENT_MAX_HOUGH_SEEDS = 16
INDEPENDENT_MAX_WALL_CANDIDATES = 16
INDEPENDENT_MAX_FORWARD_GAP_M = 0.10
INDEPENDENT_MAX_SUPPORT_GAP_M = 0.095
INDEPENDENT_MIN_LOCAL_CONTRAST = 0.05
BOUNDARY_ONLY_MIN_MEAN_INTENSITY = 0.90
BOUNDARY_ONLY_MIN_CELL_INTENSITY = 0.90
BOUNDARY_ONLY_MIN_ENVELOPE_RATIO = 0.80
POINT_BOUNDARY_MIN_SUPPORT = 3
POINT_BOUNDARY_MIN_SPAN_M = 0.10
POINT_BOUNDARY_MAX_SUPPORT_GAP_M = 0.18
POINT_BOUNDARY_MIN_MEAN_INTENSITY = 0.50
POINT_BOUNDARY_MAX_LINE_RESIDUAL_M = 0.03
BOUNDARY_PROMINENCE_BAND_M = LINE_TOLERANCE_M + 0.010
MIN_BOUNDARY_PROMINENCE = 0.55
CONVERGING_MIN_ROWS = 4
CONVERGING_SHORT_RUN_MAX_LINE_RESIDUAL_M = 0.010
CONVERGING_MAX_ROW_GAP_M = 0.055
CONVERGING_MAX_WIDTH_REOPEN_M = 0.025
CONVERGING_MIN_WIDTH_REDUCTION_M = 0.075
CONVERGING_MIN_SIDE_CLEARANCE_M = 0.03
CONVERGING_MAX_LINE_RESIDUAL_M = 0.020
MAX_ABS_WALL_HEADING_DEG = 75.0
MIN_CORRIDOR_WIDTH_M = 0.06
MAX_CORRIDOR_WIDTH_M = 0.55
MIN_POINT_WALL_CLEARANCE_FROM_SENSOR_M = 0.005
MIN_HEATMAP_WALL_CLEARANCE_FROM_SENSOR_M = 0.03
COMPATIBLE_HEADING_DEG = 5.0
COMPATIBLE_WIDTH_M = 0.05
COMPATIBLE_WALL_HEADING_DEG = 10.0
COMPATIBLE_WALL_POSITION_M = 0.075
MAX_GUIDANCE_HEADING_DEG = 55.0
METHOD = "deterministic_boundary_fit"


Point = Tuple[float, float, float]


def _median(values: Sequence[float]) -> float:
    return float(statistics.median(values))


def _rounded(value: float) -> float:
    return round(float(value), 6)


def _least_squares_line(points: Sequence[Point]) -> Optional[Tuple[float, float]]:
    if len(points) < 2:
        return None
    mean_forward = sum(point[0] for point in points) / len(points)
    mean_lateral = sum(point[1] for point in points) / len(points)
    denominator = sum(
        (point[0] - mean_forward) ** 2 for point in points
    )
    if denominator <= 1e-12:
        return None
    slope = sum(
        (point[0] - mean_forward) * (point[1] - mean_lateral)
        for point in points
    ) / denominator
    intercept = _median([
        point[1] - slope * point[0] for point in points
    ])
    return slope, intercept


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
        self._boundary_history: Deque[Tuple[Dict[str, object], ...]] = deque(
            maxlen=history_frames,
        )

    def reset(self) -> None:
        self._history.clear()
        self._boundary_history.clear()

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
            self._boundary_history.append(())
        self._history.append(self._fit_pair(cells, point_observations))
        point_boundaries = self._independent_point_boundary_candidates(
            self._points(point_observations)
        )
        point_sides = {wall["side"] for wall in point_boundaries}
        represented_sides = point_sides
        heatmap_boundaries = [
            wall
            for wall in self._independent_boundary_candidates(
                self._points(cells)
            )
            if wall["side"] not in represented_sides
        ]
        self._boundary_history.append(tuple(
            point_boundaries + heatmap_boundaries
        ))

    def boundaries_payload(self) -> Dict[str, object]:
        confirmed: List[Dict[str, object]] = []
        for side in ("left", "right"):
            frames = [
                [wall for wall in frame if wall["side"] == side]
                for frame in self._boundary_history
            ]
            seeds = [wall for frame in frames for wall in frame]
            clusters: List[List[Dict[str, object]]] = []
            for seed in seeds:
                cluster = []
                for frame in frames:
                    compatible = [
                        wall for wall in frame
                        if self._boundaries_are_compatible(wall, seed)
                    ]
                    if compatible:
                        cluster.append(min(
                            compatible,
                            key=lambda wall: (
                                -float(wall["mean_intensity"]),
                                -int(wall["support_cells"]),
                                float(wall["max_residual_m"]),
                            ),
                        ))
                clusters.append(cluster)
            if not clusters:
                continue
            cluster = max(
                clusters,
                key=lambda values: (
                    len(values),
                    sum(float(value["mean_intensity"]) for value in values),
                    sum(float(value["span_m"]) for value in values),
                ),
            )
            if len(cluster) < self._required_hits:
                continue
            confirmed.append(self._serialize_boundary(side, cluster))
        return {
            "status": "confirmed" if confirmed else "unavailable",
            "method": "independent_boundary_fit",
            "history_frames": self._history_frames,
            "required_hits": self._required_hits,
            "walls": confirmed,
        }

    @staticmethod
    def _boundaries_are_compatible(
        left: Mapping[str, object],
        right: Mapping[str, object],
    ) -> bool:
        return (
            left["side"] == right["side"]
            and left["evidence_kind"] == right["evidence_kind"]
            and abs(
                float(left["heading_deg"])
                - float(right["heading_deg"])
            ) <= COMPATIBLE_WALL_HEADING_DEG
            and abs(
                float(left["normal_offset_m"])
                - float(right["normal_offset_m"])
            ) <= COMPATIBLE_WALL_POSITION_M
        )

    def _serialize_boundary(
        self,
        side: str,
        cluster: Sequence[Mapping[str, object]],
    ) -> Dict[str, object]:
        heading_deg = _median([
            float(wall["heading_deg"]) for wall in cluster
        ])
        normal_offset_m = _median([
            float(wall["normal_offset_m"]) for wall in cluster
        ])
        start_forward = _median([
            float(wall["start"][0]) for wall in cluster
        ])
        end_forward = _median([
            float(wall["end"][0]) for wall in cluster
        ])
        heading_rad = math.radians(heading_deg)
        intercept = normal_offset_m / math.cos(heading_rad)
        slope = math.tan(heading_rad)
        start = [start_forward, intercept + slope * start_forward]
        end = [end_forward, intercept + slope * end_forward]
        span_m = math.hypot(end[0] - start[0], end[1] - start[1])
        mean_intensity = _median([
            float(wall["mean_intensity"]) for wall in cluster
        ])
        hits = len(cluster)
        confidence = min(
            1.0,
            hits / self._history_frames * 0.45
            + mean_intensity * 0.35
            + min(1.0, span_m / 0.30) * 0.20,
        )
        return {
            "wall_id": f"boundary_{side}",
            "evidence_kind": str(cluster[0]["evidence_kind"]),
            "side": side,
            "hits": hits,
            "confidence": _rounded(confidence),
            "heading_deg": _rounded(heading_deg),
            "normal_offset_m": _rounded(normal_offset_m),
            "support_cells": int(round(_median([
                float(wall["support_cells"]) for wall in cluster
            ]))),
            "span_m": _rounded(span_m),
            "mean_intensity": _rounded(mean_intensity),
            "start": [_rounded(value) for value in start],
            "end": [_rounded(value) for value in end],
        }

    def payload(self) -> Dict[str, object]:
        fits = [fit for fit in self._history if fit is not None]
        if not fits:
            return self._empty_payload()
        clusters: List[List[Dict[str, object]]] = []
        for seed in fits:
            compatible = [
                fit
                for fit in fits
                if self._fits_are_compatible(fit, seed)
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
        geometry = str(cluster[0]["geometry"])
        walls: List[Dict[str, object]] = []
        for wall_index in range(2):
            source_walls = [
                fit["walls"][wall_index]
                for fit in cluster
            ]
            start_forward = _median([
                float(wall["start"][0])
                for wall in source_walls
            ])
            start_lateral = _median([
                float(wall["start"][1])
                for wall in source_walls
            ])
            end_forward = _median([
                float(wall["end"][0])
                for wall in source_walls
            ])
            end_lateral = _median([
                float(wall["end"][1])
                for wall in source_walls
            ])
            if geometry == "parallel":
                wall_heading = _median([
                    float(wall["heading_deg"])
                    for wall in source_walls
                ])
                normal_offset = _median([
                    float(wall["normal_offset_m"])
                    for wall in source_walls
                ])
                heading_rad = math.radians(wall_heading)
                start_lateral = (
                    normal_offset / math.cos(heading_rad)
                    + math.tan(heading_rad) * start_forward
                )
                end_lateral = (
                    normal_offset / math.cos(heading_rad)
                    + math.tan(heading_rad) * end_forward
                )
            else:
                wall_heading = math.degrees(math.atan2(
                    end_lateral - start_lateral,
                    end_forward - start_forward,
                ))
                heading_rad = math.radians(wall_heading)
                normal_offset = (
                    -math.sin(heading_rad) * start_forward
                    + math.cos(heading_rad) * start_lateral
                )
            endpoint_span = math.hypot(
                end_forward - start_forward,
                end_lateral - start_lateral,
            )
            serialized_span = (
                _median([
                    float(wall["span_m"])
                    for wall in source_walls
                ])
                if geometry == "parallel"
                else endpoint_span
            )
            walls.append({
                "wall_id": f"wall_{wall_index + 1}",
                "heading_deg": _rounded(wall_heading),
                "normal_offset_m": _rounded(normal_offset),
                "support_cells": int(round(_median([
                    float(wall["support_cells"])
                    for wall in source_walls
                ]))),
                "span_m": _rounded(serialized_span),
                "mean_intensity": _rounded(_median([
                    float(wall["mean_intensity"])
                    for wall in source_walls
                ])),
                "start": [
                    _rounded(start_forward),
                    _rounded(start_lateral),
                ],
                "end": [
                    _rounded(end_forward),
                    _rounded(end_lateral),
                ],
            })
        overlap_start = max(float(wall["start"][0]) for wall in walls)
        overlap_end = min(float(wall["end"][0]) for wall in walls)
        near_center = (
            overlap_start,
            sum(
                self._wall_lateral_at(wall, overlap_start)
                for wall in walls
            ) / 2.0,
        )
        far_center = (
            overlap_end,
            sum(
                self._wall_lateral_at(wall, overlap_end)
                for wall in walls
            ) / 2.0,
        )
        measured_heading = math.degrees(math.atan2(
            far_center[1] - near_center[1],
            far_center[0] - near_center[0],
        ))
        if geometry == "converging":
            serialized_width = (
                self._wall_lateral_at(walls[1], overlap_end)
                - self._wall_lateral_at(walls[0], overlap_end)
            )
        else:
            serialized_width = abs(
                float(walls[0]["normal_offset_m"])
                - float(walls[1]["normal_offset_m"])
            )
        return {
            "status": (
                "confirmed" if hits >= self._required_hits else "candidate"
            ),
            "method": METHOD,
            "geometry": geometry,
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

    @staticmethod
    def _fits_are_compatible(
        left: Dict[str, object],
        right: Dict[str, object],
    ) -> bool:
        if (
            left["geometry"] != right["geometry"]
            or abs(
                float(left["measured_heading_deg"])
                - float(right["measured_heading_deg"])
            ) > COMPATIBLE_HEADING_DEG
            or abs(float(left["width_m"]) - float(right["width_m"]))
            > COMPATIBLE_WIDTH_M
        ):
            return False
        if left["geometry"] != "converging":
            return True
        for wall_index in range(2):
            left_wall = left["walls"][wall_index]
            right_wall = right["walls"][wall_index]
            if abs(
                float(left_wall["heading_deg"])
                - float(right_wall["heading_deg"])
            ) > COMPATIBLE_WALL_HEADING_DEG:
                return False
            for endpoint in ("start", "end"):
                for coordinate in range(2):
                    if abs(
                        float(left_wall[endpoint][coordinate])
                        - float(right_wall[endpoint][coordinate])
                    ) > COMPATIBLE_WALL_POSITION_M:
                        return False
        return True

    def _empty_payload(self) -> Dict[str, object]:
        return {
            "status": "unavailable",
            "method": METHOD,
            "geometry": None,
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
        converging = cls._converging_candidate(heatmap_points)
        if converging is not None:
            return converging
        pairs = cls._pair_candidates(
            primary_points=point_returns,
            secondary_points=heatmap_points,
            primary_min_support=MIN_POINT_WALL_SUPPORT,
            primary_min_span_m=MIN_POINT_WALL_SPAN_M,
            primary_min_clearance_m=MIN_POINT_WALL_CLEARANCE_FROM_SENSOR_M,
            primary_min_mean_intensity=MIN_CELL_INTENSITY,
            secondary_min_support=MIN_HEATMAP_WALL_SUPPORT,
            secondary_min_span_m=MIN_HEATMAP_WALL_SPAN_M,
            secondary_min_clearance_m=(
                MIN_HEATMAP_WALL_CLEARANCE_FROM_SENSOR_M
            ),
            secondary_min_mean_intensity=MIN_CELL_INTENSITY,
            require_disjoint_support=False,
        )
        if pairs:
            return min(pairs, key=lambda value: value[0])[1]
        return cls._independent_heatmap_candidate(heatmap_points)

    @classmethod
    def _independent_heatmap_candidate(
        cls,
        heatmap_points: Sequence[Point],
    ) -> Optional[Dict[str, object]]:
        side_candidates = [
            cls._independent_wall_candidates(
                [
                    point for point in heatmap_points
                    if point[1] < -CONVERGING_MIN_SIDE_CLEARANCE_M
                ]
            ),
            cls._independent_wall_candidates(
                [
                    point for point in heatmap_points
                    if point[1] > CONVERGING_MIN_SIDE_CLEARANCE_M
                ]
            ),
        ]
        pairs: List[Tuple[Tuple[object, ...], Dict[str, object]]] = []
        for left in side_candidates[0]:
            for right in side_candidates[1]:
                overlap_start = max(
                    float(left["start"][0]),
                    float(right["start"][0]),
                )
                overlap_end = min(
                    float(left["end"][0]),
                    float(right["end"][0]),
                )
                if overlap_end - overlap_start < INDEPENDENT_MIN_OVERLAP_M:
                    continue
                walls = [left, right]
                start_width = (
                    cls._wall_lateral_at(walls[1], overlap_start)
                    - cls._wall_lateral_at(walls[0], overlap_start)
                )
                end_width = (
                    cls._wall_lateral_at(walls[1], overlap_end)
                    - cls._wall_lateral_at(walls[0], overlap_end)
                )
                if not (
                    MIN_CORRIDOR_WIDTH_M
                    <= min(start_width, end_width)
                    <= max(start_width, end_width)
                    <= MAX_CORRIDOR_WIDTH_M
                ):
                    continue
                heading_delta = abs(
                    float(walls[0]["heading_deg"])
                    - float(walls[1]["heading_deg"])
                )
                if (
                    heading_delta <= COMPATIBLE_WALL_HEADING_DEG
                    and abs(start_width - end_width)
                    <= INDEPENDENT_PARALLEL_WIDTH_DELTA_M
                ):
                    geometry = "parallel"
                    width = (start_width + end_width) / 2.0
                elif (
                    end_width < start_width
                    and start_width - end_width
                    >= CONVERGING_MIN_WIDTH_REDUCTION_M - 1e-12
                ):
                    geometry = "converging"
                    width = end_width
                else:
                    continue
                near_center_lateral = (
                    cls._wall_lateral_at(walls[0], overlap_start)
                    + cls._wall_lateral_at(walls[1], overlap_start)
                ) / 2.0
                far_center_lateral = (
                    cls._wall_lateral_at(walls[0], overlap_end)
                    + cls._wall_lateral_at(walls[1], overlap_end)
                ) / 2.0
                measured_heading = math.degrees(math.atan2(
                    far_center_lateral - near_center_lateral,
                    overlap_end - overlap_start,
                ))
                if abs(measured_heading) > MAX_ABS_WALL_HEADING_DEG:
                    continue
                overlap_supports = [
                    [
                        point for point in wall["_support"]
                        if overlap_start - 1e-12
                        <= point[0]
                        <= overlap_end + 1e-12
                    ]
                    for wall in walls
                ]
                if any(len(support) < 3 for support in overlap_supports):
                    continue
                if not cls._has_boundary_prominence(
                    heatmap_points,
                    walls,
                    overlap_start,
                    overlap_end,
                ):
                    continue
                overlap_intensities = [
                    sum(point[2] for point in support) / len(support)
                    for support in overlap_supports
                ]
                minimum_intensity = min(overlap_intensities)
                mean_intensity = sum(overlap_intensities) / 2.0
                minimum_span = min(
                    math.hypot(
                        overlap_end - overlap_start,
                        cls._wall_lateral_at(wall, overlap_end)
                        - cls._wall_lateral_at(wall, overlap_start),
                    )
                    for wall in walls
                )
                support_total = sum(len(support) for support in overlap_supports)
                confidence = min(
                    1.0,
                    min(1.0, support_total / 12.0) * 0.35
                    + min(1.0, minimum_span / 0.25) * 0.25
                    + mean_intensity * 0.30
                    + min(
                        1.0,
                        1.0 - max(
                            float(wall["max_residual_m"])
                            for wall in walls
                        ) / CONVERGING_MAX_LINE_RESIDUAL_M,
                    ) * 0.10,
                )
                pair_key = (
                    overlap_end,
                    width,
                    -confidence,
                    -minimum_intensity,
                    -mean_intensity,
                    -minimum_span,
                )
                pairs.append((pair_key, {
                    "geometry": geometry,
                    "measured_heading_deg": measured_heading,
                    "width_m": width,
                    "confidence": confidence,
                    "walls": walls,
                }))
        return min(pairs, key=lambda value: value[0])[1] if pairs else None

    @classmethod
    def _independent_wall_candidates(
        cls,
        points: Sequence[Point],
    ) -> List[Dict[str, object]]:
        points = sorted(points)
        if len(points) < HEATMAP_ONLY_MIN_WALL_SUPPORT:
            return []
        hough_bins: Dict[Tuple[int, int], List[int]] = {}
        for heading_deg in range(
            -int(MAX_ABS_WALL_HEADING_DEG),
            int(MAX_ABS_WALL_HEADING_DEG) + 1,
            INDEPENDENT_HEADING_STEP_DEG,
        ):
            heading_rad = math.radians(heading_deg)
            normal_forward = -math.sin(heading_rad)
            normal_lateral = math.cos(heading_rad)
            for index, point in enumerate(points):
                offset = (
                    normal_forward * point[0]
                    + normal_lateral * point[1]
                )
                key = (
                    heading_deg,
                    round(offset / INDEPENDENT_OFFSET_BIN_M),
                )
                hough_bins.setdefault(key, []).append(index)

        seeds = sorted(
            hough_bins.items(),
            key=lambda item: (
                -len(item[1]),
                -sum(points[index][2] for index in item[1]),
                abs(item[0][0]),
                item[0],
            ),
        )[:INDEPENDENT_MAX_HOUGH_SEEDS]
        candidates: List[Dict[str, object]] = []
        seen_support = set()
        for (heading_deg, _), seed_indices in seeds:
            if len(seed_indices) < 3:
                continue
            support_indices = cls._robust_line_support(
                points,
                seed_indices,
                seed_slope=math.tan(math.radians(heading_deg)),
            )
            if len(support_indices) < HEATMAP_ONLY_MIN_WALL_SUPPORT:
                continue
            support_indices = cls._robust_line_support(
                points,
                support_indices,
            )
            if support_indices in seen_support:
                continue
            seen_support.add(support_indices)
            support = [points[index] for index in support_indices]
            ordered_support = sorted(support)
            if any(
                math.hypot(
                    right[0] - left[0],
                    right[1] - left[1],
                ) > INDEPENDENT_MAX_SUPPORT_GAP_M + 1e-12
                for left, right in zip(
                    ordered_support,
                    ordered_support[1:],
                )
            ):
                continue
            forward_rows = sorted(set(point[0] for point in support))
            if len(forward_rows) < HEATMAP_ONLY_MIN_WALL_SUPPORT:
                continue
            if any(
                right - left > INDEPENDENT_MAX_FORWARD_GAP_M + 1e-12
                for left, right in zip(forward_rows, forward_rows[1:])
            ):
                continue
            wall = cls._independent_wall_payload(support)
            if (
                float(wall["span_m"]) < INDEPENDENT_MIN_WALL_SPAN_M
                or float(wall["mean_intensity"])
                < HEATMAP_ONLY_MIN_MEAN_INTENSITY
                or float(wall["max_residual_m"])
                > CONVERGING_MAX_LINE_RESIDUAL_M
            ):
                continue
            support_set = set(support_indices)
            local_background = [
                point[2]
                for index, point in enumerate(points)
                if index not in support_set
                and float(wall["start"][0]) - 1e-12
                <= point[0]
                <= float(wall["end"][0]) + 1e-12
            ]
            if (
                len(local_background) >= 3
                and float(wall["mean_intensity"])
                < _median(local_background) + INDEPENDENT_MIN_LOCAL_CONTRAST
            ):
                continue
            candidates.append(wall)
        return sorted(
            candidates,
            key=lambda wall: (
                -float(wall["mean_intensity"]),
                -int(wall["support_cells"]),
                -float(wall["span_m"]),
                float(wall["max_residual_m"]),
                float(wall["start"][0]),
                float(wall["normal_offset_m"]),
            ),
        )[:INDEPENDENT_MAX_WALL_CANDIDATES]

    @classmethod
    def _independent_boundary_candidates(
        cls,
        heatmap_points: Sequence[Point],
    ) -> List[Dict[str, object]]:
        boundaries: List[Dict[str, object]] = []
        for side, sign in (("left", -1.0), ("right", 1.0)):
            side_points = [
                point for point in heatmap_points
                if sign * point[1] > CONVERGING_MIN_SIDE_CLEARANCE_M
            ]
            candidates = cls._independent_wall_candidates(side_points)
            eligible = []
            for wall in candidates:
                support = list(wall["_support"])
                if (
                    float(wall["mean_intensity"])
                    < BOUNDARY_ONLY_MIN_MEAN_INTENSITY
                    or min(point[2] for point in support)
                    < BOUNDARY_ONLY_MIN_CELL_INTENSITY
                ):
                    continue
                envelope_hits = 0
                for support_point in support:
                    row = [
                        point for point in side_points
                        if abs(point[0] - support_point[0])
                        <= INDEPENDENT_OFFSET_BIN_M / 2.0 + 1e-12
                    ]
                    extreme_lateral = (
                        max(point[1] for point in row)
                        if sign > 0.0
                        else min(point[1] for point in row)
                    )
                    inward_delta = sign * (
                        extreme_lateral - support_point[1]
                    )
                    if inward_delta <= INDEPENDENT_OFFSET_BIN_M + 1e-12:
                        envelope_hits += 1
                if (
                    envelope_hits / len(support)
                    < BOUNDARY_ONLY_MIN_ENVELOPE_RATIO
                ):
                    continue
                eligible.append({
                    **wall,
                    "side": side,
                    "evidence_kind": "heatmap",
                })
            if eligible:
                boundaries.append(eligible[0])
        return boundaries

    @classmethod
    def _independent_point_boundary_candidates(
        cls,
        point_observations: Sequence[Point],
    ) -> List[Dict[str, object]]:
        """Fit display-only short walls from current CFAR returns.

        Near vertical surfaces can carry an aliased elevation and be removed
        by the ground classifier even though their range/azimuth is useful in
        the top view.  This path intentionally consumes the current 2-D point
        evidence before that classifier, but still requires three collinear
        returns and the normal three-of-five temporal confirmation.
        """
        boundaries: List[Dict[str, object]] = []
        for side, sign in (("left", -1.0), ("right", 1.0)):
            points = sorted(
                point
                for point in point_observations
                if sign * point[1] > CONVERGING_MIN_SIDE_CLEARANCE_M
            )
            if len(points) < POINT_BOUNDARY_MIN_SUPPORT:
                continue
            candidates: List[Dict[str, object]] = []
            seen_support = set()
            for first_index in range(len(points) - 1):
                for last_index in range(first_index + 1, len(points)):
                    first = points[first_index]
                    last = points[last_index]
                    forward_delta = last[0] - first[0]
                    if abs(forward_delta) <= 1e-9:
                        continue
                    seed_slope = (last[1] - first[1]) / forward_delta
                    if abs(math.degrees(math.atan(seed_slope))) > (
                        MAX_ABS_WALL_HEADING_DEG
                    ):
                        continue
                    support_indices = cls._robust_line_support(
                        points,
                        (first_index, last_index),
                        seed_slope=seed_slope,
                    )
                    if len(support_indices) < POINT_BOUNDARY_MIN_SUPPORT:
                        continue
                    support_indices = cls._robust_line_support(
                        points,
                        support_indices,
                    )
                    if len(support_indices) < POINT_BOUNDARY_MIN_SUPPORT:
                        continue
                    if support_indices in seen_support:
                        continue
                    seen_support.add(support_indices)
                    support = [points[index] for index in support_indices]
                    if _least_squares_line(support) is None:
                        continue
                    wall = cls._independent_wall_payload(support)
                    ordered = list(wall["_support"])
                    if (
                        float(wall["span_m"]) < POINT_BOUNDARY_MIN_SPAN_M
                        or float(wall["mean_intensity"])
                        < POINT_BOUNDARY_MIN_MEAN_INTENSITY
                        or float(wall["max_residual_m"])
                        > POINT_BOUNDARY_MAX_LINE_RESIDUAL_M
                        or any(
                            math.hypot(
                                right[0] - left[0],
                                right[1] - left[1],
                            ) > POINT_BOUNDARY_MAX_SUPPORT_GAP_M + 1e-12
                            for left, right in zip(ordered, ordered[1:])
                        )
                    ):
                        continue
                    candidates.append({
                        **wall,
                        "side": side,
                        "evidence_kind": "point",
                    })
            if candidates:
                boundaries.append(min(
                    candidates,
                    key=lambda wall: (
                        -int(wall["support_cells"]),
                        float(wall["max_residual_m"]),
                        -float(wall["span_m"]),
                        -float(wall["mean_intensity"]),
                    ),
                ))
        return boundaries

    @staticmethod
    def _robust_line_support(
        points: Sequence[Point],
        seed_indices: Sequence[int],
        *,
        seed_slope: Optional[float] = None,
    ) -> Tuple[int, ...]:
        seed = [points[index] for index in seed_indices]
        if seed_slope is None:
            line = _least_squares_line(seed)
            if line is None:
                return ()
            slope, _ = line
        else:
            slope = seed_slope
        if not math.isfinite(slope):
            return ()
        intercept = _median([
            point[1] - slope * point[0] for point in seed
        ])
        distance_scale = math.sqrt(1.0 + slope * slope)
        return tuple(
            index
            for index, point in enumerate(points)
            if abs(point[1] - (intercept + slope * point[0]))
            / distance_scale <= LINE_TOLERANCE_M + 1e-12
        )

    @staticmethod
    def _wall_lateral_at(wall: Mapping[str, object], forward: float) -> float:
        start = wall["start"]
        end = wall["end"]
        slope = (
            (float(end[1]) - float(start[1]))
            / (float(end[0]) - float(start[0]))
        )
        return float(start[1]) + slope * (forward - float(start[0]))

    @classmethod
    def _has_boundary_prominence(
        cls,
        points: Sequence[Point],
        walls: Sequence[Mapping[str, object]],
        overlap_start: float,
        overlap_end: float,
    ) -> bool:
        overlap_points = [
            point for point in points
            if overlap_start - 1e-12
            <= point[0]
            <= overlap_end + 1e-12
        ]
        if not overlap_points:
            return False
        boundary_points = sum(
            min(
                abs(
                    point[1]
                    - cls._wall_lateral_at(wall, point[0])
                )
                for wall in walls
            ) <= BOUNDARY_PROMINENCE_BAND_M + 1e-12
            for point in overlap_points
        )
        return (
            boundary_points / len(overlap_points)
            >= MIN_BOUNDARY_PROMINENCE
        )

    @classmethod
    def _pair_candidates(
        cls,
        *,
        primary_points: Sequence[Point],
        secondary_points: Sequence[Point],
        primary_min_support: int,
        primary_min_span_m: float,
        primary_min_clearance_m: float,
        primary_min_mean_intensity: float,
        secondary_min_support: int,
        secondary_min_span_m: float,
        secondary_min_clearance_m: float,
        secondary_min_mean_intensity: float,
        require_disjoint_support: bool,
    ) -> List[Tuple[Tuple[object, ...], Dict[str, object]]]:
        pairs: List[Tuple[Tuple[object, ...], Dict[str, object]]] = []
        for left_index, left in enumerate(primary_points):
            for right in primary_points[left_index + 1:]:
                delta_forward = right[0] - left[0]
                delta_lateral = right[1] - left[1]
                pair_span = math.hypot(delta_forward, delta_lateral)
                if pair_span < primary_min_span_m:
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
                primary_inliers = tuple(
                    index
                    for index, point in enumerate(primary_points)
                    if abs(
                        normal_forward * point[0]
                        + normal_lateral * point[1]
                        - normal_offset
                    ) <= LINE_TOLERANCE_M + 1e-12
                )
                if len(primary_inliers) < primary_min_support:
                    continue
                primary_projections = [
                    unit_forward * primary_points[index][0]
                    + unit_lateral * primary_points[index][1]
                    for index in primary_inliers
                ]
                primary_span = max(primary_projections) - min(
                    primary_projections
                )
                if primary_span < primary_min_span_m:
                    continue
                normal_offset = _median([
                    normal_forward * primary_points[index][0]
                    + normal_lateral * primary_points[index][1]
                    for index in primary_inliers
                ])
                if abs(normal_offset) < primary_min_clearance_m:
                    continue
                primary = cls._wall_payload(
                    heading,
                    normal_offset,
                    [primary_points[index] for index in primary_inliers],
                )
                primary_mean_intensity = float(primary["mean_intensity"])
                if primary_mean_intensity < primary_min_mean_intensity:
                    continue
                secondary_offsets = [
                    normal_forward * point[0] + normal_lateral * point[1]
                    for point in secondary_points
                ]
                seen_support = set()
                for seed_offset in secondary_offsets:
                    secondary_inliers = tuple(
                        index
                        for index, offset in enumerate(secondary_offsets)
                        if abs(offset - seed_offset) <= LINE_TOLERANCE_M + 1e-12
                    )
                    if secondary_inliers in seen_support:
                        continue
                    seen_support.add(secondary_inliers)
                    if len(secondary_inliers) < secondary_min_support:
                        continue
                    if (
                        require_disjoint_support
                        and not set(primary_inliers).isdisjoint(
                            secondary_inliers
                        )
                    ):
                        continue
                    secondary_projections = [
                        unit_forward * secondary_points[index][0]
                        + unit_lateral * secondary_points[index][1]
                        for index in secondary_inliers
                    ]
                    secondary_span = max(secondary_projections) - min(
                        secondary_projections
                    )
                    if secondary_span < secondary_min_span_m:
                        continue
                    secondary_offset = _median([
                        secondary_offsets[index]
                        for index in secondary_inliers
                    ])
                    if abs(secondary_offset) < secondary_min_clearance_m:
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
                        [
                            secondary_points[index]
                            for index in secondary_inliers
                        ],
                    )
                    secondary_mean_intensity = float(
                        secondary["mean_intensity"]
                    )
                    if (
                        secondary_mean_intensity
                        < secondary_min_mean_intensity
                    ):
                        continue
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
                    confidence = min(
                        1.0,
                        min(1.0, support_total / 8.0) * 0.50
                        + min(1.0, minimum_span / 0.30) * 0.25
                        + mean_intensity * 0.25,
                    )
                    intensity_priority = (
                        min(
                            primary_mean_intensity,
                            secondary_mean_intensity,
                        )
                        if require_disjoint_support
                        else primary_mean_intensity
                    )
                    pair_key = (
                        -intensity_priority,
                        -confidence,
                        -support_total,
                        -minimum_span,
                        width,
                        abs(heading),
                        offsets[0],
                    )
                    pairs.append((pair_key, {
                        "geometry": "parallel",
                        "measured_heading_deg": heading,
                        "width_m": width,
                        "confidence": confidence,
                        "walls": walls,
                    }))
        return pairs

    @classmethod
    def _converging_candidate(
        cls,
        heatmap_points: Sequence[Point],
    ) -> Optional[Dict[str, object]]:
        rows: Dict[float, List[Point]] = {}
        for point in heatmap_points:
            if point[2] < HEATMAP_ONLY_MIN_MEAN_INTENSITY:
                continue
            rows.setdefault(point[0], []).append(point)
        boundaries: List[Tuple[float, Point, Point, float]] = []
        for forward_m, row in sorted(rows.items()):
            left = [
                point for point in row
                if point[1] <= -CONVERGING_MIN_SIDE_CLEARANCE_M
            ]
            right = [
                point for point in row
                if point[1] >= CONVERGING_MIN_SIDE_CLEARANCE_M
            ]
            if not left or not right:
                continue
            left_inner = max(left, key=lambda point: point[1])
            right_inner = min(right, key=lambda point: point[1])
            width_m = right_inner[1] - left_inner[1]
            if width_m <= 0.0 or width_m > MAX_CORRIDOR_WIDTH_M:
                continue
            boundaries.append((forward_m, left_inner, right_inner, width_m))

        runs: List[List[Tuple[float, Point, Point, float]]] = []
        for start_index in range(len(boundaries)):
            run = [boundaries[start_index]]
            narrowest_width = run[0][3]
            for boundary in boundaries[start_index + 1:]:
                previous = run[-1]
                if boundary[0] - previous[0] > CONVERGING_MAX_ROW_GAP_M:
                    break
                if (
                    boundary[3] - narrowest_width
                    > CONVERGING_MAX_WIDTH_REOPEN_M + 1e-12
                ):
                    break
                run.append(boundary)
                narrowest_width = min(narrowest_width, boundary[3])
            bottleneck_index = min(
                range(len(run)),
                key=lambda index: run[index][3],
            )
            run = run[:bottleneck_index + 1]
            if (
                len(run) >= CONVERGING_MIN_ROWS
                and run[0][3] - run[-1][3]
                >= CONVERGING_MIN_WIDTH_REDUCTION_M
            ):
                runs.append(run)
        if not runs:
            return None
        run = min(
            runs,
            key=lambda values: (
                values[-1][0],
                values[-1][3],
                -len(values),
                -(values[-1][0] - values[0][0]),
            ),
        )
        walls = [
            cls._anchored_wall_payload(
                [boundary[side_index] for boundary in run],
                run[-1][side_index],
            )
            for side_index in (1, 2)
        ]
        max_line_residual_m = (
            CONVERGING_SHORT_RUN_MAX_LINE_RESIDUAL_M
            if len(run) == CONVERGING_MIN_ROWS
            else CONVERGING_MAX_LINE_RESIDUAL_M
        )
        if any(
            abs(float(wall["heading_deg"])) > MAX_ABS_WALL_HEADING_DEG
            or float(wall["max_residual_m"]) > max_line_residual_m
            for wall in walls
        ):
            return None
        if not cls._has_boundary_prominence(
            heatmap_points,
            walls,
            run[0][0],
            run[-1][0],
        ):
            return None
        near_width = float(walls[1]["start"][1]) - float(
            walls[0]["start"][1]
        )
        bottleneck_width = float(walls[1]["end"][1]) - float(
            walls[0]["end"][1]
        )
        if (
            bottleneck_width < 0.0
            or bottleneck_width > MAX_CORRIDOR_WIDTH_M
            or near_width - bottleneck_width
            < CONVERGING_MIN_WIDTH_REDUCTION_M
        ):
            return None
        near_center = (
            (float(walls[0]["start"][0]) + float(walls[1]["start"][0])) / 2,
            (float(walls[0]["start"][1]) + float(walls[1]["start"][1])) / 2,
        )
        far_center = (
            (float(walls[0]["end"][0]) + float(walls[1]["end"][0])) / 2,
            (float(walls[0]["end"][1]) + float(walls[1]["end"][1])) / 2,
        )
        measured_heading = math.degrees(math.atan2(
            far_center[1] - near_center[1],
            far_center[0] - near_center[0],
        ))
        if abs(measured_heading) > MAX_ABS_WALL_HEADING_DEG:
            return None
        mean_intensity = sum(
            float(wall["mean_intensity"]) for wall in walls
        ) / 2.0
        minimum_span = min(float(wall["span_m"]) for wall in walls)
        confidence = min(
            1.0,
            min(1.0, len(run) / 6.0) * 0.45
            + min(1.0, minimum_span / 0.25) * 0.25
            + mean_intensity * 0.20
            + min(1.0, (near_width - bottleneck_width) / 0.20) * 0.10,
        )
        return {
            "geometry": "converging",
            "measured_heading_deg": measured_heading,
            "width_m": bottleneck_width,
            "confidence": confidence,
            "walls": walls,
        }

    @staticmethod
    def _independent_wall_payload(support: Sequence[Point]) -> Dict[str, object]:
        line = _least_squares_line(support)
        if line is None:
            raise ValueError("wall support must span multiple forward rows")
        slope, intercept = line
        start_forward = min(point[0] for point in support)
        end_forward = max(point[0] for point in support)
        start_lateral = intercept + slope * start_forward
        end_lateral = intercept + slope * end_forward
        heading_deg = math.degrees(math.atan(slope))
        heading_rad = math.radians(heading_deg)
        distance_scale = math.sqrt(1.0 + slope * slope)
        return {
            "heading_deg": heading_deg,
            "normal_offset_m": intercept * math.cos(heading_rad),
            "support_cells": len(support),
            "span_m": math.hypot(
                end_forward - start_forward,
                end_lateral - start_lateral,
            ),
            "mean_intensity": sum(point[2] for point in support) / len(support),
            "max_residual_m": max(
                abs(point[1] - (intercept + slope * point[0]))
                / distance_scale
                for point in support
            ),
            "start": [start_forward, start_lateral],
            "end": [end_forward, end_lateral],
            "_support": tuple(sorted(support)),
        }

    @staticmethod
    def _anchored_wall_payload(
        support: Sequence[Point],
        endpoint: Point,
    ) -> Dict[str, object]:
        line = _least_squares_line(support)
        if line is None:
            raise ValueError("wall support must span multiple forward rows")
        slope, _ = line
        start_forward = min(point[0] for point in support)
        end_forward = endpoint[0]
        end_lateral = endpoint[1]
        intercept = end_lateral - slope * end_forward
        start_lateral = intercept + slope * start_forward
        heading_deg = math.degrees(math.atan(slope))
        heading_rad = math.radians(heading_deg)
        distance_scale = math.sqrt(1.0 + slope * slope)
        return {
            "heading_deg": heading_deg,
            "normal_offset_m": intercept * math.cos(heading_rad),
            "support_cells": len(support),
            "span_m": math.hypot(
                end_forward - start_forward,
                end_lateral - start_lateral,
            ),
            "mean_intensity": sum(point[2] for point in support) / len(support),
            "max_residual_m": max(
                abs(point[1] - (intercept + slope * point[0]))
                / distance_scale
                for point in support
            ),
            "start": [start_forward, start_lateral],
            "end": [end_forward, end_lateral],
            "_support": tuple(sorted(support)),
        }

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
