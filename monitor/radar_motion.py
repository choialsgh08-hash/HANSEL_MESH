from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple


MOTION_MIN_MATCHES = 5
MOTION_MIN_MATCH_RATIO = 0.70
MOTION_MAX_RMSE_M = 0.05
MOTION_MAX_TRANSLATION_M = 0.08
MOTION_MAX_YAW_DEG = 8.0
MOTION_STABILITY_TRANSLATION_M = 0.05
MOTION_STABILITY_YAW_DEG = 2.0
MOTION_CONFIDENCE_RMSE_SCALE_M = 0.16


Point2D = Tuple[float, float]
Correspondence = Tuple[Point2D, Point2D]


def _rotate(point: Point2D, yaw_rad: float) -> Point2D:
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        cosine * point[0] - sine * point[1],
        sine * point[0] + cosine * point[1],
    )


def _empty_motion(status: str) -> dict:
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


class RadarMotionEstimator:
    def __init__(self, stabilization_frames=2):
        self.stabilization_frames = stabilization_frames
        self._previous_transform: Optional[Tuple[float, float, float]] = None
        self._stable_count = 0

    def reset(self) -> None:
        self._previous_transform = None
        self._stable_count = 0

    def update(self, correspondences, eligible_anchor_count) -> dict:
        if (
            isinstance(eligible_anchor_count, bool)
            or not isinstance(eligible_anchor_count, int)
            or eligible_anchor_count < 0
        ):
            self.reset()
            return _empty_motion("invalid_input")

        try:
            pairs: Sequence[Correspondence] = tuple(correspondences)
        except (TypeError, ValueError):
            self.reset()
            return _empty_motion("invalid_input")

        if len(pairs) > eligible_anchor_count:
            self.reset()
            return _empty_motion("invalid_input")
        if len(pairs) < MOTION_MIN_MATCHES:
            self.reset()
            return _empty_motion("insufficient_points")

        validated: List[Correspondence] = []
        try:
            for old, new in pairs:
                old_point = (float(old[0]), float(old[1]))
                new_point = (float(new[0]), float(new[1]))
                if not all(math.isfinite(value) for value in (*old_point, *new_point)):
                    self.reset()
                    return _empty_motion("invalid_input")
                validated.append((old_point, new_point))
        except (IndexError, OverflowError, TypeError, ValueError):
            self.reset()
            return _empty_motion("invalid_input")

        matched_points = len(validated)
        match_ratio = matched_points / eligible_anchor_count
        old_centroid = (
            sum(old[0] for old, _ in validated) / matched_points,
            sum(old[1] for old, _ in validated) / matched_points,
        )
        new_centroid = (
            sum(new[0] for _, new in validated) / matched_points,
            sum(new[1] for _, new in validated) / matched_points,
        )
        centered_pairs = [
            (
                (old[0] - old_centroid[0], old[1] - old_centroid[1]),
                (new[0] - new_centroid[0], new[1] - new_centroid[1]),
            )
            for old, new in validated
        ]
        cross = sum(
            ox * ny - oy * nx
            for (ox, oy), (nx, ny) in centered_pairs
        )
        dot = sum(
            ox * nx + oy * ny
            for (ox, oy), (nx, ny) in centered_pairs
        )
        yaw_rad = math.atan2(cross, dot)
        rotated_centroid = _rotate(old_centroid, yaw_rad)
        translation = (
            new_centroid[0] - rotated_centroid[0],
            new_centroid[1] - rotated_centroid[1],
        )
        try:
            residual_squares = []
            for old, new in validated:
                rotated = _rotate(old, yaw_rad)
                residual_squares.append(
                    (rotated[0] + translation[0] - new[0]) ** 2
                    + (rotated[1] + translation[1] - new[1]) ** 2
                )
            rmse = math.sqrt(sum(residual_squares) / matched_points)
        except OverflowError:
            self.reset()
            return _empty_motion("invalid_input")
        finite_estimate = all(
            math.isfinite(value)
            for value in (*translation, yaw_rad, rmse, match_ratio)
        )
        if not finite_estimate:
            self.reset()
            return _empty_motion("invalid_input")

        ego_yaw_rad = -yaw_rad
        ego_forward_m, ego_lateral_m = _rotate(
            (-translation[0], -translation[1]), ego_yaw_rad,
        )
        yaw_deg = math.degrees(yaw_rad)
        confidence = max(0.0, min(
            1.0,
            match_ratio * max(
                0.0,
                1.0 - rmse / MOTION_CONFIDENCE_RMSE_SCALE_M,
            ),
        ))
        result = _empty_motion("ok")
        result.update(
            {
                "confidence": round(confidence, 3),
                "matched_points": matched_points,
                "match_ratio": round(match_ratio, 3),
                "rmse_m": round(rmse, 4),
                "ego_forward_m": round(ego_forward_m, 4),
                "ego_lateral_m": round(ego_lateral_m, 4),
                "ego_yaw_deg": round(-yaw_deg, 3),
                "transform_forward_m": round(translation[0], 4),
                "transform_lateral_m": round(translation[1], 4),
                "transform_yaw_deg": round(yaw_deg, 3),
            }
        )

        if match_ratio < MOTION_MIN_MATCH_RATIO:
            result["status"] = "low_confidence"
            self.reset()
            return result
        if rmse > MOTION_MAX_RMSE_M:
            result["status"] = "high_rmse"
            self.reset()
            return result
        if (
            math.hypot(*translation) > MOTION_MAX_TRANSLATION_M
            or abs(yaw_deg) > MOTION_MAX_YAW_DEG
        ):
            result["status"] = "implausible_motion"
            self.reset()
            return result

        transform = (translation[0], translation[1], yaw_deg)
        if self._previous_transform is None:
            self._stable_count = 1
        else:
            translation_delta = math.hypot(
                transform[0] - self._previous_transform[0],
                transform[1] - self._previous_transform[1],
            )
            yaw_delta = abs(transform[2] - self._previous_transform[2])
            if (
                translation_delta > MOTION_STABILITY_TRANSLATION_M
                or yaw_delta > MOTION_STABILITY_YAW_DEG
            ):
                self._stable_count = 1
                self._previous_transform = transform
                result["status"] = "unstable"
                return result
            self._stable_count += 1
        self._previous_transform = transform

        if self._stable_count < self.stabilization_frames:
            result["status"] = "stabilizing"
            return result
        result["applied"] = True
        return result
