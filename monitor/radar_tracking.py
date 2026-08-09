from __future__ import annotations

import math
from dataclasses import dataclass
from statistics import median
from typing import Optional, Sequence, Tuple


DEFAULT_RANGE_STEP_M = 0.09765625
VERIFIED_NEAR_8HZ_PROFILE_ID = (
    "lsdk-05.05.04.02-presence-near-"
    "heatmap16-elev8-cfar12-8hz-v2"
)
POINT_HISTORY_FRAMES = 5
POINT_MAP_CONFIRM_HITS = 3
POINT_HAZARD_WINDOW_FRAMES = 3
POINT_HAZARD_CONFIRM_HITS = 2
POINT_MIN_QUALITY = 0.50
POINT_STRONG_QUALITY = 0.70
POINT_POSITION_RESIDUAL_LIMIT_M = 0.13
POINT_AZIMUTH_ASSOCIATION_DEG = 19.0
POINT_AZIMUTH_UNCERTAINTY_DEG = 9.5


@dataclass(frozen=True)
class RobustPointEstimate:
    forward_m: float
    lateral_m: float
    height_m: Optional[float]
    position_jitter_m: float
    latest_inlier: bool
    inlier_count: int
    rejected_count: int


def _positive_finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value) and value > 0.0


def resolve_range_step_m(
    heatmap_range_step_m: Optional[float],
    calibrated_range_step_m: Optional[float],
    calibration_profile_matches: bool,
    frame_profile_id: Optional[str],
    synthetic: bool,
) -> Optional[float]:
    if _positive_finite(heatmap_range_step_m):
        return heatmap_range_step_m
    if calibration_profile_matches and _positive_finite(
        calibrated_range_step_m
    ):
        return calibrated_range_step_m
    if synthetic or frame_profile_id == VERIFIED_NEAR_8HZ_PROFILE_ID:
        return DEFAULT_RANGE_STEP_M
    return None


def point_quality_score(
    snr_db: Optional[float], noise_db: Optional[float]
) -> float:
    del noise_db
    return 0.50 if snr_db is None else max(0.05, min(1.0, snr_db / 30.0))


def range_gate_m(range_step_m: float) -> float:
    return max(0.10, 1.25 * range_step_m)


def lateral_gate_m(distance_m: float) -> float:
    return max(
        0.05,
        distance_m * math.tan(math.radians(POINT_AZIMUTH_ASSOCIATION_DEG)),
    )


def lateral_uncertainty_m(distance_m: float) -> float:
    return max(
        0.05,
        distance_m * math.tan(math.radians(POINT_AZIMUTH_UNCERTAINTY_DEG)),
    )


def point_association_gate(
    old_forward_m: float,
    old_lateral_m: float,
    new_forward_m: float,
    new_lateral_m: float,
    range_step_m: float,
) -> bool:
    old_range_m = math.hypot(old_forward_m, old_lateral_m)
    new_range_m = math.hypot(new_forward_m, new_lateral_m)
    if abs(old_range_m - new_range_m) > range_gate_m(range_step_m):
        return False
    return abs(old_lateral_m - new_lateral_m) <= lateral_gate_m(
        max(old_range_m, new_range_m)
    )


def robust_point_estimate(
    samples: Sequence[Tuple[float, float, Optional[float]]],
    residual_limit_m: float,
) -> RobustPointEstimate:
    if not samples:
        raise ValueError("samples must not be empty")
    preliminary_forward = median(sample[0] for sample in samples)
    preliminary_lateral = median(sample[1] for sample in samples)
    inliers = [
        sample for sample in samples
        if math.hypot(
            sample[0] - preliminary_forward,
            sample[1] - preliminary_lateral,
        ) <= residual_limit_m
    ]
    if not inliers:
        inliers = list(samples)
    forward_m = median(sample[0] for sample in inliers)
    lateral_m = median(sample[1] for sample in inliers)
    measured_heights = [sample[2] for sample in inliers if sample[2] is not None]
    height_m = median(measured_heights) if measured_heights else None
    jitter_m = max(
        math.hypot(sample[0] - forward_m, sample[1] - lateral_m)
        for sample in inliers
    )
    latest_inlier = samples[-1] in inliers
    return RobustPointEstimate(
        forward_m=forward_m,
        lateral_m=lateral_m,
        height_m=height_m,
        position_jitter_m=jitter_m,
        latest_inlier=latest_inlier,
        inlier_count=len(inliers),
        rejected_count=len(samples) - len(inliers),
    )


def height_band(height_m: Optional[float]) -> str:
    if height_m is None:
        return "unknown"
    if height_m < -0.10:
        return "below_sensor"
    if height_m > 0.10:
        return "above_sensor"
    return "sensor_level"
