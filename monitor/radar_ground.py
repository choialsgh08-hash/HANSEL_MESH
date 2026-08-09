from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Optional, Tuple


def _finite_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class RadarMountGeometry:
    sensor_height_m: float = 0.065
    sensor_pitch_deg: float = 0.0
    ground_rejection_height_m: float = 0.035
    ground_uncertainty_m: float = 0.015
    filter_range_m: float = 0.60

    def __post_init__(self) -> None:
        sensor_height_m = _finite_real(
            "sensor_height_m", self.sensor_height_m
        )
        sensor_pitch_deg = _finite_real(
            "sensor_pitch_deg", self.sensor_pitch_deg
        )
        ground_rejection_height_m = _finite_real(
            "ground_rejection_height_m",
            self.ground_rejection_height_m,
        )
        ground_uncertainty_m = _finite_real(
            "ground_uncertainty_m",
            self.ground_uncertainty_m,
        )
        filter_range_m = _finite_real("filter_range_m", self.filter_range_m)
        if sensor_height_m < 0.0:
            raise ValueError("sensor_height_m must be non-negative")
        if ground_rejection_height_m < 0.0:
            raise ValueError(
                "ground_rejection_height_m must be non-negative"
            )
        if ground_rejection_height_m >= sensor_height_m:
            raise ValueError(
                "ground_rejection_height_m must be below sensor_height_m"
            )
        if ground_uncertainty_m < 0.0:
            raise ValueError("ground_uncertainty_m must be non-negative")
        if ground_rejection_height_m + ground_uncertainty_m >= sensor_height_m:
            raise ValueError(
                "effective ground rejection height must be below sensor_height_m"
            )
        if filter_range_m <= 0.0:
            raise ValueError("filter_range_m must be positive")
        object.__setattr__(self, "sensor_height_m", sensor_height_m)
        object.__setattr__(self, "sensor_pitch_deg", sensor_pitch_deg)
        object.__setattr__(
            self,
            "ground_rejection_height_m",
            ground_rejection_height_m,
        )
        object.__setattr__(self, "ground_uncertainty_m", ground_uncertainty_m)
        object.__setattr__(self, "filter_range_m", filter_range_m)

    def robot_coordinates(
        self,
        forward_m: float,
        lateral_m: float,
        height_m: float,
    ) -> Tuple[float, float, float]:
        forward = _finite_real("forward_m", forward_m)
        lateral = _finite_real("lateral_m", lateral_m)
        height = _finite_real("height_m", height_m)
        pitch = math.radians(self.sensor_pitch_deg)
        cosine = math.cos(pitch)
        sine = math.sin(pitch)
        return (
            cosine * forward - sine * height,
            lateral,
            self.sensor_height_m + sine * forward + cosine * height,
        )

    def is_ground(
        self,
        forward_m: float,
        lateral_m: float,
        height_m: Optional[float],
    ) -> bool:
        return self.ground_classification(
            forward_m,
            lateral_m,
            height_m,
        ) is not None

    def ground_classification(
        self,
        forward_m: float,
        lateral_m: float,
        height_m: Optional[float],
    ) -> Optional[str]:
        try:
            if height_m is None:
                return None
            robot_forward, robot_lateral, robot_height = (
                self.robot_coordinates(forward_m, lateral_m, height_m)
            )
        except (TypeError, ValueError):
            return None
        planar_distance = math.hypot(robot_forward, robot_lateral)
        if robot_forward < 0.0 or planar_distance > self.filter_range_m:
            return None
        if robot_height <= self.ground_rejection_height_m:
            return "ground"
        if robot_height <= (
            self.ground_rejection_height_m + self.ground_uncertainty_m
        ):
            return "ground_fringe"
        return None

    def payload(self) -> dict[str, object]:
        return {
            "sensor_height_m": self.sensor_height_m,
            "sensor_pitch_deg": self.sensor_pitch_deg,
            "ground_rejection_height_m": self.ground_rejection_height_m,
            "ground_uncertainty_m": self.ground_uncertainty_m,
            "effective_rejection_height_m": (
                self.ground_rejection_height_m + self.ground_uncertainty_m
            ),
            "filter_range_m": self.filter_range_m,
            "pitch_convention": "positive_up",
            "status": "configured",
        }
