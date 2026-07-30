"""Shared TI/native radar axis mapping."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from common.sensor_contract import RadarPoint


@dataclass(frozen=True)
class RadarAxes:
    forward_axis: str = "y"
    forward_sign: int = 1
    lateral_axis: str = "x"
    lateral_sign: int = 1

    def __post_init__(self) -> None:
        if {self.forward_axis, self.lateral_axis} != {"x", "y"}:
            raise ValueError(
                "forward_axis and lateral_axis must be x/y and differ"
            )
        if (
            self.forward_sign not in {-1, 1}
            or self.lateral_sign not in {-1, 1}
        ):
            raise ValueError("axis signs must be -1 or 1")

    def map_point(self, point: RadarPoint) -> Tuple[float, float]:
        values = {"x": point.x_m, "y": point.y_m}
        return (
            values[self.forward_axis] * self.forward_sign,
            values[self.lateral_axis] * self.lateral_sign,
        )

    def to_dict(self) -> Dict[str, object]:
        return {
            "forward_axis": self.forward_axis,
            "forward_sign": self.forward_sign,
            "lateral_axis": self.lateral_axis,
            "lateral_sign": self.lateral_sign,
            "lateral_positive": "right",
            "frame": "robot_relative_uncalibrated",
        }

    def signature(self) -> str:
        return (
            f"{self.forward_sign:+d}{self.forward_axis}:forward,"
            f"{self.lateral_sign:+d}{self.lateral_axis}:right"
        )
