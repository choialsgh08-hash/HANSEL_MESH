"""ROS-independent planar odometry math for HANSEL Head."""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import math


TAU = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % TAU - math.pi


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return standard right-handed yaw from an (x, y, z, w) quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass(frozen=True)
class Pose2D:
    stamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    path_distance_m: float


@dataclass(frozen=True)
class PoseSample:
    pose: Pose2D
    provisional: bool


class YawReference:
    """Circular-average a stationary startup heading and report relative yaw."""

    def __init__(self, sample_count: int = 30, sign: float = 1.0) -> None:
        if sample_count <= 0:
            raise ValueError("sample_count must be positive")
        if sign == 0.0:
            raise ValueError("sign must be nonzero")
        self.sample_count = int(sample_count)
        self.sign = 1.0 if sign > 0.0 else -1.0
        self._samples: list[float] = []
        self._zero: float | None = None
        self._output_at_zero = 0.0

    @property
    def ready(self) -> bool:
        return self._zero is not None

    @property
    def collected(self) -> int:
        return len(self._samples)

    def reset(self, output_at_zero: float = 0.0) -> None:
        self._samples.clear()
        self._zero = None
        self._output_at_zero = wrap_angle(output_at_zero)

    def update(self, raw_yaw: float) -> float | None:
        if not math.isfinite(raw_yaw):
            return None
        if self._zero is None:
            self._samples.append(float(raw_yaw))
            if len(self._samples) < self.sample_count:
                return None
            sin_sum = sum(math.sin(value) for value in self._samples)
            cos_sum = sum(math.cos(value) for value in self._samples)
            self._zero = math.atan2(sin_sum, cos_sum)
        return wrap_angle(
            self._output_at_zero
            + self.sign * wrap_angle(float(raw_yaw) - self._zero)
        )


class PlanarOdometry:
    """Integrate actual cumulative encoder count deltas along IMU heading."""

    def __init__(
        self,
        wheel_diameter_m: float,
        encoder_cpr: float,
        left_encoder_sign: float = 1.0,
        right_encoder_sign: float = -1.0,
        left_distance_scale: float = 1.0,
        right_distance_scale: float = 1.0,
        max_dt_s: float = 0.25,
        history_min_step_m: float = 0.005,
        max_history_points: int = 3000,
    ) -> None:
        if wheel_diameter_m <= 0.0 or encoder_cpr <= 0.0:
            raise ValueError("wheel diameter and CPR must be positive")
        if left_encoder_sign == 0.0 or right_encoder_sign == 0.0:
            raise ValueError("encoder signs must be nonzero")
        if left_distance_scale <= 0.0 or right_distance_scale <= 0.0:
            raise ValueError("distance scales must be positive")
        if max_dt_s <= 0.0 or history_min_step_m <= 0.0:
            raise ValueError("time/history thresholds must be positive")
        if max_history_points < 2:
            raise ValueError("max_history_points must be at least 2")

        nominal_m_per_count = math.pi * wheel_diameter_m / encoder_cpr
        self.left_m_per_count = nominal_m_per_count * left_distance_scale
        self.right_m_per_count = nominal_m_per_count * right_distance_scale
        self.left_sign = 1.0 if left_encoder_sign > 0.0 else -1.0
        self.right_sign = 1.0 if right_encoder_sign > 0.0 else -1.0
        self.max_dt_s = float(max_dt_s)
        self.history_min_step_m = float(history_min_step_m)
        self.max_history_points = int(max_history_points)
        self.reset()

    def reset(self) -> None:
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.path_distance_m = 0.0
        self._last_stamp_s: float | None = None
        self._last_yaw_rad: float | None = None
        self._last_counts: tuple[int, int] | None = None
        self._encoder_epoch = None
        self.linear_velocity_mps = 0.0
        self._history: list[Pose2D] = []
        self._timed_history = deque(maxlen=self.max_history_points)

    @property
    def current_pose(self) -> Pose2D | None:
        if self._last_stamp_s is None:
            return None
        return Pose2D(
            self._last_stamp_s,
            self.x_m,
            self.y_m,
            self.yaw_rad,
            self.path_distance_m,
        )

    @property
    def history(self) -> tuple[Pose2D, ...]:
        return tuple(self._history)

    def update(
        self,
        stamp_s: float,
        left_count: int,
        right_count: int,
        yaw_rad: float,
        encoder_epoch: str = "",
    ) -> Pose2D | None:
        values = (stamp_s, left_count, right_count, yaw_rad)
        if not all(math.isfinite(value) for value in values):
            return None
        if int(left_count) != left_count or int(right_count) != right_count:
            return None

        stamp_s = float(stamp_s)
        yaw_rad = wrap_angle(yaw_rad)
        counts = (int(left_count), int(right_count))
        if self._last_counts is None or encoder_epoch != self._encoder_epoch:
            self.linear_velocity_mps = 0.0
            self._last_counts = counts
            self._encoder_epoch = encoder_epoch
            self._last_stamp_s = stamp_s
            self._last_yaw_rad = yaw_rad
            self.yaw_rad = yaw_rad
            pose = self.current_pose
            if pose is not None:
                self._history.append(pose)
                self._timed_history.append(pose)
            return pose

        dt = stamp_s - self._last_stamp_s
        if dt <= 0.0:
            return None

        # Nano counters are signed 32-bit values. A wrap is not a wheel reversal.
        deltas = tuple((new - old + 2**31) % 2**32 - 2**31
                       for new, old in zip(counts, self._last_counts))
        self._last_counts = counts
        left_m = self.left_sign * deltas[0] * self.left_m_per_count
        right_m = self.right_sign * deltas[1] * self.right_m_per_count
        ds = 0.5 * (left_m + right_m)
        self.linear_velocity_mps = ds / dt
        previous_yaw = self._last_yaw_rad if self._last_yaw_rad is not None else yaw_rad
        yaw_delta = wrap_angle(yaw_rad - previous_yaw)
        midpoint_yaw = wrap_angle(previous_yaw + 0.5 * yaw_delta)

        self.x_m += ds * math.cos(midpoint_yaw)
        self.y_m += ds * math.sin(midpoint_yaw)
        self.yaw_rad = yaw_rad
        self.path_distance_m += abs(ds)
        self._last_stamp_s = stamp_s
        self._last_yaw_rad = yaw_rad
        pose = self.current_pose
        if pose is not None:
            self._timed_history.append(pose)
            if (
                not self._history
                or pose.path_distance_m - self._history[-1].path_distance_m
                >= self.history_min_step_m
            ):
                self._history.append(pose)
                if len(self._history) > self.max_history_points:
                    del self._history[: len(self._history) - self.max_history_points]
        return pose

    def pause_counts(self) -> None:
        """Rebaseline after missing heading; do not invent the unobserved path."""
        self._last_counts = None
        self.linear_velocity_mps = 0.0

    def sample_behind(self, distance_m: float, stamp_s: float | None = None) -> PoseSample | None:
        """Interpolate a pose along the driven path behind the current Head pose."""
        if distance_m < 0.0:
            raise ValueError("distance_m must not be negative")
        current = self.current_pose
        if current is None or not self._history:
            return None
        if stamp_s is not None:
            samples = list(self._timed_history)
            if not samples or stamp_s < samples[0].stamp_s or stamp_s > samples[-1].stamp_s:
                return None
            current = samples[0]
            for after in samples[1:]:
                if after.stamp_s >= stamp_s:
                    span = after.stamp_s - current.stamp_s
                    if span > self.max_dt_s:
                        return None
                    ratio = 0.0 if span <= 0 else (stamp_s - current.stamp_s) / span
                    current = Pose2D(
                        stamp_s, current.x_m + ratio * (after.x_m - current.x_m),
                        current.y_m + ratio * (after.y_m - current.y_m),
                        wrap_angle(current.yaw_rad + ratio * wrap_angle(after.yaw_rad - current.yaw_rad)),
                        current.path_distance_m + ratio * (after.path_distance_m - current.path_distance_m),
                    )
                    break
                current = after
        if distance_m == 0.0:
            return PoseSample(current, provisional=False)

        target = current.path_distance_m - float(distance_m)
        provisional = target < self._history[0].path_distance_m
        if provisional:
            first = self._history[0]
            remaining = first.path_distance_m - target
            return PoseSample(Pose2D(
                current.stamp_s,
                first.x_m - remaining * math.cos(first.yaw_rad),
                first.y_m - remaining * math.sin(first.yaw_rad),
                first.yaw_rad, target,
            ), provisional=True)

        before = self._history[0]
        points = [p for p in self._history[1:] if p.stamp_s < current.stamp_s] + [current]
        for after in points:
            if after.path_distance_m < target:
                before = after
                continue
            span = after.path_distance_m - before.path_distance_m
            ratio = 0.0 if span <= 0.0 else (target - before.path_distance_m) / span
            yaw_delta = wrap_angle(after.yaw_rad - before.yaw_rad)
            pose = Pose2D(
                stamp_s=before.stamp_s + ratio * (after.stamp_s - before.stamp_s),
                x_m=before.x_m + ratio * (after.x_m - before.x_m),
                y_m=before.y_m + ratio * (after.y_m - before.y_m),
                yaw_rad=wrap_angle(before.yaw_rad + ratio * yaw_delta),
                path_distance_m=target,
            )
            return PoseSample(pose, provisional=False)
        return PoseSample(current, provisional=False)
