"""Deterministic, profile-bound radar self-clutter calibration."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple

from common.radar_geometry import RadarAxes
from common.sensor_contract import RadarFrame, RadarHeatmap


POINT_VOXEL_M = 0.02
POINT_CLUSTER_MIN_FRACTION = 0.60
POINT_CLUSTER_MIN_RADIUS_M = 0.025
POINT_CLUSTER_MAX_RADIUS_M = 0.08
POINT_CALIBRATION_MAX_RANGE_M = 0.15


@dataclass(frozen=True)
class ClutterPointCluster:
    forward_m: float
    lateral_m: float
    height_m: float
    radius_m: float
    observation_fraction: float


@dataclass(frozen=True)
class RadarClutterModel:
    schema_version: int
    calibration_id: str
    profile_id: str
    axes: RadarAxes
    range_bins: int
    azimuth_bins: int
    range_step_m: float
    motion_mode: str
    point_clusters: Tuple[ClutterPointCluster, ...]
    heatmap_median_db: Tuple[float, ...]
    heatmap_mad_db: Tuple[float, ...]

    def to_dict(self) -> Dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "calibration_id": self.calibration_id,
            "profile_id": self.profile_id,
            "axes": self.axes.to_dict(),
            "range_bins": self.range_bins,
            "azimuth_bins": self.azimuth_bins,
            "range_step_m": self.range_step_m,
            "motion_mode": self.motion_mode,
            "point_clusters": [
                asdict(value) for value in self.point_clusters
            ],
            "heatmap_median_db": list(self.heatmap_median_db),
            "heatmap_mad_db": list(self.heatmap_mad_db),
        }

    def canonical_bytes(self) -> bytes:
        return json.dumps(
            self.to_dict(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")

    def binding_status(
        self,
        profile_id: Optional[str],
        heatmap: Optional[RadarHeatmap],
        axes: RadarAxes,
    ) -> str:
        if heatmap is None:
            return "heatmap_missing"
        matches = (
            profile_id == self.profile_id
            and heatmap.range_bins == self.range_bins
            and heatmap.azimuth_bins == self.azimuth_bins
            and math.isclose(
                heatmap.range_step_m,
                self.range_step_m,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            and heatmap.motion_mode == self.motion_mode
            and axes.signature() == self.axes.signature()
        )
        return "ok" if matches else "profile_mismatch"

    def matches_point(
        self,
        forward_m: float,
        lateral_m: float,
        height_m: float,
    ) -> bool:
        return any(
            math.dist(
                (forward_m, lateral_m, height_m),
                (
                    cluster.forward_m,
                    cluster.lateral_m,
                    cluster.height_m,
                ),
            )
            <= cluster.radius_m
            for cluster in self.point_clusters
        )


def _voxel(value: float) -> int:
    return math.floor(value / POINT_VOXEL_M)


def _adjacent_voxels(
    voxel: Tuple[int, int, int],
) -> Iterable[Tuple[int, int, int]]:
    x, y, z = voxel
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                yield x + dx, y + dy, z + dz


def _point_clusters(
    frames: Tuple[RadarFrame, ...],
    axes: RadarAxes,
) -> Tuple[ClutterPointCluster, ...]:
    observations: Dict[
        Tuple[int, int, int],
        List[Tuple[int, float, float, float]],
    ] = {}
    for frame_index, frame in enumerate(frames):
        for point in frame.points:
            forward_m, lateral_m = axes.map_point(point)
            location = (forward_m, lateral_m, point.z_m)
            if math.dist((0.0, 0.0, 0.0), location) > (
                POINT_CALIBRATION_MAX_RANGE_M
            ):
                continue
            voxel = tuple(_voxel(value) for value in location)
            observations.setdefault(voxel, []).append(
                (frame_index, *location)
            )

    unvisited = set(observations)
    components: List[Set[Tuple[int, int, int]]] = []
    while unvisited:
        start = min(unvisited)
        unvisited.remove(start)
        component = {start}
        pending = [start]
        while pending:
            current = pending.pop()
            for neighbor in _adjacent_voxels(current):
                if neighbor in unvisited:
                    unvisited.remove(neighbor)
                    component.add(neighbor)
                    pending.append(neighbor)
        components.append(component)

    clusters = []
    for component in components:
        values = [
            value
            for voxel in sorted(component)
            for value in observations[voxel]
        ]
        frame_fraction = len({value[0] for value in values}) / len(frames)
        if frame_fraction < POINT_CLUSTER_MIN_FRACTION:
            continue
        center = tuple(
            statistics.median(value[dimension] for value in values)
            for dimension in (1, 2, 3)
        )
        observed_radius = max(
            math.dist(center, value[1:]) for value in values
        )
        radius = min(
            POINT_CLUSTER_MAX_RADIUS_M,
            max(POINT_CLUSTER_MIN_RADIUS_M, observed_radius),
        )
        clusters.append(
            ClutterPointCluster(
                forward_m=center[0],
                lateral_m=center[1],
                height_m=center[2],
                radius_m=radius,
                observation_fraction=frame_fraction,
            )
        )
    return tuple(
        sorted(
            clusters,
            key=lambda value: (
                value.forward_m,
                value.lateral_m,
                value.height_m,
            ),
        )
    )


def _payload_without_id(model: RadarClutterModel) -> bytes:
    payload = model.to_dict()
    del payload["calibration_id"]
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def build_clutter_model(
    frames: Iterable[RadarFrame],
    axes: RadarAxes,
    min_frames: int = 50,
) -> RadarClutterModel:
    if isinstance(min_frames, bool) or not isinstance(min_frames, int):
        raise ValueError("min_frames must be a positive integer")
    if min_frames < 1:
        raise ValueError("min_frames must be a positive integer")

    usable = tuple(
        frame
        for frame in frames
        if frame.complete and frame.heatmap is not None
    )
    if len(usable) < min_frames:
        raise ValueError(
            f"calibration requires at least {min_frames} complete "
            "heatmap frames"
        )

    first = usable[0]
    first_heatmap = first.heatmap
    assert first_heatmap is not None
    if first.profile_id is None:
        raise ValueError("radar calibration requires a profile_id")
    for frame in usable[1:]:
        heatmap = frame.heatmap
        assert heatmap is not None
        if frame.profile_id != first.profile_id:
            raise ValueError("mixed profile input")
        if (
            heatmap.range_bins != first_heatmap.range_bins
            or heatmap.azimuth_bins != first_heatmap.azimuth_bins
            or not math.isclose(
                heatmap.range_step_m,
                first_heatmap.range_step_m,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            or heatmap.motion_mode != first_heatmap.motion_mode
        ):
            raise ValueError("mixed heatmap shape, range step, or motion mode")

    cell_values: List[List[float]] = [
        [] for _ in range(len(first_heatmap.data))
    ]
    for frame in usable:
        heatmap = frame.heatmap
        assert heatmap is not None
        span = heatmap.ceiling_db - heatmap.floor_db
        for index, value in enumerate(heatmap.data):
            cell_values[index].append(
                heatmap.floor_db + value / 255.0 * span
            )
    medians = tuple(statistics.median(values) for values in cell_values)
    mads = tuple(
        statistics.median(abs(value - median) for value in values)
        for values, median in zip(cell_values, medians)
    )

    without_id = RadarClutterModel(
        schema_version=1,
        calibration_id="",
        profile_id=first.profile_id,
        axes=axes,
        range_bins=first_heatmap.range_bins,
        azimuth_bins=first_heatmap.azimuth_bins,
        range_step_m=first_heatmap.range_step_m,
        motion_mode=first_heatmap.motion_mode,
        point_clusters=_point_clusters(usable, axes),
        heatmap_median_db=medians,
        heatmap_mad_db=mads,
    )
    calibration_id = (
        "radar-clutter-"
        + hashlib.sha256(_payload_without_id(without_id)).hexdigest()[:16]
    )
    return RadarClutterModel(
        **{
            **without_id.__dict__,
            "calibration_id": calibration_id,
        }
    )


_MODEL_KEYS = {
    "schema_version",
    "calibration_id",
    "profile_id",
    "axes",
    "range_bins",
    "azimuth_bins",
    "range_step_m",
    "motion_mode",
    "point_clusters",
    "heatmap_median_db",
    "heatmap_mad_db",
}
_AXES_KEYS = {
    "forward_axis",
    "forward_sign",
    "lateral_axis",
    "lateral_sign",
    "lateral_positive",
    "frame",
}
_CLUSTER_KEYS = {
    "forward_m",
    "lateral_m",
    "height_m",
    "radius_m",
    "observation_fraction",
}


def _strict_object(
    pairs: List[Tuple[str, object]],
) -> Dict[str, object]:
    result: Dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_nonfinite(value: str) -> object:
    raise ValueError(f"nonfinite JSON number: {value}")


def _require_keys(
    value: object,
    expected: Set[str],
    label: str,
) -> Dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(
            f"{label} schema keys are invalid "
            f"(missing={missing}, extra={extra})"
        )
    return value


def _require_int_value(
    value: object,
    label: str,
    minimum: int = 1,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    if value < minimum:
        raise ValueError(f"{label} is out of range")
    return value


def _require_float_value(
    value: object,
    label: str,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _require_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _load_axes(value: object) -> RadarAxes:
    payload = _require_keys(value, _AXES_KEYS, "axes")
    if payload["lateral_positive"] != "right":
        raise ValueError("axes.lateral_positive must be right")
    if payload["frame"] != "robot_relative_uncalibrated":
        raise ValueError("axes.frame is invalid")
    return RadarAxes(
        forward_axis=_require_string(
            payload["forward_axis"],
            "axes.forward_axis",
        ),
        forward_sign=_require_int_value(
            payload["forward_sign"],
            "axes.forward_sign",
            minimum=-1,
        ),
        lateral_axis=_require_string(
            payload["lateral_axis"],
            "axes.lateral_axis",
        ),
        lateral_sign=_require_int_value(
            payload["lateral_sign"],
            "axes.lateral_sign",
            minimum=-1,
        ),
    )


def _load_clusters(value: object) -> Tuple[ClutterPointCluster, ...]:
    if not isinstance(value, list):
        raise ValueError("point_clusters must be an array")
    result = []
    for index, item in enumerate(value):
        payload = _require_keys(
            item,
            _CLUSTER_KEYS,
            f"point_clusters[{index}]",
        )
        radius = _require_float_value(
            payload["radius_m"],
            f"point_clusters[{index}].radius_m",
        )
        fraction = _require_float_value(
            payload["observation_fraction"],
            f"point_clusters[{index}].observation_fraction",
        )
        if radius <= 0:
            raise ValueError("point cluster radius_m must be positive")
        if not 0.0 <= fraction <= 1.0:
            raise ValueError(
                "point cluster observation_fraction is out of range"
            )
        result.append(
            ClutterPointCluster(
                forward_m=_require_float_value(
                    payload["forward_m"],
                    f"point_clusters[{index}].forward_m",
                ),
                lateral_m=_require_float_value(
                    payload["lateral_m"],
                    f"point_clusters[{index}].lateral_m",
                ),
                height_m=_require_float_value(
                    payload["height_m"],
                    f"point_clusters[{index}].height_m",
                ),
                radius_m=radius,
                observation_fraction=fraction,
            )
        )
    return tuple(result)


def _load_vector(
    value: object,
    label: str,
    expected_length: int,
) -> Tuple[float, ...]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be an array")
    if len(value) != expected_length:
        raise ValueError(
            f"{label} length must equal range_bins * azimuth_bins"
        )
    result = tuple(
        _require_float_value(item, f"{label}[{index}]")
        for index, item in enumerate(value)
    )
    if label == "heatmap_mad_db" and any(item < 0 for item in result):
        raise ValueError("heatmap_mad_db values must be non-negative")
    return result


def load_clutter_model(path: Path) -> RadarClutterModel:
    try:
        text = Path(path).read_bytes().decode("utf-8")
        parsed = json.loads(
            text,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_nonfinite,
        )
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid clutter model JSON: {exc}") from exc
    except ValueError:
        raise
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"invalid clutter model: {exc}") from exc

    payload = _require_keys(parsed, _MODEL_KEYS, "clutter model")
    schema_version = _require_int_value(
        payload["schema_version"],
        "schema_version",
    )
    if schema_version != 1:
        raise ValueError("unsupported clutter model schema_version")
    range_bins = _require_int_value(payload["range_bins"], "range_bins")
    azimuth_bins = _require_int_value(
        payload["azimuth_bins"],
        "azimuth_bins",
    )
    range_step_m = _require_float_value(
        payload["range_step_m"],
        "range_step_m",
    )
    if range_step_m <= 0:
        raise ValueError("range_step_m must be positive")
    motion_mode = _require_string(payload["motion_mode"], "motion_mode")
    if motion_mode not in {"major", "minor"}:
        raise ValueError("motion_mode is invalid")
    cells = range_bins * azimuth_bins
    model = RadarClutterModel(
        schema_version=schema_version,
        calibration_id=_require_string(
            payload["calibration_id"],
            "calibration_id",
        ),
        profile_id=_require_string(payload["profile_id"], "profile_id"),
        axes=_load_axes(payload["axes"]),
        range_bins=range_bins,
        azimuth_bins=azimuth_bins,
        range_step_m=range_step_m,
        motion_mode=motion_mode,
        point_clusters=_load_clusters(payload["point_clusters"]),
        heatmap_median_db=_load_vector(
            payload["heatmap_median_db"],
            "heatmap_median_db",
            cells,
        ),
        heatmap_mad_db=_load_vector(
            payload["heatmap_mad_db"],
            "heatmap_mad_db",
            cells,
        ),
    )
    expected_id = (
        "radar-clutter-"
        + hashlib.sha256(_payload_without_id(model)).hexdigest()[:16]
    )
    if model.calibration_id != expected_id:
        raise ValueError("calibration_id is inconsistent with model payload")
    return model
