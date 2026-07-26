"""Versioned, hardware-independent sensor records for HANSEL_MESH.

The records in this module are deliberately small and dependency-free so the
same definitions can run on Raspberry Pi OS and on the Windows operator PC.
Fusion code must use ``header.monotonic_ns`` together with
``timestamp_source`` and the source-specific semantics of
``timestamp_uncertainty_ns``.  Wall-clock time is metadata only and must not
be used to integrate motion.  In particular, the current buffered UART source
stores a heuristic timing-quality scale in that field, not a strict bound.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Any, Dict, Mapping, Optional, Tuple, Union


SENSOR_SCHEMA_VERSION = 1
MAX_RADAR_POINTS = 8192
INT64_MIN = -(2**63)
INT64_MAX = 2**63 - 1

_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}$")
_DROP_PHASES = {
    "requested",
    "actuated",
    "physically_confirmed",
    "failed",
    "anchor_updated",
}
_HEALTH_STATES = {"ok", "degraded", "stale", "failed", "starting", "stopped"}
_RADAR_FRAME_TRANSITIONS = {
    "unknown",
    "first",
    "consecutive",
    "gap",
    "wrap",
    "duplicate",
    "reset_or_out_of_order",
}


def _require_mapping(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _require_id(value: object, name: str) -> str:
    if not isinstance(value, str) or not _ID_PATTERN.fullmatch(value):
        raise ValueError(f"{name} is invalid")
    return value


def validate_sensor_id(value: object, name: str = "identifier") -> str:
    """Validate and return one schema-compatible identifier."""

    return _require_id(value, name)


def _optional_id(value: object, name: str) -> Optional[str]:
    if value is None:
        return None
    return _require_id(value, name)


def _require_bool(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a boolean")
    return value


def _require_int(
    value: object,
    name: str,
    minimum: int = INT64_MIN,
    maximum: int = INT64_MAX,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if value < minimum or value > maximum:
        raise ValueError(f"{name} is out of range")
    return value


def _optional_int(
    value: object,
    name: str,
    minimum: int = INT64_MIN,
    maximum: int = INT64_MAX,
) -> Optional[int]:
    if value is None:
        return None
    return _require_int(value, name, minimum, maximum)


def _require_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _optional_float(value: object, name: str) -> Optional[float]:
    if value is None:
        return None
    return _require_float(value, name)


def _require_float_tuple(
    value: object,
    name: str,
    length: int,
) -> Tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain exactly {length} numbers")
    return tuple(
        _require_float(item, f"{name}[{index}]")
        for index, item in enumerate(value)
    )


def _optional_float_tuple(
    value: object,
    name: str,
    length: int,
) -> Optional[Tuple[float, ...]]:
    if value is None:
        return None
    return _require_float_tuple(value, name, length)


def _payload_value(payload: Mapping[str, Any], name: str) -> Any:
    if name not in payload:
        raise ValueError(f"payload.{name} is required")
    return payload[name]


@dataclass(frozen=True)
class SensorHeader:
    """Identity and host-clock observation shared by every sensor record.

    ``producer_id`` changes whenever a capture process restarts.  ``stream_id``
    is the stable logical name, for example ``radar/front`` or ``wheel/drive``.
    Sequence numbers start at one for each producer and stream.  The meaning
    of ``timestamp_uncertainty_ns`` is defined by ``timestamp_source``; it must
    not be assumed to be a confidence interval or upper bound.
    """

    mission_id: str
    unit_id: str
    boot_id: str
    producer_id: str
    stream_id: str
    seq: int
    monotonic_ns: int
    sensor_timestamp_ns: Optional[int] = None
    frame_id: Optional[str] = None
    calibration_id: Optional[str] = None
    wall_time_ns: Optional[int] = None
    timestamp_source: str = "host_capture"
    timestamp_uncertainty_ns: Optional[int] = None

    def __post_init__(self) -> None:
        _require_id(self.mission_id, "mission_id")
        _require_id(self.unit_id, "unit_id")
        _require_id(self.boot_id, "boot_id")
        _require_id(self.producer_id, "producer_id")
        _require_id(self.stream_id, "stream_id")
        _require_int(self.seq, "seq", 1)
        _require_int(self.monotonic_ns, "monotonic_ns", 0)
        _optional_int(self.sensor_timestamp_ns, "sensor_timestamp_ns", 0)
        _optional_id(self.frame_id, "frame_id")
        _optional_id(self.calibration_id, "calibration_id")
        _optional_int(self.wall_time_ns, "wall_time_ns", 0)
        _require_id(self.timestamp_source, "timestamp_source")
        _optional_int(
            self.timestamp_uncertainty_ns,
            "timestamp_uncertainty_ns",
            0,
        )


@dataclass(frozen=True)
class RadarPoint:
    x_m: float
    y_m: float
    z_m: float
    radial_velocity_mps: float
    snr_db: Optional[float] = None
    noise_db: Optional[float] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_m", _require_float(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _require_float(self.y_m, "y_m"))
        object.__setattr__(self, "z_m", _require_float(self.z_m, "z_m"))
        object.__setattr__(
            self,
            "radial_velocity_mps",
            _require_float(
                self.radial_velocity_mps,
                "radial_velocity_mps",
            ),
        )
        object.__setattr__(
            self,
            "snr_db",
            _optional_float(self.snr_db, "snr_db"),
        )
        object.__setattr__(
            self,
            "noise_db",
            _optional_float(self.noise_db, "noise_db"),
        )


@dataclass(frozen=True)
class RadarFrame:
    header: SensorHeader
    frame_number: int
    subframe_number: int
    complete: bool
    dropped_frames_since_previous: int
    points: Tuple[RadarPoint, ...]
    source_format: Optional[str] = None
    sdk_version: Optional[str] = None
    device_time_cycles: Optional[int] = None
    frame_transition: str = "unknown"
    profile_id: Optional[str] = None
    capture_baudrate: Optional[int] = None

    def __post_init__(self) -> None:
        if not isinstance(self.header, SensorHeader):
            raise ValueError("header must be a SensorHeader")
        _require_int(self.frame_number, "frame_number", 0, 2**32 - 1)
        _require_int(self.subframe_number, "subframe_number", 0, 2**32 - 1)
        _require_bool(self.complete, "complete")
        _require_int(
            self.dropped_frames_since_previous,
            "dropped_frames_since_previous",
            0,
        )
        if not isinstance(self.points, tuple):
            raise ValueError("points must be a tuple")
        if len(self.points) > MAX_RADAR_POINTS:
            raise ValueError(f"points exceeds limit of {MAX_RADAR_POINTS}")
        if any(not isinstance(point, RadarPoint) for point in self.points):
            raise ValueError("points must contain RadarPoint values")
        _optional_id(self.source_format, "source_format")
        _optional_id(self.sdk_version, "sdk_version")
        _optional_int(
            self.device_time_cycles,
            "device_time_cycles",
            0,
            2**32 - 1,
        )
        if self.frame_transition not in _RADAR_FRAME_TRANSITIONS:
            raise ValueError("frame_transition is invalid")
        _optional_id(self.profile_id, "profile_id")
        _optional_int(
            self.capture_baudrate,
            "capture_baudrate",
            1,
            10_000_000,
        )


@dataclass(frozen=True)
class ImuSample:
    header: SensorHeader
    specific_force_mps2: Tuple[float, float, float]
    angular_velocity_radps: Tuple[float, float, float]
    temperature_c: Optional[float] = None
    orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
    accel_covariance: Optional[Tuple[float, ...]] = None
    gyro_covariance: Optional[Tuple[float, ...]] = None
    orientation_covariance: Optional[Tuple[float, ...]] = None

    def __post_init__(self) -> None:
        if not isinstance(self.header, SensorHeader):
            raise ValueError("header must be a SensorHeader")
        object.__setattr__(
            self,
            "specific_force_mps2",
            _require_float_tuple(
                self.specific_force_mps2,
                "specific_force_mps2",
                3,
            ),
        )
        object.__setattr__(
            self,
            "angular_velocity_radps",
            _require_float_tuple(
                self.angular_velocity_radps,
                "angular_velocity_radps",
                3,
            ),
        )
        object.__setattr__(
            self,
            "temperature_c",
            _optional_float(self.temperature_c, "temperature_c"),
        )
        orientation = _optional_float_tuple(
            self.orientation_xyzw,
            "orientation_xyzw",
            4,
        )
        object.__setattr__(self, "orientation_xyzw", orientation)
        if orientation is not None:
            norm = math.sqrt(sum(component * component for component in orientation))
            if norm < 1e-9:
                raise ValueError("orientation_xyzw must not be a zero quaternion")
        object.__setattr__(
            self,
            "accel_covariance",
            _optional_float_tuple(
                self.accel_covariance,
                "accel_covariance",
                9,
            ),
        )
        object.__setattr__(
            self,
            "gyro_covariance",
            _optional_float_tuple(
                self.gyro_covariance,
                "gyro_covariance",
                9,
            ),
        )
        object.__setattr__(
            self,
            "orientation_covariance",
            _optional_float_tuple(
                self.orientation_covariance,
                "orientation_covariance",
                9,
            ),
        )


@dataclass(frozen=True)
class WheelState:
    header: SensorHeader
    left_ticks: int
    right_ticks: int
    sample_period_ns: int
    left_angular_velocity_radps: Optional[float] = None
    right_angular_velocity_radps: Optional[float] = None
    left_invalid_transitions: int = 0
    right_invalid_transitions: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.header, SensorHeader):
            raise ValueError("header must be a SensorHeader")
        _require_int(self.left_ticks, "left_ticks")
        _require_int(self.right_ticks, "right_ticks")
        _require_int(self.sample_period_ns, "sample_period_ns", 0)
        object.__setattr__(
            self,
            "left_angular_velocity_radps",
            _optional_float(
                self.left_angular_velocity_radps,
                "left_angular_velocity_radps",
            ),
        )
        object.__setattr__(
            self,
            "right_angular_velocity_radps",
            _optional_float(
                self.right_angular_velocity_radps,
                "right_angular_velocity_radps",
            ),
        )
        _require_int(
            self.left_invalid_transitions,
            "left_invalid_transitions",
            0,
        )
        _require_int(
            self.right_invalid_transitions,
            "right_invalid_transitions",
            0,
        )


@dataclass(frozen=True)
class DropEvent:
    header: SensorHeader
    event_id: str
    released_unit_id: str
    actuator_unit_id: str
    phase: str
    command_session_id: Optional[str] = None
    command_seq: Optional[int] = None
    anchor_keyframe_id: Optional[str] = None
    keyframe_to_unit_xyyaw: Optional[Tuple[float, float, float]] = None
    covariance_3x3: Optional[Tuple[float, ...]] = None
    estimation_method: Optional[str] = None
    confirmation_method: Optional[str] = None
    reason: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.header, SensorHeader):
            raise ValueError("header must be a SensorHeader")
        _require_id(self.event_id, "event_id")
        _require_id(self.released_unit_id, "released_unit_id")
        _require_id(self.actuator_unit_id, "actuator_unit_id")
        if self.phase not in _DROP_PHASES:
            raise ValueError("phase is invalid")
        _optional_id(self.command_session_id, "command_session_id")
        _optional_int(self.command_seq, "command_seq", 1)
        _optional_id(self.anchor_keyframe_id, "anchor_keyframe_id")
        object.__setattr__(
            self,
            "keyframe_to_unit_xyyaw",
            _optional_float_tuple(
                self.keyframe_to_unit_xyyaw,
                "keyframe_to_unit_xyyaw",
                3,
            ),
        )
        object.__setattr__(
            self,
            "covariance_3x3",
            _optional_float_tuple(
                self.covariance_3x3,
                "covariance_3x3",
                9,
            ),
        )
        if self.phase == "anchor_updated" and (
            self.anchor_keyframe_id is None
            or self.keyframe_to_unit_xyyaw is None
            or self.covariance_3x3 is None
        ):
            raise ValueError(
                "anchor_updated requires anchor_keyframe_id, "
                "keyframe_to_unit_xyyaw, and covariance_3x3"
            )
        _optional_id(self.estimation_method, "estimation_method")
        _optional_id(self.confirmation_method, "confirmation_method")
        if self.reason is not None:
            if not isinstance(self.reason, str) or len(self.reason) > 512:
                raise ValueError("reason is invalid")


@dataclass(frozen=True)
class SensorHealth:
    header: SensorHeader
    subject_stream_id: str
    status: str
    observed_rate_hz: Optional[float] = None
    last_sample_monotonic_ns: Optional[int] = None
    seq_gaps_total: int = 0
    parse_errors_total: int = 0
    producer_drops_total: int = 0
    writer_drops_total: int = 0
    device_discontinuities_total: int = 0
    queue_bytes: int = 0
    detail: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.header, SensorHeader):
            raise ValueError("header must be a SensorHeader")
        _require_id(self.subject_stream_id, "subject_stream_id")
        if self.status not in _HEALTH_STATES:
            raise ValueError("status is invalid")
        rate = _optional_float(self.observed_rate_hz, "observed_rate_hz")
        object.__setattr__(self, "observed_rate_hz", rate)
        if rate is not None and rate < 0:
            raise ValueError("observed_rate_hz must be non-negative")
        _optional_int(
            self.last_sample_monotonic_ns,
            "last_sample_monotonic_ns",
            0,
        )
        _require_int(self.seq_gaps_total, "seq_gaps_total", 0)
        _require_int(self.parse_errors_total, "parse_errors_total", 0)
        _require_int(self.producer_drops_total, "producer_drops_total", 0)
        _require_int(self.writer_drops_total, "writer_drops_total", 0)
        _require_int(
            self.device_discontinuities_total,
            "device_discontinuities_total",
            0,
        )
        _require_int(self.queue_bytes, "queue_bytes", 0)
        if self.detail is not None:
            if not isinstance(self.detail, str) or len(self.detail) > 1024:
                raise ValueError("detail is invalid")


SensorRecord = Union[RadarFrame, ImuSample, WheelState, DropEvent, SensorHealth]


def _header_to_dict(header: SensorHeader) -> Dict[str, Any]:
    return {
        "mission_id": header.mission_id,
        "unit_id": header.unit_id,
        "boot_id": header.boot_id,
        "producer_id": header.producer_id,
        "stream_id": header.stream_id,
        "seq": header.seq,
        "monotonic_ns": header.monotonic_ns,
        "sensor_timestamp_ns": header.sensor_timestamp_ns,
        "frame_id": header.frame_id,
        "calibration_id": header.calibration_id,
        "wall_time_ns": header.wall_time_ns,
        "timestamp_source": header.timestamp_source,
        "timestamp_uncertainty_ns": header.timestamp_uncertainty_ns,
    }


def _header_from_dict(value: object) -> SensorHeader:
    data = _require_mapping(value, "header")
    return SensorHeader(
        mission_id=_require_id(data.get("mission_id"), "header.mission_id"),
        unit_id=_require_id(data.get("unit_id"), "header.unit_id"),
        boot_id=_require_id(data.get("boot_id"), "header.boot_id"),
        producer_id=_require_id(data.get("producer_id"), "header.producer_id"),
        stream_id=_require_id(data.get("stream_id"), "header.stream_id"),
        seq=_require_int(data.get("seq"), "header.seq", 1),
        monotonic_ns=_require_int(
            data.get("monotonic_ns"),
            "header.monotonic_ns",
            0,
        ),
        sensor_timestamp_ns=_optional_int(
            data.get("sensor_timestamp_ns"),
            "header.sensor_timestamp_ns",
            0,
        ),
        frame_id=_optional_id(data.get("frame_id"), "header.frame_id"),
        calibration_id=_optional_id(
            data.get("calibration_id"),
            "header.calibration_id",
        ),
        wall_time_ns=_optional_int(
            data.get("wall_time_ns"),
            "header.wall_time_ns",
            0,
        ),
        timestamp_source=_require_id(
            data.get("timestamp_source", "host_capture"),
            "header.timestamp_source",
        ),
        timestamp_uncertainty_ns=_optional_int(
            data.get("timestamp_uncertainty_ns"),
            "header.timestamp_uncertainty_ns",
            0,
        ),
    )


def _point_to_dict(point: RadarPoint) -> Dict[str, Any]:
    return {
        "x_m": point.x_m,
        "y_m": point.y_m,
        "z_m": point.z_m,
        "radial_velocity_mps": point.radial_velocity_mps,
        "snr_db": point.snr_db,
        "noise_db": point.noise_db,
    }


def _point_from_dict(value: object, index: int) -> RadarPoint:
    data = _require_mapping(value, f"payload.points[{index}]")
    return RadarPoint(
        x_m=_require_float(data.get("x_m"), f"points[{index}].x_m"),
        y_m=_require_float(data.get("y_m"), f"points[{index}].y_m"),
        z_m=_require_float(data.get("z_m"), f"points[{index}].z_m"),
        radial_velocity_mps=_require_float(
            data.get("radial_velocity_mps"),
            f"points[{index}].radial_velocity_mps",
        ),
        snr_db=_optional_float(data.get("snr_db"), f"points[{index}].snr_db"),
        noise_db=_optional_float(
            data.get("noise_db"),
            f"points[{index}].noise_db",
        ),
    )


def record_to_dict(record: SensorRecord) -> Dict[str, Any]:
    """Convert a validated record into the versioned wire representation."""

    if isinstance(record, RadarFrame):
        record_type = "radar_frame"
        payload: Dict[str, Any] = {
            "frame_number": record.frame_number,
            "subframe_number": record.subframe_number,
            "complete": record.complete,
            "dropped_frames_since_previous": record.dropped_frames_since_previous,
            "points": [_point_to_dict(point) for point in record.points],
            "source_format": record.source_format,
            "sdk_version": record.sdk_version,
            "device_time_cycles": record.device_time_cycles,
            "frame_transition": record.frame_transition,
            "profile_id": record.profile_id,
            "capture_baudrate": record.capture_baudrate,
        }
    elif isinstance(record, ImuSample):
        record_type = "imu_sample"
        payload = {
            "specific_force_mps2": list(record.specific_force_mps2),
            "angular_velocity_radps": list(record.angular_velocity_radps),
            "temperature_c": record.temperature_c,
            "orientation_xyzw": (
                None
                if record.orientation_xyzw is None
                else list(record.orientation_xyzw)
            ),
            "accel_covariance": (
                None
                if record.accel_covariance is None
                else list(record.accel_covariance)
            ),
            "gyro_covariance": (
                None
                if record.gyro_covariance is None
                else list(record.gyro_covariance)
            ),
            "orientation_covariance": (
                None
                if record.orientation_covariance is None
                else list(record.orientation_covariance)
            ),
        }
    elif isinstance(record, WheelState):
        record_type = "wheel_state"
        payload = {
            "left_ticks": record.left_ticks,
            "right_ticks": record.right_ticks,
            "sample_period_ns": record.sample_period_ns,
            "left_angular_velocity_radps": record.left_angular_velocity_radps,
            "right_angular_velocity_radps": record.right_angular_velocity_radps,
            "left_invalid_transitions": record.left_invalid_transitions,
            "right_invalid_transitions": record.right_invalid_transitions,
        }
    elif isinstance(record, DropEvent):
        record_type = "drop_event"
        payload = {
            "event_id": record.event_id,
            "released_unit_id": record.released_unit_id,
            "actuator_unit_id": record.actuator_unit_id,
            "phase": record.phase,
            "command_session_id": record.command_session_id,
            "command_seq": record.command_seq,
            "anchor_keyframe_id": record.anchor_keyframe_id,
            "keyframe_to_unit_xyyaw": (
                None
                if record.keyframe_to_unit_xyyaw is None
                else list(record.keyframe_to_unit_xyyaw)
            ),
            "covariance_3x3": (
                None
                if record.covariance_3x3 is None
                else list(record.covariance_3x3)
            ),
            "estimation_method": record.estimation_method,
            "confirmation_method": record.confirmation_method,
            "reason": record.reason,
        }
    elif isinstance(record, SensorHealth):
        record_type = "sensor_health"
        payload = {
            "subject_stream_id": record.subject_stream_id,
            "status": record.status,
            "observed_rate_hz": record.observed_rate_hz,
            "last_sample_monotonic_ns": record.last_sample_monotonic_ns,
            "seq_gaps_total": record.seq_gaps_total,
            "parse_errors_total": record.parse_errors_total,
            "producer_drops_total": record.producer_drops_total,
            "writer_drops_total": record.writer_drops_total,
            "device_discontinuities_total": (
                record.device_discontinuities_total
            ),
            "queue_bytes": record.queue_bytes,
            "detail": record.detail,
        }
    else:
        raise TypeError(f"unsupported sensor record: {type(record).__name__}")

    return {
        "schema_version": SENSOR_SCHEMA_VERSION,
        "record_type": record_type,
        "header": _header_to_dict(record.header),
        "payload": payload,
    }


def record_from_dict(
    value: object,
    max_radar_points: int = MAX_RADAR_POINTS,
) -> SensorRecord:
    """Validate and decode one sensor record.

    Unknown fields in schema version 1 are ignored so optional fields can be
    added without breaking older readers.  Required fields and all known
    values remain strictly validated.
    """

    data = _require_mapping(value, "record")
    version = _require_int(data.get("schema_version"), "schema_version", 1)
    if version != SENSOR_SCHEMA_VERSION:
        raise ValueError(f"unsupported sensor schema version: {version}")
    record_type = data.get("record_type")
    if not isinstance(record_type, str):
        raise ValueError("record_type must be a string")
    header = _header_from_dict(data.get("header"))
    payload = _require_mapping(data.get("payload"), "payload")

    if record_type == "radar_frame":
        raw_points = _payload_value(payload, "points")
        if not isinstance(raw_points, list):
            raise ValueError("payload.points must be an array")
        if len(raw_points) > max_radar_points:
            raise ValueError(f"payload.points exceeds limit of {max_radar_points}")
        frame_transition = payload.get("frame_transition", "unknown")
        if not isinstance(frame_transition, str):
            raise ValueError("payload.frame_transition must be a string")
        return RadarFrame(
            header=header,
            frame_number=_require_int(
                _payload_value(payload, "frame_number"),
                "payload.frame_number",
                0,
                2**32 - 1,
            ),
            subframe_number=_require_int(
                _payload_value(payload, "subframe_number"),
                "payload.subframe_number",
                0,
                2**32 - 1,
            ),
            complete=_require_bool(
                _payload_value(payload, "complete"),
                "payload.complete",
            ),
            dropped_frames_since_previous=_require_int(
                _payload_value(payload, "dropped_frames_since_previous"),
                "payload.dropped_frames_since_previous",
                0,
            ),
            points=tuple(
                _point_from_dict(point, index)
                for index, point in enumerate(raw_points)
            ),
            source_format=_optional_id(
                payload.get("source_format"),
                "payload.source_format",
            ),
            sdk_version=_optional_id(
                payload.get("sdk_version"),
                "payload.sdk_version",
            ),
            device_time_cycles=_optional_int(
                payload.get("device_time_cycles"),
                "payload.device_time_cycles",
                0,
                2**32 - 1,
            ),
            frame_transition=frame_transition,
            profile_id=_optional_id(
                payload.get("profile_id"),
                "payload.profile_id",
            ),
            capture_baudrate=_optional_int(
                payload.get("capture_baudrate"),
                "payload.capture_baudrate",
                1,
                10_000_000,
            ),
        )

    if record_type == "imu_sample":
        return ImuSample(
            header=header,
            specific_force_mps2=_require_float_tuple(
                _payload_value(payload, "specific_force_mps2"),
                "payload.specific_force_mps2",
                3,
            ),
            angular_velocity_radps=_require_float_tuple(
                _payload_value(payload, "angular_velocity_radps"),
                "payload.angular_velocity_radps",
                3,
            ),
            temperature_c=_optional_float(
                payload.get("temperature_c"),
                "payload.temperature_c",
            ),
            orientation_xyzw=_optional_float_tuple(
                payload.get("orientation_xyzw"),
                "payload.orientation_xyzw",
                4,
            ),
            accel_covariance=_optional_float_tuple(
                payload.get("accel_covariance"),
                "payload.accel_covariance",
                9,
            ),
            gyro_covariance=_optional_float_tuple(
                payload.get("gyro_covariance"),
                "payload.gyro_covariance",
                9,
            ),
            orientation_covariance=_optional_float_tuple(
                payload.get("orientation_covariance"),
                "payload.orientation_covariance",
                9,
            ),
        )

    if record_type == "wheel_state":
        return WheelState(
            header=header,
            left_ticks=_require_int(
                _payload_value(payload, "left_ticks"),
                "payload.left_ticks",
            ),
            right_ticks=_require_int(
                _payload_value(payload, "right_ticks"),
                "payload.right_ticks",
            ),
            sample_period_ns=_require_int(
                _payload_value(payload, "sample_period_ns"),
                "payload.sample_period_ns",
                0,
            ),
            left_angular_velocity_radps=_optional_float(
                payload.get("left_angular_velocity_radps"),
                "payload.left_angular_velocity_radps",
            ),
            right_angular_velocity_radps=_optional_float(
                payload.get("right_angular_velocity_radps"),
                "payload.right_angular_velocity_radps",
            ),
            left_invalid_transitions=_require_int(
                payload.get("left_invalid_transitions", 0),
                "payload.left_invalid_transitions",
                0,
            ),
            right_invalid_transitions=_require_int(
                payload.get("right_invalid_transitions", 0),
                "payload.right_invalid_transitions",
                0,
            ),
        )

    if record_type == "drop_event":
        phase = _payload_value(payload, "phase")
        if not isinstance(phase, str):
            raise ValueError("payload.phase must be a string")
        reason = payload.get("reason")
        if reason is not None and not isinstance(reason, str):
            raise ValueError("payload.reason must be a string or null")
        return DropEvent(
            header=header,
            event_id=_require_id(
                _payload_value(payload, "event_id"),
                "payload.event_id",
            ),
            released_unit_id=_require_id(
                _payload_value(payload, "released_unit_id"),
                "payload.released_unit_id",
            ),
            actuator_unit_id=_require_id(
                _payload_value(payload, "actuator_unit_id"),
                "payload.actuator_unit_id",
            ),
            phase=phase,
            command_session_id=_optional_id(
                payload.get("command_session_id"),
                "payload.command_session_id",
            ),
            command_seq=_optional_int(
                payload.get("command_seq"),
                "payload.command_seq",
                1,
            ),
            anchor_keyframe_id=_optional_id(
                payload.get("anchor_keyframe_id"),
                "payload.anchor_keyframe_id",
            ),
            keyframe_to_unit_xyyaw=_optional_float_tuple(
                payload.get("keyframe_to_unit_xyyaw"),
                "payload.keyframe_to_unit_xyyaw",
                3,
            ),
            covariance_3x3=_optional_float_tuple(
                payload.get("covariance_3x3"),
                "payload.covariance_3x3",
                9,
            ),
            estimation_method=_optional_id(
                payload.get("estimation_method"),
                "payload.estimation_method",
            ),
            confirmation_method=_optional_id(
                payload.get("confirmation_method"),
                "payload.confirmation_method",
            ),
            reason=reason,
        )

    if record_type == "sensor_health":
        status = _payload_value(payload, "status")
        if not isinstance(status, str):
            raise ValueError("payload.status must be a string")
        detail = payload.get("detail")
        if detail is not None and not isinstance(detail, str):
            raise ValueError("payload.detail must be a string or null")
        return SensorHealth(
            header=header,
            subject_stream_id=_require_id(
                _payload_value(payload, "subject_stream_id"),
                "payload.subject_stream_id",
            ),
            status=status,
            observed_rate_hz=_optional_float(
                payload.get("observed_rate_hz"),
                "payload.observed_rate_hz",
            ),
            last_sample_monotonic_ns=_optional_int(
                payload.get("last_sample_monotonic_ns"),
                "payload.last_sample_monotonic_ns",
                0,
            ),
            seq_gaps_total=_require_int(
                payload.get("seq_gaps_total", 0),
                "payload.seq_gaps_total",
                0,
            ),
            parse_errors_total=_require_int(
                payload.get("parse_errors_total", 0),
                "payload.parse_errors_total",
                0,
            ),
            producer_drops_total=_require_int(
                payload.get("producer_drops_total", 0),
                "payload.producer_drops_total",
                0,
            ),
            writer_drops_total=_require_int(
                payload.get("writer_drops_total", 0),
                "payload.writer_drops_total",
                0,
            ),
            device_discontinuities_total=_require_int(
                payload.get("device_discontinuities_total", 0),
                "payload.device_discontinuities_total",
                0,
            ),
            queue_bytes=_require_int(
                payload.get("queue_bytes", 0),
                "payload.queue_bytes",
                0,
            ),
            detail=detail,
        )

    raise ValueError(f"unsupported record_type: {record_type}")
