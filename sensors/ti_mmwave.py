"""Streaming parser for TI xWRL6432 mmWave demo UART packets.

The MMWAVE-L-SDK documentation describes a little-endian packet with the
eight-byte magic word, fixed fields, and variable TLVs.  SDK 05.05 says
"52 bytes" in prose, but its displayed C fields and TI's parser both use
40 bytes.  The safe default is therefore 40.  A 52-byte/auto diagnostic mode
exists only for explicitly captured non-standard firmware.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math
import struct
import time
from typing import Deque, List, Optional, Sequence, Tuple, Union

from common.sensor_contract import (
    MAX_RADAR_HEATMAP_CELLS,
    RadarFrame,
    RadarHeatmap,
    RadarPoint,
    SensorHeader,
)


TI_MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
BASE_HEADER_SIZE = 40
DOCUMENTED_HEADER_SIZE = 52
TLV_HEADER_SIZE = 8

DEFAULT_FLOAT_POINT_TLV = 1
DEFAULT_SIDE_INFO_TLV = 7
DEFAULT_COMPRESSED_POINT_TLV = 301
RANGE_AZIMUTH_HEATMAP_MAJOR_TLV = 304
RANGE_AZIMUTH_HEATMAP_MINOR_TLV = 305
_KNOWN_STANDARD_TLV_TYPES = frozenset(range(1, 10))
_KNOWN_EXTENDED_TLV_TYPES = frozenset(range(301, 319))
_HEATMAP_MODE_BY_TLV = {
    RANGE_AZIMUTH_HEATMAP_MAJOR_TLV: "major",
    RANGE_AZIMUTH_HEATMAP_MINOR_TLV: "minor",
}


class TiMmwaveParseError(ValueError):
    pass


@dataclass(frozen=True)
class TiPacketHeader:
    version: int
    total_packet_len: int
    platform: int
    frame_number: int
    time_cpu_cycles: int
    num_detected_obj: int
    num_tlvs: int
    subframe_number: int

    @property
    def version_text(self) -> str:
        return ".".join(
            str((self.version >> shift) & 0xFF)
            for shift in (24, 16, 8, 0)
        )


@dataclass(frozen=True)
class TiRadarPoint:
    x_m: float
    y_m: float
    z_m: float
    radial_velocity_mps: float
    snr_db: Optional[float] = None
    noise_db: Optional[float] = None


@dataclass(frozen=True)
class TiRadarHeatmap:
    data: bytes
    range_bins: int
    azimuth_bins: int
    range_step_m: float
    tlv_type: int
    motion_mode: str
    floor_db: float
    ceiling_db: float

    def to_sensor_heatmap(self) -> RadarHeatmap:
        return RadarHeatmap(
            data=self.data,
            range_bins=self.range_bins,
            azimuth_bins=self.azimuth_bins,
            range_step_m=self.range_step_m,
            tlv_type=self.tlv_type,
            motion_mode=self.motion_mode,
            floor_db=self.floor_db,
            ceiling_db=self.ceiling_db,
        )


@dataclass(frozen=True)
class TiRadarFrame:
    header: TiPacketHeader
    header_size: int
    points: Tuple[TiRadarPoint, ...]
    complete: bool
    point_format: str
    unknown_tlvs: Tuple[Tuple[int, int], ...]
    warnings: Tuple[str, ...]
    heatmap: Optional[TiRadarHeatmap] = None
    host_receipt_monotonic_ns: Optional[int] = None
    host_receipt_uncertainty_ns: Optional[int] = None

    def to_sensor_record(
        self,
        sensor_header: SensorHeader,
        dropped_frames_since_previous: int = 0,
        frame_transition: str = "unknown",
        profile_id: Optional[str] = None,
        capture_baudrate: Optional[int] = None,
    ) -> RadarFrame:
        """Convert parser output without applying a mounting-axis transform.

        Use a ``frame_id`` such as ``radar_native`` until the measured
        radar-to-base calibration is available.  SLAM code must not assume
        these coordinates are already in ``base_link``.
        """

        return RadarFrame(
            header=sensor_header,
            frame_number=self.header.frame_number,
            subframe_number=self.header.subframe_number,
            complete=self.complete,
            dropped_frames_since_previous=dropped_frames_since_previous,
            points=tuple(
                RadarPoint(
                    x_m=point.x_m,
                    y_m=point.y_m,
                    z_m=point.z_m,
                    radial_velocity_mps=point.radial_velocity_mps,
                    snr_db=point.snr_db,
                    noise_db=point.noise_db,
                )
                for point in self.points
            ),
            source_format=f"ti-mmwave-{self.point_format}",
            sdk_version=self.header.version_text,
            device_time_cycles=self.header.time_cpu_cycles,
            frame_transition=frame_transition,
            profile_id=profile_id,
            capture_baudrate=capture_baudrate,
            heatmap=(
                None
                if self.heatmap is None
                else self.heatmap.to_sensor_heatmap()
            ),
        )


@dataclass(frozen=True)
class _ParsedTlvs:
    points: Tuple[TiRadarPoint, ...]
    complete: bool
    point_format: str
    unknown_tlvs: Tuple[Tuple[int, int], ...]
    warnings: Tuple[str, ...]
    trailing_padding: int
    recognized_point_tlvs: int
    heatmap: Optional[TiRadarHeatmap]


class TiMmwavePacketParser:
    """Parse complete TI demo packets.

    TLV identifiers are constructor arguments because TI demos can add custom
    output types.  Defaults match the standard floating point/side-info IDs
    and xWRL6432 compressed point ID used by MMWAVE-L-SDK 5.x.
    """

    def __init__(
        self,
        header_size: Union[str, int] = BASE_HEADER_SIZE,
        float_point_tlv: int = DEFAULT_FLOAT_POINT_TLV,
        side_info_tlv: int = DEFAULT_SIDE_INFO_TLV,
        compressed_point_tlv: int = DEFAULT_COMPRESSED_POINT_TLV,
        heatmap_azimuth_bins: Optional[int] = None,
        heatmap_range_bins: Optional[int] = None,
        heatmap_range_step_m: Optional[float] = None,
        tlv_length_includes_header: bool = False,
        allow_elided_empty_point_tlv: bool = False,
        allow_nonzero_padding: bool = False,
        max_packet_bytes: int = 4 * 1024 * 1024,
        max_points: int = 8192,
        max_tlvs: int = 128,
    ) -> None:
        if header_size not in ("auto", BASE_HEADER_SIZE, DOCUMENTED_HEADER_SIZE):
            raise ValueError("header_size must be 'auto', 40, or 52")
        for name, value in (
            ("float_point_tlv", float_point_tlv),
            ("side_info_tlv", side_info_tlv),
            ("compressed_point_tlv", compressed_point_tlv),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        heatmap_configuration = (
            heatmap_azimuth_bins,
            heatmap_range_bins,
            heatmap_range_step_m,
        )
        if any(value is not None for value in heatmap_configuration) and not all(
            value is not None for value in heatmap_configuration
        ):
            raise ValueError(
                "heatmap_azimuth_bins, heatmap_range_bins, and "
                "heatmap_range_step_m "
                "must be supplied together"
            )
        if heatmap_azimuth_bins is not None and (
            isinstance(heatmap_azimuth_bins, bool)
            or not isinstance(heatmap_azimuth_bins, int)
            or heatmap_azimuth_bins < 1
        ):
            raise ValueError("heatmap_azimuth_bins must be positive")
        if heatmap_range_bins is not None and (
            isinstance(heatmap_range_bins, bool)
            or not isinstance(heatmap_range_bins, int)
            or heatmap_range_bins < 1
        ):
            raise ValueError("heatmap_range_bins must be positive")
        if heatmap_range_step_m is not None and (
            isinstance(heatmap_range_step_m, bool)
            or not isinstance(heatmap_range_step_m, (int, float))
            or not math.isfinite(float(heatmap_range_step_m))
            or float(heatmap_range_step_m) <= 0
        ):
            raise ValueError("heatmap_range_step_m must be positive")
        if (
            heatmap_azimuth_bins is not None
            and heatmap_range_bins is not None
            and heatmap_azimuth_bins * heatmap_range_bins
            > MAX_RADAR_HEATMAP_CELLS
        ):
            raise ValueError(
                "heatmap dimensions exceed limit of "
                f"{MAX_RADAR_HEATMAP_CELLS} cells"
            )
        if max_packet_bytes < DOCUMENTED_HEADER_SIZE:
            raise ValueError("max_packet_bytes is too small")
        if max_points < 1:
            raise ValueError("max_points must be positive")
        if max_tlvs < 1:
            raise ValueError("max_tlvs must be positive")
        for name, value in (
            (
                "allow_elided_empty_point_tlv",
                allow_elided_empty_point_tlv,
            ),
            ("allow_nonzero_padding", allow_nonzero_padding),
        ):
            if not isinstance(value, bool):
                raise ValueError(f"{name} must be a boolean")
        self.header_size = header_size
        self.float_point_tlv = float_point_tlv
        self.side_info_tlv = side_info_tlv
        self.compressed_point_tlv = compressed_point_tlv
        self.heatmap_azimuth_bins = heatmap_azimuth_bins
        self.heatmap_range_bins = heatmap_range_bins
        self.heatmap_range_step_m = (
            None
            if heatmap_range_step_m is None
            else float(heatmap_range_step_m)
        )
        self.tlv_length_includes_header = tlv_length_includes_header
        self.allow_elided_empty_point_tlv = allow_elided_empty_point_tlv
        self.allow_nonzero_padding = allow_nonzero_padding
        self.max_packet_bytes = max_packet_bytes
        self.max_points = max_points
        self.max_tlvs = max_tlvs

    def parse_packet(self, packet: bytes) -> TiRadarFrame:
        if not isinstance(packet, bytes):
            raise TypeError("packet must be bytes")
        if len(packet) < BASE_HEADER_SIZE:
            raise TiMmwaveParseError("packet is shorter than the base header")
        if packet[:8] != TI_MAGIC_WORD:
            raise TiMmwaveParseError("packet magic word is invalid")

        fields = struct.unpack_from("<8I", packet, 8)
        header = TiPacketHeader(*fields)
        if header.total_packet_len != len(packet):
            raise TiMmwaveParseError(
                "packet length does not match total_packet_len "
                f"({len(packet)} != {header.total_packet_len})"
            )
        if header.total_packet_len > self.max_packet_bytes:
            raise TiMmwaveParseError("packet exceeds configured maximum")
        if header.num_tlvs > self.max_tlvs:
            raise TiMmwaveParseError("num_tlvs exceeds configured maximum")
        if header.num_detected_obj > self.max_points:
            raise TiMmwaveParseError("num_detected_obj exceeds configured maximum")

        sizes: Sequence[int]
        if self.header_size == "auto":
            sizes = (BASE_HEADER_SIZE, DOCUMENTED_HEADER_SIZE)
        else:
            sizes = (int(self.header_size),)

        candidates: List[Tuple[Tuple[int, int, int], int, _ParsedTlvs]] = []
        errors: List[str] = []
        for candidate_size in sizes:
            if candidate_size > len(packet):
                continue
            try:
                parsed = self._parse_tlvs(packet, candidate_size, header)
            except TiMmwaveParseError as exc:
                errors.append(f"{candidate_size}-byte header: {exc}")
                continue
            score = (
                parsed.recognized_point_tlvs
                + (1 if parsed.heatmap is not None else 0),
                1 if len(parsed.points) == header.num_detected_obj else 0,
                -parsed.trailing_padding,
            )
            candidates.append((score, candidate_size, parsed))

        if not candidates:
            detail = "; ".join(errors) if errors else "no valid header offset"
            raise TiMmwaveParseError(f"unable to parse TLVs: {detail}")
        candidates.sort(key=lambda item: item[0], reverse=True)
        _, selected_size, selected = candidates[0]
        return TiRadarFrame(
            header=header,
            header_size=selected_size,
            points=selected.points,
            complete=selected.complete,
            point_format=selected.point_format,
            unknown_tlvs=selected.unknown_tlvs,
            warnings=selected.warnings,
            heatmap=selected.heatmap,
            host_receipt_monotonic_ns=None,
            host_receipt_uncertainty_ns=None,
        )

    def _parse_tlvs(
        self,
        packet: bytes,
        offset: int,
        header: TiPacketHeader,
    ) -> _ParsedTlvs:
        cursor = offset
        raw_float_points: Optional[List[Tuple[float, float, float, float]]] = None
        raw_side_info: Optional[List[Tuple[float, float]]] = None
        compressed_points: Optional[List[TiRadarPoint]] = None
        heatmap: Optional[TiRadarHeatmap] = None
        unknown: List[Tuple[int, int]] = []
        warnings: List[str] = []
        recognized = 0

        for tlv_index in range(header.num_tlvs):
            if cursor + TLV_HEADER_SIZE > len(packet):
                raise TiMmwaveParseError(
                    f"TLV {tlv_index} header extends beyond packet"
                )
            tlv_type, declared_length = struct.unpack_from("<II", packet, cursor)
            cursor += TLV_HEADER_SIZE
            if self.tlv_length_includes_header:
                if declared_length < TLV_HEADER_SIZE:
                    raise TiMmwaveParseError(
                        f"TLV {tlv_index} length is smaller than its header"
                    )
                payload_length = declared_length - TLV_HEADER_SIZE
            else:
                payload_length = declared_length
            payload_end = cursor + payload_length
            if payload_end > len(packet):
                raise TiMmwaveParseError(
                    f"TLV {tlv_index} payload extends beyond packet"
                )
            payload = packet[cursor:payload_end]
            cursor = payload_end

            if tlv_type == self.float_point_tlv:
                if raw_float_points is not None:
                    raise TiMmwaveParseError("duplicate floating point TLV")
                raw_float_points = self._parse_float_points(payload)
                recognized += 1
            elif tlv_type == self.side_info_tlv:
                if raw_side_info is not None:
                    raise TiMmwaveParseError("duplicate side information TLV")
                raw_side_info = self._parse_side_info(payload)
            elif tlv_type == self.compressed_point_tlv:
                if compressed_points is not None:
                    raise TiMmwaveParseError("duplicate compressed point TLV")
                compressed_points = self._parse_compressed_points(payload)
                recognized += 1
            elif (
                self.heatmap_azimuth_bins is not None
                and tlv_type in _HEATMAP_MODE_BY_TLV
            ):
                if heatmap is not None:
                    raise TiMmwaveParseError(
                        "packet contains multiple range-azimuth heatmaps"
                    )
                heatmap = self._parse_heatmap(payload, tlv_type)
            else:
                unknown.append((tlv_type, payload_length))

        trailing = packet[cursor:]
        if len(trailing) > 31:
            raise TiMmwaveParseError("more than 31 padding bytes remain")
        if any(value != 0 for value in trailing):
            if not self.allow_nonzero_padding:
                raise TiMmwaveParseError(
                    "non-zero bytes remain after declared TLVs"
                )
            expected_padding = (-cursor) % 32
            if (
                len(packet) % 32 != 0
                or len(trailing) != expected_padding
            ):
                raise TiMmwaveParseError(
                    "non-zero trailing bytes are not valid 32-byte padding"
                )
            if self._starts_with_plausible_tlv(trailing):
                raise TiMmwaveParseError(
                    "non-zero trailing bytes begin with a plausible "
                    "undeclared TLV"
                )
            warnings.append(f"nonzero_padding:{len(trailing)}")

        if raw_float_points is not None and compressed_points is not None:
            raise TiMmwaveParseError(
                "packet contains both floating and compressed point clouds"
            )

        if compressed_points is not None:
            points = compressed_points
            point_format = "compressed"
            if raw_side_info is not None:
                warnings.append("side_info_ignored_for_compressed_points")
        elif raw_float_points is not None:
            points = []
            for index, values in enumerate(raw_float_points):
                snr: Optional[float] = None
                noise: Optional[float] = None
                if raw_side_info is not None and index < len(raw_side_info):
                    snr, noise = raw_side_info[index]
                points.append(
                    TiRadarPoint(
                        x_m=values[0],
                        y_m=values[1],
                        z_m=values[2],
                        radial_velocity_mps=values[3],
                        snr_db=snr,
                        noise_db=noise,
                    )
                )
            point_format = "float"
        else:
            points = []
            if (
                self.allow_elided_empty_point_tlv
                and header.num_detected_obj == 0
                and header.num_tlvs > 0
                and raw_side_info is None
            ):
                point_format = "empty"
                recognized += 1
                warnings.append("empty_point_tlv_elided")
            else:
                point_format = "none"

        complete = True
        if len(points) != header.num_detected_obj:
            complete = False
            warnings.append(
                "point_count_mismatch:"
                f"header={header.num_detected_obj},parsed={len(points)}"
            )
        if (
            raw_float_points is not None
            and raw_side_info is not None
            and len(raw_side_info) != len(raw_float_points)
        ):
            complete = False
            warnings.append(
                "side_info_count_mismatch:"
                f"points={len(raw_float_points)},side={len(raw_side_info)}"
            )

        return _ParsedTlvs(
            points=tuple(points),
            complete=complete,
            point_format=point_format,
            unknown_tlvs=tuple(unknown),
            warnings=tuple(warnings),
            trailing_padding=len(trailing),
            recognized_point_tlvs=recognized,
            heatmap=heatmap,
        )

    def _starts_with_plausible_tlv(self, trailing: bytes) -> bool:
        if len(trailing) < TLV_HEADER_SIZE:
            return False
        tlv_type, declared_length = struct.unpack_from("<II", trailing, 0)
        known_types = (
            _KNOWN_STANDARD_TLV_TYPES
            | _KNOWN_EXTENDED_TLV_TYPES
            | {
                self.float_point_tlv,
                self.side_info_tlv,
                self.compressed_point_tlv,
            }
        )
        if tlv_type in known_types:
            return True
        if tlv_type == 0:
            return False
        if self.tlv_length_includes_header:
            if declared_length < TLV_HEADER_SIZE:
                return False
            payload_length = declared_length - TLV_HEADER_SIZE
        else:
            payload_length = declared_length
        return payload_length <= len(trailing) - TLV_HEADER_SIZE

    def _parse_float_points(
        self,
        payload: bytes,
    ) -> List[Tuple[float, float, float, float]]:
        if len(payload) % 16:
            raise TiMmwaveParseError(
                "floating point TLV length is not a multiple of 16"
            )
        count = len(payload) // 16
        if count > self.max_points:
            raise TiMmwaveParseError("floating point TLV exceeds point limit")
        result: List[Tuple[float, float, float, float]] = []
        for index in range(count):
            values = struct.unpack_from("<4f", payload, index * 16)
            self._require_finite(values, f"floating point {index}")
            result.append(values)
        return result

    def _parse_side_info(self, payload: bytes) -> List[Tuple[float, float]]:
        if len(payload) % 4:
            raise TiMmwaveParseError(
                "side information TLV length is not a multiple of 4"
            )
        count = len(payload) // 4
        if count > self.max_points:
            raise TiMmwaveParseError("side information TLV exceeds point limit")
        return [
            (
                struct.unpack_from("<h", payload, index * 4)[0] * 0.1,
                struct.unpack_from("<h", payload, index * 4 + 2)[0] * 0.1,
            )
            for index in range(count)
        ]

    def _parse_compressed_points(self, payload: bytes) -> List[TiRadarPoint]:
        if len(payload) < 20:
            raise TiMmwaveParseError("compressed point TLV is shorter than units")
        xyz_unit, doppler_unit, snr_unit, noise_unit, major, minor = (
            struct.unpack_from("<4f2H", payload, 0)
        )
        self._require_finite(
            (xyz_unit, doppler_unit, snr_unit, noise_unit),
            "compressed point units",
        )
        if any(
            unit <= 0
            for unit in (xyz_unit, doppler_unit, snr_unit, noise_unit)
        ):
            raise TiMmwaveParseError("compressed point units must be positive")
        count = major + minor
        if count > self.max_points:
            raise TiMmwaveParseError("compressed point TLV exceeds point limit")
        expected = 20 + count * 10
        if len(payload) != expected:
            raise TiMmwaveParseError(
                "compressed point TLV size does not match encoded count "
                f"({len(payload)} != {expected})"
            )
        result: List[TiRadarPoint] = []
        for index in range(count):
            x, y, z, doppler, snr, noise = struct.unpack_from(
                "<hhhhBB",
                payload,
                20 + index * 10,
            )
            result.append(
                TiRadarPoint(
                    x_m=x * xyz_unit,
                    y_m=y * xyz_unit,
                    z_m=z * xyz_unit,
                    radial_velocity_mps=doppler * doppler_unit,
                    snr_db=snr * snr_unit,
                    noise_db=noise * noise_unit,
                )
            )
        return result

    def _parse_heatmap(
        self,
        payload: bytes,
        tlv_type: int,
    ) -> TiRadarHeatmap:
        assert self.heatmap_azimuth_bins is not None
        assert self.heatmap_range_bins is not None
        assert self.heatmap_range_step_m is not None
        if not payload:
            raise TiMmwaveParseError("range-azimuth heatmap TLV is empty")
        if len(payload) % 4:
            raise TiMmwaveParseError(
                "range-azimuth heatmap length is not a multiple of uint32"
            )
        cells = len(payload) // 4
        if cells > MAX_RADAR_HEATMAP_CELLS:
            raise TiMmwaveParseError(
                "range-azimuth heatmap exceeds cell limit"
            )
        expected_cells = (
            self.heatmap_range_bins * self.heatmap_azimuth_bins
        )
        if cells != expected_cells:
            raise TiMmwaveParseError(
                "range-azimuth heatmap cell count does not match configured "
                f"shape ({cells} != {self.heatmap_range_bins} * "
                f"{self.heatmap_azimuth_bins})"
            )
        raw_values = struct.unpack(f"<{cells}I", payload)
        data, floor_db, ceiling_db = self._quantize_heatmap(raw_values)
        return TiRadarHeatmap(
            data=data,
            range_bins=self.heatmap_range_bins,
            azimuth_bins=self.heatmap_azimuth_bins,
            range_step_m=self.heatmap_range_step_m,
            tlv_type=tlv_type,
            motion_mode=_HEATMAP_MODE_BY_TLV[tlv_type],
            floor_db=floor_db,
            ceiling_db=ceiling_db,
        )

    @staticmethod
    def _quantize_heatmap(
        raw_values: Sequence[int],
    ) -> Tuple[bytes, float, float]:
        """Robustly map TI detMatrix uint32 values to an 8-bit dB image."""

        log_values = [
            20.0 * math.log10(value)
            for value in raw_values
            if value > 0
        ]
        if not log_values:
            return bytes(len(raw_values)), 0.0, 1.0

        ordered = sorted(log_values)
        floor_db = TiMmwavePacketParser._percentile(ordered, 0.01)
        ceiling_db = TiMmwavePacketParser._percentile(ordered, 0.99)
        if ceiling_db - floor_db < 1.0:
            midpoint = (floor_db + ceiling_db) * 0.5
            floor_db = midpoint - 0.5
            ceiling_db = midpoint + 0.5
        scale = 255.0 / (ceiling_db - floor_db)
        quantized = bytearray(len(raw_values))
        for index, value in enumerate(raw_values):
            if value == 0:
                continue
            db = 20.0 * math.log10(value)
            mapped = round((db - floor_db) * scale)
            quantized[index] = min(255, max(0, mapped))
        return bytes(quantized), floor_db, ceiling_db

    @staticmethod
    def _percentile(ordered: Sequence[float], fraction: float) -> float:
        if len(ordered) == 1:
            return ordered[0]
        position = (len(ordered) - 1) * fraction
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return ordered[lower]
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    @staticmethod
    def _require_finite(values: Sequence[float], name: str) -> None:
        if any(not math.isfinite(value) for value in values):
            raise TiMmwaveParseError(f"{name} contains NaN or Infinity")


class TiMmwaveStreamDecoder:
    """Incrementally recover complete frames from arbitrary UART chunks."""

    def __init__(
        self,
        parser: Optional[TiMmwavePacketParser] = None,
        max_buffer_bytes: int = 8 * 1024 * 1024,
    ) -> None:
        self.parser = parser or TiMmwavePacketParser()
        if max_buffer_bytes < self.parser.max_packet_bytes:
            raise ValueError(
                "max_buffer_bytes must be at least max_packet_bytes"
            )
        self.max_buffer_bytes = max_buffer_bytes
        self.discarded_bytes = 0
        self.parse_errors = 0
        self.synchronized = False
        self.startup_sync_discarded_bytes = 0
        self.startup_sync_parse_errors = 0
        self._buffer = bytearray()
        self._receipt_segments: Deque[Tuple[int, int, int]] = deque()

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    @property
    def post_sync_discarded_bytes(self) -> int:
        if not self.synchronized:
            return 0
        return self.discarded_bytes - self.startup_sync_discarded_bytes

    @property
    def post_sync_parse_errors(self) -> int:
        if not self.synchronized:
            return 0
        return self.parse_errors - self.startup_sync_parse_errors

    def feed(
        self,
        data: bytes,
        receipt_monotonic_ns: Optional[int] = None,
        receipt_uncertainty_ns: int = 0,
    ) -> List[TiRadarFrame]:
        if not isinstance(data, bytes):
            raise TypeError("data must be bytes")
        if receipt_monotonic_ns is None:
            receipt_monotonic_ns = time.monotonic_ns()
        if (
            isinstance(receipt_monotonic_ns, bool)
            or not isinstance(receipt_monotonic_ns, int)
            or receipt_monotonic_ns < 0
        ):
            raise ValueError("receipt_monotonic_ns must be a non-negative integer")
        if (
            isinstance(receipt_uncertainty_ns, bool)
            or not isinstance(receipt_uncertainty_ns, int)
            or receipt_uncertainty_ns < 0
        ):
            raise ValueError(
                "receipt_uncertainty_ns must be a non-negative integer"
            )
        self._buffer.extend(data)
        if data:
            self._receipt_segments.append(
                (
                    len(data),
                    receipt_monotonic_ns,
                    receipt_uncertainty_ns,
                )
            )
        if len(self._buffer) > self.max_buffer_bytes:
            overflow = len(self._buffer) - self.max_buffer_bytes
            self._discard_prefix(overflow)
            self.discarded_bytes += overflow

        frames: List[TiRadarFrame] = []
        while True:
            magic_index = self._buffer.find(TI_MAGIC_WORD)
            if magic_index < 0:
                keep = min(len(self._buffer), len(TI_MAGIC_WORD) - 1)
                discard = len(self._buffer) - keep
                if discard:
                    self._discard_prefix(discard)
                    self.discarded_bytes += discard
                break
            if magic_index:
                self._discard_prefix(magic_index)
                self.discarded_bytes += magic_index
            if len(self._buffer) < 16:
                break
            frame_receipt_ns = self._receipt_segments[0][1]
            frame_uncertainty_ns = self._receipt_segments[0][2]

            total_length = struct.unpack_from("<I", self._buffer, 12)[0]
            if (
                total_length < BASE_HEADER_SIZE
                or total_length > self.parser.max_packet_bytes
            ):
                self._discard_prefix(1)
                self.discarded_bytes += 1
                self.parse_errors += 1
                continue
            if len(self._buffer) < total_length:
                next_magic = self._find_next_plausible_magic()
                if next_magic is not None:
                    self._discard_prefix(next_magic)
                    self.discarded_bytes += next_magic
                    self.parse_errors += 1
                    continue
                break

            packet = bytes(self._buffer[:total_length])
            try:
                frame = self.parser.parse_packet(packet)
            except TiMmwaveParseError:
                self._discard_prefix(1)
                self.discarded_bytes += 1
                self.parse_errors += 1
                continue
            if not self.synchronized:
                self.synchronized = True
                self.startup_sync_discarded_bytes = self.discarded_bytes
                self.startup_sync_parse_errors = self.parse_errors
            self._discard_prefix(total_length)
            frames.append(
                replace(
                    frame,
                    host_receipt_monotonic_ns=frame_receipt_ns,
                    host_receipt_uncertainty_ns=frame_uncertainty_ns,
                )
            )
        if not self.synchronized:
            self.startup_sync_discarded_bytes = self.discarded_bytes
            self.startup_sync_parse_errors = self.parse_errors
        return frames

    def _find_next_plausible_magic(self) -> Optional[int]:
        search_from = 1
        while True:
            index = self._buffer.find(TI_MAGIC_WORD, search_from)
            if index < 0:
                return None
            if len(self._buffer) < index + 16:
                return None
            candidate_length = struct.unpack_from(
                "<I",
                self._buffer,
                index + 12,
            )[0]
            if (
                BASE_HEADER_SIZE
                <= candidate_length
                <= self.parser.max_packet_bytes
            ):
                return index
            search_from = index + 1

    def _discard_prefix(self, count: int) -> None:
        if count < 0 or count > len(self._buffer):
            raise RuntimeError("internal buffer discard is out of range")
        if count == 0:
            return
        del self._buffer[:count]
        remaining = count
        while remaining:
            if not self._receipt_segments:
                raise RuntimeError("receipt segment accounting underflow")
            (
                segment_length,
                timestamp_ns,
                uncertainty_ns,
            ) = self._receipt_segments.popleft()
            if segment_length <= remaining:
                remaining -= segment_length
                continue
            self._receipt_segments.appendleft(
                (
                    segment_length - remaining,
                    timestamp_ns,
                    uncertainty_ns,
                )
            )
            remaining = 0
