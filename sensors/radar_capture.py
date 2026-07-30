"""Live TI mmWave UART capture into raw bytes and mission JSONL."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
import os
from pathlib import Path
import sys
import time
import uuid
from typing import Callable, Optional, Tuple

from common.sensor_contract import (
    MAX_RADAR_HEATMAP_CELLS,
    SensorHealth,
    validate_sensor_id,
)
from common.sensor_json import canonical_json_bytes
from sensors.header_factory import (
    SensorHeaderFactory,
    new_producer_id,
    read_linux_boot_id,
)
from sensors.mission_log import MissionLogWriter
from sensors.ti_mmwave import TiMmwavePacketParser, TiMmwaveStreamDecoder


@dataclass(frozen=True)
class RadarCaptureStats:
    frames_decoded: int
    point_cloud_frames: int
    float_point_frames: int
    compressed_point_frames: int
    empty_point_frames: int
    nonzero_padding_frames: int
    missing_point_tlv_frames: int
    complete_frames: int
    incomplete_frames: int
    points_decoded: int
    radar_frame_gaps: int
    device_discontinuities: int
    writer_drops: int
    parser_errors: int
    parser_discarded_bytes: int
    startup_sync_parse_errors: int
    startup_sync_discarded_bytes: int
    post_sync_parse_errors: int
    post_sync_discarded_bytes: int
    buffered_tail_bytes: int
    raw_bytes: int
    max_timing_quality_metric_ns: int
    mission_log: str
    raw_capture: Optional[str]
    raw_index: Optional[str]
    heatmap_frames: int = 0
    major_heatmap_frames: int = 0
    minor_heatmap_frames: int = 0
    missing_heatmap_frames: int = 0
    heatmap_cells_decoded: int = 0
    heatmap_azimuth_bins: Optional[int] = None
    heatmap_range_bins: Optional[int] = None
    heatmap_range_step_m: Optional[float] = None


@dataclass(frozen=True)
class _HealthFaultSnapshot:
    post_sync_parse_errors: int = 0
    post_sync_discarded_bytes: int = 0
    incomplete_frames: int = 0
    missing_point_tlv_frames: int = 0
    missing_heatmap_frames: int = 0
    writer_drops: int = 0
    radar_frame_gaps: int = 0
    device_discontinuities: int = 0

    def increased_since(self, previous: _HealthFaultSnapshot) -> bool:
        return any(
            getattr(self, field) > getattr(previous, field)
            for field in self.__dataclass_fields__
        )

    def any_faults(self) -> bool:
        return any(
            getattr(self, field) for field in self.__dataclass_fields__
        )


def frame_gap(previous: Optional[int], current: int) -> int:
    """Return a plausible uint32 frame gap, tolerating counter wrap/reset."""

    gap, _ = classify_frame_transition(previous, current)
    return gap


def classify_frame_transition(
    previous: Optional[int],
    current: int,
) -> Tuple[int, str]:
    """Classify uint32 frame progress and expose a new device-clock epoch."""

    if previous is None:
        return 0, "first"
    delta = (current - previous) & 0xFFFFFFFF
    if delta == 0:
        return 0, "duplicate"
    # A forward delta smaller than half the counter range includes normal wrap.
    # A larger delta is more likely a device reset or out-of-order frame.
    if delta < 0x80000000:
        gap = delta - 1
        if current < previous:
            return gap, "wrap"
        if gap:
            return gap, "gap"
        return 0, "consecutive"
    return 0, "reset_or_out_of_order"


def estimate_uart_observation_time(
    read_started_ns: int,
    read_finished_ns: int,
    chunk_bytes: int,
    baudrate: int,
    serial_timeout_s: float,
) -> Tuple[int, int]:
    """Return a host observation midpoint and a timing-quality metric.

    This is not the radar measurement time.  USB/UART buffering makes the
    exact first-byte arrival unavailable from pyserial.  The second return
    value is a heuristic scale based on the visible read window, configured
    timeout, and UART serialization time.  It is neither a statistical
    confidence interval nor a strict bound because the hidden USB/XDS110
    buffering delay is unknown.  Callers must retain the TI cycle counter and
    fit the device clock before tight IMU fusion.
    """

    for name, value in (
        ("read_started_ns", read_started_ns),
        ("read_finished_ns", read_finished_ns),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
    if read_finished_ns < read_started_ns:
        raise ValueError("read_finished_ns must not precede read_started_ns")
    if (
        isinstance(chunk_bytes, bool)
        or not isinstance(chunk_bytes, int)
        or chunk_bytes < 1
    ):
        raise ValueError("chunk_bytes must be positive")
    if (
        isinstance(baudrate, bool)
        or not isinstance(baudrate, int)
        or baudrate <= 0
    ):
        raise ValueError("baudrate must be positive")
    if (
        isinstance(serial_timeout_s, bool)
        or not isinstance(serial_timeout_s, (int, float))
        or not math.isfinite(float(serial_timeout_s))
        or serial_timeout_s <= 0
    ):
        raise ValueError("serial_timeout_s must be positive")
    midpoint_ns = (read_started_ns + read_finished_ns) // 2
    half_read_window_ns = (read_finished_ns - read_started_ns + 1) // 2
    timeout_ns = int(math.ceil(serial_timeout_s * 1_000_000_000.0))
    # 8N1 uses approximately ten line bits per byte.
    wire_ns = int(math.ceil(chunk_bytes * 10_000_000_000.0 / baudrate))
    uncertainty_ns = max(half_read_window_ns, timeout_ns, wire_ns)
    return midpoint_ns, uncertainty_ns


def _capture_boot_id(explicit_boot_id: Optional[str]) -> str:
    if explicit_boot_id:
        return explicit_boot_id
    try:
        return read_linux_boot_id()
    except RuntimeError:
        # Windows can be used for a direct USB bench capture.  A process-local
        # ID still keeps that monotonic domain separate from every other run.
        return f"host-session-{uuid.uuid4()}"


def _write_all(handle: object, data: bytes) -> None:
    view = memoryview(data)
    while view:
        written = handle.write(view)
        if written is None or written <= 0:
            raise OSError("capture write made no progress")
        view = view[written:]


def _sync_and_close_capture_files(*handles: object) -> None:
    """Best-effort close every capture file without hiding an earlier error."""

    active_error = sys.exc_info()[0] is not None
    first_error: Optional[BaseException] = None
    for handle in handles:
        if handle is None:
            continue
        try:
            os.fsync(handle.fileno())
        except BaseException as exc:
            if first_error is None:
                first_error = exc
        finally:
            try:
                handle.close()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
    if first_error is not None and not active_error:
        raise first_error


def _fsync_directory(path: Path) -> None:
    if os.name != "posix":
        return
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(str(path), flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def capture_radar_uart(
    port: str,
    baudrate: int,
    mission_log: Path,
    mission_id: str,
    profile_id: str = "unverified",
    calibration_id: Optional[str] = "uncalibrated",
    unit_id: str = "head",
    boot_id: Optional[str] = None,
    raw_capture: Optional[Path] = None,
    raw_index: Optional[Path] = None,
    duration_s: float = 0.0,
    read_size: int = 1024,
    serial_timeout_s: float = 0.01,
    health_interval_s: float = 0.5,
    overwrite: bool = False,
    header_size: int = 40,
    allow_elided_empty_point_tlv: bool = False,
    allow_nonzero_padding: bool = False,
    heatmap_azimuth_bins: Optional[int] = None,
    heatmap_range_bins: Optional[int] = None,
    heatmap_range_step_m: Optional[float] = None,
    stop_requested: Optional[Callable[[], bool]] = None,
) -> RadarCaptureStats:
    """Capture an already-running TI demo stream.

    This function intentionally does not transmit a radar configuration.  The
    exact SDK/appimage profile must be pinned before automating the shared CLI
    and processed-data UART.
    """

    if not isinstance(port, str) or not port:
        raise ValueError("port must be non-empty")
    validate_sensor_id(mission_id, "mission_id")
    validate_sensor_id(unit_id, "unit_id")
    if boot_id is not None:
        validate_sensor_id(boot_id, "boot_id")
    validate_sensor_id(profile_id, "profile_id")
    if calibration_id is not None:
        validate_sensor_id(calibration_id, "calibration_id")
    if (
        isinstance(baudrate, bool)
        or not isinstance(baudrate, int)
        or baudrate <= 0
    ):
        raise ValueError("baudrate must be positive")
    if (
        isinstance(duration_s, bool)
        or not isinstance(duration_s, (int, float))
        or not math.isfinite(float(duration_s))
        or duration_s < 0
    ):
        raise ValueError("duration_s must be non-negative")
    if isinstance(read_size, bool) or not isinstance(read_size, int) or read_size < 1:
        raise ValueError("read_size must be positive")
    if (
        isinstance(serial_timeout_s, bool)
        or not isinstance(serial_timeout_s, (int, float))
        or not math.isfinite(float(serial_timeout_s))
        or serial_timeout_s <= 0
    ):
        raise ValueError("serial_timeout_s must be positive")
    if (
        isinstance(health_interval_s, bool)
        or not isinstance(health_interval_s, (int, float))
        or not math.isfinite(float(health_interval_s))
        or health_interval_s <= 0
    ):
        raise ValueError("health_interval_s must be positive")
    if stop_requested is not None and not callable(stop_requested):
        raise ValueError("stop_requested must be callable or None")
    should_stop = stop_requested or (lambda: False)
    for name, value in (
        (
            "allow_elided_empty_point_tlv",
            allow_elided_empty_point_tlv,
        ),
        ("allow_nonzero_padding", allow_nonzero_padding),
    ):
        if not isinstance(value, bool):
            raise ValueError(f"{name} must be a boolean")
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
    mission_path = Path(mission_log)
    raw_path = None if raw_capture is None else Path(raw_capture)
    if raw_index is not None and raw_path is None:
        raise ValueError("raw_index requires raw_capture")
    index_path = (
        None
        if raw_path is None
        else (
            Path(raw_index)
            if raw_index is not None
            else Path(f"{raw_path}.chunks.jsonl")
        )
    )
    output_paths = [
        path
        for path in (mission_path, raw_path, index_path)
        if path is not None
    ]
    resolved_paths = [path.resolve(strict=False) for path in output_paths]
    if len(set(resolved_paths)) != len(resolved_paths):
        raise ValueError(
            "mission_log, raw_capture, and raw_index must be different files"
        )
    if not overwrite:
        existing = [str(path) for path in output_paths if path.exists()]
        if existing:
            raise FileExistsError(
                "capture output already exists: " + ", ".join(existing)
            )

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required; install requirements-sensors.txt"
        ) from exc

    producer_id = new_producer_id("radar-reader")
    resolved_boot_id = _capture_boot_id(boot_id)

    def new_radar_headers() -> SensorHeaderFactory:
        return SensorHeaderFactory(
            mission_id=mission_id,
            unit_id=unit_id,
            boot_id=resolved_boot_id,
            producer_id=new_producer_id("radar-device-epoch"),
            stream_id="radar/front",
            frame_id="radar_native",
            calibration_id=calibration_id,
            timestamp_source="uart_read_midpoint",
        )

    radar_headers = new_radar_headers()
    health_headers = SensorHeaderFactory(
        mission_id=mission_id,
        unit_id=unit_id,
        boot_id=resolved_boot_id,
        producer_id=producer_id,
        stream_id="health/radar",
    )
    decoder = TiMmwaveStreamDecoder(
        parser=TiMmwavePacketParser(
            header_size=header_size,
            allow_elided_empty_point_tlv=(
                allow_elided_empty_point_tlv
            ),
            allow_nonzero_padding=allow_nonzero_padding,
            heatmap_azimuth_bins=heatmap_azimuth_bins,
            heatmap_range_bins=heatmap_range_bins,
            heatmap_range_step_m=heatmap_range_step_m,
        )
    )

    frames_decoded = 0
    point_cloud_frames = 0
    float_point_frames = 0
    compressed_point_frames = 0
    empty_point_frames = 0
    nonzero_padding_frames = 0
    missing_point_tlv_frames = 0
    complete_frames = 0
    incomplete_frames = 0
    points_decoded = 0
    heatmap_frames = 0
    major_heatmap_frames = 0
    minor_heatmap_frames = 0
    missing_heatmap_frames = 0
    heatmap_cells_decoded = 0
    radar_frame_gaps = 0
    device_discontinuities = 0
    writer_drops = 0
    raw_bytes = 0
    max_timing_quality_metric_ns = 0
    previous_frame: Optional[int] = None
    last_sample_monotonic_ns: Optional[int] = None
    raw_handle = None
    index_handle = None
    chunk_seq = 0
    raw_hasher = hashlib.sha256()

    with serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=serial_timeout_s,
    ) as serial_port:
        try:
            if raw_path is not None:
                parent_existence = {
                    raw_path.parent.resolve(strict=False): (
                        raw_path.parent.exists()
                    ),
                }
                assert index_path is not None
                parent_existence[
                    index_path.parent.resolve(strict=False)
                ] = index_path.parent.exists()
                raw_path.parent.mkdir(parents=True, exist_ok=True)
                index_path.parent.mkdir(parents=True, exist_ok=True)
                raw_mode = "wb" if overwrite else "xb"
                raw_handle = raw_path.open(raw_mode, buffering=0)
                index_handle = index_path.open(raw_mode, buffering=0)
                for parent, existed in parent_existence.items():
                    _fsync_directory(parent)
                    if not existed:
                        _fsync_directory(parent.parent)
            with MissionLogWriter(
                mission_path,
                overwrite=overwrite,
            ) as writer:
                start = time.monotonic()
                next_health_at = start + health_interval_s
                last_accepted_periodic_faults = _HealthFaultSnapshot()
                stop_reason = "duration_elapsed"

                def health_fault_snapshot() -> _HealthFaultSnapshot:
                    return _HealthFaultSnapshot(
                        post_sync_parse_errors=(
                            decoder.post_sync_parse_errors
                        ),
                        post_sync_discarded_bytes=(
                            decoder.post_sync_discarded_bytes
                        ),
                        incomplete_frames=incomplete_frames,
                        missing_point_tlv_frames=(
                            missing_point_tlv_frames
                        ),
                        missing_heatmap_frames=missing_heatmap_frames,
                        writer_drops=writer_drops,
                        radar_frame_gaps=radar_frame_gaps,
                        device_discontinuities=device_discontinuities,
                    )

                def make_health_record(
                    kind: str,
                    include_buffered_tail: bool,
                    current_faults: _HealthFaultSnapshot,
                ) -> SensorHealth:
                    unresolved_startup_failure = bool(
                        include_buffered_tail
                        and not decoder.synchronized
                        and (
                            decoder.parse_errors
                            or decoder.discarded_bytes
                            or decoder.buffered_bytes
                        )
                    )
                    if kind == "periodic":
                        degraded = current_faults.increased_since(
                            last_accepted_periodic_faults
                        )
                    else:
                        degraded = bool(
                            current_faults.any_faults()
                            or unresolved_startup_failure
                            or (
                                include_buffered_tail
                                and decoder.buffered_bytes
                            )
                        )
                    if degraded:
                        status = "degraded"
                    elif frames_decoded == 0:
                        status = (
                            "stale"
                            if include_buffered_tail
                            else "starting"
                        )
                    elif point_cloud_frames == 0:
                        status = "degraded"
                    else:
                        status = "ok"
                    return SensorHealth(
                        header=health_headers.next(),
                        subject_stream_id="radar/front",
                        status=status,
                        last_sample_monotonic_ns=(
                            last_sample_monotonic_ns
                        ),
                        seq_gaps_total=radar_frame_gaps,
                        parse_errors_total=(
                            decoder.post_sync_parse_errors
                        ),
                        producer_drops_total=0,
                        writer_drops_total=writer_drops,
                        device_discontinuities_total=(
                            device_discontinuities
                        ),
                        queue_bytes=writer.stats().queued_bytes,
                        detail=(
                            f"health_kind={kind},"
                            f"frames={frames_decoded},"
                            f"points={points_decoded},"
                            f"point_cloud_frames={point_cloud_frames},"
                            f"heatmap_frames={heatmap_frames},"
                            "heatmap_expected="
                            f"{str(heatmap_azimuth_bins is not None).lower()},"
                            "heatmap_azimuth_bins="
                            f"{heatmap_azimuth_bins},"
                            "heatmap_range_bins="
                            f"{heatmap_range_bins},"
                            "heatmap_range_step_m="
                            f"{heatmap_range_step_m},"
                            "major_heatmap_frames="
                            f"{major_heatmap_frames},"
                            "minor_heatmap_frames="
                            f"{minor_heatmap_frames},"
                            "missing_heatmap_frames="
                            f"{missing_heatmap_frames},"
                            "heatmap_cells_decoded="
                            f"{heatmap_cells_decoded},"
                            "nonzero_padding_frames="
                            f"{nonzero_padding_frames},"
                            "missing_point_tlv_frames="
                            f"{missing_point_tlv_frames},"
                            f"frame_gaps={radar_frame_gaps},"
                            "post_sync_parse_errors="
                            f"{decoder.post_sync_parse_errors},"
                            "post_sync_discarded_bytes="
                            f"{decoder.post_sync_discarded_bytes},"
                            "startup_sync_parse_errors="
                            f"{decoder.startup_sync_parse_errors},"
                            "startup_sync_discarded_bytes="
                            f"{decoder.startup_sync_discarded_bytes},"
                            f"raw_parse_errors={decoder.parse_errors},"
                            "raw_discarded_bytes="
                            f"{decoder.discarded_bytes},"
                            f"buffered_tail={decoder.buffered_bytes},"
                            "device_discontinuities="
                            f"{device_discontinuities},"
                            "timestamp_source=uart_read_midpoint,"
                            "timing_metric_is_not_upper_bound=true,"
                            "max_timing_quality_metric_ns="
                            f"{max_timing_quality_metric_ns}"
                        ),
                    )

                def emit_periodic_health_if_due() -> None:
                    nonlocal last_accepted_periodic_faults
                    nonlocal next_health_at, writer_drops
                    now = time.monotonic()
                    if now < next_health_at:
                        return
                    current_faults = health_fault_snapshot()
                    health = make_health_record(
                        "periodic",
                        include_buffered_tail=False,
                        current_faults=current_faults,
                    )
                    if writer.submit(health):
                        last_accepted_periodic_faults = current_faults
                    else:
                        writer_drops += 1
                    next_health_at = now + health_interval_s

                try:
                    while duration_s == 0 or time.monotonic() - start < duration_s:
                        if should_stop():
                            stop_reason = "stop_requested"
                            break
                        read_started_ns = time.monotonic_ns()
                        chunk = serial_port.read(read_size)
                        read_finished_ns = time.monotonic_ns()
                        if not chunk:
                            emit_periodic_health_if_due()
                            continue
                        receipt_ns, receipt_uncertainty_ns = (
                            estimate_uart_observation_time(
                                read_started_ns=read_started_ns,
                                read_finished_ns=read_finished_ns,
                                chunk_bytes=len(chunk),
                                baudrate=baudrate,
                                serial_timeout_s=serial_timeout_s,
                            )
                        )
                        max_timing_quality_metric_ns = max(
                            max_timing_quality_metric_ns,
                            receipt_uncertainty_ns,
                        )
                        chunk_offset = raw_bytes
                        if raw_handle is not None:
                            _write_all(raw_handle, chunk)
                            raw_hasher.update(chunk)
                            assert index_handle is not None
                            chunk_seq += 1
                            index_record = canonical_json_bytes(
                                {
                                    "index_version": 1,
                                    "record_type": "uart_chunk",
                                    "mission_id": mission_id,
                                    "unit_id": unit_id,
                                    "boot_id": resolved_boot_id,
                                    "producer_id": producer_id,
                                    "profile_id": profile_id,
                                    "calibration_id": calibration_id,
                                    "allow_elided_empty_point_tlv": (
                                        allow_elided_empty_point_tlv
                                    ),
                                    "allow_nonzero_padding": (
                                        allow_nonzero_padding
                                    ),
                                    "heatmap_azimuth_bins": (
                                        heatmap_azimuth_bins
                                    ),
                                    "heatmap_range_bins": (
                                        heatmap_range_bins
                                    ),
                                    "heatmap_range_step_m": (
                                        heatmap_range_step_m
                                    ),
                                    "chunk_seq": chunk_seq,
                                    "byte_offset": chunk_offset,
                                    "byte_length": len(chunk),
                                    "read_started_ns": read_started_ns,
                                    "read_finished_ns": read_finished_ns,
                                    "observation_midpoint_ns": receipt_ns,
                                    "timing_quality_metric_ns": (
                                        receipt_uncertainty_ns
                                    ),
                                    "baudrate": baudrate,
                                }
                            )
                            _write_all(index_handle, index_record)
                            _write_all(index_handle, b"\n")
                        raw_bytes += len(chunk)
                        for frame in decoder.feed(
                            chunk,
                            receipt_monotonic_ns=receipt_ns,
                            receipt_uncertainty_ns=receipt_uncertainty_ns,
                        ):
                            gap, transition = classify_frame_transition(
                                previous_frame,
                                frame.header.frame_number,
                            )
                            if transition in {
                                "duplicate",
                                "reset_or_out_of_order",
                            }:
                                device_discontinuities += 1
                            if transition == "reset_or_out_of_order":
                                radar_headers = new_radar_headers()
                            previous_frame = frame.header.frame_number
                            radar_frame_gaps += gap
                            frames_decoded += 1
                            if any(
                                warning.startswith("nonzero_padding:")
                                for warning in frame.warnings
                            ):
                                nonzero_padding_frames += 1
                            if frame.point_format == "float":
                                point_cloud_frames += 1
                                float_point_frames += 1
                            elif frame.point_format == "compressed":
                                point_cloud_frames += 1
                                compressed_point_frames += 1
                            elif frame.point_format == "empty":
                                point_cloud_frames += 1
                                empty_point_frames += 1
                            else:
                                missing_point_tlv_frames += 1
                            points_decoded += len(frame.points)
                            if frame.heatmap is None:
                                if heatmap_azimuth_bins is not None:
                                    missing_heatmap_frames += 1
                            else:
                                heatmap_frames += 1
                                heatmap_cells_decoded += len(
                                    frame.heatmap.data
                                )
                                if frame.heatmap.motion_mode == "major":
                                    major_heatmap_frames += 1
                                else:
                                    minor_heatmap_frames += 1
                            if frame.complete:
                                complete_frames += 1
                            else:
                                incomplete_frames += 1
                            capture_ns = frame.host_receipt_monotonic_ns
                            if capture_ns is None:
                                capture_ns = receipt_ns
                            last_sample_monotonic_ns = capture_ns
                            record = frame.to_sensor_record(
                                radar_headers.next(
                                    capture_monotonic_ns=capture_ns,
                                    timestamp_uncertainty_ns=(
                                        frame.host_receipt_uncertainty_ns
                                    ),
                                ),
                                dropped_frames_since_previous=gap,
                                frame_transition=transition,
                                profile_id=profile_id,
                                capture_baudrate=baudrate,
                            )
                            if not writer.submit(record):
                                writer_drops += 1
                        emit_periodic_health_if_due()
                except KeyboardInterrupt:
                    stop_reason = "keyboard_interrupt"

                if index_handle is not None:
                    assert raw_handle is not None
                    footer = canonical_json_bytes(
                        {
                            "index_version": 1,
                            "record_type": "capture_end",
                            "mission_id": mission_id,
                            "unit_id": unit_id,
                            "boot_id": resolved_boot_id,
                            "producer_id": producer_id,
                            "profile_id": profile_id,
                            "calibration_id": calibration_id,
                            "allow_elided_empty_point_tlv": (
                                allow_elided_empty_point_tlv
                            ),
                            "allow_nonzero_padding": (
                                allow_nonzero_padding
                            ),
                            "heatmap_azimuth_bins": (
                                heatmap_azimuth_bins
                            ),
                            "heatmap_range_bins": (
                                heatmap_range_bins
                            ),
                            "heatmap_range_step_m": (
                                heatmap_range_step_m
                            ),
                            "chunks": chunk_seq,
                            "raw_bytes": raw_bytes,
                            "raw_sha256": raw_hasher.hexdigest(),
                            "frames_decoded": frames_decoded,
                            "heatmap_frames": heatmap_frames,
                            "missing_heatmap_frames": (
                                missing_heatmap_frames
                            ),
                            "ended_monotonic_ns": time.monotonic_ns(),
                            "stop_reason": stop_reason,
                            "baudrate": baudrate,
                        }
                    )
                    _write_all(index_handle, footer)
                    _write_all(index_handle, b"\n")
                    # The critical health record must never claim a clean end
                    # before its paired raw stream and index are durable.
                    os.fsync(raw_handle.fileno())
                    os.fsync(index_handle.fileno())

                current_faults = health_fault_snapshot()
                health = make_health_record(
                    "final",
                    include_buffered_tail=True,
                    current_faults=current_faults,
                )
                writer.write_critical(health)
        finally:
            _sync_and_close_capture_files(raw_handle, index_handle)

    return RadarCaptureStats(
        frames_decoded=frames_decoded,
        point_cloud_frames=point_cloud_frames,
        float_point_frames=float_point_frames,
        compressed_point_frames=compressed_point_frames,
        empty_point_frames=empty_point_frames,
        nonzero_padding_frames=nonzero_padding_frames,
        missing_point_tlv_frames=missing_point_tlv_frames,
        complete_frames=complete_frames,
        incomplete_frames=incomplete_frames,
        points_decoded=points_decoded,
        radar_frame_gaps=radar_frame_gaps,
        device_discontinuities=device_discontinuities,
        writer_drops=writer_drops,
        parser_errors=decoder.parse_errors,
        parser_discarded_bytes=decoder.discarded_bytes,
        startup_sync_parse_errors=decoder.startup_sync_parse_errors,
        startup_sync_discarded_bytes=(
            decoder.startup_sync_discarded_bytes
        ),
        post_sync_parse_errors=decoder.post_sync_parse_errors,
        post_sync_discarded_bytes=decoder.post_sync_discarded_bytes,
        buffered_tail_bytes=decoder.buffered_bytes,
        raw_bytes=raw_bytes,
        max_timing_quality_metric_ns=max_timing_quality_metric_ns,
        mission_log=str(mission_path),
        raw_capture=(
            None if raw_path is None else str(raw_path)
        ),
        raw_index=(
            None if index_path is None else str(index_path)
        ),
        heatmap_frames=heatmap_frames,
        major_heatmap_frames=major_heatmap_frames,
        minor_heatmap_frames=minor_heatmap_frames,
        missing_heatmap_frames=missing_heatmap_frames,
        heatmap_cells_decoded=heatmap_cells_decoded,
        heatmap_azimuth_bins=heatmap_azimuth_bins,
        heatmap_range_bins=heatmap_range_bins,
        heatmap_range_step_m=(
            None
            if heatmap_range_step_m is None
            else float(heatmap_range_step_m)
        ),
    )


def capture_stats_dict(stats: RadarCaptureStats) -> dict:
    return asdict(stats)
