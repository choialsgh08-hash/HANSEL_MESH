"""Live IWRL6432 FCCSP SPI source for the 0-30 cm operator view.

The firmware writes one complete ADC frame into its SPI buffer.  A dedicated
reader thread drains that buffer without waiting for host processing; a second
thread BPM-decodes a rolling window and emits canonical ``RadarFrame`` values.
This keeps the compact on-wire rate near 49 kB/s at 8 Hz while the browser only
receives processed range/azimuth evidence.
"""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
from queue import Empty, Full, Queue
import subprocess
import threading
import time
from typing import Callable, Deque, Optional, Sequence

import numpy as np

from common.sensor_contract import RadarFrame, RadarHeatmap, RadarPoint, SensorHeader
from sensors.ti_radar_control import (
    find_xds110_reset,
    load_commands,
    reset_xds110_target,
    select_application_port,
)
from sensors.ti_spi_adc import (
    FtdiFccspCapture,
    NearFieldAdcProfile,
    NearFieldMap,
    process_near_field_adc,
    unpack_fccsp_spi_bytes,
)


SPI_SOURCE_FORMAT = "ti-fccsp-spi-raw-bpm-v1"
SPI_PROFILE_ID = "iwrl6432-spi-near30-bpm2tx-8hz-v1"
SPI_CALIBRATION_ID = "spi-bpm-evm-phase-v1"
# Do not present zero-padding as sensor resolution.  The chirp resolves about
# 3.85 cm in range and the four-element azimuth aperture about 29 degrees.
# The 24-bin sine grid keeps a sample at +/-56 degrees; a 16-bin grid jumped
# directly from +/-49 to +/-61 and discarded the edge-wall return at 60 deg.
# This is display/ridge oversampling, not a claim of 24 independent beams.
SPI_HEATMAP_RANGE_STEP_M = 0.02
SPI_HEATMAP_RANGE_BINS = 16
SPI_HEATMAP_AZIMUTH_BINS = 24
SPI_HEATMAP_FLOOR_DB = -36.0
SPI_HEATMAP_CEILING_DB = 0.0


def _read_response(port: object, *, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    response = bytearray()
    while time.monotonic() < deadline:
        waiting = getattr(port, "in_waiting")
        chunk = port.read(waiting or 1)
        if chunk:
            response.extend(chunk)
            if b"mmwDemo:/>" in response:
                break
    return bytes(response)


def _send_command(port: object, command: str, *, timeout_s: float) -> bytes:
    port.write((command + "\n").encode("ascii"))
    port.flush()
    response = _read_response(port, timeout_s=timeout_s)
    lowered = response.lower()
    if b"error" in lowered or b"not recognized" in lowered:
        raise RuntimeError(
            f"radar command failed: {command!r}: "
            f"{response.decode('ascii', errors='replace')!r}"
        )
    if b"Done" not in response:
        raise TimeoutError(f"radar command timed out: {command!r}")
    return response


def commands_for_spi_stream(path: Path) -> tuple[str, ...]:
    """Load a SPI profile and replace its finite capture count with infinity."""

    commands = []
    frame_commands = 0
    for command in load_commands(path):
        name = command.split(maxsplit=1)[0]
        if name in {"sensorStart", "baudRate"}:
            raise ValueError(f"SPI profile must not contain {name}")
        if name == "frameCfg":
            fields = command.split()
            if len(fields) != 7:
                raise ValueError("frameCfg must contain six values")
            fields[-1] = "0"
            command = " ".join(fields)
            frame_commands += 1
        commands.append(command)
    if frame_commands != 1:
        raise ValueError("SPI profile must contain exactly one frameCfg")
    return tuple(commands)


def _heatmap_angles_deg(count: int = SPI_HEATMAP_AZIMUTH_BINS) -> np.ndarray:
    indices = np.arange(count, dtype=np.float64)
    arguments = 2.0 * (indices - count / 2.0) / count
    return np.rad2deg(np.arcsin(np.clip(arguments, -1.0, 1.0)))


def near_field_map_to_heatmap(scene: NearFieldMap) -> RadarHeatmap:
    """Resample the full host range/azimuth power image into the UI contract."""

    target_ranges = (
        np.arange(SPI_HEATMAP_RANGE_BINS, dtype=np.float64)
        * SPI_HEATMAP_RANGE_STEP_M
    )
    target_angles = _heatmap_angles_deg()
    by_range = np.empty(
        (scene.ranges_m.size, target_angles.size), dtype=np.float64
    )
    for range_index in range(scene.ranges_m.size):
        by_range[range_index] = np.interp(
            target_angles,
            scene.angles_deg,
            scene.power_db[range_index],
            left=SPI_HEATMAP_FLOOR_DB,
            right=SPI_HEATMAP_FLOOR_DB,
        )
    sampled = np.empty(
        (target_ranges.size, target_angles.size), dtype=np.float64
    )
    for angle_index in range(target_angles.size):
        sampled[:, angle_index] = np.interp(
            target_ranges,
            scene.ranges_m,
            by_range[:, angle_index],
            left=SPI_HEATMAP_FLOOR_DB,
            right=SPI_HEATMAP_FLOOR_DB,
        )
    trusted = (
        (target_ranges >= scene.trusted_minimum_range_m)
        & (target_ranges <= scene.maximum_range_m)
    )
    sampled[~trusted] = SPI_HEATMAP_FLOOR_DB
    encoded = np.rint(
        255.0
        * np.clip(
            (sampled - SPI_HEATMAP_FLOOR_DB)
            / (SPI_HEATMAP_CEILING_DB - SPI_HEATMAP_FLOOR_DB),
            0.0,
            1.0,
        )
    ).astype(np.uint8)
    return RadarHeatmap(
        data=encoded.tobytes(order="C"),
        range_bins=SPI_HEATMAP_RANGE_BINS,
        azimuth_bins=SPI_HEATMAP_AZIMUTH_BINS,
        range_step_m=SPI_HEATMAP_RANGE_STEP_M,
        tlv_type=304,
        motion_mode="major",
        floor_db=SPI_HEATMAP_FLOOR_DB,
        ceiling_db=SPI_HEATMAP_CEILING_DB,
    )


def _peak_quality(
    relative_db: float,
    coherence: float,
    angular_prominence_db: float,
) -> float:
    """Continuous evidence score; no amplitude threshold removes a peak."""

    strength = 1.0 / (1.0 + math.exp(-(relative_db + 6.0) / 2.0))
    directionality = 1.0 / (
        1.0 + math.exp(-(angular_prominence_db - 2.0) / 0.8)
    )
    # Broad, nearly angle-independent rings are characteristic of TX/RX
    # coupling near the HPF transition.  Preserve them in the soft heatmap,
    # but do not promote them to reactive point obstacles as readily as a
    # directional wall return.
    directional_weight = 0.35 + 0.65 * directionality
    return max(
        0.05,
        min(
            1.0,
            strength
            * max(0.0, min(1.0, coherence))
            * directional_weight,
        ),
    )


def near_field_map_to_radar_frame(
    scene: NearFieldMap,
    *,
    seq: int,
    frame_number: int,
    monotonic_ns: int,
    wall_time_ns: int,
    producer_id: str,
) -> RadarFrame:
    points = []
    if scene.blind_zone_blocked:
        # The ADC signature proves that something blocks the analogue blind
        # zone, but cannot resolve its exact range or angle.  Publish one
        # conservative centreline proxy so the existing hazard evaluator and
        # collision planner fail safe without inventing a wall contour.
        quality = max(0.0, min(1.0, scene.blind_zone_confidence))
        points.append(
            RadarPoint(
                x_m=0.0,
                y_m=scene.blind_zone_proxy_range_m,
                z_m=0.0,
                radial_velocity_mps=0.0,
                snr_db=30.0 * quality,
                noise_db=30.0 * (1.0 - quality),
            )
        )
    for peak in scene.peaks:
        quality = _peak_quality(
            peak.relative_db,
            peak.spatial_coherence,
            peak.angular_prominence_db,
        )
        points.append(
            RadarPoint(
                x_m=peak.x_m,
                y_m=peak.y_m,
                z_m=0.0,
                radial_velocity_mps=0.0,
                snr_db=30.0 * quality,
                noise_db=30.0 * (1.0 - quality),
            )
        )
    return RadarFrame(
        header=SensorHeader(
            mission_id="radar-spi-live",
            unit_id="head",
            boot_id=producer_id,
            producer_id=producer_id,
            stream_id="radar/front",
            seq=seq,
            monotonic_ns=monotonic_ns,
            frame_id="radar_native",
            calibration_id=SPI_CALIBRATION_ID,
            wall_time_ns=wall_time_ns,
            timestamp_source="host_spi_capture_window",
            timestamp_uncertainty_ns=125_000_000,
        ),
        frame_number=frame_number,
        subframe_number=0,
        complete=True,
        dropped_frames_since_previous=0,
        points=tuple(points),
        source_format=SPI_SOURCE_FORMAT,
        sdk_version="mmwave-l-sdk-05.05.04.02-spi",
        frame_transition="first" if seq == 1 else "consecutive",
        profile_id=SPI_PROFILE_ID,
        capture_baudrate=115_200,
        heatmap=near_field_map_to_heatmap(scene),
    )


class SpiRadarLiveSource:
    """Auto-resetting live source with capture and DSP on separate threads."""

    def __init__(
        self,
        *,
        port: Optional[str],
        xds_serial: Optional[str],
        cfg: Path,
        ftdi_index: int,
        stop_event: threading.Event,
        ingest: Callable[[RadarFrame, Optional[float]], bool],
        note: Callable[[str], None],
        error: Callable[[str], None],
        window_frames: int = 8,
        processing_stride: int = 2,
        command_timeout_s: float = 5.0,
        first_frame_timeout_s: float = 45.0,
        frame_timeout_s: float = 5.0,
    ) -> None:
        if window_frames < 4:
            raise ValueError("window_frames must be at least four")
        if processing_stride < 1:
            raise ValueError("processing_stride must be positive")
        if ftdi_index < 0:
            raise ValueError("ftdi_index must be non-negative")
        self.port = port
        self.xds_serial = xds_serial
        self.cfg = Path(cfg)
        self.ftdi_index = ftdi_index
        self.stop_event = stop_event
        self.ingest = ingest
        self.note = note
        self.error = error
        self.window_frames = window_frames
        self.processing_stride = processing_stride
        self.command_timeout_s = command_timeout_s
        self.first_frame_timeout_s = first_frame_timeout_s
        self.frame_timeout_s = frame_timeout_s
        self.profile = NearFieldAdcProfile()
        self._producer_generation = 0

    def run(self) -> None:
        retry_s = 0.5
        while not self.stop_event.is_set():
            try:
                self._run_session()
                retry_s = 0.5
            except Exception as exc:  # hardware recovery boundary
                if self.stop_event.is_set():
                    break
                self.error(f"SPI reconnect: {type(exc).__name__}: {exc}")
                self.stop_event.wait(retry_s)
                retry_s = min(5.0, retry_s * 1.7)

    def _reset_and_select_port(self) -> str:
        from serial.tools.list_ports import comports

        identity = select_application_port(
            comports(), explicit_port=self.port, xds_serial=self.xds_serial
        )
        reset_executable = find_xds110_reset(None, (Path("C:/ti"),))
        self.note(f"resetting {identity.device} / {identity.serial_number}")
        reset_xds110_target(
            reset_executable, identity.serial_number, subprocess.run
        )
        if self.stop_event.wait(0.6):
            raise InterruptedError("stop requested")
        return identity.device

    def _run_session(self) -> None:
        import serial

        device = self._reset_and_select_port()
        commands = commands_for_spi_stream(self.cfg)
        raw_queue: Queue[tuple[int, bytes, float, int]] = Queue(maxsize=16)
        worker_errors: Queue[BaseException] = Queue(maxsize=1)
        processor_stop = threading.Event()
        self._producer_generation += 1
        producer_id = (
            f"spi-bpm-{int(time.time())}-{self._producer_generation}"
        )
        worker = threading.Thread(
            target=self._process_frames,
            args=(raw_queue, worker_errors, processor_stop, producer_id),
            name="radar-spi-dsp",
            daemon=True,
        )
        with serial.Serial(
            device, 115_200, timeout=0.05, write_timeout=1.0
        ) as cli:
            cli.reset_input_buffer()
            for command in commands:
                _send_command(cli, command, timeout_s=self.command_timeout_s)
            with FtdiFccspCapture(device_index=self.ftdi_index) as capture:
                worker.start()
                _send_command(
                    cli, "sensorStart 0 0 0 0", timeout_s=self.command_timeout_s
                )
                self.note(
                    f"SPI RAW live · {device} · 8Hz · "
                    f"{self.profile.frame_bytes} B/frame"
                )
                capture_index = 0
                first = True
                try:
                    while not self.stop_event.is_set():
                        if not worker_errors.empty():
                            raise worker_errors.get_nowait()
                        payload = capture.read_frame(
                            self.profile.frame_bytes,
                            ready_timeout_s=(
                                self.first_frame_timeout_s
                                if first else self.frame_timeout_s
                            ),
                        )
                        first = False
                        capture_index += 1
                        item = (
                            capture_index,
                            payload,
                            time.monotonic(),
                            time.time_ns(),
                        )
                        try:
                            raw_queue.put_nowait(item)
                        except Full:
                            try:
                                raw_queue.get_nowait()
                            except Empty:
                                pass
                            raw_queue.put_nowait(item)
                finally:
                    processor_stop.set()
                    try:
                        _send_command(
                            cli, "sensorStop 0", timeout_s=self.command_timeout_s
                        )
                    except (OSError, RuntimeError, TimeoutError):
                        pass
        if worker.is_alive():
            worker.join(timeout=2.0)
        if not worker_errors.empty():
            raise worker_errors.get_nowait()

    def _process_frames(
        self,
        raw_queue: Queue[tuple[int, bytes, float, int]],
        worker_errors: Queue[BaseException],
        processor_stop: threading.Event,
        producer_id: str,
    ) -> None:
        window: Deque[bytes] = deque(maxlen=self.window_frames)
        since_output = 0
        output_seq = 0
        try:
            while not processor_stop.is_set() or not raw_queue.empty():
                try:
                    capture_index, payload, received_at, wall_time_ns = (
                        raw_queue.get(timeout=0.1)
                    )
                except Empty:
                    continue
                window.append(payload)
                since_output += 1
                if len(window) < 4 or since_output < self.processing_stride:
                    continue
                since_output = 0
                values = unpack_fccsp_spi_bytes(b"".join(window))
                scene = process_near_field_adc(
                    values, frames=len(window), profile=self.profile
                )
                output_seq += 1
                frame = near_field_map_to_radar_frame(
                    scene,
                    seq=output_seq,
                    frame_number=capture_index,
                    monotonic_ns=int(round(received_at * 1_000_000_000)),
                    wall_time_ns=wall_time_ns,
                    producer_id=producer_id,
                )
                self.ingest(frame, received_at)
        except BaseException as exc:
            try:
                worker_errors.put_nowait(exc)
            except Full:
                pass


__all__ = [
    "SPI_CALIBRATION_ID",
    "SPI_PROFILE_ID",
    "SPI_SOURCE_FORMAT",
    "SpiRadarLiveSource",
    "commands_for_spi_stream",
    "near_field_map_to_heatmap",
    "near_field_map_to_radar_frame",
]
