"""IWRL6432BOOST near-field RAW ADC capture and host-side processing.

This module implements the FCCSP SPI transport documented by TI for the
C232HM-DDHSL-0 and the BPM decoding required by the stock motion/presence
profiles.  The capture path intentionally keeps the wire data binary: the TI
reference tool's decimal text output is roughly an order of magnitude larger.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Callable, Optional

import numpy as np


SPEED_OF_LIGHT_M_S = 299_792_458.0
FTDI_MAX_TRANSFER_BYTES = 65_536
FCCSP_BUSY_MASK = 0x10
BLIND_ZONE_BLOCK_CONFIDENCE = 0.80

_MPSSE_DISABLE_DIVIDE_BY_5 = b"\x8a"
_MPSSE_DISABLE_ADAPTIVE_CLOCK = b"\x97"
_MPSSE_DISABLE_THREE_PHASE_CLOCK = b"\x8d"
_MPSSE_DISABLE_LOOPBACK = b"\x85"
_MPSSE_SET_LOW_BYTE_IDLE = b"\x80\x08\x0b"
_MPSSE_CS_LOW = b"\x80\x00\x0b"
_MPSSE_CS_HIGH = b"\x80\x08\x0b"
_MPSSE_READ_BYTES_POSITIVE_EDGE_MSB = b"\x20"
_MPSSE_READ_LOW_BYTE = b"\x81"


@dataclass(frozen=True)
class NearFieldAdcProfile:
    """Pinned compact chirp used for the 0-30 cm driving-assist view."""

    adc_samples: int = 128
    rx_antennas: int = 3
    coded_tx_chirps: int = 2
    chirps_per_frame: int = 8
    bursts_per_frame: int = 1
    sample_rate_hz: float = 12.5e6
    slope_hz_per_s: float = 380.0e12
    hpf_corner_hz: float = 175_000.0
    minimum_trusted_range_m: float = 0.10
    maximum_range_m: float = 0.30
    range_compensation_exponent: float = 3.0

    def __post_init__(self) -> None:
        integer_fields = (
            "adc_samples",
            "rx_antennas",
            "coded_tx_chirps",
            "chirps_per_frame",
            "bursts_per_frame",
        )
        for name in integer_fields:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.coded_tx_chirps != 2:
            raise ValueError("the supported IWRL6432 BPM profile has two coded chirps")
        if self.chirps_per_frame % self.coded_tx_chirps:
            raise ValueError("chirps_per_frame must be divisible by coded_tx_chirps")
        for name in (
            "sample_rate_hz",
            "slope_hz_per_s",
            "hpf_corner_hz",
            "minimum_trusted_range_m",
            "maximum_range_m",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.minimum_trusted_range_m >= self.maximum_range_m:
            raise ValueError("minimum_trusted_range_m must be below maximum_range_m")
        if not np.isfinite(self.range_compensation_exponent):
            raise ValueError("range_compensation_exponent must be finite")

    @property
    def frame_bytes(self) -> int:
        return (
            2
            * self.adc_samples
            * self.rx_antennas
            * self.chirps_per_frame
            * self.bursts_per_frame
        )

    @property
    def chirps_per_coded_tx(self) -> int:
        return (
            self.chirps_per_frame
            * self.bursts_per_frame
            // self.coded_tx_chirps
        )

    @property
    def hpf_corner_range_m(self) -> float:
        return (
            SPEED_OF_LIGHT_M_S
            * self.hpf_corner_hz
            / (2.0 * self.slope_hz_per_s)
        )

    @property
    def physical_range_resolution_m(self) -> float:
        return (
            SPEED_OF_LIGHT_M_S
            * self.sample_rate_hz
            / (2.0 * self.slope_hz_per_s * self.adc_samples)
        )


@dataclass(frozen=True)
class NearFieldPeak:
    range_m: float
    angle_deg: float
    x_m: float
    y_m: float
    relative_db: float
    spatial_coherence: float
    angular_prominence_db: float = 0.0


@dataclass(frozen=True)
class NearFieldMap:
    ranges_m: np.ndarray
    angles_deg: np.ndarray
    power_db: np.ndarray
    spatial_coherence: np.ndarray
    peaks: tuple[NearFieldPeak, ...]
    trusted_minimum_range_m: float
    maximum_range_m: float
    blind_zone_confidence: float = 0.0
    blind_zone_blocked: bool = False
    blind_zone_proxy_range_m: float = 0.0
    blind_zone_evidence_peak_m: float = 0.0


def unpack_fccsp_spi_bytes(payload: bytes) -> np.ndarray:
    """Convert TI's big-endian 32-bit SPI words into ordered signed int16s."""

    if len(payload) == 0 or len(payload) % 4:
        raise ValueError("SPI payload length must be a non-zero multiple of four")
    raw = np.frombuffer(payload, dtype=np.uint8).reshape(-1, 4)
    high = (raw[:, 0].astype(np.uint16) << 8) | raw[:, 1]
    low = (raw[:, 2].astype(np.uint16) << 8) | raw[:, 3]
    values = np.empty(raw.shape[0] * 2, dtype=np.uint16)
    values[0::2] = low
    values[1::2] = high
    return values.view(np.int16)


def reshape_spi_adc(
    values: np.ndarray,
    profile: NearFieldAdcProfile,
    *,
    frames: int,
) -> np.ndarray:
    """Match TI ``Test_read_adc_data_xWRLx432.m`` column-major ordering."""

    if isinstance(frames, bool) or not isinstance(frames, int) or frames <= 0:
        raise ValueError("frames must be a positive integer")
    samples = np.asarray(values, dtype=np.int16).reshape(-1)
    expected = frames * profile.frame_bytes // 2
    if samples.size != expected:
        raise ValueError(f"expected {expected} int16 samples, got {samples.size}")
    shaped = samples.reshape(
        (
            profile.adc_samples,
            profile.rx_antennas,
            profile.coded_tx_chirps,
            profile.chirps_per_coded_tx,
            frames,
        ),
        order="F",
    )
    return shaped.transpose(4, 2, 1, 3, 0).astype(np.float64)


def decode_bpm(coded: np.ndarray) -> np.ndarray:
    """Decode the ``TX0+TX1, TX0-TX1`` chirp pair into two virtual TXs."""

    adc = np.asarray(coded)
    if adc.ndim != 5 or adc.shape[1] != 2:
        raise ValueError("BPM ADC data must have shape [frame, 2, rx, chirp, sample]")
    decoded = np.empty_like(adc, dtype=np.float64)
    decoded[:, 0] = 0.5 * (adc[:, 0] + adc[:, 1])
    decoded[:, 1] = 0.5 * (adc[:, 0] - adc[:, 1])
    return decoded


def _range_fft(
    adc: np.ndarray,
    profile: NearFieldAdcProfile,
    *,
    fft_size: int,
) -> tuple[np.ndarray, np.ndarray]:
    if fft_size < profile.adc_samples:
        raise ValueError("fft_size cannot be smaller than adc_samples")
    centered = adc - adc.mean(axis=-1, keepdims=True)
    window = np.hanning(profile.adc_samples)
    spectrum = np.fft.rfft(centered * window, n=fft_size, axis=-1)
    ranges = (
        np.arange(spectrum.shape[-1], dtype=np.float64)
        * SPEED_OF_LIGHT_M_S
        * profile.sample_rate_hz
        / (2.0 * profile.slope_hz_per_s * fft_size)
    )
    return spectrum, ranges


def _azimuth_snapshots(spectrum: np.ndarray) -> np.ndarray:
    # A wall is a zero/very-low-Doppler return.  Treating each of the four
    # slow-time chirps as an independent spatial observation lets BPM phase
    # noise move a coherent static wall between opposite Capon lobes.  Form
    # the static slow-time component first (the zero-Doppler FFT bin, apart
    # from an irrelevant scale factor), then use frames as independent
    # covariance snapshots.  At the robot's intended crawling speed the
    # phase rotation over this four-chirp aperture is negligible.
    stationary = spectrum.mean(axis=3)
    # Virtual antenna order after BPM decoding is TX0/RX0..2, TX1/RX0..2.
    virtual = stationary.reshape(-1, 6, spectrum.shape[-1])
    # FCCSP reference phase coefficients from the TI profile.
    virtual *= np.array([1, -1, 1, -1, 1, -1], dtype=np.float64)[
        None, :, None
    ]
    # Row-zero virtual ULA ordered by azimuth columns 0, 1, 2, 3.
    return virtual[:, [0, 3, 2, 5], :]


def _capon_map(
    snapshots: np.ndarray,
    ranges: np.ndarray,
    angles_deg: np.ndarray,
    *,
    diagonal_loading: float = 0.08,
) -> tuple[np.ndarray, np.ndarray]:
    columns = np.arange(4, dtype=np.float64)
    steering = np.exp(
        1j
        * np.pi
        * np.outer(columns, np.sin(np.deg2rad(angles_deg)))
    )
    power = np.zeros((ranges.size, angles_deg.size), dtype=np.float64)
    coherence = np.zeros(ranges.size, dtype=np.float64)
    identity = np.eye(4, dtype=np.complex128)
    reversal = np.fliplr(identity)
    for range_index in range(ranges.size):
        data = snapshots[:, :, range_index]
        # Rows are temporal snapshots and columns are antennas.  The spatial
        # covariance is E[x x^H], therefore the antenna matrix must be
        # ``data.T @ data.conj()``.  Reversing those two conjugations forms
        # R* instead of R and mirrors every estimated azimuth.  TI's xWRL6432
        # geometry numbers the row-zero ULA from physical left to right, so a
        # positive phase progression must remain a positive/right azimuth.
        covariance = (data.T @ data.conj()) / max(data.shape[0], 1)
        # Static walls at one range are coherent sources.  A pure MVDR/Capon
        # inverse can then alternate between equally plausible left/right
        # lobes when only four virtual azimuth elements are available.
        # Forward/backward averaging restores the ULA symmetry, while the
        # conventional beam power supplies the stable, physically resolved
        # component.  Capon remains a small sharpening term rather than the
        # steering authority.
        covariance = 0.5 * (
            covariance
            + reversal @ covariance.conj() @ reversal
        )
        trace = max(float(np.real(np.trace(covariance))), 1e-12)
        covariance += identity * trace / 4.0 * diagonal_loading
        inverse = np.linalg.pinv(covariance)
        denominator = np.real(
            np.sum(steering.conj() * (inverse @ steering), axis=0)
        )
        capon = 1.0 / np.maximum(denominator, 1e-12)
        conventional = np.maximum(
            np.real(
                np.sum(
                    steering.conj() * (covariance @ steering),
                    axis=0,
                )
            ) / 16.0,
            1e-12,
        )
        # Preserve absolute range energy when putting the two estimators on
        # the same scale; per-angle normalization would erase range evidence.
        capon *= (
            float(np.max(conventional))
            / max(float(np.max(capon)), 1e-12)
        )
        power[range_index] = 0.80 * conventional + 0.20 * capon
        eigenvalues = np.maximum(np.linalg.eigvalsh(covariance), 0.0)
        coherence[range_index] = float(
            eigenvalues[-1] / max(float(eigenvalues.sum()), 1e-12)
        )
    return power, coherence


def _select_peaks(
    power_db: np.ndarray,
    ranges: np.ndarray,
    angles: np.ndarray,
    coherence: np.ndarray,
    valid_ranges: np.ndarray,
    *,
    count: int = 12,
) -> tuple[NearFieldPeak, ...]:
    candidates: list[tuple[float, int, int]] = []
    range_indices = np.where(valid_ranges)[0]
    for range_index in range_indices:
        for angle_index in range(angles.size):
            value = float(power_db[range_index, angle_index])
            r0 = max(0, range_index - 4)
            r1 = min(power_db.shape[0], range_index + 5)
            a0 = max(0, angle_index - 8)
            a1 = min(power_db.shape[1], angle_index + 9)
            if value >= float(np.max(power_db[r0:r1, a0:a1])):
                candidates.append((value, range_index, angle_index))
    candidates.sort(reverse=True)

    selected: list[NearFieldPeak] = []
    for value, range_index, angle_index in candidates:
        range_m = float(ranges[range_index])
        angle_deg = float(angles[angle_index])
        x_m = range_m * float(np.sin(np.deg2rad(angle_deg)))
        y_m = range_m * float(np.cos(np.deg2rad(angle_deg)))
        if any(
            (x_m - peak.x_m) ** 2 + (y_m - peak.y_m) ** 2 < 0.025**2
            for peak in selected
        ):
            continue
        selected.append(
            NearFieldPeak(
                range_m=range_m,
                angle_deg=angle_deg,
                x_m=x_m,
                y_m=y_m,
                relative_db=value,
                spatial_coherence=float(coherence[range_index]),
                angular_prominence_db=max(
                    0.0,
                    value - float(np.median(power_db[range_index])),
                ),
            )
        )
        if len(selected) >= count:
            break
    return tuple(selected)


def _blind_zone_guard(
    spectrum: np.ndarray,
    ranges: np.ndarray,
    profile: NearFieldAdcProfile,
    *,
    trusted_minimum_range_m: float,
) -> tuple[float, float]:
    """Detect a blocking return inside the analogue HPF blind zone.

    The 175 kHz receive HPF prevents an honest range/angle estimate below
    roughly 7 cm.  A very close broad obstacle still leaves a repeatable
    signature: energy moves from the sub-corner leakage band into the HPF
    transition band and produces broad spectral spill above the first trusted
    cell.  This guard only reports that the blind zone is blocked; it does not
    claim that the transition-band peak is the physical obstacle range.
    """

    values = np.asarray(spectrum)
    coordinates = np.asarray(ranges, dtype=np.float64)
    if values.ndim < 2 or values.shape[-1] != coordinates.size:
        raise ValueError("spectrum and ranges must share the range axis")
    power = np.mean(np.abs(values) ** 2, axis=tuple(range(values.ndim - 1)))
    leakage_band = (
        (coordinates >= profile.physical_range_resolution_m / 4.0)
        & (coordinates < profile.hpf_corner_range_m)
    )
    transition_band = (
        (coordinates >= profile.hpf_corner_range_m)
        & (coordinates < trusted_minimum_range_m)
    )
    trusted_band = (
        (coordinates >= trusted_minimum_range_m)
        & (coordinates <= profile.maximum_range_m)
    )
    if not (
        np.any(leakage_band)
        and np.any(transition_band)
        and np.any(trusted_band)
    ):
        return 0.0, 0.0

    floor = max(float(np.finfo(np.float64).tiny), 1e-12)
    leakage = max(float(np.mean(power[leakage_band])), floor)
    transition = max(float(np.mean(power[transition_band])), floor)
    trusted = max(float(np.mean(power[trusted_band])), floor)
    transition_delta_db = 10.0 * math.log10(transition / leakage)
    trusted_delta_db = 10.0 * math.log10(trusted / leakage)

    # The two terms reject the board's ordinary near-range coupling ring:
    # it is strong below the HPF corner but falls sharply in both following
    # bands.  A blocker inside the blind zone reverses the first relationship
    # and raises a broad, coherent spill into the trusted band.
    transition_score = 1.0 / (
        1.0 + math.exp(-(transition_delta_db - 0.5) / 0.75)
    )
    broad_spill_score = 1.0 / (
        1.0 + math.exp(-(trusted_delta_db + 10.0) / 1.5)
    )
    confidence = math.sqrt(transition_score * broad_spill_score)
    transition_indices = np.flatnonzero(transition_band)
    peak_index = int(transition_indices[np.argmax(power[transition_band])])
    return max(0.0, min(1.0, confidence)), float(coordinates[peak_index])


def process_near_field_adc(
    values: np.ndarray,
    *,
    frames: int,
    profile: NearFieldAdcProfile = NearFieldAdcProfile(),
    fft_size: int = 1024,
    angles_deg: Optional[np.ndarray] = None,
) -> NearFieldMap:
    """Build a robot-relative 0-30 cm evidence map from BPM RAW ADC samples."""

    coded = reshape_spi_adc(values, profile, frames=frames)
    adc = decode_bpm(coded)
    spectrum, ranges = _range_fft(adc, profile, fft_size=fft_size)
    angles = (
        # One-degree steering samples already oversample the approximately
        # 29-degree four-element aperture by a wide margin.  The old 0.25
        # degree grid quadrupled live latency without adding physical detail.
        np.linspace(-60.0, 60.0, 121)
        if angles_deg is None
        else np.asarray(angles_deg, dtype=np.float64)
    )
    if angles.ndim != 1 or angles.size < 2 or not np.all(np.isfinite(angles)):
        raise ValueError("angles_deg must be a finite one-dimensional vector")

    # A reported FFT coordinate is the centre of a finite-width range cell.
    # Keep the first cell whose complete half-resolution footprint is beyond
    # the requested near-range boundary.  The rejected partial cell is where
    # the live board shows the broad, angle-independent TX/RX coupling ring;
    # treating its centre as a target manufactured repeatable 10 cm walls.
    # This is a hardware-validity gate, not a signal-amplitude threshold.
    trusted_minimum = max(
        profile.minimum_trusted_range_m
        + profile.physical_range_resolution_m / 2.0,
        profile.hpf_corner_range_m
        + profile.physical_range_resolution_m,
    )
    blind_zone_confidence, blind_zone_evidence_peak_m = _blind_zone_guard(
        spectrum,
        ranges,
        profile,
        trusted_minimum_range_m=trusted_minimum,
    )
    valid = (ranges >= trusted_minimum) & (ranges <= profile.maximum_range_m)
    if not np.any(valid):
        raise ValueError("profile produces no bins inside the trusted range")

    # Capon is the dominant host cost.  The previous implementation solved a
    # covariance system for every FFT bin out to roughly 2.5 m even though
    # this profile publishes only 0-30 cm.  Restrict angle processing to the
    # physical interval before forming covariance matrices, then place those
    # results back into the full range axis for the unchanged public contract.
    snapshots = _azimuth_snapshots(spectrum[..., valid])
    valid_power, valid_coherence = _capon_map(
        snapshots,
        ranges[valid],
        angles,
    )
    power = np.zeros((ranges.size, angles.size), dtype=np.float64)
    coherence = np.zeros(ranges.size, dtype=np.float64)
    power[valid] = valid_power
    coherence[valid] = valid_coherence

    # An extended rubble wall lies between the R^-2 and point-target R^-4
    # radar equations.  R^-3 is the conservative default; it prevents the
    # analogue HPF transition from hiding genuine 20-30 cm returns without
    # manufacturing occupied cells where no measured energy exists.
    compensation = np.maximum(ranges, trusted_minimum) ** (
        profile.range_compensation_exponent
    )
    compensated = power * compensation[:, None]
    reference = max(float(np.percentile(compensated[valid], 99.8)), 1e-12)
    power_db = 10.0 * np.log10(np.maximum(compensated / reference, 1e-8))
    power_db[~valid] = -80.0
    peaks = _select_peaks(power_db, ranges, angles, coherence, valid)
    return NearFieldMap(
        ranges_m=ranges,
        angles_deg=angles,
        power_db=power_db,
        spatial_coherence=coherence,
        peaks=peaks,
        trusted_minimum_range_m=trusted_minimum,
        maximum_range_m=profile.maximum_range_m,
        blind_zone_confidence=blind_zone_confidence,
        blind_zone_blocked=(
            blind_zone_confidence >= BLIND_ZONE_BLOCK_CONFIDENCE
        ),
        # This is deliberately a conservative proxy inside the unresolvable
        # interval, not a fabricated range estimate.
        blind_zone_proxy_range_m=profile.hpf_corner_range_m / 2.0,
        blind_zone_evidence_peak_m=blind_zone_evidence_peak_m,
    )


def _read_command(length: int) -> bytes:
    if isinstance(length, bool) or not isinstance(length, int):
        raise ValueError("length must be an integer")
    if length < 1 or length > FTDI_MAX_TRANSFER_BYTES:
        raise ValueError("length must be between 1 and 65536")
    encoded = (length - 1).to_bytes(2, "little")
    return (
        _MPSSE_CS_LOW
        + _MPSSE_READ_BYTES_POSITIVE_EDGE_MSB
        + encoded
        + _MPSSE_CS_HIGH
    )


class FtdiFccspCapture:
    """Small D2XX/MPSSE capture wrapper matching TI's FCCSP reference tool."""

    def __init__(
        self,
        *,
        device_index: int = 0,
        clock_hz: int = 15_000_000,
        handle_factory: Optional[Callable[[int], Any]] = None,
    ) -> None:
        if isinstance(device_index, bool) or device_index < 0:
            raise ValueError("device_index must be a non-negative integer")
        if isinstance(clock_hz, bool) or clock_hz < 1 or clock_hz > 30_000_000:
            raise ValueError("clock_hz must be between 1 and 30000000")
        self._device_index = device_index
        self._clock_hz = clock_hz
        self._handle_factory = handle_factory
        self._handle: Any = None

    def __enter__(self) -> "FtdiFccspCapture":
        if self._handle_factory is None:
            try:
                import ftd2xx  # type: ignore[import-not-found]
            except ImportError as exc:  # pragma: no cover - environment-specific
                raise RuntimeError(
                    "ftd2xx is required for the C232HM capture backend"
                ) from exc
            factory = ftd2xx.open
        else:
            factory = self._handle_factory
        self._handle = factory(self._device_index)
        self._configure()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            handle.close()

    def _require_handle(self) -> Any:
        if self._handle is None:
            raise RuntimeError("FTDI capture is not open")
        return self._handle

    def _configure(self) -> None:
        handle = self._require_handle()
        handle.resetDevice()
        rx_bytes, _tx_bytes, _event_status = handle.getStatus()
        if rx_bytes:
            handle.read(rx_bytes)
        handle.setUSBParameters(65_535, 65_535)
        handle.setChars(False, 0, False, 0)
        handle.setTimeouts(200_000, 200_000)
        handle.setLatencyTimer(1)
        handle.setBitMode(0, 0)
        handle.setBitMode(0, 2)
        time.sleep(0.05)
        handle.write(_MPSSE_DISABLE_DIVIDE_BY_5)
        handle.write(_MPSSE_DISABLE_ADAPTIVE_CLOCK)
        handle.write(_MPSSE_DISABLE_THREE_PHASE_CLOCK)
        handle.write(_MPSSE_SET_LOW_BYTE_IDLE)
        divisor = int(60_000_000 / (self._clock_hz * 2) - 1)
        handle.write(bytes((0x86, divisor & 0xFF, divisor >> 8)))
        time.sleep(0.02)
        handle.write(_MPSSE_DISABLE_LOOPBACK)
        time.sleep(0.03)

    def read_gpio(self) -> int:
        handle = self._require_handle()
        handle.write(_MPSSE_READ_LOW_BYTE)
        raw = handle.read(1)
        if len(raw) != 1:
            raise RuntimeError("short FTDI GPIO read")
        return int.from_bytes(raw, "big")

    def wait_until_ready(self, *, timeout_s: float = 5.0) -> None:
        if not np.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive and finite")
        deadline = time.monotonic() + timeout_s
        while self.read_gpio() & FCCSP_BUSY_MASK:
            if time.monotonic() >= deadline:
                raise TimeoutError("radar SPI_BUSY did not assert data-ready")
            time.sleep(0.0005)

    def read_frame(self, frame_bytes: int, *, ready_timeout_s: float = 5.0) -> bytes:
        if isinstance(frame_bytes, bool) or not isinstance(frame_bytes, int):
            raise ValueError("frame_bytes must be an integer")
        if frame_bytes <= 0:
            raise ValueError("frame_bytes must be positive")
        self.wait_until_ready(timeout_s=ready_timeout_s)
        handle = self._require_handle()
        remaining = frame_bytes
        chunks: list[bytes] = []
        while remaining:
            request = min(remaining, FTDI_MAX_TRANSFER_BYTES)
            handle.write(_read_command(request))
            received = bytearray()
            while len(received) < request:
                part = handle.read(request - len(received))
                if not part:
                    raise RuntimeError("short FTDI SPI frame read")
                received.extend(part)
            chunks.append(bytes(received))
            remaining -= request
        return b"".join(chunks)

    def capture_frames(
        self,
        count: int,
        frame_bytes: int,
        *,
        first_ready_timeout_s: float = 45.0,
        ready_timeout_s: float = 5.0,
    ) -> tuple[bytes, ...]:
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("count must be a positive integer")
        frames = [
            self.read_frame(frame_bytes, ready_timeout_s=first_ready_timeout_s)
        ]
        frames.extend(
            self.read_frame(frame_bytes, ready_timeout_s=ready_timeout_s)
            for _ in range(count - 1)
        )
        return tuple(frames)


__all__ = [
    "FCCSP_BUSY_MASK",
    "FTDI_MAX_TRANSFER_BYTES",
    "FtdiFccspCapture",
    "NearFieldAdcProfile",
    "NearFieldMap",
    "NearFieldPeak",
    "decode_bpm",
    "process_near_field_adc",
    "reshape_spi_adc",
    "unpack_fccsp_spi_bytes",
]
