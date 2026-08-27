"""PC processing for IWRL6432 UART last-chirp-pair ADC data.

The UART TLV is not a complete frame, but its two BPM chirps preserve one
complex range sample per virtual antenna.  Keeping those phases separate
allows an honest, coarse range/azimuth image instead of the old range-only
non-coherent sum.
"""

from __future__ import annotations

import math
import statistics
import struct
import cmath
from collections import deque
from typing import Deque, Dict, List, Mapping, Sequence, Tuple

from common.sensor_contract import RadarAdcSnapshot


EVM_VIRTUAL_ROWS = (0, 1, 0, 0, 1, 0)
EVM_VIRTUAL_COLUMNS = (0, 1, 2, 1, 2, 3)
EVM_DESIGN_COMPENSATION = (1.0, -1.0, 1.0, -1.0, 1.0, -1.0)
AZIMUTH_VIRTUAL_INDICES = (0, 3, 2, 5)
# The first two positive FFT bins (5.9 and 11.7 cm in the live profile) are
# dominated by board/USB direct coupling.  Keep them in the display surface,
# but never promote them to persistent obstacle evidence.
ADC_MIN_RANGE_INDEX = 3
ADC_MAX_RANGE_M = 0.30
ADC_ANGLE_MIN_DEG = -60
ADC_ANGLE_MAX_DEG = 60
ADC_ANGLE_STEP_DEG = 2
ADC_ANGULAR_RESOLUTION_DEG = 29.0


def _validated_snapshot(
    snapshot: RadarAdcSnapshot,
    range_step_m: float,
) -> Tuple[int, Tuple[int, ...], List[float]]:
    if not isinstance(snapshot, RadarAdcSnapshot):
        raise ValueError("snapshot must be a RadarAdcSnapshot")
    if not math.isfinite(range_step_m) or range_step_m <= 0:
        raise ValueError("range_step_m must be finite and positive")
    if snapshot.chirps != 2 or snapshot.rx_channels != 3:
        raise ValueError("UART angle processing requires 2 chirps and 3 RX")
    count = snapshot.samples_per_chirp
    if count < 2 or count & (count - 1):
        raise ValueError("samples_per_chirp must be a power of two")
    samples = struct.unpack(
        f"<{count * snapshot.rx_channels * snapshot.chirps}h",
        snapshot.data,
    )
    window = [
        0.5 - 0.5 * math.cos(2.0 * math.pi * index / (count - 1))
        for index in range(count)
    ]
    return count, samples, window


def _virtual_range_spectra(
    snapshot: RadarAdcSnapshot,
    range_step_m: float,
) -> List[List[complex]]:
    count, samples, window = _validated_snapshot(snapshot, range_step_m)
    chirp_stride = snapshot.rx_channels * count
    tx0: List[List[complex]] = []
    tx1: List[List[complex]] = []
    for rx in range(snapshot.rx_channels):
        first = samples[rx * count:(rx + 1) * count]
        second_start = chirp_stride + rx * count
        second = samples[second_start:second_start + count]
        decoded = (
            [(a + b) * 0.5 for a, b in zip(first, second)],
            [(a - b) * 0.5 for a, b in zip(first, second)],
        )
        for output, signal in zip((tx0, tx1), decoded):
            mean = sum(signal) / count
            output.append(_fft([
                complex((sample - mean) * window[index], 0.0)
                for index, sample in enumerate(signal)
            ]))
    virtual = [*tx0, *tx1]
    return [
        [value * EVM_DESIGN_COMPENSATION[index] for value in spectrum]
        for index, spectrum in enumerate(virtual)
    ]


def _steered_power(symbols: Sequence[complex], azimuth_deg: float) -> float:
    phase_step = math.pi * math.sin(math.radians(azimuth_deg))
    result = sum(
        symbol
        * complex(
            math.cos(-phase_step * column),
            math.sin(-phase_step * column),
        )
        for symbol, column in zip(symbols, range(len(symbols)))
    )
    return result.real * result.real + result.imag * result.imag


def _coherence(symbols: Sequence[complex], azimuth_deg: float) -> float:
    phase_step = math.pi * math.sin(math.radians(azimuth_deg))
    coherent = sum(
        symbol
        * complex(
            math.cos(-phase_step * column),
            math.sin(-phase_step * column),
        )
        for symbol, column in zip(symbols, range(len(symbols)))
    )
    denominator = len(symbols) * sum(
        value.real * value.real + value.imag * value.imag
        for value in symbols
    )
    return 0.0 if denominator <= 0.0 else min(
        1.0,
        (coherent.real * coherent.real + coherent.imag * coherent.imag)
        / denominator,
    )


def _two_source_angles(symbols: Sequence[complex]) -> List[float]:
    """Resolve exactly two plane waves with the four-element ULA.

    The UART snapshot has only four azimuth elements, so a general image is
    underdetermined.  The second-order annihilating relation is nevertheless
    sufficient for the safety-relevant case of two walls at the same range.
    A scale-free determinant rejects the degenerate single-source case; this
    is model-order topology, not an amplitude gate.
    """
    if len(symbols) != 4:
        return []
    x0, x1, x2, x3 = symbols
    determinant = x1 * (-x1) - (-x0) * x2
    scale = sum(abs(value) for value in symbols) ** 2
    if scale <= 0.0 or abs(determinant) / scale <= 1e-3:
        return []
    root_sum = (x2 * (-x1) - (-x0) * x3) / determinant
    root_product = (x1 * x3 - x2 * x2) / determinant
    discriminant = cmath.sqrt(
        root_sum * root_sum - 4.0 * root_product
    )
    roots = (
        (root_sum + discriminant) / 2.0,
        (root_sum - discriminant) / 2.0,
    )
    angles = []
    for root in roots:
        if not 0.75 <= abs(root) <= 1.25:
            return []
        phase = cmath.phase(root)
        argument = max(-1.0, min(1.0, phase / math.pi))
        angle = math.degrees(math.asin(argument))
        if not ADC_ANGLE_MIN_DEG <= angle <= ADC_ANGLE_MAX_DEG:
            return []
        angles.append(angle)
    if abs(angles[0] - angles[1]) < ADC_ANGULAR_RESOLUTION_DEG:
        return []
    return sorted(angles)


def adc_range_angle_frame(
    snapshot: RadarAdcSnapshot,
    *,
    range_step_m: float,
) -> Dict[str, object]:
    """Return the current, threshold-free UART range/azimuth evidence."""

    virtual = _virtual_range_spectra(snapshot, range_step_m)
    azimuth_symbols = [virtual[index] for index in AZIMUTH_VIRTUAL_INDICES]
    angles = list(range(
        ADC_ANGLE_MIN_DEG,
        ADC_ANGLE_MAX_DEG + 1,
        ADC_ANGLE_STEP_DEG,
    ))
    maximum_range_index = min(
        snapshot.samples_per_chirp // 2 - 1,
        int(math.floor(ADC_MAX_RANGE_M / range_step_m + 1e-12)),
    )
    raw_rows: Dict[int, List[float]] = {}
    coherent_rows: Dict[int, List[float]] = {}
    all_db: List[float] = []
    for range_index in range(1, maximum_range_index + 1):
        symbols = [spectrum[range_index] for spectrum in azimuth_symbols]
        # The EVM virtual columns are ordered to match TI's native heatmap:
        # positive receive phase progression is positive map azimuth.
        powers = [_steered_power(symbols, angle) for angle in angles]
        db = [10.0 * math.log10(power + 1e-12) for power in powers]
        raw_rows[range_index] = db
        coherent_rows[range_index] = [
            _coherence(symbols, angle) for angle in angles
        ]
        all_db.extend(db)
    if not all_db or max(all_db) - min(all_db) <= 1e-9:
        floor_db, ceiling_db = 0.0, 1.0
    else:
        centre = statistics.median(all_db)
        mad = max(
            1.0,
            statistics.median(abs(value - centre) for value in all_db),
        )
        floor_db = centre - 2.0 * mad
        ceiling_db = max(floor_db + 1.0, centre + 8.0 * mad)
    span_db = ceiling_db - floor_db
    cells: List[Dict[str, object]] = []
    peaks: List[Dict[str, object]] = []
    for range_index in sorted(raw_rows):
        row = raw_rows[range_index]
        row_median = statistics.median(row)
        range_m = range_index * range_step_m
        for angle_index, azimuth_deg in enumerate(angles):
            azimuth_rad = math.radians(azimuth_deg)
            cells.append({
                "range_index": range_index,
                "azimuth_index": angle_index,
                "range_m": round(range_m, 6),
                "azimuth_deg": float(azimuth_deg),
                "forward_m": round(range_m * math.cos(azimuth_rad), 6),
                "lateral_m": round(range_m * math.sin(azimuth_rad), 6),
                "intensity": round(max(0.0, min(
                    1.0, (row[angle_index] - floor_db) / span_db,
                )), 6),
                "relative_db": round(row[angle_index] - row_median, 6),
                "coherence": round(coherent_rows[range_index][angle_index], 6),
            })
        if range_index < ADC_MIN_RANGE_INDEX:
            continue
        if max(row) - min(row) <= 1e-9:
            continue
        symbols = [spectrum[range_index] for spectrum in azimuth_symbols]
        resolved_angles = _two_source_angles(symbols)
        if resolved_angles:
            selected_angles = [
                min(angles, key=lambda value: abs(value - resolved))
                for resolved in resolved_angles
            ]
        else:
            selected_angles = [angles[max(
                range(len(row)), key=row.__getitem__
            )]]
        for azimuth_deg in selected_angles:
            peak_index = angles.index(azimuth_deg)
            peak_db = row[peak_index]
            relative_db = peak_db - row_median
            coherence = coherent_rows[range_index][peak_index]
            if relative_db <= 0.0:
                continue
            azimuth_rad = math.radians(azimuth_deg)
            peaks.append({
                "range_index": range_index,
                "range_m": round(range_m, 6),
                "azimuth_deg": float(azimuth_deg),
                "forward_m": round(range_m * math.cos(azimuth_rad), 6),
                "lateral_m": round(range_m * math.sin(azimuth_rad), 6),
                "relative_db": round(relative_db, 6),
                "coherence": round(coherence, 6),
            })
    return {
        "source": "uart_last_chirp_pair_adc",
        "direction_semantics": "range_azimuth",
        "processing": "bpm_decode_hann_range_fft_four_element_das",
        "phase_calibration": "evm_design_default",
        "range_bins": maximum_range_index + 1,
        "range_step_m": round(float(range_step_m), 9),
        "angle_min_deg": ADC_ANGLE_MIN_DEG,
        "angle_max_deg": ADC_ANGLE_MAX_DEG,
        "angle_step_deg": ADC_ANGLE_STEP_DEG,
        "angular_resolution_deg": ADC_ANGULAR_RESOLUTION_DEG,
        "floor_db": round(floor_db, 3),
        "ceiling_db": round(ceiling_db, 3),
        "cells": cells,
        "peaks": peaks,
        "complete_frame_adc": False,
    }


class RadarAdcAngularSurface:
    """Five-frame temporal persistence for coarse UART angle evidence."""

    def __init__(self, history_frames: int = 5, required_hits: int = 3):
        if history_frames < 1 or not 1 <= required_hits <= history_frames:
            raise ValueError("invalid ADC surface history")
        self.history_frames = int(history_frames)
        self.required_hits = int(required_hits)
        self._history: Deque[Tuple[float, Dict[str, object]]] = deque(
            maxlen=self.history_frames
        )

    def reset(self) -> None:
        self._history.clear()

    def ingest(
        self,
        snapshot: RadarAdcSnapshot,
        *,
        range_step_m: float,
        received_at: float,
        elapsed_frame_windows: int = 1,
        lateral_sign: int = 1,
    ) -> None:
        if not math.isfinite(received_at):
            raise ValueError("received_at must be finite")
        if not isinstance(elapsed_frame_windows, int) or isinstance(
            elapsed_frame_windows, bool
        ) or elapsed_frame_windows < 1:
            raise ValueError("elapsed_frame_windows must be positive")
        if lateral_sign not in (-1, 1) or isinstance(lateral_sign, bool):
            raise ValueError("lateral_sign must be -1 or 1")
        if self._history and received_at < self._history[-1][0]:
            raise ValueError("received_at must not move backwards")
        for _ in range(min(elapsed_frame_windows - 1, self.history_frames)):
            self._history.append((received_at, {"cells": [], "peaks": []}))
        frame = adc_range_angle_frame(snapshot, range_step_m=range_step_m)
        if lateral_sign == -1:
            for collection in (frame["cells"], frame["peaks"]):
                for item in collection:
                    item["azimuth_deg"] = -float(item["azimuth_deg"])
                    item["lateral_m"] = -float(item["lateral_m"])
        self._history.append((received_at, frame))

    def payload(self, now: float) -> Dict[str, object]:
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if not self._history:
            return {
                "source": "uart_last_chirp_pair_adc",
                "direction_semantics": "range_azimuth",
                "phase_calibration": "evm_design_default",
                "history_frames": self.history_frames,
                "required_hits": self.required_hits,
                "retention": "three_of_five_complete_frames",
                "range_step_m": None,
                "angular_resolution_deg": ADC_ANGULAR_RESOLUTION_DEG,
                "complete_frame_adc": False,
                "cells": [],
                "peaks": [],
            }
        latest = self._history[-1][1]
        grouped: Dict[Tuple[int, int], List[Tuple[float, Mapping[str, object]]]] = {}
        # One EVM beam is about 29 degrees wide.  Quantising persistence at
        # half a beam lets small frame-to-frame peak jitter contribute to the
        # same physical reflection without merging opposite-side returns.
        persistence_angle_step = ADC_ANGULAR_RESOLUTION_DEG / 2.0
        for observed_at, frame in self._history:
            for peak in frame["peaks"]:
                key = (
                    int(peak["range_index"]),
                    round(float(peak["azimuth_deg"]) /
                          persistence_angle_step),
                )
                grouped.setdefault(key, []).append((observed_at, peak))
        peaks: List[Dict[str, object]] = []
        for key, observations in sorted(grouped.items()):
            latest_observed_at, latest_peak = max(
                observations, key=lambda item: item[0]
            )
            hits = len(observations)
            relative_values = [float(item[1]["relative_db"]) for item in observations]
            coherence_values = [float(item[1]["coherence"]) for item in observations]
            variation = statistics.median(abs(
                value - statistics.median(relative_values)
            ) for value in relative_values)
            temporal = min(1.0, hits / self.required_hits)
            confidence = temporal * statistics.median(coherence_values) * math.exp(
                -variation / 6.0
            )
            peaks.append({
                **dict(latest_peak),
                "relative_db": round(statistics.median(relative_values), 6),
                "coherence": round(statistics.median(coherence_values), 6),
                "confidence": round(max(0.0, min(1.0, confidence)), 6),
                "hits": hits,
                "verified": hits >= self.required_hits,
                "age_ms": max(0, round((now - latest_observed_at) * 1000)),
                "angular_uncertainty_deg": ADC_ANGULAR_RESOLUTION_DEG / 2.0,
            })
        return {
            "source": latest["source"],
            "direction_semantics": latest["direction_semantics"],
            "processing": latest["processing"],
            "phase_calibration": latest["phase_calibration"],
            "history_frames": self.history_frames,
            "required_hits": self.required_hits,
            "retention": "three_of_five_complete_frames",
            "range_step_m": latest["range_step_m"],
            "angle_min_deg": latest["angle_min_deg"],
            "angle_max_deg": latest["angle_max_deg"],
            "angle_step_deg": latest["angle_step_deg"],
            "angular_resolution_deg": latest["angular_resolution_deg"],
            "complete_frame_adc": False,
            "cells": [dict(cell) for cell in latest["cells"]],
            "peaks": peaks,
        }


def fused_adc_heatmap_measurements(
    adc_surface: Mapping[str, object],
    heatmap_cells: Sequence[Mapping[str, object]],
) -> List[Dict[str, object]]:
    """Legacy consistency matcher; never use as independent occupancy input.

    Both arguments originate from one radar frame and antenna aperture.
    Agreement describes consistency, not independent confirmation.
    """

    range_step_m = adc_surface.get("range_step_m")
    angular_resolution_deg = adc_surface.get("angular_resolution_deg")
    if not isinstance(range_step_m, (int, float)) or not math.isfinite(
        float(range_step_m)
    ) or float(range_step_m) <= 0.0:
        return []
    if not isinstance(angular_resolution_deg, (int, float)) or not math.isfinite(
        float(angular_resolution_deg)
    ) or float(angular_resolution_deg) <= 0.0:
        return []
    result: List[Dict[str, object]] = []
    for peak in adc_surface.get("peaks", []):
        if not isinstance(peak, Mapping) or peak.get("verified") is not True:
            continue
        matches = [
            cell for cell in heatmap_cells
            if cell.get("verified") is True
            and float(cell.get("relative_db", 0.0)) > 0.0
            and abs(float(cell["range_m"]) - float(peak["range_m"]))
            <= float(range_step_m) / 2.0 + 1e-9
            and abs(float(cell["azimuth_deg"]) - float(peak["azimuth_deg"]))
            <= float(angular_resolution_deg) / 2.0 + 1e-9
        ]
        if not matches:
            continue
        match = max(matches, key=lambda cell: float(cell["confidence"]))
        adc_confidence = max(0.0, min(1.0, float(peak["confidence"])))
        heatmap_confidence = max(0.0, min(1.0, float(match["confidence"])))
        confidence = math.sqrt(adc_confidence * heatmap_confidence)
        probability = min(0.969, 0.5 + 0.469 * confidence)
        result.append({
            "forward_m": float(peak["forward_m"]),
            "lateral_m": float(peak["lateral_m"]),
            "range_m": float(peak["range_m"]),
            "azimuth_deg": float(peak["azimuth_deg"]),
            "measurement_probability": probability,
            "confidence": confidence,
            "source": "adc_heatmap_fused",
            "range_uncertainty_m": max(
                float(range_step_m) / 2.0,
                float(match.get("range_uncertainty_m", 0.0)),
            ),
            "lateral_uncertainty_m": max(
                0.025,
                float(peak["range_m"]) * math.sin(math.radians(
                    float(angular_resolution_deg) / 2.0
                )),
            ),
        })
    return result


def _fft(values: List[complex]) -> List[complex]:
    """In-place radix-2 FFT returned as a detached list."""

    size = len(values)
    if size < 2 or size & (size - 1):
        raise ValueError("samples_per_chirp must be a power of two")
    result = list(values)
    target = 0
    for source in range(1, size):
        bit = size >> 1
        while target & bit:
            target ^= bit
            bit >>= 1
        target ^= bit
        if source < target:
            result[source], result[target] = result[target], result[source]
    length = 2
    while length <= size:
        root = complex(
            math.cos(-2.0 * math.pi / length),
            math.sin(-2.0 * math.pi / length),
        )
        for start in range(0, size, length):
            factor = 1.0 + 0.0j
            half = length // 2
            for offset in range(half):
                even = result[start + offset]
                odd = result[start + offset + half] * factor
                result[start + offset] = even + odd
                result[start + offset + half] = even - odd
                factor *= root
        length *= 2
    return result


def adc_range_profile(
    snapshot: RadarAdcSnapshot,
    *,
    range_step_m: float,
) -> Dict[str, object]:
    """Return a full, threshold-free relative range-energy profile.

    Two BPM chirps are decoded into sum/difference virtual transmit channels.
    Magnitude is combined only in range; no angle is claimed because this TLV
    lacks a full frame and deployment phase calibration.
    """

    count, samples, window = _validated_snapshot(snapshot, range_step_m)
    powers = [0.0] * (count // 2)
    chirp_stride = snapshot.rx_channels * count
    for rx in range(snapshot.rx_channels):
        first = samples[rx * count:(rx + 1) * count]
        second_start = chirp_stride + rx * count
        second = samples[second_start:second_start + count]
        for virtual in (
            [(a + b) * 0.5 for a, b in zip(first, second)],
            [(a - b) * 0.5 for a, b in zip(first, second)],
        ):
            mean = sum(virtual) / count
            spectrum = _fft([
                complex((sample - mean) * window[index], 0.0)
                for index, sample in enumerate(virtual)
            ])
            for index in range(count // 2):
                powers[index] += spectrum[index].real ** 2 + spectrum[index].imag ** 2
    if max(powers, default=0.0) <= 0.0:
        values = [0] * len(powers)
        floor_db, ceiling_db = 0.0, 1.0
        peak_bin = None
    else:
        db = [10.0 * math.log10(power + 1e-12) for power in powers]
        finite = db[1:] if len(db) > 1 else db
        centre = statistics.median(finite)
        deviations = [abs(value - centre) for value in finite]
        mad = max(1.0, statistics.median(deviations))
        floor_db = centre - 2.0 * mad
        ceiling_db = max(floor_db + 1.0, centre + 8.0 * mad)
        span = ceiling_db - floor_db
        values = [
            round(255.0 * max(0.0, min(1.0, (value - floor_db) / span)))
            for value in db
        ]
        peak_bin = max(range(1, len(powers)), key=powers.__getitem__)
    return {
        "source": "uart_last_chirp_pair_adc",
        "direction_semantics": "range_only",
        "processing": "bpm_decode_hann_range_fft_noncoherent_rx_sum",
        "range_bins": len(powers),
        "range_step_m": round(float(range_step_m), 9),
        "values": values,
        "floor_db": round(floor_db, 3),
        "ceiling_db": round(ceiling_db, 3),
        "peak_bin": peak_bin,
        "peak_range_m": (
            None if peak_bin is None
            else round(peak_bin * float(range_step_m), 6)
        ),
        "complete_frame_adc": False,
    }


__all__ = [
    "RadarAdcAngularSurface",
    "adc_range_angle_frame",
    "adc_range_profile",
    "fused_adc_heatmap_measurements",
]
