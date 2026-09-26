"""Range-ordered inverse sensor model for the operator radar view.

The angle FFT contains correlated bins and multipath can be brighter than the
first physical surface.  This module reduces the image to independent antenna
beams and estimates the first visible surface along each beam.  Signal level
changes probability continuously; it is never used as an on/off threshold.
"""

from __future__ import annotations

import math
import statistics
from typing import Dict, Iterable, List, Mapping, Sequence


DEFAULT_BEAM_CENTRES_DEG = (-48.0, -24.0, 0.0, 24.0, 48.0)
DEFAULT_BEAM_WIDTH_DEG = 29.0


def first_surface_measurements(
    cells: Sequence[Mapping[str, object]],
    *,
    minimum_range_m: float,
    maximum_range_m: float,
    beam_centres_deg: Iterable[float] = DEFAULT_BEAM_CENTRES_DEG,
    beam_width_deg: float = DEFAULT_BEAM_WIDTH_DEG,
    corroborating_peaks: Sequence[Mapping[str, object]] = (),
    exclude_near_boundary: bool = False,
) -> List[Dict[str, float]]:
    """Return at most one first-surface likelihood per physical beam."""
    centres = tuple(float(value) for value in beam_centres_deg)
    rows: Dict[float, List[Mapping[str, object]]] = {}
    for cell in cells:
        range_m = float(cell["range_m"])
        # A return exactly on the trusted near boundary is left-censored: its
        # radial maximum may be inside the deliberately rejected coupling/HPF
        # interval.  It remains available to the operator heatmap, but cannot
        # prove an obstacle or clear space in the inverse sensor model.
        above_near_boundary = (
            range_m > minimum_range_m + 1e-9
            if exclude_near_boundary
            else range_m >= minimum_range_m
        )
        if above_near_boundary and range_m <= maximum_range_m + 1e-9:
            rows.setdefault(range_m, []).append(cell)

    def hypothesis(
        range_m: float,
        row: Sequence[Mapping[str, object]],
    ) -> Dict[str, float] | None:
        positive = [
            cell for cell in row
            if float(cell["relative_db"]) > 0.0
        ]
        if len(positive) < 2:
            return None
        relative_db = statistics.mean(
            float(cell["relative_db"]) for cell in positive
        )
        confidence = statistics.mean(
            float(cell["confidence"]) for cell in positive
        )
        persistence = statistics.mean(
            float(cell.get("persistence", 1.0)) for cell in positive
        )
        amplitude_likelihood = 1.0 - math.exp(-relative_db / 1.5)
        evidence_probability = max(0.0, min(
            0.995,
            amplitude_likelihood
            * confidence
            * (0.5 + 0.5 * persistence),
        ))
        return {
            "range_m": range_m,
            "relative_db": relative_db,
            "confidence": confidence,
            "evidence_probability": evidence_probability,
            "range_uncertainty_m": max(
                float(cell["range_uncertainty_m"])
                for cell in positive
            ),
        }

    def measurement(
        selected: Mapping[str, float],
        angle_deg: float,
        selected_probability: float,
    ) -> Dict[str, float]:
        angle_rad = math.radians(angle_deg)
        distance_m = selected["range_m"]
        return {
            "range_m": distance_m,
            "azimuth_deg": angle_deg,
            "forward_m": distance_m * math.cos(angle_rad),
            "lateral_m": distance_m * math.sin(angle_rad),
            "measurement_probability": min(
                0.969, 0.5 + 0.469 * selected_probability
            ),
            "confidence": selected_probability,
            "relative_db": selected["relative_db"],
            "range_uncertainty_m": selected["range_uncertainty_m"],
            "angular_uncertainty_deg": beam_width_deg / 2.0,
        }

    # The last-chirp ADC angle and the firmware heatmap are correlated views
    # of the same reflection.  Once an ADC direction is temporally verified,
    # use the heatmap only as radial/amplitude support and replace its coarse
    # angle-bin interpretation.  Never add both views as two obstacles.
    directed_by_beam: Dict[float, Dict[str, object]] = {}
    for peak in corroborating_peaks:
        if peak.get("verified") is not True:
            continue
        peak_range = float(peak["range_m"])
        peak_azimuth = float(peak["azimuth_deg"])
        peak_relative_db = float(peak.get("relative_db", 0.0))
        peak_coherence = float(peak.get("coherence", 0.0))
        if not (
            math.isfinite(peak_range)
            and math.isfinite(peak_azimuth)
            and math.isfinite(peak_relative_db)
            and math.isfinite(peak_coherence)
            and peak_range > 0.0
            and peak_relative_db > 0.0
            and peak_coherence > 0.0
        ):
            continue
        centre_deg = min(
            centres, key=lambda candidate: abs(candidate - peak_azimuth)
        )
        if abs(peak_azimuth - centre_deg) > beam_width_deg / 2.0:
            continue
        supported = []
        for range_m, full_row in sorted(rows.items()):
            row_hypothesis = hypothesis(range_m, full_row)
            if row_hypothesis is None:
                continue
            if abs(peak_range - range_m) > (
                row_hypothesis["range_uncertainty_m"] * 1.5 + 1e-9
            ):
                continue
            supported.append((
                row_hypothesis["evidence_probability"]
                * min(1.0, peak_coherence),
                -abs(peak_range - range_m),
                row_hypothesis,
            ))
        if not supported:
            continue
        selected_probability, _, selected = max(
            supported, key=lambda item: (item[0], item[1])
        )
        previous = directed_by_beam.get(centre_deg)
        if (
            previous is None
            or selected_probability > previous["probability"]
        ):
            directed_by_beam[centre_deg] = {
                "probability": selected_probability,
                "selected": selected,
                "angle_deg": peak_azimuth,
            }

    measurements: List[Dict[str, float]] = []
    consumed_ranges = {
        float(item["selected"]["range_m"])
        for item in directed_by_beam.values()
    }
    for centre_deg in centres:
        directed = directed_by_beam.get(centre_deg)
        if directed is not None:
            measurements.append(measurement(
                directed["selected"],
                float(directed["angle_deg"]),
                float(directed["probability"]),
            ))
            continue

        hypotheses: List[Dict[str, float]] = []
        for range_m, full_row in sorted(rows.items()):
            if range_m in consumed_ranges:
                continue
            beam_row = [
                cell for cell in full_row
                if min(
                    centres,
                    key=lambda candidate: abs(
                        candidate - float(cell["azimuth_deg"])
                    ),
                ) == centre_deg
            ]
            # One interpolated angle-FFT bin is not an independent target.
            # Requiring adjacency/support is a topology test, not an
            # amplitude threshold.
            beam_hypothesis = hypothesis(range_m, beam_row)
            if beam_hypothesis is None:
                continue
            hypotheses.append(beam_hypothesis)

        visibility = 1.0
        selected = None
        selected_probability = 0.0
        for beam_hypothesis in hypotheses:
            first_probability = (
                visibility * beam_hypothesis["evidence_probability"]
            )
            if first_probability > selected_probability:
                selected_probability = first_probability
                selected = beam_hypothesis
            visibility *= 1.0 - beam_hypothesis["evidence_probability"]
        if selected is None or selected_probability <= 0.0:
            continue
        measurements.append(measurement(
            selected, centre_deg, selected_probability
        ))
    return measurements
