from __future__ import annotations

from collections import deque
import math
from numbers import Real
from typing import Deque, Dict, Iterable, Tuple


GridKey = Tuple[int, int]
FrameCells = Dict[GridKey, float]


class RadarReflectivitySurface:
    def __init__(
        self,
        *,
        radius_m: float = 0.50,
        resolution_m: float = 0.025,
        history_frames: int = 5,
        required_hits: int = 3,
    ) -> None:
        self.radius_m = float(radius_m)
        self.resolution_m = float(resolution_m)
        self.history_frames = int(history_frames)
        self.required_hits = int(required_hits)
        if (
            not math.isfinite(self.radius_m)
            or self.radius_m <= 0.0
            or not math.isfinite(self.resolution_m)
            or self.resolution_m <= 0.0
            or self.history_frames < 1
            or not 1 <= self.required_hits <= self.history_frames
        ):
            raise ValueError("invalid reflectivity surface geometry")
        self._history: Deque[Tuple[float, FrameCells]] = deque(
            maxlen=self.history_frames
        )

    @staticmethod
    def _finite_real(value: object) -> float | None:
        if isinstance(value, bool) or not isinstance(value, Real):
            return None
        result = float(value)
        return result if math.isfinite(result) else None

    @staticmethod
    def _time(value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError("time must be a real number")
        result = float(value)
        if not math.isfinite(result):
            raise ValueError("time must be finite")
        return result

    def reset(self) -> None:
        self._history.clear()

    def ingest(
        self,
        observations: Iterable[object],
        received_at: float,
        *,
        elapsed_frame_windows: int = 1,
    ) -> None:
        timestamp = self._time(received_at)
        if isinstance(elapsed_frame_windows, bool) or not isinstance(
            elapsed_frame_windows, int
        ):
            raise TypeError("elapsed_frame_windows must be an integer")
        if elapsed_frame_windows < 1:
            raise ValueError("elapsed_frame_windows must be positive")
        if self._history and timestamp < self._history[-1][0]:
            raise ValueError("received_at must not move backwards")

        for _ in range(min(
            elapsed_frame_windows - 1,
            self.history_frames,
        )):
            self._history.append((timestamp, {}))

        cells: FrameCells = {}
        try:
            iterator = iter(observations)
        except TypeError:
            iterator = iter(())
        for observation in iterator:
            if not isinstance(observation, (list, tuple)) or len(observation) != 3:
                continue
            forward = self._finite_real(observation[0])
            lateral = self._finite_real(observation[1])
            intensity = self._finite_real(observation[2])
            if forward is None or lateral is None or intensity is None:
                continue
            if forward < 0.0 or math.hypot(forward, lateral) > self.radius_m:
                continue
            key = (
                round(forward / self.resolution_m),
                round(lateral / self.resolution_m),
            )
            quantized_forward = key[0] * self.resolution_m
            quantized_lateral = key[1] * self.resolution_m
            if (
                quantized_forward < 0.0
                or math.hypot(quantized_forward, quantized_lateral)
                > self.radius_m + 1e-12
            ):
                continue
            cells[key] = max(cells.get(key, 0.0), min(1.0, max(0.0, intensity)))
        self._history.append((timestamp, cells))

    def payload(self, now: float) -> dict[str, object]:
        timestamp = self._time(now)
        combined: Dict[GridKey, list[Tuple[float, float]]] = {}
        for observed_at, cells in self._history:
            for key, intensity in cells.items():
                combined.setdefault(key, []).append((observed_at, intensity))

        cells_payload = []
        for key in sorted(combined):
            hits = combined[key]
            if len(hits) < self.required_hits:
                continue
            last_seen = max(observed_at for observed_at, _ in hits)
            if timestamp < last_seen:
                raise ValueError("now must not precede observed cells")
            intensity = sum(value for _, value in hits) / len(hits)
            cells_payload.append(
                {
                    "forward_m": round(key[0] * self.resolution_m, 4),
                    "lateral_m": round(key[1] * self.resolution_m, 4),
                    "intensity": round(intensity, 4),
                    "persistence": round(
                        len(hits) / self.history_frames,
                        3,
                    ),
                    "hits": len(hits),
                    "age_ms": max(0, round((timestamp - last_seen) * 1000.0)),
                }
            )
        return {
            "radius_m": self.radius_m,
            "resolution_m": self.resolution_m,
            "history_frames": self.history_frames,
            "required_hits": self.required_hits,
            "retention": "three_of_five_complete_frames",
            "pose_mode": "sensor_relative_short_memory",
            "source": "calibrated_heatmap_residual",
            "cells": cells_payload,
        }
