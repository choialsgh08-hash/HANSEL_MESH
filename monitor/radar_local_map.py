from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class _Cell:
    log_odds: float
    confidence: float
    source: str
    range_uncertainty_m: float
    lateral_uncertainty_m: float
    hits: int
    _last_hit_ns: int
    _last_decay_ns: int
    _occupied_episode: bool


class ProbabilisticRollingGrid:
    def __init__(
        self, radius_m, resolution_m, occupied_threshold, decay_half_life_s,
        ttl_s, hit_probability, maximum_probability,
    ):
        self.radius_m = radius_m
        self.resolution_m = resolution_m
        self.occupied_threshold = occupied_threshold
        self.decay_half_life_s = decay_half_life_s
        self.ttl_s = ttl_s
        self.hit_probability = hit_probability
        self.maximum_probability = maximum_probability
        self._hit_log_odds = math.log(hit_probability / (1.0 - hit_probability))
        self._max_log_odds = math.log(maximum_probability / (1.0 - maximum_probability))
        self._cells: Dict[Tuple[int, int], _Cell] = {}
        self._decayed_cell_count = 0
        self._ttl_expired_cell_count = 0

    def add_hit(self, forward_m, lateral_m, confidence, source, range_uncertainty_m, lateral_uncertainty_m, now) -> None:
        now_ns = self._to_ns(now)
        self._advance(now_ns)
        key = self._key(forward_m, lateral_m)
        if not self._inside(key):
            return
        cell = self._cells.get(key)
        if cell is None:
            log_odds = self._hit_log_odds
            self._cells[key] = _Cell(log_odds, confidence, source, range_uncertainty_m,
                                     lateral_uncertainty_m, 1, now_ns, now_ns,
                                     self._probability(log_odds) >= self.occupied_threshold)
            return
        cell.log_odds = min(self._max_log_odds, cell.log_odds + self._hit_log_odds)
        cell.confidence = max(cell.confidence, confidence)
        cell.range_uncertainty_m = max(cell.range_uncertainty_m, range_uncertainty_m)
        cell.lateral_uncertainty_m = max(cell.lateral_uncertainty_m, lateral_uncertainty_m)
        cell.source = self._merged_source(cell.source, source)
        cell.hits = min(255, cell.hits + 1)
        cell._last_hit_ns = now_ns
        cell._last_decay_ns = now_ns
        if self._probability(cell.log_odds) >= self.occupied_threshold:
            cell._occupied_episode = True

    def transform(self, delta_forward_m, delta_lateral_m, yaw_rad, now) -> None:
        now_ns = self._to_ns(now)
        self._advance(now_ns)
        transformed: Dict[Tuple[int, int], _Cell] = {}
        cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
        for (forward_index, lateral_index), cell in self._cells.items():
            old_forward, old_lateral = self._centre(forward_index, lateral_index)
            new_forward = cos_yaw * old_forward - sin_yaw * old_lateral + delta_forward_m
            new_lateral = sin_yaw * old_forward + cos_yaw * old_lateral + delta_lateral_m
            key = self._key(new_forward, new_lateral)
            if not self._inside(key):
                continue
            prior = transformed.get(key)
            if prior is None:
                transformed[key] = cell
            else:
                transformed[key] = self._merge_collision(prior, cell)
        self._cells = transformed

    def clear(self) -> None:
        self._cells.clear()
        self._decayed_cell_count = 0
        self._ttl_expired_cell_count = 0

    def payload(self, now) -> dict:
        self._advance(self._to_ns(now))
        return {
            "radius_m": self.radius_m, "resolution_m": self.resolution_m,
            "occupied_threshold": self.occupied_threshold,
            "decay_half_life_ms": int(self.decay_half_life_s * 1000),
            "ttl_ms": int(self.ttl_s * 1000),
            "retention": "probability_decay_with_ttl",
            "pose_mode": "robot_relative_short_memory",
            "cells": self._cell_payloads(self._to_ns(now)),
        }

    def occupied_cells(self, now) -> list[dict]:
        now_ns = self._to_ns(now)
        self._advance(now_ns)
        return [cell for cell in self._cell_payloads(now_ns) if cell["occupied"]]

    def diagnostics(self) -> dict:
        occupied = sum(self._probability(cell.log_odds) >= self.occupied_threshold for cell in self._cells.values())
        return {"active_cell_count": len(self._cells), "occupied_cell_count": occupied,
                "decayed_cell_count": self._decayed_cell_count,
                "ttl_expired_cell_count": self._ttl_expired_cell_count}

    def _advance(self, now_ns: int) -> None:
        for key, cell in list(self._cells.items()):
            if now_ns - cell._last_hit_ns >= int(self.ttl_s * 1_000_000_000):
                del self._cells[key]
                self._ttl_expired_cell_count += 1
                continue
            elapsed_s = max(0, now_ns - cell._last_decay_ns) / 1_000_000_000
            if elapsed_s:
                was_occupied = self._probability(cell.log_odds) >= self.occupied_threshold
                cell.log_odds *= 2.0 ** (-elapsed_s / self.decay_half_life_s)
                cell._last_decay_ns = now_ns
                if was_occupied and self._probability(cell.log_odds) < self.occupied_threshold and cell._occupied_episode:
                    self._decayed_cell_count += 1
                    cell._occupied_episode = False

    def _cell_payloads(self, now_ns: int) -> list[dict]:
        result = []
        for (forward_index, lateral_index), cell in sorted(self._cells.items()):
            forward_m, lateral_m = self._centre(forward_index, lateral_index)
            probability = self._probability(cell.log_odds)
            result.append({
                "forward_m": forward_m, "lateral_m": lateral_m, "probability": probability,
                "confidence": cell.confidence, "occupied": probability >= self.occupied_threshold,
                "hits": cell.hits, "age_ms": int((now_ns - cell._last_hit_ns) / 1_000_000),
                "range_uncertainty_m": cell.range_uncertainty_m,
                "lateral_uncertainty_m": cell.lateral_uncertainty_m, "source": cell.source,
            })
        return result

    def _merge_collision(self, first: _Cell, second: _Cell) -> _Cell:
        winner, other = (first, second) if first.log_odds >= second.log_odds else (second, first)
        return _Cell(winner.log_odds, max(first.confidence, second.confidence),
                     self._merged_source(first.source, second.source),
                     max(first.range_uncertainty_m, second.range_uncertainty_m),
                     max(first.lateral_uncertainty_m, second.lateral_uncertainty_m),
                     max(first.hits, second.hits), max(first._last_hit_ns, second._last_hit_ns),
                     max(first._last_decay_ns, second._last_decay_ns),
                     first._occupied_episode or second._occupied_episode)

    def _key(self, forward_m, lateral_m):
        return (self._quantized_index(forward_m), self._quantized_index(lateral_m))

    def _quantized_index(self, value_m):
        magnitude = math.floor(abs(value_m) / self.resolution_m + 0.5)
        return int(math.copysign(magnitude, value_m))

    def _centre(self, forward_index, lateral_index):
        return (round(forward_index * self.resolution_m, 10), round(lateral_index * self.resolution_m, 10))

    def _inside(self, key):
        forward_m, lateral_m = self._centre(*key)
        return forward_m * forward_m + lateral_m * lateral_m <= self.radius_m * self.radius_m

    @staticmethod
    def _probability(log_odds):
        return 1.0 / (1.0 + math.exp(-log_odds))

    @staticmethod
    def _merged_source(first, second):
        return first if first == second else "mixed"

    @staticmethod
    def _to_ns(now):
        return int(now * 1_000_000_000)
