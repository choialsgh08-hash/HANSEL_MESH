"""Provider contract for HANSEL radar data."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .radar_adapter_node import RadarAdapterNode


class RadarProvider(ABC):
    def __init__(self, adapter: "RadarAdapterNode") -> None:
        self.adapter = adapter

    @abstractmethod
    def start(self) -> None:
        """Start the radar data pipeline."""

    @abstractmethod
    def stop(self) -> None:
        """Stop the pipeline and release resources."""

    def diagnostic_items(self) -> dict[str, str]:
        return {}
