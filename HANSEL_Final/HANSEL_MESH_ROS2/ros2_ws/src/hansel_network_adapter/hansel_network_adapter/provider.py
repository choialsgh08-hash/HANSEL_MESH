"""Provider contract for HANSEL network measurements."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .network_adapter_node import NetworkAdapterNode


class NetworkProvider(ABC):
    def __init__(self, adapter: "NetworkAdapterNode") -> None:
        self.adapter = adapter

    @abstractmethod
    def start(self) -> None:
        """Start collection and publish through the adapter."""

    @abstractmethod
    def stop(self) -> None:
        """Stop collection and release resources."""

    def diagnostic_items(self) -> dict[str, str]:
        return {}

    def unavailable_units(self, timeout_s: float) -> list[str]:
        """Return units whose provider data is missing/stale, when supported."""
        del timeout_s
        return []
