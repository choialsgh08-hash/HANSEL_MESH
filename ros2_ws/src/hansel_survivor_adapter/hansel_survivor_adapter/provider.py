"""Provider contract for the external AP/communication implementation."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .survivor_adapter_node import SurvivorAdapterNode


class SurvivorProvider(ABC):
    def __init__(self, adapter: "SurvivorAdapterNode") -> None:
        self.adapter = adapter

    @abstractmethod
    def start(self) -> None:
        """Start AP/session integration."""

    @abstractmethod
    def stop(self) -> None:
        """Stop integration and release resources."""

