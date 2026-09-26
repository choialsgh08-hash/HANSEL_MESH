"""Semantic input boundary; microphone recognition intentionally remains external."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class DriveIntent:
    forward_rpm: float
    turn_rpm: float
    source: str


class OperatorInputAdapter(ABC):
    @abstractmethod
    def start(self) -> None:
        """Start collecting semantic operator intents."""

    @abstractmethod
    def stop(self) -> None:
        """Stop collecting input and release resources."""


class MicrophoneInputAdapter(OperatorInputAdapter):
    """Contract for a future speech/voice-command provider.

    Audio transport, recognition, command grammar, and confirmation policy are
    deliberately undefined until the survivor/operator communication requirements
    are agreed.
    """

    def start(self) -> None:
        raise NotImplementedError("microphone provider is not configured")

    def stop(self) -> None:
        return

