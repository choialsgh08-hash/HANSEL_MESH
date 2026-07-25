"""Versioned UDP control protocol shared by the operator and robot nodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
import time
from typing import Any, Dict, Optional, Tuple


PROTOCOL_VERSION = 1
DEFAULT_TTL_MS = 750
MAX_TTL_MS = 5000

_SESSION_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{1,96}$")
RESERVED_COMMAND_FIELDS = {
    "protocol_version",
    "type",
    "session_id",
    "seq",
    "target",
    "command",
    "source",
    "sent_at",
    "sent_monotonic_ns",
    "ttl_ms",
}


def normalize_command(command: object) -> str:
    return str(command).strip().lower().replace("-", "_").replace(" ", "_")


def build_command(
    session_id: str,
    seq: int,
    target: str,
    command: str,
    source: str = "operator",
    ttl_ms: int = DEFAULT_TTL_MS,
    sent_at: Optional[float] = None,
    sent_monotonic_ns: Optional[int] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    if extra:
        reserved = RESERVED_COMMAND_FIELDS.intersection(extra)
        if reserved:
            names = ", ".join(sorted(reserved))
            raise ValueError(f"extra contains reserved protocol fields: {names}")
    message: Dict[str, Any] = {
        "protocol_version": PROTOCOL_VERSION,
        "type": "command",
        "session_id": session_id,
        "seq": seq,
        "target": target,
        "command": command,
        "source": source,
        "sent_at": time.time() if sent_at is None else sent_at,
        "sent_monotonic_ns": (
            time.monotonic_ns() if sent_monotonic_ns is None else sent_monotonic_ns
        ),
        "ttl_ms": ttl_ms,
    }
    if extra:
        message.update(extra)
    return message


def build_ack(
    request: Optional[Dict[str, Any]],
    target: str,
    status: str,
    reason: str,
    server_time: Optional[float] = None,
    server_monotonic_ns: Optional[int] = None,
) -> Dict[str, Any]:
    request = request or {}
    return {
        "protocol_version": PROTOCOL_VERSION,
        "type": "ack",
        "session_id": request.get("session_id"),
        "seq": request.get("seq"),
        "target": target,
        "command": request.get("command"),
        "status": status,
        "reason": reason,
        "server_time": time.time() if server_time is None else server_time,
        "server_monotonic_ns": (
            time.monotonic_ns()
            if server_monotonic_ns is None
            else server_monotonic_ns
        ),
    }


@dataclass(frozen=True)
class ValidationResult:
    accepted: bool
    reason: str
    command: str = ""


@dataclass
class _SessionState:
    last_seq: int
    best_offset_ns: int
    last_seen_monotonic_ns: int


class ProtocolValidator:
    """Validate freshness and ordering for one robot control endpoint."""

    def __init__(
        self,
        max_ttl_ms: int = MAX_TTL_MS,
        max_sessions: int = 256,
    ) -> None:
        self.max_ttl_ms = max_ttl_ms
        self.max_sessions = max_sessions
        self._sessions: Dict[Tuple[str, str], _SessionState] = {}

    def validate(
        self,
        message: object,
        expected_target: str,
        source_id: str,
        now_monotonic_ns: Optional[int] = None,
    ) -> ValidationResult:
        recv_monotonic_ns = (
            time.monotonic_ns()
            if now_monotonic_ns is None
            else now_monotonic_ns
        )
        if not isinstance(message, dict):
            return ValidationResult(False, "message_must_be_object")

        command = normalize_command(message.get("command", ""))
        is_stop = command == "stop"

        if message.get("protocol_version") != PROTOCOL_VERSION:
            return ValidationResult(False, "unsupported_protocol_version", command)
        if message.get("type") != "command":
            return ValidationResult(False, "invalid_message_type", command)
        if message.get("target") != expected_target:
            return ValidationResult(False, "target_mismatch", command)
        if not command or len(command) > 64:
            return ValidationResult(False, "invalid_command", command)

        session_id = message.get("session_id")
        if not isinstance(session_id, str) or not _SESSION_PATTERN.fullmatch(session_id):
            return ValidationResult(False, "invalid_session_id", command)

        seq = message.get("seq")
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 1:
            return ValidationResult(False, "invalid_seq", command)

        sent_monotonic_ns = message.get("sent_monotonic_ns")
        if (
            isinstance(sent_monotonic_ns, bool)
            or not isinstance(sent_monotonic_ns, int)
            or sent_monotonic_ns < 0
        ):
            return ValidationResult(False, "invalid_sent_monotonic_ns", command)

        ttl_ms = message.get("ttl_ms")
        if isinstance(ttl_ms, bool) or not isinstance(ttl_ms, int):
            return ValidationResult(False, "invalid_ttl", command)
        if ttl_ms < 1 or ttl_ms > self.max_ttl_ms:
            return ValidationResult(False, "invalid_ttl", command)

        if "speed" in message:
            speed = message.get("speed")
            if (
                isinstance(speed, bool)
                or not isinstance(speed, (int, float))
                or not math.isfinite(float(speed))
                or not 0.0 <= float(speed) <= 1.0
            ):
                return ValidationResult(False, "invalid_speed", command)

        key = (source_id, session_id)
        previous = self._sessions.get(key)
        previous_seq = previous.last_seq if previous is not None else 0
        duplicate = seq <= previous_seq

        # The receiver cannot judge the age of the very first packet because
        # sender and receiver monotonic clocks have unrelated origins. Require
        # a harmless stop packet to establish the session clock baseline before
        # any command that could move hardware is accepted.
        if previous is None and not is_stop:
            return ValidationResult(
                False,
                "session_requires_initial_stop",
                command,
            )

        # Sender and receiver monotonic clocks have unrelated origins. Their
        # difference is nevertheless stable during a session, so the smallest
        # observed offset is the path baseline and only excess delay consumes TTL.
        offset_ns = recv_monotonic_ns - sent_monotonic_ns
        best_offset_ns = (
            offset_ns
            if previous is None
            else min(previous.best_offset_ns, offset_ns)
        )
        excess_delay_ns = max(0, offset_ns - best_offset_ns)
        expired = excess_delay_ns > ttl_ms * 1_000_000

        # A stop packet can only reduce risk. Apply it even when a delayed or
        # duplicated datagram arrives, while retaining all identity/target checks.
        if is_stop and (duplicate or expired):
            if seq > previous_seq:
                self._remember(
                    key,
                    seq,
                    best_offset_ns,
                    recv_monotonic_ns,
                )
            if duplicate and expired:
                return ValidationResult(True, "safety_stop_duplicate_and_expired", command)
            if duplicate:
                return ValidationResult(True, "safety_stop_duplicate", command)
            return ValidationResult(True, "safety_stop_expired", command)

        if duplicate:
            return ValidationResult(False, "seq_not_strictly_increasing", command)
        if expired:
            return ValidationResult(False, "command_expired", command)

        self._remember(
            key,
            seq,
            best_offset_ns,
            recv_monotonic_ns,
        )
        return ValidationResult(True, "ok", command)

    def _remember(
        self,
        key: Tuple[str, str],
        seq: int,
        best_offset_ns: int,
        now_monotonic_ns: int,
    ) -> None:
        self._sessions[key] = _SessionState(
            last_seq=seq,
            best_offset_ns=best_offset_ns,
            last_seen_monotonic_ns=now_monotonic_ns,
        )
        if len(self._sessions) <= self.max_sessions:
            return
        oldest_key = min(
            self._sessions,
            key=lambda item: self._sessions[item].last_seen_monotonic_ns,
        )
        self._sessions.pop(oldest_key, None)
