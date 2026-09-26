"""One-shot delivery of a single rescue request to the receiver.

This is the testable core of the forwarder: it takes an already-claimed
(``sending``) request plus an HTTP client, performs the POST, and applies the
resulting state transition. It never loops, sleeps, or opens sessions — the
forwarder module owns those concerns.
"""

from __future__ import annotations

import json
import logging
import random
from collections.abc import Callable
from datetime import datetime, timedelta, timezone
from enum import Enum
from typing import Any, Protocol

import httpx
from sqlalchemy.orm import Session

from .. import security
from ..config import Settings
from ..models import RescueRequest, utcnow
from ..repositories import rescue_repository
from ..retry import DeliveryDecision, classify_response, compute_backoff_seconds

logger = logging.getLogger("rescue_network.forwarder")

# last_error column is String(500); keep a margin.
_MAX_ERROR_LEN = 480


class HttpResponse(Protocol):
    """Minimal response shape the delivery logic relies on."""

    status_code: int

    def json(self) -> Any: ...


class HttpClient(Protocol):
    """Minimal client shape (satisfied by httpx.Client and Starlette TestClient).

    We send ``content`` (raw bytes) rather than ``json`` so the exact bytes can
    be HMAC-signed and verified byte-for-byte on the receiver.
    """

    def post(
        self,
        url: str,
        *,
        content: bytes,
        headers: dict[str, str],
        timeout: float,
    ) -> HttpResponse: ...


class DeliveryOutcome(str, Enum):
    """Result of a single delivery attempt."""

    DELIVERED = "delivered"
    RETRIED = "retried"
    FAILED = "failed"


def build_receiver_payload(request: RescueRequest) -> dict[str, Any]:
    """Serialise a stored request into the receiver's expected JSON body."""
    return {
        "request_id": request.request_id,
        "source_node_id": request.source_node_id,
        "people_count": request.people_count,
        "injury_status": request.injury_status,
        "condition": request.condition,
        "message": request.message,
        "latitude": request.latitude,
        "longitude": request.longitude,
        "location_accuracy": request.location_accuracy,
        "location_text": request.location_text,
        "created_at": request.created_at.isoformat() if request.created_at else None,
    }


def _truncate(message: str) -> str:
    return message[:_MAX_ERROR_LEN]


def _ack_accepted(response: HttpResponse) -> bool:
    """Return True only for a well-formed ACK with ``accepted`` truthy."""
    try:
        body = response.json()
    except Exception:  # noqa: BLE001 - any parse failure means "not a valid ACK"
        return False
    return isinstance(body, dict) and bool(body.get("accepted"))


def _schedule_retry(
    session: Session,
    request: RescueRequest,
    now: datetime,
    error: str,
    rng: Callable[[], float],
) -> None:
    retry_count = request.retry_count + 1
    delay = compute_backoff_seconds(retry_count, rng=rng)
    next_retry_at = now + timedelta(seconds=delay)
    rescue_repository.mark_pending_retry(
        session, request, now, retry_count, next_retry_at, _truncate(error)
    )
    logger.warning(
        "delivery retry scheduled request_id=%s node_id=%s retry_count=%d in=%.1fs error=%s",
        request.request_id,
        request.source_node_id,
        retry_count,
        delay,
        error,
    )


def attempt_delivery(
    session: Session,
    request: RescueRequest,
    *,
    client: HttpClient,
    settings: Settings,
    now_fn: Callable[[], datetime] = utcnow,
    rng: Callable[[], float] = random.random,
) -> DeliveryOutcome:
    """Attempt to deliver one ``sending`` request and apply the state transition.

    The shared token is sent in the ``X-Rescue-Token`` header and is never
    logged. When signing is enabled, an ``X-Rescue-Signature`` over the exact
    body bytes is added too. Network errors, timeouts, 5xx and transient 4xx
    reschedule a retry; other 4xx (bad data) fail permanently; a valid ACK marks
    the row delivered.
    """
    now = now_fn()
    payload = build_receiver_payload(request)
    body = json.dumps(payload, separators=(",", ":")).encode()
    secret = settings.rescue_shared_token.get_secret_value()
    headers = {
        "Content-Type": "application/json",
        "X-Rescue-Token": secret,
        "X-Source-Node": settings.node_id,
    }
    if settings.require_signature:
        timestamp = f"{datetime.now(timezone.utc).timestamp():.0f}"
        headers["X-Rescue-Timestamp"] = timestamp
        headers["X-Rescue-Signature"] = security.compute_signature(
            secret, settings.node_id, timestamp, body
        )

    try:
        response = client.post(
            settings.receiver_receive_url,
            content=body,
            headers=headers,
            timeout=settings.delivery_timeout_seconds,
        )
    except httpx.RequestError as exc:
        # Connect errors + timeouts are subclasses of RequestError → retry.
        _schedule_retry(session, request, now, f"network error: {type(exc).__name__}", rng)
        return DeliveryOutcome.RETRIED

    decision = classify_response(response.status_code)

    if decision is DeliveryDecision.DELIVERED:
        if _ack_accepted(response):
            rescue_repository.mark_delivered(session, request, now)
            logger.info(
                "delivered request_id=%s node_id=%s",
                request.request_id,
                request.source_node_id,
            )
            return DeliveryOutcome.DELIVERED
        _schedule_retry(session, request, now, f"invalid ack (status {response.status_code})", rng)
        return DeliveryOutcome.RETRIED

    if decision is DeliveryDecision.RETRY:
        _schedule_retry(session, request, now, f"http {response.status_code}", rng)
        return DeliveryOutcome.RETRIED

    rescue_repository.mark_failed(
        session, request, now, _truncate(f"permanent http {response.status_code}")
    )
    logger.error(
        "delivery permanently failed request_id=%s node_id=%s status=%d",
        request.request_id,
        request.source_node_id,
        response.status_code,
    )
    return DeliveryOutcome.FAILED
