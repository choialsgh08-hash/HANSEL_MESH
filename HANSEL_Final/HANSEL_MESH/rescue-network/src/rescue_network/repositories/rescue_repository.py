"""Persistence for field-node rescue requests.

Keeps ORM/session usage isolated from API routes and business logic. Each
function takes an explicit ``Session`` so callers control the transaction
boundary (request-scoped in the web app, ``session_scope`` in the forwarder).
"""

from __future__ import annotations

from datetime import datetime

from sqlalchemy import func, or_, select
from sqlalchemy.orm import Session

from ..models import RescueRequest
from ..schemas import DeliveryStatus


def add(session: Session, request: RescueRequest) -> RescueRequest:
    """Insert a new rescue request. Caller commits."""
    session.add(request)
    session.flush()  # surface integrity errors before the caller commits
    return request


def get_by_id(session: Session, request_id: str) -> RescueRequest | None:
    """Return the request with ``request_id`` or ``None``."""
    return session.get(RescueRequest, request_id)


def list_recent(session: Session, limit: int = 100) -> list[RescueRequest]:
    """Return the most recently created requests, newest first."""
    stmt = select(RescueRequest).order_by(RescueRequest.created_at.desc()).limit(limit)
    return list(session.scalars(stmt))


# --------------------------------------------------------------------------- #
# Forwarder queries / state transitions (Phase 2). Callers own the commit.
# --------------------------------------------------------------------------- #


def claim_due(session: Session, now: datetime, limit: int) -> list[RescueRequest]:
    """Select pending requests whose retry time has arrived and mark them sending.

    Returns the claimed requests (now in ``sending`` state with ``last_attempt_at``
    stamped). Committing the ``sending`` transition before the network call means
    a crash mid-attempt leaves a recoverable ``sending`` row (see
    :func:`recover_stale_sending`). Oldest requests are served first.
    """
    stmt = (
        select(RescueRequest)
        .where(
            RescueRequest.delivery_status == DeliveryStatus.PENDING.value,
            or_(
                RescueRequest.next_retry_at.is_(None),
                RescueRequest.next_retry_at <= now,
            ),
        )
        .order_by(RescueRequest.created_at.asc())
        .limit(limit)
    )
    claimed = list(session.scalars(stmt))
    for request in claimed:
        request.delivery_status = DeliveryStatus.SENDING.value
        request.last_attempt_at = now
    return claimed


def mark_delivered(session: Session, request: RescueRequest, now: datetime) -> None:
    """Mark a request as successfully delivered."""
    request.delivery_status = DeliveryStatus.DELIVERED.value
    request.delivered_at = now
    request.last_attempt_at = now
    request.last_error = None


def mark_pending_retry(
    session: Session,
    request: RescueRequest,
    now: datetime,
    retry_count: int,
    next_retry_at: datetime,
    error: str | None,
) -> None:
    """Return a request to ``pending`` and schedule the next retry."""
    request.delivery_status = DeliveryStatus.PENDING.value
    request.retry_count = retry_count
    request.last_attempt_at = now
    request.next_retry_at = next_retry_at
    request.last_error = error


def mark_failed(
    session: Session,
    request: RescueRequest,
    now: datetime,
    error: str | None,
) -> None:
    """Mark a request permanently failed (no further retries)."""
    request.delivery_status = DeliveryStatus.FAILED.value
    request.last_attempt_at = now
    request.last_error = error


def status_counts(session: Session) -> dict[str, int]:
    """Return a ``{delivery_status: count}`` histogram (all statuses present)."""
    counts = {status.value: 0 for status in DeliveryStatus}
    stmt = select(RescueRequest.delivery_status, func.count()).group_by(
        RescueRequest.delivery_status
    )
    for status, count in session.execute(stmt):
        counts[status] = count
    return counts


def oldest_pending_created_at(session: Session) -> datetime | None:
    """Return the creation time of the oldest still-pending request, or None."""
    stmt = select(func.min(RescueRequest.created_at)).where(
        RescueRequest.delivery_status == DeliveryStatus.PENDING.value
    )
    return session.execute(stmt).scalar_one_or_none()


def recover_stale_sending(session: Session, cutoff: datetime) -> int:
    """Reset ``sending`` requests older than ``cutoff`` back to ``pending``.

    Returns the number recovered. Their ``next_retry_at`` is left untouched so
    they are picked up on the next pass.
    """
    stmt = select(RescueRequest).where(
        RescueRequest.delivery_status == DeliveryStatus.SENDING.value,
        RescueRequest.last_attempt_at < cutoff,
    )
    stale = list(session.scalars(stmt))
    for request in stale:
        request.delivery_status = DeliveryStatus.PENDING.value
    return len(stale)
