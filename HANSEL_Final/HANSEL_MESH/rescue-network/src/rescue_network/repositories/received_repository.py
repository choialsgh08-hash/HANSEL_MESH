"""Persistence for receiver-side (rescue-team) rescue requests."""

from __future__ import annotations

from sqlalchemy import func, select
from sqlalchemy.orm import Session

from ..models import ReceivedRescueRequest


def get_by_request_id(session: Session, request_id: str) -> ReceivedRescueRequest | None:
    """Return the received request with ``request_id`` or ``None``."""
    stmt = select(ReceivedRescueRequest).where(ReceivedRescueRequest.request_id == request_id)
    return session.scalars(stmt).first()


def add(session: Session, record: ReceivedRescueRequest) -> ReceivedRescueRequest:
    """Insert a received request. Caller handles IntegrityError / commit."""
    session.add(record)
    session.flush()
    return record


def list_recent(session: Session, limit: int = 200) -> list[ReceivedRescueRequest]:
    """Return the most recently received requests, newest first."""
    stmt = (
        select(ReceivedRescueRequest)
        .order_by(ReceivedRescueRequest.received_at.desc())
        .limit(limit)
    )
    return list(session.scalars(stmt))


def count(session: Session) -> int:
    """Return the total number of received requests."""
    return session.execute(select(func.count()).select_from(ReceivedRescueRequest)).scalar_one()


def list_after_id(session: Session, after_id: int, limit: int = 200) -> list[ReceivedRescueRequest]:
    """Return requests with ``id`` greater than ``after_id`` (oldest-first).

    Used by the dashboard live stream (Phase 6) to fetch only what is new.
    """
    stmt = (
        select(ReceivedRescueRequest)
        .where(ReceivedRescueRequest.id > after_id)
        .order_by(ReceivedRescueRequest.id.asc())
        .limit(limit)
    )
    return list(session.scalars(stmt))
