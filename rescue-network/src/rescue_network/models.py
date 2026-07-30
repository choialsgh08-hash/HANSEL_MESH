"""SQLAlchemy ORM models.

Phase 1 defines the field node's ``rescue_requests`` table. The receiver's
``received_rescue_requests`` table is added in Phase 2.
"""

from __future__ import annotations

from datetime import datetime, timezone

from sqlalchemy import DateTime, Integer, String, Text
from sqlalchemy.orm import Mapped, mapped_column

from .database import Base

__all__ = ["RescueRequest", "ReceivedRescueRequest", "utcnow"]


def utcnow() -> datetime:
    """Return the current time as a timezone-aware UTC datetime."""
    return datetime.now(timezone.utc)


class RescueRequest(Base):
    """A rescue request captured on a field node.

    ``delivery_status`` starts at ``pending``; the forwarder (Phase 2) advances
    it through ``sending`` → ``delivered`` (or ``failed`` for permanent errors).
    """

    __tablename__ = "rescue_requests"

    request_id: Mapped[str] = mapped_column(String(36), primary_key=True)
    source_node_id: Mapped[str] = mapped_column(String(64), nullable=False)

    people_count: Mapped[int] = mapped_column(Integer, nullable=False)
    injury_status: Mapped[str] = mapped_column(String(16), nullable=False)
    condition: Mapped[str] = mapped_column(Text, nullable=False)
    message: Mapped[str] = mapped_column(Text, nullable=False)

    # Location: manual free-text description (Phase 1). Coordinate fields are
    # kept nullable so a later GPS-capable client can populate them without a
    # schema migration.
    latitude: Mapped[float | None] = mapped_column(nullable=True)
    longitude: Mapped[float | None] = mapped_column(nullable=True)
    location_accuracy: Mapped[float | None] = mapped_column(nullable=True)
    location_text: Mapped[str | None] = mapped_column(Text, nullable=True)

    # Delivery bookkeeping (driven by the forwarder in Phase 2).
    delivery_status: Mapped[str] = mapped_column(String(16), nullable=False, default="pending")
    retry_count: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    next_retry_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    last_attempt_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)
    last_error: Mapped[str | None] = mapped_column(String(500), nullable=True)

    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, default=utcnow
    )
    delivered_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True), nullable=True)


class ReceivedRescueRequest(Base):
    """A rescue request as stored on the receiver (rescue-team) node.

    ``request_id`` carries a UNIQUE constraint so the receive endpoint stays
    idempotent: a re-delivered request never creates a second row.
    """

    __tablename__ = "received_rescue_requests"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    request_id: Mapped[str] = mapped_column(String(36), unique=True, nullable=False, index=True)
    source_node_id: Mapped[str] = mapped_column(String(64), nullable=False)

    people_count: Mapped[int] = mapped_column(Integer, nullable=False)
    injury_status: Mapped[str] = mapped_column(String(16), nullable=False)
    condition: Mapped[str] = mapped_column(Text, nullable=False)
    message: Mapped[str] = mapped_column(Text, nullable=False)

    latitude: Mapped[float | None] = mapped_column(nullable=True)
    longitude: Mapped[float | None] = mapped_column(nullable=True)
    location_accuracy: Mapped[float | None] = mapped_column(nullable=True)
    location_text: Mapped[str | None] = mapped_column(Text, nullable=True)

    # When the request was first created on the field node (carried in payload).
    original_created_at: Mapped[datetime | None] = mapped_column(
        DateTime(timezone=True), nullable=True
    )
    # When the receiver accepted it.
    received_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, default=utcnow
    )
