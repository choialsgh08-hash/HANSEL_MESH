"""Receiver-side business logic: idempotent storage of rescue requests."""

from __future__ import annotations

import logging

from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import Session

from ..models import ReceivedRescueRequest
from ..repositories import received_repository
from ..schemas import ReceivedRescuePayload

logger = logging.getLogger("rescue_network.receiver")


def store_received(
    session: Session,
    payload: ReceivedRescuePayload,
) -> tuple[ReceivedRescueRequest, bool]:
    """Store a received request idempotently.

    Returns ``(record, duplicate)``. If ``request_id`` was already stored, the
    existing row is returned with ``duplicate=True`` and no new row is created.
    The UNIQUE constraint is the source of truth: even under a race the second
    insert raises ``IntegrityError`` and is treated as a duplicate.
    """
    existing = received_repository.get_by_request_id(session, payload.request_id)
    if existing is not None:
        logger.info(
            "duplicate rescue request ignored request_id=%s source=%s",
            payload.request_id,
            payload.source_node_id,
        )
        return existing, True

    record = ReceivedRescueRequest(
        request_id=payload.request_id,
        source_node_id=payload.source_node_id,
        people_count=payload.people_count,
        injury_status=payload.injury_status.value,
        condition=payload.condition,
        message=payload.message,
        latitude=payload.latitude,
        longitude=payload.longitude,
        location_accuracy=payload.location_accuracy,
        location_text=payload.location_text,
        original_created_at=payload.created_at,
    )
    try:
        received_repository.add(session, record)
    except IntegrityError:
        # Lost a race with a concurrent insert of the same request_id.
        session.rollback()
        existing = received_repository.get_by_request_id(session, payload.request_id)
        if existing is None:  # pragma: no cover - integrity error without a row
            raise
        return existing, True

    logger.info(
        "rescue request received request_id=%s source=%s",
        payload.request_id,
        payload.source_node_id,
    )
    return record, False
