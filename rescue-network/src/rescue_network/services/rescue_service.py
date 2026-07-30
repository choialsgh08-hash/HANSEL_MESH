"""Field-node rescue-request business logic.

Turns validated input into a persisted ``RescueRequest`` and reads status back.
The API layer calls only these functions, never the repository or ORM directly.
"""

from __future__ import annotations

import logging
import uuid

from sqlalchemy.orm import Session

from ..models import RescueRequest
from ..repositories import rescue_repository
from ..schemas import DeliveryStatus, RescueRequestCreate

logger = logging.getLogger("rescue_network.field")


def create_rescue_request(
    session: Session,
    payload: RescueRequestCreate,
    source_node_id: str,
) -> RescueRequest:
    """Persist a new rescue request in ``pending`` state.

    A fresh UUID4 is generated for ``request_id``. The request is stored locally
    first; network delivery is the forwarder's job (Phase 2). Caller owns the
    transaction commit.
    """
    request = RescueRequest(
        request_id=str(uuid.uuid4()),
        source_node_id=source_node_id,
        people_count=payload.people_count,
        injury_status=payload.injury_status.value,
        condition=payload.condition,
        message=payload.message,
        location_text=payload.location_text,
        delivery_status=DeliveryStatus.PENDING.value,
        retry_count=0,
    )
    rescue_repository.add(session, request)
    logger.info(
        "rescue request stored request_id=%s node_id=%s status=%s",
        request.request_id,
        source_node_id,
        request.delivery_status,
    )
    return request


def get_rescue_request(session: Session, request_id: str) -> RescueRequest | None:
    """Return a stored request by id, or ``None`` if unknown."""
    return rescue_repository.get_by_id(session, request_id)
