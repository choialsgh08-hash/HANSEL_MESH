"""Pydantic request/response schemas and shared enums.

Validation lives here so API routes stay thin and DB models stay free of
presentation concerns.
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum

from pydantic import BaseModel, ConfigDict, Field


class InjuryStatus(str, Enum):
    """Whether anyone in the party is injured."""

    YES = "yes"
    NO = "no"
    UNKNOWN = "unknown"


class DeliveryStatus(str, Enum):
    """Delivery lifecycle of a stored rescue request."""

    PENDING = "pending"
    SENDING = "sending"
    DELIVERED = "delivered"
    FAILED = "failed"


# Bounds kept generous but finite to reject obviously bad input while not
# blocking a large-group report.
MAX_PEOPLE = 1000
MAX_MESSAGE_LEN = 2000
MAX_CONDITION_LEN = 500
MAX_LOCATION_TEXT_LEN = 500


class RescueRequestCreate(BaseModel):
    """Payload submitted from the victim's browser form."""

    model_config = ConfigDict(str_strip_whitespace=True)

    people_count: int = Field(..., ge=1, le=MAX_PEOPLE, description="Number of people")
    injury_status: InjuryStatus = Field(..., description="yes / no / unknown")
    condition: str = Field(..., min_length=1, max_length=MAX_CONDITION_LEN)
    message: str = Field(..., min_length=1, max_length=MAX_MESSAGE_LEN)
    location_text: str | None = Field(default=None, max_length=MAX_LOCATION_TEXT_LEN)


class RescueRequestAccepted(BaseModel):
    """Response returned immediately after local persistence succeeds."""

    request_id: str
    delivery_status: DeliveryStatus
    message: str


class RescueRequestStatus(BaseModel):
    """Current delivery state of a single stored request."""

    model_config = ConfigDict(from_attributes=True)

    request_id: str
    source_node_id: str
    delivery_status: DeliveryStatus
    retry_count: int
    created_at: datetime
    last_attempt_at: datetime | None = None
    delivered_at: datetime | None = None
    last_error: str | None = None


class HealthResponse(BaseModel):
    """Health probe result for the web server + database."""

    status: str
    role: str
    node_id: str
    database: str


# --------------------------------------------------------------------------- #
# Receiver-side schemas (Phase 2)
# --------------------------------------------------------------------------- #


class ReceivedRescuePayload(BaseModel):
    """Rescue request as delivered by a field node's forwarder.

    Validated on receipt so malformed payloads yield a 422 (a permanent error
    the forwarder will not retry), while well-formed ones are stored verbatim.
    """

    model_config = ConfigDict(str_strip_whitespace=True)

    request_id: str = Field(..., min_length=1, max_length=36)
    source_node_id: str = Field(..., min_length=1, max_length=64)
    people_count: int = Field(..., ge=1, le=MAX_PEOPLE)
    injury_status: InjuryStatus
    condition: str = Field(..., min_length=1, max_length=MAX_CONDITION_LEN)
    message: str = Field(..., min_length=1, max_length=MAX_MESSAGE_LEN)
    latitude: float | None = None
    longitude: float | None = None
    location_accuracy: float | None = None
    location_text: str | None = Field(default=None, max_length=MAX_LOCATION_TEXT_LEN)
    created_at: datetime | None = None


class RescueAck(BaseModel):
    """Acknowledgement returned by the receiver's receive endpoint."""

    request_id: str
    accepted: bool
    duplicate: bool


class DashboardRow(BaseModel):
    """One received request as shown on the rescue-team dashboard."""

    model_config = ConfigDict(from_attributes=True)

    received_at: datetime
    request_id: str
    source_node_id: str
    people_count: int
    injury_status: InjuryStatus
    condition: str
    message: str
    location_text: str | None = None
    original_created_at: datetime | None = None
