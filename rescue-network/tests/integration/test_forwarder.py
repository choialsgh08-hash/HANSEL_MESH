"""Integration tests for the delivery forwarder.

No real mesh network: the happy path posts into an in-process receiver
(TestClient); failure modes use fake HTTP clients. All offline.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from types import SimpleNamespace
from typing import Any

import httpx

from rescue_network import forwarder
from rescue_network.repositories import rescue_repository

# Naive UTC: SQLite (via SQLAlchemy DateTime) stores/returns naive datetimes, so
# expected values must be naive to compare equal to what is read back.
START = datetime(2026, 7, 30, 0, 0, 0)


class Clock:
    """Manually-advanced clock for deterministic next_retry_at assertions."""

    def __init__(self, start: datetime) -> None:
        self.now = start

    def __call__(self) -> datetime:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now = self.now + timedelta(seconds=seconds)


def no_jitter() -> float:
    return 0.0


class FakeResponse:
    def __init__(self, status_code: int, body: Any = None) -> None:
        self.status_code = status_code
        self._body = body if body is not None else {"accepted": True, "duplicate": False}

    def json(self) -> Any:
        return self._body


class StatusClient:
    """Always returns a fixed status/body."""

    def __init__(self, status_code: int, body: Any = None) -> None:
        self._response = FakeResponse(status_code, body)

    def post(self, *_args: Any, **_kwargs: Any) -> FakeResponse:
        return self._response


class RaisingClient:
    """Raises a network error (timeout / connect) on every post."""

    def __init__(self, exc: Exception) -> None:
        self._exc = exc

    def post(self, *_args: Any, **_kwargs: Any) -> Any:
        raise self._exc


def _read(session_factory, request_id: str) -> SimpleNamespace:
    with session_factory() as session:
        r = rescue_repository.get_by_id(session, request_id)
        return SimpleNamespace(
            status=r.delivery_status,
            retry_count=r.retry_count,
            next_retry_at=r.next_retry_at,
            last_error=r.last_error,
            delivered_at=r.delivered_at,
        )


# --------------------------------------------------------------------------- #
# Happy path + idempotency (real in-process receiver)
# --------------------------------------------------------------------------- #


def test_stored_request_starts_pending(make_pending_request, field_session_factory):
    rid = make_pending_request()
    assert _read(field_session_factory, rid).status == "pending"


def test_delivery_marks_delivered(
    make_pending_request, field_session_factory, receiver_client, settings
):
    rid = make_pending_request()
    counts = forwarder.run_once(
        field_session_factory, receiver_client, settings, now_fn=Clock(START), rng=no_jitter
    )
    assert counts["delivered"] == 1
    row = _read(field_session_factory, rid)
    assert row.status == "delivered"
    assert row.delivered_at is not None
    assert len(receiver_client.get("/api/received").json()) == 1


def test_duplicate_delivery_stores_once(
    make_pending_request, field_session_factory, receiver_client, settings
):
    rid = make_pending_request()
    forwarder.run_once(field_session_factory, receiver_client, settings, rng=no_jitter)

    # Simulate a lost ACK: reset the field row and deliver again.
    with field_session_factory() as session:
        r = rescue_repository.get_by_id(session, rid)
        r.delivery_status = "pending"
        r.next_retry_at = None
        session.commit()

    counts = forwarder.run_once(field_session_factory, receiver_client, settings, rng=no_jitter)
    assert counts["delivered"] == 1
    assert _read(field_session_factory, rid).status == "delivered"
    # Receiver deduped on request_id.
    assert len(receiver_client.get("/api/received").json()) == 1


# --------------------------------------------------------------------------- #
# Failure modes (fake clients)
# --------------------------------------------------------------------------- #


def test_timeout_keeps_pending_and_schedules_retry(
    make_pending_request, field_session_factory, settings
):
    rid = make_pending_request()
    clock = Clock(START)
    counts = forwarder.run_once(
        field_session_factory,
        RaisingClient(httpx.TimeoutException("timed out")),
        settings,
        now_fn=clock,
        rng=no_jitter,
    )
    assert counts["retried"] == 1
    row = _read(field_session_factory, rid)
    assert row.status == "pending"
    assert row.retry_count == 1
    assert row.next_retry_at == START + timedelta(seconds=5)  # initial backoff
    assert row.last_error and "network" in row.last_error


def test_connect_error_keeps_pending(make_pending_request, field_session_factory, settings):
    rid = make_pending_request()
    counts = forwarder.run_once(
        field_session_factory,
        RaisingClient(httpx.ConnectError("refused")),
        settings,
        now_fn=Clock(START),
        rng=no_jitter,
    )
    assert counts["retried"] == 1
    assert _read(field_session_factory, rid).status == "pending"


def test_5xx_retries(make_pending_request, field_session_factory, settings):
    rid = make_pending_request()
    counts = forwarder.run_once(
        field_session_factory, StatusClient(503), settings, now_fn=Clock(START), rng=no_jitter
    )
    assert counts["retried"] == 1
    row = _read(field_session_factory, rid)
    assert row.status == "pending"
    assert row.retry_count == 1
    assert "http 503" in row.last_error


def test_permanent_4xx_fails(make_pending_request, field_session_factory, settings):
    rid = make_pending_request()
    counts = forwarder.run_once(
        field_session_factory, StatusClient(400), settings, now_fn=Clock(START), rng=no_jitter
    )
    assert counts["failed"] == 1
    row = _read(field_session_factory, rid)
    assert row.status == "failed"
    assert "permanent http 400" in row.last_error


def test_invalid_ack_retries(make_pending_request, field_session_factory, settings):
    rid = make_pending_request()
    counts = forwarder.run_once(
        field_session_factory,
        StatusClient(200, {"accepted": False}),
        settings,
        now_fn=Clock(START),
        rng=no_jitter,
    )
    assert counts["retried"] == 1
    assert _read(field_session_factory, rid).status == "pending"


def test_retry_count_increments_across_passes(
    make_pending_request, field_session_factory, settings
):
    rid = make_pending_request()
    clock = Clock(START)
    client = StatusClient(503)

    forwarder.run_once(field_session_factory, client, settings, now_fn=clock, rng=no_jitter)
    assert _read(field_session_factory, rid).retry_count == 1

    # Advance past next_retry_at (5s) so the request is due again.
    clock.advance(10)
    forwarder.run_once(field_session_factory, client, settings, now_fn=clock, rng=no_jitter)
    row = _read(field_session_factory, rid)
    assert row.retry_count == 2
    assert row.next_retry_at == clock.now + timedelta(seconds=10)  # second backoff


def test_not_retried_before_next_retry_at(make_pending_request, field_session_factory, settings):
    rid = make_pending_request()
    clock = Clock(START)
    forwarder.run_once(
        field_session_factory, StatusClient(503), settings, now_fn=clock, rng=no_jitter
    )
    # No clock advance: next_retry_at is in the future, so nothing is claimed.
    counts = forwarder.run_once(
        field_session_factory, StatusClient(503), settings, now_fn=clock, rng=no_jitter
    )
    assert counts == {"recovered": 0, "delivered": 0, "retried": 0, "failed": 0}
    assert _read(field_session_factory, rid).retry_count == 1


# --------------------------------------------------------------------------- #
# Crash recovery + restart resume
# --------------------------------------------------------------------------- #


def test_stale_sending_recovered_and_delivered(
    make_pending_request, field_session_factory, receiver_client, settings
):
    rid = make_pending_request()
    # Simulate a forwarder that died mid-attempt: row stuck in "sending" with an
    # old last_attempt_at.
    stale_time = START - timedelta(seconds=settings.stale_sending_seconds + 60)
    with field_session_factory() as session:
        r = rescue_repository.get_by_id(session, rid)
        r.delivery_status = "sending"
        r.last_attempt_at = stale_time
        session.commit()

    counts = forwarder.run_once(
        field_session_factory, receiver_client, settings, now_fn=Clock(START), rng=no_jitter
    )
    assert counts["recovered"] == 1
    assert counts["delivered"] == 1
    assert _read(field_session_factory, rid).status == "delivered"


def test_restart_resumes_pending_after_failure(
    make_pending_request, field_session_factory, receiver_client, settings
):
    rid = make_pending_request()
    clock = Clock(START)

    # First attempt fails (network down) → pending with a retry scheduled.
    forwarder.run_once(
        field_session_factory,
        RaisingClient(httpx.ConnectError("down")),
        settings,
        now_fn=clock,
        rng=no_jitter,
    )
    assert _read(field_session_factory, rid).status == "pending"

    # "Restart": a new pass with the receiver reachable and the retry time due.
    clock.advance(10)
    counts = forwarder.run_once(
        field_session_factory, receiver_client, settings, now_fn=clock, rng=no_jitter
    )
    assert counts["delivered"] == 1
    assert _read(field_session_factory, rid).status == "delivered"
    assert len(receiver_client.get("/api/received").json()) == 1
