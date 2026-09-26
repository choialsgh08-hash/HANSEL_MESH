"""Integration tests for the receiver node HTTP API."""

from __future__ import annotations

import uuid

from fastapi.testclient import TestClient

from rescue_network.config import Settings
from rescue_network.database import create_db_engine, create_session_factory, init_db
from rescue_network.receiver_app import _fetch_new_received
from rescue_network.schemas import ReceivedRescuePayload
from rescue_network.services import receiver_service
from tests.conftest import TEST_TOKEN

AUTH = {"X-Rescue-Token": TEST_TOKEN}


def _payload(**overrides):
    data = {
        "request_id": str(uuid.uuid4()),
        "source_node_id": "node-01",
        "people_count": 3,
        "injury_status": "yes",
        "condition": "건물 붕괴 위험",
        "message": "즉시 구조 필요",
        "location_text": "3층 계단",
        "created_at": "2026-07-30T00:00:00+00:00",
    }
    data.update(overrides)
    return data


def test_receive_stores_and_acks(receiver_client: TestClient):
    payload = _payload()
    res = receiver_client.post("/api/rescue/receive", json=payload, headers=AUTH)
    assert res.status_code == 200
    body = res.json()
    assert body == {"request_id": payload["request_id"], "accepted": True, "duplicate": False}

    listed = receiver_client.get("/api/received").json()
    assert len(listed) == 1
    assert listed[0]["request_id"] == payload["request_id"]
    assert listed[0]["condition"] == "건물 붕괴 위험"


def test_receive_without_token_401(receiver_client: TestClient):
    res = receiver_client.post("/api/rescue/receive", json=_payload())
    assert res.status_code == 401
    assert receiver_client.get("/api/received").json() == []


def test_receive_wrong_token_401(receiver_client: TestClient):
    res = receiver_client.post(
        "/api/rescue/receive", json=_payload(), headers={"X-Rescue-Token": "nope"}
    )
    assert res.status_code == 401


def test_receive_is_idempotent(receiver_client: TestClient):
    payload = _payload()
    first = receiver_client.post("/api/rescue/receive", json=payload, headers=AUTH).json()
    second = receiver_client.post("/api/rescue/receive", json=payload, headers=AUTH).json()

    assert first["duplicate"] is False
    assert second["accepted"] is True
    assert second["duplicate"] is True
    # Only one row despite two deliveries.
    assert len(receiver_client.get("/api/received").json()) == 1


def test_receive_invalid_body_422(receiver_client: TestClient):
    res = receiver_client.post("/api/rescue/receive", json=_payload(people_count=0), headers=AUTH)
    assert res.status_code == 422


def test_receive_bad_injury_enum_422(receiver_client: TestClient):
    res = receiver_client.post(
        "/api/rescue/receive", json=_payload(injury_status="maybe"), headers=AUTH
    )
    assert res.status_code == 422


def test_dashboard_html(receiver_client: TestClient):
    res = receiver_client.get("/dashboard")
    assert res.status_code == 200
    assert "text/html" in res.headers["content-type"]
    assert "구조대 대시보드" in res.text


def test_health_ok(receiver_client: TestClient):
    body = receiver_client.get("/health").json()
    assert body["status"] == "ok"
    assert body["role"] == "receiver"
    assert body["database"] == "ok"


def test_metrics_counts_received(receiver_client: TestClient):
    receiver_client.post("/api/rescue/receive", json=_payload(), headers=AUTH)
    res = receiver_client.get("/metrics")
    assert res.status_code == 200
    assert 'rescue_received_total{node_id="receiver-node",role="receiver"} 1.0' in res.text


def test_incremental_fetch_for_stream(receiver_settings: Settings):
    """The SSE stream's fetch helper returns only rows newer than last_id."""
    engine = create_db_engine(receiver_settings.receiver_database_url)
    init_db(engine)
    factory = create_session_factory(engine)

    ids: list[int] = []
    with factory() as session:
        for _ in range(3):
            record, _dup = receiver_service.store_received(
                session,
                ReceivedRescuePayload(
                    request_id=str(uuid.uuid4()),
                    source_node_id="n",
                    people_count=1,
                    injury_status="no",
                    condition="c",
                    message="m",
                ),
            )
            session.flush()
            ids.append(record.id)
        session.commit()

    # From the start: all three, and last_id advances to the newest.
    items, last = _fetch_new_received(factory, 0)
    assert len(items) == 3
    assert last == ids[-1]
    # From the first id: only the two after it.
    items2, _ = _fetch_new_received(factory, ids[0])
    assert len(items2) == 2
    # From the newest: nothing new, last_id unchanged.
    items3, last3 = _fetch_new_received(factory, ids[-1])
    assert items3 == []
    assert last3 == ids[-1]
