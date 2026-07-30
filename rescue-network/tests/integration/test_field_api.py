"""Integration tests for the field node HTTP API (offline, temp SQLite)."""

from __future__ import annotations

import uuid

from fastapi.testclient import TestClient


def _payload(**overrides):
    data = {
        "people_count": 3,
        "injury_status": "unknown",
        "condition": "건물 안 고립",
        "message": "빨리 와주세요",
        "location_text": "3층 계단 근처",
    }
    data.update(overrides)
    return data


def test_get_form_returns_html(client: TestClient):
    res = client.get("/")
    assert res.status_code == 200
    assert "text/html" in res.headers["content-type"]
    assert "구조 요청" in res.text


def test_post_rescue_persists_pending(client: TestClient):
    res = client.post("/api/rescue", json=_payload())
    assert res.status_code == 201
    body = res.json()
    assert body["delivery_status"] == "pending"
    assert body["message"]
    # request_id is a valid UUID
    uuid.UUID(body["request_id"])


def test_post_rescue_assigns_unique_ids(client: TestClient):
    first = client.post("/api/rescue", json=_payload()).json()["request_id"]
    second = client.post("/api/rescue", json=_payload()).json()["request_id"]
    assert first != second


def test_post_rescue_invalid_returns_422(client: TestClient):
    res = client.post("/api/rescue", json=_payload(people_count=0))
    assert res.status_code == 422


def test_post_rescue_bad_injury_enum_returns_422(client: TestClient):
    res = client.post("/api/rescue", json=_payload(injury_status="maybe"))
    assert res.status_code == 422


def test_get_status_reflects_stored_request(client: TestClient):
    request_id = client.post("/api/rescue", json=_payload()).json()["request_id"]
    res = client.get(f"/api/rescue/{request_id}")
    assert res.status_code == 200
    body = res.json()
    assert body["request_id"] == request_id
    assert body["delivery_status"] == "pending"
    assert body["source_node_id"] == "test-node"
    assert body["retry_count"] == 0


def test_get_status_unknown_returns_404(client: TestClient):
    res = client.get(f"/api/rescue/{uuid.uuid4()}")
    assert res.status_code == 404


def test_health_ok(client: TestClient):
    res = client.get("/health")
    assert res.status_code == 200
    body = res.json()
    assert body["status"] == "ok"
    assert body["database"] == "ok"
    assert body["role"] == "field"
    assert body["node_id"] == "test-node"


def test_location_text_optional_accepted(client: TestClient):
    res = client.post("/api/rescue", json=_payload(location_text=None))
    assert res.status_code == 201


def test_metrics_reports_pending(client: TestClient):
    client.post("/api/rescue", json=_payload())
    res = client.get("/metrics")
    assert res.status_code == 200
    assert "text/plain" in res.headers["content-type"]
    assert 'rescue_requests_total_pending{node_id="test-node",role="field"} 1.0' in res.text
    assert "rescue_oldest_pending_age_seconds" in res.text
