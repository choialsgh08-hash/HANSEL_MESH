"""Integration tests for HMAC-signed delivery (require_signature=True)."""

from __future__ import annotations

import json
import uuid
from collections.abc import Iterator
from datetime import datetime, timezone
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from rescue_network import forwarder, security
from rescue_network.config import NodeRole, Settings
from rescue_network.database import create_db_engine, create_session_factory, init_db
from rescue_network.receiver_app import create_app as create_receiver_app
from rescue_network.repositories import rescue_repository
from rescue_network.schemas import RescueRequestCreate
from rescue_network.services import rescue_service

TOKEN = "signing-secret"


@pytest.fixture
def signed_receiver_settings(tmp_path: Path) -> Settings:
    return Settings(
        node_role=NodeRole.RECEIVER,
        node_id="receiver-node",
        data_dir=tmp_path,
        rescue_shared_token=TOKEN,
        require_signature=True,
        _env_file=None,
    )


@pytest.fixture
def signed_field_settings(tmp_path: Path) -> Settings:
    return Settings(
        node_role=NodeRole.FIELD,
        node_id="field-01",
        data_dir=tmp_path,
        rescue_shared_token=TOKEN,
        receiver_url="http://testserver",
        require_signature=True,
        _env_file=None,
    )


@pytest.fixture
def signed_receiver_client(signed_receiver_settings: Settings) -> Iterator[TestClient]:
    with TestClient(create_receiver_app(signed_receiver_settings)) as c:
        yield c


def _body() -> bytes:
    payload = {
        "request_id": str(uuid.uuid4()),
        "source_node_id": "field-01",
        "people_count": 2,
        "injury_status": "yes",
        "condition": "고립",
        "message": "도와주세요",
    }
    return json.dumps(payload, separators=(",", ":")).encode()


def test_signature_required_token_alone_rejected(signed_receiver_client: TestClient):
    body = _body()
    # Old-style token header is not enough when signatures are required.
    res = signed_receiver_client.post(
        "/api/rescue/receive",
        content=body,
        headers={"Content-Type": "application/json", "X-Rescue-Token": TOKEN},
    )
    assert res.status_code == 401


def test_valid_signature_accepted(signed_receiver_client: TestClient):
    body = _body()
    ts = f"{datetime.now(timezone.utc).timestamp():.0f}"
    sig = security.compute_signature(TOKEN, "field-01", ts, body)
    res = signed_receiver_client.post(
        "/api/rescue/receive",
        content=body,
        headers={
            "Content-Type": "application/json",
            "X-Source-Node": "field-01",
            "X-Rescue-Timestamp": ts,
            "X-Rescue-Signature": sig,
        },
    )
    assert res.status_code == 200
    assert res.json()["accepted"] is True


def test_tampered_body_rejected(signed_receiver_client: TestClient):
    body = _body()
    ts = f"{datetime.now(timezone.utc).timestamp():.0f}"
    sig = security.compute_signature(TOKEN, "field-01", ts, body)
    res = signed_receiver_client.post(
        "/api/rescue/receive",
        content=b'{"request_id":"x","source_node_id":"field-01","people_count":9,'
        b'"injury_status":"no","condition":"c","message":"m"}',
        headers={
            "Content-Type": "application/json",
            "X-Source-Node": "field-01",
            "X-Rescue-Timestamp": ts,
            "X-Rescue-Signature": sig,  # signature is for the ORIGINAL body
        },
    )
    assert res.status_code == 401


def test_signed_forwarder_end_to_end(
    signed_field_settings: Settings, signed_receiver_client: TestClient
):
    engine = create_db_engine(signed_field_settings.field_database_url)
    init_db(engine)
    factory = create_session_factory(engine)
    with factory() as s:
        req = rescue_service.create_rescue_request(
            s,
            RescueRequestCreate(people_count=1, injury_status="no", condition="c", message="m"),
            source_node_id="field-01",
        )
        rid = req.request_id
        s.commit()

    counts = forwarder.run_once(factory, signed_receiver_client, signed_field_settings)
    assert counts["delivered"] == 1
    with factory() as s:
        assert rescue_repository.get_by_id(s, rid).delivery_status == "delivered"
    assert len(signed_receiver_client.get("/api/received").json()) == 1
