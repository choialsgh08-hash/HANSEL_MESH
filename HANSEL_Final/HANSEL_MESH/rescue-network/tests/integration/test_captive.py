"""Integration tests for the opt-in captive portal (Phase 6)."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from rescue_network.config import NodeRole, Settings
from rescue_network.field_app import create_app


@pytest.fixture
def captive_client(tmp_path: Path) -> Iterator[TestClient]:
    settings = Settings(
        node_role=NodeRole.FIELD,
        node_id="test-node",
        data_dir=tmp_path,
        captive_portal=True,
        _env_file=None,
    )
    with TestClient(create_app(settings)) as c:
        yield c


def test_android_probe_redirects_to_form(captive_client: TestClient):
    res = captive_client.get("/generate_204", follow_redirects=False)
    assert res.status_code == 302
    assert res.headers["location"] == "/"


def test_apple_probe_serves_portal(captive_client: TestClient):
    res = captive_client.get("/hotspot-detect.html")
    assert res.status_code == 200
    assert "구조 요청" in res.text


def test_unknown_path_redirects_to_form(captive_client: TestClient):
    res = captive_client.get("/anything/else", follow_redirects=False)
    assert res.status_code == 302
    assert res.headers["location"] == "/"


def test_real_routes_still_win(captive_client: TestClient):
    # Form and health must not be shadowed by the catch-all.
    assert captive_client.get("/").status_code == 200
    assert captive_client.get("/health").json()["status"] == "ok"


def test_captive_off_by_default(client: TestClient):
    # The default field app (fixture from conftest) has no catch-all.
    assert client.get("/no-such-path", follow_redirects=False).status_code == 404
