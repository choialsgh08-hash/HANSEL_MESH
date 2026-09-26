"""Shared pytest fixtures.

Every test runs fully offline against throwaway SQLite files in a temp dir. The
field app, the receiver app, and the forwarder all share the same temp
``data_dir`` (distinct field.db / receiver.db files) and the same shared token.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from sqlalchemy.orm import Session, sessionmaker

from rescue_network.config import NodeRole, Settings
from rescue_network.database import create_db_engine, create_session_factory, init_db
from rescue_network.field_app import create_app as create_field_app
from rescue_network.receiver_app import create_app as create_receiver_app
from rescue_network.schemas import RescueRequestCreate
from rescue_network.services import rescue_service

TEST_TOKEN = "test-token"


@pytest.fixture
def settings(tmp_path: Path) -> Settings:
    """Field-node settings pointed at an isolated temp data dir.

    ``receiver_url`` is the TestClient base host so the forwarder posts straight
    into the in-process receiver app.
    """
    return Settings(
        node_role=NodeRole.FIELD,
        node_id="test-node",
        data_dir=tmp_path,
        rescue_shared_token=TEST_TOKEN,
        receiver_url="http://testserver",
        _env_file=None,
    )


@pytest.fixture
def receiver_settings(tmp_path: Path) -> Settings:
    """Receiver-node settings sharing the temp dir + token with ``settings``."""
    return Settings(
        node_role=NodeRole.RECEIVER,
        node_id="receiver-node",
        data_dir=tmp_path,
        rescue_shared_token=TEST_TOKEN,
        _env_file=None,
    )


@pytest.fixture
def client(settings: Settings) -> Iterator[TestClient]:
    """TestClient for a freshly-built field app."""
    app = create_field_app(settings)
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def receiver_client(receiver_settings: Settings) -> Iterator[TestClient]:
    """TestClient for a freshly-built receiver app (auth header required)."""
    app = create_receiver_app(receiver_settings)
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def field_session_factory(settings: Settings) -> sessionmaker[Session]:
    """A session factory bound to the field DB (for forwarder-side tests)."""
    engine = create_db_engine(settings.field_database_url)
    init_db(engine)
    return create_session_factory(engine)


@pytest.fixture
def make_pending_request(
    field_session_factory: sessionmaker[Session],
) -> Callable[..., str]:
    """Return a helper that stores a pending rescue request and returns its id."""

    def _make(**overrides: object) -> str:
        payload = RescueRequestCreate(
            people_count=overrides.pop("people_count", 2),
            injury_status=overrides.pop("injury_status", "yes"),
            condition=overrides.pop("condition", "고립됨"),
            message=overrides.pop("message", "도와주세요"),
            location_text=overrides.pop("location_text", "3층"),
        )
        with field_session_factory() as session:
            request = rescue_service.create_rescue_request(
                session, payload, source_node_id="test-node"
            )
            request_id = request.request_id
            session.commit()
        return request_id

    return _make
