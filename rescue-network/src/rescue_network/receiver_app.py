"""Receiver-node FastAPI application (Phase 2).

Rescue-team side: accepts rescue requests forwarded over the mesh, stores them
idempotently, and serves a dashboard. Runs on the mesh IP/port the field nodes
target, e.g.::

    uvicorn rescue_network.receiver_app:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import asyncio
import json
import logging
from collections.abc import Iterator
from datetime import datetime, timezone
from pathlib import Path

from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, PlainTextResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from sqlalchemy import text
from sqlalchemy.orm import Session

from .config import Settings, get_settings
from .database import create_db_engine, create_session_factory, init_db
from .monitoring import render_prometheus
from .repositories import received_repository
from .schemas import (
    DashboardRow,
    HealthResponse,
    ReceivedRescuePayload,
    RescueAck,
)
from .security import verify_signature, verify_token
from .services import receiver_service

logger = logging.getLogger("rescue_network.receiver")

_PACKAGE_DIR = Path(__file__).resolve().parent
_TEMPLATES = Jinja2Templates(directory=str(_PACKAGE_DIR / "templates"))


def _fetch_new_received(session_factory, last_id: int) -> tuple[list[dict], int]:
    """Return (new dashboard rows as dicts, updated last_id) since ``last_id``.

    Extracted from the SSE endpoint so the incremental-fetch logic is unit
    testable without a live streaming connection.
    """
    with session_factory() as session:
        rows = received_repository.list_after_id(session, last_id, limit=200)
        items = [DashboardRow.model_validate(row).model_dump(mode="json") for row in rows]
        new_last = rows[-1].id if rows else last_id
    return items, new_last


def create_app(settings: Settings | None = None) -> FastAPI:
    """Build a receiver-node FastAPI app (optionally with injected settings)."""
    settings = settings or get_settings()

    engine = create_db_engine(settings.receiver_database_url)
    init_db(engine)
    session_factory = create_session_factory(engine)

    app = FastAPI(title="Rescue Network — Receiver Node", version="0.1.0")
    app.mount(
        "/static",
        StaticFiles(directory=str(_PACKAGE_DIR / "static")),
        name="static",
    )

    def get_session() -> Iterator[Session]:
        session = session_factory()
        try:
            yield session
            session.commit()
        except Exception:
            session.rollback()
            raise
        finally:
            session.close()

    async def require_auth(request: Request) -> None:
        """Authenticate a receive request via shared token or HMAC signature.

        With ``require_signature`` the HMAC over the raw body (+ timestamp +
        source node) must be valid and fresh; otherwise the shared token header
        must match. Neither the token nor the secret is ever logged.
        """
        secret = settings.rescue_shared_token.get_secret_value()
        if settings.require_signature:
            authorized = verify_signature(
                secret,
                request.headers.get("X-Source-Node"),
                request.headers.get("X-Rescue-Timestamp"),
                await request.body(),
                request.headers.get("X-Rescue-Signature"),
                now_epoch=datetime.now(timezone.utc).timestamp(),
                max_skew_seconds=settings.signature_max_skew_seconds,
            )
        else:
            authorized = verify_token(request.headers.get("X-Rescue-Token"), secret)
        if not authorized:
            logger.warning("rejected receive: invalid or missing credentials")
            raise HTTPException(status_code=401, detail="invalid credentials")

    @app.post("/api/rescue/receive", response_model=RescueAck)
    def receive_rescue(
        payload: ReceivedRescuePayload,
        session: Session = Depends(get_session),
        _auth: None = Depends(require_auth),
    ) -> RescueAck:
        """Idempotently store a forwarded rescue request and ACK it."""
        _record, duplicate = receiver_service.store_received(session, payload)
        return RescueAck(request_id=payload.request_id, accepted=True, duplicate=duplicate)

    @app.get("/api/received", response_model=list[DashboardRow])
    def list_received(
        session: Session = Depends(get_session),
        limit: int = 200,
    ) -> list[DashboardRow]:
        """Return recently received requests as JSON (dashboard polling)."""
        limit = max(1, min(limit, 500))
        rows = received_repository.list_recent(session, limit=limit)
        return [DashboardRow.model_validate(row) for row in rows]

    # Poll interval for the server-side stream loop (seconds).
    _stream_poll = 1.5

    @app.get("/api/received/stream")
    async def stream_received(request: Request) -> StreamingResponse:
        """Server-Sent Events stream: a snapshot then each new request as it lands.

        Server-side polling turned into a client-side push — the dashboard opens
        one long-lived connection instead of re-fetching every few seconds.
        """

        async def event_gen():
            last_id = 0  # 0 => first pass streams the full current snapshot
            yield ": connected\n\n"
            while True:
                items, last_id = _fetch_new_received(session_factory, last_id)
                for item in items:
                    yield f"data: {json.dumps(item)}\n\n"
                # Check disconnect AFTER flushing so the snapshot always arrives.
                if await request.is_disconnected():
                    break
                await asyncio.sleep(_stream_poll)

        return StreamingResponse(event_gen(), media_type="text/event-stream")

    @app.get("/dashboard", response_class=HTMLResponse)
    def dashboard(request: Request) -> HTMLResponse:
        """Rescue-team dashboard (HTML shell; data loaded via /api/received)."""
        return _TEMPLATES.TemplateResponse(request, "dashboard.html", {"node_id": settings.node_id})

    @app.get("/metrics", response_class=PlainTextResponse)
    def metrics(session: Session = Depends(get_session)) -> PlainTextResponse:
        """Prometheus-format metrics for this receiver node."""
        total = received_repository.count(session)
        body = render_prometheus(
            {"rescue_received_total": float(total)},
            {"node_id": settings.node_id, "role": settings.node_role.value},
        )
        return PlainTextResponse(body)

    @app.get("/health", response_model=HealthResponse)
    def health(session: Session = Depends(get_session)) -> HealthResponse:
        """Report receiver web-server and database health."""
        try:
            session.execute(text("SELECT 1"))
            db_status = "ok"
        except Exception:  # pragma: no cover - defensive; surfaced, not swallowed
            logger.exception("database health check failed")
            db_status = "error"
        return HealthResponse(
            status="ok" if db_status == "ok" else "degraded",
            role=settings.node_role.value,
            node_id=settings.node_id,
            database=db_status,
        )

    return app


# Module-level ASGI app for `uvicorn rescue_network.receiver_app:app`.
app = create_app()
