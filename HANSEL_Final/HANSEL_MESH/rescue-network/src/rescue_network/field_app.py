"""Field-node FastAPI application (Phase 1).

Serves the victim rescue-request form and the local intake API. Requests are
persisted to SQLite immediately; a successful local write yields an accepted
response regardless of any later network delivery.

Run with::

    uvicorn rescue_network.field_app:app --host 0.0.0.0 --port 80
"""

from __future__ import annotations

import logging
from collections.abc import Iterator
from datetime import datetime, timezone
from pathlib import Path

from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from sqlalchemy import text
from sqlalchemy.orm import Session

from . import captive
from .config import Settings, get_settings
from .database import create_db_engine, create_session_factory, init_db
from .monitoring import field_metrics, render_prometheus
from .repositories import rescue_repository
from .schemas import (
    DeliveryStatus,
    HealthResponse,
    RescueRequestAccepted,
    RescueRequestCreate,
    RescueRequestStatus,
)
from .services import rescue_service

logger = logging.getLogger("rescue_network.field")

_PACKAGE_DIR = Path(__file__).resolve().parent
_TEMPLATES = Jinja2Templates(directory=str(_PACKAGE_DIR / "templates"))


def create_app(settings: Settings | None = None) -> FastAPI:
    """Build a field-node FastAPI app.

    Accepting an optional ``settings`` keeps the app testable (tests inject a
    temp-dir config) instead of relying on a hidden global.
    """
    settings = settings or get_settings()

    engine = create_db_engine(settings.field_database_url)
    init_db(engine)
    session_factory = create_session_factory(engine)

    app = FastAPI(title="Rescue Network — Field Node", version="0.1.0")
    app.mount(
        "/static",
        StaticFiles(directory=str(_PACKAGE_DIR / "static")),
        name="static",
    )

    def get_session() -> Iterator[Session]:
        """Yield a request-scoped session, committing on success."""
        session = session_factory()
        try:
            yield session
            session.commit()
        except Exception:
            session.rollback()
            raise
        finally:
            session.close()

    @app.get("/", response_class=HTMLResponse)
    def rescue_form(request: Request) -> HTMLResponse:
        """Return the victim-facing rescue-request form."""
        return _TEMPLATES.TemplateResponse(
            request, "rescue_form.html", {"node_id": settings.node_id}
        )

    @app.post("/api/rescue", response_model=RescueRequestAccepted, status_code=201)
    def submit_rescue(
        payload: RescueRequestCreate,
        session: Session = Depends(get_session),
    ) -> RescueRequestAccepted:
        """Validate + persist a rescue request; respond as soon as it is stored."""
        request = rescue_service.create_rescue_request(
            session, payload, source_node_id=settings.node_id
        )
        return RescueRequestAccepted(
            request_id=request.request_id,
            delivery_status=DeliveryStatus(request.delivery_status),
            message="구조 요청이 저장되었습니다.",
        )

    @app.get("/api/rescue/{request_id}", response_model=RescueRequestStatus)
    def rescue_status(
        request_id: str,
        session: Session = Depends(get_session),
    ) -> RescueRequestStatus:
        """Return the current delivery status of a stored request."""
        request = rescue_service.get_rescue_request(session, request_id)
        if request is None:
            raise HTTPException(status_code=404, detail="request not found")
        return RescueRequestStatus.model_validate(request)

    @app.get("/metrics", response_class=PlainTextResponse)
    def metrics(session: Session = Depends(get_session)) -> PlainTextResponse:
        """Prometheus-format delivery metrics for this field node."""
        counts = rescue_repository.status_counts(session)
        oldest = rescue_repository.oldest_pending_created_at(session)
        now = datetime.now(timezone.utc).replace(tzinfo=None)
        age = (now - oldest).total_seconds() if oldest is not None else 0.0
        text_body = render_prometheus(
            field_metrics(counts, age),
            {"node_id": settings.node_id, "role": settings.node_role.value},
        )
        return PlainTextResponse(text_body)

    @app.get("/health", response_model=HealthResponse)
    def health(session: Session = Depends(get_session)) -> HealthResponse:
        """Report web-server and database health."""
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

    # Captive-portal routes must be registered LAST (catch-all lowest priority).
    if settings.captive_portal:
        captive.register(app)

    return app


# Module-level ASGI app for `uvicorn rescue_network.field_app:app`.
app = create_app()
