"""Standalone rescue-request forwarder process.

Run as its own process (not a web-server background task) so the web server and
delivery loop fail independently::

    python -m rescue_network.forwarder

Each pass:
  1. recovers stale ``sending`` rows (crashed prior attempts) back to pending,
  2. claims due ``pending`` rows and marks them ``sending`` (committed),
  3. delivers each one, committing the outcome per request.

Committing per request means a restart resumes cleanly with no lost work.
"""

from __future__ import annotations

import logging
import random
import signal
import time
from collections.abc import Callable
from datetime import datetime, timedelta

import httpx
from sqlalchemy.orm import Session, sessionmaker

from . import monitoring
from .config import Settings, get_settings
from .database import create_db_engine, create_session_factory, init_db
from .models import utcnow
from .repositories import rescue_repository
from .services import delivery_service
from .services.delivery_service import DeliveryOutcome, HttpClient

logger = logging.getLogger("rescue_network.forwarder")


def run_once(
    session_factory: sessionmaker[Session],
    client: HttpClient,
    settings: Settings,
    *,
    now_fn: Callable[[], datetime] = utcnow,
    rng: Callable[[], float] = random.random,
) -> dict[str, int]:
    """Run a single forwarding pass. Returns a small outcome tally.

    Safe to call repeatedly (that is exactly what the loop does) and safe to
    call from tests with an injected client / clock.
    """
    counts = {"recovered": 0, "delivered": 0, "retried": 0, "failed": 0}

    # 1) Recover stale sending rows.
    with session_factory() as session:
        cutoff = now_fn() - timedelta(seconds=settings.stale_sending_seconds)
        counts["recovered"] = rescue_repository.recover_stale_sending(session, cutoff)
        session.commit()

    # 2) Claim a batch of due requests (commit the sending checkpoint).
    with session_factory() as session:
        claimed = rescue_repository.claim_due(session, now_fn(), settings.forwarder_batch_size)
        claimed_ids = [request.request_id for request in claimed]
        session.commit()

    # 3) Deliver each claimed request in its own transaction.
    for request_id in claimed_ids:
        with session_factory() as session:
            request = rescue_repository.get_by_id(session, request_id)
            if request is None:  # pragma: no cover - deleted between passes
                continue
            outcome = delivery_service.attempt_delivery(
                session, request, client=client, settings=settings, now_fn=now_fn, rng=rng
            )
            session.commit()
        if outcome is DeliveryOutcome.DELIVERED:
            counts["delivered"] += 1
        elif outcome is DeliveryOutcome.FAILED:
            counts["failed"] += 1
        else:
            counts["retried"] += 1

    return counts


class _StopFlag:
    """Cooperative stop flag toggled by SIGINT/SIGTERM for clean shutdown."""

    def __init__(self) -> None:
        self.stop = False

    def request_stop(self, *_args: object) -> None:
        self.stop = True


def run_forever(
    session_factory: sessionmaker[Session],
    client: HttpClient,
    settings: Settings,
    *,
    stop_flag: _StopFlag | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Poll-and-deliver loop until a stop is requested."""
    flag = stop_flag or _StopFlag()
    logger.info(
        "forwarder started node_id=%s receiver=%s poll=%.1fs",
        settings.node_id,
        settings.receiver_url,
        settings.forwarder_poll_interval_seconds,
    )
    while not flag.stop:
        try:
            run_once(session_factory, client, settings)
            emit_health(session_factory, settings, client)
        except Exception:  # noqa: BLE001 - loop must survive a bad pass
            logger.exception("forwarder pass failed; continuing")
        if flag.stop:
            break
        sleep(settings.forwarder_poll_interval_seconds)
    logger.info("forwarder stopped")


def emit_health(
    session_factory: sessionmaker[Session],
    settings: Settings,
    client: HttpClient,
    *,
    now_fn: Callable[[], datetime] = utcnow,
) -> list[str]:
    """Log a delivery-health summary and raise alerts for anything wrong.

    Returns the alert messages (empty when healthy). Alerts are always logged;
    if ``alert_webhook`` is set they are also POSTed best-effort.
    """
    with session_factory() as session:
        counts = rescue_repository.status_counts(session)
        oldest = rescue_repository.oldest_pending_created_at(session)
    now = now_fn()
    now_naive = now.replace(tzinfo=None) if now.tzinfo else now
    age = (now_naive - oldest).total_seconds() if oldest is not None else 0.0

    logger.info(
        "health node_id=%s pending=%d sending=%d delivered=%d failed=%d oldest_pending=%.0fs",
        settings.node_id,
        counts.get("pending", 0),
        counts.get("sending", 0),
        counts.get("delivered", 0),
        counts.get("failed", 0),
        age,
    )

    messages = monitoring.alerts(
        counts, age, pending_age_threshold=settings.alert_pending_age_seconds
    )
    for msg in messages:
        logger.warning("ALERT node_id=%s %s", settings.node_id, msg)
    if messages and settings.alert_webhook:
        _post_alert(client, settings, messages)
    return messages


def _post_alert(client: HttpClient, settings: Settings, messages: list[str]) -> None:
    """Best-effort webhook notification; never breaks the loop."""
    import json

    body = json.dumps({"node_id": settings.node_id, "alerts": messages}).encode()
    try:
        client.post(
            settings.alert_webhook,
            content=body,
            headers={"Content-Type": "application/json"},
            timeout=settings.delivery_timeout_seconds,
        )
    except Exception:  # noqa: BLE001 - alerting must not affect delivery
        logger.exception("alert webhook failed")


def main() -> None:
    """Entry point for ``python -m rescue_network.forwarder``."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    settings = get_settings()

    engine = create_db_engine(settings.field_database_url)
    init_db(engine)
    session_factory = create_session_factory(engine)

    flag = _StopFlag()
    signal.signal(signal.SIGINT, flag.request_stop)
    signal.signal(signal.SIGTERM, flag.request_stop)

    with httpx.Client() as client:
        run_forever(session_factory, client, settings, stop_flag=flag)


if __name__ == "__main__":
    main()
