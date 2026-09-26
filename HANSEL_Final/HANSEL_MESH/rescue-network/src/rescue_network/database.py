"""SQLite engine / session management.

Concurrency notes (web server + forwarder both touch the field DB):

* ``journal_mode=WAL`` lets readers and a writer coexist.
* ``busy_timeout`` makes writers wait for a lock instead of failing instantly.
* ``check_same_thread=False`` is required because FastAPI serves requests on a
  threadpool; safety is preserved by using one Session per request.

All timestamps are stored in UTC (see ``models`` defaults).
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from sqlalchemy import Engine, create_engine, event
from sqlalchemy.orm import DeclarativeBase, Session, sessionmaker


class Base(DeclarativeBase):
    """Declarative base for all ORM models."""


# 5 seconds, expressed in milliseconds for the SQLite PRAGMA.
_BUSY_TIMEOUT_MS = 5000


def _apply_sqlite_pragmas(dbapi_connection, _connection_record) -> None:
    """Enable WAL + a busy timeout on every new SQLite connection."""
    cursor = dbapi_connection.cursor()
    try:
        cursor.execute("PRAGMA journal_mode=WAL")
        cursor.execute(f"PRAGMA busy_timeout={_BUSY_TIMEOUT_MS}")
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.execute("PRAGMA synchronous=NORMAL")
    finally:
        cursor.close()


def create_db_engine(database_url: str) -> Engine:
    """Create an ``Engine`` for ``database_url`` with SQLite pragmas applied.

    For file-based SQLite URLs the parent directory is created if missing so a
    fresh node can start without a manual ``mkdir``.
    """
    if database_url.startswith("sqlite:///") and ":memory:" not in database_url:
        db_path = Path(database_url.removeprefix("sqlite:///"))
        db_path.parent.mkdir(parents=True, exist_ok=True)

    engine = create_engine(
        database_url,
        future=True,
        connect_args={"check_same_thread": False},
    )
    event.listen(engine, "connect", _apply_sqlite_pragmas)
    return engine


def create_session_factory(engine: Engine) -> sessionmaker[Session]:
    """Return a configured ``sessionmaker`` bound to ``engine``."""
    return sessionmaker(bind=engine, autoflush=False, expire_on_commit=False, future=True)


def init_db(engine: Engine) -> None:
    """Create all tables that do not yet exist."""
    # Import models for their side effect of registering with ``Base.metadata``.
    from . import models  # noqa: F401

    Base.metadata.create_all(engine)


@contextmanager
def session_scope(session_factory: sessionmaker[Session]) -> Iterator[Session]:
    """Provide a transactional session scope.

    Commits on success, rolls back on any exception, and always closes. Used by
    non-request code paths (e.g. the forwarder in Phase 2). Exceptions are
    re-raised, never swallowed.
    """
    session = session_factory()
    try:
        yield session
        session.commit()
    except Exception:
        session.rollback()
        raise
    finally:
        session.close()
