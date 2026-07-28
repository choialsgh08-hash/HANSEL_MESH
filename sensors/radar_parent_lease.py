"""OS-released parent-death notification for owned radar children."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import tempfile
import threading
from typing import BinaryIO
import uuid

from sensors.radar_owner_lock import (
    _lock_file,
    _open_lock_file,
    _unlock_file,
)

RADAR_PARENT_LEASE_ROOT = (
    Path(tempfile.gettempdir()) / "hansel-radar-parent-leases"
)


@dataclass
class ParentDeathLease:
    path: Path
    _handle: BinaryIO | None

    def release(self) -> None:
        handle = self._handle
        if handle is None:
            return
        self._handle = None
        try:
            _unlock_file(handle)
        finally:
            handle.close()
        try:
            self.path.unlink()
        except (FileNotFoundError, PermissionError):
            pass


@dataclass(frozen=True)
class ParentDeathWatcher:
    ready: threading.Event
    stop_requested: threading.Event
    thread: threading.Thread


def create_parent_death_lease(
    root: Path,
    role: str,
) -> ParentDeathLease:
    """Create and hold a unique lease released automatically at parent death."""

    if not isinstance(root, Path):
        raise ValueError("parent-death lease root must be a Path")
    if not isinstance(role, str) or not role.strip():
        raise ValueError("parent-death lease role must be non-empty")
    root.mkdir(parents=True, exist_ok=True)
    path = root / (
        f"{role.strip()}-{os.getpid()}-{uuid.uuid4().hex}.lease"
    )
    handle = _open_lock_file(path)
    try:
        _lock_file(handle)
    except BaseException:
        handle.close()
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        raise
    return ParentDeathLease(path, handle)


def start_parent_death_watcher(path: Path) -> ParentDeathWatcher:
    """Return events driven by the release of a parent's unique lease."""

    if not isinstance(path, Path):
        raise ValueError("parent-death lease path must be a Path")
    ready = threading.Event()
    stop_requested = threading.Event()

    def wait_for_parent_exit() -> None:
        handle: BinaryIO | None = None
        try:
            handle = _open_lock_file(path)
            ready.set()
            _lock_file(handle, blocking=True)
        except BaseException:
            ready.set()
        else:
            try:
                _unlock_file(handle)
            except OSError:
                pass
        finally:
            if handle is not None:
                handle.close()
            try:
                path.unlink()
            except (FileNotFoundError, PermissionError):
                pass
            stop_requested.set()

    thread = threading.Thread(
        target=wait_for_parent_exit,
        name="radar-parent-death",
        daemon=True,
    )
    thread.start()
    return ParentDeathWatcher(ready, stop_requested, thread)


__all__ = [
    "RADAR_PARENT_LEASE_ROOT",
    "ParentDeathLease",
    "ParentDeathWatcher",
    "create_parent_death_lease",
    "start_parent_death_watcher",
]
