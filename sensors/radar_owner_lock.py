"""Cross-process ownership lock for one physical XDS110 radar."""

from __future__ import annotations

from dataclasses import dataclass
import errno
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from typing import BinaryIO

_IS_WINDOWS = os.name == "nt"

RADAR_OWNER_LOCK_ROOT = (
    Path(tempfile.gettempdir()) / "hansel-radar-owner-locks"
)
RADAR_UART_LOCK_ROOT = (
    Path(tempfile.gettempdir()) / "hansel-radar-uart-locks"
)
_WINDOWS_LOCK_RETRY_S = 0.05


def _lock_path(root: Path, serial_number: str) -> Path:
    digest = hashlib.sha256(serial_number.encode("utf-8")).hexdigest()[:20]
    return root / f"xds110-{digest}.lock"


def _lock_file(
    handle: BinaryIO,
    *,
    offset: int = 0,
    blocking: bool = False,
) -> None:
    handle.seek(offset)
    if _IS_WINDOWS:
        import msvcrt

        if not blocking:
            msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            return
        while True:
            try:
                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
                return
            except OSError as exc:
                if not _is_lock_contention(exc):
                    raise
                time.sleep(_WINDOWS_LOCK_RETRY_S)
    else:
        import fcntl

        mode = fcntl.LOCK_EX
        if not blocking:
            mode |= fcntl.LOCK_NB
        fcntl.lockf(handle.fileno(), mode, 1, offset, os.SEEK_SET)


def _is_lock_contention(error: OSError) -> bool:
    return error.errno in {
        errno.EACCES,
        errno.EAGAIN,
        errno.EDEADLK,
    } or getattr(error, "winerror", None) in {33, 36}


def _unlock_file(handle: BinaryIO, *, offset: int = 0) -> None:
    handle.seek(offset)
    if _IS_WINDOWS:
        import msvcrt

        msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
    else:
        import fcntl

        fcntl.lockf(
            handle.fileno(),
            fcntl.LOCK_UN,
            1,
            offset,
            os.SEEK_SET,
        )


def _open_lock_file(path: Path) -> BinaryIO:
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
    handle = os.fdopen(descriptor, "r+b", buffering=0)
    try:
        size = path.stat().st_size
        if size < 2:
            handle.seek(size)
            handle.write(b"\0" * (2 - size))
            handle.flush()
        return handle
    except BaseException:
        handle.close()
        raise


def _read_owner(path: Path) -> tuple[object, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return "unknown", "unknown"
    if not isinstance(payload, dict):
        return "unknown", "unknown"
    return payload.get("pid", "unknown"), payload.get("run_id", "unknown")


def _write_owner(path: Path, payload: bytes) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


@dataclass
class RadarOwnerLock:
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

    def __enter__(self) -> "RadarOwnerLock":
        return self

    def __exit__(self, *exc_info: object) -> None:
        del exc_info
        self.release()


def acquire_radar_owner_lock(
    root: Path,
    serial_number: str,
    run_id: str,
) -> RadarOwnerLock:
    """Acquire an OS-released process-lifetime lock for one XDS110 serial."""

    serial_number = serial_number.strip()
    run_id = run_id.strip()
    if not serial_number:
        raise ValueError("XDS110 serial number must not be empty")
    if not run_id:
        raise ValueError("radar run ID must not be empty")
    root.mkdir(parents=True, exist_ok=True)
    path = _lock_path(root, serial_number)
    handle = _open_lock_file(path)
    owner_acquired = False
    try:
        _lock_file(handle, offset=1, blocking=True)
        try:
            try:
                _lock_file(handle, offset=0)
            except OSError as exc:
                owner_pid, owner_run_id = _read_owner(
                    path.with_suffix(".owner.json")
                )
                raise RuntimeError(
                    "XDS110 radar is already owned by "
                    f"PID {owner_pid}, run ID {owner_run_id!r}"
                ) from exc
            owner_acquired = True

            payload = (
                json.dumps(
                    {
                        "pid": os.getpid(),
                        "run_id": run_id,
                        "xds_serial": serial_number,
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                )
                + "\n"
            ).encode("utf-8")
            _write_owner(path.with_suffix(".owner.json"), payload)
        finally:
            _unlock_file(handle, offset=1)
    except BaseException:
        if owner_acquired:
            _unlock_file(handle, offset=0)
        handle.close()
        raise
    handle.seek(0)
    return RadarOwnerLock(path, handle)


__all__ = [
    "RADAR_OWNER_LOCK_ROOT",
    "RADAR_UART_LOCK_ROOT",
    "RadarOwnerLock",
    "acquire_radar_owner_lock",
]
