"""Node-to-node authentication.

Two mechanisms, sharing one secret:

* **Shared token** (default) — the secret travels verbatim in ``X-Rescue-Token``
  and is compared in constant time.
* **HMAC signature** (opt-in, ``RESCUE_REQUIRE_SIGNATURE=1``) — each request
  carries an ``X-Rescue-Signature`` (HMAC-SHA256 over timestamp + node + body)
  and an ``X-Rescue-Timestamp``. This protects body integrity and, via a
  timestamp freshness window, blocks replay. The secret itself never crosses
  the wire.

The secret value is never logged anywhere in the codebase.
"""

from __future__ import annotations

import hashlib
import hmac
import secrets


def verify_token(provided: str | None, expected: str) -> bool:
    """Return ``True`` iff ``provided`` matches ``expected`` (constant time).

    A missing/empty token is always rejected. ``secrets.compare_digest`` avoids
    leaking match length via timing.
    """
    if not provided:
        return False
    return secrets.compare_digest(provided, expected)


def compute_signature(secret: str, node_id: str, timestamp: str, body: bytes) -> str:
    """Return the hex HMAC-SHA256 over ``timestamp\\n node_id\\n body``.

    Both sides build the message identically from the raw transmitted body
    bytes, so no canonical-JSON agreement is needed.
    """
    message = timestamp.encode() + b"\n" + node_id.encode() + b"\n" + body
    return hmac.new(secret.encode(), message, hashlib.sha256).hexdigest()


def verify_signature(
    secret: str,
    node_id: str | None,
    timestamp: str | None,
    body: bytes,
    provided: str | None,
    *,
    now_epoch: float,
    max_skew_seconds: float,
) -> bool:
    """Verify an HMAC signature and that its timestamp is fresh.

    Rejects when any header is missing, the timestamp is unparseable or outside
    ``±max_skew_seconds`` (replay defence), or the signature does not match.
    """
    if not node_id or not timestamp or not provided:
        return False
    try:
        ts = float(timestamp)
    except ValueError:
        return False
    if abs(now_epoch - ts) > max_skew_seconds:
        return False
    expected = compute_signature(secret, node_id, timestamp, body)
    return secrets.compare_digest(provided, expected)
