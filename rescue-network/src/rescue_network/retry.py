"""Pure retry policy: backoff timing + delivery-error classification.

No database or I/O here, so the policy is trivially unit-testable. Callers pass
their own ``rng`` for deterministic tests.
"""

from __future__ import annotations

import random
from collections.abc import Callable
from enum import Enum

# Exponential backoff bounds (seconds).
INITIAL_BACKOFF_SECONDS = 5.0
MAX_BACKOFF_SECONDS = 300.0  # 5 minutes
# Extra delay added on top of the exponential term, as a fraction of it.
JITTER_RATIO = 0.2

# 4xx statuses that are transient/config-related rather than "bad data", so the
# forwarder keeps retrying them instead of dropping a rescue request:
#   408 Request Timeout, 425 Too Early, 429 Too Many Requests,
#   401 Unauthorized, 403 Forbidden (token likely misconfigured, will be fixed).
_RETRYABLE_4XX = frozenset({401, 403, 408, 425, 429})


class DeliveryDecision(str, Enum):
    """What to do with a request after an HTTP response."""

    DELIVERED = "delivered"
    RETRY = "retry"
    PERMANENT_FAIL = "permanent_fail"


def compute_backoff_seconds(
    retry_count: int,
    *,
    rng: Callable[[], float] = random.random,
) -> float:
    """Return the delay before the ``retry_count``-th retry.

    ``retry_count`` is 1 for the first retry, 2 for the second, and so on. The
    exponential term is ``5s * 2**(retry_count-1)`` capped at 5 minutes, plus up
    to ``JITTER_RATIO`` of that term as jitter. With ``rng()==0`` the result is
    exactly the (capped) exponential term, which makes tests deterministic.
    """
    if retry_count < 1:
        retry_count = 1
    exponential = min(MAX_BACKOFF_SECONDS, INITIAL_BACKOFF_SECONDS * (2 ** (retry_count - 1)))
    jitter = exponential * JITTER_RATIO * rng()
    return exponential + jitter


def classify_response(status_code: int) -> DeliveryDecision:
    """Map an HTTP status code to a delivery decision.

    * 2xx                     → delivered
    * 5xx / retryable 4xx     → retry (network-ish or transient)
    * other 4xx (bad data)    → permanent failure
    """
    if 200 <= status_code < 300:
        return DeliveryDecision.DELIVERED
    if status_code >= 500 or status_code in _RETRYABLE_4XX:
        return DeliveryDecision.RETRY
    return DeliveryDecision.PERMANENT_FAIL
