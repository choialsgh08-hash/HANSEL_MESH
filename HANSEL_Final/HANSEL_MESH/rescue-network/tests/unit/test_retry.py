"""Unit tests for the retry policy (backoff + error classification)."""

from __future__ import annotations

import pytest

from rescue_network.retry import (
    INITIAL_BACKOFF_SECONDS,
    MAX_BACKOFF_SECONDS,
    DeliveryDecision,
    classify_response,
    compute_backoff_seconds,
)

# rng() -> 0.0 removes jitter, giving the exact exponential term.
_no_jitter = lambda: 0.0  # noqa: E731


@pytest.mark.parametrize(
    ("retry_count", "expected"),
    [(1, 5.0), (2, 10.0), (3, 20.0), (4, 40.0), (5, 80.0), (6, 160.0)],
)
def test_backoff_is_exponential(retry_count, expected):
    assert compute_backoff_seconds(retry_count, rng=_no_jitter) == expected


def test_backoff_starts_at_initial():
    assert compute_backoff_seconds(1, rng=_no_jitter) == INITIAL_BACKOFF_SECONDS


def test_backoff_capped_at_max():
    # 5 * 2**10 = 5120s would blow past the cap.
    assert compute_backoff_seconds(11, rng=_no_jitter) == MAX_BACKOFF_SECONDS


def test_backoff_clamps_nonpositive_retry_count():
    assert compute_backoff_seconds(0, rng=_no_jitter) == INITIAL_BACKOFF_SECONDS


def test_backoff_adds_bounded_jitter():
    # Full jitter (rng()->1.0) adds JITTER_RATIO (0.2) of the exponential term.
    base = compute_backoff_seconds(2, rng=_no_jitter)
    jittered = compute_backoff_seconds(2, rng=lambda: 1.0)
    assert jittered > base
    assert jittered == pytest.approx(base * 1.2)


@pytest.mark.parametrize("status", [200, 201, 204])
def test_classify_2xx_delivered(status):
    assert classify_response(status) is DeliveryDecision.DELIVERED


@pytest.mark.parametrize("status", [500, 502, 503, 504])
def test_classify_5xx_retry(status):
    assert classify_response(status) is DeliveryDecision.RETRY


@pytest.mark.parametrize("status", [401, 403, 408, 425, 429])
def test_classify_transient_4xx_retry(status):
    assert classify_response(status) is DeliveryDecision.RETRY


@pytest.mark.parametrize("status", [400, 404, 409, 415, 422])
def test_classify_bad_data_4xx_permanent(status):
    assert classify_response(status) is DeliveryDecision.PERMANENT_FAIL
