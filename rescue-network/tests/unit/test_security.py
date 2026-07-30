"""Unit tests for token + HMAC-signature verification."""

from __future__ import annotations

import pytest

from rescue_network.security import compute_signature, verify_signature, verify_token


def test_matching_token_accepted():
    assert verify_token("s3cret", "s3cret") is True


@pytest.mark.parametrize("provided", [None, "", "wrong", "s3cre", "s3crett"])
def test_non_matching_token_rejected(provided):
    assert verify_token(provided, "s3cret") is False


SECRET = "shared-secret"
NODE = "node-01"
BODY = b'{"request_id":"abc","people_count":2}'


def _sig(ts: str, body: bytes = BODY, node: str = NODE) -> str:
    return compute_signature(SECRET, node, ts, body)


def test_valid_signature_accepted():
    ts = "1000"
    assert verify_signature(
        SECRET, NODE, ts, BODY, _sig(ts), now_epoch=1000.0, max_skew_seconds=300
    )


def test_tampered_body_rejected():
    ts = "1000"
    assert not verify_signature(
        SECRET,
        NODE,
        ts,
        b'{"tampered":true}',
        _sig(ts),
        now_epoch=1000.0,
        max_skew_seconds=300,
    )


def test_stale_timestamp_rejected():
    ts = "1000"
    assert not verify_signature(
        SECRET, NODE, ts, BODY, _sig(ts), now_epoch=2000.0, max_skew_seconds=300
    )


def test_wrong_secret_rejected():
    ts = "1000"
    bad = compute_signature("other-secret", NODE, ts, BODY)
    assert not verify_signature(SECRET, NODE, ts, BODY, bad, now_epoch=1000.0, max_skew_seconds=300)


@pytest.mark.parametrize(
    "node,ts,sig",
    [(None, "1000", "x"), (NODE, None, "x"), (NODE, "1000", None)],
)
def test_missing_headers_rejected(node, ts, sig):
    assert not verify_signature(SECRET, node, ts, BODY, sig, now_epoch=1000.0, max_skew_seconds=300)


def test_unparseable_timestamp_rejected():
    assert not verify_signature(
        SECRET, NODE, "not-a-number", BODY, "x", now_epoch=1000.0, max_skew_seconds=300
    )
