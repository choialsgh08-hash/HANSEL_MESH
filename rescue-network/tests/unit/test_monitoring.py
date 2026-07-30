"""Unit tests for monitoring helpers (metrics + alert decisions)."""

from __future__ import annotations

from rescue_network.monitoring import alerts, field_metrics, render_prometheus


def test_render_prometheus_attaches_labels():
    text = render_prometheus({"foo_total": 3.0}, {"node_id": "n1", "role": "field"})
    assert 'foo_total{node_id="n1",role="field"} 3.0' in text
    assert text.endswith("\n")


def test_field_metrics_covers_all_statuses():
    m = field_metrics({"pending": 2, "delivered": 5}, 12.0)
    assert m["rescue_requests_total_pending"] == 2.0
    assert m["rescue_requests_total_delivered"] == 5.0
    assert m["rescue_requests_total_failed"] == 0.0  # absent -> 0
    assert m["rescue_oldest_pending_age_seconds"] == 12.0


def test_alerts_empty_when_healthy():
    assert alerts({"pending": 1, "failed": 0}, 10.0, pending_age_threshold=600) == []


def test_alerts_flag_failed():
    msgs = alerts({"failed": 2}, 0.0, pending_age_threshold=600)
    assert any("FAILED" in m for m in msgs)


def test_alerts_flag_stuck_pending():
    msgs = alerts({"pending": 1}, 900.0, pending_age_threshold=600)
    assert any("stuck" in m for m in msgs)


def test_alerts_can_raise_both():
    msgs = alerts({"failed": 1, "pending": 1}, 900.0, pending_age_threshold=600)
    assert len(msgs) == 2
