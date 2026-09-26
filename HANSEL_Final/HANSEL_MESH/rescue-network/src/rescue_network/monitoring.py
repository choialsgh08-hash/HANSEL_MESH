"""Monitoring helpers: metrics rendering + alert decisions.

Pure functions (no DB, no IO) so they are trivially unit-testable. The apps and
the forwarder feed in numbers they read from the database.
"""

from __future__ import annotations

from .schemas import DeliveryStatus


def render_prometheus(metrics: dict[str, float], labels: dict[str, str]) -> str:
    """Render ``name -> value`` pairs as Prometheus text exposition format.

    ``labels`` (e.g. node id / role) are attached to every metric line.
    """
    label_str = ",".join(f'{k}="{v}"' for k, v in labels.items())
    suffix = f"{{{label_str}}}" if label_str else ""
    lines = [f"{name}{suffix} {value}" for name, value in metrics.items()]
    return "\n".join(lines) + "\n"


def field_metrics(status_counts: dict[str, int], oldest_pending_age: float) -> dict[str, float]:
    """Build the field node's metric set from DB-derived numbers."""
    metrics: dict[str, float] = {}
    for status in DeliveryStatus:
        metrics[f"rescue_requests_total_{status.value}"] = float(status_counts.get(status.value, 0))
    metrics["rescue_oldest_pending_age_seconds"] = float(oldest_pending_age)
    return metrics


def alerts(
    status_counts: dict[str, int],
    oldest_pending_age: float,
    *,
    pending_age_threshold: float,
) -> list[str]:
    """Return human-readable alert messages for anything that looks wrong.

    Empty list means healthy. Two conditions are checked:
    permanently-failed requests exist, or a pending request has aged past the
    threshold (delivery is stuck).
    """
    messages: list[str] = []
    failed = status_counts.get(DeliveryStatus.FAILED.value, 0)
    if failed > 0:
        messages.append(f"{failed} rescue request(s) permanently FAILED")
    if oldest_pending_age > pending_age_threshold:
        messages.append(
            f"oldest pending request is {oldest_pending_age:.0f}s old "
            f"(threshold {pending_age_threshold:.0f}s) — delivery may be stuck"
        )
    return messages
