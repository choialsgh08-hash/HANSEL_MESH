"""UDP JSON bridge for HANSEL_MESH monitor/metrics_agent.py snapshots."""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

from diagnostic_msgs.msg import KeyValue
from hansel_interfaces.msg import DetachRecommendation, NetworkStatus

from .provider import NetworkProvider


def _kv(key: str, value: Any) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = "" if value is None else str(value)
    return item


def _number(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class HanselMeshMetricsUdpProvider(NetworkProvider):
    """Receive one JSON snapshot per UDP datagram and expose ROS status.

    Expected top-level fields match HANSEL_MESH/monitor/metrics_agent.py:
    node, mesh_if, bat_if, ts, links, end_to_end and bat0.
    """

    PROVIDER_NAME = "HANSEL_MESH.metrics_agent/udp-json-v1"

    def __init__(self, adapter) -> None:
        super().__init__(adapter)
        p = adapter.get_parameter
        self.host = str(p("udp_bind_host").value)
        self.port = int(p("udp_bind_port").value)
        self.max_datagram_bytes = int(p("udp_max_datagram_bytes").value)
        self.degraded_tq = float(p("degraded_tq_threshold").value)
        self.degraded_signal = float(p("degraded_signal_dbm_threshold").value)
        self.degraded_loss = float(p("degraded_loss_percent_threshold").value)
        self.critical_loss = float(p("critical_loss_percent_threshold").value)
        self.recommendation_enabled = bool(p("recommendation_enabled").value)
        self.recommendation_samples = max(
            1, int(p("recommendation_consecutive_samples").value)
        )
        self.recommendation_cooldown_s = max(
            0.0, float(p("recommendation_cooldown_s").value)
        )
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._lock = threading.Lock()
        self._received = 0
        self._invalid = 0
        self._last_node = ""
        self._last_receive_monotonic: float | None = None
        self._last_receive_by_unit: dict[str, float] = {}
        self._critical_counts: dict[str, int] = {}
        self._last_recommendation: dict[str, float] = {}

    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.settimeout(0.5)
        self._socket = sock
        self._thread = threading.Thread(
            target=self._receive_loop,
            name="hansel-mesh-metrics-udp",
            daemon=True,
        )
        self._thread.start()
        self.adapter.get_logger().info(
            f"HANSEL_MESH metrics UDP listening on {self.host}:{self.port}"
        )

    def stop(self) -> None:
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None

    def _receive_loop(self) -> None:
        assert self._socket is not None
        while not self._stop.is_set():
            try:
                payload, _address = self._socket.recvfrom(self.max_datagram_bytes)
            except socket.timeout:
                continue
            except OSError:
                if not self._stop.is_set():
                    self.adapter.get_logger().exception("network UDP receive failed")
                break
            try:
                snapshot = json.loads(payload.decode("utf-8"))
                if not isinstance(snapshot, dict):
                    raise ValueError("snapshot must be a JSON object")
                self._handle_snapshot(snapshot)
            except Exception as exc:
                with self._lock:
                    self._invalid += 1
                self.adapter.get_logger().warning(f"invalid network snapshot: {exc}")

    def _handle_snapshot(self, snapshot: dict[str, Any]) -> None:
        unit = str(snapshot.get("node", "")).strip()
        if not unit:
            raise ValueError("snapshot.node is missing")
        if unit not in self.adapter.units:
            raise ValueError(f"unknown snapshot node: {unit}")
        links = snapshot.get("links") or []
        if not isinstance(links, list):
            links = []
        end_to_end = snapshot.get("end_to_end") or {}
        if not isinstance(end_to_end, dict):
            end_to_end = {}
        bat0 = snapshot.get("bat0") or {}
        if not isinstance(bat0, dict):
            bat0 = {}

        best_link = self._best_link(links)
        base_stats = self._base_stats(end_to_end)
        state, reason = self._classify(best_link, base_stats)

        msg = NetworkStatus()
        msg.stamp = self.adapter.get_clock().now().to_msg()
        msg.unit_id = unit
        msg.next_hop = self._next_hop(best_link)
        msg.link_state = state
        msg.provider = self.PROVIDER_NAME
        metrics = [
            _kv("source_ts", snapshot.get("ts")),
            _kv("mesh_if", snapshot.get("mesh_if", "")),
            _kv("bat_if", snapshot.get("bat_if", "")),
            _kv("link_count", len(links)),
            _kv("classification", reason),
        ]
        if best_link:
            for key in (
                "peer",
                "mac",
                "ip",
                "tq",
                "signal_dbm",
                "tx_mbps",
                "expected_mbps",
                "last_seen_s",
                "nexthop",
                "direct",
            ):
                if key in best_link:
                    metrics.append(_kv(f"best_{key}", best_link.get(key)))
        for key, value in sorted(base_stats.items()):
            metrics.append(_kv(f"base_{key}", value))
        for key in ("rx_bytes", "rx_packets", "rx_dropped", "tx_bytes", "tx_packets", "tx_dropped"):
            if key in bat0:
                metrics.append(_kv(f"bat0_{key}", bat0.get(key)))
        msg.metrics = metrics
        self.adapter.publish_status(msg)

        loss = _number(
            base_stats.get(
                "loss_pct",
                base_stats.get("loss_percent", base_stats.get("packet_loss_percent")),
            ),
            0.0,
        )
        critical = (
            state == NetworkStatus.LINK_DOWN
            or (loss if loss is not None else 0.0) >= self.critical_loss
        )
        self._maybe_recommend(unit, critical, reason, metrics)
        now_monotonic = time.monotonic()
        with self._lock:
            self._received += 1
            self._last_node = unit
            self._last_receive_monotonic = now_monotonic
            self._last_receive_by_unit[unit] = now_monotonic

    @staticmethod
    def _best_link(links: list[Any]) -> dict[str, Any]:
        candidates = [item for item in links if isinstance(item, dict)]
        if not candidates:
            return {}
        return max(
            candidates,
            key=lambda item: (
                _number(item.get("tq"), -1.0) or -1.0,
                _number(item.get("signal_dbm"), -999.0) or -999.0,
            ),
        )

    @staticmethod
    def _base_stats(end_to_end: dict[str, Any]) -> dict[str, Any]:
        for key in ("base", "192.168.50.1"):
            value = end_to_end.get(key)
            if isinstance(value, dict):
                return value
        for value in end_to_end.values():
            if isinstance(value, dict) and (
                value.get("peer") == "base" or value.get("name") == "base"
            ):
                return value
        return {}

    def _classify(
        self, best_link: dict[str, Any], base_stats: dict[str, Any]
    ) -> tuple[int, str]:
        tq = _number(best_link.get("tq"))
        signal = _number(best_link.get("signal_dbm"))
        loss = _number(
            base_stats.get("loss_pct", base_stats.get("loss_percent", base_stats.get("packet_loss_percent")))
        )
        if not best_link and (loss is None or loss >= self.critical_loss):
            return NetworkStatus.LINK_DOWN, "no BATMAN neighbor and base unreachable"
        degraded_reasons = []
        if tq is not None and tq < self.degraded_tq:
            degraded_reasons.append(f"TQ {tq:.0f}<{self.degraded_tq:.0f}")
        if signal is not None and signal < self.degraded_signal:
            degraded_reasons.append(
                f"signal {signal:.0f}<{self.degraded_signal:.0f} dBm"
            )
        if loss is not None and loss > self.degraded_loss:
            degraded_reasons.append(
                f"loss {loss:.1f}>{self.degraded_loss:.1f}%"
            )
        if degraded_reasons:
            return NetworkStatus.LINK_DEGRADED, "; ".join(degraded_reasons)
        return NetworkStatus.LINK_UP, "mesh link available"

    @staticmethod
    def _next_hop(best_link: dict[str, Any]) -> str:
        for key in ("peer", "nexthop", "ip", "mac"):
            value = best_link.get(key)
            if value:
                return str(value)
        return ""

    def _maybe_recommend(
        self,
        unit: str,
        critical: bool,
        reason: str,
        metrics: list[KeyValue],
    ) -> None:
        if not self.recommendation_enabled or unit in {"base", "head"}:
            return
        if not critical:
            self._critical_counts[unit] = 0
            return
        count = self._critical_counts.get(unit, 0) + 1
        self._critical_counts[unit] = count
        if count < self.recommendation_samples:
            return
        now = time.monotonic()
        if now - self._last_recommendation.get(unit, -1e9) < self.recommendation_cooldown_s:
            return
        msg = DetachRecommendation()
        msg.stamp = self.adapter.get_clock().now().to_msg()
        msg.released_unit_id = unit
        msg.severity = DetachRecommendation.SEVERITY_CRITICAL
        msg.reason = reason
        msg.metrics_snapshot = metrics
        msg.provider = self.PROVIDER_NAME
        self.adapter.publish_recommendation(msg)
        self._last_recommendation[unit] = now
        self._critical_counts[unit] = 0

    def unavailable_units(self, timeout_s: float) -> list[str]:
        now = time.monotonic()
        with self._lock:
            return sorted(
                unit
                for unit in self.adapter.units
                if unit not in self._last_receive_by_unit
                or now - self._last_receive_by_unit[unit] > timeout_s
            )

    def diagnostic_items(self) -> dict[str, str]:
        with self._lock:
            age = (
                "never"
                if self._last_receive_monotonic is None
                else f"{time.monotonic() - self._last_receive_monotonic:.2f}"
            )
            return {
                "mode": "hansel_mesh_udp",
                "bind": f"{self.host}:{self.port}",
                "received_datagrams": str(self._received),
                "invalid_datagrams": str(self._invalid),
                "last_node": self._last_node,
                "last_receive_age_s": age,
                "units_seen": ",".join(sorted(self._last_receive_by_unit)),
            }
