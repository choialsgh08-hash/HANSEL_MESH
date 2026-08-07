"""ROS adapter for HANSEL_MESH network metrics or an external provider plugin."""

from __future__ import annotations

import importlib

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from hansel_interfaces.msg import DetachRecommendation, NetworkStatus

from .provider import NetworkProvider


def load_provider(specification: str, adapter: "NetworkAdapterNode") -> NetworkProvider:
    module_name, separator, class_name = specification.partition(":")
    if not separator:
        raise ValueError("provider_plugin must use python.module:ClassName")
    provider_class = getattr(importlib.import_module(module_name), class_name)
    provider = provider_class(adapter)
    if not isinstance(provider, NetworkProvider):
        raise TypeError(f"{specification} does not implement NetworkProvider")
    return provider


class NetworkAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("network_adapter")
        self.declare_parameter("units", ["base", "head", "node1", "node2", "node3"])
        self.declare_parameter("provider_mode", "hansel_mesh_udp")
        self.declare_parameter("provider_plugin", "")
        self.declare_parameter("unavailable_publish_period_s", 2.0)
        self.declare_parameter("data_alive_timeout_s", 12.0)
        self.declare_parameter("udp_bind_host", "0.0.0.0")
        self.declare_parameter("udp_bind_port", 7100)
        self.declare_parameter("udp_max_datagram_bytes", 1048576)
        self.declare_parameter("degraded_tq_threshold", 180.0)
        self.declare_parameter("degraded_signal_dbm_threshold", -75.0)
        self.declare_parameter("degraded_loss_percent_threshold", 20.0)
        self.declare_parameter("critical_loss_percent_threshold", 90.0)
        self.declare_parameter("recommendation_enabled", True)
        self.declare_parameter("recommendation_consecutive_samples", 3)
        self.declare_parameter("recommendation_cooldown_s", 30.0)
        self.units = set(str(unit) for unit in self.get_parameter("units").value)
        self.status_publisher = self.create_publisher(
            NetworkStatus, "/hansel/network/status", 10
        )
        self.recommendation_publisher = self.create_publisher(
            DetachRecommendation, "/hansel/network/detach_recommendation", 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.provider: NetworkProvider | None = None
        self.provider_error = ""
        mode = str(self.get_parameter("provider_mode").value)
        specification = str(self.get_parameter("provider_plugin").value)
        try:
            if specification:
                self.provider = load_provider(specification, self)
            elif mode == "hansel_mesh_udp":
                from .hansel_mesh_provider import HanselMeshMetricsUdpProvider

                self.provider = HanselMeshMetricsUdpProvider(self)
            elif mode not in {"", "none"}:
                raise ValueError(f"unsupported provider_mode: {mode}")
            if self.provider is not None:
                self.provider.start()
        except Exception as exc:
            self.provider = None
            self.provider_error = str(exc)
            self.get_logger().error(f"network provider failed: {exc}")
        self.create_timer(
            float(self.get_parameter("unavailable_publish_period_s").value),
            self._publish_adapter_state,
        )

    def publish_status(self, msg: NetworkStatus) -> None:
        if msg.unit_id not in self.units:
            raise ValueError(f"NetworkStatus has unknown unit_id: {msg.unit_id}")
        msg.data_available = True
        self.status_publisher.publish(msg)

    def publish_recommendation(self, msg: DetachRecommendation) -> None:
        if msg.released_unit_id not in self.units or msg.released_unit_id in {"base", "head"}:
            raise ValueError(f"invalid released_unit_id: {msg.released_unit_id}")
        self.recommendation_publisher.publish(msg)

    def _publish_adapter_state(self) -> None:
        now = self.get_clock().now().to_msg()
        stale_units = sorted(self.units) if self.provider is None else self.provider.unavailable_units(
            float(self.get_parameter("data_alive_timeout_s").value)
        )
        for unit in stale_units:
            msg = NetworkStatus()
            msg.stamp = now
            msg.unit_id = unit
            msg.link_state = NetworkStatus.LINK_UNKNOWN
            msg.data_available = False
            msg.provider = "" if self.provider is None else type(self.provider).__name__
            self.status_publisher.publish(msg)

        status = DiagnosticStatus()
        status.name = "hansel/network/adapter"
        status.hardware_id = "network_provider"
        if self.provider is None:
            status.level = DiagnosticStatus.WARN
            status.message = (
                f"provider UNAVAILABLE: {self.provider_error}"
                if self.provider_error
                else "provider UNAVAILABLE"
            )
            details = {}
        elif stale_units:
            status.level = DiagnosticStatus.WARN
            status.message = "network provider running; unit metrics missing/stale"
            details = self.provider.diagnostic_items()
        else:
            status.level = DiagnosticStatus.OK
            status.message = "network provider running"
            details = self.provider.diagnostic_items()
        details.update(
            {
                "provider_mode": str(self.get_parameter("provider_mode").value),
                "provider_plugin": str(self.get_parameter("provider_plugin").value),
                "stale_units": ",".join(stale_units),
            }
        )
        status.values = [KeyValue(key=k, value=str(v)) for k, v in details.items()]
        array = DiagnosticArray()
        array.header.stamp = now
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    def destroy_node(self) -> bool:
        if self.provider is not None:
            self.provider.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NetworkAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
