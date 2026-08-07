"""Survivor/AP adapter shell; media and protocols remain intentionally external."""

from __future__ import annotations

import importlib

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from hansel_interfaces.msg import SurvivorApStatus, SurvivorCommEvent

from .provider import SurvivorProvider


def load_provider(
    specification: str, adapter: "SurvivorAdapterNode"
) -> SurvivorProvider:
    module_name, separator, class_name = specification.partition(":")
    if not separator:
        raise ValueError("provider_plugin must use python.module:ClassName")
    provider_class = getattr(importlib.import_module(module_name), class_name)
    provider = provider_class(adapter)
    if not isinstance(provider, SurvivorProvider):
        raise TypeError(f"{specification} does not implement SurvivorProvider")
    return provider


class SurvivorAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("survivor_adapter")
        self.declare_parameter("units", ["head", "node1", "node2", "node3"])
        self.declare_parameter("provider_plugin", "")
        self.units = set(self.get_parameter("units").value)
        self.ap_publishers = {
            unit: self.create_publisher(
                SurvivorApStatus,
                f"/hansel/{unit}/survivor_ap/status",
                10,
            )
            for unit in self.units
        }
        self.event_publisher = self.create_publisher(
            SurvivorCommEvent, "/hansel/survivor/event", 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.provider: SurvivorProvider | None = None
        self.provider_error = ""
        specification = str(self.get_parameter("provider_plugin").value)
        if specification:
            try:
                self.provider = load_provider(specification, self)
                self.provider.start()
            except Exception as exc:
                self.provider_error = str(exc)
                self.get_logger().error(f"survivor provider failed: {exc}")
        self.create_timer(2.0, self._publish_adapter_state)

    def publish_ap_status(self, msg: SurvivorApStatus) -> None:
        if msg.unit_id not in self.ap_publishers:
            raise ValueError(f"SurvivorApStatus has unknown unit_id: {msg.unit_id}")
        msg.data_available = True
        self.ap_publishers[msg.unit_id].publish(msg)

    def publish_event(self, msg: SurvivorCommEvent) -> None:
        if msg.unit_id not in self.units:
            raise ValueError(f"SurvivorCommEvent has unknown unit_id: {msg.unit_id}")
        self.event_publisher.publish(msg)

    def _publish_adapter_state(self) -> None:
        now = self.get_clock().now().to_msg()
        if self.provider is None:
            for unit, publisher in self.ap_publishers.items():
                msg = SurvivorApStatus()
                msg.stamp = now
                msg.unit_id = unit
                msg.service_state = SurvivorApStatus.SERVICE_UNAVAILABLE
                msg.connected_client_count = -1
                msg.data_available = False
                msg.message = "provider UNAVAILABLE"
                publisher.publish(msg)

        array = DiagnosticArray()
        array.header.stamp = now
        status = DiagnosticStatus()
        status.name = "hansel/survivor/adapter"
        status.hardware_id = "survivor_ap_provider"
        if self.provider is None:
            status.level = DiagnosticStatus.WARN
            status.message = (
                f"provider UNAVAILABLE: {self.provider_error}"
                if self.provider_error
                else "provider UNAVAILABLE"
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = "provider loaded"
        status.values = [
            KeyValue(key="media_protocol", value="TBD"),
            KeyValue(key="motor_network_access", value="must be blocked externally"),
        ]
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    def destroy_node(self) -> bool:
        if self.provider is not None:
            self.provider.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SurvivorAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
