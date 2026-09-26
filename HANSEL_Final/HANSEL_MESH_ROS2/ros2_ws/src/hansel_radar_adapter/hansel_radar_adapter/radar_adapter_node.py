"""ROS adapter for HANSEL_MESH mission-log radar data or provider plugins."""

from __future__ import annotations

import importlib
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from .provider import RadarProvider


def load_provider(specification: str, adapter: "RadarAdapterNode") -> RadarProvider:
    module_name, separator, class_name = specification.partition(":")
    if not separator:
        raise ValueError("provider_plugin must use python.module:ClassName")
    provider_class = getattr(importlib.import_module(module_name), class_name)
    provider = provider_class(adapter)
    if not isinstance(provider, RadarProvider):
        raise TypeError(f"{specification} does not implement RadarProvider")
    return provider


class RadarAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("radar_adapter")
        self.declare_parameter("provider_mode", "mission_log")
        self.declare_parameter("provider_plugin", "")
        self.declare_parameter("mission_log_path", "")
        self.declare_parameter("mission_log_start_at_end", True)
        self.declare_parameter("mission_log_poll_period_s", 0.1)
        self.declare_parameter("frame_id", "radar_link")
        self.declare_parameter("max_points_per_frame", 8192)
        self.declare_parameter("publish_occupancy_grid", True)
        self.declare_parameter("grid_resolution_m", 0.05)
        self.declare_parameter("grid_forward_range_m", 3.0)
        self.declare_parameter("grid_lateral_width_m", 3.0)
        self.declare_parameter("data_alive_timeout_s", 0.75)
        self.declare_parameter("data_fault_timeout_s", 2.0)
        point_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.points_publisher = self.create_publisher(
            PointCloud2, "/hansel/radar/points", point_qos
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, "/hansel/radar/map", 2)
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._last_data_monotonic: float | None = None
        self.provider: RadarProvider | None = None
        self.provider_error = ""
        mode = str(self.get_parameter("provider_mode").value)
        specification = str(self.get_parameter("provider_plugin").value)
        try:
            if specification:
                self.provider = load_provider(specification, self)
            elif mode == "mission_log":
                from .hansel_mesh_provider import HanselMeshMissionLogProvider

                self.provider = HanselMeshMissionLogProvider(self)
            elif mode not in {"", "none"}:
                raise ValueError(f"unsupported provider_mode: {mode}")
            if self.provider is not None:
                self.provider.start()
        except Exception as exc:
            self.provider = None
            self.provider_error = str(exc)
            self.get_logger().error(f"radar provider failed: {exc}")
        self.create_timer(1.0, self._publish_diagnostics)

    def publish_points(self, msg: PointCloud2) -> None:
        self._last_data_monotonic = time.monotonic()
        self.points_publisher.publish(msg)

    def publish_map(self, msg: OccupancyGrid) -> None:
        self._last_data_monotonic = time.monotonic()
        self.map_publisher.publish(msg)

    def publish_tf(self, msg: TFMessage) -> None:
        self.tf_publisher.publish(msg)

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "hansel/radar/adapter"
        status.hardware_id = "radar_provider"
        timeout = float(self.get_parameter("data_alive_timeout_s").value)
        age = (
            None
            if self._last_data_monotonic is None
            else time.monotonic() - self._last_data_monotonic
        )
        fault_timeout = float(self.get_parameter("data_fault_timeout_s").value)
        alive = age is not None and age <= timeout
        if self.provider is None:
            status.level = DiagnosticStatus.WARN
            status.message = (
                f"provider UNAVAILABLE: {self.provider_error}"
                if self.provider_error
                else "provider UNAVAILABLE"
            )
            details = {}
        elif alive:
            status.level = DiagnosticStatus.OK
            status.message = "radar data alive"
            details = self.provider.diagnostic_items()
        elif age is not None and age > fault_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = "provider running; radar data fault timeout"
            details = self.provider.diagnostic_items()
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "provider running; radar data stale"
            details = self.provider.diagnostic_items()
        details.update(
            {
                "provider_mode": str(self.get_parameter("provider_mode").value),
                "provider_plugin": str(self.get_parameter("provider_plugin").value),
                "control_coupling": "none (RViz visualization only)",
                "data_age_s": "never" if age is None else f"{age:.2f}",
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
    node = RadarAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
