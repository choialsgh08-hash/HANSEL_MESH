"""Route HANSEL_MESH semantic commands to active ROS unit namespaces."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from hansel_interfaces.msg import ActiveChain, MotionCommand
from .chain_registry import roles_from_entries


class CommandRouter(Node):
    def __init__(self) -> None:
        super().__init__("command_router")
        self.declare_parameter("ordered_units", ["head", "node1", "node2", "node3"])
        self.declare_parameter("roles", ["head=head", "node1=rear", "node2=rear", "node3=rear"])
        self.declare_parameter("initial_active_drive_units", ["head", "node1", "node2"])
        self.ordered_units = list(self.get_parameter("ordered_units").value)
        self.roles = roles_from_entries(list(self.get_parameter("roles").value))
        self.active_units = list(self.get_parameter("initial_active_drive_units").value)
        self._last_sequence_by_source: dict[str, int] = {}
        motion_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publishers = {
            unit: self.create_publisher(MotionCommand, f"/hansel/{unit}/command/motion", motion_qos)
            for unit in self.ordered_units
        }
        self.create_subscription(MotionCommand, "/hansel/system/command/motion", self._on_command, motion_qos)
        self.create_subscription(ActiveChain, "/hansel/system/state/active_chain", self._on_active_chain, latched_qos)

    @staticmethod
    def _normalize_rear(command: str) -> str:
        command = command.strip().lower().replace("-", "_").replace(" ", "_")
        if command in {"forward_left", "forward_right", "mild_forward_left", "mild_forward_right"}:
            return "slow_forward"
        if command in {"backward_left", "backward_right", "mild_backward_left", "mild_backward_right"}:
            return "slow_backward"
        if command in {"left", "right"} or command.startswith("head_servo_") or command.startswith("front_"):
            return "stop"
        return command

    def _on_active_chain(self, msg: ActiveChain) -> None:
        self.active_units = [u for u in self.ordered_units if u in msg.active_drive_units]

    def _on_command(self, msg: MotionCommand) -> None:
        source = msg.source or "unknown"
        previous = self._last_sequence_by_source.get(source)
        if previous is not None and msg.sequence <= previous:
            return
        self._last_sequence_by_source[source] = int(msg.sequence)
        for unit in self.active_units:
            routed = MotionCommand()
            routed.stamp = msg.stamp
            routed.sequence = msg.sequence
            routed.command = msg.command if self.roles[unit] == "head" else self._normalize_rear(msg.command)
            routed.speed_scale = msg.speed_scale
            routed.source = msg.source
            self.publishers[unit].publish(routed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
