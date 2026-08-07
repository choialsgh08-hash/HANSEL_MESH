"""Log state changes and events without flooding continuous telemetry."""

from __future__ import annotations

from functools import partial

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node

from hansel_interfaces.msg import (
    ActiveChain,
    CameraReceiveStatus,
    DetachRecommendation,
    UnitState,
)


STATE_NAMES = {
    UnitState.INITIALIZING: "INITIALIZING",
    UnitState.STOPPED: "STOPPED",
    UnitState.ACTIVE: "ACTIVE",
    UnitState.DETACHING: "DETACHING",
    UnitState.RELAY_ASSUMED: "RELAY_ASSUMED",
    UnitState.ESTOP: "ESTOP",
    UnitState.FAULT: "FAULT",
}


class EventLogger(Node):
    def __init__(self) -> None:
        super().__init__("event_logger")
        self.declare_parameter("units", ["head", "node1", "node2", "node3"])
        self._unit_states: dict[str, int] = {}
        self._active_units: tuple[str, ...] = ()
        self._camera_receiving: bool | None = None
        self._diagnostics: dict[str, tuple[int, str]] = {}

        for unit in self.get_parameter("units").value:
            self.create_subscription(
                UnitState,
                f"/hansel/{unit}/state/unit",
                partial(self._on_unit_state, unit),
                10,
            )
        self.create_subscription(
            ActiveChain,
            "/hansel/system/state/active_chain",
            self._on_chain,
            10,
        )
        self.create_subscription(
            DetachRecommendation,
            "/hansel/network/detach_recommendation",
            self._on_recommendation,
            10,
        )
        self.create_subscription(
            CameraReceiveStatus,
            "/hansel/camera/receive_status",
            self._on_camera,
            10,
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )

    def _on_unit_state(self, unit: str, msg: UnitState) -> None:
        previous = self._unit_states.get(unit)
        if previous == msg.operation_state:
            return
        self._unit_states[unit] = msg.operation_state
        state = STATE_NAMES.get(msg.operation_state, str(msg.operation_state))
        self.get_logger().info(f"{unit} -> {state}: {msg.status_message}")

    def _on_chain(self, msg: ActiveChain) -> None:
        active = tuple(msg.active_drive_units)
        if active == self._active_units:
            return
        self._active_units = active
        self.get_logger().info(
            "software-estimated active drive units: " + ",".join(active)
        )

    def _on_recommendation(self, msg: DetachRecommendation) -> None:
        self.get_logger().warning(
            f"network recommendation: leave {msg.released_unit_id} "
            f"severity={msg.severity} reason={msg.reason}"
        )

    def _on_camera(self, msg: CameraReceiveStatus) -> None:
        if self._camera_receiving == msg.receiving:
            return
        self._camera_receiving = msg.receiving
        label = "receiving" if msg.receiving else "not receiving"
        self.get_logger().info(f"camera is {label}")

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            current = (status.level, status.message)
            if self._diagnostics.get(status.name) == current:
                continue
            self._diagnostics[status.name] = current
            if status.level >= DiagnosticStatus.ERROR:
                self.get_logger().error(f"{status.name}: {status.message}")
            elif status.level == DiagnosticStatus.WARN:
                self.get_logger().warning(f"{status.name}: {status.message}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EventLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

