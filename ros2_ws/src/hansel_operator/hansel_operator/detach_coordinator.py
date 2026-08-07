"""Coordinate MANUAL/AUTO detach without asserting physical separation."""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from hansel_interfaces.msg import ActiveChain, DetachMode, DetachRecommendation
from hansel_interfaces.srv import DetachUnit, PrepareDetach, TriggerDetach

from .chain_registry import ChainRegistry, roles_from_entries


class DetachCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("detach_coordinator")
        self._callbacks = ReentrantCallbackGroup()
        self.declare_parameter("ordered_units", ["head", "node1", "node2", "node3"])
        self.declare_parameter(
            "roles", ["head=head", "node1=rear", "node2=rear", "node3=rear"]
        )
        self.declare_parameter(
            "initial_active_drive_units", ["head", "node1", "node2", "node3"]
        )
        self.declare_parameter("stop_ack_timeout_s", 2.0)
        self.declare_parameter("actuator_ack_timeout_s", 5.0)
        self.declare_parameter("auto_duplicate_guard_s", 5.0)

        ordered_units = list(self.get_parameter("ordered_units").value)
        self.registry = ChainRegistry(
            ordered_units=ordered_units,
            roles=roles_from_entries(list(self.get_parameter("roles").value)),
            active_drive_units=list(
                self.get_parameter("initial_active_drive_units").value
            ),
        )
        self.mode = DetachMode.MANUAL
        self._request_sequence = 0
        self._execution_lock = threading.Lock()
        self._last_auto_request: dict[str, float] = {}

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.chain_publisher = self.create_publisher(
            ActiveChain, "/hansel/system/state/active_chain", latched_qos
        )
        self.create_subscription(
            DetachMode,
            "/hansel/detach/mode",
            self._on_mode,
            latched_qos,
            callback_group=self._callbacks,
        )
        self.create_subscription(
            DetachRecommendation,
            "/hansel/network/detach_recommendation",
            self._on_recommendation,
            10,
            callback_group=self._callbacks,
        )
        self.create_service(
            DetachUnit,
            "/hansel/detach/execute",
            self._on_execute,
            callback_group=self._callbacks,
        )
        self.stop_clients = {
            unit: self.create_client(
                PrepareDetach,
                f"/hansel/{unit}/prepare_detach",
                callback_group=self._callbacks,
            )
            for unit in ordered_units
        }
        self.detach_clients = {
            unit: self.create_client(
                TriggerDetach,
                f"/hansel/{unit}/trigger_detach",
                callback_group=self._callbacks,
            )
            for unit in ordered_units
        }
        self.create_timer(
            0.2, self._publish_initial_chain_once, callback_group=self._callbacks
        )
        self._initial_published = False

    def _publish_initial_chain_once(self) -> None:
        if not self._initial_published:
            self._initial_published = True
            self._publish_chain()

    def _on_mode(self, msg: DetachMode) -> None:
        if msg.mode not in {DetachMode.MANUAL, DetachMode.AUTO}:
            self.get_logger().warning(f"ignoring invalid detach mode: {msg.mode}")
            return
        self.mode = msg.mode
        label = "AUTO" if self.mode == DetachMode.AUTO else "MANUAL"
        self.get_logger().info(f"detach mode changed to {label} by {msg.source}")

    def _on_recommendation(self, msg: DetachRecommendation) -> None:
        if self.mode != DetachMode.AUTO:
            return
        released = msg.released_unit_id
        now = time.monotonic()
        guard = float(self.get_parameter("auto_duplicate_guard_s").value)
        if now - self._last_auto_request.get(released, -1e9) < guard:
            return
        self._last_auto_request[released] = now

        request = DetachUnit.Request()
        request.released_unit_id = released
        request.mode = DetachUnit.Request.AUTO
        request.source = f"network:{msg.provider or 'unknown'}"
        response = DetachUnit.Response()
        self._perform_detach(request, response)
        if response.command_completed:
            self.get_logger().warning(
                f"AUTO detach command completed for {released}; "
                "physical separation not asserted"
            )
        else:
            self.get_logger().error(
                f"AUTO detach command did not complete for {released}: "
                f"{response.message}"
            )

    def _on_execute(
        self, request: DetachUnit.Request, response: DetachUnit.Response
    ) -> DetachUnit.Response:
        return self._perform_detach(request, response)

    def _perform_detach(
        self, request: DetachUnit.Request, response: DetachUnit.Response
    ) -> DetachUnit.Response:
        if not self._execution_lock.acquire(blocking=False):
            response.accepted = False
            response.command_completed = False
            response.message = "another detach sequence is in progress"
            return response
        safe_stop_hold_placed = False
        sequence = 0
        released = request.released_unit_id
        try:
            if request.mode not in {DetachUnit.Request.MANUAL, DetachUnit.Request.AUTO}:
                response.accepted = False
                response.command_completed = False
                response.message = "invalid detach mode"
                return response
            if released not in self.registry.active_drive_units:
                response.accepted = False
                response.command_completed = False
                response.message = f"released unit is not active: {released}"
                return response
            try:
                actuator = self.registry.actuator_for(released)
            except ValueError as exc:
                response.accepted = False
                response.command_completed = False
                response.message = str(exc)
                return response

            response.accepted = True
            response.actuator_unit_id = actuator
            self._request_sequence += 1
            sequence = self._request_sequence

            safe_stop_hold_placed = True
            stopped, message = self._stop_all_active(sequence, released)
            if not stopped:
                response.command_completed = False
                response.message = message
                return response
            completed, message = self._trigger_actuator(
                sequence, actuator, released
            )
            if not completed:
                response.command_completed = False
                response.message = message
                return response

            self.registry.mark_relay_assumed(released)
            self._publish_chain()
            response.command_completed = True
            response.message = (
                f"detach command completed for {released} by {actuator}; "
                "physical separation not asserted"
            )
            return response
        finally:
            if safe_stop_hold_placed:
                self._release_stop_holds(
                    sequence,
                    released,
                    list(self.registry.active_drive_units),
                )
            self._execution_lock.release()

    def _stop_all_active(
        self, sequence: int, released: str
    ) -> tuple[bool, str]:
        futures = {}
        for unit in list(self.registry.active_drive_units):
            client = self.stop_clients[unit]
            if not client.service_is_ready():
                return False, f"stop service unavailable: {unit}"
            request = PrepareDetach.Request()
            request.request_sequence = sequence
            request.released_unit_id = released
            request.hold = True
            futures[unit] = client.call_async(request)

        timeout = float(self.get_parameter("stop_ack_timeout_s").value)
        deadline = time.monotonic() + timeout
        for unit, future in futures.items():
            if not self._wait_future(future, deadline):
                return False, f"stop ACK timeout: {unit}"
            try:
                result = future.result()
            except Exception as exc:
                return False, f"stop service error for {unit}: {exc}"
            if result is None or not result.stopped:
                detail = result.message if result is not None else "no response"
                return False, f"stop not acknowledged by {unit}: {detail}"
        return True, "all active units acknowledged safe stop"

    def _release_stop_holds(
        self, sequence: int, released: str, units: list[str]
    ) -> None:
        futures = []
        for unit in units:
            client = self.stop_clients[unit]
            if not client.service_is_ready():
                self.get_logger().error(
                    f"cannot release detach safe-stop hold: {unit}"
                )
                continue
            request = PrepareDetach.Request()
            request.request_sequence = sequence
            request.released_unit_id = released
            request.hold = False
            futures.append((unit, client.call_async(request)))
        deadline = time.monotonic() + float(
            self.get_parameter("stop_ack_timeout_s").value
        )
        for unit, future in futures:
            if not self._wait_future(future, deadline):
                self.get_logger().error(
                    f"detach safe-stop hold release timeout: {unit}"
                )

    def _trigger_actuator(
        self, sequence: int, actuator: str, released: str
    ) -> tuple[bool, str]:
        client = self.detach_clients[actuator]
        if not client.service_is_ready():
            return False, f"detach actuator service unavailable: {actuator}"
        request = TriggerDetach.Request()
        request.request_sequence = sequence
        request.released_unit_id = released
        future = client.call_async(request)
        deadline = time.monotonic() + float(
            self.get_parameter("actuator_ack_timeout_s").value
        )
        if not self._wait_future(future, deadline):
            return False, f"detach command ACK timeout: {actuator}"
        try:
            result = future.result()
        except Exception as exc:
            return False, f"detach actuator service error: {exc}"
        if result is None or not result.accepted or not result.command_completed:
            detail = result.message if result is not None else "no response"
            return False, f"detach command did not complete: {detail}"
        return True, result.message

    @staticmethod
    def _wait_future(future: object, deadline: float) -> bool:
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()

    def _publish_chain(self) -> None:
        ordered, active, relay = self.registry.snapshot()
        msg = ActiveChain()
        msg.stamp = self.get_clock().now().to_msg()
        msg.ordered_units = ordered
        msg.active_drive_units = active
        msg.relay_assumed_units = relay
        msg.software_estimate = True
        self.chain_publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DetachCoordinator()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
