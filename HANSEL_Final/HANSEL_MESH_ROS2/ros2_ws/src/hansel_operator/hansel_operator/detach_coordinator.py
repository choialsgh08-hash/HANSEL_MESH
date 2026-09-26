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

from hansel_interfaces.msg import ActiveChain, DetachEvent, DetachMode, DetachRecommendation
from hansel_interfaces.srv import DetachUnit, PrepareDetach, TriggerDetach

from .chain_registry import ChainRegistry, roles_from_entries


class DetachCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("detach_coordinator")
        self._callbacks = ReentrantCallbackGroup()
        self.declare_parameter("ordered_units", ["head", "node1", "node2"])
        self.declare_parameter(
            "roles", ["head=head", "node1=rear", "node2=rear"]
        )
        self.declare_parameter(
            "initial_active_drive_units", ["head", "node1", "node2"]
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
        self._release_lock = threading.Lock()
        self._pending_releases = {}
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
        self.event_publisher = self.create_publisher(
            DetachEvent, "/hansel/detach/events",
            QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
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
        self._retry_stop_releases()

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
        uncertain_tail = []
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
            actuation_stamp = self.get_clock().now().to_msg()
            completed, message, dispatched = self._trigger_actuator(
                sequence, actuator, released
            )
            if dispatched:
                uncertain_tail = self.registry.released_tail(released)
            if not completed:
                response.command_completed = False
                response.message = message
                return response

            for unit in uncertain_tail:
                self.registry.mark_relay_assumed(unit)
            self._publish_chain()
            for unit in uncertain_tail:
                event = DetachEvent()
                event.stamp = actuation_stamp
                event.request_sequence = sequence
                event.actuator_unit_id = actuator
                event.released_unit_id = unit
                self.event_publisher.publish(event)
            uncertain_tail = []
            response.command_completed = True
            response.message = (
                f"detach command completed for {released} by {actuator}; "
                "physical separation not asserted"
            )
            return response
        finally:
            try:
                if safe_stop_hold_placed:
                    self._release_stop_holds(
                        sequence,
                        released,
                        [u for u in self.registry.active_drive_units if u not in uncertain_tail],
                    )
                if uncertain_tail:
                    self.get_logger().error(
                        f"detach completion uncertain; holding {uncertain_tail} stopped. "
                        "Inspect coupling/servo before retrying the detach command."
                    )
            finally:
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
            request = PrepareDetach.Request()
            request.request_sequence = sequence
            request.released_unit_id = released
            request.hold = False
            with self._release_lock:
                self._pending_releases[unit] = (request, None, 0.0)
            if not client.service_is_ready():
                continue
            try:
                futures.append((unit, client.call_async(request)))
            except Exception as exc:
                self.get_logger().error(f"hold release send failed; retrying {unit}: {exc}")
        deadline = time.monotonic() + float(
            self.get_parameter("stop_ack_timeout_s").value
        )
        for unit, future in futures:
            if self._wait_future(future, deadline) and self._release_succeeded(future):
                with self._release_lock:
                    pending = self._pending_releases.get(unit)
                    if pending and pending[0].request_sequence == sequence:
                        self._pending_releases.pop(unit)
            else:
                self.get_logger().error(
                    f"detach hold release pending; retrying: {unit}"
                )

    @staticmethod
    def _release_succeeded(future):
        try:
            result = future.result()
            return result is not None and result.stopped
        except Exception:
            return False

    def _retry_stop_releases(self):
        with self._release_lock:
            for unit, (request, future, deadline) in list(self._pending_releases.items()):
                if future is not None:
                    if not future.done() and time.monotonic() < deadline:
                        continue
                    if future.done() and self._release_succeeded(future):
                        self._pending_releases.pop(unit)
                        continue
                    if not future.done():
                        future.cancel()
                client = self.stop_clients[unit]
                if client.service_is_ready():
                    try:
                        self._pending_releases[unit] = (
                            request, client.call_async(request),
                            time.monotonic() + float(self.get_parameter("stop_ack_timeout_s").value))
                    except Exception as exc:
                        self.get_logger().error(f"hold release retry failed for {unit}: {exc}")

    def _trigger_actuator(
        self, sequence: int, actuator: str, released: str
    ) -> tuple[bool, str, bool]:
        client = self.detach_clients[actuator]
        if not client.service_is_ready():
            return False, f"detach actuator service unavailable: {actuator}", False
        request = TriggerDetach.Request()
        request.request_sequence = sequence
        request.released_unit_id = released
        try:
            future = client.call_async(request)
        except Exception as exc:
            return False, f"detach request dispatch unconfirmed: {exc}", True
        deadline = time.monotonic() + float(
            self.get_parameter("actuator_ack_timeout_s").value
        )
        if not self._wait_future(future, deadline):
            return False, f"detach command ACK timeout: {actuator}", True
        try:
            result = future.result()
        except Exception as exc:
            return False, f"detach actuator service error: {exc}", True
        if result is None or not result.accepted or not result.command_completed:
            detail = result.message if result is not None else "no response"
            return False, f"detach command did not complete: {detail}", result is None or result.accepted
        return True, result.message, True

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
