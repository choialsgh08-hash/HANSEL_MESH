"""One-way Head-local CDR datagrams; no localization DDS on the mesh."""
from __future__ import annotations

import fcntl
import os
from pathlib import Path
import socket
import struct
import time


def default_socket():
    return f"/tmp/hansel-inputs-{os.getuid()}.sock"


class LocalInputs:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = self.path.with_suffix(".lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise RuntimeError(f"Local odometry input already owned: {path}")
        self.path.unlink(missing_ok=True)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            self.socket.bind(str(self.path))
            os.chmod(self.path, 0o600)
            self.socket.setblocking(False)
        except OSError:
            self.socket.close()
            self.lock.close()
            raise

    def receive(self):
        # Bounded work per timer; timestamps use the same host monotonic clock.
        result = []
        for _ in range(100):
            try:
                data = self.socket.recv(65536)
            except BlockingIOError:
                break
            if len(data) < 9 or data[:1] not in (b"W", b"A", b"D", b"E"):
                continue
            age = time.monotonic() - struct.unpack("!d", data[1:9])[0]
            if not 0 <= age < 0.3:
                continue
            result.append((data[:1], data[9:]))
        return result

    def close(self):
        self.socket.close()
        self.path.unlink(missing_ok=True)
        self.lock.close()


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from hansel_interfaces.msg import WheelState, ActiveChain, DetachEvent
    from diagnostic_msgs.msg import DiagnosticArray

    rclpy.init()
    node = Node("head_local_inputs")
    transport = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    transport.setblocking(False)
    path = os.environ.get("HANSEL_INPUT_SOCKET", default_socket())

    def send(kind, message):
        payload = kind + struct.pack("!d", time.monotonic()) + serialize_message(message)
        try:
            transport.sendto(payload, path)
        except (FileNotFoundError, ConnectionRefusedError, BlockingIOError):
            pass  # The local consumer may be restarting; never queue old wheels.

    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(WheelState, "/hansel/head/state/wheels", lambda m: send(b"W", m), 1)
    def on_diagnostics(message):
        message.status = [s for s in message.status if s.name == "hansel/head/unit_controller"]
        if message.status:
            send(b"D", message)
    node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 1)
    active = [None]
    def on_chain(message):
        active[0] = message
        send(b"A", message)
    node.create_subscription(ActiveChain, "/hansel/system/state/active_chain", on_chain, retained)
    node.create_timer(1.0, lambda: send(b"A", active[0]) if active[0] is not None else None)
    events = {}
    def on_detach(message):
        events[message.released_unit_id] = message
        send(b"E", message)
    node.create_subscription(DetachEvent, "/hansel/detach/events", on_detach,
                             QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def repeat_events():
        for event in events.values():
            send(b"E", event)
    node.create_timer(1.0, repeat_events)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
