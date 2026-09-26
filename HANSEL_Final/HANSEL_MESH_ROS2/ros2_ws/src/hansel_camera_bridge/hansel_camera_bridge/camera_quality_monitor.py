"""Receive H.264/RTP UDP packets and publish operator-side quality status."""

from __future__ import annotations

import socket
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.node import Node

from hansel_interfaces.msg import CameraReceiveStatus

from .rtp_metrics import RtpMetricsTracker


class CameraQualityMonitor(Node):
    def __init__(self) -> None:
        super().__init__("camera_quality_monitor")
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("receive_timeout_s", 1.5)
        self.declare_parameter("source", "head_camera_rtp")
        self._tracker = RtpMetricsTracker(
            receive_timeout_s=float(
                self.get_parameter("receive_timeout_s").value
            )
        )
        self._lock = threading.Lock()
        self._running = True
        self._bind_error = ""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.settimeout(0.2)
        try:
            self._socket.bind(
                (
                    str(self.get_parameter("bind_host").value),
                    int(self.get_parameter("port").value),
                )
            )
        except OSError as exc:
            self._bind_error = str(exc)
            self.get_logger().error(f"camera RTP bind failed: {exc}")

        self.status_publisher = self.create_publisher(
            CameraReceiveStatus, "/hansel/camera/receive_status", 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_timer(
            float(self.get_parameter("publish_period_s").value),
            self._publish_status,
        )
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        if not self._bind_error:
            self._thread.start()

    def _receive_loop(self) -> None:
        while self._running:
            try:
                packet, _peer = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                with self._lock:
                    self._tracker.observe(packet, time.monotonic())
            except ValueError as exc:
                self.get_logger().warning(f"invalid RTP packet: {exc}")

    def _publish_status(self) -> None:
        now_mono = time.monotonic()
        with self._lock:
            snapshot = self._tracker.snapshot(now_mono)
        now_ros = self.get_clock().now()
        msg = CameraReceiveStatus()
        msg.stamp = now_ros.to_msg()
        msg.receiving = snapshot.receiving and not self._bind_error
        msg.receive_fps = float(snapshot.receive_fps)
        msg.loss_rate = float(snapshot.loss_rate)
        msg.bitrate_bps = int(snapshot.bitrate_bps)
        if snapshot.last_frame_monotonic is not None:
            age = max(0.0, now_mono - snapshot.last_frame_monotonic)
            msg.last_frame_stamp = (now_ros - Duration(seconds=age)).to_msg()
        msg.total_packets = snapshot.total_packets
        msg.lost_packets = snapshot.lost_packets
        msg.source = str(self.get_parameter("source").value)
        self.status_publisher.publish(msg)

        array = DiagnosticArray()
        array.header.stamp = now_ros.to_msg()
        status = DiagnosticStatus()
        status.name = "hansel/camera/receive"
        status.hardware_id = "operator_pc"
        if self._bind_error:
            status.level = DiagnosticStatus.ERROR
            status.message = f"UDP bind failed: {self._bind_error}"
        elif msg.receiving:
            status.level = DiagnosticStatus.OK
            status.message = "receiving RTP"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "RTP receive timeout"
        status.values = [
            KeyValue(key="receive_fps", value=f"{msg.receive_fps:.3f}"),
            KeyValue(key="loss_rate", value=f"{msg.loss_rate:.6f}"),
            KeyValue(key="bitrate_bps", value=str(msg.bitrate_bps)),
        ]
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    def destroy_node(self) -> bool:
        self._running = False
        self._socket.close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraQualityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
