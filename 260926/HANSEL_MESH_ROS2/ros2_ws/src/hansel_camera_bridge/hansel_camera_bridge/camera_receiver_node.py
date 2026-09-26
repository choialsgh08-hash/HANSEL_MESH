"""GStreamer H.264/RTP receiver with local JPEG preview and quality status."""

from __future__ import annotations

import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from hansel_interfaces.msg import CameraReceiveStatus

from .rtp_metrics import RtpMetricsTracker


class CameraReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_receiver")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter("payload_type", 96)
        self.declare_parameter("latency_ms", 100)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("receive_timeout_s", 1.5)
        self.declare_parameter("frame_id", "camera_link")

        self.image_publisher = self.create_publisher(
            CompressedImage, "/hansel/camera/image/compressed", 2
        )
        self.status_publisher = self.create_publisher(
            CameraReceiveStatus, "/hansel/camera/receive_status", 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._tracker = RtpMetricsTracker(
            float(self.get_parameter("receive_timeout_s").value)
        )
        self._lock = threading.Lock()
        self._gst = None
        self._pipeline = None
        self._startup_error = ""
        self._start_pipeline()
        self.create_timer(
            float(self.get_parameter("publish_period_s").value),
            self._publish_status,
        )
        self.create_timer(0.25, self._poll_bus)

    def _start_pipeline(self) -> None:
        try:
            import gi  # type: ignore

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst  # type: ignore

            Gst.init(None)
            self._gst = Gst
            address = str(self.get_parameter("bind_address").value)
            port = int(self.get_parameter("port").value)
            payload_type = int(self.get_parameter("payload_type").value)
            latency = int(self.get_parameter("latency_ms").value)
            quality = int(self.get_parameter("jpeg_quality").value)
            pipeline_text = (
                f'udpsrc address="{address}" port={port} '
                f'caps="application/x-rtp,media=video,encoding-name=H264,'
                f'payload={payload_type}" ! tee name=t '
                "t. ! queue leaky=downstream max-size-buffers=100 ! "
                "appsink name=rtp_sink emit-signals=true drop=true "
                "max-buffers=100 sync=false "
                "t. ! queue ! "
                f"rtpjitterbuffer latency={latency} drop-on-latency=true ! "
                "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                f"jpegenc quality={quality} ! "
                "appsink name=video_sink emit-signals=true drop=true "
                "max-buffers=1 sync=false"
            )
            self._pipeline = Gst.parse_launch(pipeline_text)
            self._pipeline.get_by_name("rtp_sink").connect(
                "new-sample", self._on_rtp_sample
            )
            self._pipeline.get_by_name("video_sink").connect(
                "new-sample", self._on_video_sample
            )
            result = self._pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("GStreamer pipeline failed to enter PLAYING")
        except Exception as exc:
            self._startup_error = str(exc)
            self.get_logger().error(f"camera receiver unavailable: {exc}")

    def _sample_bytes(self, sink) -> bytes:
        sample = sink.emit("pull-sample")
        if sample is None:
            return b""
        buffer = sample.get_buffer()
        success, mapping = buffer.map(self._gst.MapFlags.READ)
        if not success:
            return b""
        try:
            return bytes(mapping.data)
        finally:
            buffer.unmap(mapping)

    def _on_rtp_sample(self, sink):
        packet = self._sample_bytes(sink)
        if packet:
            try:
                with self._lock:
                    self._tracker.observe(packet, time.monotonic())
            except ValueError as exc:
                self.get_logger().warning(f"invalid RTP packet: {exc}")
        return self._gst.FlowReturn.OK

    def _on_video_sample(self, sink):
        jpeg = self._sample_bytes(sink)
        if jpeg:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(self.get_parameter("frame_id").value)
            msg.format = "jpeg"
            msg.data = jpeg
            self.image_publisher.publish(msg)
        return self._gst.FlowReturn.OK

    def _poll_bus(self) -> None:
        if self._pipeline is None or self._gst is None:
            return
        bus = self._pipeline.get_bus()
        while True:
            message = bus.pop_filtered(
                self._gst.MessageType.ERROR | self._gst.MessageType.WARNING
            )
            if message is None:
                break
            if message.type == self._gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self._startup_error = f"{error}: {debug}"
                self.get_logger().error(self._startup_error)
            else:
                warning, debug = message.parse_warning()
                self.get_logger().warning(f"{warning}: {debug}")

    def _publish_status(self) -> None:
        now_mono = time.monotonic()
        with self._lock:
            snapshot = self._tracker.snapshot(now_mono)
        now_ros = self.get_clock().now()
        msg = CameraReceiveStatus()
        msg.stamp = now_ros.to_msg()
        msg.receiving = snapshot.receiving and not self._startup_error
        msg.receive_fps = float(snapshot.receive_fps)
        msg.loss_rate = float(snapshot.loss_rate)
        msg.bitrate_bps = int(snapshot.bitrate_bps)
        if snapshot.last_frame_monotonic is not None:
            age = max(0.0, now_mono - snapshot.last_frame_monotonic)
            msg.last_frame_stamp = (now_ros - Duration(seconds=age)).to_msg()
        msg.total_packets = snapshot.total_packets
        msg.lost_packets = snapshot.lost_packets
        msg.source = "gstreamer_h264_rtp_receiver"
        self.status_publisher.publish(msg)

        array = DiagnosticArray()
        array.header.stamp = now_ros.to_msg()
        status = DiagnosticStatus()
        status.name = "hansel/camera/receiver"
        status.hardware_id = "operator_pc"
        if self._startup_error:
            status.level = DiagnosticStatus.ERROR
            status.message = self._startup_error
        elif msg.receiving:
            status.level = DiagnosticStatus.OK
            status.message = "receiving and decoding H.264/RTP"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "H.264/RTP receive timeout"
        status.values = [
            KeyValue(key="receive_fps", value=f"{msg.receive_fps:.3f}"),
            KeyValue(key="loss_rate", value=f"{msg.loss_rate:.6f}"),
            KeyValue(key="bitrate_bps", value=str(msg.bitrate_bps)),
        ]
        array.status = [status]
        self.diagnostics_publisher.publish(array)

    def destroy_node(self) -> bool:
        if self._pipeline is not None and self._gst is not None:
            self._pipeline.set_state(self._gst.State.NULL)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
