"""Publish a static JPEG test card for one-PC RQT verification."""

from __future__ import annotations

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from hansel_interfaces.msg import CameraReceiveStatus


class DummyCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("dummy_camera_publisher")
        self.declare_parameter("publish_rate_hz", 5.0)
        asset = Path(get_package_share_directory("hansel_camera_bridge")) / "test_assets" / "test_pattern.jpg"
        self._jpeg = asset.read_bytes()
        self._image_pub = self.create_publisher(
            CompressedImage, "/hansel/camera/image/compressed", 2
        )
        self._status_pub = self.create_publisher(
            CameraReceiveStatus, "/hansel/camera/receive_status", 10
        )
        rate = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f"publishing camera test pattern from {asset}")

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        image = CompressedImage()
        image.header.stamp = now
        image.header.frame_id = "dummy_camera"
        image.format = "jpeg"
        image.data = self._jpeg
        self._image_pub.publish(image)

        status = CameraReceiveStatus()
        status.stamp = now
        status.receiving = True
        status.receive_fps = float(self.get_parameter("publish_rate_hz").value)
        status.loss_rate = 0.0
        status.bitrate_bps = int(len(self._jpeg) * 8 * status.receive_fps)
        status.last_frame_stamp = now
        status.total_packets = 0
        status.lost_packets = 0
        status.source = "dummy_camera_publisher"
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
