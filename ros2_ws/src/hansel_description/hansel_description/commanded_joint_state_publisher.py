"""Publish the commanded (not measured) front assembly joint angle."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from hansel_interfaces.msg import HeadAngleState


class CommandedJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("commanded_joint_state_publisher")
        self.declare_parameter("joint_name", "front_tilt_joint")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            HeadAngleState,
            "/hansel/head/state/front_angle",
            self._on_angle,
            10,
        )

    def _on_angle(self, msg: HeadAngleState) -> None:
        joint = JointState()
        joint.header.stamp = msg.stamp
        joint.name = [str(self.get_parameter("joint_name").value)]
        joint.position = [math.radians(msg.commanded_angle_deg)]
        self.publisher.publish(joint)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CommandedJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

