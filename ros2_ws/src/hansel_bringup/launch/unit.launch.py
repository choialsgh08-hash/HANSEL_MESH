"""Launch one Head/Rear unit; pass a device-specific ROS parameter file."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("hansel_bringup"),
        "config",
        "unit_runtime.yaml",
    )
    unit_id = LaunchConfiguration("unit_id")
    return LaunchDescription(
        [
            DeclareLaunchArgument("unit_id", default_value="head"),
            DeclareLaunchArgument("role", default_value="head"),
            DeclareLaunchArgument("hardware_backend", default_value="nano_serial"),
            DeclareLaunchArgument("nano_serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="hansel_unit_control",
                executable="unit_controller",
                namespace=["/hansel/", unit_id],
                name="unit_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "unit_id": unit_id,
                        "role": LaunchConfiguration("role"),
                        "hardware_backend": LaunchConfiguration("hardware_backend"),
                        "nano_serial_port": LaunchConfiguration("nano_serial_port"),
                    },
                ],
            ),
        ]
    )

