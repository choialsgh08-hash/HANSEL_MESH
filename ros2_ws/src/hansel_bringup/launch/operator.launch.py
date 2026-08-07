"""Launch operator routing, adapters, logging, camera and optional RQT."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    path = os.path.join(
        get_package_share_directory("hansel_bringup"),
        "config",
        "robot_chain.yaml",
    )
    with open(path, encoding="utf-8") as stream:
        chain = yaml.safe_load(stream)["robot_chain"]
    ordered = chain["ordered_units"]
    parameters = {
        "ordered_units": ordered,
        "roles": [f"{unit}={chain['roles'][unit]}" for unit in ordered],
        "initial_active_drive_units": chain["initial_active_drive_units"],
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rqt", default_value="true"),
            DeclareLaunchArgument("use_camera_receiver", default_value="true"),
            DeclareLaunchArgument("use_network_adapter", default_value="true"),
            DeclareLaunchArgument("use_radar_adapter", default_value="false"),
            DeclareLaunchArgument("network_udp_port", default_value="7100"),
            DeclareLaunchArgument("radar_mission_log", default_value=""),
            Node(
                package="hansel_operator",
                executable="command_router",
                parameters=[parameters],
                output="screen",
            ),
            Node(
                package="hansel_operator",
                executable="detach_coordinator",
                parameters=[parameters],
                output="screen",
            ),
            Node(
                package="hansel_operator",
                executable="event_logger",
                parameters=[{"units": ordered}],
                output="screen",
            ),
            Node(
                package="hansel_camera_bridge",
                executable="camera_receiver",
                condition=IfCondition(LaunchConfiguration("use_camera_receiver")),
                output="screen",
            ),
            Node(
                package="hansel_network_adapter",
                executable="network_adapter",
                parameters=[
                    {
                        "units": ["base", *ordered],
                        "provider_mode": "hansel_mesh_udp",
                        "udp_bind_port": ParameterValue(
                            LaunchConfiguration("network_udp_port"), value_type=int
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_network_adapter")),
                output="screen",
            ),
            Node(
                package="hansel_radar_adapter",
                executable="radar_adapter",
                parameters=[
                    {
                        "provider_mode": "mission_log",
                        "mission_log_path": LaunchConfiguration("radar_mission_log"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_radar_adapter")),
                output="screen",
            ),
            Node(
                package="rqt_gui",
                executable="rqt_gui",
                arguments=["--standalone", "hansel_operator/HanselPanel"],
                condition=IfCondition(LaunchConfiguration("use_rqt")),
                output="screen",
            ),
        ]
    )
