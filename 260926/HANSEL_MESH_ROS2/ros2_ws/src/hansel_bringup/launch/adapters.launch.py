"""Launch HANSEL_MESH-compatible built-in adapters or external plugins."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("network_mode", default_value="hansel_mesh_udp"),
            DeclareLaunchArgument("network_provider", default_value=""),
            DeclareLaunchArgument("network_udp_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("network_udp_port", default_value="7100"),
            DeclareLaunchArgument("radar_mode", default_value="none"),
            DeclareLaunchArgument("radar_provider", default_value=""),
            DeclareLaunchArgument("radar_mission_log", default_value=""),
            DeclareLaunchArgument("radar_start_at_end", default_value="true"),
            DeclareLaunchArgument("survivor_provider", default_value=""),
            Node(
                package="hansel_network_adapter",
                executable="network_adapter",
                parameters=[
                    {
                        "provider_mode": LaunchConfiguration("network_mode"),
                        "provider_plugin": LaunchConfiguration("network_provider"),
                        "udp_bind_host": LaunchConfiguration("network_udp_host"),
                        "udp_bind_port": ParameterValue(
                            LaunchConfiguration("network_udp_port"), value_type=int
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="hansel_radar_adapter",
                executable="radar_adapter",
                parameters=[
                    {
                        "provider_mode": LaunchConfiguration("radar_mode"),
                        "provider_plugin": LaunchConfiguration("radar_provider"),
                        "mission_log_path": LaunchConfiguration("radar_mission_log"),
                        "mission_log_start_at_end": ParameterValue(
                            LaunchConfiguration("radar_start_at_end"), value_type=bool
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="hansel_survivor_adapter",
                executable="survivor_adapter",
                parameters=[
                    {"provider_plugin": LaunchConfiguration("survivor_provider")}
                ],
                output="screen",
            ),
        ]
    )
