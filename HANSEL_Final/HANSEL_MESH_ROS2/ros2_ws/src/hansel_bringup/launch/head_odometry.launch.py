"""Launch provisional BNO085/rear-encoder odometry on Head."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _odometry_node(context):
    parameters = [LaunchConfiguration("params_file").perform(context)]
    extra = LaunchConfiguration("imu_params_file").perform(context)
    if extra:
        parameters.append(extra)
    return [Node(
        package="hansel_localization",
        executable="head_odometry",
        name="head_odometry",
        parameters=parameters,
        output="screen",
    )]


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("hansel_bringup"),
        "config",
        "head_odometry.yaml",
    )
    rviz_config = os.path.join(
        get_package_share_directory("hansel_bringup"),
        "rviz",
        "head_odometry.rviz",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("imu_params_file", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            OpaqueFunction(function=_odometry_node),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )
