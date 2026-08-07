"""Launch the complete GPIO-free HANSEL graph for one-PC validation."""

from __future__ import annotations

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package: str, relative_path: str) -> dict:
    path = os.path.join(get_package_share_directory(package), relative_path)
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _rviz_actions(context):
    enabled = LaunchConfiguration("use_rviz").perform(context).lower() in {"1", "true", "yes", "on"}
    if not enabled:
        return []
    bringup_share = get_package_share_directory("hansel_bringup")
    description_share = get_package_share_directory("hansel_description")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(description_share, "urdf", "hansel_head.urdf.xacro")]),
        value_type=str,
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="hansel_description",
            executable="commanded_joint_state_publisher",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(bringup_share, "rviz", "hansel.rviz")],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    chain = _load_yaml("hansel_bringup", "config/robot_chain.yaml")["robot_chain"]
    unit_config = _load_yaml("hansel_bringup", "config/unit_defaults.yaml")
    ordered = chain["ordered_units"]
    roles = chain["roles"]
    active = chain["initial_active_drive_units"]
    shared_chain_parameters = {
        "ordered_units": ordered,
        "roles": [f"{unit}={roles[unit]}" for unit in ordered],
        "initial_active_drive_units": active,
    }

    actions = [
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_rqt", default_value="true"),
        DeclareLaunchArgument("use_camera_receiver", default_value="false"),
        DeclareLaunchArgument("use_dummy_camera", default_value="true"),
        DeclareLaunchArgument("use_adapter_stubs", default_value="false"),
        Node(package="hansel_operator", executable="command_router", name="command_router",
             output="screen", parameters=[shared_chain_parameters]),
        Node(package="hansel_operator", executable="detach_coordinator", name="detach_coordinator",
             output="screen", parameters=[shared_chain_parameters]),
        Node(package="hansel_operator", executable="event_logger", name="event_logger",
             output="screen", parameters=[{"units": ordered}]),
    ]

    for unit in ordered:
        parameters = dict(unit_config["defaults"])
        parameters.update(unit_config["per_unit"].get(unit, {}))
        parameters.update({"unit_id": unit, "role": roles[unit], "hardware_backend": "dummy"})
        actions.append(Node(
            package="hansel_unit_control", executable="unit_controller",
            namespace=f"/hansel/{unit}", name="unit_controller",
            output="screen", parameters=[parameters],
        ))

    actions.extend([
        Node(package="hansel_camera_bridge", executable="camera_receiver", name="camera_receiver",
             output="screen", condition=IfCondition(LaunchConfiguration("use_camera_receiver"))),
        Node(package="hansel_camera_bridge", executable="dummy_camera_publisher",
             name="dummy_camera_publisher", output="screen",
             condition=IfCondition(LaunchConfiguration("use_dummy_camera"))),
        Node(package="hansel_network_adapter", executable="network_adapter", name="network_adapter",
             output="screen", parameters=[{"units": ["base", *ordered], "provider_mode": "hansel_mesh_udp"}],
             condition=IfCondition(LaunchConfiguration("use_adapter_stubs"))),
        Node(package="hansel_radar_adapter", executable="radar_adapter", name="radar_adapter",
             output="screen", parameters=[{"provider_mode": "none"}],
             condition=IfCondition(LaunchConfiguration("use_adapter_stubs"))),
        Node(package="hansel_survivor_adapter", executable="survivor_adapter", name="survivor_adapter",
             output="screen", parameters=[{"units": ordered}],
             condition=IfCondition(LaunchConfiguration("use_adapter_stubs"))),
        Node(package="rqt_gui", executable="rqt_gui",
             arguments=["--standalone", "hansel_operator/HanselPanel"],
             condition=IfCondition(LaunchConfiguration("use_rqt")), output="screen"),
        OpaqueFunction(function=_rviz_actions),
    ])
    return LaunchDescription(actions)
