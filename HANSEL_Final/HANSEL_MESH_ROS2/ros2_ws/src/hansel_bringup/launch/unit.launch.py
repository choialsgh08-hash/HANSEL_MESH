"""Launch Head/Nano or rear/GPIO without changing the Nano launch defaults."""
from __future__ import annotations
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _unit_node(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    unit_id, role, backend = value('unit_id'), value('role'), value('hardware_backend')
    if backend not in ('dummy','nano_serial','rpi_gpio'):
        raise ValueError('hardware_backend must be dummy, nano_serial or rpi_gpio')
    if backend == 'rpi_gpio' and role != 'rear':
        raise ValueError('rpi_gpio requires role:=rear; head uses nano_serial')
    files = [value('params_file')]
    overrides = {
        'unit_id': unit_id, 'role': role, 'hardware_backend': backend,
        'nano_serial_port': value('nano_serial_port'),
    }
    if backend == 'rpi_gpio':
        files.append(value('gpio_params_file'))
    maximum = value('maximum_pwm_percent')
    if maximum:
        overrides['maximum_pwm_percent'] = float(maximum)
    return [Node(
        package='hansel_unit_control', executable='unit_controller',
        namespace='/hansel/' + unit_id, name='unit_controller', output='screen',
        parameters=files + [overrides],
    )]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('hansel_bringup'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('unit_id', default_value='head'),
        DeclareLaunchArgument('maximum_pwm_percent', default_value=''),
        DeclareLaunchArgument('role', default_value='head'),
        DeclareLaunchArgument('hardware_backend', default_value='nano_serial'),
        DeclareLaunchArgument('nano_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(config,'unit_runtime.yaml')),
        DeclareLaunchArgument('gpio_params_file', default_value=os.path.join(config,'node_gpio.yaml')),
        OpaqueFunction(function=_unit_node),
    ])
