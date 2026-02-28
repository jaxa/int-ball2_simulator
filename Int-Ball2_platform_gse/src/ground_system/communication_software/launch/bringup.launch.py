"""Launch file for communication_software nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('communication_software')
    params_yaml = os.path.join(pkg_share, 'config', 'params.yml')
    config_yaml = os.path.join(pkg_share, 'config', 'config.yml')

    telecommand_bridge_node = Node(
        package='communication_software',
        executable='telecommand_bridge.py',
        name='telecommand_bridge',
        output='screen',
        parameters=[params_yaml, {'communication_config_path': config_yaml}],
    )

    telemetry_bridge_node = Node(
        package='communication_software',
        executable='telemetry_bridge.py',
        name='telemetry_bridge',
        output='screen',
        parameters=[params_yaml, {'communication_config_path': config_yaml}],
    )

    return LaunchDescription([
        telecommand_bridge_node,
        telemetry_bridge_node,
    ])
