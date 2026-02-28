#!/usr/bin/python3
# -*- coding: utf-8 -*-
# license removed for brevity
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('trans_communication')
    default_config_path = os.path.join(package_share, 'config', 'config.yml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to the config YAML file'
        ),
        DeclareLaunchArgument(
            'ocs_host',
            default_value='localhost',
            description='OCS host address'
        ),
        DeclareLaunchArgument(
            'ocs_port',
            default_value='34567',
            description='OCS port'
        ),
        DeclareLaunchArgument(
            'receive_port',
            default_value='23456',
            description='Receive port for TCP telecommands'
        ),
        DeclareLaunchArgument(
            'lex_min_user_data_bytes',
            default_value='68',
            description='Minimum user data bytes for LEX'
        ),
        DeclareLaunchArgument(
            'telemetry_rate',
            default_value='1',
            description='Telemetry send rate in Hz'
        ),
        DeclareLaunchArgument(
            'wait_after_send_error',
            default_value='3',
            description='Wait time in seconds after send error'
        ),
        DeclareLaunchArgument(
            'ros_wait_for_service_time',
            default_value='10',
            description='Timeout in seconds for waiting for ROS services'
        ),
        DeclareLaunchArgument(
            'telemetry_send_port_default_index',
            default_value='0',
            description='Default telemetry send port index'
        ),
        DeclareLaunchArgument(
            'original_telemetry_message_size_threshold',
            default_value='1300',
            description='Telemetry data size threshold per packet before pickle'
        ),
        Node(
            package='trans_communication',
            executable='trans_communication_node',
            name='trans_communication',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'ocs_host': LaunchConfiguration('ocs_host'),
                'ocs_port': LaunchConfiguration('ocs_port'),
                'receive_port': LaunchConfiguration('receive_port'),
                'lex_min_user_data_bytes': LaunchConfiguration('lex_min_user_data_bytes'),
                'telemetry_rate': LaunchConfiguration('telemetry_rate'),
                'wait_after_send_error': LaunchConfiguration('wait_after_send_error'),
                'ros_wait_for_service_time': LaunchConfiguration('ros_wait_for_service_time'),
                'telemetry_send_port': [49100, 49101],
                'telemetry_send_port_default_index': LaunchConfiguration('telemetry_send_port_default_index'),
                'original_telemetry_message_size_threshold': LaunchConfiguration(
                    'original_telemetry_message_size_threshold'),
            }],
        ),
    ])
