from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    platform_manager_share = get_package_share_directory('platform_manager')
    config_file = os.path.join(platform_manager_share, 'config', 'config_for_simulator.yml')

    # Declare all launch arguments
    declared_args = [
        DeclareLaunchArgument('color_with_camera_mic', default_value='[0.0, 0.0, 1.0]'),
        DeclareLaunchArgument('container_ros_master_uri', default_value='http://localhost:11311'),
        DeclareLaunchArgument('default_time_to_go_secs_long', default_value='10.0'),
        DeclareLaunchArgument('default_time_to_go_secs_short', default_value='3.0'),
        DeclareLaunchArgument('enable_shutdown', default_value='false'),
        DeclareLaunchArgument('host_ib2_workspace',
                              default_value='/home/nvidia/IB2/Int-Ball2_platform_simulator'),
        DeclareLaunchArgument('multipliers_for_action_cancellation_time_calculation',
                              default_value='2'),
        DeclareLaunchArgument('rate', default_value='1'),
        DeclareLaunchArgument('required_battery_remain', default_value='20'),
        DeclareLaunchArgument('required_storage_ratio', default_value='10.0'),
        DeclareLaunchArgument('run_on_simulator', default_value='true'),
        DeclareLaunchArgument('shutdown_battery_remain', default_value='5'),
        DeclareLaunchArgument('shutdown_storage_ratio', default_value='5.0'),
        DeclareLaunchArgument('temperature_to_cool', default_value='60.0'),
        DeclareLaunchArgument('temperature_to_revive', default_value='50.0'),
        DeclareLaunchArgument('temperature_to_shutdown', default_value='70.0'),
        DeclareLaunchArgument('user_container_name', default_value='ib2_user'),
        DeclareLaunchArgument('waiting_time_for_server', default_value='30.0'),
        DeclareLaunchArgument('waiting_time_for_topic', default_value='3.0'),
        DeclareLaunchArgument('wifi_duration', default_value='5.0'),
    ]

    # Node reads config_for_simulator.yml internally via yaml.safe_load,
    # so do NOT pass it as a --params-file (it has non-ROS2-parameter YAML structure).
    platform_manager_node = Node(
        package='platform_manager',
        executable='platform_manager',
        name='platform_manager',
        output='screen',
        parameters=[
            {
                'color_with_camera_mic': LaunchConfiguration('color_with_camera_mic'),
                'container_ros_master_uri': LaunchConfiguration('container_ros_master_uri'),
                'default_time_to_go_secs_long': LaunchConfiguration('default_time_to_go_secs_long'),
                'default_time_to_go_secs_short': LaunchConfiguration('default_time_to_go_secs_short'),
                'enable_shutdown': LaunchConfiguration('enable_shutdown'),
                'host_ib2_workspace': LaunchConfiguration('host_ib2_workspace'),
                'multipliers_for_action_cancellation_time_calculation':
                    LaunchConfiguration('multipliers_for_action_cancellation_time_calculation'),
                'rate': LaunchConfiguration('rate'),
                'run_on_simulator': LaunchConfiguration('run_on_simulator'),
                'required_battery_remain': LaunchConfiguration('required_battery_remain'),
                'required_storage_ratio': LaunchConfiguration('required_storage_ratio'),
                'shutdown_battery_remain': LaunchConfiguration('shutdown_battery_remain'),
                'shutdown_storage_ratio': LaunchConfiguration('shutdown_storage_ratio'),
                'temperature_to_cool': LaunchConfiguration('temperature_to_cool'),
                'temperature_to_revive': LaunchConfiguration('temperature_to_revive'),
                'temperature_to_shutdown': LaunchConfiguration('temperature_to_shutdown'),
                'user_container_name': LaunchConfiguration('user_container_name'),
                'waiting_time_for_server': LaunchConfiguration('waiting_time_for_server'),
                'waiting_time_for_topic': LaunchConfiguration('waiting_time_for_topic'),
                'wifi_duration': LaunchConfiguration('wifi_duration'),
            },
        ],
    )

    return LaunchDescription(declared_args + [platform_manager_node])
