from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('platform_monitor')
    default_config = os.path.join(pkg_share, 'config', 'config.yml')

    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='1'),
        DeclareLaunchArgument('config_path', default_value=default_config),

        Node(
            package='platform_monitor',
            executable='platform_monitor',
            name='platform_monitor',
            output='screen',
            parameters=[{
                'rate': LaunchConfiguration('rate'),
                'config_path': LaunchConfiguration('config_path'),
            }],
        ),
    ])
