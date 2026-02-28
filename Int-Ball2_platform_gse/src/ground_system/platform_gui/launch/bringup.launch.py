"""Launch file for platform_gui with communication_software."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gui_only_arg = DeclareLaunchArgument(
        'gui_only', default_value='false',
        description='If true, skip launching communication_software')

    comm_sw_share = get_package_share_directory('communication_software')
    comm_sw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(comm_sw_share, 'launch', 'bringup.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('gui_only')),
    )

    platform_gui_node = Node(
        package='platform_gui',
        executable='platform_gui',
        name='platform_gui',
        output='screen',
    )

    return LaunchDescription([
        gui_only_arg,
        comm_sw_launch,
        platform_gui_node,
    ])
