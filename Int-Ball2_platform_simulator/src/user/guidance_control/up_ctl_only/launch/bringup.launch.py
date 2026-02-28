"""Launch file for up_ctl_only with user_program_interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ib2_gazebo_share = get_package_share_directory('ib2_gazebo')
    up_ctl_only_share = get_package_share_directory('up_ctl_only')
    user_program_interface_share = get_package_share_directory(
        'user_program_interface')

    sim_yaml = os.path.join(ib2_gazebo_share, 'sim', 'sim.yaml')
    custom_yaml = os.path.join(ib2_gazebo_share, 'sim', 'custom.yaml')
    ctl_yaml = os.path.join(up_ctl_only_share, 'config', 'ctl.yaml')

    up_ctl_only_node = Node(
        package='up_ctl_only',
        executable='up_ctl_only',
        name='up_ctl_only',
        parameters=[sim_yaml, custom_yaml, ctl_yaml, {'use_sim_time': True}],
        output='screen',
    )

    user_program_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(user_program_interface_share,
                         'launch', 'user_program_interface.launch.py')
        ),
    )

    return LaunchDescription([
        up_ctl_only_node,
        user_program_interface_launch,
    ])
