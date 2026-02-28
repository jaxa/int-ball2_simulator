from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ib2_gazebo_share = get_package_share_directory('ib2_gazebo')
    ctl_only_share = get_package_share_directory('ctl_only')

    sim_yaml = os.path.join(ib2_gazebo_share, 'sim', 'sim.yaml')
    custom_yaml = os.path.join(ib2_gazebo_share, 'sim', 'custom.yaml')
    ctl_yaml = os.path.join(ctl_only_share, 'config', 'ctl.yaml')

    return LaunchDescription([
        Node(
            package='ctl_only',
            executable='ctl_only',
            name='ctl_only',
            parameters=[sim_yaml, custom_yaml, ctl_yaml, {'use_sim_time': True}],
            output='screen',
        ),
    ])
