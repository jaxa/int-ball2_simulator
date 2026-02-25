from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ctl_only_share = get_package_share_directory('ctl_only')
    ctl_yaml = os.path.join(ctl_only_share, 'config', 'ctl.yaml')

    return LaunchDescription([
        Node(
            package='fsm',
            executable='fsm',
            name='fsm',
            parameters=[ctl_yaml],
            output='screen',
        ),
    ])
