from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    trans_communication_share = get_package_share_directory('trans_communication')
    platform_sim_tools_share = get_package_share_directory('platform_sim_tools')

    # Include trans_communication's bringup launch
    trans_communication_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(trans_communication_share, 'launch', 'bringup.launch.py')
        )
    )

    # Override the config_path parameter for the trans_communication node
    # In ROS 2, we use a Node with remappings/parameters to override
    override_config = Node(
        package='trans_communication',
        executable='trans_communication',
        name='trans_communication',
        parameters=[{
            'config_path': os.path.join(
                platform_sim_tools_share, 'config', 'trans_communication_config.yml'),
        }],
    )

    return LaunchDescription([
        trans_communication_bringup,
        override_config,
    ])
