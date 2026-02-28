from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    trans_communication_share = get_package_share_directory('trans_communication')
    platform_sim_tools_share = get_package_share_directory('platform_sim_tools')

    sim_config_path = os.path.join(
        platform_sim_tools_share, 'config', 'trans_communication_config.yml')

    # Include trans_communication's bringup launch with simulator config
    trans_communication_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(trans_communication_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'config_path': sim_config_path,
        }.items()
    )

    return LaunchDescription([
        trans_communication_bringup,
    ])
