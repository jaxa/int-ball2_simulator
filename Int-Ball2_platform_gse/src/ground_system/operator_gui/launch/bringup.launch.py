"""Launch file for operator_gui with communication_software."""

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
    operator_gui_share = get_package_share_directory('operator_gui')

    comm_sw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(comm_sw_share, 'launch', 'bringup.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('gui_only')),
    )

    # Static transform publisher: base -> iss_body
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_broadcaster',
        arguments=['0', '1', '0.75', '1.5708', '0', '0', 'base', 'iss_body'],
    )

    operator_gui_node = Node(
        package='operator_gui',
        executable='operator_gui',
        name='operator_gui',
        output='screen',
    )

    return LaunchDescription([
        gui_only_arg,
        comm_sw_launch,
        static_tf_node,
        operator_gui_node,
    ])
