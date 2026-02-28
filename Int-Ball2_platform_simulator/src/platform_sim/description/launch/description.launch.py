"""Launch file for Int-Ball2 description: robot_state_publishers + RViz2."""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_urdf_for_rviz(urdf_path: str, package_share: str) -> str:
    """Load URDF and convert model:// URIs to package:// URIs for RViz.

    Equivalent to the original load_gazebo_urdf_for_rviz.sh script.
    """
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    # Convert model:// references to package://description/model/ for RViz
    urdf_content = re.sub(
        r'model://',
        'package://description/model/',
        urdf_content,
    )
    return urdf_content


def generate_launch_description():
    pkg_share = get_package_share_directory('description')

    # Load URDFs
    ib2_urdf = load_urdf_for_rviz(
        os.path.join(pkg_share, 'model', 'ib2', 'ib2.urdf'),
        pkg_share,
    )
    custom_object_01_urdf = load_urdf_for_rviz(
        os.path.join(pkg_share, 'model', 'custom_object_01', 'custom_object_01.urdf'),
        pkg_share,
    )
    iss_urdf_path = os.path.join(pkg_share, 'urdf', 'iss.urdf')
    with open(iss_urdf_path, 'r') as f:
        iss_urdf = f.read()

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'urdf.rviz')],
    )

    # robot_state_publisher for Int-Ball2
    ib2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ib2_state_publisher',
        parameters=[{
            'robot_description': ib2_urdf,
            'publish_frequency': 50.0,
        }],
        remappings=[
            ('robot_description', 'ib2_description'),
        ],
    )

    # robot_state_publisher for custom_object_01
    custom_object_01_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='custom_object_01_state_publisher',
        parameters=[{
            'robot_description': custom_object_01_urdf,
            'publish_frequency': 50.0,
        }],
        remappings=[
            ('robot_description', 'custom_object_01_description'),
        ],
    )

    # robot_state_publisher for ISS
    iss_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='iss_state_publisher',
        parameters=[{
            'robot_description': iss_urdf,
            'publish_frequency': 50.0,
        }],
        remappings=[
            ('robot_description', 'iss_description'),
        ],
    )

    # trans nodes for moving models (TF publishing)
    # NOTE: The 'trans' package must also be ported to ROS 2.
    trans_ib2 = Node(
        package='trans',
        executable='trans',
        name='trans_ib2',
        output='screen',
        parameters=[{
            'model_name': 'ib2',
            'world_frame': 'base',
            'base_frame': 'body',
            'updateFreqHz': 50,
        }],
    )

    trans_iss = Node(
        package='trans',
        executable='trans',
        name='trans_iss',
        output='screen',
        parameters=[{
            'model_name': 'iss',
            'world_frame': 'base',
            'base_frame': 'iss_body',
            'updateFreqHz': 50,
        }],
    )

    trans_custom_object_01 = Node(
        package='trans',
        executable='trans',
        name='trans_custom_object_01',
        output='screen',
        parameters=[{
            'model_name': 'custom_object_01',
            'world_frame': 'base',
            'base_frame': 'custom_object_01_body',
            'updateFreqHz': 50,
        }],
    )

    return LaunchDescription([
        rviz_node,
        ib2_state_publisher,
        custom_object_01_state_publisher,
        iss_state_publisher,
        trans_ib2,
        trans_iss,
        trans_custom_object_01,
    ])
