"""Launch file for Int-Ball2 Gazebo Harmonic simulation.

Replaces the original sim.launch (ROS 1 / Gazebo Classic).
Launches:
  1. Gazebo Harmonic (gz sim) with the ISS world
  2. ros_gz_bridge for sensor/pose topics
  3. ISS model spawning
  4. Propulsion node
  5. Control nodes (optional)
  6. Description / RViz (optional)
  7. Parameter loading (sim.yaml, custom.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    ib2_gazebo_share = get_package_share_directory('ib2_gazebo')
    description_share = get_package_share_directory('description')

    # Launch arguments
    use_platform_arg = DeclareLaunchArgument(
        'use_platform', default_value='true')
    use_flight_sw_arg = DeclareLaunchArgument(
        'use_flight_sw', default_value='true')
    use_ctl_only_arg = DeclareLaunchArgument(
        'use_ctl_only', default_value='true')
    use_fsm_arg = DeclareLaunchArgument(
        'use_fsm', default_value='true')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true')

    # World SDF file path
    world_file = os.path.join(ib2_gazebo_share, 'worlds', 'empty.sdf')

    # Set GZ_SIM_RESOURCE_PATH so gz-sim can find model:// URIs
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(description_share, 'model'),
    )

    # --- Gazebo Harmonic (gz sim) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # --- ros_gz_bridge: bridge sensor and pose topics ---
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            # Camera topics
            '/camera_main/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_main/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Model poses (for trans nodes)
            '/model/ib2/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/model/iss/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/model/custom_object_01/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # --- Spawn ISS model ---
    iss_urdf_path = os.path.join(description_share, 'urdf', 'iss.urdf')
    spawn_iss = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_iss',
        output='screen',
        arguments=[
            '-file', iss_urdf_path,
            '-name', 'iss',
            '-x', '0', '-y', '0', '-z', '0',
            '-R', '0', '-P', '0', '-Y', '0',
        ],
    )

    # --- Load simulation parameters ---
    sim_yaml = os.path.join(ib2_gazebo_share, 'sim', 'sim.yaml')
    custom_yaml = os.path.join(ib2_gazebo_share, 'sim', 'custom.yaml')

    # --- Propulsion node ---
    # NOTE: The 'prop' package must be ported to ROS 2 first.
    prop_node = Node(
        package='prop',
        executable='prop',
        name='prop',
        parameters=[sim_yaml, custom_yaml],
    )

    # --- Control nodes (conditional) ---
    # NOTE: ctl_only and fsm packages must be ported to ROS 2 first.
    ctl_only_node = Node(
        package='ctl_only',
        executable='ctl_only',
        name='ctl_only',
        parameters=[sim_yaml, custom_yaml],
        condition=IfCondition(LaunchConfiguration('use_ctl_only')),
    )

    fsm_node = Node(
        package='fsm',
        executable='fsm',
        name='fsm',
        parameters=[sim_yaml, custom_yaml],
        condition=IfCondition(LaunchConfiguration('use_fsm')),
    )

    platform_control_group = GroupAction(
        actions=[ctl_only_node, fsm_node],
        condition=UnlessCondition(LaunchConfiguration('use_flight_sw')),
    )

    # --- Description / RViz launch ---
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'description.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # Arguments
        use_platform_arg,
        use_flight_sw_arg,
        use_ctl_only_arg,
        use_fsm_arg,
        gui_arg,
        rviz_arg,
        # Environment
        gz_resource_path,
        # Gazebo
        gz_sim,
        bridge_node,
        spawn_iss,
        # Nodes
        prop_node,
        platform_control_group,
        # Visualization
        description_launch,
    ])
