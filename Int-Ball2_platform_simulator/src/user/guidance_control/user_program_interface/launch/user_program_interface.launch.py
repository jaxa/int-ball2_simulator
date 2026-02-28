"""Launch file for user_program_interface node."""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():
    # Platform launch parameters (control which nodes to start/stop)
    platform_params = GroupAction(
        actions=[
            PushRosNamespace('platform_launch'),
            SetParameter('sensor_fusion', True),
            SetParameter('slam_wrapper', True),
            SetParameter('ctl_only', False),
            SetParameter('fsm', True),
            SetParameter('camera_left', False),
            SetParameter('camera_right', False),
        ]
    )

    user_program_interface_node = Node(
        package='user_program_interface',
        executable='user_program_interface',
        name='user_program_interface',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        platform_params,
        user_program_interface_node,
    ])
