from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():
    platform_launch_params = GroupAction([
        PushRosNamespace('platform_launch'),
        # If value set to false, the target nodes will be terminated when user logic is started.
        SetParameter(name='sensor_fusion', value=True),
        SetParameter(name='slam_wrapper', value=False),
        SetParameter(name='ctl_only', value=False),
        SetParameter(name='fsm', value=False),
        # If you want to start up camera_left and camera_right,
        # you need to stop (set false) slam_wrapper.
        SetParameter(name='camera_left', value=True),
        SetParameter(name='camera_right', value=True),
    ])

    test_node = Node(
        package='sample_tests',
        executable='test_visual_slam',
        name='test_visual_slam',
        output='screen',
        # The parameters to be used in the user's program can be set
    )

    return LaunchDescription([
        platform_launch_params,
        test_node,
    ])
