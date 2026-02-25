from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='user_template',
            executable='user_template_node',
            name='user_template',
            output='screen',
            parameters=[{
                'custom_parameter_integer': 1,
                'custom_parameter_float': 2.0,
                'custom_parameter_string': 'custom',
                'custom_parameter_boolean': True,
            }],
        ),
    ])
