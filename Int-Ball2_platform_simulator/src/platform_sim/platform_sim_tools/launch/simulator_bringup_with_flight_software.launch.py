from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ib2_gazebo_share = get_package_share_directory('ib2_gazebo')
    platform_sim_tools_share = get_package_share_directory('platform_sim_tools')
    platform_monitor_share = get_package_share_directory('platform_monitor')
    trans_communication_share = get_package_share_directory('trans_communication')

    # Declare launch arguments
    ocs_host_arg = DeclareLaunchArgument('ocs_host', default_value='localhost')
    ocs_port_arg = DeclareLaunchArgument('ocs_port', default_value='34567')

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ib2_gazebo_share, 'launch', 'sim.launch.py')
        )
    )

    # Launch platform_manager
    platform_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(platform_sim_tools_share, 'launch',
                         'platform_manager_bringup.launch.py')
        )
    )

    # Launch platform_monitor
    platform_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(platform_monitor_share, 'launch', 'bringup.launch.py')
        )
    )

    # Launch trans_communication with simulator config overrides
    trans_communication_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(platform_sim_tools_share, 'launch',
                         'trans_communication_bringup.launch.py')
        )
    )

    # Override trans_communication parameters via a separate node parameter set
    # In ROS 2, namespace-scoped parameter overrides are done through the node itself
    trans_communication_overrides = Node(
        package='trans_communication',
        executable='trans_communication',
        name='trans_communication',
        namespace='',
        parameters=[{
            'ocs_host': LaunchConfiguration('ocs_host'),
            'ocs_port': LaunchConfiguration('ocs_port'),
            # 49100 and 49101 are also used in normal operations
            'telemetry_send_port': [49100, 49101, 49200, 49201],
        }],
    )

    # Add trans_communication script directory to PYTHONPATH
    trans_communication_script_dir = os.path.join(trans_communication_share, 'script')
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=current_pythonpath + ':' + trans_communication_script_dir
        if current_pythonpath else trans_communication_script_dir
    )

    # Launch sim_minimal_telemetry_publisher node
    sim_minimal_telemetry_publisher_node = Node(
        package='platform_sim_tools',
        executable='sim_minimal_telemetry_publisher',
        name='sim_minimal_telemetry_publisher',
        output='screen',
        parameters=[{
            'transcommunication_config': os.path.join(
                platform_sim_tools_share, 'config', 'trans_communication_config.yml'),
            'ocs_host': LaunchConfiguration('ocs_host'),
            'ocs_port': LaunchConfiguration('ocs_port'),
        }],
    )

    return LaunchDescription([
        ocs_host_arg,
        ocs_port_arg,
        set_pythonpath,
        gazebo_launch,
        platform_manager_launch,
        platform_monitor_launch,
        trans_communication_launch,
        trans_communication_overrides,
        sim_minimal_telemetry_publisher_node,
    ])
