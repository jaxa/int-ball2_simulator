from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    # Launch Gazebo simulation (use_flight_sw=true by default, so ctl_only/fsm won't launch)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ib2_gazebo_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    # Launch platform_manager
    platform_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(platform_sim_tools_share, 'launch',
                         'platform_manager_bringup.launch.py')
        )
    )

    # Launch platform_monitor (explicitly set rate to '1.0' to avoid type conflict
    # with platform_manager's rate which is declared as integer)
    platform_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(platform_monitor_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={'rate': '1.0'}.items()
    )

    # Launch trans_communication with simulator config
    trans_communication_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(platform_sim_tools_share, 'launch',
                         'trans_communication_bringup.launch.py')
        ),
        launch_arguments={
            'ocs_host': LaunchConfiguration('ocs_host'),
            'ocs_port': LaunchConfiguration('ocs_port'),
        }.items()
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
        gui_arg,
        rviz_arg,
        set_pythonpath,
        gazebo_launch,
        platform_manager_launch,
        platform_monitor_launch,
        trans_communication_launch,
        sim_minimal_telemetry_publisher_node,
    ])
