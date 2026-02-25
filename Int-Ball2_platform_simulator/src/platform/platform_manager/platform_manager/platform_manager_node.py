#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import docker
import subprocess
import os
import signal
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from datetime import datetime
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA, Empty, Float64MultiArray
from builtin_interfaces.msg import Time as TimeMsg
from ib2_msgs.msg import (
    BatteryChargeInfo,
    CtlStatus,
    CtlStatusType,
    LEDColors,
    Navigation,
    PowerStatus,
    SystemStatus,
)
from ib2_msgs.action import (
    CtlCommand,
    NavigationStartUp,
)
from ib2_msgs.srv import (
    SwitchPower,
)
from platform_msgs.msg import (
    ManagerStatus,
    Mode,
    OperationType,
    UserLogic,
)
from platform_msgs.srv import (
    SetOperationType,
    StopProcessingUserNode,
    UserLogicCommand,
    UserNodeCommand,
)

from ament_index_python.packages import get_package_share_directory


# Build name lookup dicts from message constants.
# In ROS 2 message constants are class-level attributes; iterate through the
# class dict and keep entries whose name is ALL_UPPER.
def _build_name_map(msg_cls):
    return {value: name for name, value in vars(msg_cls).items()
            if name.isupper() and isinstance(value, int)}


MODE_NAMES = _build_name_map(Mode)
OPERATION_TYPE_NAMES = _build_name_map(OperationType)
CTL_RESULT_NAMES = _build_name_map(CtlCommand.Result)
GOAL_STATUS_NAMES = _build_name_map(GoalStatus)
CTL_STATUS_NAMES = _build_name_map(CtlStatusType)


class ClosableFlag(object):

    def __init__(self):
        self.flag = False

    def __enter__(self):
        self.flag = True

    def __exit__(self, exception_type, exception_value, traceback):
        self.flag = False


class PlatformManager(Node):
    """Platform Manager for Int-Ball2."""
    PLATFORM_LAUNCH_PREFIX = '/platform_launch/'
    NODE_ID = 'platform_manager'

    def __init__(self):
        super().__init__(PlatformManager.NODE_ID)
        self.get_logger().debug('PlatformManager.__init__ in')

        self.__battery_remain = None
        self.__container = None
        self.__default_stop_launch_names = []
        self.__docker_client = docker.from_env()
        self.__is_battery_low = False
        self.__is_container_shutdown = ClosableFlag()
        self.__is_cooling = False
        self.__is_short_disk_space = False
        self.__is_wifi_disconnected = False
        self.__last_ctl_command_cancel_execution_time = None
        self.__last_ctl_command_id = None
        self.__last_image = None
        self.__last_launch = None
        self.__last_user = None
        self.__last_user_logic = None
        self.__last_wifi_connected_time = self.get_clock().now()
        self.__mode = None
        self.__operation_type = None
        self.__post_command = {}
        self.__pre_command = {}
        self.__prev_system_status = None
        # In ROS 2 we manage launch processes via subprocess.Popen
        self.__launch_processes = {}  # name -> subprocess.Popen or None
        self.__current_launch_config_params = None
        self.__launch_parameter = {}
        self.__system_status = None
        self.__use_ctl = False
        self.__use_nav = False

        # Track last goal handle for ctl action
        self.__ctl_goal_handle = None
        self.__ctl_goal_id = None

        # ------------------------------------------------------------------
        # Declare and read parameters
        # ------------------------------------------------------------------
        self.declare_parameter('color_with_camera_mic', [0.0, 0.0, 1.0])
        self.declare_parameter('container_ros_master_uri', 'http://172.20.10.10:11311')
        self.declare_parameter('default_time_to_go_secs_long', 10.0)
        self.declare_parameter('default_time_to_go_secs_short', 3.0)
        self.declare_parameter('enable_shutdown', True)
        self.declare_parameter('host_ib2_workspace', '/home/sec/Int-Ball2_platform_simulator')
        self.declare_parameter('multipliers_for_action_cancellation_time_calculation', 2)
        self.declare_parameter('rate', 1)
        self.declare_parameter('required_battery_remain', 20)
        self.declare_parameter('required_storage_ratio', 10.0)
        self.declare_parameter('run_on_simulator', False)
        self.declare_parameter('shutdown_battery_remain', 5)
        self.declare_parameter('shutdown_storage_ratio', 5.0)
        self.declare_parameter('temperature_to_cool', 80.0)
        self.declare_parameter('temperature_to_revive', 70.0)
        self.declare_parameter('temperature_to_shutdown', 90.0)
        self.declare_parameter('user_container_name', 'ib2_user')
        self.declare_parameter('waiting_time_for_server', 10.0)
        self.declare_parameter('waiting_time_for_topic', 3.0)
        self.declare_parameter('wifi_duration', 5.0)
        self.declare_parameter('bringup_packages', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('acceptable_packages', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('non_simultaneous_package_info', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('bringup_camera_main', False)

        color_list = self.get_parameter('color_with_camera_mic').get_parameter_value().double_array_value
        if not color_list:
            color_list = [0.0, 0.0, 1.0]
        if len(color_list) != 3:
            self.get_logger().error(
                'len(color_with_camera_mic) should be 3. now {}. return black color(non-color).'
                .format(len(color_list)))
            self.get_logger().debug('PlatformManager.__init__ out')
            raise ValueError('Invalid color_with_camera_mic length')
        self.__color_with_camera_mic = ColorRGBA(
            r=float(color_list[0]), g=float(color_list[1]), b=float(color_list[2]), a=1.0)

        self.__container_ros_master_uri = self.get_parameter(
            'container_ros_master_uri').get_parameter_value().string_value
        self.__default_time_to_go_secs_long = self.get_parameter(
            'default_time_to_go_secs_long').get_parameter_value().double_value
        self.__default_time_to_go_secs_short = self.get_parameter(
            'default_time_to_go_secs_short').get_parameter_value().double_value
        self.__enable_shutdown = self.get_parameter(
            'enable_shutdown').get_parameter_value().bool_value
        self.__host_ib2_workspace = self.get_parameter(
            'host_ib2_workspace').get_parameter_value().string_value
        self.__multipliers_for_action_cancellation_time_calculation = self.get_parameter(
            'multipliers_for_action_cancellation_time_calculation').get_parameter_value().integer_value
        rate_hz = self.get_parameter('rate').get_parameter_value().integer_value
        if rate_hz <= 0:
            rate_hz = 1
        self.__rate_period = 1.0 / rate_hz
        self.__required_battery_remain = self.get_parameter(
            'required_battery_remain').get_parameter_value().integer_value
        self.__required_storage_ratio = self.get_parameter(
            'required_storage_ratio').get_parameter_value().double_value
        self.__run_on_simulator = self.get_parameter(
            'run_on_simulator').get_parameter_value().bool_value
        self.__shutdown_battery_remain = self.get_parameter(
            'shutdown_battery_remain').get_parameter_value().integer_value
        self.__shutdown_storage_ratio = self.get_parameter(
            'shutdown_storage_ratio').get_parameter_value().double_value
        self.__temperature_to_cool = self.get_parameter(
            'temperature_to_cool').get_parameter_value().double_value
        self.__temperature_to_revive = self.get_parameter(
            'temperature_to_revive').get_parameter_value().double_value
        self.__temperature_to_shutdown = self.get_parameter(
            'temperature_to_shutdown').get_parameter_value().double_value
        self.__user_container_name = self.get_parameter(
            'user_container_name').get_parameter_value().string_value
        self.__waiting_time_for_server = self.get_parameter(
            'waiting_time_for_server').get_parameter_value().double_value
        self.__waiting_time_for_topic = self.get_parameter(
            'waiting_time_for_topic').get_parameter_value().double_value
        self.__wifi_duration = Duration(
            seconds=self.get_parameter('wifi_duration').get_parameter_value().double_value)

        # ------------------------------------------------------------------
        # Load bringup_packages from config YAML (loaded externally via
        # platform_sim_tools or parameter file).
        # Since bringup_packages is a complex nested list of dicts, we load
        # it directly from the YAML config file rather than via ROS params.
        # ------------------------------------------------------------------
        config_path = os.path.join(
            get_package_share_directory('platform_manager'), 'config')
        # Determine which config to use: simulator or default
        if self.__run_on_simulator:
            config_file = os.path.join(config_path, 'config_for_simulator.yml')
            if not os.path.exists(config_file):
                config_file = os.path.join(config_path, 'config.yml')
        else:
            config_file = os.path.join(config_path, 'config.yml')

        with open(config_file) as f:
            config_yaml = yaml.safe_load(f)

        self.__bringup_packages = config_yaml.get('bringup_packages', [])
        bringup_package_names = [p['name'] for p in self.__bringup_packages]
        self.get_logger().info('Bringup package names: {}'.format(bringup_package_names))

        self.__acceptable_packages = config_yaml.get('acceptable_packages', [])
        if not self.__run_on_simulator and self.__acceptable_packages:
            self.get_logger().error('acceptable_packages can be set only when run_on_simulator is true')
            self.get_logger().debug('PlatformManager.__init__ out')
            raise ValueError('acceptable_packages set when not on simulator')
        self.get_logger().info(
            'Acceptable package names '
            '(If these packages are specified in user\'s launch file, it is not an error, '
            'but the node will not be activated.): {}'.format(self.__acceptable_packages))
        check_packages = set(bringup_package_names) & set(self.__acceptable_packages)
        if len(check_packages) > 0:
            self.get_logger().error(
                'The same package name cannot be included in '
                'both \'bringup_packages\' and \'acceptable_packages\'. : {}'
                .format(check_packages))
            self.get_logger().debug('PlatformManager.__init__ out')
            raise ValueError('Duplicate packages in bringup and acceptable')

        if 'sensor_fusion' in bringup_package_names:
            self.get_logger().info('Controls sensor_fusion node')
            self.__use_nav = True
        elif self.__run_on_simulator:
            self.get_logger().info('Controls navigation plugin')
            self.__use_nav = True
        else:
            self.get_logger().info('sensor_fusion node and navigation plugin are not controlled')
            self.__use_nav = False
        if 'ctl_only' in bringup_package_names or 'ctl' in bringup_package_names:
            self.get_logger().info('Controls ctl_only node and ctl node')
            self.__use_ctl = True
        else:
            self.get_logger().info('ctl_only node and ctl node are not controlled')
            self.__use_ctl = False

        self.__non_simultaneous_package_info = config_yaml.get('non_simultaneous_package_info', [])

        try:
            # Publisher
            self.__status_publisher = self.create_publisher(
                ManagerStatus, '~/status', 1)
            self.__ib2_user_start_publisher = self.create_publisher(
                UserLogic, '/ib2_user/start', 1)
            self.__led_color_publisher_left = self.create_publisher(
                LEDColors, '/led_display_left/led_colors', 1)
            self.__led_color_publisher_right = self.create_publisher(
                LEDColors, '/led_display_right/led_colors', 1)
            self.__fan_duty_publisher = self.create_publisher(
                Float64MultiArray, '/ctl/duty', 1)

            # Service server
            self.__set_operation_type_server = self.create_service(
                SetOperationType, '~/set_operation_type',
                self.__callback_for_set_operation_type)
            self.__user_node_server = self.create_service(
                UserNodeCommand, '~/user_node',
                self.__callback_for_user_node)
            self.__user_logic_server = self.create_service(
                UserLogicCommand, '~/user_logic',
                self.__callback_for_user_logic)

            # Action client
            self.__target_action_client = ActionClient(
                self, CtlCommand, '/ctl/command')
            self.__navigation_start_up_client = ActionClient(
                self, NavigationStartUp, '/sensor_fusion/navigation_start_up')

            # Subscribers
            self.__action_goal_subscriber = self.create_subscription(
                CtlCommand.Goal, '/trans_communication/action_goal',
                self.__execute_action_goal, 10)
            self.__reboot_subscriber = self.create_subscription(
                Empty, '/trans_communication/reboot',
                self.__reboot, 10)
            # In ROS 2, action feedback is received via action client callbacks,
            # but to match the original architecture (subscribing to feedback topic),
            # we subscribe to the action feedback topic directly.
            self.__target_action_feedback_subscriber = self.create_subscription(
                CtlCommand.Impl.FeedbackMessage, '/ctl/command/_action/feedback',
                self.__ctl_command_feedback, 10)

            # Service client
            self.__stop_processing_user_node = self.create_client(
                StopProcessingUserNode, '/ib2_user/stop')

            if not self.__run_on_simulator:
                self.__slam_wrapper_switch_power = self.create_client(
                    SwitchPower, '/slam_wrapper/switch_power')

            # Subscriber
            self.__system_status_subscriber = self.create_subscription(
                SystemStatus, '/system_monitor/status',
                self.__system_status_update, 10)
            self.__battery_charge_info_subscriber = self.create_subscription(
                BatteryChargeInfo, '/dock/battery_charge_info',
                self.__battery_charge_info_update, 10)
            self.__user_complete_subscriber = self.create_subscription(
                TimeMsg, '/ib2_user/complete',
                self.__user_complete, 10)
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().debug('PlatformManager.__init__ out')
            raise e

        for package_config in self.__bringup_packages:
            # package_config contains following values:
            # - name (*required)
            # - package (optional)
            # - launch_file (optional)
            # - pre_command (optional)
            # - post_command (optional)
            # - startup (optional)

            self.get_logger().info(str(package_config))
            name = package_config['name']
            if name in self.__acceptable_packages:
                self.get_logger().info(
                    '{} will not be started because it is defined within acceptable_packages'
                    .format(name))
                continue
            package = package_config.get('package', name)
            launch_file = package_config.get('launch_file', 'bringup.launch')
            startup = package_config.get('startup', True)
            self.__launch_parameter[name] = {
                'name': name,
                'package': package,
                'launch_file': launch_file,
                'startup': startup
            }
            if package_config.get('pre_command', None):
                self.__pre_command[name] = package_config['pre_command']
            if package_config.get('post_command', None):
                self.__post_command[name] = package_config['post_command']
            if not startup:
                self.__default_stop_launch_names.append(name)
            self.__setup_launch_process(name)
        self.__start_launch_process(startup=True)

        # ***** Provisional processing *****
        # Control camera_main node
        self.__bringup_camera_main = self.get_parameter(
            'bringup_camera_main').get_parameter_value().bool_value
        self.__camera_main_process = None
        if not self.__run_on_simulator and self.__bringup_camera_main:
            self.get_logger().info('Launch camera_main node')
            self.__camera_main_process = self.__launch_ros2(
                'gscam', 'bringup_main.launch.py')
        # **********************************

        self.__set_mode(Mode.USER_OFF)

        self.__set_operation_type(OperationType.NAV_OFF, raise_wait_error=True)

        # Create a periodic timer to replace the ROS 1 while-loop with rate.sleep()
        self.__main_timer = self.create_timer(self.__rate_period, self.__main_loop)

        self.get_logger().debug('PlatformManager.__init__ out')

    def destroy_node(self):
        self.__stop_and_remove_container(suppress_logs=True)
        self.__stop_launch_process(suppress_logs=True)
        if self.__camera_main_process is not None:
            self.__terminate_process(self.__camera_main_process)
        super().destroy_node()

    # ------------------------------------------------------------------
    # Launch process helpers (replacing roslaunch Python API)
    # ------------------------------------------------------------------
    def __launch_ros2(self, package, launch_file):
        """Launch a ROS 2 package via subprocess."""
        # Determine extension: .launch.py for ROS 2, or .launch for ROS 1 style
        # In the simulator context, launch files may have been ported to .launch.py
        # Try .launch.py first, fall back to .launch via ros2 launch
        cmd = ['ros2', 'launch', package, launch_file]
        self.get_logger().info('Launching: {}'.format(' '.join(cmd)))
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
            return proc
        except Exception as e:
            self.get_logger().error('Failed to launch {} {}: {}'.format(
                package, launch_file, e))
            return None

    def __terminate_process(self, proc):
        """Terminate a subprocess (and its process group) gracefully."""
        if proc is None:
            return
        try:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass

    def __is_process_alive(self, proc):
        """Check if a subprocess is still running."""
        if proc is None:
            return False
        return proc.poll() is None

    def __setup_launch_process(self, parameter_name):
        self.get_logger().debug('PlatformManager.__setup_launch_process in')
        name = self.__launch_parameter[parameter_name]['name']
        package = self.__launch_parameter[parameter_name]['package']
        launch_file_name = self.__launch_parameter[parameter_name]['launch_file']

        self.get_logger().info('Set up for execute launch: {} {}'.format(
            package, launch_file_name))
        # Initialize with None; actual subprocess is started in __start_launch_process
        self.__launch_processes[name] = None

        self.get_logger().debug('PlatformManager.__setup_launch_process out')

    def __get_current_launch_process_keys(self, *, started_only=False, stopped_only=False):
        self.get_logger().debug('PlatformManager.__get_current_launch_process_keys in')

        current_launch = [(key, self.__is_process_alive(self.__launch_processes[key]))
                          for key in self.__launch_processes.keys()]
        if started_only:
            current_launch = [c for c in current_launch if c[1] is True]
        if stopped_only:
            current_launch = [c for c in current_launch if c[1] is False]

        self.get_logger().debug('PlatformManager.__get_current_launch_process_keys out')
        return [c[0] for c in current_launch]

    def __start_launch_process(self, *, targets=(), startup=None):
        self.get_logger().debug('PlatformManager.__start_launch_process in')

        target_process_keys = set(self.__get_current_launch_process_keys(stopped_only=True))
        if targets:
            target_process_keys &= set(targets)

        if not target_process_keys:
            self.get_logger().info('There are no launch processes that can be started.')
            if targets:
                self.get_logger().info(
                    'The target nodes({}) was not running.'.format(targets))
            else:
                self.get_logger().info(
                    'There are no launch processes that can be stopped.')
        else:
            for process in target_process_keys:
                if startup is not None and self.__launch_parameter[process]['startup'] != startup:
                    self.get_logger().info(
                        'Because \'startup\' is not {},'
                        ' {} will be removed from startup targets.'.format(startup, process))
                    continue
                if process in self.__pre_command:
                    self.get_logger().info('Execute command: {}'.format(
                        self.__pre_command[process]))
                    cmd_return_list = subprocess.Popen(
                        self.__pre_command[process],
                        shell=True,
                        encoding='utf-8',
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        universal_newlines=True).stdout.readlines()
                    for cmd_return in cmd_return_list:
                        self.get_logger().info(cmd_return.rstrip("\n"))
                self.get_logger().info('{} will be started.'.format(process))
                # Terminate any existing process before relaunching
                if self.__launch_processes.get(process) is not None:
                    self.__terminate_process(self.__launch_processes[process])
                package = self.__launch_parameter[process]['package']
                launch_file = self.__launch_parameter[process]['launch_file']
                self.__launch_processes[process] = self.__launch_ros2(
                    package, launch_file)

        self.get_logger().debug('PlatformManager.__start_launch_process out')

    def __stop_launch_process(self, *, targets=(), suppress_logs=False):
        self._logdebug(suppress_logs, 'PlatformManager.__stop_launch_process in')

        target_process_keys = set(self.__get_current_launch_process_keys(started_only=True))
        if targets:
            target_process_keys &= set(targets)
        if 'sensor_fusion' in target_process_keys:
            self._loginfo(suppress_logs,
                          'Since sensor_fusion node will be terminated, '
                          'associated ctl_only node will also be stopped (as {}).'
                          .format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            self.__check_and_call_ctl_command_standby(suppress_logs=suppress_logs)

        if not target_process_keys:
            if targets:
                self._loginfo(suppress_logs,
                              'The target nodes({}) was not running.'.format(targets))
            else:
                self._loginfo(suppress_logs,
                              'There are no launch processes that can be stopped.')

        else:
            for process in target_process_keys:
                self.__terminate_process(self.__launch_processes[process])
                self.__launch_processes[process] = None
                self._loginfo(suppress_logs,
                              '{} has been stopped.'.format(process))

                if process in self.__post_command:
                    self._loginfo(suppress_logs,
                                  'Execute command: {}'.format(
                                      self.__post_command[process]))
                    cmd_return_list = subprocess.Popen(
                        self.__post_command[process],
                        shell=True,
                        encoding='utf-8',
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        universal_newlines=True).stdout.readlines()
                    for cmd_return in cmd_return_list:
                        self._loginfo(suppress_logs, cmd_return.rstrip("\n"))

        self._logdebug(suppress_logs, 'PlatformManager.__stop_launch_process out')

    # ------------------------------------------------------------------
    # Logging helpers (suppress-aware)
    # ------------------------------------------------------------------
    def _logerr(self, suppress, msg):
        if not suppress:
            self.get_logger().error(msg)

    def _loginfo(self, suppress, msg):
        if not suppress:
            self.get_logger().info(msg)

    def _logdebug(self, suppress, msg):
        if not suppress:
            self.get_logger().debug(msg)

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def __callback_for_set_operation_type(self, request, response):
        self.get_logger().debug('PlatformManager.__callback_for_set_operation_type in')

        if self.__mode == Mode.USER_IN_PROGRESS:
            self.get_logger().debug('PlatformManager.__callback_for_set_operation_type out')
            response.result = SetOperationType.Response.USER_LOGIC_IN_PROGRESS
            return response

        if request.type.type == self.__operation_type:
            self.get_logger().debug('PlatformManager.__callback_for_set_operation_type out')
            response.result = SetOperationType.Response.NO_CHANGE
            return response

        if request.type.type in [OperationType.NAV_OFF, OperationType.NAV_ON]:
            self.__set_operation_type(request.type.type)
            self.get_logger().debug('PlatformManager.__callback_for_set_operation_type out')
            response.result = SetOperationType.Response.SUCCESS
            return response

        self.get_logger().debug('PlatformManager.__callback_for_set_operation_type out')
        response.result = SetOperationType.Response.ERROR
        return response

    def __callback_for_user_node(self, request, response):
        self.get_logger().debug('PlatformManager.__callback_for_user_node in')

        def error_return(response_result):
            self.get_logger().debug('PlatformManager.__callback_for_user_node out')
            response.result = response_result
            return response

        if request.command == UserNodeCommand.Request.START:
            if (self.__is_battery_low or self.__is_cooling or
                    self.__is_short_disk_space or self.__is_wifi_disconnected):
                self.get_logger().error('When off-nominal, user node cannot be started')
                return error_return(UserNodeCommand.Response.OFF_NOMINAL)

            #
            # Validate request parameters.
            #

            if self.__mode != Mode.USER_OFF:
                return error_return(UserNodeCommand.Response.NODE_ALREADY_RUNNING)

            # In ROS 2 we check if the package exists using ament_index
            try:
                get_package_share_directory(request.user)
            except Exception:
                return error_return(UserNodeCommand.Response.USER_NOT_FOUND)

            # Check if the launch file exists within the package share dir
            try:
                pkg_share = get_package_share_directory(request.user)
                launch_path = os.path.join(pkg_share, 'launch', request.launch)
                if not os.path.exists(launch_path):
                    # Try alternate extensions
                    alt = launch_path + '.py'
                    if not os.path.exists(alt):
                        return error_return(UserNodeCommand.Response.LAUNCH_NOT_FOUND)
            except Exception:
                return error_return(UserNodeCommand.Response.LAUNCH_NOT_FOUND)

            try:
                self.__docker_client.images.get(request.image)
            except Exception:
                return error_return(UserNodeCommand.Response.IMAGE_NOT_FOUND)

            # Check the contents of the launch file for platform_launch params
            launch_config_params = self.__preprocess_user_launch_config(
                request.user, request.launch)
            if launch_config_params is None:
                return error_return(UserNodeCommand.Response.INVALID_LAUNCH)

            #
            # Start user node
            #
            if not self.__start_container(request.image, request.user, request.launch):
                return error_return(UserNodeCommand.Response.ERROR)
            # Keep launch_config_params for starting user logic
            self.__current_launch_config_params = launch_config_params
            self.__set_mode(Mode.USER_READY)

        elif request.command == UserNodeCommand.Request.STOP:
            #
            # Stop user node
            #
            self.__user_off_processing()

            if self.__mode == Mode.USER_IN_PROGRESS:
                self.__processing_after_user_logic_stop()

        else:
            return error_return(UserNodeCommand.Response.INVALID_COMMAND)

        self.get_logger().debug('PlatformManager.__callback_for_user_node out')
        response.result = UserNodeCommand.Response.SUCCESS
        return response

    def __processing_after_user_logic_stop(self):
        self.get_logger().debug('PlatformManager.__processing_after_user_logic_stop in')

        #
        # Control the nodes specified by the launch config.
        #

        # Stop (if necessary)
        if self.__default_stop_launch_names:
            self.__stop_launch_process(targets=set(self.__default_stop_launch_names))

        # ***** Provisional processing *****
        # Restart camera_main node
        if not self.__run_on_simulator and self.__bringup_camera_main:
            self.get_logger().info('Restart camera_main node')
            self.__terminate_process(self.__camera_main_process)
            self.__camera_main_process = self.__launch_ros2(
                'gscam', 'bringup_main.launch.py')
        # **********************************

        # Start
        self.__start_launch_process(startup=True)

        if self.__operation_type == OperationType.NAV_ON:
            self.__nav_on_processing()
        elif self.__operation_type == OperationType.NAV_OFF:
            self.__nav_off_processing()

        self.get_logger().debug('PlatformManager.__processing_after_user_logic_stop out')

    def __callback_for_user_logic(self, request, response):
        self.get_logger().debug('PlatformManager.__callback_for_user_logic in')

        def error_return(response_result):
            self.get_logger().debug('PlatformManager.__callback_for_user_logic out')
            response.result = response_result
            return response

        if request.command == UserLogicCommand.Request.START:
            if (self.__is_battery_low or self.__is_cooling or
                    self.__is_short_disk_space or self.__is_wifi_disconnected):
                self.get_logger().error('When off-nominal, user logic cannot be started')
                return error_return(UserLogicCommand.Response.OFF_NOMINAL)

            if self.__mode == Mode.USER_OFF:
                return error_return(UserLogicCommand.Response.NODE_NOT_STARTED)

            if self.__mode == Mode.USER_IN_PROGRESS:
                return error_return(UserLogicCommand.Response.LOGIC_ALREADY_RUNNING)

            #
            # Control the nodes specified by the launch config.
            # When transitioning to USER_READY mode, current_launch_config_params is set.
            #
            self.__launch_control_based_on_user_launch(
                self.__current_launch_config_params)

            #
            # Requests user node to start processing.
            #
            self.get_logger().info('Publish /ib2_user/start: {}'.format(request.logic))
            self.__ib2_user_start_publisher.publish(request.logic)
            self.__set_mode(Mode.USER_IN_PROGRESS)
            self.__last_user_logic = request.logic

        elif request.command == UserLogicCommand.Request.STOP:
            if self.__mode == Mode.USER_OFF:
                return error_return(UserLogicCommand.Response.NODE_NOT_STARTED)

            if self.__mode == Mode.USER_READY:
                return error_return(UserLogicCommand.Response.LOGIC_NOT_STARTED)

            #
            # Requests user node to stop the process.
            #
            self.get_logger().info('Call /ib2_user/stop')
            req = StopProcessingUserNode.Request()
            future = self.__stop_processing_user_node.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            if future.result() is not None:
                stop_processing_result = future.result().result
                if stop_processing_result == StopProcessingUserNode.Response.SUCCESS:
                    self.get_logger().info('User logic has stopped successfully.')
                    self.__processing_after_user_logic_stop()
                    self.__set_mode(Mode.USER_READY)
                else:
                    self.get_logger().error('Failed to stop user logic.')
                    return error_return(UserLogicCommand.Response.ERROR)
            else:
                self.get_logger().error('Failed to call /ib2_user/stop service.')
                return error_return(UserLogicCommand.Response.ERROR)

        else:
            return error_return(UserLogicCommand.Response.INVALID_COMMAND)

        self.get_logger().debug('PlatformManager.__callback_for_user_logic out')
        return error_return(UserLogicCommand.Response.SUCCESS)

    # ------------------------------------------------------------------
    # Launch config validation for user launch files
    # ------------------------------------------------------------------
    def __preprocess_user_launch_config(self, user, launch):
        """
        In ROS 2 we cannot easily introspect launch files programmatically
        like roslaunch.config.load_config_default did. Instead, we parse
        the user's launch file looking for platform_launch parameters.

        For the simulator, user launch files contain <group ns="platform_launch">
        with <param name="xxx" value="true/false"/> entries.  We parse these
        from the XML launch file directly.

        Returns a dict of {name: bool} or None if validation fails.
        """
        self.get_logger().debug('PlatformManager.__preprocess_user_launch_config in')

        try:
            pkg_share = get_package_share_directory(user)
            launch_path = os.path.join(pkg_share, 'launch', launch)
            if not os.path.exists(launch_path):
                alt = launch_path + '.py'
                if os.path.exists(alt):
                    launch_path = alt
                else:
                    self.get_logger().warning(
                        'Launch file not found: {}'.format(launch_path))
                    return None
        except Exception as e:
            self.get_logger().warning(
                'Invalid launch parameters ({}, {}). {}'.format(user, launch, e))
            self.get_logger().debug('PlatformManager.__preprocess_user_launch_config out')
            return None

        # Try to parse XML launch file for platform_launch group params
        platform_launch_params = {}
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(launch_path)
            root = tree.getroot()
            for group in root.findall('.//group'):
                ns = group.get('ns', '')
                if ns == 'platform_launch':
                    for param in group.findall('param'):
                        pname = param.get('name', '')
                        pvalue = param.get('value', 'false').lower()
                        platform_launch_params[pname] = (pvalue == 'true')
        except ET.ParseError:
            # Might be a Python launch file - try to extract params from it
            try:
                with open(launch_path, 'r') as f:
                    content = f.read()
                # Simple heuristic: look for platform_launch params in Python launch
                import re
                # Look for patterns like: ('platform_launch/name', 'true'|True)
                matches = re.findall(
                    r'platform_launch/(\w+).*?(?:value\s*=\s*["\']?(true|false)["\']?'
                    r'|(?:True|False))',
                    content, re.IGNORECASE)
                for name, value in matches:
                    platform_launch_params[name] = (value.lower() == 'true')
            except Exception:
                pass
        except Exception:
            pass

        # platform_launch_params must be present
        if not platform_launch_params:
            self.get_logger().warning(
                'Invalid launch file ({}). platform_launch parameters not found.'
                .format(launch_path))
            self.get_logger().debug('PlatformManager.__preprocess_user_launch_config out')
            return None

        # Check for duplicate settings - in dict form duplicates are impossible
        # (last one wins), but we check the count from parsing
        param_names = list(platform_launch_params.keys())
        if len(param_names) != len(set(param_names)):
            self.get_logger().warning(
                'Invalid launch file ({}). '
                'Within platform_launch, parameter name must be unique.'
                .format(launch_path))
            self.get_logger().debug('PlatformManager.__preprocess_user_launch_config out')
            return None

        # All package names must be in bringup_packages or acceptable_packages
        bringup_package_names = set([p['name'] for p in self.__bringup_packages])
        platform_launch_diff = set(param_names) - bringup_package_names
        if platform_launch_diff:
            acceptable_packages_diff = platform_launch_diff - set(self.__acceptable_packages)
            if acceptable_packages_diff:
                self.get_logger().warning(
                    'Invalid launch file ({}). {} cannot be the target of operations.'
                    .format(launch_path, acceptable_packages_diff))
                self.get_logger().debug('PlatformManager.__preprocess_user_launch_config out')
                return None

        # Check non_simultaneous_package_info
        platform_launch_params_true = [
            name for name, value in platform_launch_params.items() if value is True]
        for non_simultaneous_packages in self.__non_simultaneous_package_info:
            if (set(non_simultaneous_packages) & set(platform_launch_params_true) ==
                    set(non_simultaneous_packages)):
                self.get_logger().warning(
                    'Invalid launch file ({}). '
                    '{} cannot be started at the same time.'
                    .format(launch_path, non_simultaneous_packages))
                self.get_logger().debug('PlatformManager.__preprocess_user_launch_config out')
                return None

        self.get_logger().debug('PlatformManager.__preprocess_user_launch_config out')
        return platform_launch_params

    def __launch_control_based_on_user_launch(self, launch_config_params):
        """
        Control launch processes based on user launch config params.
        launch_config_params is a dict: {name: bool(start=True/stop=False)}
        """
        self.get_logger().debug('PlatformManager.__launch_control_based_on_user_launch in')

        if launch_config_params is None:
            self.get_logger().debug('PlatformManager.__launch_control_based_on_user_launch out')
            return

        stop_target = [name for name, start in launch_config_params.items() if not start]
        start_target = [name for name, start in launch_config_params.items() if start]

        # Stop
        self.get_logger().info('Nodes specified as the stop target in the launch file: {}'.format(
            ', '.join(stop_target) if stop_target else 'None'))
        current_started = set(stop_target) & set(
            self.__get_current_launch_process_keys(started_only=True))
        self.get_logger().info('Actual nodes to be stopped: {}'.format(
            ', '.join(current_started) if current_started else 'None'))
        if current_started:
            self.__stop_launch_process(targets=current_started)

        # Start
        self.get_logger().info('Nodes specified as the start target in the launch file: {}'.format(
            ', '.join(start_target) if start_target else 'None'))
        current_stopped = set(start_target) & set(
            self.__get_current_launch_process_keys(stopped_only=True))
        self.get_logger().info('Actual nodes to be started: {}'.format(
            ', '.join(current_stopped) if current_stopped else 'None'))
        if current_stopped:
            self.__start_launch_process(targets=current_stopped)

        # Processing for simulation
        if 'sensor_fusion' in stop_target and self.__run_on_simulator:
            self.__wait_action_server(self.__navigation_start_up_client)
            # make the navigation stand-by
            self.get_logger().info(
                'Stopping navigation node (or stop nav plugin function)...')
            navigation_request = NavigationStartUp.Goal()
            navigation_request.command = NavigationStartUp.Goal.OFF
            self.__send_goal_and_wait(
                self.__navigation_start_up_client, navigation_request)
            self.get_logger().info('Navigation node is successfully stopped.')

        self.get_logger().debug('PlatformManager.__launch_control_based_on_user_launch out')

    # ------------------------------------------------------------------
    # Container management (Docker - mostly pure Python, minimal changes)
    # ------------------------------------------------------------------
    def __start_container(self, image, user, launch):
        self.get_logger().debug('PlatformManager.__start_container in')

        if not self.__container:
            try:
                self.__container = self.__docker_client.containers.run(
                    image,
                    environment=[
                        'ROS_MASTER_URI={}'.format(self.__container_ros_master_uri),
                        'IB2_PACKAGE={}'.format(user),
                        'IB2_LAUNCH_FILE={}'.format(launch),
                        'IB2_WORKSPACE={}'.format(self.__host_ib2_workspace)
                    ],
                    mounts=[docker.types.Mount(
                        target=self.__host_ib2_workspace,
                        source=self.__host_ib2_workspace,
                        type='bind',
                        read_only=True)],
                    name=self.__user_container_name,
                    detach=True
                )
                self.__last_image = image
                self.__last_user = user
                self.__last_launch = launch
                self.get_logger().info(
                    'Container started. id:{}, image:{}, user:{}, launch:{}'.format(
                        self.__container.id, image, user, launch))
            except Exception as e:
                self.get_logger().error(str(e))
                self.get_logger().error('Failed to start container.')
                self.get_logger().debug('PlatformManager.__start_container out')
                return False
        else:
            self.get_logger().warning('Container already started.')
        self.get_logger().debug('PlatformManager.__start_container out')
        return True

    def __stop_and_remove_container(self, *, suppress_logs=False):
        self._logdebug(suppress_logs, 'PlatformManager.__stop_and_remove_container in')

        if self.__container is not None:
            self._loginfo(suppress_logs,
                          'Stop and remove container: {}'.format(self.__container.id))
            try:
                self.__container.stop()
                self.__container.remove()
            except Exception:
                pass
            self.__container = None
            # In ROS 2 there is no rosmaster stale node cleanup needed

        self._logdebug(suppress_logs, 'PlatformManager.__stop_and_remove_container out')

    def __apply_current_container_status(self):
        self.get_logger().debug('PlatformManager.__apply_current_container_status in')

        if self.__mode != Mode.USER_OFF:
            if not self.__container:
                self.__user_off_processing(warn='Container object not exists.')
            else:
                try:
                    self.__container.reload()
                    if self.__container.status != 'running':
                        self.__user_off_processing(
                            warn='Container is not running status ({}).'.format(
                                self.__container.status))
                except Exception as e:
                    self.__user_off_processing(warn=str(e))

        self.get_logger().debug('PlatformManager.__apply_current_container_status out')

    def __user_off_processing(self, *, info=None, warn=None):
        self.get_logger().debug('PlatformManager.__user_off_processing in')

        if not self.__is_container_shutdown.flag:
            with self.__is_container_shutdown:
                if warn:
                    self.get_logger().warning(str(warn))
                if info:
                    self.get_logger().info(str(info))
                self.__stop_and_remove_container()
                self.__current_launch_config_params = None
                # Restart the flight software's nodes.
                self.__start_launch_process(startup=True)
                if self.__mode != Mode.USER_OFF:
                    self.__set_mode(Mode.USER_OFF)

        self.get_logger().debug('PlatformManager.__user_off_processing out')

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------
    def __battery_charge_info_update(self, msg):
        self.get_logger().debug('PlatformManager.__battery_charge_info_update in')
        self.__battery_remain = msg.battery_remain

        if self.__battery_remain:
            if self.__battery_remain <= self.__shutdown_battery_remain:
                self.get_logger().error(
                    'Battery remain is lower than {}, current: {} .'
                    .format(self.__shutdown_battery_remain, self.__battery_remain))
                self.__shutdown()

            if self.__battery_remain > self.__required_battery_remain:
                self.__is_battery_low = False
            elif (self.__battery_remain <= self.__required_battery_remain and
                    not self.__is_battery_low):
                self.__set_off_nominal_status_flag(battery_low=True)
                self.__user_off_processing(
                    warn='Battery remain is lower than {}, current: {} .'
                         .format(self.__required_battery_remain, self.__battery_remain))

        self.get_logger().debug('PlatformManager.__battery_charge_info_update out')

    def __user_complete(self, msg):
        self.get_logger().debug('PlatformManager.__user_complete in')

        if self.__mode == Mode.USER_IN_PROGRESS:
            self.get_logger().info(
                'Received notification from user program node '
                'that processing was complete at {}.'.format(msg))
            self.__processing_after_user_logic_stop()
            self.__set_mode(Mode.USER_READY)

        self.get_logger().debug('PlatformManager.__user_complete out')

    # ------------------------------------------------------------------
    # Action client helpers
    # ------------------------------------------------------------------
    def __send_goal_and_wait(self, action_client, goal_msg):
        """Send a goal to an action client and wait for the result.
        Returns (goal_handle, result_wrapper) tuple, or (None, None) if rejected.
        """
        goal_handle_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_handle_future)
        goal_handle = goal_handle_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('Goal was rejected by action server')
            return None, None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return goal_handle, result_future.result()

    def __send_ctl_goal(self, goal_msg, done_cb=None):
        """Send a goal to the ctl action client (non-blocking)."""
        goal_handle_future = self.__target_action_client.send_goal_async(goal_msg)

        def on_goal_response(future):
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warning('Ctl goal was rejected')
                return
            self.__ctl_goal_handle = goal_handle
            self.__ctl_goal_id = goal_handle.goal_id
            if done_cb:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(
                    lambda f: self.__handle_ctl_result(f, done_cb))

        goal_handle_future.add_done_callback(on_goal_response)

    def __handle_ctl_result(self, future, done_cb):
        """Handle ctl action result."""
        try:
            result_wrapper = future.result()
            status = result_wrapper.status
            result = result_wrapper.result
            done_cb(status, result)
        except Exception as e:
            self.get_logger().error(
                'Error getting ctl action result: {}'.format(e))

    def __cancel_ctl_action(self):
        self.get_logger().debug('PlatformManager.__cancel_ctl_action in')
        if self.__ctl_goal_handle is not None:
            try:
                cancel_future = self.__ctl_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)
                self.get_logger().info('Successfully canceled action of ctl.')
            except Exception as e:
                self.get_logger().warning(
                    'Failed to cancel ctl action: {}'.format(e))
            self.__ctl_goal_handle = None
        self.get_logger().debug('PlatformManager.__cancel_ctl_action out')

    def __callback_for_moving_done(self, terminal_state, result):
        self.get_logger().debug('PlatformManager.__callback_for_moving_done in')
        if terminal_state == GoalStatus.STATUS_SUCCEEDED:
            if result.type == CtlCommand.Result.TERMINATE_SUCCESS:
                self.get_logger().info('Successfully moved.')
            else:
                self.get_logger().warning(
                    'Moving FAILED. Result : {}'.format(
                        CTL_RESULT_NAMES.get(result.type, 'UNKNOWN')))
        else:
            self.get_logger().error(
                'Action is failed with some error. GoalStatus: {}.'.format(
                    GOAL_STATUS_NAMES.get(terminal_state, 'UNKNOWN')))
        self.get_logger().debug('PlatformManager.__callback_for_moving_done out')

    def __ctl_command_feedback(self, feedback_msg):
        """
        Handle ctl command feedback.
        In ROS 2, the feedback topic message is CtlCommand.Impl.FeedbackMessage
        which contains goal_id and feedback fields.
        """
        self.get_logger().debug('PlatformManager.__ctl_command_feedback in')
        if self.__mode == Mode.USER_IN_PROGRESS:
            self.get_logger().debug('PlatformManager.__ctl_command_feedback out')
            return

        # Extract goal_id and feedback from the message
        feedback_goal_id = feedback_msg.goal_id
        feedback = feedback_msg.feedback

        # Check if this feedback is for the last command we sent
        if (self.__ctl_goal_id is None or
                feedback_goal_id != self.__ctl_goal_id):
            self.get_logger().debug(
                'Feedback is not for the last ctl command executed by platform_manager. '
                'Ignore it.')
            self.get_logger().debug('PlatformManager.__ctl_command_feedback out')
            return

        goal_id_str = str(feedback_goal_id)
        if self.__last_ctl_command_id != goal_id_str:
            self.get_logger().info(
                'New Ctl command was called. id: {}'.format(goal_id_str))
            self.__last_ctl_command_id = goal_id_str

            time_to_go_secs = feedback.time_to_go.sec
            if time_to_go_secs > 0:
                self.__last_ctl_command_cancel_execution_time = (
                    self.get_clock().now() + Duration(
                        seconds=time_to_go_secs *
                        self.__multipliers_for_action_cancellation_time_calculation))
            else:
                # Use default time based on command type
                # We cannot easily access the original goal type from the feedback,
                # so use the short default
                default_time_to_go_secs = self.__default_time_to_go_secs_short
                self.get_logger().warning(
                    'Re-set the value of time_to_go.secs to {} '
                    'because the actual value ({}) is less than or equal to 0.'
                    .format(default_time_to_go_secs, time_to_go_secs))
                self.__last_ctl_command_cancel_execution_time = (
                    self.get_clock().now() + Duration(
                        seconds=default_time_to_go_secs *
                        self.__multipliers_for_action_cancellation_time_calculation))
            self.get_logger().info('Set the timeout time to {}.'.format(
                datetime.fromtimestamp(
                    self.__last_ctl_command_cancel_execution_time.nanoseconds / 1e9)))

        # Check if the action has timed out
        if (self.__ctl_goal_handle is not None and
                self.__last_ctl_command_cancel_execution_time is not None and
                self.get_clock().now() > self.__last_ctl_command_cancel_execution_time):
            self.get_logger().error('Cancel the long non-completed Ctl command.')
            self.__cancel_ctl_action()
            self.get_logger().error(
                'Send {} action to ctl node because there may have been an error '
                'in ctl node or sensor_fusion node'.format(
                    CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            self.__send_ctl_goal(
                self.__generate_ctl_command_goal_with_type_only(CtlStatusType.STAND_BY))
        self.get_logger().debug('PlatformManager.__ctl_command_feedback out')

    def __generate_ctl_command_goal_with_type_only(self, goal_type):
        self.get_logger().debug(
            'PlatformManager.__generate_ctl_command_goal_with_type_only in')
        goal = CtlCommand.Goal()
        goal.type = CtlStatusType(type=goal_type)
        goal.target.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.get_logger().debug(
            'PlatformManager.__generate_ctl_command_goal_with_type_only out')
        return goal

    def __execute_action_goal(self, goal):
        """
        Handle incoming action goal from /trans_communication/action_goal.
        In ROS 2, goal.type is CtlStatusType, so goal.type.type gives the int value.
        """
        self.get_logger().debug('PlatformManager.__execute_action_goal in')
        goal_type_value = goal.type.type
        self.get_logger().info(
            'PlatformManager has received a control command: {}(value={})'
            .format(CTL_STATUS_NAMES.get(goal_type_value, 'UNKNOWN'),
                    goal_type_value))

        if goal_type_value not in [CtlStatusType.MOVE_TO_RELATIVE_TARGET,
                                   CtlStatusType.MOVE_TO_ABSOLUTE_TARGET,
                                   CtlStatusType.KEEP_POSE,
                                   CtlStatusType.STOP_MOVING,
                                   CtlStatusType.STAND_BY]:
            self.get_logger().warning(
                'Int-Ball2 will only accept the following command: {}, {}, {}, {} and {}'.format(
                    CTL_STATUS_NAMES[CtlStatusType.MOVE_TO_RELATIVE_TARGET],
                    CTL_STATUS_NAMES[CtlStatusType.MOVE_TO_ABSOLUTE_TARGET],
                    CTL_STATUS_NAMES[CtlStatusType.KEEP_POSE],
                    CTL_STATUS_NAMES[CtlStatusType.STOP_MOVING],
                    CTL_STATUS_NAMES[CtlStatusType.STAND_BY],
                ))
            self.get_logger().warning(
                'REJECT your action. (type value: {})'
                .format(CTL_STATUS_NAMES.get(goal_type_value, 'UNKNOWN')))
            self.get_logger().debug('PlatformManager.__execute_action_goal out')
            return

        if (self.__operation_type == OperationType.NAV_OFF and
                goal_type_value in [
                    CtlStatusType.MOVE_TO_RELATIVE_TARGET,
                    CtlStatusType.MOVE_TO_ABSOLUTE_TARGET,
                    CtlStatusType.KEEP_POSE,
                    CtlStatusType.STOP_MOVING]):
            self.get_logger().warning(
                'When in {} mode, the guidance control cannot be executed '
                '(Only {} is accepted)'
                .format(OPERATION_TYPE_NAMES.get(OperationType.NAV_OFF, 'NAV_OFF'),
                        CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            self.get_logger().warning(
                'REJECT your action. (type value: {})'
                .format(CTL_STATUS_NAMES.get(goal_type_value, 'UNKNOWN')))
            self.get_logger().debug('PlatformManager.__execute_action_goal out')
            return

        # update timestamp as this system
        telecommand_stamp = goal.target.header.stamp
        goal.target.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            'Timestamp in goal action is updated as the system time. '
            '(source: {})(new: {})'.format(
                datetime.fromtimestamp(
                    telecommand_stamp.sec + telecommand_stamp.nanosec / 1e9),
                datetime.fromtimestamp(
                    goal.target.header.stamp.sec +
                    goal.target.header.stamp.nanosec / 1e9)))

        self.get_logger().info('PlatformManager get message goal:\n{}'.format(goal))
        if (goal_type_value in [CtlStatusType.KEEP_POSE, CtlStatusType.STOP_MOVING] and
                self.__ctl_goal_handle is not None):
            self.get_logger().info(
                'Cancel the current ctl command because STOP command ({}) was requested'
                .format(CTL_STATUS_NAMES[goal_type_value]))
            self.__cancel_ctl_action()
        elif goal_type_value == CtlStatusType.STAND_BY:
            self.get_logger().warning(
                'Send {} action to ctl node.'.format(
                    CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            self.__send_ctl_goal(
                self.__generate_ctl_command_goal_with_type_only(CtlStatusType.STAND_BY))
        else:
            self.__send_ctl_goal(
                goal, done_cb=self.__callback_for_moving_done)

        self.get_logger().debug('PlatformManager.__execute_action_goal out')

    def __check_and_call_ctl_command_standby(self, *, suppress_logs=False):
        self._logdebug(suppress_logs,
                       'PlatformManager.__check_and_call_ctl_command_standby in')

        try:
            # In ROS 2, use create_subscription with a one-shot approach
            # or simply wait for a message on the topic
            ctl_status_msg = None
            ctl_status_received = [False]

            def ctl_status_cb(msg):
                nonlocal ctl_status_msg
                ctl_status_msg = msg
                ctl_status_received[0] = True

            temp_sub = self.create_subscription(
                CtlStatus, '/ctl/status', ctl_status_cb, 1)

            # Wait for the message with timeout
            timeout = self.__waiting_time_for_topic
            start_time = self.get_clock().now()
            while not ctl_status_received[0]:
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout:
                    break

            self.destroy_subscription(temp_sub)

            if ctl_status_msg is not None:
                if ctl_status_msg.type.type != CtlStatusType.STAND_BY:
                    goal = self.__generate_ctl_command_goal_with_type_only(
                        CtlStatusType.STAND_BY)
                    self.__send_goal_and_wait(
                        self.__target_action_client, goal)
                    self._loginfo(suppress_logs,
                                  'ctl_only node is successfully stopped (as {}).'
                                  .format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
                else:
                    self._loginfo(suppress_logs,
                                  'ctl_only node is already stopped (as {}).'
                                  .format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            else:
                self._loginfo(suppress_logs, 'ctl_only node is not running.')
        except Exception:
            self._loginfo(suppress_logs, 'ctl_only node is not running.')

        self._logdebug(suppress_logs,
                       'PlatformManager.__check_and_call_ctl_command_standby out')

    def __wait_action_server(self, action_client, raise_wait_error=False):
        self.get_logger().debug('PlatformManager.__wait_action_server in')

        wait_result = action_client.wait_for_server(
            timeout_sec=self.__waiting_time_for_server)
        if not wait_result:
            self.get_logger().error(
                'Action server has not started after {} seconds.'
                .format(self.__waiting_time_for_server))
            if raise_wait_error:
                self.get_logger().error('PlatformManager will stop.')
                self.get_logger().debug('PlatformManager.__wait_action_server out')
                raise RuntimeError('Action server not available')

        self.get_logger().debug('PlatformManager.__wait_action_server out')
        return bool(wait_result)

    # ------------------------------------------------------------------
    # Navigation / Control processing
    # ------------------------------------------------------------------
    def __nav_off_processing(self, *, raise_wait_error=False):
        self.get_logger().debug('PlatformManager.__nav_off_processing in')

        if self.__use_ctl:
            self.__wait_action_server(
                self.__target_action_client, raise_wait_error=raise_wait_error)
            self.__check_and_call_ctl_command_standby()

        if self.__use_nav:
            self.__wait_action_server(
                self.__navigation_start_up_client,
                raise_wait_error=raise_wait_error)
            # make the navigation stand-by
            self.get_logger().info('Stopping navigation node...')
            navigation_request = NavigationStartUp.Goal()
            navigation_request.command = NavigationStartUp.Goal.OFF
            self.__send_goal_and_wait(
                self.__navigation_start_up_client, navigation_request)
            self.get_logger().info('Navigation node is successfully stopped.')

        self.get_logger().debug('PlatformManager.__nav_off_processing out')

    def __nav_on_processing(self, *, raise_wait_error=False):
        self.get_logger().debug('PlatformManager.__nav_on_processing in')

        execute_ctl_on = self.__use_ctl

        if self.__use_nav:
            if not self.__navigation_start_up(raise_wait_error=raise_wait_error):
                execute_ctl_on = False

        if execute_ctl_on:
            if self.__wait_action_server(
                    self.__target_action_client,
                    raise_wait_error=raise_wait_error):
                # make the ctl on
                self.get_logger().info(
                    'Start ctl node...(as {})'.format(
                        CTL_STATUS_NAMES[CtlStatusType.KEEP_POSE]))
                goal = self.__generate_ctl_command_goal_with_type_only(
                    CtlStatusType.KEEP_POSE)
                # do not wait here because detect errors in __ctl_command_feedback
                self.__send_ctl_goal(goal)
                self.get_logger().info('Ctl node will start.')

        self.get_logger().debug('PlatformManager.__nav_on_processing out')

    def __navigation_start_up(self, *, raise_wait_error=False):
        self.get_logger().debug('PlatformManager.__navigation_start_up in')

        result = False

        if self.__wait_action_server(
                self.__navigation_start_up_client,
                raise_wait_error=raise_wait_error):
            # make the slam on
            if not self.__run_on_simulator:
                try:
                    if not self.__slam_wrapper_switch_power.wait_for_service(
                            timeout_sec=self.__waiting_time_for_server):
                        self.get_logger().error(
                            '/slam_wrapper/switch_power service not available')
                        return False
                    self.get_logger().info(
                        'Notifies slam_wrapper node to start processing '
                        '(calls switch_power service).')
                    req = SwitchPower.Request()
                    req.power = PowerStatus(status=PowerStatus.ON)
                    future = self.__slam_wrapper_switch_power.call_async(req)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                except Exception as e:
                    self.get_logger().error(str(e))
                    self.get_logger().debug('PlatformManager.__navigation_start_up out')
                    return False

            # make the navigation on
            self.get_logger().info(
                'Start navigation node (or start nav plugin function)...')
            if self.__check_navigation_startup_on():
                self.get_logger().info(
                    'Navigation node (or nav plugin) is successfully started.')
                result = True
            else:
                self.get_logger().error(
                    'Navigation node (or nav plugin) could not be started.')

        self.get_logger().debug('PlatformManager.__navigation_start_up out')
        return result

    # ------------------------------------------------------------------
    # Mode / operation type management
    # ------------------------------------------------------------------
    def __set_mode(self, mode):
        self.get_logger().debug('PlatformManager.__set_mode in')
        if mode != self.__mode:
            self.__mode = mode
            self.get_logger().info(
                'Mode changed to {}'.format(MODE_NAMES.get(self.__mode, 'UNKNOWN')))
            if self.__mode in [Mode.USER_OFF, Mode.USER_READY]:
                # if self.__operation_type is undefined (None), no action is taken
                if self.__operation_type == OperationType.NAV_OFF:
                    self.__nav_off_processing()
                elif self.__operation_type == OperationType.NAV_ON:
                    self.__nav_on_processing()
        self.get_logger().debug('PlatformManager.__set_mode out')

    def __set_operation_type(self, operation_type, *, raise_wait_error=False):
        self.get_logger().debug('PlatformManager.__set_operation_type in')
        if operation_type != self.__operation_type:
            self.__operation_type = operation_type
            self.get_logger().info(
                'Operation type changed to {}'.format(
                    OPERATION_TYPE_NAMES.get(self.__operation_type, 'UNKNOWN')))
            if self.__operation_type == OperationType.NAV_OFF:
                self.__nav_off_processing(raise_wait_error=raise_wait_error)
            elif self.__operation_type == OperationType.NAV_ON:
                self.__nav_on_processing(raise_wait_error=raise_wait_error)
        self.get_logger().debug('PlatformManager.__set_operation_type out')

    def __set_off_nominal_status_flag(self, *, battery_low=False,
                                      short_disk_space=False,
                                      temperature_to_cool=False,
                                      wifi_disconnected=False):
        self.get_logger().debug('PlatformManager.__set_off_nominal_status_flag in')

        if battery_low:
            self.__is_battery_low = True
        if short_disk_space:
            self.__is_short_disk_space = True
        if temperature_to_cool:
            self.__is_cooling = True
        if wifi_disconnected:
            self.__is_wifi_disconnected = True

        if [self.__is_battery_low, self.__is_short_disk_space,
                self.__is_cooling].count(True) >= 2:
            self.get_logger().error(
                'Multiple off-nominal events were detected simultaneously.')
            self.get_logger().error(
                'Off-nominal event statuses: Battery:{}, Disk space:{}, Temperature:{}.'
                .format(self.__is_battery_low, self.__is_short_disk_space, self.__is_cooling))
            self.__shutdown()

        self.get_logger().debug('PlatformManager.__set_off_nominal_status_flag out')

    def __check_short_disk_space(self):
        self.get_logger().debug('PlatformManager.__check_short_disk_space in')
        short_disk_space = False
        shutdown_required = False
        for disk_space in self.__system_status.disk_spaces:
            if disk_space.remain <= self.__shutdown_storage_ratio:
                self.get_logger().error(
                    'Disk space is lower than {} %, current: {} %. '
                    'The system shutdown will be executed.'
                    '(path: {})'.format(
                        self.__shutdown_storage_ratio,
                        disk_space.remain, disk_space.path))
                shutdown_required = True
            elif disk_space.remain <= self.__required_storage_ratio:
                self.get_logger().warning(
                    'Disk space is lower than {} %, current: {} %.'
                    '(path: {})'.format(
                        self.__required_storage_ratio,
                        disk_space.remain, disk_space.path))
                short_disk_space = True
        self.get_logger().debug('PlatformManager.__check_short_disk_space out')
        return short_disk_space, shutdown_required

    def __system_status_update(self, msg):
        self.get_logger().debug('PlatformManager.__system_status_update in')
        self.__prev_system_status = self.__system_status
        self.__system_status = msg
        # operation for disk-space status
        short_disk_space, shutdown_required = self.__check_short_disk_space()
        if shutdown_required:
            self.get_logger().error('Force termination of platform_manager node.')
            os._exit(1)

        if not short_disk_space:
            self.__is_short_disk_space = False
        elif short_disk_space and not self.__is_short_disk_space:
            self.__set_off_nominal_status_flag(short_disk_space=True)
            self.__user_off_processing()

        # operation for wifi-connection status
        if self.__system_status.wifi_connected:
            self.__last_wifi_connected_time = Time.from_msg(
                self.__system_status.check_time)
            self.__is_wifi_disconnected = False
        else:
            check_time = Time.from_msg(self.__system_status.check_time)
            disconnected_duration = check_time - self.__last_wifi_connected_time
            if disconnected_duration > self.__wifi_duration:
                if not self.__is_wifi_disconnected:
                    self.__set_off_nominal_status_flag(wifi_disconnected=True)
                    self.__user_off_processing(
                        warn='Wifi has been disconnected for {} seconds.'
                             .format(disconnected_duration.nanoseconds / 1e9))
            else:
                self.__is_wifi_disconnected = False

        # operation for temperature status
        if self.__system_status.temperature >= self.__temperature_to_shutdown:
            self.get_logger().warning(
                'Temperature is over {} degrees (current: {} degrees).'
                .format(self.__temperature_to_shutdown,
                        self.__system_status.temperature))
            self.__shutdown()
        elif (self.__system_status.temperature >= self.__temperature_to_cool and
                not self.__is_cooling):
            self.__set_off_nominal_status_flag(temperature_to_cool=True)
            self.__user_off_processing(
                warn='Temperature is over {} degrees (current: {} degrees). '
                     'Stop user processing.'
                     .format(self.__temperature_to_cool,
                             self.__system_status.temperature))
        elif (self.__is_cooling and
                self.__system_status.temperature <= self.__temperature_to_revive):
            self.get_logger().info(
                'Temperature come under {} degrees (current: {} degrees).'
                .format(self.__temperature_to_revive,
                        self.__system_status.temperature))
            self.__is_cooling = False

        self.get_logger().debug('PlatformManager.__system_status_update out')

    def __check_navigation_startup_on(self):
        self.get_logger().debug('PlatformManager.__check_navigation_startup_on in')

        navigation_request = NavigationStartUp.Goal()
        navigation_request.command = NavigationStartUp.Goal.ON

        goal_handle, result_wrapper = self.__send_goal_and_wait(
            self.__navigation_start_up_client, navigation_request)

        if goal_handle is None:
            self.get_logger().debug('PlatformManager.__check_navigation_startup_on out')
            return False

        navigation_start_up_result = result_wrapper.result if result_wrapper else None

        self.get_logger().debug('PlatformManager.__check_navigation_startup_on out')
        return (navigation_start_up_result is not None and
                navigation_start_up_result.type == NavigationStartUp.Result.ON_READY)

    # ------------------------------------------------------------------
    # System commands
    # ------------------------------------------------------------------
    def __reboot(self, msg):
        self.get_logger().debug('PlatformManager.__reboot in')
        if not self.__enable_shutdown:
            self.get_logger().warning(
                'Cancel the reboot because the setting "enable_shutdown" is False.')
            self.get_logger().debug('PlatformManager.__reboot out')
            return
        self.get_logger().warning('REBOOT THE SYSTEM.')
        subprocess.call('/usr/bin/sudo /sbin/shutdown -r now', shell=True)
        self.get_logger().error('Reboot failed.')
        self.get_logger().debug('PlatformManager.__reboot out')

    def __shutdown(self):
        self.get_logger().debug('PlatformManager.__shutdown in')
        if not self.__enable_shutdown:
            self.get_logger().warning(
                'Cancel the shutdown because the setting "enable_shutdown" is False.')
            self.get_logger().debug('PlatformManager.__shutdown out')
            return
        self.get_logger().warning('SHUTDOWN THE SYSTEM.')
        subprocess.call('/usr/bin/sudo /sbin/shutdown -P now', shell=True)
        self.get_logger().error('Shutdown failed.')
        self.get_logger().debug('PlatformManager.__shutdown out')

    # ------------------------------------------------------------------
    # Periodic publishers (called from timer)
    # ------------------------------------------------------------------
    def __publish_status(self):
        self.get_logger().debug('PlatformManager.__publish_status in')
        msg = ManagerStatus(
            stamp=self.get_clock().now().to_msg(),
            type=OperationType(type=self.__operation_type if self.__operation_type is not None else 0),
            mode=Mode(mode=self.__mode if self.__mode is not None else 0),
            start_container=bool(self.__container),
            last_user_logic=self.__last_user_logic if self.__last_user_logic is not None else UserLogic(),
            last_image=self.__last_image if self.__last_image is not None else '',
            last_user=self.__last_user if self.__last_user is not None else '',
            last_launch=self.__last_launch if self.__last_launch is not None else '',
        )
        self.__status_publisher.publish(msg)
        self.get_logger().debug('PlatformManager.__publish_status out')

    def __publish_led_colors(self):
        self.get_logger().debug('PlatformManager.__publish_led_colors in')
        if self.__mode != Mode.USER_IN_PROGRESS:
            self.__led_color_publisher_left.publish(
                LEDColors(colors=[self.__color_with_camera_mic] * 8))
            self.__led_color_publisher_right.publish(
                LEDColors(colors=[self.__color_with_camera_mic] * 8))
        self.get_logger().debug('PlatformManager.__publish_led_colors out')

    def __publish_fan_duty(self):
        self.get_logger().debug('PlatformManager.__publish_fan_duty in')
        if (self.__operation_type == OperationType.NAV_OFF and
                self.__mode != Mode.USER_IN_PROGRESS):
            self.__fan_duty_publisher.publish(
                Float64MultiArray(data=[0.0] * 8))
        self.get_logger().debug('PlatformManager.__publish_fan_duty out')

    # ------------------------------------------------------------------
    # Main timer callback (replaces while-loop + rate.sleep())
    # ------------------------------------------------------------------
    def __main_loop(self):
        self.__apply_current_container_status()
        self.__publish_status()
        self.__publish_led_colors()
        self.__publish_fan_duty()


def main(args=None):
    rclpy.init(args=args)
    node = PlatformManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
