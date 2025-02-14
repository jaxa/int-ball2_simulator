#!/usr/bin/python3
# -*- coding:utf-8 -*-
import docker
import rosgraph
import roslaunch
import rosnode
import rospkg
import rospy
import subprocess
import os
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from datetime import datetime
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA, Empty, Float64MultiArray, Time
from ib2_msgs.msg import (
    BatteryChargeInfo,
    CtlCommandAction,
    CtlCommandActionFeedback,
    CtlCommandGoal,
    CtlCommandResult,
    CtlStatus,
    CtlStatusType,
    LEDColors,
    Navigation,
    NavigationStartUpAction,
    NavigationStartUpGoal,
    NavigationStartUpResult,
    PowerStatus,
    Slam,
    SystemStatus,
)
from ib2_msgs.srv import (
    SwitchPower,
    SwitchPowerRequest,
)
from platform_msgs.msg import (
    ManagerStatus,
    Mode,
    OperationType,
    UserLogic,
)
from platform_msgs.srv import (
    SetOperationType,
    SetOperationTypeResponse,
    StopProcessingUserNode,
    StopProcessingUserNodeResponse,
    UserLogicCommand,
    UserLogicCommandRequest,
    UserLogicCommandResponse,
    UserNodeCommand,
    UserNodeCommandRequest,
    UserNodeCommandResponse,
)

MODE_NAMES = {value: name for name, value in vars(Mode).items() if name.isupper()}
OPERATION_TYPE_NAMES = {value: name for name, value in vars(OperationType).items() if name.isupper()}
CTL_RESULT_NAMES = {value: name for name, value in vars(CtlCommandResult).items() if name.isupper()}
GOAL_STATUS_NAMES = {value: name for name, value in vars(GoalStatus).items() if name.isupper()}
CTL_STATUS_NAMES = {value: name for name, value in vars(CtlStatusType).items() if name.isupper()}


def logerr(suppress, msg):
    if not suppress:
        rospy.logerr(msg)


def loginfo(suppress, msg):
    if not suppress:
        rospy.loginfo(msg)


def logdebug(suppress, msg):
    if not suppress:
        rospy.logdebug(msg)


class ClosableFlag(object):

    def __init__(self):
        self.flag = False

    def __enter__(self):
        self.flag = True

    def __exit__(self, execption_type, exception_value, traceback):
        self.flag = False


class PlatformManager(object):
    """Platform Manager for Int-Ball2."""
    PLATFORM_LAUNCH_PREFIX = '/platform_launch/'
    NODE_ID = 'platform_manager'

    def __init__(self):
        rospy.logdebug('PlatformManager.__init__ in')
        rospy.init_node(PlatformManager.NODE_ID)

        self.__battery_remain = None
        self.__container = None
        self.__default_stop_roslaunch_names = []
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
        self.__last_wifi_connected_time = rospy.Time.now()
        self.__mode = None
        self.__operation_type = None
        self.__post_command = {}
        self.__pre_command = {}
        self.__prev_system_status = None
        self.__roslaunch = {}
        self.__current_roslaunch_config = None
        self.__roslaunch_parameter = {}
        self.__system_status = None
        self.__use_ctl = False
        self.__use_nav = False

        # Rosparam
        color_list = rospy.get_param('~color_with_camera_mic', [0.0, 0.0, 1.0])
        if len(color_list) != 3:
            rospy.logerr('len(color_with_camera_mic) should be 3. now {}. return black color(non-color).'
                         .format(len(color_list)))
            rospy.logdebug('PlatformManager.__init__ out')
            raise
        self.__color_with_camera_mic = ColorRGBA(r=color_list[0], g=color_list[1], b=color_list[2])

        self.__container_ros_master_uri = rospy.get_param('~container_ros_master_uri')
        self.__enable_shutdown = rospy.get_param('~enable_shutdown')
        self.__host_ib2_workspace = rospy.get_param('~host_ib2_workspace')
        self.__multipliers_for_action_cancellation_time_calculation = rospy.get_param(
            '~multipliers_for_action_cancellation_time_calculation', 2)
        self.__rate = rospy.Rate(rospy.get_param('~rate', 1))
        self.__required_battery_remain = rospy.get_param('~required_battery_remain', 20)
        self.__required_storage_ratio = rospy.get_param('~required_storage_ratio', 10.0)
        self.__run_on_simulator = rospy.get_param('~run_on_simulator', False)
        self.__shutdown_battery_remain = rospy.get_param('~shutdown_battery_remain', 5)
        self.__shutdown_storage_ratio = rospy.get_param('~shutdown_storage_ratio', 5.0)
        self.__temperature_to_cool = rospy.get_param('~temperature_to_cool', 80.0)
        self.__temperature_to_revive = rospy.get_param('~temperature_to_revive', 70.0)
        self.__temperature_to_shutdown = rospy.get_param('~temperature_to_shutdown', 90.0)
        self.__user_container_name = rospy.get_param('~user_container_name')
        self.__waiting_time_for_server = rospy.get_param('~waiting_time_for_server', 10.0)
        self.__waiting_time_for_topic = rospy.get_param('~waiting_time_for_topic', 10.0)
        self.__wifi_duration = rospy.Duration(rospy.get_param('~wifi_duration', 5.0))

        # load from config.yml
        self.__bringup_packages = rospy.get_param('~bringup_packages', [])
        bringup_package_names = [p['name'] for p in self.__bringup_packages]
        rospy.loginfo('Bringup package names: {}'.format(bringup_package_names))
        self.__acceptable_packages = rospy.get_param('~acceptable_packages', [])
        if not self.__run_on_simulator and self.__acceptable_packages:
            rospy.logerr('acceptable_packages can be set only when run_on_simulator is true')
            rospy.logdebug('PlatformManager.__init__ out')
            raise
        rospy.loginfo('Acceptable package names '
                      '(If these packages are specified in user\'s roslaunch file, it is not an error, '
                      'but the node will not be activated.): {}'.format(self.__acceptable_packages))
        check_packages = set(bringup_package_names) & set(self.__acceptable_packages)
        if len(check_packages) > 0:
            rospy.logerr('The same package name cannot be included in '
                         'both \'bringup_packages\' and \'acceptable_packages\'. : {}'
                         .format(check_packages))
            rospy.logdebug('PlatformManager.__init__ out')
            raise

        if 'sensor_fusion' in bringup_package_names:
            rospy.loginfo('Controls sensor_fusion node')
            self.__use_nav = True
        elif self.__run_on_simulator:
            rospy.loginfo('Controls navigation plugin')
            self.__use_nav = True
        else:
            rospy.loginfo('sensor_fusion node and navigatoin plugin are not controlled')
            self.__use_nav = False
        if 'ctl_only' in bringup_package_names or 'ctl' in bringup_package_names:
            rospy.loginfo('Controls ctl_only node and ctl node')
            self.__use_ctl = True
        else:
            rospy.loginfo('ctl_only node and ctl node are not controlled')
            self.__use_ctl = False

        # load from config.yml
        self.__non_simultaneous_package_info = rospy.get_param('~non_simultaneous_package_info', [])

        try:
            # Publisher
            self.__status_publisher = rospy.Publisher('~status', ManagerStatus, queue_size=1)
            self.__ib2_user_start_publisher = rospy.Publisher('/ib2_user/start', UserLogic, queue_size=1)
            self.__led_color_publisher_left = rospy.Publisher('/led_display_left/led_colors', LEDColors, queue_size=1)
            self.__led_color_publisher_right = rospy.Publisher('/led_display_right/led_colors', LEDColors, queue_size=1)
            self.__fan_duty_publisher = rospy.Publisher('/ctl/duty', Float64MultiArray, queue_size=1)

            # Service server
            self.__set_operation_type_server = rospy.Service('~set_operation_type',
                                                             SetOperationType,
                                                             self.__callback_for_set_operation_type)
            self.__user_node_server = rospy.Service('~user_node',
                                                    UserNodeCommand,
                                                    self.__callback_for_user_node)
            self.__user_logic_server = rospy.Service('~user_logic',
                                                     UserLogicCommand,
                                                     self.__callback_for_user_logic)

            # Action client
            self.__target_action_client = SimpleActionClient('/ctl/command', CtlCommandAction)
            self.__navigation_start_up_client = SimpleActionClient(
                '/sensor_fusion/navigation_start_up', NavigationStartUpAction)
            self.__action_goal_subscriber = rospy.Subscriber('/trans_communication/action_goal',
                                                             CtlCommandGoal, self.__execute_action_goal)
            self.__reboot_subscriber = rospy.Subscriber('/trans_communication/reboot', Empty, self.__reboot)
            self.__target_action_feedback_subscriber = rospy.Subscriber('/ctl/command/feedback',
                                                                        CtlCommandActionFeedback,
                                                                        self.__ctl_command_feedback)

            # Service client
            self.__stop_processing_user_node = rospy.ServiceProxy('/ib2_user/stop', StopProcessingUserNode)

            if not self.__run_on_simulator:
                self.__slam_wrapper_switch_power = rospy.ServiceProxy('/slam_wrapper/switch_power', SwitchPower)

            # Subscriber
            self.__system_status_subscriber = rospy.Subscriber(
                '/system_monitor/status', SystemStatus, self.__system_status_update)
            self.__battery_charge_info_subscriber = rospy.Subscriber(
                '/dock/battery_charge_info', BatteryChargeInfo, self.__battery_charge_info_update)
            self.__user_complete_subscriber = rospy.Subscriber(
                '/ib2_user/complete', Time, self.__user_complete)
        except Exception as e:
            rospy.logerr(e)
            rospy.logdebug('PlatformManager.__init__ out')
            raise e

        for package_config in self.__bringup_packages:
            # package_config contains following values:
            # - name (*required)
            # - package (optional)
            # - launch_file (optional)
            # - pre_command (optional)
            # - post_command (optional)
            # - startup (optional)

            rospy.loginfo(package_config)
            # 'name' is used as the key of roslaunch processes (self.__roslaunch)
            name = package_config['name']
            if name in self.__acceptable_packages:
                rospy.loginfo('{} will not be started because it is defined within acceptable_packages'
                              .format(name))
                continue
            package = package_config.get('package', name)
            launch_file = package_config.get('launch_file', 'bringup.launch')
            startup = package_config.get('startup', True)
            self.__roslaunch_parameter[name] = {
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
                self.__default_stop_roslaunch_names.append(name)
            self.__setup_roslaunch_process(name)
        self.__start_roslaunch_process(startup=True)

        # ***** Provisional processing *****
        # Control camera_main node
        self.__bringup_camera_main = rospy.get_param('~bringup_camera_main', False)
        if not self.__run_on_simulator and self.__bringup_camera_main:
            camera_main_launch_file = roslaunch.rlutil.resolve_launch_arguments(['gscam', 'bringup_main.launch'])
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
            self.__camera_main_roslaunch = roslaunch.parent.ROSLaunchParent(uuid, camera_main_launch_file)
            rospy.loginfo('Launch camera_main node')
            self.__camera_main_roslaunch.start()
        # **********************************

        self.__set_mode(Mode.USER_OFF)

        self.__set_operation_type(OperationType.NAV_OFF, raise_wait_error=True)

        rospy.logdebug('PlatformManager.__init__ out')

    def __del__(self):
        self.__stop_and_remove_container(suppress_logs=True)
        self.__stop_roslaunch_process(suppress_logs=True)

    def __callback_for_set_operation_type(self, request):
        rospy.logdebug('PlatformManager.__callback_for_set_operation_type in')

        if self.__mode == Mode.USER_IN_PROGRESS:
            rospy.logdebug('PlatformManager.__callback_for_set_operation_type out')
            return SetOperationTypeResponse(result=SetOperationTypeResponse.USER_LOGIC_IN_PROGRESS)

        if request.type.type == self.__operation_type:
            rospy.logdebug('PlatformManager.__callback_for_set_operation_type out')
            return SetOperationTypeResponse(result=SetOperationTypeResponse.NO_CHANGE)

        if request.type.type in [OperationType.NAV_OFF, OperationType.NAV_ON]:
            self.__set_operation_type(request.type.type)
            rospy.logdebug('PlatformManager.__callback_for_set_operation_type out')
            return SetOperationTypeResponse(result=SetOperationTypeResponse.SUCCESS)

        rospy.logdebug('PlatformManager.__callback_for_set_operation_type out')
        return SetOperationTypeResponse(result=SetOperationTypeResponse.ERROR)

    def __callback_for_user_node(self, request):
        rospy.logdebug('PlatformManager.__callback_for_user_node in')

        def error_return(response_result):
            rospy.logdebug('PlatformManager.__callback_for_user_node out')
            return UserNodeCommandResponse(result=response_result)

        if request.command == UserNodeCommandRequest.START:
            if self.__is_battery_low or self.__is_cooling or self.__is_short_disk_space or self.__is_wifi_disconnected:
                rospy.logerr('When off-nominal, user node cannot be started')
                return error_return(UserNodeCommandResponse.OFF_NOMINAL)

            #
            # Validate request parameters.
            #

            if self.__mode != Mode.USER_OFF:
                return error_return(UserNodeCommandResponse.NODE_ALREADY_RUNNING)

            try:
                rospkg.RosPack().get_path(request.user)
            except Exception:
                return error_return(UserNodeCommandResponse.USER_NOT_FOUND)

            try:
                # expected arguments = (package-name + relative-roslaunch-filename) = (user + launch)
                roslaunch.rlutil.resolve_launch_arguments((request.user, request.launch))
            except Exception:
                return error_return(UserNodeCommandResponse.LAUNCH_NOT_FOUND)

            try:
                self.__docker_client.images.get(request.image)
            except Exception:
                return error_return(UserNodeCommandResponse.IMAGE_NOT_FOUND)

            # Check the contents of the launch file
            roslaunch_config_obj = self.__preprocess_user_roslaunch_config(request.user, request.launch)
            if roslaunch_config_obj is None:
                return error_return(UserNodeCommandResponse.INVALID_LAUNCH)

            #
            # Start user node
            #
            if not self.__start_container(request.image, request.user, request.launch):
                return error_return(UserNodeCommandResponse.ERROR)
            # Keep roslaunch_config for starting user logic
            self.__current_roslaunch_config = roslaunch_config_obj
            self.__set_mode(Mode.USER_READY)

        elif request.command == UserNodeCommandRequest.STOP:
            #
            # Stop user node
            #
            self.__user_off_processing()

            if self.__mode == Mode.USER_IN_PROGRESS:
                self.__processing_after_user_logic_stop()

        else:
            return error_return(UserNodeCommandResponse.INVALID_COMMAND)

        rospy.logdebug('PlatformManager.__callback_for_user_node out')
        return UserNodeCommandResponse(result=UserNodeCommandResponse.SUCCESS)

    def __processing_after_user_logic_stop(self):
        rospy.logdebug('PlatformManager.__processing_after_user_logic_stop in')

        #
        # Control the nodes specified by the roslaunch config.
        #

        # Stop (if necessary)
        if self.__default_stop_roslaunch_names:
            self.__stop_roslaunch_process(targets=set(self.__default_stop_roslaunch_names))

        # ***** Provisional processing *****
        # Restart camera_main node
        if not self.__run_on_simulator and self.__bringup_camera_main:
            rospy.loginfo('Restart camera_main node')
            self.__camera_main_roslaunch.shutdown()
            camera_main_launch_file = roslaunch.rlutil.resolve_launch_arguments(
                ['gscam', 'bringup_main.launch'])
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
            self.__camera_main_roslaunch = roslaunch.parent.ROSLaunchParent(uuid, camera_main_launch_file)
            self.__camera_main_roslaunch.start()
        # **********************************

        # Start
        self.__start_roslaunch_process(startup=True)

        if self.__operation_type == OperationType.NAV_ON:
            self.__nav_on_processing()
        elif self.__operation_type == OperationType.NAV_OFF:
            self.__nav_off_processing()

        rospy.logdebug('PlatformManager.__processing_after_user_logic_stop out')

    def __callback_for_user_logic(self, request):
        rospy.logdebug('PlatformManager.__callback_for_user_logic in')

        def error_return(response_result):
            rospy.logdebug('PlatformManager.__callback_for_user_logic out')
            return UserLogicCommandResponse(result=response_result)

        if request.command == UserLogicCommandRequest.START:
            if self.__is_battery_low or self.__is_cooling or self.__is_short_disk_space or self.__is_wifi_disconnected:
                rospy.logerr('When off-nominal, user logic cannot be started')
                return error_return(UserLogicCommandResponse.OFF_NOMINAL)

            if self.__mode == Mode.USER_OFF:
                return error_return(UserLogicCommandResponse.NODE_NOT_STARTED)

            if self.__mode == Mode.USER_IN_PROGRESS:
                return error_return(UserLogicCommandResponse.LOGIC_ALREADY_RUNNING)

            #
            # Control the nodes specified by the roslaunch config.
            # When trasitioning to USER_READY mode, current_roslaunch_config is set.
            #
            self.__roslaunch_control_based_on_user_roslaunch(self.__current_roslaunch_config)

            #
            # Requests user node to start processing.
            #
            rospy.loginfo('Publish {}: {}'.format(
                self.__ib2_user_start_publisher.resolved_name, request.logic))
            self.__ib2_user_start_publisher.publish(request.logic)
            self.__set_mode(Mode.USER_IN_PROGRESS)
            self.__last_user_logic = request.logic

        elif request.command == UserLogicCommandRequest.STOP:
            if self.__mode == Mode.USER_OFF:
                return error_return(UserLogicCommandResponse.NODE_NOT_STARTED)

            if self.__mode == Mode.USER_READY:
                return error_return(UserLogicCommandResponse.LOGIC_NOT_STARTED)

            #
            # Requests user node to stop the process.
            #
            rospy.loginfo('Call {}'.format(self.__stop_processing_user_node.resolved_name))
            stop_processing_result = self.__stop_processing_user_node().result
            if stop_processing_result == StopProcessingUserNodeResponse.SUCCESS:
                rospy.loginfo('User logic has stopped successfully.')
                self.__processing_after_user_logic_stop()
                self.__set_mode(Mode.USER_READY)

            else:
                rospy.logerr('Failed to stop user logic.')
                return error_return(UserLogicCommandResponse.ERROR)

        else:
            return error_return(UserLogicCommandResponse.INVALID_COMMAND)

        rospy.logdebug('PlatformManager.__callback_for_user_logic out')
        return error_return(UserLogicCommandResponse.SUCCESS)

    def __setup_roslaunch_process(self, parameter_name):
        rospy.logdebug('PlatformManager.__setup_roslaunch_process in')
        name = self.__roslaunch_parameter[parameter_name]['name']
        package = self.__roslaunch_parameter[parameter_name]['package']
        launch_file_name = self.__roslaunch_parameter[parameter_name]['launch_file']

        launch_file = roslaunch.rlutil.resolve_launch_arguments([package, launch_file_name])
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)

        rospy.loginfo('Set up for execute roslaunch: {} {}'.format(package, launch_file))
        self.__roslaunch[name] = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

        rospy.logdebug('PlatformManager.__setup_roslaunch_process out')

    def __get_current_roslaunch_process_keys(self, *, started_only=False, stopped_only=False):
        rospy.logdebug('PlatformManager.__get_current_roslaunch_process_keys in')

        def is_alive(launch): return bool(launch and
                                          launch.pm and
                                          launch.pm.is_alive() and
                                          launch.pm.get_active_names())

        current_roslaunch = [(key, is_alive(self.__roslaunch[key]))
                             for key in self.__roslaunch.keys()]
        if started_only:
            current_roslaunch = [c for c in current_roslaunch if c[1] is True]
        if stopped_only:
            current_roslaunch = [c for c in current_roslaunch if c[1] is False]

        rospy.logdebug('PlatformManager.__get_current_roslaunch_process_keys out')
        return [c[0] for c in current_roslaunch]

    def __start_roslaunch_process(self, *, targets=(), startup=None):
        rospy.logdebug('PlatformManager.__start_roslaunch_process in')

        target_process_keys = set(self.__get_current_roslaunch_process_keys(stopped_only=True))
        if targets:
            target_process_keys &= targets

        if not target_process_keys:
            rospy.loginfo('There are no roslaunch process that can be started.')
            if targets:
                rospy.loginfo('The target nodes({}) was not running.'.format(targets))
            else:
                rospy.loginfo('There are no roslaunch process that can be stopped.')
        else:
            for process in target_process_keys:
                if startup is not None and self.__roslaunch_parameter[process]['startup'] != startup:
                    rospy.loginfo('Because \'startup\' is not {},'
                                  ' {} will be removed from startup targets.'.format(startup, process))
                    continue
                if process in self.__pre_command:
                    rospy.loginfo('Execute command: {}'.format(self.__pre_command[process]))
                    cmd_return_list = subprocess.Popen(self.__pre_command[process],
                                                       shell=True,
                                                       encoding='utf-8',
                                                       stdout=subprocess.PIPE,
                                                       stderr=subprocess.STDOUT,
                                                       universal_newlines=True).stdout.readlines()
                    for cmd_return in cmd_return_list:
                        rospy.loginfo(cmd_return.rstrip("\n"))
                rospy.loginfo('{} will be started.'.format(process))
                # Roslaunch process (ROSLaunchParent) needs to be
                # reinitialized (regenerated) after calling start.
                # If pm (process monitor) exists, the roslaunch process has already been started.
                if self.__roslaunch[process].pm:
                    self.__setup_roslaunch_process(process)
                self.__roslaunch[process].start()

        rospy.logdebug('PlatformManager.__start_roslaunch_process out')

    def __stop_roslaunch_process(self, *, targets=(), suppress_logs=False):
        logdebug(suppress_logs, 'PlatformManager.__stop_roslaunch_process in')

        target_process_keys = set(self.__get_current_roslaunch_process_keys(started_only=True))
        if targets:
            target_process_keys &= targets
        if 'sensor_fusion' in target_process_keys:
            loginfo(suppress_logs, 'Since sensor_fusion node will be terminated, '
                                   'associated ctl_only node will also be stopped (as {}).'
                                   .format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            self.__check_and_call_ctl_command_standby(suppress_logs=suppress_logs)

        if not target_process_keys:
            if targets:
                loginfo(suppress_logs, 'The target nodes({}) was not running.'.format(targets))
            else:
                loginfo(suppress_logs, 'There are no roslaunch process that can be stopped.')

        else:
            for process in target_process_keys:
                self.__roslaunch[process].shutdown()
                loginfo(suppress_logs, '{} has been stopped.'.format(process))

                if process in self.__post_command:
                    loginfo(suppress_logs, 'Execute command: {}'.format(self.__post_command[process]))
                    cmd_return_list = subprocess.Popen(self.__post_command[process],
                                                       shell=True,
                                                       encoding='utf-8',
                                                       stdout=subprocess.PIPE,
                                                       stderr=subprocess.STDOUT,
                                                       universal_newlines=True).stdout.readlines()
                    for cmd_return in cmd_return_list:
                        loginfo(suppress_logs, cmd_return.rstrip("\n"))

        logdebug(suppress_logs, 'PlatformManager.__stop_roslaunch_process out')

    def __cleanup_stale_node(self):
        rospy.logdebug('PlatformManager.__cleanup_stale_node in')
        _, targets = rosnode.rosnode_ping_all()
        if targets:
            rospy.loginfo('Cleanup \"stale\" nodes: {}'.format(targets))
            master = rosgraph.Master(PlatformManager.NODE_ID)
            rosnode.cleanup_master_blacklist(master, targets)
        rospy.logdebug('PlatformManager.__cleanup_stale_node out')

    def __roslaunch_control_based_on_user_roslaunch(self, roslaunch_config):
        rospy.logdebug('PlatformManager.__roslaunch_control_based_on_user_roslaunch in')

        # launch_set[0]=roslaunch_name, launch_set[1]=Start(True)/Stop(False)
        launch_set_list = [(key.replace(PlatformManager.PLATFORM_LAUNCH_PREFIX, ''), bool(param.value))
                           for key, param in roslaunch_config.params.items()
                           if key.startswith(PlatformManager.PLATFORM_LAUNCH_PREFIX)]

        stop_target = [launch_set[0] for launch_set in launch_set_list if launch_set[1] is False]
        start_target = [launch_set[0] for launch_set in launch_set_list if launch_set[1] is True]

        # Stop
        rospy.loginfo('Nodes specified as the stop target in the launch file: {}'.format(
            ', '.join(stop_target) if stop_target else 'None'))
        current_started = set(stop_target) & set(self.__get_current_roslaunch_process_keys(started_only=True))
        rospy.loginfo('Actual nodes to be stopped: {}'.format(
            ', '.join(current_started) if current_started else 'None'))
        if current_started:
            self.__stop_roslaunch_process(targets=current_started)

        # Start
        rospy.loginfo('Nodes specified as the start target in the launch file: {}'.format(
            ', '.join(start_target) if start_target else 'None'))
        current_stopped = set(start_target) & set(self.__get_current_roslaunch_process_keys(stopped_only=True))
        rospy.loginfo('Actual nodes to be started: {}'.format(
            ', '.join(current_stopped) if current_stopped else 'None'))
        if current_stopped:
            self.__start_roslaunch_process(targets=current_stopped)

        # Processing for simulation
        if 'sensor_fusion' in stop_target and self.__run_on_simulator:
            self.__wait_action_server(self.__navigation_start_up_client)
            # make the navigation stand-by
            rospy.loginfo('Stopping navigation node (or stop nav plugin function)...')
            navigation_request = NavigationStartUpGoal()
            navigation_request.command = NavigationStartUpGoal.OFF
            self.__navigation_start_up_client.send_goal_and_wait(navigation_request)
            rospy.loginfo('Navigation node is successfully stopped.')

        rospy.logdebug('PlatformManager.__roslaunch_control_based_on_user_roslaunch out')

    def __preprocess_user_roslaunch_config(self, user, launch):
        rospy.logdebug('PlatformManager.__preprocess_user_roslaunch_config in')

        try:
            # expected arguments = (package-name + relative-roslaunch-filename) = (user + launch)
            launch_files = roslaunch.rlutil.resolve_launch_arguments((user, launch))
            roslaunch_config = roslaunch.config.load_config_default(launch_files, None)

        except Exception as e:
            rospy.logwarn('Invalid roslaunch parameters ({}, {}). {}'.format(user, launch, e))
            rospy.logdebug('PlatformManager.__preprocess_user_roslaunch_config out')
            return None

        #
        # validate roslaunch configuration
        #

        platform_launch_params = [key.replace(PlatformManager.PLATFORM_LAUNCH_PREFIX, '')
                                  for key, param in roslaunch_config.params.items()
                                  if key.startswith(PlatformManager.PLATFORM_LAUNCH_PREFIX)]

        # platform_launch_params (<group ns="platform_launch">) must be present
        if not platform_launch_params:
            rospy.logwarn('Invalid roslaunch file ({}). platform_launch parameters not found.'.format(launch_files))
            rospy.logdebug('PlatformManager.__preprocess_user_roslaunch_config out')
            return None

        # Check for duplicate settings within platform_launch_params.
        if len(platform_launch_params) != len(set(platform_launch_params)):
            rospy.logwarn('Invalid roslaunch file ({}). '
                          'Within platform_launch, parameter name must be unique.'.format(launch_files))
            rospy.logdebug('PlatformManager.__preprocess_user_roslaunch_config out')
            return None

        # all package names specified in platform_launch_params(<group ns="platform_launch">)
        # must be included in the bringup_packages or acceptable_packages
        bringup_package_names = set([p['name'] for p in self.__bringup_packages])
        platform_launch_diff = set(platform_launch_params) - set(bringup_package_names)
        if platform_launch_diff:
            acceptable_packages_diff = platform_launch_diff - set(self.__acceptable_packages)
            if acceptable_packages_diff:
                rospy.logwarn('Invalid roslaunch file ({}). {} cannot be the target of operations.'
                              .format(launch_files, acceptable_packages_diff))
                rospy.logdebug('PlatformManager.__preprocess_user_roslaunch_config out')
                return None

        # the packages defined in non_simultaneous_package_info (rosparam, define in config.yml)
        # cannot be start at the same time
        platform_launch_params_true = [key.replace(PlatformManager.PLATFORM_LAUNCH_PREFIX, '')
                                       for key, param in roslaunch_config.params.items()
                                       if key.startswith(PlatformManager.PLATFORM_LAUNCH_PREFIX)
                                       and param.value is True]
        for non_simultaneous_packages in self.__non_simultaneous_package_info:
            if set(non_simultaneous_packages) & set(platform_launch_params_true) == set(non_simultaneous_packages):
                rospy.logwarn('Invalid roslaunch file ({}). '
                              '{} cannot be started at the same time.'.format(launch_files, non_simultaneous_packages))
                rospy.logdebug('PlatformManager.__preprocess_user_roslaunch_config out')
                return None

        rospy.logdebug('PlatformManager.__preprocess_user_roslaunch_config out')
        return roslaunch_config

    def __start_container(self, image, user, launch):
        rospy.logdebug('PlatformManager.__start_container in')

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
                    mounts=[docker.types.Mount(target=self.__host_ib2_workspace,
                                               source=self.__host_ib2_workspace,
                                               type='bind',
                                               read_only=True)],
                    name=self.__user_container_name,
                    detach=True
                )
                self.__last_image = image
                self.__last_user = user
                self.__last_launch = launch
                rospy.loginfo('Container started. id:{}, image:{}, user:{}, launch:{}'.format(
                    self.__container.id, image, user, launch))
            except Exception as e:
                rospy.logerr(e)
                rospy.logerr('Failed to start container.')
                rospy.logdebug('PlatformManager.__start_container out')
                return False
        else:
            rospy.logwarn('Container already started.')
        rospy.logdebug('PlatformManager.__start_container out')
        return True

    def __stop_and_remove_container(self,  *, suppress_logs=False):
        logdebug(suppress_logs, 'PlatformManager.__stop_and_remove_container in')

        if self.__container is not None:
            loginfo(suppress_logs, 'Stop and remove container: {}'.format(self.__container.id))
            self.__container.stop()
            self.__container.remove()
            self.__container = None
            self.__cleanup_stale_node()

        logdebug(suppress_logs, 'PlatformManager.__stop_and_remove_container out')

    def __apply_current_container_status(self):
        rospy.logdebug('PlatformManager.__apply_current_container_status in')

        if self.__mode != Mode.USER_OFF:
            if not self.__container:
                self.__user_off_processing(warn='Container object not exists.')
            else:
                try:
                    self.__container.reload()
                    if self.__container.status != 'running':
                        self.__user_off_processing(warn='Container is not running status ({}).'
                                                   .format(self.__container.status))

                except Exception as e:
                    self.__user_off_processing(warn=e)

        rospy.logdebug('PlatformManager.__apply_current_container_status out')

    def __user_off_processing(self, *, info=None, warn=None):
        rospy.logdebug('PlatformManager.__user_off_processing in')

        if not self.__is_container_shutdown.flag:
            with self.__is_container_shutdown:
                if warn:
                    rospy.logwarn(warn)
                if info:
                    rospy.loginfo(info)
                self.__stop_and_remove_container()
                self.__current_roslaunch_config = None
                # Restart the flight software's nodes.
                self.__start_roslaunch_process(startup=True)
                if self.__mode != Mode.USER_OFF:
                    self.__set_mode(Mode.USER_OFF)

        rospy.logdebug('PlatformManager.__user_off_processing out')

    def __battery_charge_info_update(self, msg):
        rospy.logdebug('PlatformManager.__battery_charge_info_update in')
        self.__battery_remain = msg.battery_remain

        if self.__battery_remain:
            if self.__battery_remain <= self.__shutdown_battery_remain:
                rospy.logerr('Battery remain is lower than {}, current: {} .'
                             .format(self.__shutdown_battery_remain, self.__battery_remain))
                self.__shutdown()

            if self.__battery_remain > self.__required_battery_remain:
                self.__is_battery_low = False
            elif self.__battery_remain <= self.__required_battery_remain and not self.__is_battery_low:
                self.__set_off_nominal_status_flag(battery_low=True)
                self.__user_off_processing(warn='Battery remain is lower than {}, current: {} .'
                                                .format(self.__required_battery_remain, self.__battery_remain))

        rospy.logdebug('PlatformManager.__battery_charge_info_update out')

    def __user_complete(self, msg):
        rospy.logdebug('PlatformManager.__user_complete in')

        if self.__mode == Mode.USER_IN_PROGRESS:
            rospy.loginfo('Received notification from user program node '
                          'that processing was complete at {}.'
                          .format(msg.data))
            self.__processing_after_user_logic_stop()
            self.__set_mode(Mode.USER_READY)

        rospy.logdebug('PlatformManager.__user_complete out')

    def __cancel_ctl_action(self):
        rospy.logdebug('PlatformManager.__cancel_ctl_action in')
        if (self.__target_action_client.gh is not None and
                self.__target_action_client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]):
            rospy.loginfo('Canceling action of ctl...')
            self.__target_action_client.cancel_goal()
            self.__target_action_client.wait_for_result()
            rospy.loginfo('Successfully canceled action of ctl.')
        rospy.logdebug('PlatformManager.__cancel_ctl_action out')

    def __callback_for_moving_done(self, terminal_state, result):
        rospy.logdebug('PlatformManager.__callback_for_moving_done in')
        if terminal_state == GoalStatus.SUCCEEDED:
            if result.type == CtlCommandResult.TERMINATE_SUCCESS:
                rospy.loginfo('Successfully moved.')
            else:
                rospy.logwarn('Moving FAILED. Result : {}'.format(CTL_RESULT_NAMES[result.type]))
        else:
            rospy.logerr('Action is failed with some error. GoalStatus: {}.'.format(GOAL_STATUS_NAMES[terminal_state]))
        rospy.logdebug('PlatformManager.__callback_for_moving_done out')

    def __get_last_ctl_command_goal(self):
        rospy.logdebug('PlatformManager.__get_last_ctl_command_goal in')
        if self.__target_action_client.gh is not None:
            goal = self.__target_action_client.gh.comm_state_machine.action_goal
        else:
            goal = None
        rospy.logdebug('PlatformManager.__get_last_ctl_command_goal out')
        return goal

    def __ctl_command_feedback(self, ctl_feedback):
        # NOTE: This function is assumed to be the same as the task_manager node implementation, just in case.
        #       It also contains if branches for RELEASE and DOCKING.

        rospy.logdebug('PlatformManager.__ctl_command_feedback in')
        if self.__mode == Mode.USER_IN_PROGRESS:
            rospy.logdebug('PlatformManager.__ctl_command_feedback out')
            return

        last_ctl_command_goal = self.__get_last_ctl_command_goal()
        if (last_ctl_command_goal is None or
                last_ctl_command_goal.goal_id.id != ctl_feedback.status.goal_id.id):
            rospy.logdebug('{} is not the last ctl command executed by platform_manager. Ignore it.'
                           .format(ctl_feedback.status.goal_id.id))
            rospy.logdebug('PlatformManager.__ctl_command_feedback out')
            return

        if self.__last_ctl_command_id != ctl_feedback.status.goal_id.id:
            rospy.loginfo('New Ctl command was called. id: {}'.format(ctl_feedback.status.goal_id.id))
            self.__last_ctl_command_id = ctl_feedback.status.goal_id.id

            if ctl_feedback.feedback.time_to_go.secs > 0:
                self.__last_ctl_command_cancel_execution_time = rospy.Time.now() + rospy.Duration(
                    ctl_feedback.feedback.time_to_go.secs *
                    self.__multipliers_for_action_cancellation_time_calculation)
            else:
                if last_ctl_command_goal.goal.type.type == CtlStatusType.RELEASE:
                    default_time_to_go_secs = self.__default_time_to_go_secs_long
                else:
                    default_time_to_go_secs = self.__default_time_to_go_secs_short
                rospy.logwarn('Re-set the value of time_to_go.secs to {} '
                              'because the actual value ({}) is less than or equal to 0.'
                              .format(default_time_to_go_secs, ctl_feedback.feedback.time_to_go.secs))
                self.__last_ctl_command_cancel_execution_time = rospy.Time.now() + rospy.Duration(
                    default_time_to_go_secs *
                    self.__multipliers_for_action_cancellation_time_calculation)
            rospy.loginfo('Set the timeout time to {}.'.format(
                datetime.fromtimestamp(self.__last_ctl_command_cancel_execution_time.to_sec())))

        # - DOCKING_STAND_BY is not used in platform_flight_software.
        # - If a node other than platform_manager calls /ctl/command,
        #   self.__last_ctl_command_cancel_execution_time may be None.
        # if (self.__target_action_client.get_state() == GoalStatus.ACTIVE and
        #         self.__ctl_status != CtlStatusType.DOCKING_STAND_BY and
        #         rospy.Time.now() > self.__last_ctl_command_cancel_execution_time):
        if (self.__target_action_client.get_state() == GoalStatus.ACTIVE and
                self.__last_ctl_command_cancel_execution_time is not None and
                rospy.Time.now() > self.__last_ctl_command_cancel_execution_time):
            rospy.logerr('Cancel the long non-completed Ctl command.')
            self.__cancel_ctl_action()
            rospy.logerr('Send {} action to ctl node because there may have been an error '
                         'in ctl node or sensor_fusion node'.format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            self.__target_action_client.send_goal(
                self.__generate_ctl_command_goal_with_type_only(CtlStatusType.STAND_BY))
        rospy.logdebug('PlatformManager.__ctl_command_feedback out')

    def __generate_ctl_command_goal_with_type_only(self, goal_type):
        rospy.logdebug('PlatformManager.__generate_ctl_command_goal_with_type_only in')
        goal = CtlCommandGoal(type=CtlStatus(type=goal_type))
        goal.target.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        rospy.logdebug('PlatformManager.__generate_ctl_command_goal_with_type_only out')
        return goal

    def __execute_action_goal(self, goal):
        rospy.logdebug('PlatformManager.__execute_action_goal in')
        rospy.loginfo('PlatformManager has received a control command: {}(value={})'
                      .format(CTL_STATUS_NAMES.get(goal.type.type, 'UNKNOWN'), goal.type.type))

        if goal.type.type not in [CtlStatusType.MOVE_TO_RELATIVE_TARGET,
                                  CtlStatusType.MOVE_TO_ABSOLUTE_TARGET,
                                  CtlStatusType.KEEP_POSE,
                                  CtlStatusType.STOP_MOVING,
                                  CtlStatusType.STAND_BY]:
            rospy.logwarn('Int-Ball2 will only accept the following command: {}, {}, {}, {} and {}'.format(
                CTL_STATUS_NAMES[CtlStatusType.MOVE_TO_RELATIVE_TARGET],
                CTL_STATUS_NAMES[CtlStatusType.MOVE_TO_ABSOLUTE_TARGET],
                CTL_STATUS_NAMES[CtlStatusType.KEEP_POSE],
                CTL_STATUS_NAMES[CtlStatusType.STOP_MOVING],
                CTL_STATUS_NAMES[CtlStatusType.STAND_BY],
            ))
            rospy.logwarn('REJECT your action. (type value: {})'
                          .format(CTL_STATUS_NAMES.get(goal.type.type, 'UNKNOWN')))
            rospy.logdebug('PlatformManager.__execute_action_goal out')
            return

        if self.__operation_type == OperationType.NAV_OFF and goal.type.type in [
                CtlStatusType.MOVE_TO_RELATIVE_TARGET,
                CtlStatusType.MOVE_TO_ABSOLUTE_TARGET,
                CtlStatusType.KEEP_POSE,
                CtlStatusType.STOP_MOVING]:
            rospy.logwarn('When in {} mode, the guidance control cannot be executed (Only {} is accepted)'
                          .format(MODE_NAMES[OperationType.NAV_OFF], CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            rospy.logwarn('REJECT your action. (type value: {})'
                          .format(CTL_STATUS_NAMES.get(goal.type.type, 'UNKNOWN')))
            rospy.logdebug('PlatformManager.__execute_action_goal out')
            return

        # update timestamp as this system
        telecommand_stamp = goal.target.header.stamp
        goal.target.header.stamp = rospy.Time.now()
        rospy.loginfo('Timestamp in goal action is updated as the system time. (source: {})(new: {})'.format(
            datetime.fromtimestamp(telecommand_stamp.to_sec()),
            datetime.fromtimestamp(goal.target.header.stamp.to_sec())))

        rospy.loginfo('PlatformManager get message goal:\n{}'.format(goal))
        if (goal.type.type in [CtlStatusType.KEEP_POSE, CtlStatusType.STOP_MOVING] and
                self.__target_action_client.gh is not None and
                self.__target_action_client.get_state() == GoalStatus.ACTIVE):
            rospy.loginfo('Cancel the current ctl command because STOP command ({}) was requested'
                          .format(CTL_STATUS_NAMES[goal.type.type]))
            self.__cancel_ctl_action()
        elif goal.type.type == CtlStatusType.STAND_BY:
            rospy.logwarn('Send {} action to ctl node.'.format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            self.__target_action_client.send_goal(
                self.__generate_ctl_command_goal_with_type_only(CtlStatusType.STAND_BY))
        else:
            self.__target_action_client.send_goal(goal, done_cb=self.__callback_for_moving_done)

        rospy.logdebug('PlatformManager.__execute_action_goal out')

    def __check_and_call_ctl_command_standby(self, *, suppress_logs=False):
        logdebug(suppress_logs, 'PlatformManager.__check_and_call_ctl_command_standby in')

        try:
            ctl_status = rospy.wait_for_message('/ctl/status',
                                                CtlStatus,
                                                timeout=rospy.Duration(self.__waiting_time_for_topic))
            if ctl_status.type.type != CtlStatusType.STAND_BY:
                goal = self.__generate_ctl_command_goal_with_type_only(CtlStatusType.STAND_BY)
                self.__target_action_client.send_goal_and_wait(goal)
                loginfo(suppress_logs, 'ctl_only node is successfully stopped (as {}).'
                        .format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
            else:
                loginfo(suppress_logs, 'ctl_only node is already stopped (as {}).'
                        .format(CTL_STATUS_NAMES[CtlStatusType.STAND_BY]))
        except Exception:
            loginfo(suppress_logs, 'ctl_only node is not running.')

        logdebug(suppress_logs, 'PlatformManager.__check_and_call_ctl_command_standby out')

    def __wait_action_server(self, simple_action_client, raise_wait_error=False):
        rospy.logdebug('PlatformManager.__wait_action_server in')

        wait_result = simple_action_client.wait_for_server(
                            rospy.Duration(self.__waiting_time_for_server))
        if not wait_result:
            rospy.logerr('{} has not started after {} seconds.'
                         .format(simple_action_client.action_client.ns,
                                 self.__waiting_time_for_server))
            if raise_wait_error:
                rospy.logerr('PlatformManager will stop.')
                rospy.logdebug('PlatformManager.__wait_action_server out')
                raise

        rospy.logdebug('PlatformManager.__wait_action_server out')
        return bool(wait_result)

    def __nav_off_processing(self, *, raise_wait_error=False):
        rospy.logdebug('PlatformManager.__nav_off_processing in')

        if self.__use_ctl:
            self.__wait_action_server(self.__target_action_client, raise_wait_error=raise_wait_error)
            self.__check_and_call_ctl_command_standby()

        if self.__use_nav:
            self.__wait_action_server(self.__navigation_start_up_client, raise_wait_error=raise_wait_error)
            # make the navigation stand-by
            rospy.loginfo('Stopping navigation node...')
            navigation_request = NavigationStartUpGoal()
            navigation_request.command = NavigationStartUpGoal.OFF
            self.__navigation_start_up_client.send_goal_and_wait(navigation_request)
            rospy.loginfo('Navigation node is successfully stopped.')

        rospy.logdebug('PlatformManager.__nav_off_processing out')

    def __nav_on_processing(self, *, raise_wait_error=False):
        rospy.logdebug('PlatformManager.__nav_on_processing in')

        execute_ctl_on = self.__use_ctl

        if self.__use_nav:
            if not self.__navigation_start_up(raise_wait_error=raise_wait_error):
                execute_ctl_on = False

        if execute_ctl_on:
            if self.__wait_action_server(self.__target_action_client, raise_wait_error=raise_wait_error):
                # make the ctl on
                rospy.loginfo('Start ctl node...(as {})'.format(CTL_STATUS_NAMES[CtlStatusType.KEEP_POSE]))
                goal = self.__generate_ctl_command_goal_with_type_only(CtlStatusType.KEEP_POSE)
                # do not wait here because detect errors in __ctl_command_feedback
                self.__target_action_client.send_goal(goal)
                rospy.loginfo('Ctl node will start.')

        rospy.logdebug('PlatformManager.__nav_on_processing out')

    def __navigation_start_up(self, *, raise_wait_error=False):
        rospy.logdebug('PlatformManager.__navigation_start_up in')

        result = False

        if self.__wait_action_server(self.__navigation_start_up_client, raise_wait_error=raise_wait_error):
            # make the slam on
            if not self.__run_on_simulator:
                try:
                    rospy.wait_for_service('/slam_wrapper/switch_power', 
                                           rospy.Duration(self.__waiting_time_for_server))
                    rospy.loginfo('Notifies slam_wrapper node to start processing (calls switch_power service).')
                    self.__slam_wrapper_switch_power(SwitchPowerRequest(power=PowerStatus(status=PowerStatus.ON)))
                except Exception as e:
                    rospy.logerr(e)
                    rospy.logdebug('PlatformManager.__navigation_start_up out')
                    return False

            # make the navigation on
            rospy.loginfo('Start navigation node (or start nav plugin function)...')
            if self.__check_navigation_startup_on():
                rospy.loginfo('Navigation node (or nav plugin) is successfully started.')
                result = True
            else:
                rospy.logerr('Navigation node (or nav plugin) could not be started.')

        rospy.logdebug('PlatformManager.__navigation_start_up out')
        return result

    def __set_mode(self, mode):
        rospy.logdebug('PlatformManager.__set_mode in')
        if mode != self.__mode:
            self.__mode = mode
            rospy.loginfo('Mode changed to {}'.format(MODE_NAMES[self.__mode]))
            if self.__mode in [Mode.USER_OFF, Mode.USER_READY]:
                # if self.__operation_type is undefined (None), no action is taken
                if self.__operation_type == OperationType.NAV_OFF:
                    self.__nav_off_processing()
                elif self.__operation_type == OperationType.NAV_ON:
                    self.__nav_on_processing()
        rospy.logdebug('PlatformManager.__set_mode out')

    def __set_operation_type(self, operation_type, *, raise_wait_error=False):
        rospy.logdebug('PlatformManager.__set_operation_type in')
        if operation_type != self.__operation_type:
            self.__operation_type = operation_type
            rospy.loginfo('Operation type changed to {}'.format(OPERATION_TYPE_NAMES[self.__operation_type]))
            if self.__operation_type == OperationType.NAV_OFF:
                self.__nav_off_processing(raise_wait_error=raise_wait_error)
            elif self.__operation_type == OperationType.NAV_ON:
                self.__nav_on_processing(raise_wait_error=raise_wait_error)
        rospy.logdebug('PlatformManager.__set_operation_type out')

    def __set_off_nominal_status_flag(self, *, battery_low=False, short_disk_space=False,
                                      temperature_to_cool=False, 
                                      wifi_disconnected=False):
        rospy.logdebug('PlatformManager.__set_off_nominal_status_flag in')

        if battery_low:
            self.__is_battery_low = True
        if short_disk_space:
            self.__is_short_disk_space = True
        if temperature_to_cool:
            self.__is_cooling = True
        if wifi_disconnected:
            self.__is_wifi_disconnected = True

        if [self.__is_battery_low, self.__is_short_disk_space, self.__is_cooling].count(True) >= 2:
            rospy.logerr('Multiple off-nominal events were detected simultaneously.')
            rospy.logerr('Off-nominal event statuses: Battery:{}, Disk space:{}, Temperature:{}.'
                         .format(self.__is_battery_low, self.__is_short_disk_space, self.__is_cooling))
            self.__shutdown()

        rospy.logdebug('PlatformManager.__set_off_nominal_status_flag out')

    def __check_short_disk_space(self):
        rospy.logdebug('PlatformManager.__check_short_disk_space in')
        short_disk_space = False
        shutdown_required = False
        for disk_space in self.__system_status.disk_spaces:
            if disk_space.remain <= self.__shutdown_storage_ratio:
                rospy.logerr('Disk space is lower than {} %, current: {} %. The system shutdown will be executed.'
                             '(path: {})'.format(self.__shutdown_storage_ratio, disk_space.remain, disk_space.path))
                shutdown_required = True
            elif disk_space.remain <= self.__required_storage_ratio:
                rospy.logwarn('Disk space is lower than {} %, current: {} %.'
                              '(path: {})'.format(self.__required_storage_ratio, disk_space.remain, disk_space.path))
                short_disk_space = True
        rospy.logdebug('PlatformManager.__check_short_disk_space out')
        return short_disk_space, shutdown_required

    def __system_status_update(self, msg):
        rospy.logdebug('PlatformManager.__system_status_update in')
        self.__prev_system_status = self.__system_status
        self.__system_status = msg
        # operation for disk-space status
        short_disk_space, shutdown_required = self.__check_short_disk_space()
        if shutdown_required:
            rospy.logerr('Force termination of platform_manager node.')
            os._exit(1)

        if not short_disk_space:
            self.__is_short_disk_space = False
        elif short_disk_space and not self.__is_short_disk_space:
            self.__set_off_nominal_status_flag(short_disk_space=True)
            self.__user_off_processing()

        # operation for wifi-connection status
        if self.__system_status.wifi_connected:
            self.__last_wifi_connected_time = self.__system_status.check_time
            self.__is_wifi_disconnected = False
        else:
            disconnected_duration = self.__system_status.check_time - self.__last_wifi_connected_time
            if disconnected_duration > self.__wifi_duration:
                if not self.__is_wifi_disconnected:
                    self.__set_off_nominal_status_flag(wifi_disconnected=True)
                    self.__user_off_processing(warn='Wifi has been disconnected for {} seconds.'
                                                    .format(disconnected_duration.to_sec()))
            else:
                self.__is_wifi_disconnected = False

        # operation for temperature status
        if self.__system_status.temperature >= self.__temperature_to_shutdown:
            rospy.logwarn('Temperature is over {} degrees (current: {} degrees).'
                          .format(self.__temperature_to_shutdown, self.__system_status.temperature))
            self.__shutdown()
        elif self.__system_status.temperature >= self.__temperature_to_cool and not self.__is_cooling:
            self.__set_off_nominal_status_flag(temperature_to_cool=True)
            self.__user_off_processing(warn='Temperature is over {} degrees (current: {} degrees). '
                                            'Stop user processing.'
                                            .format(self.__temperature_to_cool, self.__system_status.temperature))
        elif self.__is_cooling and self.__system_status.temperature <= self.__temperature_to_revive:
            rospy.loginfo('Temperature come under {} degrees (current: {} degrees).'
                          .format(self.__temperature_to_revive, self.__system_status.temperature))
            self.__is_cooling = False

        rospy.logdebug('PlatformManager.__system_status_update out')

    def __check_navigation_startup_on(self):
        rospy.logdebug('PlatformManager.__check_navigation_startup_on in')

        navigation_request = NavigationStartUpGoal()
        navigation_request.command = NavigationStartUpGoal.ON
        self.__navigation_start_up_client.send_goal_and_wait(navigation_request)
        navigation_start_up_result = self.__navigation_start_up_client.get_result()

        rospy.logdebug('PlatformManager.__check_navigation_startup_on out')
        return (navigation_start_up_result and
                navigation_start_up_result.type == NavigationStartUpResult.ON_READY)

    def __reboot(self, msg):
        rospy.logdebug('PlatformManager.__reboot in')
        if not self.__enable_shutdown:
            rospy.logwarn('Cancel the reboot because the setting \"enable_shutdown\" is False.')
            rospy.logdebug('PlatformManager.__reboot out')
            return
        rospy.logwarn('REBOOT THE SYSTEM.')
        subprocess.call('/usr/bin/sudo /sbin/shutdown -r now', shell=True)
        rospy.logerr('Reboot failed.')
        rospy.logdebug('PlatformManager.__reboot out')

    def __shutdown(self):
        rospy.logdebug('PlatformManager.__shutdown in')
        if not self.__enable_shutdown:
            rospy.logwarn('Cancel the shutdown because the setting \"enable_shutdown\" is False.')
            rospy.logdebug('PlatformManager.__shutdown out')
            return
        rospy.logwarn('SHUTDOWN THE SYSTEM.')
        subprocess.call('/usr/bin/sudo /sbin/shutdown -P now', shell=True)
        rospy.logerr('Shutdown failed.')
        rospy.logdebug('PlatformManager.__shutdown out')

    def __publish_status(self):
        rospy.logdebug('PlatformManager.__publish_status in')
        msg = ManagerStatus(
            stamp=rospy.Time.now(),
            type=OperationType(type=self.__operation_type),
            mode=(Mode(mode=self.__mode)),
            start_container=bool(self.__container),
            last_user_logic=self.__last_user_logic,
            last_image=self.__last_image,
            last_user=self.__last_user,
            last_launch=self.__last_launch,
        )
        self.__status_publisher.publish(msg)
        rospy.logdebug('PlatformManager.__publish_status out')

    def __publish_led_colors(self):
        rospy.logdebug('PlatformManager.__publish_led_colors in')
        if self.__mode != Mode.USER_IN_PROGRESS:
            self.__led_color_publisher_left.publish([self.__color_with_camera_mic] * 8)
            self.__led_color_publisher_right.publish([self.__color_with_camera_mic] * 8)
        rospy.logdebug('PlatformManager.__publish_led_colors in')

    def __publish_fan_duty(self):
        rospy.logdebug('PlatformManager.__publish_led_colors in')
        if self.__operation_type == OperationType.NAV_OFF and self.__mode != Mode.USER_IN_PROGRESS:
            self.__fan_duty_publisher.publish(Float64MultiArray(data=[0.0] * 8))
        rospy.logdebug('PlatformManager.__publish_led_colors in')

    def run(self):
        rospy.logdebug('PlatformManager.run in')
        rospy.loginfo('PlatformManager has started.')

        while not rospy.is_shutdown():
            self.__apply_current_container_status()
            self.__publish_status()
            self.__publish_led_colors()
            self.__publish_fan_duty()
            self.__rate.sleep()

        rospy.loginfo('PlatformManager has finished.')
        rospy.logdebug('PlatformManager.run out')


if __name__ == '__main__':
    platform_manager = PlatformManager()
    platform_manager.run()
