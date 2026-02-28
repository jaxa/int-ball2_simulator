#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import math
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
)
from ib2_msgs.msg import (
    CtlStatus,
    CtlStatusType,
    Navigation,
)
from ib2_msgs.action import (
    CtlCommand,
    NavigationStartUp,
)
from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import StopProcessingUserNode
from std_msgs.msg import (
    Header,
    Float64MultiArray,
    Int8,
)
from builtin_interfaces.msg import Time

# ***********************************************************************************
# To convert quaternions in ROS applications, the tf(tf2) package is usually used.
# However, to import tf(tf2) package from Python3 in ROS melodic,
# the libraries needs to be rebuilt.
# In this source code, we use the quaternion handling functions from the tf package.
# See the following URL for the source code details.
# https://github.com/ros/geometry/blob/melodic-devel/tf/src/tf/transformations.py

import numpy

# transformations.py

# Copyright (c) 2006, Christoph Gohlke
# Copyright (c) 2006-2009, The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holders nor the names of any
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}


_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]


def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.
    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple
    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
    True
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = numpy.empty((4, ), dtype=numpy.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion


def quaternion_multiply(quaternion1, quaternion0):
    """Return multiplication of two quaternions.
    >>> q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> numpy.allclose(q, [-44, -14, 48, 28])
    True
    """
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return numpy.array((
        x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
        x1*y0 - y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=numpy.float64)

# ***********************************************************************************


class UserProgramInterface(Node):

    MAX_MSG_SIZE = 800
    TORQUE_GAIN_NORMAL_MODE = 0
    TORQUE_GAIN_IB2_MODE = 1

    def __init__(self):
        super().__init__('user_program_interface')

        self.__current_logic = None
        self.__logic_in_progress = False
        self.__status = ''
        self.__stop_requested = False
        self.__thread = None
        # Store goal handle for cancellation
        self.__ctl_command_goal_handle = None

        # Use ReentrantCallbackGroup for action clients to avoid deadlocks
        self._action_cb_group = ReentrantCallbackGroup()

        # Publisher
        self.__status_publisher = self.create_publisher(
            UserNodeStatus, '/ib2_user/status', 1)
        self.__complete_publisher = self.create_publisher(
            Time, '/ib2_user/complete', 1)
        self.__gc_wrench_mode_publisher = self.create_publisher(
            Int8, '/ib2_user/guidance_control/wrench_mode', 1)

        # Subscriber
        self.__subscriber_start = self.create_subscription(
            UserLogic, '/ib2_user/start', self.__callback_start, 10)
        self.__subscriber_torque_gain = self.create_subscription(
            Float64MultiArray, '/ib2_user/guidance_control/torque_gain',
            self.__callback_torque_gain, 10)

        # Service server
        self.__stop_processing_server = self.create_service(
            StopProcessingUserNode, '/ib2_user/stop', self.__stop_processing)

        # Action client (with ReentrantCallbackGroup)
        self.__ctl_command_client = ActionClient(
            self, CtlCommand, '/ctl/command',
            callback_group=self._action_cb_group)
        self.__navigation_start_up_client = ActionClient(
            self, NavigationStartUp, '/sensor_fusion/navigation_start_up',
            callback_group=self._action_cb_group)

        # Status publishing timer (1 Hz)
        self.__status_timer = self.create_timer(1.0, self.__publish_status)

        # __process_count_up
        self.__status_count = 0

    def __wait_future(self, future, timeout_sec=120.0):
        """Poll-wait for a future to complete without calling spin."""
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > timeout_sec:
                self.get_logger().warn(
                    'Future wait timed out after {} seconds'.format(timeout_sec))
                return False
            time.sleep(0.05)
        return True

    def __publish_status(self):
        """Timer callback: publish status at 1 Hz."""
        msg = list(self.__status.encode(
            encoding='utf-8')[:UserProgramInterface.MAX_MSG_SIZE])
        if len(msg) < UserProgramInterface.MAX_MSG_SIZE:
            msg.extend(
                [0] * (UserProgramInterface.MAX_MSG_SIZE - len(msg)))
        self.__status_publisher.publish(UserNodeStatus(
            stamp=self.get_clock().now().to_msg(),
            msg=msg
        ))

    def __generate_ctl_command_goal(self, goal_type, position, orientation):
        """
            Generate a request for /ctl/command
        """
        goal_msg = CtlCommand.Goal()
        goal_msg.type = CtlStatusType(type=goal_type)
        goal_msg.target = PoseStamped(
            header=Header(stamp=self.get_clock().now().to_msg()),
            pose=Pose(position=position, orientation=orientation)
        )
        return goal_msg

    def __generate_ctl_command_goal_with_type_only(self, goal_type):
        """
            Generate a request for /ctl/command that does not include
            position and orientation specifications (e.g. KEEP_POSE).
        """
        goal_msg = CtlCommand.Goal()
        goal_msg.type = CtlStatusType(type=goal_type)
        # Even in cases where the Pose value is not used (e.g. KEEP_POSE),
        # it is necessary to set a valid quaternion value.
        goal_msg.target.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return goal_msg

    def __send_goal_and_wait(self, action_client, goal_msg):
        """
            Send a goal to an action client and wait for the result.
            Returns the goal handle, or None if not accepted.
        """
        goal_handle_future = action_client.send_goal_async(goal_msg)
        if not self.__wait_future(goal_handle_future, timeout_sec=10.0):
            return None
        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal was rejected by action server')
            return None

        result_future = goal_handle.get_result_async()
        self.__wait_future(result_future, timeout_sec=120.0)
        return goal_handle

    def __cancel_ctl_command(self):
        """
            If /ctl/command is running, cancel it.
        """
        if self.__ctl_command_goal_handle is not None:
            cancel_future = self.__ctl_command_goal_handle.cancel_goal_async()
            self.__wait_future(cancel_future, timeout_sec=10.0)
            self.__ctl_command_goal_handle = None
            self.get_logger().info('Successfully canceled action of ctl.')

    def __wait_for_ctl_command(self):
        """
            Wait until /ctl/command is ready to be called.
        """
        wait_result = False
        while not self.__stop_requested and not wait_result:
            wait_result = self.__ctl_command_client.wait_for_server(timeout_sec=1.0)

        return bool(not self.__stop_requested)

    def __send_ctl_command_goal_and_wait(self, goal_msg):
        """
            Send a ctl command goal and wait for the result.
            Stores the goal handle for potential cancellation.
        """
        goal_handle_future = self.__ctl_command_client.send_goal_async(goal_msg)
        if not self.__wait_future(goal_handle_future, timeout_sec=10.0):
            self.get_logger().error('Ctl command send_goal timed out')
            return
        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Ctl command goal was rejected')
            return
        self.__ctl_command_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self.__wait_future(result_future, timeout_sec=120.0)
        self.__ctl_command_goal_handle = None

    def __process_keep_pose(self):
        """
            KEEP_POSE only
        """
        self.__status = 'KEEP_POSE'
        if not self.__wait_for_ctl_command():
            return
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal_with_type_only(
                CtlStatusType.KEEP_POSE)
        )

    def __process_absolute_target_origin(self):
        """
            MOVE_TO_ABSOLUTE_TARGET Origin
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        # position is set in meters
        position = Point(x=0.0, y=0.0, z=0.0)
        # Set the quaternion directly
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_ABSOLUTE_TARGET, position, orientation)
        )

    def __process_absolute_target_001(self):
        """
            MOVE_TO_ABSOLUTE_TARGET 1
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        # position is set in meters
        position = Point(x=1.0, y=0.0, z=0.0)
        # Set the quaternion directly
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_ABSOLUTE_TARGET, position, orientation)
        )

    def __process_absolute_target_home(self):
        """
            MOVE_TO_ABSOLUTE_TARGET 1
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        # position is set in meters
        position = Point(x=1.0, y=0.0, z=-1.0)
        # Set the quaternion directly
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_ABSOLUTE_TARGET, position, orientation)
        )

    def __process_absolute_target_002(self):
        """
            MOVE_TO_ABSOLUTE_TARGET 1
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        # position is set in meters
        position = Point(x=2.0, y=0.0, z=-1.0)
        # Set the quaternion directly
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_ABSOLUTE_TARGET, position, orientation)
        )

    def __process_relative_target_001(self):
        """
            MOVE_TO_RELATIVE_TARGET 1
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        position = Point(x=1.5, y=0.3, z=-1.0)
        # Convert euler angles to quaternion.
        # The arguments are euler's roll, pitch and yaw angles (radian)
        quaternion_values = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        orientation = Quaternion(
            x=quaternion_values[0],
            y=quaternion_values[1],
            z=quaternion_values[2],
            w=quaternion_values[3],
        )
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_RELATIVE_TARGET, position, orientation)
        )

    def __process_relative_target_002(self):
        """
            MOVE_TO_RELATIVE_TARGET 2
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        position = Point(x=-0.2, y=0.5, z=0.0)
        quaternion_values_1 = quaternion_from_euler(math.pi / 2.0, 0.0, 0.0)
        quaternion_values_2 = quaternion_from_euler(0.0, math.pi / 2.0, 0.0)
        quaternion_values = quaternion_multiply(quaternion_values_1, quaternion_values_2)
        orientation = Quaternion(
            x=quaternion_values[0],
            y=quaternion_values[1],
            z=quaternion_values[2],
            w=quaternion_values[3],
        )
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_RELATIVE_TARGET, position, orientation)
        )

    def __process_relative_target_003(self):
        """
            MOVE_TO_RELATIVE_TARGET 3
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        position = Point(x=-0.2, y=0.2, z=0.2)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_RELATIVE_TARGET, position, orientation)
        )

    def __process_relative_target_004(self):
        """
            MOVE_TO_RELATIVE_TARGET 4
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return
        position = Point(x=-0.0, y=0.0, z=0.0)
        quaternion_values = quaternion_from_euler(
            math.pi / 4.0, math.pi / 4.0, math.pi / 2.0)
        orientation = Quaternion(
            x=quaternion_values[0],
            y=quaternion_values[1],
            z=quaternion_values[2],
            w=quaternion_values[3],
        )
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_RELATIVE_TARGET, position, orientation)
        )

    def __process_relative_target_x_P05(self):
        """
            MOVE_TO_RELATIVE_TARGET
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        position = Point(x=0.5, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_RELATIVE_TARGET, position, orientation)
        )

    def __process_relative_target_yaw_Ppi(self):
        """
            MOVE_TO_RELATIVE_TARGET
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        position = Point(x=0.0, y=0.0, z=0.0)
        quaternion_values = quaternion_from_euler(0, 0, math.pi)
        orientation = Quaternion(
            x=quaternion_values[0],
            y=quaternion_values[1],
            z=quaternion_values[2],
            w=quaternion_values[3],
        )
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_RELATIVE_TARGET, position, orientation)
        )

    def __process_relative_target_005(self):
        """
            MOVE_TO_RELATIVE_TARGET 5
        """
        self.__status = sys._getframe().f_code.co_name
        if not self.__wait_for_ctl_command():
            return

        position = Point(x=0.2, y=0.0, z=0.0)
        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.__send_ctl_command_goal_and_wait(
            self.__generate_ctl_command_goal(
                CtlStatusType.MOVE_TO_RELATIVE_TARGET, position, orientation)
        )

    def __process_count_up(self):
        """
            Count up test.
        """
        self.__status_count = 0

        while self.__status_count < 300 and not self.__stop_requested:
            self.__status = 'count: {}'.format(self.__status_count)
            self.__status_count = self.__status_count + 1
            time.sleep(1.0)

    def __process_complete(self):
        """
            Notify the Int-Ball2 management software that the process is complete.
        """
        self.__status = sys._getframe().f_code.co_name

        # Finish processing
        self.__logic_in_progress = False
        self.__status = ''
        self.__current_logic = None
        self.get_logger().info('finish processing')

        self.__complete_publisher.publish(
            Time(sec=self.get_clock().now().to_msg().sec,
                 nanosec=self.get_clock().now().to_msg().nanosec))

    def __process_execution(self, process_list):
        for process in process_list:
            if self.__stop_requested:
                break
            self.get_logger().info('Call {}'.format(process.__name__))
            process()

    def __process_idling(self, process_list: list):
        process = process_list[0]
        while True:
            if self.__stop_requested:
                break
            time.sleep(0.1)
        self.get_logger().info('Call {}'.format(process.__name__))
        process()

    def __callback_start(self, msg):
        self.get_logger().info('start {}'.format(msg))

        # Start navigation node
        waiting_time_for_action_server = 20
        self.get_logger().info('Waiting for navigation_start_up server...')
        wait_result = self.__navigation_start_up_client.wait_for_server(
                          timeout_sec=float(waiting_time_for_action_server))
        if not wait_result:
            self.__status = '{} has not started after {} seconds.'.format(
                                '/sensor_fusion/navigation_start_up',
                                waiting_time_for_action_server)
            self.get_logger().error(self.__status)
            self.__process_complete()
            return

        self.get_logger().info('Navigation server found, sending ON goal...')
        navigation_request = NavigationStartUp.Goal()
        navigation_request.command = NavigationStartUp.Goal.ON

        goal_handle_future = self.__navigation_start_up_client.send_goal_async(
            navigation_request)
        if not self.__wait_future(goal_handle_future, timeout_sec=10.0):
            self.__status = 'Navigation start up send_goal timed out.'
            self.get_logger().error(self.__status)
            self.__process_complete()
            return
        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            self.__status = 'Navigation start up goal was rejected.'
            self.get_logger().error(self.__status)
            self.__process_complete()
            return

        self.get_logger().info('Navigation goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        if not self.__wait_future(result_future, timeout_sec=30.0):
            self.__status = 'Navigation start up result timed out.'
            self.get_logger().error(self.__status)
            self.__process_complete()
            return
        navigation_start_up_result = result_future.result().result
        self.get_logger().info('Navigation result: type={}'.format(
            navigation_start_up_result.type if navigation_start_up_result else 'None'))

        if not (navigation_start_up_result and
                navigation_start_up_result.type == NavigationStartUp.Result.ON_READY):
            self.__status = 'Navigation node could not be started.'
            self.get_logger().error(self.__status)
            self.__process_complete()
            return
        self.get_logger().info('Navigation started, dispatching id={}'.format(msg.id))

        if self.__logic_in_progress:
            self.__status = 'Another logic is running: {}'.format(
                self.__current_logic)
            self.get_logger().info(self.__status)
            return

        if msg.id == 1:
            # Set wrench mode : Normal
            self.__publish_wrench_mode(
                UserProgramInterface.TORQUE_GAIN_NORMAL_MODE)
            process_list = [
                self.__process_complete,
            ]
            self.__thread = threading.Thread(
                target=self.__process_idling, args=[process_list],
                daemon=True)
            self.__thread.start()

        elif msg.id == 2:
            # Set wrench mode : Forward to IB2 mode
            self.__publish_wrench_mode(
                UserProgramInterface.TORQUE_GAIN_IB2_MODE)
            process_list = [
                self.__process_complete,
            ]
            self.__thread = threading.Thread(
                target=self.__process_idling, args=[process_list],
                daemon=True)
            self.__thread.start()

        elif msg.id == 3:
            process_list = [
                self.__process_keep_pose,
                self.__process_complete,
            ]
            self.__thread = threading.Thread(
                target=self.__process_execution, args=[process_list],
                daemon=True)
            self.__thread.start()

        elif msg.id == 4:
            # Count up test
            process_list = [
                self.__process_count_up,
                self.__process_complete,
            ]
            self.__thread = threading.Thread(
                target=self.__process_execution, args=[process_list],
                daemon=True)
            self.__thread.start()

        elif msg.id == 5:
            # MOVE_TO_ABSOLUTE_TARGET and MOVE_TO_RELATIVE_TARGET
            process_list = [
                # self.__process_absolute_target_001,
                # self.__process_absolute_target_002,
                # self.__process_absolute_target_home,
                # self.__process_relative_target_001,
                # self.__process_relative_target_002,
                # self.__process_relative_target_003,
                # self.__process_relative_target_004,
                self.__process_complete,
            ]
            self.__thread = threading.Thread(
                target=self.__process_execution, args=[process_list],
                daemon=True)
            self.__thread.start()

        elif msg.id == 6:
            process_list = [
                self.__process_relative_target_x_P05,
                self.__process_relative_target_yaw_Ppi,
                self.__process_relative_target_x_P05,
                self.__process_relative_target_yaw_Ppi,
                self.__process_complete,
            ]
            self.__thread = threading.Thread(
                target=self.__process_execution, args=[process_list],
                daemon=True)
            self.__thread.start()

        else:
            self.__status = 'Unimplemented logic: {}'.format(msg)
            self.get_logger().warning(self.__status)
            return

        self.__logic_in_progress = True
        self.__current_logic = msg

    def __stop_processing(self, request, response):
        self.__stop_requested = True
        self.__cancel_ctl_command()
        if self.__thread:
            self.__thread.join(timeout=10.0)
            self.__thread = None

        # Finish processing
        self.__logic_in_progress = False
        self.__status = ''
        self.__current_logic = None
        self.get_logger().info('finish processing')

        self.__stop_requested = False
        response.result = StopProcessingUserNode.Response.SUCCESS
        return response

    def __publish_wrench_mode(self, mode: int):
        ros_msg_int = Int8(data=mode)
        self.__gc_wrench_mode_publisher.publish(ros_msg_int)

        if mode == UserProgramInterface.TORQUE_GAIN_NORMAL_MODE:
            self.__status = 'Set wrench mode : Normal'

        elif mode == UserProgramInterface.TORQUE_GAIN_IB2_MODE:
            self.__status = 'Set wrench mode : Forward to IB2 mode'

        self.get_logger().info(self.__status)

    def __callback_torque_gain(self, float64MultiArrayMsg):
        torque_gain = float64MultiArrayMsg.data
        self.__status = (
            f'Set torque gain : [{torque_gain[0]}, '
            f'{torque_gain[1]}, {torque_gain[2]}]')
        self.get_logger().info(self.__status)


def main(args=None):
    rclpy.init(args=args)
    node = UserProgramInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
