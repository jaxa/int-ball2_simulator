#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import (
    WrenchStamped,
)
from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import (
    StopProcessingUserNode,
)
from std_msgs.msg import (
    Float64MultiArray,
)
from builtin_interfaces.msg import Time


class TestFanDutyNode(Node):

    MAX_MSG_SIZE = 800

    def __init__(self):
        super().__init__('user_template')

        self.__current_logic = None
        self.__logic_in_progress = False
        self.__status = ''
        self.__stop_requested = False
        self.__thread = None

        # Publisher
        self.__status_publisher = self.create_publisher(UserNodeStatus, '/ib2_user/status', 1)
        self.__complete_publisher = self.create_publisher(Time, '/ib2_user/complete', 1)

        # Subscriber
        self.__subscriber_start = self.create_subscription(
            UserLogic, '/ib2_user/start', self.__callback_start, 10)

        # Service server
        self.__stop_processing_server = self.create_service(
            StopProcessingUserNode, '/ib2_user/stop', self.__stop_processing)

        #####################################################
        # Variables specific to each processing process
        #####################################################
        self.__last_ctl_wrench = None
        self.__last_fan_duty = None

        # Subscriber
        self.__subscriber_ctl_wrench = self.create_subscription(
            WrenchStamped, '/ctl/wrench', self.__callback_ctl_wrench, 10)

        # Publisher
        self.__fan_duty_publisher = self.create_publisher(Float64MultiArray, '/ctl/duty', 1)

        # Timer for periodic status publishing (replaces rospy.Rate loop)
        self.__timer = self.create_timer(1.0, self.__publish_status)

    def __publish_status(self):
        msg = list(self.__status.encode(encoding='utf-8')[:TestFanDutyNode.MAX_MSG_SIZE])
        if len(msg) < TestFanDutyNode.MAX_MSG_SIZE:
            msg.extend([0] * (TestFanDutyNode.MAX_MSG_SIZE - len(msg)))
        now = self.get_clock().now().to_msg()
        self.__status_publisher.publish(UserNodeStatus(
            stamp=now,
            msg=msg
        ))

    def __callback_ctl_wrench(self, msg):
        """
            Subscribe force and torque.
        """
        self.__last_ctl_wrench = msg.wrench
        self.__status = '/ctl/wrench: \n{} \n/fan/duty: {}'.format(self.__last_ctl_wrench, self.__last_fan_duty)

    def __process_publish_fan_duty(self):
        """
            PWM control of fans.
        """

        sleep_duration = 5.0  # seconds

        def publish_and_sleep(fan_duty_msg):
            self.__fan_duty_publisher.publish(fan_duty)
            self.__last_fan_duty = fan_duty
            self.__status = '/ctl/wrench: {}, /fan/duty: {}'.format(self.__last_ctl_wrench, self.__last_fan_duty)
            time.sleep(sleep_duration)

        # fan_duty must be defined as an 8-element array.
        # One element is the control value of one fan.
        # The value of each element is a small number between 0 and 1.
        fan_duty = Float64MultiArray(
            data=[0.3] * 8
        )
        publish_and_sleep(fan_duty)

        # To stop all fans, set all values to zero.
        fan_duty = Float64MultiArray(
            data=[0.0] * 8
        )
        publish_and_sleep(fan_duty)

        fan_duty = Float64MultiArray(
            data=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
        )
        publish_and_sleep(fan_duty)

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

        now = self.get_clock().now().to_msg()
        self.__complete_publisher.publish(Time(sec=now.sec, nanosec=now.nanosec))

    def __process_execution(self, process_list):
        for process in process_list:
            if self.__stop_requested:
                break
            self.get_logger().info('Call {}'.format(process.__name__))
            process()

    def __callback_start(self, msg):
        self.get_logger().info('start {}'.format(msg))

        if self.__logic_in_progress:
            self.__status = 'Another logic is running: {}'.format(self.__current_logic)
            self.get_logger().info(self.__status)
            return

        if msg.id == 1:
            # Publish fan duty
            process_list = [
                self.__process_publish_fan_duty,
                self.__process_complete,
            ]
            self.__thread = threading.Thread(target=self.__process_execution, args=[process_list])
            self.__thread.start()

        else:
            self.__status = 'Unimplemented logic: {}'.format(msg)
            self.get_logger().warning(self.__status)
            return

        self.__logic_in_progress = True
        self.__current_logic = msg

    def __stop_processing(self, request, response):
        self.__stop_requested = True
        if self.__thread:
            self.__thread.join()
            self.__thread = None

        # Finish processing
        self.__logic_in_progress = False
        self.__status = ''
        self.__current_logic = None
        self.get_logger().info('finish processing')

        self.__stop_requested = False
        response.result = StopProcessingUserNode.Response.SUCCESS
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TestFanDutyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
