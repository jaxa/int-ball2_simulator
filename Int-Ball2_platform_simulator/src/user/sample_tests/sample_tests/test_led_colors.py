#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from ib2_msgs.msg import (
    LEDColors,
)
from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import (
    StopProcessingUserNode,
)
from std_msgs.msg import (
    ColorRGBA,
)
from builtin_interfaces.msg import Time


class TestLedColorsNode(Node):

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
        self.__led_display_left_publisher = self.create_publisher(
            LEDColors, '/led_display_left/led_colors', 1)

        # Timer for periodic status publishing (replaces rospy.Rate loop)
        self.__timer = self.create_timer(1.0, self.__publish_status)

    def __publish_status(self):
        msg = list(self.__status.encode(encoding='utf-8')[:TestLedColorsNode.MAX_MSG_SIZE])
        if len(msg) < TestLedColorsNode.MAX_MSG_SIZE:
            msg.extend([0] * (TestLedColorsNode.MAX_MSG_SIZE - len(msg)))
        now = self.get_clock().now().to_msg()
        self.__status_publisher.publish(UserNodeStatus(
            stamp=now,
            msg=msg
        ))

    def __process_publish_led_colors(self):
        """
            Publish led colors.
        """

        # After declaring a publisher, it takes a certain amount of time
        # before other nodes can actually subscribe to it.
        # The following are possible policies to deal with this.
        # 1. Do nothing. In most cases, publishers will be implemented to publish topics
        #    periodically, and the subscriber side should be implemented to process topics
        #    sequentially as they become available.
        # 2. Insert a wait process after declaring the publisher
        # 3. Enable "transient local" durability QoS. This is the ROS 2 equivalent of
        #    the ROS 1 "latch" option. The last published message is stored and sent
        #    to any new subscribers.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        led_display_right_publisher = self.create_publisher(
            LEDColors, '/led_display_right/led_colors', qos_profile=latched_qos)
        sleep_duration = 10.0  # seconds

        # LEDColors must be defined as an 8-element array.
        # The alpha(a) value of ColorRGBA is ignored.
        # The value of each element is a small number between 0 and 1.
        # ATTENTION: When set to 1.0, the light is quite dazzling.
        left_color = LEDColors(
            colors=[ColorRGBA(r=0.3, g=0.0, b=0.0, a=0.0)] * 8
        )

        right_color = LEDColors(
            colors=[ColorRGBA(r=0.0, g=0.3, b=0.0, a=0.0)] * 8
        )

        self.__led_display_left_publisher.publish(left_color)
        led_display_right_publisher.publish(right_color)
        time.sleep(sleep_duration)

        left_color = LEDColors(
            colors=[
                ColorRGBA(r=0.6, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.4, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.2, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.4, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.6, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.4, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.2, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.4, g=0.0, b=0.0, a=0.0),
            ]
        )

        right_color = LEDColors(
            colors=[
                ColorRGBA(r=0.7, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.7, g=0.0, b=0.0, a=0.0),
                ColorRGBA(r=0.0, g=0.7, b=0.0, a=0.0),
                ColorRGBA(r=0.0, g=0.7, b=0.0, a=0.0),
                ColorRGBA(r=0.0, g=0.0, b=0.7, a=0.0),
                ColorRGBA(r=0.0, g=0.0, b=0.7, a=0.0),
                ColorRGBA(r=0.7, g=0.7, b=0.0, a=0.0),
                ColorRGBA(r=0.7, g=0.7, b=0.0, a=0.0),
            ]
        )

        self.__led_display_left_publisher.publish(left_color)
        led_display_right_publisher.publish(right_color)
        time.sleep(sleep_duration)

        self.destroy_publisher(led_display_right_publisher)

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
            # Publish led colors
            process_list = [
                self.__process_publish_led_colors,
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
    node = TestLedColorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
