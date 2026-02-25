#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import StopProcessingUserNode


class UserTemplateNode(Node):

    MAX_MSG_SIZE = 800

    def __init__(self):
        super().__init__('user_template')

        # Node-specific ros parameters (declare with defaults, then read)
        self.declare_parameter('custom_parameter_integer', 0)
        self.declare_parameter('custom_parameter_float', 0.0)
        self.declare_parameter('custom_parameter_string', '')
        self.declare_parameter('custom_parameter_boolean', False)

        self.__custom_parameter_integer = (
            self.get_parameter('custom_parameter_integer')
            .get_parameter_value().integer_value
        )
        self.__custom_parameter_float = (
            self.get_parameter('custom_parameter_float')
            .get_parameter_value().double_value
        )
        self.__custom_parameter_string = (
            self.get_parameter('custom_parameter_string')
            .get_parameter_value().string_value
        )
        self.__custom_parameter_boolean = (
            self.get_parameter('custom_parameter_boolean')
            .get_parameter_value().bool_value
        )

        # Publisher
        self.__status_publisher = self.create_publisher(
            UserNodeStatus, '/ib2_user/status', 1
        )

        # Subscriber
        self.__subscriber_start = self.create_subscription(
            UserLogic, '/ib2_user/start', self.__callback_start, 10
        )

        # Service server
        self.__stop_processing_server = self.create_service(
            StopProcessingUserNode, '/ib2_user/stop', self.__stop_processing
        )

        # Timer at 1 Hz (replaces rospy.Rate + while loop)
        self.__status = 'stopped'
        self.__timer = self.create_timer(1.0, self.__timer_callback)

    def __callback_start(self, msg):
        self.__status = 'start {}: {}, {}, {}, {}'.format(
            msg,
            self.__custom_parameter_integer,
            self.__custom_parameter_float,
            self.__custom_parameter_string,
            self.__custom_parameter_boolean,
        )
        self.get_logger().info(self.__status)

    def __stop_processing(self, request, response):
        self.__status = 'finish processing'
        self.get_logger().info(self.__status)
        response.result = StopProcessingUserNode.Response.SUCCESS
        return response

    def __timer_callback(self):
        msg_bytes = list(
            self.__status.encode(encoding='utf-8')[: UserTemplateNode.MAX_MSG_SIZE]
        )
        if len(msg_bytes) < UserTemplateNode.MAX_MSG_SIZE:
            msg_bytes.extend([0] * (UserTemplateNode.MAX_MSG_SIZE - len(msg_bytes)))

        status_msg = UserNodeStatus()
        status_msg.stamp = self.get_clock().now().to_msg()
        status_msg.msg = msg_bytes
        self.__status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UserTemplateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
