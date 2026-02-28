#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import (
    StopProcessingUserNode,
)
from sensor_msgs.msg import (
    Image,
)
from builtin_interfaces.msg import Time


class TestCameraImagesNode(Node):

    MAX_MSG_SIZE = 800

    def __init__(self):
        super().__init__('subscribe_camera_images')

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

        # __process_subscribe_camera_images
        self.__main_image_raw_count = 0
        self.__left_image_raw_count = 0
        self.__right_image_raw_count = 0

        # Timer for periodic status publishing (replaces rospy.Rate loop)
        self.__timer = self.create_timer(1.0, self.__publish_status)

    def __publish_status(self):
        msg = list(self.__status.encode(encoding='utf-8')[:TestCameraImagesNode.MAX_MSG_SIZE])
        if len(msg) < TestCameraImagesNode.MAX_MSG_SIZE:
            msg.extend([0] * (TestCameraImagesNode.MAX_MSG_SIZE - len(msg)))
        now = self.get_clock().now().to_msg()
        self.__status_publisher.publish(UserNodeStatus(
            stamp=now,
            msg=msg
        ))

    def __process_subscribe_camera_images(self):
        """
            Subscribe to the camera's images.
        """

        # Register a member function as a callback.
        # When a callback is registered with subscriber,
        # the subscribed topics are queued and callback is invoked sequentially
        camera_main_subscriber = self.create_subscription(
            Image, '/camera_main/image_raw', self.__callback_for_image_raw, 10)

        # Register a non-member function as a callback.
        # In the following example refers to "self",
        # but it is also possible to implement without referring to it.
        def camera_left_callback(msg):
            # msg.data is actual matrix data, size is (step * rows(height))
            self.__status = 'left stamp:{} data[0]:{} count-main:{} count-left:{} count-right:{}'.format(
                msg.header.stamp,
                msg.data[0],
                self.__main_image_raw_count,
                self.__left_image_raw_count,
                self.__right_image_raw_count)
            self.__left_image_raw_count = self.__left_image_raw_count + 1
        camera_left_subscriber = self.create_subscription(
            Image, '/camera_left/image_raw', camera_left_callback, 10)

        execution_time_secs = 60
        start_time = time.monotonic()

        while not self.__stop_requested and (time.monotonic() - start_time < execution_time_secs):
            # In ROS 2, wait_for_message is available via a utility.
            # We use a one-shot subscription approach instead.
            try:
                msg = self._wait_for_message_once('/camera_right', Image, timeout_sec=1.0)
                if msg is not None:
                    stamp = msg.header.stamp
                    sample_data_str = str(msg.data[0])
                else:
                    stamp = self.get_clock().now().to_msg()
                    sample_data_str = '-'
            except Exception:
                stamp = self.get_clock().now().to_msg()
                sample_data_str = '-'

            self.__status = 'right stamp:{} data[0]:{} count-main:{} count-left:{} count-right:{}'.format(
                stamp,
                sample_data_str,
                self.__main_image_raw_count,
                self.__left_image_raw_count,
                self.__right_image_raw_count)
            self.__right_image_raw_count = self.__right_image_raw_count + 1

        self.destroy_subscription(camera_main_subscriber)
        self.destroy_subscription(camera_left_subscriber)

    def _wait_for_message_once(self, topic, msg_type, timeout_sec=1.0):
        """
            Wait for a single message on a topic, with a timeout.
            Returns the message or None if timed out.
        """
        received_msg = [None]
        event = threading.Event()

        def _cb(msg):
            received_msg[0] = msg
            event.set()

        sub = self.create_subscription(msg_type, topic, _cb, 1)
        event.wait(timeout=timeout_sec)
        self.destroy_subscription(sub)
        return received_msg[0]

    def __callback_for_image_raw(self, msg):
        """
            Example of a callback that subscribes to camera images.
        """

        # msg.data is actual matrix data, size is (step * rows(height))
        self.__status = 'main stamp:{} data[0]:{} count-main:{} count-left:{} count-right:{}'.format(
            msg.header.stamp,
            msg.data[0],
            self.__main_image_raw_count,
            self.__left_image_raw_count,
            self.__right_image_raw_count)
        self.__main_image_raw_count = self.__main_image_raw_count + 1

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
            # Subscribe camera images
            process_list = [
                self.__process_subscribe_camera_images,
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
    node = TestCameraImagesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
