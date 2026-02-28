#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from ib2_msgs.msg import (
    Slam,
)
from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import (
    StopProcessingUserNode,
)
from builtin_interfaces.msg import Time


class TestVisualSlamNode(Node):

    MAX_MSG_SIZE = 800

    def __init__(self):
        super().__init__('user_template')

        self.__current_logic = None
        self.__logic_in_progress = False
        self.__status = ''
        self.__stop_requested = False
        self.__thread = None

        # Publisher
        self.__status_publisher = self.create_publisher(
            UserNodeStatus, '/ib2_user/status', 1)
        self.__complete_publisher = self.create_publisher(
            Time, '/ib2_user/complete', 1)

        # Subscriber
        self.__subscriber_start = self.create_subscription(
            UserLogic, '/ib2_user/start', self.__callback_start, 10)

        # Service server
        self.__stop_processing_server = self.create_service(
            StopProcessingUserNode, '/ib2_user/stop', self.__stop_processing)

        # Timer for periodic status publishing (replaces rospy.Rate loop)
        self.__timer = self.create_timer(1.0, self.__publish_status)

    def __publish_status(self):
        msg = list(self.__status.encode(encoding='utf-8')
                   [:TestVisualSlamNode.MAX_MSG_SIZE])
        if len(msg) < TestVisualSlamNode.MAX_MSG_SIZE:
            msg.extend([0] * (TestVisualSlamNode.MAX_MSG_SIZE - len(msg)))
        now = self.get_clock().now().to_msg()
        self.__status_publisher.publish(UserNodeStatus(
            stamp=now,
            msg=msg
        ))

    def __process_publish_visual_slam(self):
        """
            Publish Visual SLAM values.
        """

        # After declaring a publisher, it takes a certain amount of time
        # before other nodes can actually subscribe to it.
        # The following are possible policies to deal with this.
        # 1. Do nothing. In most cases, publishers will be implemented to publish topics
        #    periodically, and the subscriber side should be implemented to process topics
        #    sequentially as they become available.
        # 2. Insert a wait process after declaring the publisher
        # 3. Enable "transient local" durability QoS (ROS 2 equivalent of the ROS 1 "latch" option).
        slam_publisher = self.create_publisher(Slam, '/slam_wrapper/slam', 1)
        time.sleep(2.0)

        sleep_duration = 10.0  # seconds

        slam_publisher.publish(Slam(
            stamp=self.get_clock().now().to_msg(),
            slam_status=1,
            # *_s: position
            x_s=0.3,
            y_s=0.3,
            z_s=0.3,
            # q*: orientation (quaternion)
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            # v_*: velocity
            v_x=0.3,
            v_y=0.3,
            v_z=0.3,
            # w_*: angular velocity
            w_x=0.1,
            w_y=0.1,
            w_z=0.1,
            # point: number of feature points
            point=110,
        ))
        time.sleep(sleep_duration)

        slam_publisher.publish(Slam(
            stamp=self.get_clock().now().to_msg(),
            slam_status=1,
            # *_s: position
            x_s=0.6,
            y_s=0.6,
            z_s=0.6,
            # q*: orientation (quaternion)
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            # v_*: velocity
            v_x=0.2,
            v_y=0.2,
            v_z=0.2,
            # w_*: angular velocity
            w_x=0.3,
            w_y=0.3,
            w_z=0.3,
            # point: number of feature points
            point=90,
        ))
        time.sleep(sleep_duration)

        slam_publisher.publish(Slam(
            stamp=self.get_clock().now().to_msg(),
            slam_status=1,
            # *_s: position
            x_s=0.9,
            y_s=0.9,
            z_s=0.9,
            # q*: orientation (quaternion)
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            # v_*: velocity
            v_x=0.1,
            v_y=0.1,
            v_z=0.1,
            # w_*: angular velocity
            w_x=0.5,
            w_y=0.5,
            w_z=0.5,
            # point: number of feature points
            point=100,
        ))
        time.sleep(sleep_duration)

        self.destroy_publisher(slam_publisher)

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
            # Publish Visual SLAM results
            process_list = [
                self.__process_publish_visual_slam,
                self.__process_complete,
            ]
            self.__thread = threading.Thread(
                target=self.__process_execution, args=[process_list])
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
    node = TestVisualSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
