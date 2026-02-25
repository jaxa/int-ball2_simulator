#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    Vector3,
)
from ib2_msgs.msg import (
    IMU,
    Navigation,
    NavigationStatus,
)
from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import (
    StopProcessingUserNode,
)
from std_msgs.msg import (
    Header,
)
from builtin_interfaces.msg import Time


class TestNavigationNode(Node):

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

        # __callback_for_imu
        self.__navigatoin_publisher = self.create_publisher(
            Navigation, '/sensor_fusion/navigation', 1)

        # Timer for periodic status publishing (replaces rospy.Rate loop)
        self.__timer = self.create_timer(1.0, self.__publish_status)

    def __publish_status(self):
        msg = list(self.__status.encode(encoding='utf-8')[:TestNavigationNode.MAX_MSG_SIZE])
        if len(msg) < TestNavigationNode.MAX_MSG_SIZE:
            msg.extend([0] * (TestNavigationNode.MAX_MSG_SIZE - len(msg)))
        now = self.get_clock().now().to_msg()
        self.__status_publisher.publish(UserNodeStatus(
            stamp=now,
            msg=msg
        ))

    def __process_publish_navigation_values(self):
        """
            Publish navigation values
        """

        # After declaring a publisher, it takes a certain amount of time
        # before other nodes can actually subscribe to it.
        # The following are possible policies to deal with this.
        # 1. Do nothing. In most cases, publishers will be implemented to publish topics
        #    periodically, and the subscriber side should be implemented to process topics
        #    sequentially as they become available.
        # 2. Insert a wait process after declaring the publisher
        # 3. Enable "transient local" durability QoS (ROS 2 equivalent of the ROS 1 "latch" option).
        navigatoin_publisher = self.create_publisher(Navigation, '/sensor_fusion/navigation', 1)
        time.sleep(2.0)

        sleep_duration = 10.0  # seconds

        def test_callback_001(msg):
            navigatoin_publisher.publish(Navigation(
                pose=PoseStamped(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    pose=Pose(
                        position=Point(x=2.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                ),
                twist=Twist(
                    # linear: velocity
                    linear=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
                    # angular: angular velocity
                    angular=Vector3(x=msg.gyro_x, y=msg.gyro_y, z=msg.gyro_z),
                ),
                # a: acceleration
                a=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
                # status:
                #  NAV_FUSION: IMU sensor + Visual SLAM
                #  NAV_INERTIAL: IMU sensor only
                #  NAV_SLAM: Visual SLAM only
                # marker: Whether the marker is detected or not.
                status=NavigationStatus(status=NavigationStatus.NAV_FUSION, marker=False),
            ))
        # IMU
        # stamp: timestamp
        # acc_x, acc_y, acc_z: velocity
        # gyro_x, gyro_y, gyro_z: angular velocity
        # temperature: temperature near IMU sensor
        imu_subscriber = self.create_subscription(IMU, '/imu/imu', test_callback_001, 10)
        time.sleep(sleep_duration)
        self.destroy_subscription(imu_subscriber)

        def test_callback_002(msg):
            navigatoin_publisher.publish(Navigation(
                pose=PoseStamped(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    pose=Pose(
                        position=Point(x=0.0, y=2.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                ),
                twist=Twist(
                    linear=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
                    angular=Vector3(x=msg.gyro_x, y=msg.gyro_y, z=msg.gyro_z),
                ),
                a=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
                status=NavigationStatus(status=NavigationStatus.NAV_INERTIAL, marker=False),
            ))
        imu_subscriber = self.create_subscription(IMU, '/imu/imu', test_callback_002, 10)
        time.sleep(sleep_duration)
        self.destroy_subscription(imu_subscriber)

        imu_subscriber = self.create_subscription(IMU, '/imu/imu', self.__callback_for_imu, 10)
        time.sleep(sleep_duration)
        self.destroy_subscription(imu_subscriber)

        self.destroy_publisher(navigatoin_publisher)

    def __callback_for_imu(self, msg):
        """
            Example of a callback that subscribes to IMU sensor values.
        """
        # IMU
        # stamp: timestamp
        # acc_x, acc_y, acc_z: velocity
        # gyro_x, gyro_y, gyro_z: angular velocity
        # temperature: temperature near IMU sensor
        self.__navigatoin_publisher.publish(Navigation(
            pose=PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg()),
                pose=Pose(
                    position=Point(x=2.0, y=0.0, z=0.5),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
            twist=Twist(
                linear=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
                angular=Vector3(x=msg.gyro_x, y=msg.gyro_y, z=msg.gyro_z),
            ),
            a=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
            status=NavigationStatus(status=NavigationStatus.NAV_FUSION, marker=False),
        ))

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
            # Publish navigation values
            #  (Essentially, it outputs the sensor fusion result of the IMU sensor value and the Visual SLAM result)
            process_list = [
                self.__process_publish_navigation_values,
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
    node = TestNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
