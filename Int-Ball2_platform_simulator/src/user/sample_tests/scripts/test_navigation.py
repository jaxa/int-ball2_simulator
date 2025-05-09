#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import sys
import threading
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
    StopProcessingUserNodeResponse
)
from std_msgs.msg import (
    Header,
    Time,
)


class UserTemplate:

    MAX_MSG_SIZE = 800

    def __init__(self):
        rospy.init_node('user_template')

        self.__current_logic = None
        self.__logic_in_progress = False
        self.__rate = rospy.Rate(1)
        self.__status = ''
        self.__stop_requested = False
        self.__thread = None

        # Publisher
        self.__status_publisher = rospy.Publisher('/ib2_user/status', UserNodeStatus, queue_size=1)
        self.__complete_publisher = rospy.Publisher('/ib2_user/complete', Time, queue_size=1)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)

        #####################################################
        # Variables specific to each processing process
        #####################################################

        # __callback_for_imu
        self.__navigatoin_publisher = rospy.Publisher('/sensor_fusion/navigation', Navigation, queue_size=1)

    def __process_publish_navigation_values(self):
        """
            Publish navigation values
        """

        # After declaring rospy.Publisher, it takes a certain amount of time
        # before other nodes can actually subscribe to it.
        # The following are possible policies to deal with this.
        # 1. Do nothing. In most cases, rospy.Publisher will be implemented to publish topics
        #    periodically, and the subscriber side should be implemented to process topics
        #    sequentially as the become available.
        # 2. Insert a wait process after declaring rospy.Publisher
        # 3. Enable "latch" option. The latch contains the last published message and is set to any new subscribers.
        navigatoin_publisher = rospy.Publisher('/sensor_fusion/navigation', Navigation, queue_size=1)
        rospy.sleep(rospy.Duration(secs=2))

        sleep_duration = rospy.Duration(secs=10)

        def test_callback_001(msg):
            navigatoin_publisher.publish(Navigation(
                pose=PoseStamped(
                    header=Header(stamp=rospy.Time.now()),
                    pose=Pose(
                        position=Point(x=2.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1),
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
        imu_subscriber = rospy.Subscriber('/imu/imu', IMU, test_callback_001)
        rospy.sleep(sleep_duration)
        imu_subscriber.unregister()

        def test_callback_002(msg):
            navigatoin_publisher.publish(Navigation(
                pose=PoseStamped(
                    header=Header(stamp=rospy.Time.now()),
                    pose=Pose(
                        position=Point(x=0.0, y=2.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1),
                    )
                ),
                twist=Twist(
                    linear=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
                    angular=Vector3(x=msg.gyro_x, y=msg.gyro_y, z=msg.gyro_z),
                ),
                a=Vector3(x=msg.acc_x, y=msg.acc_y, z=msg.acc_z),
                status=NavigationStatus(status=NavigationStatus.NAV_INERTIAL, marker=False),
            ))
        imu_subscriber = rospy.Subscriber('/imu/imu', IMU, test_callback_002)
        rospy.sleep(sleep_duration)
        imu_subscriber.unregister()

        imu_subscriber = rospy.Subscriber('/imu/imu', IMU, self.__callback_for_imu)
        rospy.sleep(sleep_duration)
        imu_subscriber.unregister()

        navigatoin_publisher.unregister()

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
                header=Header(stamp=rospy.Time.now()),
                pose=Pose(
                    position=Point(x=2.0, y=0.0, z=0.5),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1),
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
        rospy.loginfo('finish processing')

        self.__complete_publisher.publish(Time(data=rospy.Time.now()))

    def __process_execution(self, process_list):
        for process in process_list:
            if self.__stop_requested:
                break
            rospy.loginfo('Call {}'.format(process.__name__))
            process()

    def __callback_start(self, msg):
        rospy.loginfo('start {}'.format(msg))

        if self.__logic_in_progress:
            self.__status = 'Another logic is running: {}'.format(self.__current_logic)
            rospy.loginfo(self.__status)
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
            rospy.logwarn(self.__status)
            return

        self.__logic_in_progress = True
        self.__current_logic = msg

    def __stop_processing(self, request):
        self.__stop_requested = True
        if self.__thread:
            self.__thread.join()
            self.__thread = None

        # Finish processing
        self.__logic_in_progress = False
        self.__status = ''
        self.__current_logic = None
        rospy.loginfo('finish processing')

        self.__stop_requested = False
        return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)

    def run(self):
        while not rospy.is_shutdown():
            msg = list(self.__status.encode(encoding='utf-8')[:UserTemplate.MAX_MSG_SIZE])
            if len(msg) < UserTemplate.MAX_MSG_SIZE:
                msg.extend([0]*(UserTemplate.MAX_MSG_SIZE-len(msg)))
            self.__status_publisher.publish(UserNodeStatus(
                stamp=rospy.Time.now(),
                msg=msg
            ))
            self.__rate.sleep()


if __name__ == '__main__':
    UserTemplate().run()
