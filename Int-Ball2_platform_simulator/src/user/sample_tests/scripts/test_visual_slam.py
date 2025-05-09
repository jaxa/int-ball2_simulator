#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import sys
import threading
from ib2_msgs.msg import (
    Slam,
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
        self.__status_publisher = rospy.Publisher(
            '/ib2_user/status', UserNodeStatus, queue_size=1)
        self.__complete_publisher = rospy.Publisher(
            '/ib2_user/complete', Time, queue_size=1)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber(
            '/ib2_user/start', UserLogic, self.__callback_start)

        # Service server
        self.__stop_processing_server = rospy.Service(
            '/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)

    def __process_publish_visual_slam(self):
        """
            Publish Visual SLAM values.
        """

        # After declaring rospy.Publisher, it takes a certain amount of time
        # before other nodes can actually subscribe to it.
        # The following are possible policies to deal with this.
        # 1. Do nothing. In most cases, rospy.Publisher will be implemented to publish topics
        #    periodically, and the subscriber side should be implemented to process topics
        #    sequentially as the become available.
        # 2. Insert a wait process after declaring rospy.Publisher
        # 3. Enable "latch" option. The latch contains the last published message and is set to any new subscribers.
        slam_publisher = rospy.Publisher('/slam_wrapper/slam', Slam, queue_size=1)
        rospy.sleep(rospy.Duration(secs=2))

        sleep_duration = rospy.Duration(secs=10)

        slam_publisher.publish(Slam(
            stamp=rospy.Time.now(),
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
        rospy.sleep(sleep_duration)

        slam_publisher.publish(Slam(
            stamp=rospy.Time.now(),
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
        rospy.sleep(sleep_duration)

        slam_publisher.publish(Slam(
            stamp=rospy.Time.now(),
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
        rospy.sleep(sleep_duration)

        slam_publisher.unregister()

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
            msg = list(self.__status.encode(encoding='utf-8')
                       [:UserTemplate.MAX_MSG_SIZE])
            if len(msg) < UserTemplate.MAX_MSG_SIZE:
                msg.extend([0]*(UserTemplate.MAX_MSG_SIZE-len(msg)))
            self.__status_publisher.publish(UserNodeStatus(
                stamp=rospy.Time.now(),
                msg=msg
            ))
            self.__rate.sleep()


if __name__ == '__main__':
    UserTemplate().run()
