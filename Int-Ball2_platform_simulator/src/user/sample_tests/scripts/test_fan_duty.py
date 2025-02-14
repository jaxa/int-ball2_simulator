#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import sys
import threading
from geometry_msgs.msg import (
    WrenchStamped,
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
    Float64MultiArray,
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
        self.__last_ctl_wrench = None
        self.__last_fan_duty = None

        # Subscriber
        self.__subscriber_ctl_wrench = rospy.Subscriber('/ctl/wrench', WrenchStamped, self.__callback_ctl_wrench)

        # Publisher
        self.__fan_duty_publisher = rospy.Publisher('/ctl/duty', Float64MultiArray, queue_size=1)

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

        sleep_duration = rospy.Duration(secs=5)

        def publish_and_sleep(fan_duty_msg):
            self.__fan_duty_publisher.publish(fan_duty)
            self.__last_fan_duty = fan_duty
            self.__status = '/ctl/wrench: {}, /fan/duty: {}'.format(self.__last_ctl_wrench, self.__last_fan_duty)
            rospy.sleep(sleep_duration)

        # fan_duty must be defined as an 8-element array.
        # One element is the control value of one fan.
        # The value of each element is a small number between 0 and 1.
        fan_duty = Float64MultiArray(
            data=[0.3] * 8
        )
        publish_and_sleep(fan_duty)

        # To stop all fans, set all values to zero.
        fan_duty = Float64MultiArray(
            data=[0] * 8
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
            # Publish fan duty
            process_list = [
                self.__process_publish_fan_duty,
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
