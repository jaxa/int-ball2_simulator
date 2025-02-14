#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import (
    StopProcessingUserNode,
    StopProcessingUserNodeResponse,
)


class UserTemplate:

    MAX_MSG_SIZE = 800

    def __init__(self):
        rospy.init_node('user_template')

        # Node-specific ros parameters
        self.__custom_parameter_integer = rospy.get_param('~custom_parameter_integer')
        self.__custom_parameter_float = rospy.get_param('~custom_parameter_float')
        self.__custom_parameter_string = rospy.get_param('~custom_parameter_string')
        self.__custom_parameter_boolean = rospy.get_param('~custom_parameter_boolean')

        # Publisher
        self.__status_publisher = rospy.Publisher('/ib2_user/status', UserNodeStatus, queue_size=1)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)

        self.__rate = rospy.Rate(1)
        self.__status = 'stopped'

    def __callback_start(self, msg):
        self.__status = 'start {}: {}, {}, {}, {}'.format(
            msg,
            self.__custom_parameter_integer,
            self.__custom_parameter_float,
            self.__custom_parameter_string,
            self.__custom_parameter_boolean)
        rospy.loginfo(self.__status)

    def __stop_processing(self, request):
        self.__status = 'finish processing'
        rospy.loginfo(self.__status)
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
