#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import sys
import threading
from ib2_msgs.msg import (
    LEDColors,
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
    ColorRGBA,
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
        self.__led_display_left_publisher = rospy.Publisher('/led_display_left/led_colors', LEDColors, queue_size=1)

    def __process_publish_led_colors(self):
        """
            Publish led colors.
        """

        # After declaring rospy.Publisher, it takes a certain amount of time
        # before other nodes can actually subscribe to it.
        # The following are possible policies to deal with this.
        # 1. Do nothing. In most cases, rospy.Publisher will be implemented to publish topics
        #    periodically, and the subscriber side should be implemented to process topics
        #    sequentially as the become available.
        # 2. Insert a wait process after declaring rospy.Publisher
        # 3. Enable "latch" option. The latch contains the last published message and is set to any new subscribers.
        led_display_right_publisher = rospy.Publisher('/led_display_right/led_colors', LEDColors,
                                                      queue_size=1, latch=True)
        sleep_duration = rospy.Duration(secs=10)

        # LEDColors must be defined as an 8-element array.
        # The alpha(a) value of ColorRGBA is ignored.
        # The value of each element is a small number between 0 and 1.
        # ATTENTION: When set to 1.0, the light is quite dazzling.
        left_color = LEDColors(
            colors=[ColorRGBA(r=0.3, g=0, b=0, a=0)] * 8
        )

        right_color = LEDColors(
            colors=[ColorRGBA(r=0, g=0.3, b=0, a=0)] * 8
        )

        self.__led_display_left_publisher.publish(left_color)
        led_display_right_publisher.publish(right_color)
        rospy.sleep(sleep_duration)

        left_color = LEDColors(
            colors=[
                ColorRGBA(r=0.6, g=0, b=0, a=0),
                ColorRGBA(r=0.4, g=0, b=0, a=0),
                ColorRGBA(r=0.2, g=0, b=0, a=0),
                ColorRGBA(r=0.4, g=0, b=0, a=0),
                ColorRGBA(r=0.6, g=0, b=0, a=0),
                ColorRGBA(r=0.4, g=0, b=0, a=0),
                ColorRGBA(r=0.2, g=0, b=0, a=0),
                ColorRGBA(r=0.4, g=0, b=0, a=0),
            ]
        )

        right_color = LEDColors(
            colors=[
                ColorRGBA(r=0.7, g=0, b=0, a=0),
                ColorRGBA(r=0.7, g=0, b=0, a=0),
                ColorRGBA(r=0, g=0.7, b=0, a=0),
                ColorRGBA(r=0, g=0.7, b=0, a=0),
                ColorRGBA(r=0, g=0, b=0.7, a=0),
                ColorRGBA(r=0, g=0, b=0.7, a=0),
                ColorRGBA(r=0.7, g=0.7, b=0, a=0),
                ColorRGBA(r=0.7, g=0.7, b=0, a=0),
            ]
        )

        self.__led_display_left_publisher.publish(left_color)
        led_display_right_publisher.publish(right_color)
        rospy.sleep(sleep_duration)

        led_display_right_publisher.unregister()

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
            # Publish led colors
            process_list = [
                self.__process_publish_led_colors,
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
