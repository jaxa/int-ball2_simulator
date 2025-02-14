#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import sys
import threading
from platform_msgs.msg import (
    UserNodeStatus,
    UserLogic,
)
from platform_msgs.srv import (
    StopProcessingUserNode,
    StopProcessingUserNodeResponse
)
from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import (
    Time,
)


class UserTemplate:

    MAX_MSG_SIZE = 800

    def __init__(self):
        rospy.init_node('subscribe_camera_images')

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

        # __process_subscribe_camera_images
        self.__main_image_raw_count = 0
        self.__left_image_raw_count = 0
        self.__right_image_raw_count = 0

    def __process_subscribe_camera_images(self):
        """
            Subscribe to the camera's images.
        """

        # Register a member function as a callback.
        # When a callback is registered with subscriber,
        # the subscribed topics are queued and callback is invoked sequentially
        camera_main_subscriber = rospy.Subscriber('/camera_main/image_raw', Image, self.__callback_for_image_raw)

        # Register a non-member function as a callback.
        # In the following example refers to \"self\,
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
        camera_left_subscriber = rospy.Subscriber('/camera_left/image_raw', Image, camera_left_callback)

        execution_time = rospy.Duration(secs=60)
        start_time = rospy.Time.now()
        # http://wiki.ros.org/roscpp/Overview/Time
        # Time and Duration arithmetic
        # - duration + duration = duration
        # - duration - duration = duration
        # - time + duration = time
        # - time - time = duration
        # - time + time is undefined
        while not self.__stop_requested and (rospy.Time.now() - start_time < execution_time):

            # Subscribes to a message only once.
            # When the timeout occurs, an Exception is raised.
            # Topic published before calling wait_for_message is not queued and is discarded,
            try:
                msg = rospy.wait_for_message('/camera_right', Image, timeout=1)
                # msg.data is actual matrix data, size is (step * rows(height))
                stamp = msg.header.stamp
                sample_data_str = str(msg.data[0])
            except rospy.exceptions.ROSException:
                stamp = rospy.Time.now()
                sample_data_str = '-'

            self.__status = 'right stamp:{} data[0]:{} count-main:{} count-left:{} count-right:{}'.format(
                stamp,
                sample_data_str,
                self.__main_image_raw_count,
                self.__left_image_raw_count,
                self.__right_image_raw_count)
            self.__right_image_raw_count = self.__right_image_raw_count + 1

        camera_main_subscriber.unregister()
        camera_left_subscriber.unregister()

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
            # Subscribe camera images
            process_list = [
                self.__process_subscribe_camera_images,
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
        rospy.loginfo('TEST before loop')
        while not rospy.is_shutdown():
            rospy.loginfo('TEST loop')
            msg = list(self.__status.encode(encoding='utf-8')[:UserTemplate.MAX_MSG_SIZE])
            if len(msg) < UserTemplate.MAX_MSG_SIZE:
                msg.extend([0]*(UserTemplate.MAX_MSG_SIZE-len(msg)))
            self.__status_publisher.publish(UserNodeStatus(
                stamp=rospy.Time.now(),
                msg=msg
            ))
            self.__rate.sleep()


if __name__ == '__main__':
    rospy.loginfo('TEST main')
    UserTemplate().run()
