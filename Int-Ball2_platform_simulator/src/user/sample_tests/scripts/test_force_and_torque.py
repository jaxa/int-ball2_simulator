#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import sys
import threading
from geometry_msgs.msg import (
    Vector3,
    Wrench,
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
        self.__force_and_torque_publisher = rospy.Publisher('/ctl/wrench', WrenchStamped, queue_size=1)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)

    def __process_controll_force_and_torque(self):
        """
            Control of force and torque in guidance control.
        """

        sleep_duration = rospy.Duration(secs=5)

        wrench = WrenchStamped(
            header=Header(stamp=rospy.Time.now()),
            wrench=Wrench(
                force=Vector3(x=0.5, y=0, z=0),
                torque=Vector3(x=0.5, y=0, z=0)
            )
        )
        self.__force_and_torque_publisher.publish(wrench)
        rospy.sleep(sleep_duration)

        wrench = WrenchStamped(
            header=Header(stamp=rospy.Time.now()),
            wrench=Wrench(
                force=Vector3(x=0, y=0.5, z=0),
                torque=Vector3(x=0, y=0.5, z=0)
            )
        )
        self.__force_and_torque_publisher.publish(wrench)
        rospy.sleep(sleep_duration)

        wrench = WrenchStamped(
            header=Header(stamp=rospy.Time.now()),
            wrench=Wrench(
                force=Vector3(x=-0.5, y=0.0, z=0.5),
                torque=Vector3(x=0, y=0, z=0.0)
            )
        )
        self.__force_and_torque_publisher.publish(wrench)
        rospy.sleep(sleep_duration)

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
            # Control force and torque
            process_list = [
                self.__process_controll_force_and_torque,
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
