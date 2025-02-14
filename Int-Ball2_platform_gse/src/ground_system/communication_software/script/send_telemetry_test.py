#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import roslib
import socketserver
import socket
import concurrent.futures
import yaml
from std_msgs.msg import Time
import contextlib
import io
import roslib.message
from functools import partial
import threading
import pickle
from communication_software.srv import *
from ib2_msgs.msg import *
from ib2_msgs.srv import *

MAX_BUFFER_SIZE = 1472

if __name__ == '__main__':
    rospy.init_node('send_telemetry_test', anonymous=True)

    telemetry_dict = {}

    # timestamp
    time_stamp = Time()
    now = rospy.Time.now()
    time_stamp.data = now
    temp_buf = io.BytesIO()
    time_stamp.serialize(temp_buf)
    telemetry_dict[0] = temp_buf.getvalue()

    # main_camera_params
    temp_buf = io.BytesIO()
    main_camera_params_msg = MainCameraParams()
    main_camera_params_msg.zoom = 9.0
    main_camera_params_msg.gain = 2.0
    main_camera_params_msg.serialize(temp_buf)
    telemetry_dict[3] = temp_buf.getvalue()

    # navigation
    temp_buf = io.BytesIO()
    navigatin_msg = Navigation()
    navigatin_msg.pose.pose.position.x = 9
    navigatin_msg.pose.pose.orientation.w = 1
    navigatin_msg.serialize(temp_buf)
    telemetry_dict[9] = temp_buf.getvalue()

    # pickle
    pickled_telemetry = pickle.dumps(telemetry_dict, protocol=3)

    # send
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with contextlib.closing(sock):
        sock.sendto(pickled_telemetry, ('localhost', 12345))
        rospy.loginfo('[send_telemetry]finish to send. size {}'.format(len(pickled_telemetry)))
        if len(pickled_telemetry) > MAX_BUFFER_SIZE:
            rospy.logerr('[send_telemetry]size {} (max {})'.format(len(pickled_telemetry), MAX_BUFFER_SIZE))
