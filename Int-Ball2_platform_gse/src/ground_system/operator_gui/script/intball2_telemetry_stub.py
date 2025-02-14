#!/usr/bin/env python3
import roslib
import rospy
import geometry_msgs.msg
import socketserver
import socket
import concurrent.futures
import yaml
from std_msgs.msg import Time, UInt8, UInt16
import contextlib
import io
import roslib.message
from functools import partial
import threading
import pickle
from communication_software.srv import *
from ib2_msgs.msg import *
from ib2_msgs.srv import *

# Python3でtfを利用するためには, tfパッケージのソースビルドが必要
# import tf

MAX_BUFFER_SIZE = 1472

if __name__ == '__main__':
    rospy.init_node('intball2_telemetry_stub')
    r = rospy.Rate(1)

    add_x = -0.1
    set_x = 5
    add_y = -0.1
    set_y = 0.1
    add_z = -0.1
    set_z = -0.5
    set_euler = 1
    add_euler = 0.05
    # Python3でtfを利用するためには, tfパッケージのソースビルドが必要
    # br = tf.TransformBroadcaster()
    quaternion = [0, 0, 0, 1]
    set_quaternion = 0.1
    while not rospy.is_shutdown():
        """
        # telemetry
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'iss_body'
        pose.pose.position.x = set_x
        pose.pose.position.y = set_y
        pose.pose.position.z = set_z

        quaternion = tf.transformations.quaternion_from_euler(set_euler, 0, -set_euler)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        # tf body
        send_transform = geometry_msgs.msg.TransformStamped()
        send_transform.header.frame_id = 'iss_body'
        send_transform.header.stamp = rospy.Time.now()
        send_transform.child_frame_id = 'body'
        send_transform.transform.translation.x = pose.pose.position.x
        send_transform.transform.translation.y = pose.pose.position.y
        send_transform.transform.translation.z = pose.pose.position.z
        send_transform.transform.rotation.x = pose.pose.orientation.x
        send_transform.transform.rotation.y = pose.pose.orientation.y
        send_transform.transform.rotation.z = pose.pose.orientation.z
        send_transform.transform.rotation.w = pose.pose.orientation.w
        """

        telemetry_dict = {}

        # timestamp
        time_stamp = Time()
        now = rospy.Time.now()
        time_stamp.data = now
        temp_buf = io.BytesIO()
        time_stamp.serialize(temp_buf)
        telemetry_dict[0] = temp_buf.getvalue()

        # ID of the last command received
        last_executed = UInt16()
        last_executed.data = 0
        temp_buf = io.BytesIO()
        last_executed.serialize(temp_buf)
        telemetry_dict[1] = temp_buf.getvalue()

        # Information about splitting
        split_number_msg = UInt8()
        split_number_msg.data = 1
        temp_buf = io.BytesIO()
        split_number_msg.serialize(temp_buf)
        telemetry_dict[2] = temp_buf.getvalue()
        split_index_msg = UInt8()
        split_index_msg.data = 1
        temp_buf = io.BytesIO()
        split_index_msg.serialize(temp_buf)
        telemetry_dict[3] = temp_buf.getvalue()

        # navigation
        temp_buf = io.BytesIO()
        navigatin_msg = Navigation()
        navigatin_msg.pose.pose.position.x = set_x
        navigatin_msg.pose.pose.position.y = set_y
        navigatin_msg.pose.pose.position.z = set_z
        # Python3でtfを利用するためには, tfパッケージのソースビルドが必要
        # quaternion = tf.transformations.quaternion_from_euler(set_euler, 0, -set_euler)
        # navigatin_msg.pose.pose.orientation.x = quaternion[0]
        # navigatin_msg.pose.pose.orientation.y = quaternion[1]
        # navigatin_msg.pose.pose.orientation.z = quaternion[2]
        # navigatin_msg.pose.pose.orientation.w = quaternion[3]
        quaternion[0] = quaternion[0] + set_quaternion
        quaternion[1] = quaternion[1] - set_quaternion
        quaternion[2] = quaternion[2] + set_quaternion
        navigatin_msg.pose.pose.orientation.x = quaternion[0]
        navigatin_msg.pose.pose.orientation.y = quaternion[1]
        navigatin_msg.pose.pose.orientation.z = quaternion[2]
        navigatin_msg.pose.pose.orientation.w = quaternion[3]
        navigatin_msg.serialize(temp_buf)
        telemetry_dict[302] = temp_buf.getvalue()

        # led gains
        # left
        temp_buf = io.BytesIO()
        led_gains_left_msg = RosParam()
        led_gains_left_msg.id = '/led_display_left/gains'
        led_gains_left_msg.value = '[[0.1,0.2,0.3],[0.2,0.3,0.4],[0.3,0.4,0.5],[0.4,0.5,0.6],[0.5,0.6,0.7],[0.6,0.7,0.8],[0.7,0.8,0.9],[0.8,0.9,1.0]]'
        led_gains_left_msg.type = 'string'
        led_gains_left_msg.serialize(temp_buf)
        telemetry_dict[10000] = temp_buf.getvalue()

        # right
        temp_buf = io.BytesIO()
        led_gains_right_msg = RosParam()
        led_gains_right_msg.id = '/led_display_right/gains'
        led_gains_right_msg.value = '[[1.1,1.2,1.3],[1.2,1.3,1.4],[1.3,1.4,1.5],[1.4,1.5,1.6],[1.5,1.6,1.7],[1.6,1.7,1.8],[1.7,1.8,1.9],[1.8,1.9,2.0]]'
        led_gains_right_msg.type = 'string'
        led_gains_right_msg.serialize(temp_buf)
        telemetry_dict[10001] = temp_buf.getvalue()

        # ib2_msgs/GetRosParams
        temp_buf = io.BytesIO()
        get_rosparam_msg = RosParam()
        get_rosparam_msg.id = '/aaaaaaaa'
        get_rosparam_msg.value = 'aaaaaaaa'
        get_rosparam_msg.type = ''
        get_rosparam_msg.serialize(temp_buf)
        time_stamp.serialize(temp_buf)
        telemetry_dict[11000] = temp_buf.getvalue()
        temp_buf = io.BytesIO()
        get_rosparam_msg = RosParam()
        get_rosparam_msg.id = '/eeeeeeeeeeee'
        get_rosparam_msg.value = 'eeeeeeeeeeeeee'
        get_rosparam_msg.type = ''
        get_rosparam_msg.serialize(temp_buf)
        time_stamp.serialize(temp_buf)
        telemetry_dict[11001] = temp_buf.getvalue()

        # ib2_msgs/SetRosParams
        temp_buf = io.BytesIO()
        set_rosparam_msg = RosParam()
        set_rosparam_msg.id = '/qqqqqqqqqqqq'
        set_rosparam_msg.value = 'ssssssssss'
        set_rosparam_msg.type = ''
        set_rosparam_msg.serialize(temp_buf)
        time_stamp.serialize(temp_buf)
        telemetry_dict[12001] = temp_buf.getvalue()
        temp_buf = io.BytesIO()
        set_rosparam_msg = RosParam()
        set_rosparam_msg.id = '/ppppppp'
        set_rosparam_msg.value = 'bbbbbbbbbbbbb'
        set_rosparam_msg.type = ''
        set_rosparam_msg.serialize(temp_buf)
        time_stamp.serialize(temp_buf)
        telemetry_dict[12002] = temp_buf.getvalue()


        # pickle
        pickled_telemetry = pickle.dumps(telemetry_dict, protocol=3)

        # send
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        with contextlib.closing(sock):
            sock.sendto(pickled_telemetry, ('localhost', 34567))
            rospy.loginfo('[send_telemetry]finish to send. size {}'.format(len(pickled_telemetry)))
            if len(pickled_telemetry) > MAX_BUFFER_SIZE:
                rospy.logerr('[send_telemetry]size {} (max {})'.format(len(pickled_telemetry), MAX_BUFFER_SIZE))

        r.sleep()

        set_x = set_x + add_x
        if set_x > 0.9 or set_x < 0.1:
            add_x = -add_x
        set_y = set_y + add_y
        if set_y > 0.5 or set_y < -0.5:
            add_y = -add_y
        set_z = set_z + add_z
        if set_z > 0 or set_z < -0.5:
            add_z = -add_z
        set_euler = set_euler + add_z
        if set_euler > 1.7 or set_euler < 1.2:
            add_euler = -add_euler
        if quaternion[0] > 1:
            quaternion = [0, 0, 0, 1]
