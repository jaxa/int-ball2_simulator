#!/usr/bin/python3
# -*- coding: utf-8 -*-

import contextlib
import genpy
import rospy
import roslib.packages
import io
import socket
import yaml
from communication_software.srv import *
from communication_software.msg import *
from ib2_msgs.msg import *
from ib2_msgs.srv import *
from std_msgs.msg import *

if __name__ == '__main__':
    rospy.wait_for_service('telecommand_bridge')
    try:
        telecommand_server_proxy = rospy.ServiceProxy('telecommand_bridge', Telecommand)
        request = TelecommandRequest()
        command = Message()

        # Service, serialize
        #msg = SwitchPowerRequest()
        #buff = io.BytesIO()
        #msg.serialize(buff)

        #command.format = Message.SERIALIZED_BINARY_DATA
        #command.name = '/intball2/camera_and_microphone/camera/switch_power'
        #command.data = buff.getvalue()
        #request.command.append(command)
        #resp = telecommand_server_proxy(request)
        #print(resp)

        # Prop
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buff = io.BytesIO()
        msg.serialize(buff)
        command.msg_type = Message.INTBALL2_SERIALIZED_BINARY_DATA
        command.name = '/ctl/duty'
        command.data = buff.getvalue()
        request.command = command
        resp = telecommand_server_proxy(request)
        print(resp)

        # # CTL
        # msg = UpdateParameterRequest()
        # buff = io.BytesIO()
        # msg.serialize(buff)
        # command.msg_type = Message.INTBALL2_SERIALIZED_BINARY_DATA
        # command.name = '/ctl/update_params'
        # command.data = buff.getvalue()
        # request.command = command
        # resp = telecommand_server_proxy(request)
        # print(resp)

        # LED
        # msg = LEDColors()
        # for i in range(8):
        #     msg.colors.append(ColorRGBA())
        #     msg.colors[i].r = 1.0
        #     msg.colors[i].g = 1.0
        #     msg.colors[i].b = 1.0
        #     msg.colors[i].a = 0.0

        # buff = io.BytesIO()
        # msg.serialize(buff)
        # command.msg_type = Message.INTBALL2_SERIALIZED_BINARY_DATA
        # command.name = '/led_display/led_colors'
        # command.data = buff.getvalue()
        # request.command = command
        # resp = telecommand_server_proxy(request)
        # print(resp)

        # # Service
        # msg = GetRosParamsRequest()
        # msg.ids = ['/parameter_manager/publish_param_list_path','/parameter_manager/rate']
        # # msg.ids = ['/parameter_manager/publish_param_list_path']

        # buff = io.BytesIO()
        # msg.serialize(buff)
        # command.msg_type = Message.INTBALL2_SERIALIZED_BINARY_DATA
        # command.name = '/parameter_manager/get_ros_params'
        # command.data = buff.getvalue()
        # request.command = command
        # resp = telecommand_server_proxy(request)
        # print(resp)

        # Topic, yaml
        #msg = TargetGoal()
        #command = Message()
        #command.format = Message.INTBALL2_YAML_UTF8
        ## /target_action_topic/action_goall
        #command.id = '/thl/fan_status'
        #command.data = str(msg).strip().encode('utf-8')
        #request.command.append(command)
        #resp = telecommand_server_proxy(request)
        #print(resp)

        # Docking station

#         # telemetry ip
#         command_id = [0x02, 0x12]
#         command_value = [10, 20, 83, 38]

#         # port number
# #        command_id = [0x03, 0x12]
# #        port_value = 12345
# #        command_value = [port_value.to_bytes(2, 'little')[0], port_value.to_bytes(2, 'little')[1]]
#         command_bytes_list = command_id + command_value
#         command.msg_type = Message.DOCK_ROW_BINARY_DATA
#         command.name = ''
#         command.data = bytes(command_bytes_list)
#         request.command = command
#         resp = telecommand_server_proxy(request)
#         print(resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
