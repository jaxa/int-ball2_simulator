#!/usr/bin/python3
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
import roslib.message
import io
import socketserver
import socket
import concurrent.futures
import pickle
import yaml
from std_msgs.msg import Time, UInt16, UInt8
from ib2_msgs import *
from communication_software.msg import *
from struct import *

NODE_LOG_LEVEL=rospy.INFO



class TelemetryBridge(object):

    def __init__(self):
        rospy.init_node('telemetry_bridge', log_level=NODE_LOG_LEVEL)
        try:
            self.communication_config_path = rospy.get_param('communication_config_path')
            self.intball2_telemetry_receive_port = rospy.get_param('intball2_telemetry_receive_port')
            self.intball2_telemetry_multicast_receive = rospy.get_param('intball2_telemetry_multicast_receive')
            self.intball2_telemetry_multicast_group = rospy.get_param('intball2_telemetry_multicast_group')
            self.dock_telemetry_receive_port = rospy.get_param('dock_telemetry_receive_port')
            self.dock_telemetry_multicast_receive = rospy.get_param('dock_telemetry_multicast_receive')
            self.dock_telemetry_multicast_group = rospy.get_param('dock_telemetry_multicast_group')
            self.address_intball2 = ('0.0.0.0', self.intball2_telemetry_receive_port)
            IntBall2TelemetryHandler.publisher = rospy.Publisher('telemetry_intball2', Telemetry, queue_size=100)
            IntBall2TelemetryHandler.print_telemetry = rospy.get_param('print_telemetry')
            self.address_dock = ('0.0.0.0', self.dock_telemetry_receive_port)
            DockTelemetryHandler.publisher = rospy.Publisher('telemetry_dock', Telemetry, queue_size=100)
            DockTelemetryHandler.print_telemetry = rospy.get_param('print_telemetry')
        except KeyError as e:
            rospy.logerr(('telemetry_bridge will stop. '
                          'because of the lack of rosparams "{}".').format(e))
            raise e

        buff = io.BytesIO()
        Time().serialize(buff)
        IntBall2TelemetryHandler.addtional_timestamp_length = buff.tell()

    def start(self):
        try:
            # Load a config file
            with open(self.communication_config_path, 'r') as yaml_file:
                rospy.logdebug('config yaml path {}'.format(self.communication_config_path))
                IntBall2TelemetryHandler.intball_app_config = yaml.load(yaml_file, Loader=yaml.FullLoader)
                rospy.logdebug('loaded yaml {}'.format(IntBall2TelemetryHandler.intball_app_config))
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr(('telemetry_bridge will stop. '
                          'because of the failure to read the config file: '
                          '"{}".').format(self.communication_config_path))
            raise e

        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            self.server_intball2 = socketserver.ThreadingUDPServer(self.address_intball2, IntBall2TelemetryHandler)
            if self.intball2_telemetry_multicast_receive:
                self.server_intball2.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                                       struct.pack('4sl',
                                                                   socket.inet_aton(self.intball2_telemetry_multicast_group),
                                                                   socket.INADDR_ANY))
            self.server_dock = socketserver.ThreadingUDPServer(self.address_dock, DockTelemetryHandler)
            if self.dock_telemetry_multicast_receive:
                self.server_dock.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                                   struct.pack('4sl',
                                                               socket.inet_aton(self.dock_telemetry_multicast_group),
                                                               socket.INADDR_ANY))
 

            executor.submit(self.server_intball2.serve_forever)
            executor.submit(self.server_dock.serve_forever)

            rospy.loginfo("Ready to telemetry_bridge.")
            rospy.on_shutdown(self.shutdown_handler)
            rospy.spin()

    def shutdown_handler(self):
        self.server_intball2.shutdown()
        self.server_dock.shutdown()

class IntBall2TelemetryHandler(socketserver.BaseRequestHandler, object):
    intball_app_config = {}
    publisher = None
    print_telemetry = False
    addtional_timestamp_length = 0

    def handle(self):
        rospy.loginfo('client: {}'.format(self.client_address))
        datagram = self.request[0]
        rospy.loginfo('intball2 telemetry len {}'.format(len(datagram)))
        rospy.logdebug('intball2 telemetry data {}'.format(datagram))

        telemetry_msg = Telemetry()
        telemetry_msg.received_time = rospy.get_rostime()

        # unpickle
        datagram_obj = io.BytesIO()
        datagram_obj.write(datagram)
        datagram_obj.seek(0)
        unpickled = pickle.load(datagram_obj)
        rospy.logdebug('unpickled {}'.format(unpickled))
        rospy.loginfo('unpickled.keys() {}'.format(unpickled.keys()))

        print_data_dict = []

        # append header
        unpickled_index = IntBall2TelemetryHandler.intball_app_config['telemetry']['header']['id_timestamp']
        telemetry_msg.data.append(self.create_telemetry_single_message(unpickled_index,
                                                                       'timestamp',
                                                                       unpickled[unpickled_index]))
        if self.print_telemetry:
            data = Time()
            data.deserialize(unpickled[unpickled_index])
            print_data_dict.append({'id': unpickled_index, 'name': 'timestamp', 'data': data})

        unpickled_index = IntBall2TelemetryHandler.intball_app_config['telemetry']['header']['id_last_executed_command']
        telemetry_msg.data.append(self.create_telemetry_single_message(unpickled_index,
                                                                       'last_executed_command',
                                                                       unpickled[unpickled_index]))
        if self.print_telemetry:
            data = UInt16()
            data.deserialize(unpickled[unpickled_index])
            print_data_dict.append({'id': unpickled_index, 'name': 'last_executed_command', 'data': data})

        unpickled_index = IntBall2TelemetryHandler.intball_app_config['telemetry']['header']['id_split_number']
        telemetry_msg.data.append(self.create_telemetry_single_message(unpickled_index,
                                                                       'split_number',
                                                                       unpickled[unpickled_index]))
        if self.print_telemetry:
            data = UInt8()
            data.deserialize(unpickled[unpickled_index])
            print_data_dict.append({'id': unpickled_index, 'name': 'split_number', 'data': data})

        unpickled_index = IntBall2TelemetryHandler.intball_app_config['telemetry']['header']['id_current_split_index']
        telemetry_msg.data.append(self.create_telemetry_single_message(unpickled_index,
                                                                       'current_split_index',
                                                                       unpickled[unpickled_index]))
        if self.print_telemetry:
            data = UInt8()
            data.deserialize(unpickled[unpickled_index])
            print_data_dict.append({'id': unpickled_index, 'name': 'current_split_index', 'data': data})

        unpickled_index = IntBall2TelemetryHandler.intball_app_config['telemetry']['header']['id_sending_port_index']
        telemetry_msg.data.append(self.create_telemetry_single_message(unpickled_index,
                                                                       'sending_port_index',
                                                                       unpickled[unpickled_index]))
        if self.print_telemetry:
            data = UInt8()
            data.deserialize(unpickled[unpickled_index])
            print_data_dict.append({'id': unpickled_index, 'name': 'id_sending_port_index', 'data': data})

        # purse telemetry contents: normal
        for config in IntBall2TelemetryHandler.intball_app_config['telemetry']['normal']:
            content_id = config['id']
            content_name = config['name']
            rospy.logdebug('config: id {} name {}'.format(content_id, content_name))
            if content_id not in unpickled.keys():
                rospy.logdebug('{} is not included'.format(content_name))
                continue
            rospy.logdebug('deserialize id {}, name {} type {}'.format(content_id,
                                                                       content_name,
                                                                       config['data_class']))
            self.append_telemetry_data(content_id, content_name, config['data_class'], config,
                                       unpickled[content_id], print_data_dict, telemetry_msg)

        # purse telemetry contents: split
        for config in IntBall2TelemetryHandler.intball_app_config['telemetry']['split']:
            if 'value_config' in config:
                for c in config['value_config']:
                    content_id = c['id']
                    content_name = config['name'] + '.' + c['value']
                    if content_id not in unpickled.keys():
                        rospy.logdebug('{} is not included'.format(content_name))
                        continue
                    rospy.logdebug('deserialize id {}, name {} type {}'.format(content_id,
                                                                               content_name,
                                                                               c['data_class']))
                    self.append_telemetry_data(content_id, content_name, c['data_class'], c,
                                               unpickled[content_id], print_data_dict, telemetry_msg)

            if 'split_config' in config:
                for c in config['split_config']:
                    content_id_range = c['id_range']
                    content_name = config['name'] + '.' + c['value_to_split']
                    filtered_unpickled = {key : unpickled[key]
                                          for key in unpickled.keys()
                                          if key >= content_id_range[0] and key <= content_id_range[1]}
                    if len(filtered_unpickled) > 0:
                        for key in filtered_unpickled.keys():
                            content_id = key
                            unpickled_data = filtered_unpickled[key]
                            rospy.logdebug('deserialize id {}, name {} type {}'.format(content_id,
                                                                                       content_name,
                                                                                       c['data_class_after_split']))
                            self.append_telemetry_data(content_id, content_name, c['data_class_after_split'], c,
                                                       unpickled_data, print_data_dict, telemetry_msg)
                    else:
                        rospy.logdebug('{} is not included'.format(content_name))
                        continue

        IntBall2TelemetryHandler.publisher.publish(telemetry_msg)
        if self.print_telemetry:
            for print_data in print_data_dict:
                rospy.loginfo('---------------------------------------------------')
                rospy.loginfo('ID:{} Name:{}'.format(print_data['id'], print_data['name']))
                rospy.loginfo(str(print_data['data']))
                if 'additional_timestamp' in print_data:
                    rospy.loginfo('Additional timestamp: {}.{}'.format(print_data['additional_timestamp'].secs,
                                                                       print_data['additional_timestamp'].nsecs))

    def create_telemetry_single_message(self, id, name, data):
        telemetry_single_message = Message()
        telemetry_single_message.id = id
        telemetry_single_message.msg_type = Message.INTBALL2_SERIALIZED_BINARY_DATA
        telemetry_single_message.name = name
        telemetry_single_message.data = data
        return telemetry_single_message

    def append_telemetry_data(self, content_id, content_name, data_class_name, config,
                              unpickled_data, print_data_dict, telemetry_msg):
        try:
            # check deserialize
            data_class = roslib.message.get_message_class(data_class_name)
            if data_class is None:
                # service response message
                data_class = roslib.message.get_service_class(data_class_name)._response_class
            data = data_class()
            data.deserialize(unpickled_data)
            rospy.logdebug('serialized data length {} (id={})'.format(len(unpickled_data), content_id))

            msg = self.create_telemetry_single_message(content_id, content_name, unpickled_data)
            if 'add_timestamp' in config and config['add_timestamp']:
                self.append_additional_timestamp(unpickled_data, msg)
            telemetry_msg.data.append(msg)
            if self.print_telemetry:
                if msg.additional_timestamp.secs > 0:
                    data = {'id': content_id, 'name': content_name, 'data': data,
                            'additional_timestamp': msg.additional_timestamp}
                else:
                    data = {'id': content_id, 'name': content_name, 'data': data}
                print_data_dict.append(data)
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn('cannot deserialize id {}, name {}, type {}'.format(content_id,
                                                                              content_name,
                                                                              data_class_name))

    def append_additional_timestamp(self, unpickled_data, msg):
        timestamp = Time()
        timestamp.deserialize(unpickled_data[-IntBall2TelemetryHandler.addtional_timestamp_length:])
        msg.additional_timestamp = timestamp.data

class DockTelemetryHandler(socketserver.BaseRequestHandler, object):
    publisher = None
    print_telemetry = False

    def handle(self):
        rospy.loginfo('client: {}'.format(self.client_address))
        datagram = self.request[0]
        rospy.loginfo('docking station telemetry len {}'.format(len(datagram)))
        rospy.logdebug('docking station telemetry data {}'.format(datagram))

        rospy.logdebug('dock: Telemetry Counter {}'.format(datagram[0]))
        rospy.logdebug('dock: Command Counter {}'.format(datagram[1]))
        rospy.logdebug('dock: Charge State {}'.format(datagram[2]))
        rospy.logdebug('dock: Motor State {}'.format(datagram[3]))
        rospy.logdebug('dock: Switch State {}'.format(datagram[4]))
        rospy.logdebug('dock: Command State {}'.format(datagram[5]))
        rospy.logdebug('dock: Version {}'.format(datagram[6]))
        rospy.logdebug('dock: Motor Temp {}'.format(int.from_bytes(datagram[7:9], byteorder='little', signed=True)))
        rospy.logdebug('dock: DCDC Temp {}'.format(int.from_bytes(datagram[9:11], byteorder='little', signed=True)))
        rospy.logdebug('dock: Power1 Current {}'.format(int.from_bytes(datagram[11:13], byteorder='little')))
        rospy.logdebug('dock: Power2 Current {}'.format(int.from_bytes(datagram[13:15], byteorder='little')))
        rospy.logdebug('dock: Temp Alert {}'.format(datagram[15]))
        rospy.logdebug('dock: Charge Time {}'.format(int.from_bytes(datagram[16:18], byteorder='little')))

        telemetry_msg = Telemetry()
        telemetry_msg.received_time = rospy.get_rostime()
        telemetry_single_message = Message()
        telemetry_single_message.msg_type = Message.DOCK_ROW_BINARY_DATA
        telemetry_single_message.name = 'dock'
        telemetry_single_message.data = datagram
        telemetry_msg.data.append(telemetry_single_message)

        DockTelemetryHandler.publisher.publish(telemetry_msg)
        if self.print_telemetry:
            rospy.loginfo(str(telemetry_msg))

if __name__ == '__main__':
    server = TelemetryBridge()
    server.start()
