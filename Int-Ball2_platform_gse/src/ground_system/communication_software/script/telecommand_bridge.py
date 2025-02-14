#!/usr/bin/python3
# -*- coding: utf-8 -*-

import contextlib
import genpy
import io
import json
import rospy
import roslib.message
import socket
import std_msgs
import threading
import yaml
from communication_software.msg import *
from communication_software.srv import *

NODE_LOG_LEVEL=rospy.INFO
TELECOMMAND_DATA_MAX_SIZE = 100


class TelecommandBridge(object):
    LOCK_TELECOMMAND = threading.RLock()

    def __init__(self):
        rospy.init_node('telecommand_bridge', log_level=NODE_LOG_LEVEL)
        try:
            self.communication_config_path = rospy.get_param('communication_config_path')
            self.intball2_target_ip = rospy.get_param('intball2_telecommand_target_ip')
            self.intball2_target_port = rospy.get_param('intball2_telecommand_target_port')
            self.intball2_target_index = 0
            self.address_intball2 = (self.intball2_target_ip[self.intball2_target_index],
                                     self.intball2_target_port[self.intball2_target_index])
            self.dock_target_ip = rospy.get_param('dock_telecommand_target_ip')
            self.dock_target_port = rospy.get_param('dock_telecommand_target_port')
            self.address_dock = (self.dock_target_ip, self.dock_target_port)
            self.pocs_target_host = rospy.get_param('pocs_target_host')
            self.pocs_target_port = rospy.get_param('pocs_target_port')
            self.address_pocs = (self.pocs_target_host, self.pocs_target_port)
            self.simulate_pocs = rospy.get_param('simulate_pocs')
            self.command_seq = 0
            self.last_telecommand_timestamp = rospy.Time.now()
            self.required_telecommand_duration = rospy.Duration(rospy.get_param('required_telecommand_duration'))
            self.telecommand_timeout = rospy.get_param('telecommand_timeout')
        except KeyError as e:
            rospy.logerr(('telecommand_bridge will stop. '
                          'because of the lack of rosparams "{}".').format(e))
            raise e

        try:
            # Create header bytes for communication with POCS

            # Int-Ball2
            self.intball2_pocs_header = []
            for intball_ip, intball_port in zip(self.intball2_target_ip, self.intball2_target_port):
                header_buf = bytearray()
                intball_ip_addr_split = intball_ip.split('.')
                header_buf.append(int(intball_ip_addr_split[0]))
                header_buf.append(int(intball_ip_addr_split[1]))
                header_buf.append(int(intball_ip_addr_split[2]))
                header_buf.append(int(intball_ip_addr_split[3]))
                header_buf += int(intball_port).to_bytes(2, byteorder='big')
                rospy.logdebug('pocs_header(Int-Ball2): ip {} port {} header {}'
                                .format(intball_ip, intball_port, header_buf.hex()))
                self.intball2_pocs_header.append(bytes(header_buf))

            # Docking station
            header_buf = bytearray()
            dock_ip_addr_split = self.dock_target_ip.split('.')
            header_buf.append(int(dock_ip_addr_split[0]))
            header_buf.append(int(dock_ip_addr_split[1]))
            header_buf.append(int(dock_ip_addr_split[2]))
            header_buf.append(int(dock_ip_addr_split[3]))
            header_buf += int(self.dock_target_port).to_bytes(2, byteorder='big')
            rospy.logdebug('pocs_header(Docking station): ip {} port {} header {}'
                            .format(self.dock_target_ip, self.dock_target_port, header_buf.hex()))
            self.dock_pocs_header = bytes(header_buf)
        except Exception as e:
            raise e

    def __handle_telecommand_bridge_setting(self, request):
        if  request.intball_index >= len(self.intball2_target_ip):
            rospy.logerr('Invalid range: intball_index {}'.format(request.intball_index))
            return SettingResponse(TelecommandResponse.FAILED)
        self.intball2_target_index = request.intball_index
        self.address_intball2 = (self.intball2_target_ip[self.intball2_target_index],
                                 self.intball2_target_port[self.intball2_target_index])
        rospy.loginfo('Settings changed: intball2_target {}'.format(self.address_intball2))
        return SettingResponse(TelecommandResponse.SUCCESS)

    def __handle_telecommand_bridge(self, request):
        if request.command.msg_type == Message.DOCK_ROW_BINARY_DATA:
            return self.__send_dock_command(request)
        else:
            return self.__send_intball2_command(request)

    def __send_intball2_command(self, request):
        # Create a command bytes
        try:
            command_bytes = self.__parse_ros_command_request(request)
        except Exception as e:
            return TelecommandResponse(TelecommandResponse.INVALID_COMMAND,
                                       '{}: {}'.format(type(e).__name__, str(e)))

        self.command_seq = self.command_seq + 1 if self.command_seq < 255 else 1

        # Create a command header bytes
        command_header = bytearray([self.command_seq, 0, 0])
        command_bytes_list = []

        max_data_length_without_header = TELECOMMAND_DATA_MAX_SIZE - len(command_header)
        all_command_bytes_size_without_null = len(command_header) * (-(-len(command_bytes) //
                                                                     max_data_length_without_header)) + len(command_bytes)
        if all_command_bytes_size_without_null > TELECOMMAND_DATA_MAX_SIZE:
            split_start = 0
            while split_start < all_command_bytes_size_without_null:
                split_end = split_start + max_data_length_without_header
                if split_end > all_command_bytes_size_without_null:
                    split_end = all_command_bytes_size_without_null
                command_bytes_list.append(command_bytes[split_start:split_end])
                split_start = split_end
        else:
            command_bytes_list = [command_bytes]
        command_header[1] = len(command_bytes_list)
        rospy.logdebug('address={} packet_count={} all_command_bytes_size_without_null={}'
                      .format(self.address_intball2,
                              len(command_bytes_list),
                              all_command_bytes_size_without_null))

        # Send the command bytes to POCS or Int-Ball
        try:
            with self.LOCK_TELECOMMAND:
                for i in range(0,len(command_bytes_list)):
                    current_timestamp = rospy.Time.now()
                    current_telecommand_duration = current_timestamp - self.last_telecommand_timestamp
                    if current_telecommand_duration < self.required_telecommand_duration:
                        sleep_duration = self.required_telecommand_duration - current_telecommand_duration
                        rospy.loginfo('Wait {} second for the next telecommand transmission.'.format(sleep_duration.to_sec()))
                        rospy.sleep(self.required_telecommand_duration - current_telecommand_duration)
                    command_header[2] = i + 1
                    self.__send_bytes(command_header + command_bytes_list[i], self.address_intball2,
                                      self.intball2_pocs_header[self.intball2_target_index])
                    self.last_telecommand_timestamp = rospy.Time.now()
                rospy.loginfo('Int-Ball2\'s telecommand has been sent.')

        except Exception as e:
            rospy.logerr('{}: {}'.format(type(e).__name__, str(e)))
            self.last_telecommand_timestamp = rospy.Time.now()
            return TelecommandResponse(TelecommandResponse.SEND_FAILED,
                                       '{}: {}'.format(type(e).__name__, str(e)))

        return TelecommandResponse(TelecommandResponse.SUCCESS, '')

    def __send_dock_command(self, request):
        rospy.logdebug('address={} len(request.command.data)={}'
                      .format(self.address_dock, len(request.command.data)))

        # Send the command bytes to POCS or Docking station
        try:
            with self.LOCK_TELECOMMAND:
                current_timestamp = rospy.Time.now()
                current_telecommand_duration = current_timestamp - self.last_telecommand_timestamp
                if current_telecommand_duration < self.required_telecommand_duration:
                    sleep_duration = self.required_telecommand_duration - current_telecommand_duration
                    rospy.loginfo('Wait {} second for the next telecommand transmission.'.format(sleep_duration.to_sec()))
                    rospy.sleep(self.required_telecommand_duration - current_telecommand_duration)

                self.__send_bytes(request.command.data, self.address_dock, self.dock_pocs_header)
                rospy.loginfo('Docking station\'s telecommand has been sent.')
                self.last_telecommand_timestamp = rospy.Time.now()

        except Exception as e:
            rospy.logerr('{}: {}'.format(type(e).__name__, str(e)))
            self.last_telecommand_timestamp = rospy.Time.now()
            return TelecommandResponse(TelecommandResponse.SEND_FAILED,
                                       '{}: {}'.format(type(e).__name__, str(e)))

        return TelecommandResponse(TelecommandResponse.SUCCESS, '')

    def __parse_ros_command_request(self, request):
        parse_result = None

        config = [c for c in self.communication_config['telecommand'] if c['name'] == request.command.name]
        if not config:
            raise ValueError('\'name\' of request (name={})'.format(request.command.name))
        if len(config) >= 2:
            raise ValueError('duplicate name \'{}\' in the config yaml file'.format(request.command.name))

        config = config[0]
        if config['type'] == 'topic':
            data_class = roslib.message.get_message_class(config['data_class'])
            if data_class is None:
                raise ValueError('\'data_class\' of id{}'.format(config['id']))

        elif config['type'] == 'service/request':
            tmp_data_class = roslib.message.get_service_class(config['data_class'])
            if tmp_data_class is None:
                raise ValueError('\'data_class\' of id{}'.format(config['id']))
            data_class =  tmp_data_class._request_class

        if data_class is None:
            raise ValueError('\'data_class\' of "{}"'.format(config['id']))

        msg = data_class()
        rospy.logdebug('data_class:{} data_length:{}'.format(data_class, len(request.command.data)))
        if request.command.msg_type == Message.INTBALL2_SERIALIZED_BINARY_DATA:
            try:
                # Check the format of the data bytes
                msg.deserialize(request.command.data)

                # Create a command bytes to send
                parse_result = config['id'].to_bytes(2, 'little') + request.command.data
                rospy.loginfo('class={} id={} name={} data={}'.format(data_class.__name__, config['id'],
                                                                      request.command.name, str(msg)))
            except Exception as e:
                rospy.logerr(e)
                raise ValueError('\'command.data\' of request')

        if request.command.msg_type == Message.INTBALL2_YAML_UTF8:
            try:
                # Check the format of the data bytes
                buff = io.Byte__send_dock_commandsIO()
                msg_args = yaml.load(request.command.data.decode('utf-8'), Loader=yaml.FullLoader)
                if type(msg_args) == dict:
                    msg_args = [msg_args]
                now = rospy.get_rostime()
                fill_keys = { 'now': now, 'auto': std_msgs.msg.Header(stamp=now) }
                roslib.message.fill_message_args(msg, msg_args, fill_keys)
                msg.serialize(buff)

                # Create a command bytes to send
                parse_result = config['id'].to_bytes(2, 'little') + buff.getvalue()
                rospy.loginfo('class={} id={} name={} data={}'.format(data_class.__name__, config['id'],
                                                                      request.command.name, str(msg)))
            except Exception as e:
                rospy.logerr(e)
                raise ValueError('\'command.data\' of request')

        return parse_result

    def __send_bytes(self, bytes, address, pocs_header):
        socket_address = address

        with io.BytesIO() as srv_struct:
            if self.simulate_pocs:

                # Simulate POCS (append data length)
                srv_struct.write(b'\x00\x00')

                # Write command bytes to the send buffer
                srv_struct.write(bytes)

                # It must be an even number of bytes
                self.__padding(srv_struct)
                len_srv = srv_struct.tell() - 2

                # Simulate POCS (zero padding)
                remaining = TELECOMMAND_DATA_MAX_SIZE - (srv_struct.tell() - 2)
                if remaining > 0:
                    padding_bytes = bytearray(remaining)
                    srv_struct.write(padding_bytes)

                rospy.loginfo('Simulate POCS: len_srv={} remaining={}'.format(len_srv, remaining))

                # Simulate POCS (effective data length)
                srv_struct.seek(0)
                srv_struct.write(len_srv.to_bytes(2, 'big'))

            else:
                socket_address = self.address_pocs

                # Write header bytes to the send buffer
                srv_struct.write(pocs_header)
                srv_struct.write(b'\x00\x00')

                # Write command bytes to the send buffer
                srv_struct.write(bytes)

                # It must be an even number of bytes
                self.__padding(srv_struct)

                # Data size calculation and writing
                len_srv = srv_struct.tell() - 2 - len(pocs_header)
                srv_struct.seek(0 + len(pocs_header))
                srv_struct.write(len_srv.to_bytes(2, 'big'))

            rospy.loginfo('Service: address={} send_length={} (simulate_pocs={})'
                          .format(socket_address, len(srv_struct.getvalue()), self.simulate_pocs))
            rospy.loginfo('Data: {}'.format(srv_struct.getvalue().hex()))

            # Send message
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(self.telecommand_timeout)
            with contextlib.closing(client):
                client.connect(socket_address)
                client.sendall(srv_struct.getvalue())

    def __padding(self, struct):
        check_len = struct.tell() - 2
        if check_len % 2 != 0:
            struct.write(b'\x00')

    def start(self):
        try:
            # Load a config file
            with open(self.communication_config_path, 'r') as yaml_file:
                rospy.logdebug('config yaml path {}'.format(self.communication_config_path))
                self.communication_config = yaml.load(yaml_file, Loader=yaml.FullLoader)
                rospy.logdebug('loaded yaml {}'.format(self.communication_config))
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr(('telecommand_bridge will stop. '
                          'because of the failure to read the config file: '
                          '"{}".').format(self.communication_config_path))
            raise e

        rospy.Service('telecommand_bridge', Telecommand, self.__handle_telecommand_bridge)
        rospy.Service('telecommand_bridge_setting', Setting, self.__handle_telecommand_bridge_setting)
        rospy.loginfo("Ready to telecommand_bridge.")
        rospy.spin()

if __name__ == '__main__':
    server = TelecommandBridge()
    server.start()
