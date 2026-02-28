#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import contextlib
import io
import json
import socket
import threading
import yaml

import rclpy
from rclpy.node import Node
from communication_software.msg import Message
from communication_software.srv import Telecommand, Setting

TELECOMMAND_DATA_MAX_SIZE = 100


class TelecommandBridge(Node):
    LOCK_TELECOMMAND = threading.RLock()

    def __init__(self):
        super().__init__('telecommand_bridge')

        self.declare_parameter('communication_config_path', '')
        self.declare_parameter('intball2_telecommand_target_ip', ['127.0.0.1'])
        self.declare_parameter('intball2_telecommand_target_port', [23456])
        self.declare_parameter('dock_telecommand_target_ip', '0.0.0.0')
        self.declare_parameter('dock_telecommand_target_port', 49303)
        self.declare_parameter('pocs_target_host', '0.0.0.0')
        self.declare_parameter('pocs_target_port', 34567)
        self.declare_parameter('simulate_pocs', True)
        self.declare_parameter('required_telecommand_duration', 1.0)
        self.declare_parameter('telecommand_timeout', 5.0)

        try:
            self.communication_config_path = self.get_parameter('communication_config_path').value
            self.intball2_target_ip = self.get_parameter('intball2_telecommand_target_ip').value
            self.intball2_target_port = self.get_parameter('intball2_telecommand_target_port').value
            self.intball2_target_index = 0
            self.address_intball2 = (self.intball2_target_ip[self.intball2_target_index],
                                     self.intball2_target_port[self.intball2_target_index])
            self.dock_target_ip = self.get_parameter('dock_telecommand_target_ip').value
            self.dock_target_port = self.get_parameter('dock_telecommand_target_port').value
            self.address_dock = (self.dock_target_ip, self.dock_target_port)
            self.pocs_target_host = self.get_parameter('pocs_target_host').value
            self.pocs_target_port = self.get_parameter('pocs_target_port').value
            self.address_pocs = (self.pocs_target_host, self.pocs_target_port)
            self.simulate_pocs = self.get_parameter('simulate_pocs').value
            self.command_seq = 0
            self.last_telecommand_time = self.get_clock().now()
            self.required_telecommand_duration_sec = self.get_parameter('required_telecommand_duration').value
            self.telecommand_timeout = self.get_parameter('telecommand_timeout').value
        except Exception as e:
            self.get_logger().error(
                'telecommand_bridge will stop. '
                'because of parameter error: {}'.format(e))
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
                self.get_logger().debug('pocs_header(Int-Ball2): ip {} port {} header {}'
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
            self.get_logger().debug('pocs_header(Docking station): ip {} port {} header {}'
                                    .format(self.dock_target_ip, self.dock_target_port, header_buf.hex()))
            self.dock_pocs_header = bytes(header_buf)
        except Exception as e:
            raise e

    def __handle_telecommand_bridge_setting(self, request, response):
        if request.intball_index >= len(self.intball2_target_ip):
            self.get_logger().error('Invalid range: intball_index {}'.format(request.intball_index))
            response.result = Setting.Response.FAILED
            return response
        self.intball2_target_index = request.intball_index
        self.address_intball2 = (self.intball2_target_ip[self.intball2_target_index],
                                 self.intball2_target_port[self.intball2_target_index])
        self.get_logger().info('Settings changed: intball2_target {}'.format(self.address_intball2))
        response.result = Setting.Response.SUCCESS
        return response

    def __handle_telecommand_bridge(self, request, response):
        if request.command.msg_type == Message.DOCK_ROW_BINARY_DATA:
            return self.__send_dock_command(request, response)
        else:
            return self.__send_intball2_command(request, response)

    def __send_intball2_command(self, request, response):
        # Create command bytes from raw data
        try:
            command_bytes = bytes(request.command.data)
        except Exception as e:
            response.result = Telecommand.Response.INVALID_COMMAND
            response.message = '{}: {}'.format(type(e).__name__, str(e))
            return response

        self.command_seq = self.command_seq + 1 if self.command_seq < 255 else 1

        # Create a command header bytes
        command_header = bytearray([self.command_seq, 0, 0])
        command_bytes_list = []

        max_data_length_without_header = TELECOMMAND_DATA_MAX_SIZE - len(command_header)
        all_command_bytes_size_without_null = (len(command_header) *
                                               (-(-len(command_bytes) //
                                                  max_data_length_without_header)) + len(command_bytes))
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

        # Send the command bytes
        try:
            with self.LOCK_TELECOMMAND:
                import time
                for i in range(0, len(command_bytes_list)):
                    current_time = self.get_clock().now()
                    duration_since_last = (current_time - self.last_telecommand_time).nanoseconds / 1e9
                    if duration_since_last < self.required_telecommand_duration_sec:
                        sleep_sec = self.required_telecommand_duration_sec - duration_since_last
                        self.get_logger().info(
                            'Wait {} second for the next telecommand transmission.'.format(sleep_sec))
                        time.sleep(sleep_sec)
                    command_header[2] = i + 1
                    self.__send_bytes(command_header + command_bytes_list[i], self.address_intball2,
                                      self.intball2_pocs_header[self.intball2_target_index])
                    self.last_telecommand_time = self.get_clock().now()
                self.get_logger().info("Int-Ball2's telecommand has been sent.")

        except Exception as e:
            self.get_logger().error('{}: {}'.format(type(e).__name__, str(e)))
            self.last_telecommand_time = self.get_clock().now()
            response.result = Telecommand.Response.SEND_FAILED
            response.message = '{}: {}'.format(type(e).__name__, str(e))
            return response

        response.result = Telecommand.Response.SUCCESS
        response.message = ''
        return response

    def __send_dock_command(self, request, response):
        try:
            with self.LOCK_TELECOMMAND:
                import time
                current_time = self.get_clock().now()
                duration_since_last = (current_time - self.last_telecommand_time).nanoseconds / 1e9
                if duration_since_last < self.required_telecommand_duration_sec:
                    sleep_sec = self.required_telecommand_duration_sec - duration_since_last
                    self.get_logger().info(
                        'Wait {} second for the next telecommand transmission.'.format(sleep_sec))
                    time.sleep(sleep_sec)

                self.__send_bytes(bytes(request.command.data), self.address_dock, self.dock_pocs_header)
                self.get_logger().info("Docking station's telecommand has been sent.")
                self.last_telecommand_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error('{}: {}'.format(type(e).__name__, str(e)))
            self.last_telecommand_time = self.get_clock().now()
            response.result = Telecommand.Response.SEND_FAILED
            response.message = '{}: {}'.format(type(e).__name__, str(e))
            return response

        response.result = Telecommand.Response.SUCCESS
        response.message = ''
        return response

    def __send_bytes(self, data_bytes, address, pocs_header):
        socket_address = address

        with io.BytesIO() as srv_struct:
            if self.simulate_pocs:
                # Simulate POCS (append data length)
                srv_struct.write(b'\x00\x00')

                # Write command bytes to the send buffer
                srv_struct.write(data_bytes)

                # It must be an even number of bytes
                self.__padding(srv_struct)
                len_srv = srv_struct.tell() - 2

                # Simulate POCS (zero padding)
                remaining = TELECOMMAND_DATA_MAX_SIZE - (srv_struct.tell() - 2)
                if remaining > 0:
                    padding_bytes = bytearray(remaining)
                    srv_struct.write(padding_bytes)

                self.get_logger().info(
                    'Simulate POCS: len_srv={} remaining={}'.format(len_srv, remaining))

                # Simulate POCS (effective data length)
                srv_struct.seek(0)
                srv_struct.write(len_srv.to_bytes(2, 'big'))

            else:
                socket_address = self.address_pocs

                # Write header bytes to the send buffer
                srv_struct.write(pocs_header)
                srv_struct.write(b'\x00\x00')

                # Write command bytes to the send buffer
                srv_struct.write(data_bytes)

                # It must be an even number of bytes
                self.__padding(srv_struct)

                # Data size calculation and writing
                len_srv = srv_struct.tell() - 2 - len(pocs_header)
                srv_struct.seek(0 + len(pocs_header))
                srv_struct.write(len_srv.to_bytes(2, 'big'))

            self.get_logger().info('Service: address={} send_length={} (simulate_pocs={})'
                                   .format(socket_address, len(srv_struct.getvalue()), self.simulate_pocs))
            self.get_logger().info('Data: {}'.format(srv_struct.getvalue().hex()))

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
                self.get_logger().debug('config yaml path {}'.format(self.communication_config_path))
                self.communication_config = yaml.load(yaml_file, Loader=yaml.FullLoader)
                self.get_logger().debug('loaded yaml {}'.format(self.communication_config))
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().error(
                'telecommand_bridge will stop. '
                'because of the failure to read the config file: '
                '"{}".'.format(self.communication_config_path))
            raise e

        self.create_service(Telecommand, 'telecommand_bridge', self.__handle_telecommand_bridge)
        self.create_service(Setting, 'telecommand_bridge_setting', self.__handle_telecommand_bridge_setting)
        self.get_logger().info("Ready to telecommand_bridge.")
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = TelecommandBridge()
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
