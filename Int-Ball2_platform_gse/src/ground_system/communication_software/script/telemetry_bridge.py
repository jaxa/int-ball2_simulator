#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# license removed for brevity

import rclpy
from rclpy.node import Node
import io
import socketserver
import socket
import concurrent.futures
import pickle
import yaml
from std_msgs.msg import UInt16, UInt8
from builtin_interfaces.msg import Time
from communication_software.msg import Telemetry, Message


class TelemetryBridge(Node):

    def __init__(self):
        super().__init__('telemetry_bridge')
        try:
            self.declare_parameter('communication_config_path', '')
            self.declare_parameter('intball2_telemetry_receive_port', 34567)
            self.declare_parameter('intball2_telemetry_multicast_receive', False)
            self.declare_parameter('intball2_telemetry_multicast_group', '1.0.0.0')
            self.declare_parameter('dock_telemetry_receive_port', 49304)
            self.declare_parameter('dock_telemetry_multicast_receive', False)
            self.declare_parameter('dock_telemetry_multicast_group', '1.0.0.0')
            self.declare_parameter('print_telemetry', False)

            self.communication_config_path = self.get_parameter('communication_config_path').value
            self.intball2_telemetry_receive_port = self.get_parameter('intball2_telemetry_receive_port').value
            self.intball2_telemetry_multicast_receive = self.get_parameter('intball2_telemetry_multicast_receive').value
            self.intball2_telemetry_multicast_group = self.get_parameter('intball2_telemetry_multicast_group').value
            self.dock_telemetry_receive_port = self.get_parameter('dock_telemetry_receive_port').value
            self.dock_telemetry_multicast_receive = self.get_parameter('dock_telemetry_multicast_receive').value
            self.dock_telemetry_multicast_group = self.get_parameter('dock_telemetry_multicast_group').value

            self.address_intball2 = ('0.0.0.0', self.intball2_telemetry_receive_port)
            IntBall2TelemetryHandler.publisher = self.create_publisher(Telemetry, 'telemetry_intball2', 100)
            IntBall2TelemetryHandler.print_telemetry = self.get_parameter('print_telemetry').value
            IntBall2TelemetryHandler.node = self

            self.address_dock = ('0.0.0.0', self.dock_telemetry_receive_port)
            DockTelemetryHandler.publisher = self.create_publisher(Telemetry, 'telemetry_dock', 100)
            DockTelemetryHandler.print_telemetry = self.get_parameter('print_telemetry').value
            DockTelemetryHandler.node = self
        except Exception as e:
            self.get_logger().error(
                'telemetry_bridge will stop. '
                'because of parameter error: {}'.format(e))
            raise e

    def start(self):
        try:
            # Load a config file
            with open(self.communication_config_path, 'r') as yaml_file:
                self.get_logger().debug('config yaml path {}'.format(self.communication_config_path))
                IntBall2TelemetryHandler.intball_app_config = yaml.load(yaml_file, Loader=yaml.FullLoader)
                self.get_logger().debug('loaded yaml {}'.format(IntBall2TelemetryHandler.intball_app_config))
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().error(
                'telemetry_bridge will stop. '
                'because of the failure to read the config file: '
                '"{}".'.format(self.communication_config_path))
            raise e

        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            self.server_intball2 = socketserver.ThreadingUDPServer(self.address_intball2, IntBall2TelemetryHandler)
            if self.intball2_telemetry_multicast_receive:
                self.server_intball2.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                                       socket.inet_aton(self.intball2_telemetry_multicast_group)
                                                       + socket.inet_aton('0.0.0.0'))
            self.server_dock = socketserver.ThreadingUDPServer(self.address_dock, DockTelemetryHandler)
            if self.dock_telemetry_multicast_receive:
                self.server_dock.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                                   socket.inet_aton(self.dock_telemetry_multicast_group)
                                                   + socket.inet_aton('0.0.0.0'))

            executor.submit(self.server_intball2.serve_forever)
            executor.submit(self.server_dock.serve_forever)

            self.get_logger().info("Ready to telemetry_bridge.")
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                pass
            finally:
                self.server_intball2.shutdown()
                self.server_dock.shutdown()


class IntBall2TelemetryHandler(socketserver.BaseRequestHandler):
    intball_app_config = {}
    publisher = None
    print_telemetry = False
    node = None

    def handle(self):
        logger = IntBall2TelemetryHandler.node.get_logger()
        logger.info('client: {}'.format(self.client_address))
        datagram = self.request[0]
        logger.info('intball2 telemetry len {}'.format(len(datagram)))
        logger.debug('intball2 telemetry data {}'.format(datagram))

        telemetry_msg = Telemetry()
        now = IntBall2TelemetryHandler.node.get_clock().now().to_msg()
        telemetry_msg.received_time = now

        # unpickle
        datagram_obj = io.BytesIO()
        datagram_obj.write(datagram)
        datagram_obj.seek(0)
        unpickled = pickle.load(datagram_obj)
        logger.debug('unpickled {}'.format(unpickled))
        logger.info('unpickled.keys() {}'.format(unpickled.keys()))

        # NOTE: In ROS 2, direct binary serialization/deserialization of messages
        # is not supported in the same way as ROS 1. The telemetry data handling
        # would need to be updated to use ROS 2's serialization mechanisms.
        # For now, raw binary data is forwarded as-is in the Message.data field.

        for key, value in unpickled.items():
            msg = Message()
            msg.id = key
            msg.msg_type = Message.INTBALL2_SERIALIZED_BINARY_DATA
            msg.name = str(key)
            if isinstance(value, bytes):
                msg.data = list(value)
            elif isinstance(value, (list, tuple)):
                msg.data = list(value)
            else:
                msg.data = list(bytes(str(value), 'utf-8'))
            telemetry_msg.data.append(msg)

        IntBall2TelemetryHandler.publisher.publish(telemetry_msg)
        if self.print_telemetry:
            logger.info(str(telemetry_msg))


class DockTelemetryHandler(socketserver.BaseRequestHandler):
    publisher = None
    print_telemetry = False
    node = None

    def handle(self):
        logger = DockTelemetryHandler.node.get_logger()
        logger.info('client: {}'.format(self.client_address))
        datagram = self.request[0]
        logger.info('docking station telemetry len {}'.format(len(datagram)))
        logger.debug('docking station telemetry data {}'.format(datagram))

        telemetry_msg = Telemetry()
        now = DockTelemetryHandler.node.get_clock().now().to_msg()
        telemetry_msg.received_time = now
        telemetry_single_message = Message()
        telemetry_single_message.msg_type = Message.DOCK_ROW_BINARY_DATA
        telemetry_single_message.name = 'dock'
        telemetry_single_message.data = list(datagram)
        telemetry_msg.data.append(telemetry_single_message)

        DockTelemetryHandler.publisher.publish(telemetry_msg)
        if self.print_telemetry:
            logger.info(str(telemetry_msg))


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
