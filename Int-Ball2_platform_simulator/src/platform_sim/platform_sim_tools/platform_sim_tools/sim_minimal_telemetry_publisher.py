#!/usr/bin/python3
# -*- coding:utf-8 -*-
from ib2_msgs.msg import BatteryChargeInfo
from logging import getLogger, basicConfig, INFO
from std_msgs.msg import Bool
from telemetry import TelemetryHeaderBuilder
import io
import os
import pickle
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import socket
import subprocess
import sys
import time
import yaml


basicConfig(level=INFO)
logger = getLogger(os.path.basename(__file__))


class SimMinimalTelemetryPublisher(Node):
    LEX_MIN_USER_DATA_BYTES = 68
    LEX_MAX_USER_DATA_BYTES = 1472
    SENDING_SLEEP_TIME_SEC = 3
    SENDING_SOURCE_PORT = 22234
    TELEMETRY_ID_BATTERY_CHARGE_INFO = 2301
    TELEMETRY_ID_NORMAL_FLIGHT_SOFTWARE_STATUS = 9001
    TELEMETRY_ID_PLATFORM_FLIGHT_SOFTWARE_STATUS = 9002
    intball_app_config = None

    def __init__(self):
        super().__init__('sim_minimal_telemetry_publisher')

        self.declare_parameter('transcommunication_config', '')
        self.declare_parameter('ocs_host', 'localhost')
        self.declare_parameter('ocs_port', 34567)

        self.__transcommunication_config = self.get_parameter(
            'transcommunication_config').get_parameter_value().string_value
        self.__ocs_host = self.get_parameter(
            'ocs_host').get_parameter_value().string_value
        self.__ocs_port = self.get_parameter(
            'ocs_port').get_parameter_value().integer_value

        # Read setting YAML file
        try:
            with open(self.__transcommunication_config, 'r') as yaml_file:
                logger.info('config yaml path {}'.format(self.__transcommunication_config))
                self.intball_app_config = yaml.load(
                    yaml_file, Loader=yaml.FullLoader)
                logger.info('loaded yaml {}'.format(
                    self.intball_app_config))
        except Exception as e:
            logger.error(e)
            logger.error('trans communication node will stop.'
                         'because of the failure to read the config file: "{}".'
                         .format(self.__transcommunication_config))
            logger.info('TransCommunication.receive out')
            raise e
        self.telemetry_header_builder = TelemetryHeaderBuilder(self.intball_app_config)

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('', SimMinimalTelemetryPublisher.SENDING_SOURCE_PORT))

        # Use a timer to periodically send telemetry instead of a while loop
        self.timer = self.create_timer(
            SimMinimalTelemetryPublisher.SENDING_SLEEP_TIME_SEC,
            self.send_telemetry)

    def send_telemetry(self):
        try:
            data_dict = {}

            self.telemetry_header_builder.write_header(0xFFFF, 1, 1, data_dict, 0xFF,
                                                       ros_timestamp=self.get_clock().now().to_msg())

            # Battery
            battery_charge_info = BatteryChargeInfo()
            battery_charge_info.battery_remain = 100
            temp_buf = io.BytesIO()
            battery_charge_info.serialize(temp_buf)
            data_dict[SimMinimalTelemetryPublisher.TELEMETRY_ID_BATTERY_CHARGE_INFO] = temp_buf.getvalue()

            # State of the normal flight software (ROS)
            normal_flight_software_status = Bool(data=False)
            temp_buf = io.BytesIO()
            normal_flight_software_status.serialize(temp_buf)
            data_dict[SimMinimalTelemetryPublisher.TELEMETRY_ID_NORMAL_FLIGHT_SOFTWARE_STATUS] = temp_buf.getvalue()

            # State of the platform flight software (ROS)
            platform_flight_software_status = Bool()
            # Check that the process of the platform_manager node is up and running
            cmd_return = subprocess.run(('ps aux | '
                                         'grep -v grep | '
                                         'grep \'platform_manager\' | '
                                         'wc -l'),
                                        shell=True, check=False,
                                        stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
            platform_flight_software_status.data = (cmd_return
                                                    and cmd_return.isdecimal()
                                                    and int(cmd_return) != 0)
            temp_buf = io.BytesIO()
            platform_flight_software_status.serialize(temp_buf)
            data_dict[SimMinimalTelemetryPublisher.TELEMETRY_ID_PLATFORM_FLIGHT_SOFTWARE_STATUS] = temp_buf.getvalue()

            # Format the data for transmission
            pickled_telemetry = pickle.dumps(data_dict, protocol=3)
            zero_padded_pickled_telemetry = pickled_telemetry + (self.LEX_MIN_USER_DATA_BYTES -
                                                                 sys.getsizeof(pickled_telemetry)) * b'\0'
            if len(zero_padded_pickled_telemetry) > SimMinimalTelemetryPublisher.LEX_MAX_USER_DATA_BYTES:
                logger.error('[send_telemetry] telemetry size {} exceeds max size (={} bytes)'.format(
                    len(zero_padded_pickled_telemetry), SimMinimalTelemetryPublisher.LEX_MAX_USER_DATA_BYTES))
                raise Exception("telemetry size error")

            # Send telemetry
            self.udp_socket.sendto(zero_padded_pickled_telemetry, (self.__ocs_host, self.__ocs_port))
            logger.info('[send_telemetry] finish to send. port {}, len {}, byte_size {}'.format(
                        self.udp_socket, len(zero_padded_pickled_telemetry),
                        sys.getsizeof(zero_padded_pickled_telemetry)))

        except Exception as e:
            logger.error('[send_telemetry]fail to send. error {}'.format(e))


def main(args=None):
    rclpy.init(args=args)
    node = SimMinimalTelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.udp_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
