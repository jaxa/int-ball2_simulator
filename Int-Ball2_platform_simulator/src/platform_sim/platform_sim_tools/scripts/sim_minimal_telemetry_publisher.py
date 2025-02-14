#!/usr/bin/python3
# -*- coding:utf-8 -*-
from ib2_msgs.msg import BatteryChargeInfo
from logging import getLogger, basicConfig, INFO
from std_msgs.msg import Bool
from telemetry import TelemetryHeaderBuilder
import io
import os
import pickle
import rospy
import socket
import subprocess
import sys
import time
import yaml


basicConfig(level=INFO)
logger = getLogger(os.path.basename(__file__))


class SimMinimalTelemetryPublisher(object):
    LEX_MIN_USER_DATA_BYTES = 68
    LEX_MAX_USER_DATA_BYTES = 1472
    SENDING_SLEEP_TIME_SEC = 3
    SENDING_SOURCE_PORT = 22234
    TELEMETRY_ID_BATTERY_CHARGE_INFO = 2301
    TELEMETRY_ID_NORMAL_FLIGHT_SOFTWARE_STATUS = 9001
    TELEMETRY_ID_PLATFORM_FLIGHT_SOFTWARE_STATUS = 9002
    intball_app_config = None

    def __init__(self):
        rospy.init_node('sim_minimal_telemetry_publisher')

        self.__transcommunication_config = rospy.get_param('~transcommunication_config')
        self.__ocs_host = rospy.get_param('~ocs_host')
        self.__ocs_port = rospy.get_param('~ocs_port')

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

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', SimMinimalTelemetryPublisher.SENDING_SOURCE_PORT))

    def run(self):

        while not rospy.is_shutdown():
            try:
                data_dict = {}

                self.telemetry_header_builder.write_header(0xFFFF, 1, 1, data_dict, 0xFF,
                                                           ros_timestamp=rospy.Time.now())

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
                # Check that the process of roslaunch is up and running
                cmd_return = subprocess.run(('ps aux | '
                                             'grep -v grep | '
                                             'grep \'platform_manager.py __name:=platform_manager\' | '
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
                self.socket.sendto(zero_padded_pickled_telemetry, (self.__ocs_host, self.__ocs_port))
                logger.info('[send_telemetry] finish to send. port {}, len {}, byte_size {}'.format(
                            self.socket, len(zero_padded_pickled_telemetry),
                            sys.getsizeof(zero_padded_pickled_telemetry)))

                time.sleep(SimMinimalTelemetryPublisher.SENDING_SLEEP_TIME_SEC)
            except Exception as e:
                logger.error('[send_telemetry]fail to send. error {}'.format(e))
                time.sleep(SimMinimalTelemetryPublisher.SENDING_SLEEP_TIME_SEC)


if __name__ == '__main__':
    publisher = SimMinimalTelemetryPublisher()
    publisher.run()
