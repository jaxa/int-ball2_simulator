#!/usr/bin/python3
# -*- coding: utf-8 -*-
# license removed for brevity
import concurrent.futures
import pickle
import socket
import socketserver
import sys
import threading
import time
import yaml
from copy import deepcopy
from functools import partial
from importlib import import_module

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from ib2_msgs.msg import Mode
from builtin_interfaces.msg import Time

from trans_communication.telemetry_header_builder import TelemetryHeaderBuilder

# Reentrant lock
LOCK_TELEMETRY_DICT = threading.RLock()


def get_message_class(data_class_str):
    """Dynamically import a ROS 2 message class from a string like 'ib2_msgs/CtlStatus'.

    Equivalent to roslib.message.get_message_class in ROS 1.
    """
    parts = data_class_str.split('/')
    if len(parts) != 2:
        return None
    package_name = parts[0]
    class_name = parts[1]
    try:
        module = import_module('{}.msg'.format(package_name))
        return getattr(module, class_name)
    except (ImportError, AttributeError):
        return None


def get_service_class(data_class_str):
    """Dynamically import a ROS 2 service class from a string like 'ib2_msgs/UpdateParameter'.

    Equivalent to roslib.message.get_service_class in ROS 1.
    """
    parts = data_class_str.split('/')
    if len(parts) != 2:
        return None
    package_name = parts[0]
    class_name = parts[1]
    try:
        module = import_module('{}.srv'.format(package_name))
        return getattr(module, class_name)
    except (ImportError, AttributeError):
        return None


class TransCommunication(Node):
    # limit size of telemetry buffer
    MAX_BUFFER_SIZE = 1472

    # Maximum number of split for the telemetry_dict
    MAX_TELEMETRY_SPLIT_NUMBER = 255

    # Subscriber queue size for telemetry
    TELEMETRY_SUBSCRIBER_QUEUE_SIZE = 10

    def __init__(self):
        super().__init__('trans_communication')
        self.get_logger().debug('TransCommunication.__init__ in')

        try:
            self.declare_parameter('config_path', '')
            self.declare_parameter('ocs_host', 'localhost')
            self.declare_parameter('ocs_port', 34567)
            self.declare_parameter('receive_port', 23456)
            self.declare_parameter('lex_min_user_data_bytes', 68)
            self.declare_parameter('telemetry_rate', 1)
            self.declare_parameter('wait_after_send_error', 3)
            self.declare_parameter('ros_wait_for_service_time', 10)
            self.declare_parameter('telemetry_send_port', [49100, 49101])
            self.declare_parameter('telemetry_send_port_default_index', 0)
            self.declare_parameter('original_telemetry_message_size_threshold', 1300)

            self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
            self.ocs_host = self.get_parameter('ocs_host').get_parameter_value().string_value
            self.ocs_port = self.get_parameter('ocs_port').get_parameter_value().integer_value
            self.receive_port = self.get_parameter('receive_port').get_parameter_value().integer_value
            self.lex_min_user_data_bytes = self.get_parameter(
                'lex_min_user_data_bytes').get_parameter_value().integer_value
            self.telemetry_rate = self.get_parameter('telemetry_rate').get_parameter_value().integer_value
            self.wait_after_send_error = self.get_parameter(
                'wait_after_send_error').get_parameter_value().integer_value
            self.ros_wait_for_service_time = self.get_parameter(
                'ros_wait_for_service_time').get_parameter_value().integer_value
            self.telemetry_send_port = list(self.get_parameter(
                'telemetry_send_port').get_parameter_value().integer_array_value)
            self.telemetry_send_port_default_index = int(self.get_parameter(
                'telemetry_send_port_default_index').get_parameter_value().integer_value)
            self.original_telemetry_message_size_threshold = self.get_parameter(
                'original_telemetry_message_size_threshold').get_parameter_value().integer_value

            if not self.config_path:
                raise KeyError('config_path')

        except Exception as e:
            self.get_logger().error(
                'trans communication node will stop. '
                'because of the lack of parameters "{}".'.format(e))
            self.get_logger().debug('TransCommunication.__init__ out')
            raise e

        # Read setting YAML file
        try:
            with open(self.config_path, 'r') as yaml_file:
                self.get_logger().debug('config yaml path {}'.format(self.config_path))
                OcsCommandHandler.intball_app_config = yaml.load(
                    yaml_file, Loader=yaml.FullLoader)
                self.get_logger().debug('loaded yaml {}'.format(
                    OcsCommandHandler.intball_app_config))
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().error(
                'trans communication node will stop. '
                'because of the failure to read the config file: "{}".'.format(self.config_path))
            self.get_logger().debug('TransCommunication.receive out')
            raise e
        self.telemetry_header_builder = TelemetryHeaderBuilder(OcsCommandHandler.intball_app_config)

        # socket for telemetry transmission
        self.telemetry_sock = []
        # list that associates socket and telemetry ID
        self.telemetry_id_for_bind = {}
        for i, port in enumerate(self.telemetry_send_port):
            bind_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            bind_socket.bind(('', port))
            self.telemetry_sock.append(bind_socket)
            self.telemetry_id_for_bind[i] = []

        # subscriber for mode change
        self.mode_change_sub = self.create_subscription(
            Mode, '/task_manager/mode', self.change_mode, 10)

        # Store node reference in OcsCommandHandler so it can use logging and service calls
        OcsCommandHandler.node = self

        self.get_logger().debug('TransCommunication.__init__ out')

    def send_telemetry(self, ocs_host, ocs_port, socket_index):
        self.get_logger().debug('TransCommunication.send_telemetry in')
        # rate for a loop (Hz)
        rate = self.create_rate(self.telemetry_rate)

        while rclpy.ok():
            with LOCK_TELEMETRY_DICT:
                send_data_dict_list = self.split_telemetry_dict(OcsCommandHandler.telemetry_dict, socket_index)

            if len(send_data_dict_list) == 0:
                self.get_logger().warning(
                    'There is no data to send on port {}. Other nodes may not have started up.'.format(
                        self.telemetry_send_port[socket_index]))
                time.sleep(self.wait_after_send_error)
                continue

            data_index = 1
            if len(send_data_dict_list) <= TransCommunication.MAX_TELEMETRY_SPLIT_NUMBER:
                data_number = len(send_data_dict_list)
            else:
                data_number = TransCommunication.MAX_TELEMETRY_SPLIT_NUMBER

            for data_dict in send_data_dict_list:
                if data_index > TransCommunication.MAX_TELEMETRY_SPLIT_NUMBER:
                    self.get_logger().warning(
                        'The number of splits of the telemetry_dict '
                        'has reached the maximum value {}.'.format(
                            TransCommunication.MAX_TELEMETRY_SPLIT_NUMBER))
                    self.get_logger().warning(
                        'The remaining data of the telemetry_dict is not sent.')
                    break
                try:
                    # header
                    # In ROS 2, get current time as a builtin_interfaces/Time message
                    now_msg = self.get_clock().now().to_msg()
                    self.telemetry_header_builder.write_header(
                        OcsCommandHandler.last_executed_command,
                        data_number, data_index, data_dict, socket_index,
                        now_msg)

                    # pickle
                    pickled_telemetry = pickle.dumps(data_dict, protocol=3)
                    # zero padding
                    if sys.getsizeof(pickled_telemetry) < self.lex_min_user_data_bytes:
                        # pad zero so that getsizeof(pickled_telemetry) = self.lex_min_user_data_size
                        zero_padded_pickled_telemetry = pickled_telemetry + (self.lex_min_user_data_bytes -
                                                                             sys.getsizeof(pickled_telemetry)) * b'\0'
                        self.get_logger().debug(
                            "Zero padded, pickled_telemetry len {}, byte size {}".format(
                                len(zero_padded_pickled_telemetry),
                                sys.getsizeof(zero_padded_pickled_telemetry)))
                    else:
                        if sys.getsizeof(pickled_telemetry) % 4 != 0:
                            # pad zero so that getsizeof(pickled_telemetry) % 4 = 0
                            padding_length = 4 - \
                                (sys.getsizeof(pickled_telemetry) % 4)
                            zero_padded_pickled_telemetry = pickled_telemetry + padding_length * b'\0'
                            self.get_logger().debug(
                                "Zero padded, pickled_telemetry len {}, byte size {}".format(
                                    len(zero_padded_pickled_telemetry),
                                    sys.getsizeof(zero_padded_pickled_telemetry)))
                        else:
                            zero_padded_pickled_telemetry = pickled_telemetry

                    if len(zero_padded_pickled_telemetry) > TransCommunication.MAX_BUFFER_SIZE:
                        self.get_logger().error(
                            '[send_telemetry] telemetry size {} exceeds max size (={} bytes)'.format(
                                len(zero_padded_pickled_telemetry), TransCommunication.MAX_BUFFER_SIZE))
                        raise Exception("telemetry size error")
                    # Send telemetry
                    self.telemetry_sock[socket_index].sendto(
                        zero_padded_pickled_telemetry, (ocs_host, ocs_port))
                    self.get_logger().info(
                        '[send_telemetry] finish to send. port {}, len {}, byte_size {}'.format(
                            self.telemetry_send_port[socket_index],
                            len(zero_padded_pickled_telemetry),
                            sys.getsizeof(zero_padded_pickled_telemetry)))
                    rate.sleep()
                except Exception as e:
                    self.get_logger().warning(
                        '[send_telemetry]fail to send. error {}'.format(e))
                    self.get_logger().info(
                        'wait {} seconds'.format(self.wait_after_send_error))
                    time.sleep(self.wait_after_send_error)

                data_index = data_index + 1

        self.get_logger().debug('TransCommunication.send_telemetry out')

    def shutdown_handler(self):
        self.get_logger().debug('TransCommunication.shutdown_handler in')
        if hasattr(self, '_tcp_server'):
            self._tcp_server.shutdown()
        for s in self.telemetry_sock:
            s.close()
        self.get_logger().debug('TransCommunication.shutdown_handler out')

    def receive(self):
        self.get_logger().debug('TransCommunication.receive in')

        # Telecommand settings
        OcsCommandHandler.receives.clear()
        for config in OcsCommandHandler.intball_app_config['telecommand']:
            try:
                com_id = config['id']
                OcsCommandHandler.receives[com_id] = {}
                OcsCommandHandler.receives[com_id]['type'] = config['type']
                OcsCommandHandler.receives[com_id]['name'] = config['name']
                OcsCommandHandler.receives[com_id]['maintenance_mode'] = config['maintenance_mode']

                if config['type'] == 'topic':
                    # Publisher settings
                    msg_class = get_message_class(config['data_class'])
                    if msg_class is None:
                        raise ValueError('"data_class" of "{}"'.format(config['name']))
                    OcsCommandHandler.receives[com_id]['pub'] = self.create_publisher(
                        msg_class, config['name'], 100)
                    OcsCommandHandler.receives[com_id]['msg_class'] = msg_class

                elif config['type'] == 'service/request':
                    # Service settings
                    srv_class = get_service_class(config['data_class'])
                    if srv_class is None:
                        raise ValueError('"data_class" of "{}"'.format(config['name']))
                    OcsCommandHandler.receives[com_id]['srv_class'] = srv_class
                    OcsCommandHandler.receives[com_id]['request_class'] = srv_class.Request
                    OcsCommandHandler.receives[com_id]['response_class'] = srv_class.Response

                    # Create service client
                    client = self.create_client(srv_class, config['name'])
                    OcsCommandHandler.receives[com_id]['service_client'] = client

                    if config.get('skip_wait'):
                        self.get_logger().info(
                            'Skip waiting for the service to start: {}'.format(
                                OcsCommandHandler.receives[com_id]['name']))
                    else:
                        # Wait for service
                        if not client.wait_for_service(
                                timeout_sec=float(self.ros_wait_for_service_time)):
                            self.get_logger().warning(
                                'Service not available after timeout: {}'.format(
                                    OcsCommandHandler.receives[com_id]['name']))
                            continue

                else:
                    self.get_logger().error(
                        'The config "type" of "{}" in "receive" is invalid. '
                        'Delete this setting in this running.'.format(config['name']))
            except (KeyError, ValueError) as e:
                self.get_logger().error('The config is invalid.("{}")'.format(e))
                if 'id' in config:
                    self.get_logger().error(
                        'This element is removed from OcsCommandHandler.receives. (id: "{}")'.format(
                            config['id']))
                    OcsCommandHandler.receives.pop(config['id'])
                continue
            except Exception as e:
                # In case wait_for_service timeouts or other errors
                self.get_logger().warning(str(e))
                continue

        self.get_logger().debug('finish ocs receive setting')

        OcsCommandHandler.telemetry_dict.clear()
        OcsCommandHandler.telemetry_config.clear()

        # Telemetry settings (normal)
        for config in OcsCommandHandler.intball_app_config['telemetry']['normal']:
            try:
                is_add_timestamp = bool('add_timestamp' in config and config['add_timestamp'])

                if config['type'] == 'topic':
                    # If topic is sent to OCS as telemetry, registration of callback function is required.
                    msg_class = get_message_class(config['data_class'])
                    if msg_class is None:
                        raise ValueError('"data_class" of "{}"'.format(config['name']))
                    self.create_subscription(
                        msg_class, config['name'],
                        partial(OcsCommandHandler.update_telemetry_dict,
                                id=config['id'], add_timestamp=is_add_timestamp),
                        TransCommunication.TELEMETRY_SUBSCRIBER_QUEUE_SIZE)
                elif config['type'] == 'service/response':
                    # For service, do nothing here.
                    pass
                else:
                    self.get_logger().error(
                        'The config "type" of "{}" in "telemetry.normal" is invalid. '
                        'Delete this setting in this running.'.format(config['name']))
                    continue

                telemetry_id = config['id']
                OcsCommandHandler.telemetry_config[telemetry_id] = {}
                OcsCommandHandler.telemetry_config[telemetry_id]['name'] = config['name']
                OcsCommandHandler.telemetry_config[telemetry_id]['add_timestamp'] = is_add_timestamp

                if 'bind_port_index' in config:
                    # If the definition of bind_port_index exists,
                    # keep the link between the telemetry ID and the telemetry transmission port.
                    self.telemetry_id_for_bind[int(config['bind_port_index'])].append(config['id'])
                else:
                    # If the definition of bind_port_index does not exist,
                    # keep the link between the data ID and the default telemetry transmission port.
                    self.telemetry_id_for_bind[self.telemetry_send_port_default_index].append(config['id'])

            except (KeyError, ValueError) as e:
                self.get_logger().error('The config is invalid.("{}")'.format(e))
                if 'id' in config:
                    self.get_logger().error(
                        'This element is removed from OcsCommandHandler.'
                        'telemetry_config. (id: "{}")'.format(config['id']))
                    OcsCommandHandler.telemetry_config.pop(config['id'], None)

        # Telemetry settings (split)
        for config in OcsCommandHandler.intball_app_config['telemetry']['split']:
            try:
                telemetry_id = config['name']
                OcsCommandHandler.split_telemetry_config[telemetry_id] = []

                if config['type'] == 'topic':
                    # If topic is sent to OCS as telemetry, registration of callback function is required.
                    msg_class = get_message_class(config['data_class'])
                    if msg_class is None:
                        raise ValueError('"data_class" of "{}"'.format(config['name']))
                    self.create_subscription(
                        msg_class, config['name'],
                        partial(OcsCommandHandler.split_array_and_update_telemetry_dict,
                                topic_id=config['name']),
                        TransCommunication.TELEMETRY_SUBSCRIBER_QUEUE_SIZE)
                elif config['type'] == 'service/response':
                    # For service, do nothing here.
                    pass
                else:
                    self.get_logger().error(
                        'The config "type" of "{}" in "telemetry.split" is invalid. '
                        'Delete this setting in this running.'.format(config['name']))
                    continue

                # Non-split value
                if 'value_config' in config:
                    for value_config in config['value_config']:
                        handler_config = {}
                        handler_config['id'] = value_config['id']
                        handler_config['value'] = value_config['value']
                        handler_config['data_class'] = get_message_class(value_config['data_class'])
                        if handler_config['data_class'] is None:
                            raise ValueError('"data_class" of "{}"'.format(value_config['value']))
                        handler_config['add_timestamp'] = bool(
                            'add_timestamp' in value_config and value_config['add_timestamp'])
                        handler_config['data_class_attribute'] = value_config['data_class_attribute']
                        OcsCommandHandler.split_telemetry_config[telemetry_id].append(handler_config)
                        if 'bind_port_index' in value_config:
                            # If the definition of bind_port_index exists,
                            # keep the link between the telemetry ID and the telemetry transmission port.
                            self.telemetry_id_for_bind[
                                int(value_config['bind_port_index'])].append(handler_config['id'])
                        else:
                            # If the definition of bind_port_index does not exist,
                            # keep the link between the data ID and the default telemetry transmission port.
                            self.telemetry_id_for_bind[
                                self.telemetry_send_port_default_index].append(handler_config['id'])

                # The value to split (list, tuple, etc.)
                if 'split_config' in config:
                    for split_config in config['split_config']:
                        handler_config = {}
                        handler_config['value_to_split'] = split_config['value_to_split']
                        handler_config['split_value_key'] = split_config['split_value_key']
                        handler_config['assignable_id'] = list(range(
                            split_config['id_range'][0],
                            split_config['id_range'][1] + 1))
                        handler_config['msg_id_dict'] = {}
                        handler_config['add_timestamp'] = bool(
                            'add_timestamp' in split_config and split_config['add_timestamp'])
                        OcsCommandHandler.split_telemetry_config[telemetry_id].append(handler_config)

                        if 'bind_port_index' in split_config:
                            # If the definition of bind_port_index exists,
                            # keep the link between the telemetry ID and the telemetry transmission port.
                            self.telemetry_id_for_bind[
                                int(split_config['bind_port_index'])].extend(
                                    handler_config['assignable_id'])
                        else:
                            # If the definition of bind_port_index does not exist,
                            # keep the link between the data ID and the default telemetry transmission port.
                            self.telemetry_id_for_bind[
                                self.telemetry_send_port_default_index].extend(
                                    handler_config['assignable_id'])
            except (KeyError, ValueError) as e:
                self.get_logger().error('The config is invalid.("{}")'.format(e))
                if 'id' in config:
                    self.get_logger().error(
                        'This element is removed from OcsCommandHandler.'
                        'telemetry_config. (id: "{}")'.format(config['id']))
                    OcsCommandHandler.telemetry_config.pop(config['id'], None)
        self.get_logger().debug('telemetry_id_for_bind: {}'.format(self.telemetry_id_for_bind))

        if not rclpy.ok():
            # If ROS has already been shutdown, the server will not be started
            self.get_logger().debug('TransCommunication.receive out')
            return

        with concurrent.futures.ThreadPoolExecutor(
                max_workers=4 + len(self.telemetry_send_port) * 2) as executor:
            # Receiver from OCS

            # ThreadingTCPServer is not used to prevent parallel execution of commands.
            server = socketserver.TCPServer(
                ('0.0.0.0', self.receive_port), OcsCommandHandler)
            self._tcp_server = server
            executor.submit(server.serve_forever)

            # Telemetry-sender for OCS
            for i in range(len(self.telemetry_send_port)):
                executor.submit(self.send_telemetry, self.ocs_host, self.ocs_port, i)

            # Wait for completion (spin handles callbacks in the main thread)
            rclpy.spin(self)

        self.get_logger().debug('TransCommunication.receive out')

    def change_mode(self, msg):
        self.get_logger().debug('TransCommunication.change_mode in')
        OcsCommandHandler.is_maintenance = (msg.mode == Mode.MAINTENANCE)
        self.get_logger().debug('TransCommunication.change_mode out')

    def split_telemetry_dict(self, telemetry_dict, socket_index):
        self.get_logger().debug('TransCommunication.split_telemetry_dict in')
        result_dict_list = list()
        tmp_dict = dict()
        check_size = 0

        for telemetry_id in [k for k in telemetry_dict.keys()
                             if k in self.telemetry_id_for_bind[socket_index]]:
            check_size += sys.getsizeof(telemetry_dict[telemetry_id])
            if len(tmp_dict.keys()) > 0 and check_size >= self.original_telemetry_message_size_threshold:
                result_dict_list.append(tmp_dict)
                tmp_dict = dict()
                check_size = sys.getsizeof(telemetry_dict[telemetry_id])
            tmp_dict[telemetry_id] = telemetry_dict[telemetry_id]

        if len(tmp_dict.keys()) > 0:
            result_dict_list.append(tmp_dict)

        self.get_logger().debug('len(result_dict_list) {}'.format(len(result_dict_list)))
        self.get_logger().debug('TransCommunication.split_telemetry_dict out')
        return result_dict_list


class OcsCommandHandler(socketserver.BaseRequestHandler, object):
    intball_app_config = {}
    telemetry_config = {}
    split_telemetry_config = {}
    telemetry_dict = {}
    receives = {}
    receive_buffer = []
    msg_com_id = 0
    previous_sequence = 0
    previous_packet_index = 0
    is_maintenance = False
    last_executed_command = 0
    node = None  # Reference to the ROS 2 node for logging and service calls

    @staticmethod
    def _get_logger():
        """Get the ROS 2 logger from the node reference."""
        if OcsCommandHandler.node is not None:
            return OcsCommandHandler.node.get_logger()
        return rclpy.logging.get_logger('ocs_command_handler')

    @staticmethod
    def update_telemetry_dict(data, id=None, add_timestamp=False):
        logger = OcsCommandHandler._get_logger()
        logger.debug('OcsCommandHandler.update_telemetry_dict in')
        logger.debug('[update_telemetry_dict] id {}, data {}'.format(id, data))
        serialized_data = serialize_message(data)
        if add_timestamp:
            # Add a timestamp to the end of the data
            if OcsCommandHandler.node is not None:
                timestamp_msg = OcsCommandHandler.node.get_clock().now().to_msg()
            else:
                timestamp_msg = Time()
            serialized_timestamp = serialize_message(timestamp_msg)
            serialized_data = serialized_data + serialized_timestamp
        with LOCK_TELEMETRY_DICT:
            OcsCommandHandler.telemetry_dict[id] = serialized_data
            logger.debug('[update_telemetry_dict] dict {}'.format(OcsCommandHandler.telemetry_dict))
        logger.debug('OcsCommandHandler.update_telemetry_dict out')

    @staticmethod
    def split_array_and_update_telemetry_dict(data, topic_id=None):
        logger = OcsCommandHandler._get_logger()
        logger.debug('OcsCommandHandler.split_array_and_update_telemetry_dict in')
        with LOCK_TELEMETRY_DICT:
            try:
                for config in OcsCommandHandler.split_telemetry_config[topic_id]:
                    if 'split_value_key' not in config:
                        # If split_value_key is not defined, downlink the value as is.
                        value_msg = config['data_class']()
                        setattr(value_msg, config['data_class_attribute'], getattr(data, config['value']))

                        # update telemetry dict
                        OcsCommandHandler.update_telemetry_dict(
                            value_msg, config['id'], config['add_timestamp'])

                    elif config['split_value_key']:
                        # If split_value_key is a valid string, divide the values.
                        # The ID is associated with the value indicated by split_value_key.

                        # split array
                        for split_msg in getattr(data, config['value_to_split']):
                            msg_key = config['value_to_split'] + "_" + getattr(
                                split_msg, config['split_value_key'])
                            if msg_key not in config['msg_id_dict'].keys():
                                # assign a new ID
                                if len(config['assignable_id']) > 0:
                                    config['msg_id_dict'][msg_key] = config['assignable_id'].pop(0)
                                else:
                                    logger.warning(
                                        'An ID could not be assigned to the data: {}'.format(msg_key))
                                    logger.debug(
                                        'OcsCommandHandler.split_array_and_update_telemetry_dict out')
                                    return
                            # update telemetry dict
                            OcsCommandHandler.update_telemetry_dict(
                                split_msg, config['msg_id_dict'][msg_key],
                                config['add_timestamp'])
                    else:
                        # if split_value_key is an empty string, divide the values
                        # and reassign the ID every time.
                        assignable_id_list = deepcopy(config['assignable_id'])

                        # split array
                        for split_msg in getattr(data, config['value_to_split']):
                            # assign a new ID
                            if len(assignable_id_list) > 0:
                                msg_id = assignable_id_list.pop(0)
                            else:
                                logger.warning(
                                    'An ID could not be assigned to the data: {}'.format(topic_id))
                                logger.debug(
                                    'OcsCommandHandler.split_array_and_update_telemetry_dict out')
                                return
                            # update telemetry dict
                            OcsCommandHandler.update_telemetry_dict(
                                split_msg, msg_id, config['add_timestamp'])

                        # cleanup old data
                        OcsCommandHandler.telemetry_dict = {
                            new_key: OcsCommandHandler.telemetry_dict[new_key]
                            for new_key
                            in OcsCommandHandler.telemetry_dict.keys()
                            if new_key not in assignable_id_list}

            except Exception as e:
                logger.error('The telemetry data cannot be accepted.("{}")'.format(e))

        logger.debug('OcsCommandHandler.split_array_and_update_telemetry_dict out')

    def handle(self):
        logger = OcsCommandHandler._get_logger()
        logger.debug('OcsCommandHandler.handle in')
        logger.debug('Start OcsCommandHandler')
        sock = self.request
        data = sock.recv(102)
        # Size information of transferred data is set to first 2 byte. (POCS-specification)
        # Next 1 byte : Sequence No. [1, 2, ...]
        sequence = data[2]
        # Next 1 byte : Number of packet [1, 2, ...]
        number_of_packet = data[3]
        # Next 1 byte : Index of packet [1, 2, ..., number_of_packet]
        packet_index = data[4]
        if packet_index == 1:
            # When first packet, next 2 bytes: Message ID
            OcsCommandHandler.msg_com_id = int.from_bytes(data[5:7], 'little')
            logger.debug(
                'recv data sequence :{}, packet_index : {}/{}, msg id : {}, data : {}'
                .format(sequence, packet_index, number_of_packet, OcsCommandHandler.msg_com_id, data))
        else:
            logger.debug(
                'recv data sequence :{}, packet_index : {}/{},            , data : {}'
                .format(sequence, packet_index, number_of_packet, data))

        if number_of_packet > 1:
            if packet_index == 1:
                OcsCommandHandler.previous_sequence = sequence
                OcsCommandHandler.previous_packet_index = packet_index
            elif packet_index > 1:
                # check sequence No. is same in splited data
                if sequence != OcsCommandHandler.previous_sequence:
                    logger.error(
                        "Error, sequence is not same in splited data")
                    OcsCommandHandler.receive_buffer.clear()
                    OcsCommandHandler.msg_com_id = 0
                    OcsCommandHandler.previous_sequence = 0
                    OcsCommandHandler.previous_packet_index = 0
                    logger.debug('OcsCommandHandler.handle out')
                    return
                OcsCommandHandler.previous_sequence = sequence

                # check packet_index is sequential
                if packet_index != OcsCommandHandler.previous_packet_index + 1:
                    logger.error(
                        "Error, splited data is NOT resceived sequentially")
                    OcsCommandHandler.receive_buffer.clear()
                    OcsCommandHandler.msg_com_id = 0
                    OcsCommandHandler.previous_sequence = 0
                    OcsCommandHandler.previous_packet_index = 0
                    logger.debug('OcsCommandHandler.handle out')
                    return
                OcsCommandHandler.previous_packet_index = packet_index

            # store received data to buffer
            if packet_index == 1:
                OcsCommandHandler.receive_buffer.append(data[7:])
            else:
                OcsCommandHandler.receive_buffer.append(data[5:])

            if packet_index < number_of_packet:
                logger.info(
                    "Wait for next splited data: packet_index {}, number_of_packet {}"
                    .format(packet_index, number_of_packet))
                logger.debug('OcsCommandHandler.handle out')
                return

            # If all splitted data is received, merge data
            received_user_data = OcsCommandHandler.receive_buffer[0]
            for index in range(len(OcsCommandHandler.receive_buffer)):
                if index >= 1:
                    received_user_data += OcsCommandHandler.receive_buffer[index]
            OcsCommandHandler.receive_buffer.clear()
        else:
            received_user_data = data[7:]
            logger.debug("recv data is not splited")

        receive = OcsCommandHandler.receives[OcsCommandHandler.msg_com_id]

        # Check mode
        if receive['maintenance_mode'] and not OcsCommandHandler.is_maintenance:
            logger.info(
                'Maintenace-mode-only telecommand is filtered out : {}'.format(receive['name']))
        else:
            self.send_in_data(OcsCommandHandler.msg_com_id, received_user_data)

        logger.debug('OcsCommandHandler.handle out')

    def send_in_data(self, msg_id=None, data=None):
        logger = OcsCommandHandler._get_logger()
        logger.debug('OcsCommandHandler.send_in_data in')
        logger.debug('msg_id of send in data : {}'.format(msg_id))
        receive = OcsCommandHandler.receives[msg_id]

        if receive['type'] == 'topic':
            # Create message instance according to msg_class
            msg_class = receive['msg_class']
            # Convert telecommand into publishing message
            logger.debug("receive topic data: {}".format(data))
            msg = deserialize_message(data, msg_class)
            receive['pub'].publish(msg)
            logger.debug(
                "trans_communication publish message to {}, data:{}".format(receive['name'], msg))
            OcsCommandHandler.last_executed_command = msg_id

        elif receive['type'] == 'service/request':
            srv_name = receive["name"]
            service_client = receive["service_client"]
            # Create request instance for service according to srv_class
            request_class = receive['request_class']
            # Convert telecommand into request
            logger.debug("receive service data: {}".format(data))
            srv_request = deserialize_message(data, request_class)

            # Call service asynchronously and wait for result
            future = service_client.call_async(srv_request)
            # Wait for the service response in a thread-safe manner
            # Use a timeout to prevent indefinite blocking
            timeout_sec = 10.0
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    logger.error(
                        'Service call to {} timed out after {} seconds'.format(
                            srv_name, timeout_sec))
                    break
                time.sleep(0.01)

            if future.done():
                response = future.result()
                logger.debug(
                    "trans_communication send service request to {}".format(receive['name']))
                OcsCommandHandler.last_executed_command = msg_id

                # If response is sent to OCS, pack response data into telemetry data.
                check_sending_response = [
                    check_key for check_key in OcsCommandHandler.telemetry_config.keys()
                    if OcsCommandHandler.telemetry_config[check_key]['name'] == srv_name]
                if check_sending_response:
                    response_id = check_sending_response[0]
                    OcsCommandHandler.update_telemetry_dict(
                        response, response_id,
                        OcsCommandHandler.telemetry_config[response_id]['add_timestamp'])
                elif srv_name in OcsCommandHandler.split_telemetry_config:
                    OcsCommandHandler.split_array_and_update_telemetry_dict(response, srv_name)

        logger.debug('OcsCommandHandler.send_in_data out')


def main(args=None):
    rclpy.init(args=args)
    node = TransCommunication()
    try:
        node.receive()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_handler()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
