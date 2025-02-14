import io
import rospy
from std_msgs.msg import Time, UInt16, UInt8


class TelemetryHeaderBuilder(object):
    def __init__(self, config):
        # Telemetry settings (header)
        self.id_timestamp = int(config['telemetry']['header']['id_timestamp'])
        self.id_last_executed_command = int(config['telemetry']['header']['id_last_executed_command'])
        self.id_split_number = int(config['telemetry']['header']['id_split_number'])
        self.id_current_split_index = int(config['telemetry']['header']['id_current_split_index'])
        self.id_sending_port_index = int(config['telemetry']['header']['id_sending_port_index'])

    def write_header(self, last_executed_command, split_number, split_index, data_dict, socket_index, ros_timestamp):
        # Stamp telemetry itself
        self.set_serialized_ros_msg(Time, ros_timestamp, data_dict, self.id_timestamp)

        # ID of the last command received
        self.set_serialized_ros_msg(UInt16, last_executed_command,
                                    data_dict, self.id_last_executed_command)

        # Information about splitting
        self.set_serialized_ros_msg(UInt8, split_number, data_dict, self.id_split_number)
        self.set_serialized_ros_msg(UInt8, split_index, data_dict, self.id_current_split_index)

        # Information about sending port
        self.set_serialized_ros_msg(UInt8, socket_index, data_dict, self.id_sending_port_index)

    def set_serialized_ros_msg(self, ros_msg_type, value, output_dict, output_index):
        msg = ros_msg_type()
        msg.data = value
        temp_buf = io.BytesIO()
        msg.serialize(temp_buf)
        output_dict[output_index] = temp_buf.getvalue()
