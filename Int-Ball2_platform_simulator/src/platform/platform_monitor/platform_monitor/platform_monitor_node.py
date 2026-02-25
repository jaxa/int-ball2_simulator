#!/usr/bin/python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# (see original license header in ROS 1 version)

from platform_msgs.msg import ContainerStatus, MonitorStatus, NodeStatusValue
import docker
import rclpy
from rclpy.node import Node
import yaml


class PlatformMonitor(Node):
    """Platform status monitor."""

    def __init__(self):
        super().__init__('platform_monitor')

        self._docker_client = docker.from_env()
        self._status_pub = self.create_publisher(MonitorStatus, '~/status', 10)
        self._monitor_status = MonitorStatus()

        self.declare_parameter('rate', 1.0)
        self.declare_parameter('config_path', '')

        rate = self.get_parameter('rate').value
        self._config_path = self.get_parameter('config_path').value

        if not self._config_path:
            self.get_logger().error('config_path parameter is not set.')
            raise RuntimeError('config_path parameter is not set.')

        try:
            with open(self._config_path, 'r') as yaml_file:
                config = yaml.load(yaml_file, Loader=yaml.FullLoader)

                def set_node_prefix(node):
                    return node if node.startswith('/') else '/' + node
                self._ignore_nodes = set(
                    [set_node_prefix(node) for node in config['ignore_nodes']])
                self.get_logger().info(f'Ignore nodes: {self._ignore_nodes}')

                self._target_machine = config.get('target_machine', None)
                self.get_logger().info(f'Target machine: {self._target_machine}')
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().error(
                f'platform_monitor node will stop. Failed to read config: "{self._config_path}".')
            raise e

        self._timer = self.create_timer(1.0 / rate, self._timer_callback)

    def _timer_callback(self):
        self._set_monitor_status()
        self._status_pub.publish(self._monitor_status)

    def _get_system_state(self):
        """Get system state similar to rosgraph.Master.getSystemState().

        Returns (publications, subscriptions, services) where each is a list of
        (name, [node_names]).
        """
        node_names_and_ns = self.get_node_names_and_namespaces()

        pub_dict = {}
        sub_dict = {}
        srv_dict = {}

        for node_name, namespace in node_names_and_ns:
            if namespace == '/':
                full_name = '/' + node_name
            else:
                full_name = namespace.rstrip('/') + '/' + node_name

            try:
                pubs = self.get_publisher_names_and_types_by_node(node_name, namespace)
                for topic, _ in pubs:
                    pub_dict.setdefault(topic, []).append(full_name)

                subs = self.get_subscriber_names_and_types_by_node(node_name, namespace)
                for topic, _ in subs:
                    sub_dict.setdefault(topic, []).append(full_name)

                srvs = self.get_service_names_and_types_by_node(node_name, namespace)
                for service, _ in srvs:
                    srv_dict.setdefault(service, []).append(full_name)
            except Exception:
                continue

        publications = list(pub_dict.items())
        subscriptions = list(sub_dict.items())
        services = list(srv_dict.items())

        return (publications, subscriptions, services)

    def _get_node_status_values(self, state):
        node_status_values = []
        for msg_name, node_list in state:
            target_nodes = set(node_list) - self._ignore_nodes
            node_status_values.extend(
                [NodeStatusValue(node=node, value=msg_name)
                 for node in target_nodes])

        return sorted(node_status_values,
                       key=lambda ns: (ns.node, ns.value))

    def _set_monitor_status(self):
        state = self._get_system_state()

        self._monitor_status = MonitorStatus()
        self._monitor_status.check_time = self.get_clock().now().to_msg()

        # Publications
        self._monitor_status.publications = self._get_node_status_values(state[0])

        # Subscriptions
        self._monitor_status.subscriptions = self._get_node_status_values(state[1])

        # Services
        self._monitor_status.services = self._get_node_status_values(state[2])

        # Running containers
        try:
            running_containers = self._docker_client.containers.list(
                filters={'status': 'running'})
            self._monitor_status.containers = [
                ContainerStatus(
                    image=' '.join(c.image.tags),
                    id=c.short_id,
                    status=ContainerStatus.RUNNING)
                for c in running_containers
            ]
        except Exception as e:
            self.get_logger().warn(f'Failed to get Docker containers: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PlatformMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
