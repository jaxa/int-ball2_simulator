#!/usr/bin/python3

# https://github.com/ros/ros_comm/blob/melodic-devel/tools/rosnode/src/rosnode/__init__.py
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from platform_msgs.msg import ContainerStatus, MonitorStatus, NodeStatusValue
import docker
import rosgraph
import rospy
import socket
import urllib.parse as urlparse
import yaml


class PlatformMonitor(object):
    """Platform status monitor."""
    ROS_ID = "platform_monitor"

    def __init__(self):
        rospy.logdebug('PlatformMonitor.__init__ in')
        rospy.init_node(PlatformMonitor.ROS_ID)

        self._docker_client = docker.from_env()
        self._status_pub = rospy.Publisher('~status', MonitorStatus, queue_size=10)
        self._master = rosgraph.Master(PlatformMonitor.ROS_ID)
        self._monitor_status = MonitorStatus()

        try:
            self._rate = rospy.Rate(rospy.get_param('~rate'))
            self._config_path = rospy.get_param('~config_path')
        except KeyError as e:
            rospy.logerr('{} node will stop. because of the lack of rosparams "{}".'
                         .format(PlatformMonitor.ROS_ID, e))
            rospy.logdebug('PlatformMonitor.__init__ out')
            raise e

        try:
            with open(self._config_path, 'r') as yaml_file:
                config = yaml.load(yaml_file, Loader=yaml.FullLoader)

                def set_node_prefix(node): return node if node.startswith('/') else '/' + node
                self._ignore_nodes = set([set_node_prefix(node) for node in config['ignore_nodes']])
                rospy.loginfo('Ignore nodes: {}'.format(self._ignore_nodes))

                self._target_machine = config['target_machine']
                rospy.loginfo('Target machine: {}'.format(self._target_machine))
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr('{} node will stop. because of the failure to read the whitelite file: "{}".'
                         .format(PlatformMonitor.ROS_ID, self._config_path))
            rospy.logdebug('PlatformMonitor.__init__ out')
            raise e

        rospy.logdebug('PlatformMonitor.__init__ out')

    def _get_node_status_values(self, state):
        rospy.logdebug('PlatformMonitor._get_node_status_values in')

        node_status_values = []
        for msg_name, node_list in state:
            target_nodes = set(node_list) - self._ignore_nodes
            if self._target_machine:
                target_nodes = self._get_nodes_by_machine(target_nodes)
            node_status_values.extend([NodeStatusValue(node=node, value=msg_name) for node in target_nodes])

        rospy.logdebug('PlatformMonitor._get_node_status_values out')
        return sorted(node_status_values, key=lambda node_status: (node_status.node, node_status.value))

    def _get_nodes_by_machine(self, node_names):
        rospy.logdebug('PlatformMonitor.get_nodes_by_machine in')
        try:
            machine_actual = [host[4][0] for host in socket.getaddrinfo(self._target_machine, 0, 0, 0, socket.SOL_TCP)]
        except Exception as e:
            rospy.logerr('{} node will stop. because of the lack of rosparams "{}".'
                         .format(PlatformMonitor.ROS_ID, e))
            rospy.logdebug('PlatformMonitor.get_nodes_by_machine out')
            raise e

        # get all the node names, lookup their uris, parse the hostname
        # from the uris, and then compare the resolved hostname against
        # the requested machine name.
        matches = [self._target_machine] + machine_actual
        not_matches = []  # cache lookups
        retval = []
        for n in node_names:
            try:
                uri = self._master.lookupNode(n)
            except rosgraph.MasterError:
                # it's possible that the state changes as we are doing lookups. this is a soft-fail
                continue

            h = urlparse.urlparse(uri).hostname
            if h in matches:
                retval.append(n)
            elif h in not_matches:
                continue
            else:
                r = [host[4][0] for host in socket.getaddrinfo(h, 0, 0, 0, socket.SOL_TCP)]
                if set(r) & set(machine_actual):
                    matches.append(r)
                    retval.append(n)
                else:
                    not_matches.append(r)

        rospy.logdebug('PlatformMonitor.get_nodes_by_machine out')
        return retval

    def _set_monitor_status(self):
        rospy.logdebug('PlatformMonitor._set_monitor_status in')

        state = self._master.getSystemState()

        self._monitor_status.check_time = rospy.Time.now()

        # Publications
        self._monitor_status.publications = self._get_node_status_values(state[0])

        # Subscriptions
        self._monitor_status.subscriptions = self._get_node_status_values(state[1])

        # Services
        self._monitor_status.services = self._get_node_status_values(state[2])

        # Running containers
        running_containers = self._docker_client.containers.list(filters={'status': 'running'})
        self._monitor_status.containers = [
            ContainerStatus(image=' '.join(c.image.tags),
                            id=c.short_id,
                            status=ContainerStatus.RUNNING)
            for c in running_containers
        ]

        rospy.logdebug('PlatformMonitor._set_monitor_status out')

    def run(self):
        rospy.logdebug('PlatformMonitor.run in')
        while not rospy.is_shutdown():
            self._set_monitor_status()
            self._status_pub.publish(self._monitor_status)
            self._rate.sleep()
        rospy.logdebug('PlatformMonitor.run out')


if __name__ == '__main__':
    node = PlatformMonitor()
    node.run()
