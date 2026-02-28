#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Clean-up script for platform_manager.

In ROS 2 we cannot use the roslaunch API to introspect launch files and get
node names.  Instead, we read the config YAML to get the bringup package info
and kill processes whose command-line contains the known node names from those
packages.  We also stop and remove the ib2_user Docker container.
"""

import docker
import os
import psutil
import yaml

from ament_index_python.packages import get_package_share_directory


def main():
    # Stop and remove ib2_user container
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get('ib2_user')
        if container.status != 'exited':
            container.stop()
        container.remove()
    except Exception:
        pass

    # Kill the processes for nodes launched by flight_software.
    try:
        # Read the list of nodes to be launched from the configuration file.
        config_path = os.path.join(
            get_package_share_directory('platform_manager'), 'config', 'config.yml')
        with open(config_path) as file:
            config_yaml = yaml.safe_load(file)

        # In ROS 2, node processes are typically identified by their
        # executable name or by ros2-specific command line arguments.
        # We look for processes whose command line contains the node
        # name as part of a --ros-args or __node:= pattern, or simply
        # the package executable name.
        def check_cmdline(cmdline, nodes):
            """Check if process cmdline matches any of the target node names."""
            cmdline_str = ' '.join(cmdline)
            for node_name in nodes:
                # ROS 2 style: --ros-args ... __node:=name or -r __node:=name
                if '__node:={}'.format(node_name) in cmdline_str:
                    return True
                # ROS 1 style (in case of mixed environments)
                if '__name:={}'.format(node_name) in cmdline_str:
                    return True
                # Also match by executable name or 'ros2 launch package launch_file'
                if node_name in cmdline_str:
                    return True
            return False

        for package_config in config_yaml.get('bringup_packages', []):
            package = package_config.get('package', package_config['name'])
            name = package_config['name']

            # We cannot read launch files to get node names in ROS 2 easily,
            # so we use the package name and bringup name as targets
            target_nodes = [name, package]
            print('Target nodes for cleanup: {}'.format(target_nodes))

            target_processes = [p for p in psutil.process_iter(['pid', 'cmdline'])
                                if p.info['cmdline'] and
                                check_cmdline(p.info['cmdline'], target_nodes)]
            for process in target_processes:
                print('Kill process: {}'.format(process))
                try:
                    process.kill()
                except Exception as e:
                    print('Failed to kill process {}: {}'.format(process, e))
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
