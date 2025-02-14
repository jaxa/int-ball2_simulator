#!/usr/bin/python3
# -*- coding:utf-8 -*-

import docker
import os
import psutil
import roslaunch
import yaml

# Stop and remove ib2_user container
try:
    docker_client = docker.from_env()
    container = docker_client.containers.get('ib2_user')
    if container.status != 'exited':
        container.stop()
    container.remove()
except Exception:
    pass

# Kill the process on the node launched by flight_software.
try:
    # Read the list of nodes to be launched
    # from the configuration file.
    config_path = os.path.join(os.path.dirname(__file__), '../config/config.yml')
    with open(config_path) as file:
        config_yaml = yaml.safe_load(file)

        def check_cmdline(cmdline, nodes): return bool(
                [c for c in cmdline
                 if c.startswith('__name:=') and
                 c.replace('__name:=', '') in nodes]
            )

        for package_config in config_yaml['bringup_packages']:
            package = package_config.get('package', package_config['name'])
            launch_file = package_config.get('launch_file', 'bringup.launch')

            # Read launch files and get the names of the nodes that can be launched
            launch_files = roslaunch.rlutil.resolve_launch_arguments([package, launch_file])
            roslaunch_config = roslaunch.config.load_config_default(launch_files, None)
            target_nodes = [n.name for n in roslaunch_config.nodes]
            print('Target nodes: {}'.format(target_nodes))

            target_processes = [p for p in psutil.process_iter()
                                if check_cmdline(p.cmdline(), target_nodes)]
            for process in target_processes:
                print('Kill process: {}'.format(process))
                process.kill()
except Exception as e:
    print(e)
