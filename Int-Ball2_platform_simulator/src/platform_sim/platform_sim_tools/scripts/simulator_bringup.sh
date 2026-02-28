#!/bin/bash

# Source the ROS 2 workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Attempt to source the workspace setup; adjust the path if your install space differs
if [ -f "${SCRIPT_DIR}/../../../../install/setup.bash" ]; then
    source "${SCRIPT_DIR}/../../../../install/setup.bash"
fi

export ROS_IP=$(ip a show docker0 | grep 'inet ' | cut -d ' ' -f 6 | cut -d '/' -f 1)

# Run clean_up before and after the simulation
ros2 run platform_manager clean_up.py
ros2 launch platform_sim_tools simulator_bringup_with_flight_software.launch.py
ros2 run platform_manager clean_up.py
