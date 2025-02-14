#!/bin/bash

export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=${HOME}/catkin_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
source /opt/ros/melodic/setup.bash
source ${HOME}/catkin_ws/devel/setup.bash
ps -ef | grep python3 | grep communication_software | grep -v grep && PARAM="gui_only:=true"
roslaunch debug_gui bringup.launch ${PARAM}

