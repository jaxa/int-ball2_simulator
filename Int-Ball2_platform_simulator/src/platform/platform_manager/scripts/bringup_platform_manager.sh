#!/bin/bash

export ROS_IP=`ip a show docker0 | grep 'inet ' | cut -d ' ' -f 6 | cut -d '/' -f 1`
export ROS_MASTER_URI="http://${ROS_IP}:11311"
rosrun platform_manager clean_up.py
roslaunch platform_manager bringup.launch
rosrun platform_manager clean_up.py
