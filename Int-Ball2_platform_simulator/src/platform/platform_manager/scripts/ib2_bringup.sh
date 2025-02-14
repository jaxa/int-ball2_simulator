#!/bin/bash

export ROS_IP=`ip a show docker0 | grep 'inet ' | cut -d ' ' -f 6 | cut -d '/' -f 1`
export ROS_MASTER_URI="http://${ROS_IP}:11311"
rosrun platform_manager clean_up.py
roslaunch platform_manager ib2_bringup.launch --pid=/var/log/flight_software/flight_software.pid
rosrun platform_manager clean_up.py
