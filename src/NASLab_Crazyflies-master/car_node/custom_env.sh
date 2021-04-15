#!/usr/bin/env bash

export ROS_HOSTNAME=192.168.1.145
export ROS_IP=192.168.1.145
export ROS_MASTER_URI=http://naslab-NUC8i7HVK:11311/
export ROSLAUNCH_SSH_UNKNOWN=1
source /home/pi/catkin_ws/devel/setup.bash
exec "$@"
