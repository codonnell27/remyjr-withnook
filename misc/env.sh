#!/usr/bin/env bash

export ROS_IP=10.42.0.1
export ROS_HOSTNAME=10.42.0.27
export ROS_MASTER_URI=http://10.42.0.1:11311

source /home/pi/catkin_ws/devel/setup.bash
exec "$@"

