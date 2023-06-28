#!/bin/bash
source /opt/ros/noetic/setup.bash
source  /home/ubuntu/eimo_remote/devel/setup.sh

export ROS_MASTER_URI=http://192.168.31.16:11311
export ROS_IP=192.168.31.16
exec "$@"