#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /home/cuong/warthog/devel/setup.bash

ROS_IP_ADDR="$(hostname -I | awk '{print $1}')"
if [ -n "$ROS_IP_ADDR" ]; then
  export ROS_IP="$ROS_IP_ADDR"
  export ROS_MASTER_URI="http://${ROS_IP_ADDR}:11311"
  unset ROS_HOSTNAME
fi

exec roslaunch /home/cuong/warthog/src/outdoor_waypoint_nav/launch/outdoor/start_on_boot.launch
