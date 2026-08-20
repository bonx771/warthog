#!/bin/bash

set -e #exit on first error

export WARTHOG_TOP_PLATE_ENABLED="false"
export WARTHOG_IMU_XYZ="-0.122 0.0 0.7"
export WARTHOG_IMU_RPY="0.0 0.0  0.0"

roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_continuous.launch
