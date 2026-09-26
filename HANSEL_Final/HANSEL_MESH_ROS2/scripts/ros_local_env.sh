#!/bin/bash
# Source for Head-local sensor processes only. No bat0 DDS traffic.
_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$HOME/ros2_lyrical/install/local_setup.bash"
source "$_root/ros2_ws/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${HANSEL_LOCAL_DOMAIN:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$_root/ros2_ws/src/hansel_bringup/config/dds/local.xml"
export PYTHONPATH="$_root/vendor/python:$_root/ros2_ws/src/hansel_localization${PYTHONPATH:+:$PYTHONPATH}"
unset _root
