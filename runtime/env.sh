#!/usr/bin/env bash

export CONTEST1_UAV_ROOT="$HOME/contest2_uav"

export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.102

unset ROS_PACKAGE_PATH
unset CMAKE_PREFIX_PATH

source /opt/ros/noetic/setup.bash
source "$CONTEST1_UAV_ROOT/runtime/sunray_ws/devel/setup.bash" --extend
source "$CONTEST1_UAV_ROOT/runtime/Fast-Drone-250/devel/setup.bash" --extend
source "$CONTEST1_UAV_ROOT/runtime/ego_uav_ws/devel/setup.bash" --extend
