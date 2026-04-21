#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash

: "${GAZEBO_WORLD_FILE:=/workspace/worlds/polygon_5x5.world}"

ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
BRIDGE_PID=$!

cleanup() {
  kill "${BRIDGE_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r -s -v 4 ${GAZEBO_WORLD_FILE}"
