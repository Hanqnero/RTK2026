#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

echo "=== ros2 node list ==="
ros2 node list
echo
echo "=== ros2 action list ==="
ros2 action list
echo
echo "=== key topics (map / scan / odom / cmd_vel) ==="
ros2 topic list | grep -E '(^/map$|^/scan$|^/odom$|^/cmd_vel$)' || true

