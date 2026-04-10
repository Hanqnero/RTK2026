#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

exec ros2 launch /scripts/robot.launch.py
