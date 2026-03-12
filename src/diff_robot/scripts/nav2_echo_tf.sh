#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

ros2 run tf2_ros tf2_echo map base_footprint

