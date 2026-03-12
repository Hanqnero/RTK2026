#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

ros2 topic echo /cmd_vel

