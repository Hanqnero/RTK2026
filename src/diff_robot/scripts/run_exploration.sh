#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
exec ros2 launch diff_robot exploration.launch.py use_sim_time:=True "$@"
