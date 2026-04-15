#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

exec ros2 launch gazebo_ros gazebo.launch.py \
  world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world \
  gui:=false \
  verbose:=true
