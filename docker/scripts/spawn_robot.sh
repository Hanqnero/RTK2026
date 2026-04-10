#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

echo "Waiting for /spawn_entity service..."
until ros2 service list 2>/dev/null | grep -q "^/spawn_entity$"; do
  sleep 2
done
echo "/spawn_entity is up, spawning rtk2026..."

# генерируем URDF из xacro
xacro /workspace/urdf/rtk2026_gazebo.urdf.xacro > /tmp/rtk2026.urdf

ros2 run gazebo_ros spawn_entity.py \
  -entity rtk2026 \
  -file /tmp/rtk2026.urdf \
  -x -2.0 -y -0.5 -z 0.03 \
  -Y 0.0
