#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash

# генерируем URDF из xacro
xacro /workspace/urdf/rtk2026_gazebo.urdf.xacro > /tmp/rtk2026.urdf

# По умолчанию держим старт в вершине 5, но разворачиваем робота на 180°
# в сторону вершины 4. Позу можно переопределить через env, чтобы Gazebo
# и локализация использовали один и тот же yaw.
SPAWN_X="${ROBOT_SPAWN_X:-3.45}"
SPAWN_Y="${ROBOT_SPAWN_Y:-0.55}"
SPAWN_Z="${ROBOT_SPAWN_Z:-0.03}"
SPAWN_YAW="${ROBOT_SPAWN_YAW:-3.141592653589793}"

echo "Spawning rtk2026 into Gazebo Sim (with retries)..."
echo "Spawn pose: x=${SPAWN_X} y=${SPAWN_Y} z=${SPAWN_Z} yaw=${SPAWN_YAW}"
for i in $(seq 1 30); do
  if ros2 run ros_gz_sim create \
    -name rtk2026 \
    -file /tmp/rtk2026.urdf \
    -x "${SPAWN_X}" -y "${SPAWN_Y}" -z "${SPAWN_Z}" \
    -Y "${SPAWN_YAW}"; then
    echo "Spawn succeeded."
    exit 0
  fi
  echo "Spawn attempt ${i}/30 failed, retrying..."
  sleep 2
done

echo "Spawn failed after retries."
exit 1
