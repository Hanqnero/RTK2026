#!/usr/bin/env bash
set -euo pipefail

# Пример добавления keepout-полигона через Vector Object Server.
# Запускать внутри контейнера route_editor (или через docker exec).

source /opt/ros/jazzy/setup.bash || true
source /workspace/install/setup.bash || true

echo "[1/3] Текущие фигуры:"
ros2 service call /vector_object_server/get_shapes nav2_msgs/srv/GetShapes "{}"

echo "[2/3] Добавляем keepout-полигон (пример зоны паребрика)..."
ros2 service call /vector_object_server/add_shapes nav2_msgs/srv/AddShapes \
  "polygons:
  - header:
      frame_id: map
    points:
    - {x: -1.00, y: 0.20}
    - {x: -1.00, y: 0.60}
    - {x:  1.00, y: 0.60}
    - {x:  1.00, y: 0.20}
    closed: true
    value: 100"

echo "[3/3] Фигуры после добавления:"
ros2 service call /vector_object_server/get_shapes nav2_msgs/srv/GetShapes "{}"
