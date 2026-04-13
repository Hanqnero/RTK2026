#!/usr/bin/env bash
set -e

ROOT="/Users/kamilishakov/CursorProjects/ROBO-RTK/RTK2026"
DESKTOP_DIR="$ROOT/ros-humble-desktop-m1_2-mac"
USERNAME="kamilishakov"
USERID="501"
GROUPID="20"
CONTAINER_NAME="rtk2026_rviz"
PI_IP="192.168.2.2"

if ! docker info >/dev/null 2>&1; then
  echo "Ошибка: Docker не запущен. Запустите Docker Desktop и повторите."
  exit 1
fi

echo "[1/3] Сборка образа ros-humble-desktop (если нужно)..."
cd "$DESKTOP_DIR"
docker buildx build --platform linux/amd64 -t ros-humble-desktop:latest \
  --build-arg username="$USERNAME" \
  --build-arg userid="$USERID" \
  --build-arg groupid="$GROUPID" .

echo "[2/3] Запуск VNC контейнера на порту 5900..."
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
docker run --platform linux/amd64 -d --name "$CONTAINER_NAME" \
  --network host \
  -v "$DESKTOP_DIR/supervisor-conf-d:/etc/supervisor/conf.d" \
  -v "$DESKTOP_DIR/cyclonedds_pi.xml:/tmp/cyclonedds_pi.xml:ro" \
  -v "$ROOT:/home/$USERNAME/RTK2026" \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/tmp/cyclonedds_pi.xml \
  --entrypoint "" \
  ros-humble-desktop:latest \
  bash -c "sudo /usr/bin/supervisord & sleep infinity"

echo "Ожидание старта VNC (5 сек)..."
sleep 5

echo "[3/3] Сборка RTK2026 и запуск RViz..."
docker exec "$CONTAINER_NAME" bash -c "
  set -e
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=/tmp/cyclonedds_pi.xml
  export ROS_DOMAIN_ID=0

  cd /home/$USERNAME/RTK2026
  echo '[3.1] Сборка пакетов описания...'
  colcon build --symlink-install --packages-up-to rtk2026_description rtk2026_interfaces
  source install/setup.bash

  echo '[3.2] Проверка топиков Pi...'
  timeout 5 ros2 topic list || echo 'Топики Pi не видны — проверьте сеть'

  echo '[3.3] Запуск RViz...'
  exec rviz2 -d /home/$USERNAME/RTK2026/src/rtk2026_description/rviz/rtk2026_sim_slam.rviz
"

echo ""
echo "Подключитесь через TigerVNC: localhost:5900"
open vnc://localhost:5900 2>/dev/null || echo "Откройте TigerVNC вручную: localhost:5900"
