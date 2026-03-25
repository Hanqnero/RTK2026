#!/usr/bin/env bash
set -e

ROOT="/Users/kamilishakov/CursorProjects/ROBO-RTK/RTK2026"
DESKTOP_DIR="$ROOT/ros-humble-desktop-m1_2-mac"
USERNAME="kamilishakov"
USERID="501"
GROUPID="20"
CONTAINER_NAME="rtk2026_desktop"

if ! docker info >/dev/null 2>&1; then
  echo "Ошибка: Docker не запущен или недоступен (unix://\$HOME/.docker/run/docker.sock)."
  echo "Запустите Docker Desktop и повторите попытку."
  exit 1
fi

echo "[1/3] Сборка образа ros-humble-desktop (amd64)..."
cd "$DESKTOP_DIR"
docker buildx build --platform linux/amd64 -t ros-humble-desktop:latest \
  --build-arg username="$USERNAME" \
  --build-arg userid="$USERID" \
  --build-arg groupid="$GROUPID" .

echo "[2/3] Запуск контейнера с VNC на порту 5900..."
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
# Запуск supervisord в фоне и sleep infinity, чтобы контейнер не завершался после падения entrypoint
docker run --platform linux/amd64 -d --name "$CONTAINER_NAME" \
  -v "$DESKTOP_DIR/supervisor-conf-d:/etc/supervisor/conf.d" \
  -v "$ROOT:/home/$USERNAME/RTK2026" \
  -p 5900:5900 \
  --entrypoint "" \
  ros-humble-desktop:latest \
  bash -c "sudo /usr/bin/supervisord & sleep infinity"

echo "Ожидание старта VNC (5 сек)..."
sleep 5

echo "[3/3] Сборка RTK2026 и запуск трассовой симуляции в контейнере..."
docker exec -it "$CONTAINER_NAME" bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  cd /home/kamilishakov/RTK2026
  echo "[3.1] colcon build..."
  colcon build --symlink-install --packages-up-to rtk2026_simulation diff_robot
  source install/setup.bash
  echo "[3.2] ros2 launch rtk2026_diff_robot_track..."
  exec ros2 launch rtk2026_simulation rtk2026_diff_robot_track.launch.py \
    use_sim_time:=true explore:=true use_rviz:=true use_gazebo_gui:=false
'
