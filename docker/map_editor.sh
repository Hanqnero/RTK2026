#!/usr/bin/env bash
# Режим правки костмапы и запретных зон.
#
# Поднимает в контейнере rtk2026_sim: карту, костмапу, vector_object_server,
# keepout_click_tool и RViz. Робота и симуляции нет - это офлайн-режим над
# готовой картой.
#
#     docker/map_editor.sh                      # карта города по умолчанию
#     docker/map_editor.sh maps/my_map.yaml     # другая карта
#     docker/map_editor.sh --stop               # остановить
#
# Дальше: открыть http://127.0.0.1:6080, инструментом Publish Point
# наклацать вершины многоугольника и вызвать
#
#     docker/map_editor.sh --commit
#
# Зона сразу уходит в маску и записывается в <карта>.zones.json рядом с
# самой картой. Отдельного шага сохранения нет.

set -euo pipefail

CONTAINER=rtk2026_sim
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Порядок обязателен: backport с vector_object_server идёт до workspace.
SETUP='source /opt/ros/jazzy/setup.bash
       source /opt/vo_ws/install/setup.bash
       source /workspace/install/setup.bash'

in_container() { docker exec "$CONTAINER" bash -lc "$SETUP; $1"; }

case "${1:-}" in
  --stop)
      docker exec "$CONTAINER" bash -c '
        me=$$
        for p in $(pgrep -f "rviz2|vector_object_server|keepout_click|route_server|nav2_costmap|planner_server|controller_server|bt_navigator|lifecycle_manager|costmap_filter_info|map_server|static_transform_publisher|ros2 launch"); do
          [ "$p" = "$me" ] || kill -9 "$p" 2>/dev/null
        done' || true
      echo "режим остановлен"
      exit 0 ;;
  --commit)  in_container 'ros2 service call /keepout_click_tool/commit std_srvs/srv/Trigger'; exit 0 ;;
  --undo)    in_container 'ros2 service call /keepout_click_tool/undo std_srvs/srv/Trigger'; exit 0 ;;
  --clear)   in_container 'ros2 service call /keepout_click_tool/clear std_srvs/srv/Trigger'; exit 0 ;;
  --drop)    in_container 'ros2 service call /keepout_click_tool/drop_zones std_srvs/srv/Trigger'; exit 0 ;;
  --param)   shift; in_container "ros2 param set /costmap $*"; exit 0 ;;
esac

MAP_HOST="${1:-maps/polygon_5x5.yaml}"
MAP="/workspace/${MAP_HOST#*/}"
MAP="/workspace/$(realpath --relative-to="$ROOT" "$ROOT/$MAP_HOST" 2>/dev/null || echo "$MAP_HOST")"
ZONES="${MAP%.yaml}.zones.json"
GRAPH="${GRAPH:-/workspace/maps/graph}"

if ! docker ps --filter "name=$CONTAINER" --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "контейнер $CONTAINER не запущен:" >&2
    echo "  docker compose -f docker/docker-compose.sim.yml up -d sim" >&2
    exit 1
fi

# Один экземпляр: два keepout_click_tool ловили бы одни и те же клики.
"$0" --stop >/dev/null 2>&1 || true

docker exec "$CONTAINER" bash -lc "$SETUP
  export DISPLAY=:1
  nohup ros2 launch rtk2026_route_nav route_tool_with_costmap.launch.py \
    yaml_filename:=$MAP \
    graph_filepath:=$GRAPH \
    zones_path:=$ZONES \
    use_vector_server:=true \
    enable_lane_manager:=false \
    publish_map_odom_static_tf:=true \
    start_rviz:=true \
    use_sim_time:=false > /tmp/map_editor.log 2>&1 &"

printf 'карта  %s\nзоны   %s\nграф   %s\n\n' "$MAP" "$ZONES" "$GRAPH"
echo "жду готовности..."
for _ in $(seq 1 30); do
    if docker exec "$CONTAINER" grep -q "Keepout click tool ready" /tmp/map_editor.log 2>/dev/null; then
        echo "готово -> http://127.0.0.1:6080"
        exit 0
    fi
    sleep 1
done
echo "не поднялось за 30 с, смотрите: docker exec $CONTAINER tail -40 /tmp/map_editor.log" >&2
exit 1
