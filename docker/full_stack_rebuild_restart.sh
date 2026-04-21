#!/usr/bin/env bash
# Полный цикл: down → build → up всего стека sim + route_editor_full → пауза на подъём Gazebo → снимок и опционально follow логов lane/Nav2.
# Использование (из каталога docker):
#   ./full_stack_rebuild_restart.sh
#   BUILD_SIM=1 ./full_stack_rebuild_restart.sh     # также пересобрать образ rtk2026-sim (gazebo/robot/...)
#   FOLLOW=1 ./full_stack_rebuild_restart.sh        # не выходить, слить логи route_editor (Ctrl+C)
#   TAIL_LINES=300 FOLLOW=0 ./full_stack_rebuild_restart.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

COMPOSE=(docker compose -f docker-compose.sim.yml -f docker-compose.route_editor.full.yml)
BUILD_SIM="${BUILD_SIM:-0}"
FOLLOW="${FOLLOW:-0}"
TAIL_LINES="${TAIL_LINES:-200}"
WAIT_SEC="${WAIT_SEC:-25}"

echo "[1/4] docker compose down --remove-orphans"
"${COMPOSE[@]}" down --remove-orphans

echo "[2/4] docker compose build"
if [[ "$BUILD_SIM" == "1" ]]; then
  echo "      (BUILD_SIM=1: сборка образов симуляции)"
  "${COMPOSE[@]}" build gazebo robot odometry localization
fi
"${COMPOSE[@]}" build route_editor_full

echo "[3/4] docker compose up -d"
"${COMPOSE[@]}" up -d

echo "[4/4] ожидание ${WAIT_SEC}s (Gazebo spawn, узлы Nav2)..."
sleep "$WAIT_SEC"

ROUTE_CID=$("${COMPOSE[@]}" ps -q route_editor_full 2>/dev/null || true)
if [[ -z "${ROUTE_CID:-}" ]]; then
  echo "Ошибка: контейнер route_editor_full не найден." >&2
  "${COMPOSE[@]}" ps -a
  exit 1
fi

FILTER_REGEX='lane_decision_manager|lane_state|target_pick|nav2_goal_publish|navigate_to_pose|navigate_to_pose succeeded|goal was rejected|not available yet|current_vertex|previous_vertex|result status|timeout|rearming|no_limiter|no_target|no_goal|Aborting handle'

echo ""
echo "========== Снимок логов route_editor (навигация по вершинам / цели), последние ~${TAIL_LINES} строк отфильтровано =========="
docker logs --tail "$TAIL_LINES" "$ROUTE_CID" 2>&1 | grep -E "$FILTER_REGEX" || true
echo "================================================================================================"

if [[ "$FOLLOW" == "1" ]]; then
  echo "FOLLOW=1: поток логов (тот же фильтр), Ctrl+C для выхода"
  docker logs -f "$ROUTE_CID" 2>&1 | grep -E --line-buffered "$FILTER_REGEX"
else
  echo "Подсказка: FOLLOW=1 $0 — смотреть поток в реальном времени."
fi
