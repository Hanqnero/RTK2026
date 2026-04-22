#!/bin/bash
set -e
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

NOVNC_PORT="${NOVNC_PORT:-6082}"

Xvfb :1 -screen 0 "${SCREEN_SIZE:-1600x900x24}" -ac +extension GLX +render -noreset &
sleep 1
fluxbox >/tmp/fluxbox_rviz.log 2>&1 &
x11vnc -display :1 -forever -shared -nopw -listen 0.0.0.0 -rfbport 5900 >/tmp/x11vnc_rviz.log 2>&1 &
websockify --web=/usr/share/novnc/ "${NOVNC_PORT}" localhost:5900 >/tmp/websockify_rviz.log 2>&1 &
sleep 2

CFG="${RVIZ_CONFIG:-/workspace/map_edit.rviz}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
if [[ ! -f "$CFG" ]]; then
  echo "RVIZ_CONFIG не найден: $CFG, запуск без конфига" >&2
  exec rviz2 --ros-args -p use_sim_time:="${USE_SIM_TIME}"
fi
exec rviz2 -d "$CFG" --ros-args -p use_sim_time:="${USE_SIM_TIME}"
