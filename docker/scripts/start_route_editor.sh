#!/usr/bin/env bash
# noVNC + ros2 launch route_tool_with_costmap.
# Launch defaults живут в launch-файле; этот скрипт только форвардит явные overrides из env.
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /workspace/install/setup.bash

export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root-route}"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

NOVNC_PORT="${NOVNC_PORT:-6084}"
AUTO_LOAD_KEEPOUT="${AUTO_LOAD_KEEPOUT:-true}"

append_launch_arg() {
  local launch_name="$1"
  local env_name="$2"
  local value="${!env_name-}"
  if [[ -n "$value" ]]; then
    LAUNCH_ARGS+=("${launch_name}:=${value}")
  fi
}

if [[ -n "${MAP_YAML-}" && ! -f "${MAP_YAML}" ]]; then
  echo "MAP_YAML не найден: ${MAP_YAML}" >&2
  exit 1
fi

rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true

Xvfb :1 -screen 0 "${SCREEN_SIZE:-1600x900x24}" -ac +extension GLX +render -noreset &
sleep 1
fluxbox >/tmp/fluxbox_route_edit.log 2>&1 &
x11vnc -display :1 -forever -shared -nopw -listen 0.0.0.0 -rfbport 5900 >/tmp/x11vnc_route.log 2>&1 &
if [[ ! -f /usr/share/novnc/index.html ]]; then
  ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html
fi
websockify --web=/usr/share/novnc/ "${NOVNC_PORT}" localhost:5900 >/tmp/websockify_route.log 2>&1 &
sleep 2

LAUNCH_ARGS=()
append_launch_arg yaml_filename MAP_YAML
append_launch_arg use_sim_time USE_SIM_TIME
append_launch_arg graph_filepath GRAPH_FILEPATH
append_launch_arg costmap_params_file COSTMAP_PARAMS_FILE
append_launch_arg use_vector_server USE_VECTOR_SERVER
append_launch_arg lane_params_file LANE_PARAMS_FILE
append_launch_arg lane_manager_executable LANE_MANAGER_EXECUTABLE
append_launch_arg lane_pose_topic LANE_POSE_TOPIC
append_launch_arg lane_current_vertex LANE_CURRENT_VERTEX
append_launch_arg lane_previous_vertex LANE_PREVIOUS_VERTEX
append_launch_arg lane_detected_sign_target_vertex LANE_DETECTED_SIGN_TARGET_VERTEX
append_launch_arg lane_direction_mode LANE_DIRECTION_MODE
append_launch_arg lane_tick_rate_hz LANE_TICK_RATE_HZ
append_launch_arg lane_log_every_n_ticks LANE_LOG_EVERY_N_TICKS
append_launch_arg publish_map_odom_static_tf PUBLISH_MAP_ODOM_STATIC_TF
append_launch_arg nav2_execution_params_file NAV2_EXECUTION_PARAMS_FILE
append_launch_arg start_rviz START_RVIZ
append_launch_arg enable_lane_manager ENABLE_LANE_MANAGER

ros2 launch rtk2026_route_nav route_tool_with_costmap.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

if [[ "${USE_VECTOR_SERVER-}" == "true" && "${AUTO_LOAD_KEEPOUT}" == "true" ]]; then
  sleep 8
  for json_file in /workspace/maps/keepout_parapets.json /workspace/maps/keepout_pedestals_0p8.json; do
    if [[ -f "${json_file}" ]]; then
      python3 /scripts/apply_keepout_json.py "${json_file}" || true
    fi
  done
fi

wait "${LAUNCH_PID}"
