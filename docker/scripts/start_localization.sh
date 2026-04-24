#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /workspace/install/setup.bash

GRAPH_FILEPATH="${GRAPH_FILEPATH:-/workspace/runtime_config/graph.geojson}"
INITIAL_VERTEX_ID="${INITIAL_VERTEX_ID:-5}"
INITIAL_YAW="${INITIAL_YAW:-${ROBOT_SPAWN_YAW:-3.141592653589793}}"
LOCALIZATION_START_DELAY_SEC="${LOCALIZATION_START_DELAY_SEC:-12}"
LOCALIZATION_USE_SIM_TIME="${LOCALIZATION_USE_SIM_TIME:-false}"

if [[ ! -f "${GRAPH_FILEPATH}" ]]; then
  echo "GRAPH_FILEPATH not found: ${GRAPH_FILEPATH}" >&2
  exit 1
fi

read -r INITIAL_X INITIAL_Y INITIAL_YAW < <(
  python3 - "${GRAPH_FILEPATH}" "${INITIAL_VERTEX_ID}" "${INITIAL_YAW}" <<'PY'
import json
import sys

graph_path = sys.argv[1]
vertex_id = int(sys.argv[2])
initial_yaw = float(sys.argv[3])

with open(graph_path, encoding="utf-8") as fh:
    graph = json.load(fh)

for feature in graph.get("features", []):
    if feature.get("geometry", {}).get("type") != "Point":
        continue
    properties = feature.get("properties", {})
    if int(properties.get("id", -1)) != vertex_id:
        continue
    coords = feature.get("geometry", {}).get("coordinates", [])
    if len(coords) < 2:
        raise SystemExit(f"vertex {vertex_id} in {graph_path} has invalid coordinates")
    initial_x = float(coords[0])
    initial_y = float(coords[1])
    print(f"{initial_x:.6f} {initial_y:.6f} {initial_yaw:.6f}")
    break
else:
    raise SystemExit(f"vertex {vertex_id} not found in {graph_path}")
PY
)

echo "Starting AMCL with graph-aligned initial pose:"
echo "  graph=${GRAPH_FILEPATH}"
echo "  initial_vertex_id=${INITIAL_VERTEX_ID}"
echo "  initial_pose=(${INITIAL_X}, ${INITIAL_Y}, yaw=${INITIAL_YAW})"

sleep "${LOCALIZATION_START_DELAY_SEC}"

ros2 run nav2_amcl amcl --ros-args \
  --params-file /workspace/amcl.yaml \
  -p use_sim_time:="${LOCALIZATION_USE_SIM_TIME}" \
  -p initial_pose.x:="${INITIAL_X}" \
  -p initial_pose.y:="${INITIAL_Y}" \
  -p initial_pose.yaw:="${INITIAL_YAW}" &

exec ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:="${LOCALIZATION_USE_SIM_TIME}" \
  -p autostart:=true \
  -p node_names:=[amcl]
