#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${PI_HOST:-pi@192.168.2.2}"
CONTAINER="${CONTAINER:-rtk2026}"
IPM_DATA="${IPM_DATA:-[0.17, 0.0, 0.45, 0.55, -0.03, 0.03]}"

echo "[1/4] Restarting container on ${PI_HOST}..."
ssh "${PI_HOST}" "docker restart ${CONTAINER} >/dev/null"

echo "[2/4] Starting full perception + lane + SIFT sign stack..."
ssh "${PI_HOST}" "docker exec -d ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_bringup pi_full.launch.py use_lane:=true lane_detector_mode:=centerline sign_detector_backend:=sift use_slam:=false use_localization:=false > /tmp/ros_launch.log 2>&1'"

echo "[3/4] Applying IPM calibration once: ${IPM_DATA}"
sleep 4
ssh "${PI_HOST}" "docker exec ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic pub --once /camera/ipm_tuning/set std_msgs/msg/Float32MultiArray \"{data: ${IPM_DATA}}\" >/dev/null 2>&1'"

echo "[4/4] Sanity checks:"
ssh "${PI_HOST}" "docker exec ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 node list | grep -E \"^/(detect_sign|image_relay_autorace|ipm_tuner|foxglove_bridge)$\" && ros2 param get /detect_sign dataset_root && timeout 4 ros2 topic echo /camera/ipm_tuning/current --once || true && grep -n \"SIFT sign detector\" /tmp/ros_launch.log | tail -n 1 || true'"

echo
echo "Foxglove: ws://192.168.2.2:8765"
echo "Note: if /camera/ipm_tuning/current rolls back to default, disable any Foxglove publisher on /camera/ipm_tuning/set."
