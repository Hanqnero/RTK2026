#!/usr/bin/env bash
# Build and run RealSense calibration container on Mac (Apple Silicon).
#
# USB setup (one-time):
#   Docker Desktop → Settings → Features in Development → Enable USB sharing
#   Add your RealSense device (Intel Corp., usually 8086:xxxx)
#   Restart Docker Desktop
#
# Usage:
#   ./scripts/run_calibration_mac.sh            # intrinsic calibration
#   ./scripts/run_calibration_mac.sh extrinsic  # extrinsic (IPM) calibration
#
# Then open Foxglove Studio → Connect to ws://localhost:8765
#   Intrinsic:  import foxglove_layouts/calibration_intrinsic.json
#   Extrinsic:  import foxglove_layouts/calibration_extrinsic.json

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
MODE="${1:-intrinsic}"
IMAGE="rtk2026-calib:latest"

echo "=== Building $IMAGE ==="
docker buildx build \
  --platform linux/arm64 \
  -t "$IMAGE" \
  -f "$PROJECT_DIR/docker/mac/Dockerfile" \
  "$PROJECT_DIR"

echo "=== Starting calibration container (mode: $MODE) ==="
echo "    Foxglove: ws://localhost:8765"

if [ "$MODE" = "extrinsic" ]; then
  EXTRINSIC_ARG="true"
else
  EXTRINSIC_ARG="false"
fi

docker run --rm -it \
  --name rtk2026-calib \
  --privileged \
  -p 8765:8765 \
  "$IMAGE" \
  /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /workspace/install/setup.bash && \
    ros2 launch rtk2026_peripherals calibration.launch.py extrinsic:=$EXTRINSIC_ARG
  "
