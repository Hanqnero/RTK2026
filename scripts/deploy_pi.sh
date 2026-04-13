#!/usr/bin/env bash
# Transfer Docker image to Raspberry Pi and start the container
# Usage: ./scripts/deploy_pi.sh [pi@192.168.2.2]
set -e
PI="${1:-pi@192.168.2.2}"

echo "==> Transferring rtk2026:latest to $PI..."
echo "    (this takes ~3-5 min depending on connection speed)"
docker save rtk2026:latest | gzip | ssh "$PI" "gunzip | docker load"

echo ""
echo "==> Starting container on Pi..."
ssh "$PI" "
  docker rm -f rtk2026 2>/dev/null || true
  docker run -d --name rtk2026 \
    --privileged \
    -v /dev:/dev \
    --network host \
    rtk2026:latest
  echo 'Waiting for foxglove_bridge to start...'
  sleep 4
  docker logs rtk2026 2>&1 | grep -i foxglove | tail -3
"

echo ""
echo "==> Done. Connect Foxglove Studio to:"
echo "    ws://192.168.2.2:8765"
