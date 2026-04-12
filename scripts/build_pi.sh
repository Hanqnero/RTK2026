#!/usr/bin/env bash
# Build arm64 Docker image for Raspberry Pi (Apple Silicon Mac)
# Кэш слоёв хранится локально → повторные сборки пропускают неизменённые слои.
# Usage: ./scripts/build_pi.sh
set -e
cd "$(dirname "$0")/.."

CACHE_DIR="$HOME/.cache/docker-buildx/rtk2026-pi"
mkdir -p "$CACHE_DIR"

# Создаём builder с поддержкой кэша (если ещё нет)
if ! docker buildx inspect rtk2026-builder &>/dev/null; then
  echo "==> Creating buildx builder 'rtk2026-builder'..."
  docker buildx create --name rtk2026-builder --driver docker-container --use
else
  docker buildx use rtk2026-builder
fi

echo "==> Building rtk2026:latest for linux/arm64 (with local cache)..."
docker buildx build \
  --platform linux/arm64 \
  -t rtk2026:latest \
  -f docker/pi/Dockerfile \
  --cache-from type=local,src="$CACHE_DIR" \
  --cache-to   type=local,dest="$CACHE_DIR",mode=max \
  --load \
  .

echo ""
echo "Build complete. Deploy to Pi with:"
echo "  ./scripts/deploy_pi.sh"
