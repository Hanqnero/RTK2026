#!/usr/bin/env bash
set -euo pipefail

# Создаёт отдельную keepout-маску из обычной карты.
# Пример:
#   ./docker/scripts/create_keepout_mask.sh \
#     ./docker/saved_maps/rtk2026_arena.yaml \
#     ./docker/saved_maps/rtk2026_keepout_mask.yaml

SRC_MAP_YAML="${1:-./docker/saved_maps/rtk2026_arena.yaml}"
DST_MASK_YAML="${2:-./docker/saved_maps/rtk2026_keepout_mask.yaml}"

python3 - "$SRC_MAP_YAML" "$DST_MASK_YAML" <<'PY'
import shutil
import sys
from pathlib import Path

src_yaml = Path(sys.argv[1]).resolve()
dst_yaml = Path(sys.argv[2]).resolve()

if not src_yaml.exists():
    raise SystemExit(f"Source map yaml not found: {src_yaml}")

lines = src_yaml.read_text(encoding="utf-8").splitlines()
image_line = next((ln for ln in lines if ln.strip().startswith("image:")), None)
if image_line is None:
    raise SystemExit(f"'image:' field not found in {src_yaml}")

src_image_raw = image_line.split(":", 1)[1].strip()
src_image_path = Path(src_image_raw)
if not src_image_path.is_absolute():
    src_image_path = (src_yaml.parent / src_image_path).resolve()

dst_yaml.parent.mkdir(parents=True, exist_ok=True)
dst_image_path = dst_yaml.with_suffix(".pgm")

shutil.copy2(src_image_path, dst_image_path)
new_lines = []
for ln in lines:
    if ln.strip().startswith("image:"):
        new_lines.append(f"image: {dst_image_path.name}")
    else:
        new_lines.append(ln)

dst_yaml.write_text("\n".join(new_lines) + "\n", encoding="utf-8")
print(f"Mask YAML: {dst_yaml}")
print(f"Mask PGM : {dst_image_path}")
PY
