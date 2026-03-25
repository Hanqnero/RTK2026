#!/usr/bin/env python3
# Патч image_projection.py: масштабирует опорные точки гомографии
# пропорционально реальному разрешению (базовое: 320x240).
# Расширяет диапазоны параметров для удобной калибровки на 640x480+.

import re
import sys

path = "/workspace/src/turtlebot3_autorace/turtlebot3_autorace_camera/turtlebot3_autorace_camera/image_projection.py"
if len(sys.argv) > 1:
    path = sys.argv[1]

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Расширить integer_range (top: 0-120 -> 0-500; bottom: 0-320 -> 0-800)
content = content.replace(
    "from_value=0,\n                    to_value=120,",
    "from_value=0,\n                    to_value=500,",
)
content = content.replace(
    "from_value=0,\n                    to_value=320,",
    "from_value=0,\n                    to_value=800,",
)

# 2. Заменить блок "setting homography variables" на масштабируемый
old_homography = """\
        # setting homography variables
        top_x = self.top_x
        top_y = self.top_y
        bottom_x = self.bottom_x
        bottom_y = self.bottom_y"""

new_homography = """\
        # setting homography variables (auto-scaled from 320x240 base)
        h_img, w_img = cv_image_original.shape[:2]
        sx = w_img / 320.0
        sy = h_img / 240.0
        cx = w_img // 2
        cy_top = int(180 * sy)
        cy_bot = int(120 * sy)
        top_x = int(self.top_x * sx)
        top_y = int(self.top_y * sy)
        bottom_x = int(self.bottom_x * sx)
        bottom_y = int(self.bottom_y * sy)"""

if old_homography not in content:
    sys.exit("patch_image_projection: 'setting homography variables' block not found")
content = content.replace(old_homography, new_homography)

# 3. Заменить калибровочные линии: 160 -> cx, 180 -> cy_top, 120 -> cy_bot
content = content.replace(
    "(160 - top_x, 180 - top_y)",
    "(cx - top_x, cy_top - top_y)",
)
content = content.replace(
    "(160 + top_x, 180 - top_y)",
    "(cx + top_x, cy_top - top_y)",
)
content = content.replace(
    "(160 + bottom_x, 120 + bottom_y)",
    "(cx + bottom_x, cy_bot + bottom_y)",
)
content = content.replace(
    "(160 - bottom_x, 120 + bottom_y)",
    "(cx - bottom_x, cy_bot + bottom_y)",
)

# 4. pts_src
pts_src_old = """\
        pts_src = np.array([
            [160 - top_x, 180 - top_y],
            [160 + top_x, 180 - top_y],
            [160 + bottom_x, 120 + bottom_y],
            [160 - bottom_x, 120 + bottom_y]
        ])"""

pts_src_new = """\
        pts_src = np.array([
            [cx - top_x, cy_top - top_y],
            [cx + top_x, cy_top - top_y],
            [cx + bottom_x, cy_bot + bottom_y],
            [cx - bottom_x, cy_bot + bottom_y]
        ])"""

if pts_src_old not in content:
    sys.exit("patch_image_projection: pts_src block not found (already patched?)")
content = content.replace(pts_src_old, pts_src_new)

# 5. pts_dst и warpPerspective
pts_dst_old = "pts_dst = np.array([[200, 0], [800, 0], [800, 600], [200, 600]])"
pts_dst_new = """\
out_w, out_h = 1000, 600
        margin = int(out_w * 0.2)
        pts_dst = np.array([[margin, 0], [out_w - margin, 0], [out_w - margin, out_h], [margin, out_h]])"""

if pts_dst_old not in content:
    sys.exit("patch_image_projection: pts_dst block not found")
content = content.replace(pts_dst_old, pts_dst_new)

content = content.replace(
    "cv2.warpPerspective(cv_image_original, h, (1000, 600))",
    "cv2.warpPerspective(cv_image_original, h, (out_w, out_h))",
)
content = content.replace(
    "triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)",
    "triangle1 = np.array([[0, out_h - 1], [0, int(out_h * 0.57)], [margin, out_h - 1]], np.int32)",
)
content = content.replace(
    "triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)",
    "triangle2 = np.array([[out_w - 1, out_h - 1], [out_w - 1, int(out_h * 0.57)], [out_w - margin - 1, out_h - 1]], np.int32)",
)

# 6. Защита от None-гомографии (вырожденная трапеция при экстремальных параметрах)
content = content.replace(
    "        # homography process\n        cv_image_homography = cv2.warpPerspective",
    "        # homography process\n        if h is None:\n            self.get_logger().warn('Homography is None (degenerate trapezoid), skipping frame')\n            return\n        cv_image_homography = cv2.warpPerspective",
)

with open(path, "w", encoding="utf-8") as f:
    f.write(content)
print("Patched:", path)
