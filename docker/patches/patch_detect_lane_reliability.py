#!/usr/bin/env python3
# Заменяет row-based reliability metric на fraction-based в detect_lane.
# Оригинальный алгоритм: reliability растёт только если линия присутствует
# в >=500 строках из 600 (how_much_short <= 100). Для 1/10 робота на повороте
# жёлтая линия занимает ~150-280 строк (short=320-450), поэтому reliability
# неизбежно падает до 0 и зелёная зона не появляется.
# Новый алгоритм: reliability растёт если fraction_num > 3000 (пикселей линии),
# что соответствует реальному наличию линии в кадре.

import re
import sys

path = "/workspace/src/turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_lane.py"
if len(sys.argv) > 1:
    path = sys.argv[1]

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

old_white = (
    "        how_much_short = 0\n\n"
    "        for i in range(0, 600):\n"
    "            if np.count_nonzero(mask[i, ::]) > 0:\n"
    "                how_much_short += 1\n\n"
    "        how_much_short = 600 - how_much_short\n\n"
    "        if how_much_short > 100:\n"
    "            if self.reliability_white_line >= 5:\n"
    "                self.reliability_white_line -= 5\n"
    "        elif how_much_short <= 100:\n"
    "            if self.reliability_white_line <= 99:\n"
    "                self.reliability_white_line += 5"
)

new_white = (
    "        if fraction_num > 3000:\n"
    "            if self.reliability_white_line <= 99:\n"
    "                self.reliability_white_line += 5\n"
    "        else:\n"
    "            if self.reliability_white_line >= 5:\n"
    "                self.reliability_white_line -= 5"
)

old_yellow = (
    "        how_much_short = 0\n\n"
    "        for i in range(0, 600):\n"
    "            if np.count_nonzero(mask[i, ::]) > 0:\n"
    "                how_much_short += 1\n\n"
    "        how_much_short = 600 - how_much_short\n\n"
    "        if how_much_short > 100:\n"
    "            if self.reliability_yellow_line >= 5:\n"
    "                self.reliability_yellow_line -= 5\n"
    "        elif how_much_short <= 100:\n"
    "            if self.reliability_yellow_line <= 99:\n"
    "                self.reliability_yellow_line += 5"
)

new_yellow = (
    "        if fraction_num > 3000:\n"
    "            if self.reliability_yellow_line <= 99:\n"
    "                self.reliability_yellow_line += 5\n"
    "        else:\n"
    "            if self.reliability_yellow_line >= 5:\n"
    "                self.reliability_yellow_line -= 5"
)

content_new = content.replace(old_white, new_white, 1)
if content_new == content:
    sys.exit("patch_detect_lane_reliability: white block not found")

content_new2 = content_new.replace(old_yellow, new_yellow, 1)
if content_new2 == content_new:
    sys.exit("patch_detect_lane_reliability: yellow block not found")

with open(path, "w", encoding="utf-8") as f:
    f.write(content_new2)
print("Patched:", path)
