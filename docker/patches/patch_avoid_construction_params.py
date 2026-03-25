#!/usr/bin/env python3
# Патч avoid_construction.py: параметры danger_*, speed, габариты робота из ROS.

import re
import sys

path = "/workspace/src/turtlebot3_autorace/turtlebot3_autorace_mission/turtlebot3_autorace_mission/avoid_construction.py"
if len(sys.argv) > 1:
    path = sys.argv[1]

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

# Определяем отступ по строке с danger_distance (в репозитории 4 или 8 пробелов)
m = re.search(r'^(\s+)self\.danger_distance = 0\.24', content, re.MULTILINE)
if not m:
    sys.exit("patch_avoid_construction_params: self.danger_distance = 0.24 not found")
indent = m.group(1)

# Блок для замены: от self.bridge до self.last_turn_error (между значением и # может быть разный пробел)
old_block = re.escape(indent) + r"self\.bridge = CvBridge\(\)\n"
old_block += re.escape(indent) + r"self\.lane_detected = False\n\n"
old_block += re.escape(indent) + r"# Parameter settings\n"
old_block += re.escape(indent) + r"self\.danger_distance = 0\.24\s+# Danger zone y threshold \(meters\)\n"
old_block += re.escape(indent) + r"self\.danger_width = 0\.12\s+# Danger zone x width \(meters\)\n"
old_block += re.escape(indent) + r"self\.speed = 0\.03\s+# Forward speed during avoidance\n\n"
old_block += re.escape(indent) + r"# PD control parameters \(for turning\)\n"
old_block += re.escape(indent) + r"self\.turn_Kp = 0\.45\n"
old_block += re.escape(indent) + r"self\.turn_Kd = 0\.03\n"
old_block += re.escape(indent) + r"self\.turn_threshold = 0\.05\s+# Tolerance to decide when turning is complete\n"
old_block += re.escape(indent) + r"self\.last_turn_error = 0\.0"

new_block = indent + "self.bridge = CvBridge()\n"
new_block += indent + "self.lane_detected = False\n\n"
new_block += indent + "# Parameter settings (diff_robot_fifth: ~0.08 x 0.05 m; zone only in front of robot)\n"
new_block += indent + "self.declare_parameter('danger_distance', 0.12)\n"
new_block += indent + "self.declare_parameter('danger_width', 0.06)\n"
new_block += indent + "self.declare_parameter('speed', 0.02)\n"
new_block += indent + "self.declare_parameter('robot_width_m', 0.05)\n"
new_block += indent + "self.declare_parameter('robot_height_m', 0.08)\n"
new_block += indent + "self.declare_parameter('turn_Kp', 0.45)\n"
new_block += indent + "self.declare_parameter('turn_Kd', 0.03)\n"
new_block += indent + "self.declare_parameter('turn_threshold', 0.05)\n"
new_block += indent + "self.danger_distance = self.get_parameter('danger_distance').value\n"
new_block += indent + "self.danger_width = self.get_parameter('danger_width').value\n"
new_block += indent + "self.speed = self.get_parameter('speed').value\n"
new_block += indent + "self.robot_width_m = self.get_parameter('robot_width_m').value\n"
new_block += indent + "self.robot_height_m = self.get_parameter('robot_height_m').value\n"
new_block += indent + "self.turn_Kp = self.get_parameter('turn_Kp').value\n"
new_block += indent + "self.turn_Kd = self.get_parameter('turn_Kd').value\n"
new_block += indent + "self.turn_threshold = self.get_parameter('turn_threshold').value\n"
new_block += indent + "self.last_turn_error = 0.0"

content_new, n = re.subn(old_block, new_block, content, count=1)
if n != 1:
    sys.exit("patch_avoid_construction_params: init block not found (regex did not match)")
content = content_new

# visualization(): габариты из self (отступ может быть разный)
content = re.sub(
    r"^(\s+)robot_width_m = 0\.12\n\1robot_height_m = 0\.12",
    r"\1robot_width_m = self.robot_width_m\n\1robot_height_m = self.robot_height_m",
    content,
    count=1,
    flags=re.MULTILINE,
)

with open(path, "w", encoding="utf-8") as f:
    f.write(content)
print("Patched:", path)
