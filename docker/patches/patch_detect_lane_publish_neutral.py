#!/usr/bin/env python3
# При Lane state 0 раньше принудительно публиковали фиксированный центр (500 = прямо),
# из-за чего робот начинал "тянуться" к серединной линии даже при пропаже детекции.
# Теперь при отсутствии центра используем последнюю валидную оценку (prev_centerx).
# Отступы вычисляются из исходника (regex), без хардкода.

import re
import sys

path = "/workspace/src/turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_lane.py"
if len(sys.argv) > 1:
    path = sys.argv[1]

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

# Определяем отступ по строке с pub_image_type (только пробелы/табы)
m = re.search(r"^([ \t]+)(?:if|elif)\s+self\.pub_image_type\s*==\s*['\"]raw['\"]", content, re.MULTILINE)
if not m:
    sys.exit("patch_detect_lane_publish_neutral: pub_image_type == 'raw' not found")
indent = m.group(1)
# Внутренние строки блока могут иметь любой отступ (\s+)
raw_block = re.escape(indent) + r"(?:elif|if)\s+self\.pub_image_type\s*==\s*['\"]raw['\"]\s*:\s*\n"
raw_block += r"\s+if\s+self\.is_center_x_exist\s*:\s*\n"
raw_block += r"\s+#\s*publishes lane center\s*\n"
raw_block += r"\s+msg_desired_center\s*=\s*Float64\(\)\s*\n"
raw_block += r"\s+msg_desired_center\.data\s*=\s*centerx\.item\(350\)\s*\n"
raw_block += r"\s+self\.pub_lane\.publish\(msg_desired_center\)\s*\n\s*\n"
raw_block += r"\s+self\.pub_image_lane\.publish\s*\(\s*self\.cvBridge\.cv2_to_imgmsg\s*\(\s*final\s*,\s*['\"]bgr8['\"]\s*\)\s*\)"

inner = indent + (" " * max(1, len(indent)))
inner2 = inner + (" " * max(1, len(indent)))
raw_repl = indent + "elif self.pub_image_type == 'raw':\n"
raw_repl += inner + "msg_desired_center = Float64()\n"
raw_repl += inner + "try:\n"
raw_repl += inner2 + "_cx = centerx.item(350)\n"
raw_repl += inner2 + "self.prev_centerx = _cx\n"
raw_repl += inner + "except (NameError, UnboundLocalError):\n"
raw_repl += inner2 + "_cx = None\n"
raw_repl += inner + "msg_desired_center.data = _cx if _cx is not None else getattr(self, 'prev_centerx', 500.0)\n"
raw_repl += inner + "self.pub_lane.publish(msg_desired_center)\n"
raw_repl += inner + "self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))"

content_new, n = re.subn(raw_block, raw_repl, content, count=1)
if n != 1:
    sys.exit("patch_detect_lane_publish_neutral: raw block not found")
content = content_new

# Compressed block
m2 = re.search(r"^(\s+)(?:if|elif)\s+self\.pub_image_type\s*==\s*['\"]compressed['\"]", content, re.MULTILINE)
if not m2:
    sys.exit("patch_detect_lane_publish_neutral: pub_image_type == 'compressed' not found")
indent_c = m2.group(1)
inner_c = indent_c + (" " * max(1, len(indent_c)))
inner_c2 = inner_c + (" " * max(1, len(indent_c)))

comp_block = re.escape(indent_c) + r"(?:if|elif)\s+self\.pub_image_type\s*==\s*['\"]compressed['\"]\s*:\s*\n"
comp_block += r"\s+if\s+self\.is_center_x_exist\s*:\s*\n"
comp_block += r"\s+#\s*publishes lane center\s*\n"
comp_block += r"\s+msg_desired_center\s*=\s*Float64\(\)\s*\n"
comp_block += r"\s+msg_desired_center\.data\s*=\s*centerx\.item\(350\)\s*\n"
comp_block += r"\s+self\.pub_lane\.publish\(msg_desired_center\)\s*\n\s*\n"
comp_block += r"\s+self\.pub_image_lane\.publish\s*\(\s*self\.cvBridge\.cv2_to_compressed_imgmsg\s*\(\s*final\s*,\s*['\"]jpg['\"]\s*\)\s*\)"

comp_repl = indent_c + "if self.pub_image_type == 'compressed':\n"
comp_repl += inner_c + "msg_desired_center = Float64()\n"
comp_repl += inner_c + "msg_desired_center.data = centerx.item(350) if self.is_center_x_exist else 500.0\n"
comp_repl += inner_c + "self.pub_lane.publish(msg_desired_center)\n"
comp_repl += inner_c + "self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))"

content_new2, n2 = re.subn(comp_block, comp_repl, content, count=1)
if n2 != 1:
    sys.exit("patch_detect_lane_publish_neutral: compressed block not found")
content = content_new2

with open(path, "w", encoding="utf-8") as f:
    f.write(content)
print("Patched:", path)
