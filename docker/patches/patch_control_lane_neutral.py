#!/usr/bin/env python3
# Патч control_lane:
# 1. twist.linear.x = self.MAX_VEL  (без снижения скорости на поворотах)
# 2. Kd 0.007 -> 0.002 (при константной скорости большой Kd вызывает осцилляции)
# 3. World-reset detection: sim time jump backward → reset last_error + avoid_active

import sys

path = "/workspace/src/turtlebot3_autorace/turtlebot3_autorace_mission/turtlebot3_autorace_mission/control_lane.py"
if len(sys.argv) > 1:
    path = sys.argv[1]

with open(path, "r", encoding="utf-8") as f:
    content = f.read()

# Patch 1: constant linear velocity
old1 = "twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)"
new1 = "twist.linear.x = self.MAX_VEL"
if old1 not in content:
    sys.exit("patch_control_lane_neutral: linear.x line not found")
content = content.replace(old1, new1, 1)

# Patch 2: lower Kd (constant speed needs less derivative gain)
old2 = "Kd = 0.007"
new2 = "Kd = 0.002"
if old2 not in content:
    sys.exit("patch_control_lane_neutral: Kd = 0.007 not found")
content = content.replace(old2, new2, 1)

# Patch 3: world-reset detection — inject timer + reset logic into __init__ and add method
old3 = (
    "        # PD control related variables\n"
    "        self.last_error = 0\n"
    "        self.MAX_VEL = 0.1\n"
    "\n"
    "        # Avoidance mode related variables\n"
    "        self.avoid_active = False\n"
    "        self.avoid_twist = Twist()"
)
new3 = (
    "        # PD control related variables\n"
    "        self.last_error = 0\n"
    "        self.MAX_VEL = 0.1\n"
    "\n"
    "        # Avoidance mode related variables\n"
    "        self.avoid_active = False\n"
    "        self.avoid_twist = Twist()\n"
    "\n"
    "        # World-reset detection\n"
    "        self._last_sim_time = None\n"
    "        self.create_timer(0.2, self._check_world_reset)"
)
if old3 not in content:
    sys.exit("patch_control_lane_neutral: avoid_active block not found")
content = content.replace(old3, new3, 1)

old4 = "    def callback_get_max_vel(self, max_vel_msg):"
new4 = (
    "    def _check_world_reset(self):\n"
    "        now_sec = self.get_clock().now().nanoseconds * 1e-9\n"
    "        if self._last_sim_time is not None and now_sec < self._last_sim_time - 1.0:\n"
    "            self.get_logger().info('World reset detected in control_lane. Resetting state.')\n"
    "            self.last_error = 0\n"
    "            self.avoid_active = False\n"
    "        self._last_sim_time = now_sec\n"
    "\n"
    "    def callback_get_max_vel(self, max_vel_msg):"
)
if old4 not in content:
    sys.exit("patch_control_lane_neutral: callback_get_max_vel not found")
content = content.replace(old4, new4, 1)

with open(path, "w", encoding="utf-8") as f:
    f.write(content)
print("Patched:", path)
