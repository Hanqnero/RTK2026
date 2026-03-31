#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${PI_HOST:-pi@192.168.2.2}"
CONTAINER="${CONTAINER:-rtk2026}"
RUN_TELEOP="${1:-}"
USE_CAMERA="${USE_CAMERA:-false}"

echo "[1/5] Restarting ${CONTAINER} on ${PI_HOST}..."
ssh "${PI_HOST}" "docker restart ${CONTAINER} >/dev/null"

echo "[2/5] Starting base stack (pi_full: SLAM on, lane off)..."
ssh "${PI_HOST}" "docker exec -d ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; nohup ros2 launch rtk2026_bringup pi_full.launch.py use_lane:=false use_slam:=false use_localization:=true use_camera:=${USE_CAMERA} > /tmp/ros_slam.log 2>&1 &'"

echo "[3/5] Starting Nav2 in SLAM mode..."
ssh "${PI_HOST}" "docker exec -d ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; nohup ros2 launch rtk2026_nav2_explorer rtk2026_nav2_slam.launch.py use_sim_time:=false > /tmp/ros_nav2.log 2>&1 &'"

echo "[4/5] Starting goal bridge for Foxglove click (/move_base_simple/goal -> /goal_pose)..."
ssh "${PI_HOST}" "docker exec ${CONTAINER} bash -lc 'cat > /tmp/goal_pose_relay_inline.py <<\"PY\"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalRelay(Node):
    def __init__(self):
        super().__init__(\"goal_pose_relay_inline\")
        self.pub = self.create_publisher(PoseStamped, \"/goal_pose\", 10)
        self.create_subscription(PoseStamped, \"/move_base_simple/goal\", self.cb, 10)

    def cb(self, msg: PoseStamped):
        self.pub.publish(msg)

rclpy.init()
node = GoalRelay()
rclpy.spin(node)
PY
chmod +x /tmp/goal_pose_relay_inline.py'"
ssh "${PI_HOST}" "docker exec -d ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; nohup python3 /tmp/goal_pose_relay_inline.py > /tmp/goal_relay.log 2>&1 &'"

echo "[5/5] Sanity checks..."
sleep 6
ssh "${PI_HOST}" "docker exec ${CONTAINER} bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  ros2 node list | grep -E \"^/(foxglove_bridge|base_controller|arduino_bridge|slam_toolbox|bt_navigator|planner_server|controller_server)$\" || true
  echo ---
  ros2 topic list | grep -E \"^/map$|^/goal_pose$|^/move_base_simple/goal$|^/cmd_vel$\" || true
'"

echo
echo "Foxglove WS: ws://192.168.2.2:8765"
echo "For Nav2 interactive goals: click in Foxglove on /move_base_simple/goal (PoseStamped)."
echo "Goal relay forwards to /goal_pose."

if [[ "${RUN_TELEOP}" == "--teleop" ]]; then
  echo
  echo "Starting keyboard teleop (Ctrl-C to stop):"
  ssh -t "${PI_HOST}" "docker exec -it ${CONTAINER} bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
  '"
fi
