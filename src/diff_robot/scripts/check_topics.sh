#!/bin/bash
# Check required topics for Gazebo + Nav2 SLAM stack (run inside container after launch).
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash 2>/dev/null || true

echo "=== Required topics ==="
echo "/scan       - LaserScan from Gazebo (slam_toolbox, costmaps)"
echo "/odom       - Odometry from Gazebo diff_drive"
echo "/map        - OccupancyGrid from slam_toolbox (RViz Map display)"
echo "/tf, /tf_static - Transforms (map->odom, odom->base_link, ...)"
echo "/cmd_vel    - Velocity commands (Nav2 -> Gazebo)"
echo ""

echo "=== Topic list (filtered) ==="
ros2 topic list | grep -E "scan|odom|map|cmd_vel|tf" || true

echo ""
echo "=== /scan (one message) ==="
timeout 3 ros2 topic echo /scan --once 2>/dev/null && echo "OK" || echo "No message or timeout"

echo ""
echo "=== /odom (one message) ==="
timeout 3 ros2 topic echo /odom --once 2>/dev/null && echo "OK" || echo "No message or timeout"

echo ""
echo "=== /map (one message) ==="
timeout 5 ros2 topic echo /map --once 2>/dev/null && echo "OK" || echo "No message or timeout (slam_toolbox may need robot movement first)"

echo ""
echo "=== Nodes (slam_toolbox, nav2) ==="
ros2 node list | grep -E "slam|controller_server|planner_server" || true
