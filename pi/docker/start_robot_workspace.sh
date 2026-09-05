#!/usr/bin/env bash
# Build and enter the bind-mounted real-robot workspace.

set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /opt/vendor_ws/install/setup.bash
set -u

if ! ros2 pkg prefix sllidar_ros2 >/dev/null; then
    printf '%s\n' \
        'sllidar_ros2 is missing from the vendor underlay.' \
        'Rebuild the ROS image; do not reuse the cached lidar_driver layer.' >&2
    exit 1
fi

cd /workspaces/robot_ws
colcon build \
    --symlink-install \
    --packages-select \
        rtk2026_driver \
        rtk2026_description \
        rtk2026_localization \
        rtk2026_slam \
        rtk2026_observability \
        rtk2026_bringup \
        rtk2026_interfaces

set +u
source /workspaces/robot_ws/install/setup.bash
set -u

exec bash
