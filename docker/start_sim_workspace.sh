#!/usr/bin/env bash
# Build and enter the bind-mounted simulation workspace.

set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /opt/vo_ws/install/setup.bash
set -u

cd /workspaces/sim_ws
colcon build \
    --symlink-install \
    --packages-select \
        rtk2026_interfaces \
        rtk2026_pose_graph \
        rtk2026_graph \
        rtk2026_route_nav \
        rtk2026_city_nav \
        rtk2026_nav2 \
        rtk2026_vector_objects \
        rtk2026_description \
        rtk2026_driver \
        rtk2026_slam \
        rtk2026_localization \
        rtk2026_observability \
        rtk2026_tracked_sim \
        rtk2026_cv \
        rtk2026_bringup

set +u
source /workspaces/sim_ws/install/setup.bash
set -u

exec bash
