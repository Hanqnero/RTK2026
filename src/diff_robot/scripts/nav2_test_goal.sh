#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{pose: {
    header: {frame_id: \"map\"},
    pose: {
      position: {x: 2.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"

