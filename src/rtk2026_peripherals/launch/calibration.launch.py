# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# Calibration launch for Intel RealSense + Foxglove.
# Connect Foxglove Studio to ws://localhost:8765, import the matching layout.
#
# Intrinsic mode (default):
#   ros2 launch rtk2026_peripherals calibration.launch.py
#   → RealSense + image_proc + foxglove_bridge
#   → Load foxglove_layouts/calibration_intrinsic.json
#   → Inspect /camera/color/camera_info for K, D, P matrices
#
# Extrinsic mode (IPM bird's-eye-view calibration):
#   ros2 launch rtk2026_peripherals calibration.launch.py extrinsic:=true
#   → additionally runs image_compensation + image_projection (calibration_mode=true)
#   → Load foxglove_layouts/calibration_extrinsic.json
#   → Tune via: ros2 param set /camera/image_projection top_x 70
#               ros2 param set /camera/image_projection top_y 4
#               ros2 param set /camera/image_projection bottom_x 260
#               ros2 param set /camera/image_projection bottom_y 160
#   → Save final values to:
#     src/turtlebot3_autorace/turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    extrinsic_arg = DeclareLaunchArgument(
        "extrinsic",
        default_value="false",
        description="Set true to run extrinsic (IPM) calibration nodes",
    )
    extrinsic = LaunchConfiguration("extrinsic")

    # --- RealSense camera ---
    # Depth/IMU disabled for calibration — color only, faster pipeline.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("rtk2026_peripherals"), "launch", "realsense_camera.launch.py"
            ])
        ]),
        launch_arguments={
            "enable_depth": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "pointcloud_enable": "false",
            "align_depth_enable": "false",
        }.items(),
    )

    # --- image_proc: rectify color image ---
    # Runs in camera/color namespace so it picks up:
    #   /camera/color/image_raw + /camera/color/camera_info
    # and publishes:
    #   /camera/color/image_rect_color
    image_proc_container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="camera/color",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_color",
                namespace="camera/color",
                parameters=[{"queue_size": 5}],
            )
        ],
    )

    # --- Extrinsic: turtlebot3_autorace IPM nodes ---
    try:
        autorace_share = get_package_share_directory("turtlebot3_autorace_camera")
        compensation_param = os.path.join(
            autorace_share, "calibration", "extrinsic_calibration", "compensation.yaml"
        )
        projection_param = os.path.join(
            autorace_share, "calibration", "extrinsic_calibration", "projection.yaml"
        )
    except Exception:
        compensation_param = ""
        projection_param = ""

    # brightness/color compensation (input: image_rect_color → output: image_compensated)
    image_compensation_node = Node(
        package="turtlebot3_autorace_camera",
        executable="image_compensation",
        namespace="camera",
        name="image_compensation",
        output="screen",
        parameters=[compensation_param, {"is_extrinsic_camera_calibration_mode": True}],
        remappings=[
            ("/camera/image_input", "/camera/color/image_rect_color"),
            ("/camera/image_input/compressed", "/camera/color/image_rect_color/compressed"),
            ("/camera/image_output", "/camera/image_compensated"),
            ("/camera/image_output/compressed", "/camera/image_compensated/compressed"),
        ],
        condition=IfCondition(extrinsic),
    )

    # IPM homography projection (input: image_compensated → output: image_projected + image_extrinsic_calib)
    image_projection_node = Node(
        package="turtlebot3_autorace_camera",
        executable="image_projection",
        namespace="camera",
        name="image_projection",
        output="screen",
        parameters=[projection_param, {"is_extrinsic_camera_calibration_mode": True}],
        remappings=[
            ("/camera/image_input", "/camera/image_compensated"),
            ("/camera/image_input/compressed", "/camera/image_compensated/compressed"),
            ("/camera/image_output", "/camera/image_projected"),
            ("/camera/image_output/compressed", "/camera/image_projected/compressed"),
            ("/camera/image_calib", "/camera/image_extrinsic_calib"),
            ("/camera/image_calib/compressed", "/camera/image_extrinsic_calib/compressed"),
        ],
        condition=IfCondition(extrinsic),
    )

    # --- Foxglove bridge ---
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
            "send_buffer_limit": 10000000,
        }],
    )

    return LaunchDescription([
        extrinsic_arg,
        realsense_launch,
        image_proc_container,
        image_compensation_node,
        image_projection_node,
        foxglove_bridge_node,
    ])
