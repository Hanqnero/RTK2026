# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("rtk2026_peripherals"), "launch", "realsense_camera.launch.py"
            ])
        ]),
        launch_arguments={
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
        }.items(),
    )

    calibration_node = Node(
        package="rtk2026_peripherals",
        executable="board_calibration",
        name="board_calibration",
        output="screen",
        parameters=[{
            "image_topic": "/camera/color/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "overlay_topic": "/camera/calibration/chessboard_overlay",
            "status_topic": "/camera/calibration/status",
            "rms_topic": "/camera/calibration/rms_error",
            "board_cols": 7,
            "board_rows": 10,
            "square_size_m": 0.025,
            "min_samples": 12,
            "max_samples": 30,
        }],
    )

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
        realsense_launch,
        calibration_node,
        foxglove_bridge_node,
    ])
