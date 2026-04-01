# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Sign detector node (YOLO ONNX). Subscribes to a RealSense color topic,
# publishes SignDetection on /sign_detections.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rtk2026_peripherals")
    default_config = os.path.join(pkg_share, "config", "sign_detector.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to sign_detector.yaml parameter file",
        ),
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/camera/color/image_raw",
            description="RealSense color image topic",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/aligned_depth_to_color/image_raw",
            description="Aligned depth topic — set to '' to disable depth gating",
        ),
        Node(
            package="rtk2026_peripherals",
            executable="sign_detector",
            name="sign_detector",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "camera_topic": LaunchConfiguration("camera_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                },
            ],
        ),
    ])
