# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Sign detector node (SIFT-based). Subscribes to a camera topic,
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

    config = LaunchConfiguration("config")
    camera_topic = LaunchConfiguration("camera_topic")
    dataset_root = LaunchConfiguration("dataset_root")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to sign_detector.yaml parameter file",
        ),
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/camera/image_raw",
            description="Camera image topic to subscribe to",
        ),
        DeclareLaunchArgument(
            "dataset_root",
            default_value="",
            description="Override dataset_root from config (leave empty to use config value)",
        ),
        Node(
            package="rtk2026_peripherals",
            executable="sign_detector",
            name="sign_detector",
            output="screen",
            parameters=[
                config,
                # Command-line overrides take priority over the yaml file.
                # Empty string means "don't override" — the yaml value is used.
                {"camera_topic": camera_topic},
            ],
        ),
    ])
