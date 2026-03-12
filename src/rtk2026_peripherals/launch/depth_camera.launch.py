# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# USB camera (stereo/depth: use realsense or ascamera separately). MentorPi-style.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("rtk2026_peripherals"), "config", "usb_cam.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config_path,
                              description="Path to usb_cam yaml"),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            parameters=[LaunchConfiguration("config")],
            remappings=[
                ("image_raw", "camera/image_raw"),
                ("camera_info", "camera/camera_info"),
            ],
        ),
    ])
