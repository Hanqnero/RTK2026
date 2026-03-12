# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Publish fake LaserScan on /scan for testing without lidar.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="lidar_link"),
        DeclareLaunchArgument("topic", default_value="scan"),
        Node(
            package="rtk2026_peripherals",
            executable="fake_scan",
            name="fake_scan",
            output="screen",
            parameters=[{
                "frame_id": LaunchConfiguration("frame_id"),
                "topic": LaunchConfiguration("topic"),
                "rate": 10.0,
            }],
        ),
    ])
