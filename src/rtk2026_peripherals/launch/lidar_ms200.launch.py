# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# 2D lidar MS200 (oradar_lidar). Frame lidar_link, topic /scan.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id", default="lidar_link")
    scan_topic = LaunchConfiguration("scan_topic", default="scan")
    port_name = LaunchConfiguration("port_name", default="/dev/ldlidar")

    ms200_node = Node(
        package="oradar_lidar",
        executable="oradar_scan",
        name="ms200",
        output="screen",
        parameters=[
            {"device_model": "MS200"},
            {"frame_id": frame_id},
            {"scan_topic": "MS200/scan"},
            {"port_name": port_name},
            {"baudrate": 230400},
            {"angle_min": 0.0},
            {"angle_max": 360.0},
            {"range_min": 0.05},
            {"range_max": 12.0},
            {"clockwise": False},
            {"motor_speed": 15},
        ],
        remappings=[("MS200/scan", scan_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="lidar_link"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("port_name", default_value="/dev/ldlidar"),
        ms200_node,
    ])
