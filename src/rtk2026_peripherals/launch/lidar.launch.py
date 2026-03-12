# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# 2D lidar LD19 (ldlidar_stl_ros2). Frame lidar_link, topic /scan. See MentorPi peripherals.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id", default="lidar_link")
    scan_topic = LaunchConfiguration("scan_topic", default="scan")
    port_name = LaunchConfiguration("port_name", default="/dev/ldlidar")

    ld19_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="ld19",
        output="screen",
        parameters=[
            {
                "topic_name": scan_topic,
                "product_name": "LDLiDAR_LD19",
                "port_baudrate": 230400,
                "port_name": port_name,
                "frame_id": frame_id,
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="lidar_link",
                              description="TF frame_id for scan"),
        DeclareLaunchArgument("scan_topic", default_value="scan",
                              description="LaserScan topic name"),
        DeclareLaunchArgument("port_name", default_value="/dev/ldlidar",
                              description="Serial port for lidar"),
        ld19_node,
    ])
