# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# 2D lidar Slamtec RPLIDAR C1 (sllidar_ros2). Frame lidar_link, topic /scan.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration(
        "serial_port",
        default="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_fc9214c1fa63ef119a7ae3a9c169b110-if00-port0",
    )
    frame_id = LaunchConfiguration("frame_id", default="lidar_link")

    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "serial_baudrate": 460800,
                "frame_id": frame_id,
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
        remappings=[("scan", "scan")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_fc9214c1fa63ef119a7ae3a9c169b110-if00-port0",
                              description="Serial port for lidar"),
        DeclareLaunchArgument("frame_id", default_value="lidar_link",
                              description="TF frame_id for scan"),
        sllidar_node,
    ])
