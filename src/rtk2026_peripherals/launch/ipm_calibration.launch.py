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
        ])
    )

    ipm_node = Node(
        package="rtk2026_peripherals",
        executable="image_relay_autorace",
        name="image_relay_autorace",
        output="screen",
        parameters=[{
            "use_ipm": True,
            "camera_height_m": 0.11,
            "camera_pitch_rad": 0.35,
            "y_near_m": 0.05,
            "y_far_m": 0.40,
            "x_left_m": -0.22,
            "x_right_m": 0.22,
            "projected_topic": "/camera/image_projected",
            "compensated_topic": "/camera/image_compensated",
            "camera_info_topic": "/camera/color/camera_info",
            "image_topic": "/camera/color/image_raw",
        }],
    )

    ipm_tuner_node = Node(
        package="rtk2026_peripherals",
        executable="ipm_tuner",
        name="ipm_tuner",
        output="screen",
        parameters=[{
            "target_node": "/image_relay_autorace",
            "set_topic": "/camera/ipm_tuning/set",
            "current_topic": "/camera/ipm_tuning/current",
            "status_topic": "/camera/ipm_tuning/status",
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
        ipm_node,
        ipm_tuner_node,
        foxglove_bridge_node,
    ])
