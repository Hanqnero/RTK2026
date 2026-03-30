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

    image_projection_node = Node(
        package="turtlebot3_autorace_camera",
        executable="image_projection",
        name="image_projection_calib",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_autorace_camera"),
                "calibration",
                "extrinsic_calibration",
                "projection.yaml",
            ]),
            {"is_extrinsic_camera_calibration_mode": True},
        ],
        remappings=[
            ("/camera/image_input/compressed", "/camera/color/image_raw/compressed"),
            ("/camera/image_output", "/camera/image_projected"),
            ("/camera/image_output/compressed", "/camera/image_projected/compressed"),
            ("/camera/image_calib", "/camera/image_extrinsic_calib"),
            ("/camera/image_calib/compressed", "/camera/image_extrinsic_calib/compressed"),
        ],
    )

    projection_tuner_node = Node(
        package="rtk2026_peripherals",
        executable="projection_tuner",
        name="projection_tuner",
        output="screen",
        parameters=[{
            "target_node": "/image_projection_calib",
            "set_topic": "/camera/extrinsic_tuning/set",
            "current_topic": "/camera/extrinsic_tuning/current",
            "status_topic": "/camera/extrinsic_tuning/status",
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
        image_projection_node,
        projection_tuner_node,
        foxglove_bridge_node,
    ])
