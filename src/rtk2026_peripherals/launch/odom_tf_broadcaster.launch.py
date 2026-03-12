# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Publish tf odom -> base_link from /odom (for sim when bridge does not publish tf).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="odom"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="rtk2026_peripherals",
            executable="odom_tf_broadcaster",
            name="odom_tf_broadcaster",
            output="screen",
            parameters=[{
                "odom_topic": LaunchConfiguration("odom_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
