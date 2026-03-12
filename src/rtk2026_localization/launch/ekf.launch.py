# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    ekf_config = PathJoinSubstitution([
        FindPackageShare("rtk2026_localization"), "config", "ekf.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time"),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        ),
    ])
