# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    autostart = LaunchConfiguration("autostart", default="true")
    params_file = PathJoinSubstitution([
        FindPackageShare("rtk2026_navigation"), "config", "nav2_params.yaml"
    ])
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation/Gazebo clock",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Auto-start lifecycle bringup (false = call manage_nodes later)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=params_file,
            description="Full path to Nav2 params YAML",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments=[
                ("params_file", params_file),
                ("use_sim_time", use_sim_time),
                ("autostart", autostart),
            ],
        ),
    ])
