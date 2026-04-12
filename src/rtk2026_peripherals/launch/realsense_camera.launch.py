# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Intel RealSense wrapper around realsense2_camera/rs_launch.py.
# IMU: gyro + accel enabled, объединяются в /camera/imu (unite_imu_method=2).
# Топики: /camera/color/image_raw, /camera/depth/image_rect_raw,
#         /camera/imu, /camera/color/camera_info

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rs_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value=""),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_infra1", default_value="false"),
        DeclareLaunchArgument("enable_infra2", default_value="false"),
        DeclareLaunchArgument("enable_gyro", default_value="true"),
        DeclareLaunchArgument("enable_accel", default_value="true"),
        # unite_imu_method=2: линейная интерполяция accel+gyro → /camera/imu
        DeclareLaunchArgument("unite_imu_method", default_value="2"),
        DeclareLaunchArgument("pointcloud_enable", default_value="false"),
        DeclareLaunchArgument("align_depth_enable", default_value="true"),
        DeclareLaunchArgument("enable_sync", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                "camera_name": LaunchConfiguration("camera_name"),
                "camera_namespace": LaunchConfiguration("camera_namespace"),
                "enable_color": LaunchConfiguration("enable_color"),
                "enable_depth": LaunchConfiguration("enable_depth"),
                "enable_infra1": LaunchConfiguration("enable_infra1"),
                "enable_infra2": LaunchConfiguration("enable_infra2"),
                "enable_gyro": LaunchConfiguration("enable_gyro"),
                "enable_accel": LaunchConfiguration("enable_accel"),
                "unite_imu_method": LaunchConfiguration("unite_imu_method"),
                "pointcloud.enable": LaunchConfiguration("pointcloud_enable"),
                "align_depth.enable": LaunchConfiguration("align_depth_enable"),
                "enable_sync": LaunchConfiguration("enable_sync"),
            }.items(),
        ),
    ])
