# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# IMU complementary filter and optional tf_broadcaster_imu. Expects imu_raw topic from driver.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_tf_broadcaster = LaunchConfiguration("use_tf_broadcaster", default="true")
    imu_raw_topic = LaunchConfiguration("imu_raw_topic", default="imu_raw")
    imu_topic = LaunchConfiguration("imu_topic", default="imu")

    imu_filter_node = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        name="imu_filter",
        output="screen",
        parameters=[
            {
                "use_mag": False,
                "do_bias_estimation": True,
                "do_adaptive_gain": True,
                "publish_debug_topics": False,
            }
        ],
        remappings=[
            ("imu/data_raw", imu_raw_topic),
            ("imu/data", imu_topic),
        ],
    )

    tf_broadcaster_node = Node(
        package="rtk2026_peripherals",
        executable="tf_broadcaster_imu",
        name="tf_broadcaster_imu",
        output="screen",
        parameters=[{"imu_topic": imu_topic}],
        condition=IfCondition(use_tf_broadcaster),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_tf_broadcaster", default_value="true",
                              description="Run tf_broadcaster_imu (imu_link -> imu_optical_frame)"),
        DeclareLaunchArgument("imu_raw_topic", default_value="imu_raw",
                              description="Topic from IMU driver (sensor_msgs/Imu)"),
        DeclareLaunchArgument("imu_topic", default_value="imu",
                              description="Filtered IMU topic for EKF"),
        imu_filter_node,
        tf_broadcaster_node,
    ])
