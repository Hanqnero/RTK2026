# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_description = LaunchConfiguration("use_description", default="true")
    use_fake_encoder = LaunchConfiguration("use_fake_encoder", default="false")
    publish_tf = LaunchConfiguration("publish_tf", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_foxglove = LaunchConfiguration("use_foxglove", default="false")

    declared = [
        DeclareLaunchArgument(
            "use_description",
            default_value="true",
            description="If true, include robot description (robot_state_publisher, rviz).",
        ),
        DeclareLaunchArgument(
            "use_fake_encoder",
            default_value="false",
            description="If true, run fake_encoder instead of arduino_bridge (no serial needed).",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="If false, base_controller does not publish odom->base_link (use with EKF).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 (set false for headless, e.g. Raspberry Pi).",
        ),
        DeclareLaunchArgument(
            "use_foxglove",
            default_value="false",
            description="Launch foxglove_bridge (WebSocket on port 8765 for Foxglove Studio).",
        ),
    ]

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rtk2026_description"), "launch", "view_rtk2026.launch.py"
            ])
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
        condition=IfCondition(use_description),
    )

    driver_config = PathJoinSubstitution([
        FindPackageShare("rtk2026_driver"), "config", "arduino_bridge.yaml"
    ])
    base_config = PathJoinSubstitution([
        FindPackageShare("rtk2026_base"), "config", "base_controller.yaml"
    ])

    arduino_bridge_node = Node(
        package="rtk2026_driver",
        executable="arduino_bridge",
        name="arduino_bridge",
        parameters=[driver_config],
        output="screen",
        condition=UnlessCondition(use_fake_encoder),
    )
    fake_encoder_node = Node(
        package="rtk2026_driver",
        executable="fake_encoder",
        name="fake_encoder",
        parameters=[driver_config],
        output="screen",
        condition=IfCondition(use_fake_encoder),
    )
    base_controller_node = Node(
        package="rtk2026_base",
        executable="base_controller",
        name="base_controller",
        parameters=[base_config, {"publish_tf": publish_tf}],
        output="screen",
    )
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765}],
        output="screen",
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription(
        declared
        + [
            description_launch,
            arduino_bridge_node,
            fake_encoder_node,
            base_controller_node,
            foxglove_bridge_node,
        ]
    )
