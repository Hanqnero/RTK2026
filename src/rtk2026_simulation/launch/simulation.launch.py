# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _robot_description_string():
    pkg_share = get_package_share_directory("rtk2026_description")
    xacro_path = Path(pkg_share) / "urdf" / "rtk2026.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(xacro_path), "prefix:="],
        capture_output=True,
        text=True,
        timeout=10,
    )
    result.check_returncode()
    return result.stdout


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_slam = LaunchConfiguration("use_slam", default="false")
    use_navigation = LaunchConfiguration("use_navigation", default="false")
    use_localization = LaunchConfiguration("use_localization", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="false")
    rviz_config = LaunchConfiguration("rviz_config", default="rtk2026.rviz")
    use_fake_scan = LaunchConfiguration("use_fake_scan", default="false")
    use_fake_odom = LaunchConfiguration("use_fake_odom", default="false")

    description_package = "rtk2026_description"
    robot_description = {"robot_description": _robot_description_string()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ekf_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_localization"), "launch", "ekf.launch.py"
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_slam"), "launch", "slam.launch.py"
    ])
    slam_fake_odom_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_slam"), "launch", "slam_fake_odom.launch.py"
    ])
    nav_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_navigation"), "launch", "navigation.launch.py"
    ])
    fake_scan_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_peripherals"), "launch", "fake_scan.launch.py"
    ])
    odom_tf_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_peripherals"), "launch", "odom_tf_broadcaster.launch.py"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(description_package), "rviz", rviz_config
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (required when running with Isaac Lab).",
        ),
        DeclareLaunchArgument(
            "use_slam",
            default_value="false",
            description="Start slam_toolbox; expects /scan from simulator.",
        ),
        DeclareLaunchArgument(
            "use_navigation",
            default_value="false",
            description="Start Nav2; expects map and /scan.",
        ),
        DeclareLaunchArgument(
            "use_localization",
            default_value="false",
            description="Start EKF (fuse /odom from simulator).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="rtk2026.rviz",
            description="RViz config filename (in rtk2026_description/rviz). Use rtk2026_sim_slam.rviz for odom trajectory + map.",
        ),
        DeclareLaunchArgument(
            "use_fake_scan",
            default_value="false",
            description="Start fake_scan (/scan) when sim does not provide LaserScan.",
        ),
        DeclareLaunchArgument(
            "use_fake_odom",
            default_value="false",
            description="Publish static tf odom->base_link so Nav2 starts when Isaac/sim does not publish /odom.",
        ),
        DeclareLaunchArgument(
            "nav2_trigger_delay_sec",
            default_value="30",
            description="Seconds to wait before checking TF and calling Nav2 manage_nodes (use_fake_odom one-container).",
        ),
        DeclareLaunchArgument(
            "nav2_trigger_tf_timeout_sec",
            default_value="120",
            description="Max seconds to wait for TF (source_frame->target_frame) before calling manage_nodes anyway.",
        ),
        DeclareLaunchArgument(
            "nav2_trigger_odom_frame",
            default_value="odom",
            description="TF frame ID that Nav2 expects (target_frame for trigger wait).",
        ),
        DeclareLaunchArgument(
            "nav2_trigger_base_frame",
            default_value="base_link",
            description="TF frame ID of robot base (source_frame for trigger wait).",
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments=[("use_sim_time", use_sim_time)],
            condition=IfCondition(use_localization),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_fake_odom_launch),
            launch_arguments=[("use_sim_time", use_sim_time)],
            condition=IfCondition(
                PythonExpression(["'", use_slam, "' == 'true' and '", use_fake_odom, "' == 'true'"]),
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments=[("use_sim_time", use_sim_time)],
            condition=IfCondition(
                PythonExpression(["'", use_slam, "' == 'true' and '", use_fake_odom, "' == 'false'"]),
            ),
        ),
        # Nav2 when NOT use_fake_odom (e.g. Isaac): start Nav2 with default autostart after 30s.
        GroupAction(
            [
                TimerAction(
                    period=30.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(nav_launch),
                            launch_arguments=[("use_sim_time", use_sim_time)],
                        ),
                    ],
                ),
            ],
            condition=IfCondition(
                PythonExpression(["'", use_navigation, "' == 'true' and '", use_fake_odom, "' == 'false'"]),
            ),
        ),
        # Nav2 one-container (use_fake_odom): start Nav2 with autostart:=false; trigger waits for TF then manage_nodes.
        GroupAction(
            [
                TimerAction(
                    period=30.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(nav_launch),
                            launch_arguments=[
                                ("use_sim_time", use_sim_time),
                                ("autostart", "false"),
                            ],
                        ),
                        Node(
                            package="rtk2026_peripherals",
                            executable="trigger_nav2_bringup",
                            name="trigger_nav2_bringup",
                            output="screen",
                            parameters=[
                                {"use_sim_time": use_sim_time},
                                {"delay_sec": LaunchConfiguration("nav2_trigger_delay_sec")},
                                {"tf_timeout_sec": LaunchConfiguration("nav2_trigger_tf_timeout_sec")},
                                {"target_frame": LaunchConfiguration("nav2_trigger_odom_frame")},
                                {"source_frame": LaunchConfiguration("nav2_trigger_base_frame")},
                            ],
                        ),
                    ],
                ),
            ],
            condition=IfCondition(
                PythonExpression(["'", use_navigation, "' == 'true' and '", use_fake_odom, "' == 'true'"]),
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fake_scan_launch),
            condition=IfCondition(use_fake_scan),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_tf_launch),
            launch_arguments=[("use_sim_time", use_sim_time)],
            condition=IfCondition(
                PythonExpression([
                    "'", use_sim_time, "' == 'true' and '", use_fake_odom, "' == 'false'",
                ])
            ),
        ),
        Node(
            package="rtk2026_peripherals",
            executable="static_odom_tf_publisher",
            name="static_odom_tf_publisher",
            output="log",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_fake_odom),
        ),
        Node(
            package="rtk2026_peripherals",
            executable="clock_publisher",
            name="clock_publisher",
            output="log",
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(use_fake_odom),
        ),
        Node(
            package="rtk2026_peripherals",
            executable="static_map_publisher",
            name="static_map_publisher",
            output="log",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_fake_odom),
        ),
    ])
