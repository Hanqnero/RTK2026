# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_description = LaunchConfiguration("use_description", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_foxglove = LaunchConfiguration("use_foxglove", default="false")
    use_fake_encoder = LaunchConfiguration("use_fake_encoder", default="false")
    use_localization = LaunchConfiguration("use_localization", default="false")
    use_slam = LaunchConfiguration("use_slam", default="false")
    use_navigation = LaunchConfiguration("use_navigation", default="false")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    publish_tf = PythonExpression([
        "'false' if 'true' == '", use_localization, "' else 'true'"
    ])

    driver_base_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_bringup"), "launch", "rtk2026_driver_base.launch.py"
    ])
    ekf_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_localization"), "launch", "ekf.launch.py"
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_slam"), "launch", "slam.launch.py"
    ])
    nav_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_navigation"), "launch", "navigation.launch.py"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_description", default_value="true",
                             description="Include robot description and rviz."),
        DeclareLaunchArgument("use_rviz", default_value="true",
                             description="Launch RViz2 (set false for headless, e.g. Raspberry Pi)."),
        DeclareLaunchArgument("use_foxglove", default_value="false",
                             description="Launch foxglove_bridge on port 8765."),
        DeclareLaunchArgument("use_fake_encoder", default_value="false",
                             description="Use fake encoder (no hardware)."),
        DeclareLaunchArgument("use_localization", default_value="false",
                             description="Start EKF; base_controller will not publish tf."),
        DeclareLaunchArgument("use_slam", default_value="false",
                             description="Start slam_toolbox (requires /scan)."),
        DeclareLaunchArgument("use_navigation", default_value="false",
                             description="Start Nav2 (requires map and /scan)."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                             description="Use simulation time."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_base_launch),
            launch_arguments=[
                ("use_description", use_description),
                ("use_rviz", use_rviz),
                ("use_foxglove", use_foxglove),
                ("use_fake_encoder", use_fake_encoder),
                ("publish_tf", publish_tf),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments=[("use_sim_time", use_sim_time)],
            condition=IfCondition(use_localization),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments=[("use_sim_time", use_sim_time)],
            condition=IfCondition(use_slam),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments=[
                ("use_sim_time", use_sim_time),
            ],
            condition=IfCondition(use_navigation),
        ),
    ])
