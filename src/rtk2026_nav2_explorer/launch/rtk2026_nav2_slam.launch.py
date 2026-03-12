from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map", default="/tmp/empty.yaml")

    nav2_bringup_launch = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bringup"),
            "launch",
            "bringup_launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time in Nav2 + SLAM.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rtk2026_nav2_explorer"),
                        "config",
                        "nav2_params_slam.yaml",
                    ]
                ),
                description="Nav2 params file tuned for SLAM exploration.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="/tmp/empty.yaml",
                description="Initial map yaml (ignored by slam_toolbox, kept for API compatibility).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam": "True",
                    "params_file": params_file,
                    "map": map_yaml,
                }.items(),
            ),
        ]
    )

