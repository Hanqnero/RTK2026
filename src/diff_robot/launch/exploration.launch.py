from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (true for Gazebo).",
        ),
        DeclareLaunchArgument(
            "explore_period_sec",
            default_value="5.0",
            description="Period (seconds) between frontier checks.",
        ),
        DeclareLaunchArgument(
            "min_frontier_size",
            default_value="5",
            description="Minimum frontier cluster size (cells).",
        ),
        Node(
            package="custom_explorer",
            executable="explorer",
            name="explorer",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"explore_period_sec": LaunchConfiguration("explore_period_sec")},
                {"min_frontier_size": LaunchConfiguration("min_frontier_size")},
            ],
        ),
    ])
