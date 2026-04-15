from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odometry_cfg = PathJoinSubstitution([
        FindPackageShare("rtk2026_odometry"),
        "config",
        "odometry.yaml",
    ])
    sim_encoder_cfg = PathJoinSubstitution([
        FindPackageShare("rtk2026_odometry"),
        "config",
        "sim_encoder.yaml",
    ])

    return LaunchDescription([
        Node(
            package="rtk2026_odometry",
            executable="sim_encoder",
            parameters=[sim_encoder_cfg, {"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="rtk2026_odometry",
            executable="wheel_odometry",
            parameters=[odometry_cfg, {"use_sim_time": True}],
            output="screen",
        ),
    ])
