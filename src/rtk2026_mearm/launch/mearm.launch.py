from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare("rtk2026_mearm"),
        "config",
        "mearm.yaml",
    ])

    dry_run = LaunchConfiguration("dry_run")
    block = LaunchConfiguration("block")
    address = LaunchConfiguration("address")
    vendor_path = LaunchConfiguration("vendor_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Log commands without talking to the PWM/I2C hardware.",
        ),
        DeclareLaunchArgument(
            "block",
            default_value="0",
            description="PCA9685 servo connector block, from 0 to 3.",
        ),
        DeclareLaunchArgument(
            "address",
            default_value="64",
            description="PCA9685 I2C address as an integer, 64 means 0x40.",
        ),
        DeclareLaunchArgument(
            "vendor_path",
            default_value="",
            description="Optional path to vendor/mearm when not using the packaged copy.",
        ),
        Node(
            package="rtk2026_mearm",
            executable="mearm_node",
            name="mearm_node",
            parameters=[
                config_path,
                {
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "block": ParameterValue(block, value_type=int),
                    "address": ParameterValue(address, value_type=int),
                    "vendor_path": vendor_path,
                },
            ],
            output="screen",
        ),
    ])
