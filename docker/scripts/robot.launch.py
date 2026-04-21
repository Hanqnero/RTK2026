from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' /workspace/urdf/rtk2026_gazebo.urdf.xacro']),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        # Gazebo bridge publishes /scan in frame "rtk2026/base_footprint/lidar".
        # Add a static alias so RViz can transform LaserScan via existing lidar_link frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'rtk2026/base_footprint/lidar'],
        ),
    ])
