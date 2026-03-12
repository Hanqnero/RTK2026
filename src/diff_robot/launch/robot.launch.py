import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # File paths for Docker image layout: package at /workspace/src/diff_robot
    pkg_share = "/workspace/src/diff_robot"
    urdf_file_path = os.path.join(pkg_share, "urdf", "diff_robot.urdf")
    rviz_config_file_path = os.path.join(pkg_share, "urdf", "rviz.rviz")
    world_file_default = os.path.join(pkg_share, "world", "silverstone_track.world")

    # Прочитать URDF робота и отдать его robot_state_publisher (для отображения в RViz)
    try:
        with open(urdf_file_path, "r") as f:
            robot_description_xml = f.read()
    except Exception:
        robot_description_xml = ""

    robot_description = {"robot_description": robot_description_xml}

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file_path,
            description='Absolute path to robot URDF file'),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_file_path,
            description='Absolute path to RViz config file'),
        DeclareLaunchArgument(
            name='world',
            default_value=world_file_default,
            description='Absolute path to world file'),
        DeclareLaunchArgument(
            name='x', default_value='0.0', description='Initial x position of the robot'),
        DeclareLaunchArgument(
            name='y', default_value='0.0', description='Initial y position of the robot'),
        DeclareLaunchArgument(
            name='z', default_value='0.15', description='Initial z position of the robot'),
        DeclareLaunchArgument(
            name='R', default_value='0.0', description='Initial roll orientation of the robot'),
        DeclareLaunchArgument(
            name='P', default_value='0.0', description='Initial pitch orientation of the robot'),
        DeclareLaunchArgument(
            name='Y', default_value='0.0', description='Initial yaw orientation of the robot'),

        # Публикация TF по URDF (RobotModel в RViz)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # Gazebo launch
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
            output='screen'),

        # Spawn the robot after a longer delay so Gazebo (especially heavy worlds like small_city) is fully up
        TimerAction(
            period=20.0,
            actions=[
                Node(package='gazebo_ros', executable='spawn_entity.py',
                     arguments=[
                         '-file', LaunchConfiguration('model'),
                         '-entity', 'diff_robot',
                         # Дольше ждём сервис /spawn_entity для тяжёлых миров (small_city)
                         '-timeout', '120.0',
                         '-x', LaunchConfiguration('x'),
                         '-y', LaunchConfiguration('y'),
                         '-z', LaunchConfiguration('z'),
                         '-R', LaunchConfiguration('R'),
                         '-P', LaunchConfiguration('P'),
                         '-Y', LaunchConfiguration('Y')],
                     output='screen'),
            ]),

        # Launch RViz
        Node(package='rviz2', executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', LaunchConfiguration('rvizconfig')]),
    ])
