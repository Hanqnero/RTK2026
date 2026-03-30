# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Полный стек для Raspberry Pi: arduino_bridge + base_controller + лидар +
# RealSense (цвет + глубина + IMU) + SLAM + EKF + foxglove_bridge на порту 8765.
# Запуск: ros2 launch rtk2026_bringup pi_full.launch.py
# Опции:  use_slam:=false         — без SLAM
#         use_camera:=false       — без камеры
#         use_localization:=true  — включить EKF (odom + IMU)
#         camera_driver:=usb_cam  — вернуться на usb_cam
#         use_lane:=true          — запустить lane detection + control + sign detection

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam", default="true")
    use_camera = LaunchConfiguration("use_camera", default="true")
    use_localization = LaunchConfiguration("use_localization", default="true")
    use_fake_odom = LaunchConfiguration("use_fake_odom", default="false")
    camera_driver = LaunchConfiguration("camera_driver", default="realsense")
    use_lane = LaunchConfiguration("use_lane", default="false")
    lane_detector_mode = LaunchConfiguration("lane_detector_mode", default="lane")

    lidar_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_peripherals"), "launch", "lidar.launch.py"
    ])
    usb_camera_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_peripherals"), "launch", "depth_camera.launch.py"
    ])
    realsense_camera_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_peripherals"), "launch", "realsense_camera.launch.py"
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_slam"), "launch", "slam.launch.py"
    ])
    ekf_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_localization"), "launch", "ekf.launch.py"
    ])
    description_launch = PathJoinSubstitution([
        FindPackageShare("rtk2026_description"), "launch", "view_rtk2026.launch.py"
    ])
    driver_config = PathJoinSubstitution([
        FindPackageShare("rtk2026_driver"), "config", "arduino_bridge.yaml"
    ])
    base_config = PathJoinSubstitution([
        FindPackageShare("rtk2026_base"), "config", "base_controller.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_slam", default_value="true",
                              description="Запустить slam_toolbox (публикует /map)."),
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="Запустить камеру."),
        DeclareLaunchArgument("use_localization", default_value="true",
                              description="Запустить EKF (odom + IMU от RealSense)."),
        DeclareLaunchArgument("use_fake_odom", default_value="false",
                              description="Публиковать статичный odom→base_footprint (когда нет энкодеров)."),
        DeclareLaunchArgument("camera_driver", default_value="realsense",
                              description="Драйвер камеры: realsense или usb_cam."),
        DeclareLaunchArgument("use_lane", default_value="false",
                              description="Запустить lane detection + control_lane + sign_distance + lane_mission."),
        DeclareLaunchArgument("lane_detector_mode", default_value="lane",
                              description="Детектор для линии: lane или centerline."),

        # Описание робота (robot_state_publisher + joint_state_publisher, без RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={"use_rviz": "false"}.items(),
        ),

        # Arduino serial bridge — читает энкодеры, отправляет команды моторам
        Node(
            package="rtk2026_driver",
            executable="arduino_bridge",
            name="arduino_bridge",
            parameters=[driver_config],
            output="screen",
        ),

        # Base controller — одометрия, TF odom→base_link, подписан на /cmd_vel
        Node(
            package="rtk2026_base",
            executable="base_controller",
            name="base_controller",
            parameters=[base_config, {"publish_tf": True}],
            output="screen",
        ),

        # Лидар LD19 → /scan
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
        ),

        # usb_cam → /camera/image_raw (запасной вариант)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(usb_camera_launch),
            condition=IfCondition(PythonExpression([
                "'", use_camera, "' == 'true' and '", camera_driver, "' == 'usb_cam'"
            ])),
        ),

        # RealSense → /camera/color/*, /camera/depth/*, /camera/imu
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_camera_launch),
            condition=IfCondition(PythonExpression([
                "'", use_camera, "' == 'true' and '", camera_driver, "' == 'realsense'"
            ])),
        ),

        # Fake odom TF — статичный odom→base_footprint когда нет энкодеров
        # Отключить когда энкодеры подключены (use_fake_odom:=false)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fake_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            condition=IfCondition(use_fake_odom),
        ),

        # EKF — фьюжн odom + IMU (/camera/imu) → лучший TF odom→base_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            condition=IfCondition(use_localization),
        ),

        # SLAM Toolbox → /map (требует /scan и /tf)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(use_slam),
        ),

        # Foxglove Bridge — WebSocket :8765 для Foxglove Studio
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "send_buffer_limit": 10000000,
            }],
            output="screen",
        ),

        # === Lane following pipeline (use_lane:=true) ===

        # image_compensation → image_projection → /camera/image_projected
        Node(
            package="turtlebot3_autorace_camera",
            executable="image_compensation",
            name="image_compensation",
            namespace="camera",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("turtlebot3_autorace_camera"),
                "calibration", "extrinsic_calibration", "compensation.yaml")],
            remappings=[
                ("/camera/image_input",            "/camera/color/image_rect_color"),
                ("/camera/image_input/compressed",  "/camera/color/image_rect_color/compressed"),
                ("/camera/image_output",            "/camera/image_compensated"),
                ("/camera/image_output/compressed", "/camera/image_compensated/compressed"),
            ],
            condition=IfCondition(use_lane),
        ),
        Node(
            package="turtlebot3_autorace_camera",
            executable="image_projection",
            name="image_projection",
            namespace="camera",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("turtlebot3_autorace_camera"),
                "calibration", "extrinsic_calibration", "projection.yaml")],
            remappings=[
                ("/camera/image_input",            "/camera/image_compensated"),
                ("/camera/image_input/compressed",  "/camera/image_compensated/compressed"),
                ("/camera/image_output",            "/camera/image_projected"),
                ("/camera/image_output/compressed", "/camera/image_projected/compressed"),
            ],
            condition=IfCondition(use_lane),
        ),

        # detect_lane — классический режим по полосе между левой/правой границей
        Node(
            package="turtlebot3_autorace_detect",
            executable="detect_lane",
            name="detect_lane",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("turtlebot3_autorace_detect"),
                "param", "lane", "lane.yaml")],
            remappings=[
                ("/detect/image_input",             "/camera/image_projected"),
            ],
            condition=IfCondition(PythonExpression([
                "'", use_lane, "' == 'true' and '", lane_detector_mode, "' == 'lane'"
            ])),
        ),

        # detect_centerline — следование вдоль одной центральной линии
        Node(
            package="turtlebot3_autorace_detect",
            executable="detect_centerline",
            name="detect_centerline",
            output="screen",
            parameters=[{
                "is_detection_calibration_mode": False,
            }],
            remappings=[
                ("/detect/image_input", "/camera/image_projected"),
            ],
            condition=IfCondition(PythonExpression([
                "'", use_lane, "' == 'true' and '", lane_detector_mode, "' == 'centerline'"
            ])),
        ),

        # control_lane — PD регулятор → /control/cmd_vel
        Node(
            package="turtlebot3_autorace_mission",
            executable="control_lane",
            name="control_lane",
            output="screen",
            parameters=[{
                "Kp": 0.0025,
                "Kd": 0.007,
                "max_vel": 0.1,
                "lane_center_px": 500.0,
            }],
            remappings=[("/control/lane", "/detect/lane")],
            condition=IfCondition(use_lane),
        ),

        # sign_distance — глубина до знака через RealSense aligned depth
        Node(
            package="rtk2026_peripherals",
            executable="sign_distance",
            name="sign_distance",
            output="screen",
            condition=IfCondition(use_lane),
        ),

        # lane_mission — итоговый /cmd_vel с учётом знаков
        Node(
            package="rtk2026_peripherals",
            executable="lane_mission",
            name="lane_mission",
            output="screen",
            parameters=[{
                "behaviors_file": os.path.join(
                    get_package_share_directory("rtk2026_peripherals"),
                    "config", "sign_behaviors.yaml"),
                "trigger_distance_m": 0.1,
            }],
            condition=IfCondition(use_lane),
        ),
    ])
