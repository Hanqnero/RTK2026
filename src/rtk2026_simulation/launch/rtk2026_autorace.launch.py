# Launch TurtleBot3 Autorace pipeline: lane keeping via PD regulator, sign detection, obstacle avoidance.
# Expects: Gazebo + diff_robot already running with /camera/image_raw, /scan, /odom.
#
# Data flow:
#   camera → image_relay_autorace (IPM) → detect_lane → /control/lane
#   → control_lane (PD + avoid mux) → /control/cmd_vel
#   → lane_mission (sign-based angular constraint filter) → /cmd_vel
#
# Args:
#   lane_calibration_mode:=true  — detect_lane in calibration mode (tune in rqt, save lane.yaml)
#   use_camera_calibration:=true — run turtlebot3_autorace_camera (intrinsic+extrinsic)
#   lane_image_source:=ipm|camera — bird's-eye topic for detect_lane
#   use_slam:=true               — run slam_toolbox for map building / odometry correction

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time          = LaunchConfiguration("use_sim_time",           default="true")
    mission               = LaunchConfiguration("mission",                default="construction")
    lane_calibration_mode = LaunchConfiguration("lane_calibration_mode",  default="false")
    use_camera_calibration= LaunchConfiguration("use_camera_calibration", default="false")
    lane_image_source     = LaunchConfiguration("lane_image_source",      default="ipm")
    use_slam              = LaunchConfiguration("use_slam",               default="false")
    camera_driver         = LaunchConfiguration("camera_driver",          default="realsense")

    ipm_projected_topic   = "/rtk_autorace_ipm/image_projected"
    ipm_compensated_topic = "/rtk_autorace_ipm/image_compensated"

    try:
        detect_share = get_package_share_directory("turtlebot3_autorace_detect")
        lane_param = os.path.join(detect_share, "param", "lane", "lane.yaml")
    except Exception:
        lane_param = None

    try:
        camera_share = get_package_share_directory("turtlebot3_autorace_camera")
        intrinsic_launch = os.path.join(camera_share, "launch", "intrinsic_camera_calibration.launch.py")
        extrinsic_launch = os.path.join(camera_share, "launch", "extrinsic_camera_calibration.launch.py")
    except Exception:
        camera_share = None
        intrinsic_launch = extrinsic_launch = None

    try:
        slam_share = get_package_share_directory("slam_toolbox")
        slam_launch = os.path.join(slam_share, "launch", "online_async_launch.py")
    except Exception:
        slam_share = None
        slam_launch = None

    # ------------------------------------------------------------------
    # IPM relay: /camera/image_raw → bird's-eye projected/compensated
    # ------------------------------------------------------------------
    # RealSense публикует /camera/color/image_raw и /camera/color/camera_info
    # Симулятор публикует /camera/image_raw и /camera/camera_info
    relay_node = Node(
        package="rtk2026_peripherals",
        executable="image_relay_autorace",
        name="image_relay_autorace",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"use_ipm": True},
            {"camera_height_m": 0.048},
            {"camera_pitch_rad": 0.3490},
            {"y_near_m": 0.05},
            {"y_far_m": 0.40},
            {"x_left_m": -0.22},
            {"x_right_m": 0.22},
            {"ipm_vertical_flip": False},
            {"projected_topic": ipm_projected_topic},
            {"compensated_topic": ipm_compensated_topic},
            {"image_topic": PythonExpression([
                "'/camera/color/image_raw' if '", camera_driver, "' == 'realsense' else '/camera/image_raw'"
            ])},
            {"camera_info_topic": PythonExpression([
                "'/camera/color/camera_info' if '", camera_driver, "' == 'realsense' else '/camera/camera_info'"
            ])},
        ],
    )

    # ------------------------------------------------------------------
    # Speed cap publisher (used by avoid_construction to cap speed)
    # ------------------------------------------------------------------
    max_vel_node = Node(
        package="rtk2026_peripherals",
        executable="autorace_max_vel_publisher",
        name="autorace_max_vel_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"max_vel": 0.032},
            {"publish_interval_sec": 0.5},
        ],
    )

    # ------------------------------------------------------------------
    # detect_lane: publishes /control/lane (lane center offset)
    # ------------------------------------------------------------------
    detect_lane_common_params = (
        [
            {"use_sim_time": use_sim_time, "is_detection_calibration_mode": lane_calibration_mode},
            lane_param,
        ]
        if lane_param
        else [{"use_sim_time": use_sim_time, "is_detection_calibration_mode": lane_calibration_mode}]
    )

    detect_lane_node_ipm = Node(
        package="turtlebot3_autorace_detect",
        executable="detect_lane",
        name="detect_lane",
        output="screen",
        parameters=detect_lane_common_params,
        remappings=[
            ("/detect/image_input",            ipm_projected_topic),
            ("/detect/image_input/compressed", ipm_projected_topic + "/compressed"),
            ("/detect/lane",                   "/control/lane"),
        ],
        condition=IfCondition(PythonExpression(["'", lane_image_source, "' != 'camera'"])),
    )

    detect_lane_node_camera = Node(
        package="turtlebot3_autorace_detect",
        executable="detect_lane",
        name="detect_lane",
        output="screen",
        parameters=detect_lane_common_params,
        remappings=[
            ("/detect/image_input",            "/camera/image_projected"),
            ("/detect/image_input/compressed", "/camera/image_projected/compressed"),
            ("/detect/lane",                   "/control/lane"),
        ],
        condition=IfCondition(PythonExpression(["'", lane_image_source, "' == 'camera'"])),
    )

    # ------------------------------------------------------------------
    # control_lane: PD regulator + avoid mux → /control/cmd_vel
    # NOTE: no remapping to /cmd_vel — lane_mission_node does that
    # ------------------------------------------------------------------
    control_lane_node = Node(
        package="turtlebot3_autorace_mission",
        executable="control_lane",
        name="control_lane",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        # control_lane publishes natively to /control/cmd_vel — consumed by lane_mission
        condition=UnlessCondition(lane_calibration_mode),
    )

    # ------------------------------------------------------------------
    # lane_mission: sign-based angular velocity constraint filter
    #   /detect/traffic_sign → clamp angular.z → /cmd_vel
    # ------------------------------------------------------------------
    lane_mission_node = Node(
        package="rtk2026_peripherals",
        executable="lane_mission",
        name="lane_mission",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"vote_window":    10},
            {"vote_threshold":  0.5},
            {"behaviors_file": PathJoinSubstitution([
                FindPackageShare("rtk2026_peripherals"), "config", "sign_behaviors.yaml"
            ])},
        ],
        condition=UnlessCondition(lane_calibration_mode),
    )

    # ------------------------------------------------------------------
    # detect_intersection_sign: publishes /detect/traffic_sign
    #   (UInt8: 1=intersection, 2=left, 3=right)
    # ------------------------------------------------------------------
    # detect_intersection_sign uses raw camera (not IPM) — signs are vertical,
    # bird's-eye warping destroys them. Raw forward view is correct input.
    raw_image_topic = PythonExpression([
        "'/camera/color/image_raw' if '", camera_driver, "' == 'realsense' else '/camera/image_raw'"
    ])
    raw_image_compressed_topic = PythonExpression([
        "'/camera/color/image_raw/compressed' if '", camera_driver, "' == 'realsense' else '/camera/image_raw/compressed'"
    ])
    detect_intersection_sign_node = Node(
        package="turtlebot3_autorace_detect",
        executable="detect_intersection_sign",
        name="detect_intersection_sign",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/detect/image_input",            raw_image_topic),
            ("/detect/image_input/compressed", raw_image_compressed_topic),
        ],
        condition=UnlessCondition(lane_calibration_mode),
    )

    # ------------------------------------------------------------------
    # avoid_construction: lidar obstacle avoidance → /avoid_control
    # Delayed 15 s: lane_state=1 at startup triggers false avoidance otherwise
    # ------------------------------------------------------------------
    avoid_construction_node = Node(
        package="turtlebot3_autorace_mission",
        executable="avoid_construction",
        name="avoid_construction",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"danger_distance": 0.12},
            {"danger_width": 0.06},
            {"speed": 0.02},
            {"robot_width_m": 0.05},
            {"robot_height_m": 0.08},
        ],
        remappings=[
            ("/camera/image_projected", ipm_projected_topic),
        ],
        condition=UnlessCondition(lane_calibration_mode),
    )

    # ------------------------------------------------------------------
    # detect_construction_sign (original mission sign, kept for compatibility)
    # ------------------------------------------------------------------
    # detect_construction_sign also uses raw camera — sign is a vertical board
    detect_construction_sign_node = Node(
        package="turtlebot3_autorace_detect",
        executable="detect_construction_sign",
        name="detect_construction_sign",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/detect/image_input",            raw_image_topic),
            ("/detect/image_input/compressed", raw_image_compressed_topic),
        ],
    )

    # ------------------------------------------------------------------
    # sign_distance: расстояние до знака по depth-камере RealSense
    # Публикует /detect/sign_distance (Float32, метры) при активном знаке
    # ------------------------------------------------------------------
    sign_distance_node = Node(
        package="rtk2026_peripherals",
        executable="sign_distance",
        name="sign_distance",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"roi_width": 120},
            {"roi_height": 80},
            {"min_depth_m": 0.1},
            {"max_depth_m": 5.0},
            {"sign_timeout_sec": 1.0},
        ],
        condition=UnlessCondition(lane_calibration_mode),
    )

    # ------------------------------------------------------------------
    # SLAM toolbox (optional, use_slam:=true)
    # Provides /map and corrected /tf odom→base_footprint
    # ------------------------------------------------------------------
    actions = [
        DeclareLaunchArgument("use_sim_time",           default_value="true",
                              description="Use sim time"),
        DeclareLaunchArgument("lane_calibration_mode",  default_value="false",
                              description="detect_lane calibration mode (tune in rqt)"),
        DeclareLaunchArgument("use_camera_calibration", default_value="false",
                              description="Use turtlebot3_autorace_camera for image_projected"),
        DeclareLaunchArgument("lane_image_source",      default_value="ipm",
                              description="Bird's-eye source for detect_lane: ipm or camera"),
        DeclareLaunchArgument("use_slam",               default_value="false",
                              description="Run slam_toolbox for map building + odometry correction"),
        DeclareLaunchArgument("mission",                default_value="construction",
                              description="Mission type for sign detection"),
        DeclareLaunchArgument("camera_driver",          default_value="realsense",
                              description="Camera driver: realsense или sim (usb_cam/gazebo)"),
        max_vel_node,
        relay_node,
    ]

    if intrinsic_launch and os.path.isfile(intrinsic_launch):
        actions.append(
            GroupAction(
                condition=IfCondition(use_camera_calibration),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(intrinsic_launch),
                        launch_arguments={"use_sim_time": use_sim_time}.items(),
                    ),
                ],
            ),
        )
    if extrinsic_launch and os.path.isfile(extrinsic_launch):
        actions.append(
            GroupAction(
                condition=IfCondition(use_camera_calibration),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(extrinsic_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "calibration_mode": lane_calibration_mode,
                        }.items(),
                    ),
                ],
            ),
        )

    if slam_launch and os.path.isfile(slam_launch):
        actions.append(
            GroupAction(
                condition=IfCondition(use_slam),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(slam_launch),
                        launch_arguments={"use_sim_time": use_sim_time}.items(),
                    ),
                ],
            ),
        )

    actions.extend([
        detect_lane_node_ipm,
        detect_lane_node_camera,
        control_lane_node,
        lane_mission_node,
        detect_intersection_sign_node,
        # Delay avoid_construction 15s: lane_state=1 at startup triggers false avoidance
        TimerAction(period=15.0, actions=[avoid_construction_node]),
        detect_construction_sign_node,
        sign_distance_node,
    ])

    return LaunchDescription(actions)
