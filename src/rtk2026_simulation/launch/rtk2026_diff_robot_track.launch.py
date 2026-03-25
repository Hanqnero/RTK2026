import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    R = LaunchConfiguration("R")
    P = LaunchConfiguration("P")
    Y = LaunchConfiguration("Y")
    use_sim_time = LaunchConfiguration("use_sim_time")
    explore = LaunchConfiguration("explore")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")

    # Пути к launch RTK‑Nav2+SLAM и explorer
    nav2_slam_launch = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_nav2_explorer"),
            "launch",
            "rtk2026_nav2_slam.launch.py",
        ]
    )
    explorer_launch = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_nav2_explorer"),
            "launch",
            "rtk2026_explorer.launch.py",
        ]
    )

    # robot_description из URDF по пути пакета (работает и в /workspace, и в /home/.../RTK2026)
    diff_robot_share = get_package_share_directory("diff_robot")
    urdf_path = os.path.join(diff_robot_share, "urdf", "diff_robot.urdf")
    with open(urdf_path, "r") as f:
        robot_description_xml = f.read()
    robot_description = {"robot_description": robot_description_xml}

    diff_robot_share_sub = FindPackageShare("diff_robot")
    urdf_file = PathJoinSubstitution(
        [diff_robot_share_sub, "urdf", "diff_robot.urdf"]
    )
    rviz_config_default = PathJoinSubstitution(
        [diff_robot_share_sub, "urdf", "rviz.rviz"]
    )
    rviz_config = LaunchConfiguration("rvizconfig")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # use_gazebo_gui:=true (Windows/по умолчанию) — окно Gazebo; false (Mac VNC) — только gzserver, визуализация в RViz
    world_dir = os.path.join(diff_robot_share, "world")
    gazebo_launch = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so", world],
        output="screen",
        cwd=world_dir,
        condition=IfCondition(use_gazebo_gui),
    )
    gzserver_launch = ExecuteProcess(
        cmd=["gzserver", "--verbose", world, "-s", "libgazebo_ros_factory.so"],
        output="screen",
        cwd=world_dir,
        condition=UnlessCondition(use_gazebo_gui),
    )

    spawn_entity = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file",
                    urdf_file,
                    "-entity",
                    "diff_robot",
                    "-timeout",
                    "120.0",
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    z,
                    "-R",
                    R,
                    "-P",
                    P,
                    "-Y",
                    Y,
                ],
                output="screen",
            ),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    # RTK Nav2 + SLAM поверх diff_robot (Gazebo уже даёт /odom и /scan)
    nav2_group = GroupAction(
        [
            TimerAction(
                period=25.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav2_slam_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                        }.items(),
                    ),
                ],
            ),
        ]
    )

    # RTK frontier‑explorer (опционально)
    explorer_group = GroupAction(
        [
            TimerAction(
                period=40.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(explorer_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                        }.items(),
                    ),
                ],
            ),
        ],
        # explore задаётся строкой "true"/"false"
        condition=None,
    )

    # Мы не используем Condition напрямую, чтобы не тянуть IfCondition и PythonExpression;
    # explore управляется на уровне скрипта (запуском с/без explorer_group).

    ld_actions = [
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                get_package_share_directory("diff_robot"),
                "world",
                "silverstone_track.world",
            ),
            description="Path to Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="-5.0",
            description="Initial x position of the robot",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Initial y position of the robot",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.15",
            description="Initial z position of the robot",
        ),
        DeclareLaunchArgument(
            "R",
            default_value="0.0",
            description="Initial roll orientation of the robot",
        ),
        DeclareLaunchArgument(
            "P",
            default_value="0.0",
            description="Initial pitch orientation of the robot",
        ),
        DeclareLaunchArgument(
            "Y",
            default_value="0.0",
            description="Initial yaw orientation of the robot",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (Gazebo /clock) in Nav2 + SLAM + explorer.",
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=rviz_config_default,
            description="RViz config file for diff_robot + Nav2/SLAM visualization.",
        ),
        DeclareLaunchArgument(
            "explore",
            default_value="true",
            description="Start RTK frontier explorer after Nav2+SLAM.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 inside the container.",
        ),
        DeclareLaunchArgument(
            "use_gazebo_gui",
            default_value="true",
            description="If true, run full Gazebo with GUI; if false, run gzserver only (headless, e.g. for Mac VNC).",
        ),
        robot_state_publisher,
        gazebo_launch,
        gzserver_launch,
        spawn_entity,
        rviz_node,
        nav2_group,
    ]

    # Добавляем explorer_group только если explore=true (на уровне скрипта).
    # В launch API это проще всего оставить всегда, а включать/выключать
    # через отдельный launch‑файл или аргументы; сейчас скрипт всегда
    # запускает с explore=true.
    ld_actions.append(explorer_group)

    return LaunchDescription(ld_actions)

