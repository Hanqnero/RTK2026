"""Запуск Gazebo Sim, EKF, SLAM Toolbox и RViz для RTK2026."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Сформировать стек симуляции с настраиваемыми миром и RViz."""

    # ~ Основные аргументы запуска
    world = LaunchConfiguration("world")
    use_meshes = LaunchConfiguration("use_meshes")
    use_rviz = LaunchConfiguration("use_rviz")
    use_slam = LaunchConfiguration("use_slam")
    rviz_config = LaunchConfiguration("rviz_config")

    # ~ Пути к установленным ресурсам проекта
    sim_xacro = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_description"),
            "urdf",
            "rtk2026_diff_drive_sim.urdf.xacro",
        ]
    )
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_description"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # ~ Разворачиваем Xacro прямо при запуске.
    #
    # Полученная строка одновременно передаётся robot_state_publisher и
    # ros_gz_sim create через ROS-топик /robot_description
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                sim_xacro,
                " use_meshes:=",
                use_meshes,
            ]
        ),
        value_type=str,
    )

    # ~ Запускаем только сервер Gazebo Harmonic.
    #
    # Флаг -s отключает отдельное окно Gazebo. В Docker через noVNC будет
    # отображаться RViz, а физика и датчики продолжат работать в фоне.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": ["-r -s -v 3 ", world],
        }.items(),
    )

    # ~ Статическая часть TF-дерева и описание робота.
    #
    # robot_state_publisher создаёт ветви:
    # base_footprint -> base_link -> imu_link;
    #                              -> lidar_link -> lidar_frame;
    #                              -> camera_link -> camera_optical_frame.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    # ~ Создаём модель в уже запущенном мире.
    #
    # Колёса касаются пола при z=0. Небольшой запас по высоте позволяет
    # Gazebo спокойно разрешить первые контакты без проникновения геометрии.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rtk2026",
        output="screen",
        arguments=[
            "-name",
            "rtk2026",
            "-topic",
            "/robot_description",
            "-allow_renaming",
            "false",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.03",
        ],
    )

    # ~ Мост Gazebo Transport -> ROS 2.
    #
    # Символ [ означает одностороннее направление из Gazebo в ROS.
    # /clock нужен всем узлам с use_sim_time=true, /scan — SLAM или AMCL,
    # /imu/data — локальному EKF.
    # /ground_truth/odom нужен только диагностике и будущей оценке экспериментов;
    # ground_truth/tf намеренно не мостим, чтобы не создать второго издателя
    # odom -> base_footprint.
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            (
                "/ground_truth/odom@nav_msgs/msg/Odometry"
                "[gz.msgs.Odometry"
            ),
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ~ Публикация состояний вращающихся joints.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            controller_config,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # ~ Штатный дифференциальный привод.
    #
    # Внутренние топики контроллера приводим к общему API робота:
    #   /cmd_vel   : geometry_msgs/msg/TwistStamped;
    #   /wheel/odom: сырая nav_msgs/msg/Odometry по joint feedback.
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_diff_drive_controller",
        output="screen",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            controller_config,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--controller-ros-args",
            (
                "--ros-args "
                "--remap /diff_drive_controller/cmd_vel:=/cmd_vel "
                "--remap /diff_drive_controller/odom:=/wheel/odom"
            ),
        ],
    )

    # Плагин gz_ros2_control создаётся вместе с моделью. Небольшая задержка
    # не является условием синхронизации: spawner дополнительно ждёт сервисы
    # /controller_manager до 60 секунд.
    delayed_controller_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ],
    )

    # ~ Локальный EKF одометрии.
    #
    # Контроллер публикует /wheel/odom без TF. EKF фильтрует скорости,
    # публикует /odometry/filtered и является единственным владельцем
    # динамической трансформации odom -> base_footprint.
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rtk2026_localization"),
                    "launch",
                    "ekf.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # ~ Запуск slam_toolbox в режиме mapping.
    #
    # Конфиг остаётся владельцем пакета rtk2026_slam. Здесь меняется только
    # источник времени: все timestamps берутся из Gazebo /clock.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rtk2026_bringup"),
                    "launch",
                    "slam_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(use_slam),
    )

    # ~ RViz запускается в том же X-дисплее, который отдаёт noVNC.
    # Его можно отключить аргументом use_rviz:=false для headless-тестов.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rtk2026_description"),
                        "worlds",
                        "rtk2026_slam_world.sdf",
                    ]
                ),
                description="Абсолютный путь к Gazebo SDF world.",
            ),
            DeclareLaunchArgument(
                "use_meshes",
                default_value="false",
                description="Использовать STL вместо упрощённой визуальной геометрии.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Запустить RViz для локального X11/noVNC-дисплея.",
            ),
            DeclareLaunchArgument(
                "use_slam",
                default_value="true",
                description=(
                    "Запустить slam_toolbox. Укажите false перед запуском AMCL."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rtk2026_bringup"),
                        "rviz",
                        "rtk2026_sim_slam.rviz",
                    ]
                ),
                description="RViz-конфигурация для проверки SLAM.",
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            gazebo_bridge,
            delayed_controller_spawners,
            ekf,
            slam,
            rviz,
        ]
    )
