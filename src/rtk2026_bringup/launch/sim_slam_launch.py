"""Запуск Gazebo Sim, EKF, выбранного SLAM и RViz для RTK2026."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _robot_state_publisher(
    robot_description: ParameterValue,
    condition: IfCondition,
) -> Node:
    """Создать общий robot_state_publisher для выбранной модели."""

    return Node(
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
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    """Сформировать единый параметризованный стек симуляции."""

    # ~ Основные аргументы запуска
    robot_model = LaunchConfiguration("robot_model")
    world = LaunchConfiguration("world")
    use_meshes = LaunchConfiguration("use_meshes")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    slam_mode = LaunchConfiguration("slam_mode")
    rtabmap_config = LaunchConfiguration("rtabmap_config")
    rtabmap_database = LaunchConfiguration("rtabmap_database")
    rtabmap_localization = LaunchConfiguration("rtabmap_localization")
    rtabmap_args = LaunchConfiguration("rtabmap_args")
    rviz_config = LaunchConfiguration("rviz_config")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    diff_drive_condition = IfCondition(
        EqualsSubstitution(
            robot_model,
            "diff_drive",
        )
    )
    tracked_condition = IfCondition(
        EqualsSubstitution(
            robot_model,
            "tracked",
        )
    )
    lidar_slam_condition = IfCondition(
        EqualsSubstitution(
            slam_mode,
            "lidar",
        )
    )
    visual_slam_condition = IfCondition(
        EqualsSubstitution(
            slam_mode,
            "visual",
        )
    )

    # Gazebo-модели принадлежат rtk2026_description и устанавливаются вместе
    # с пакетом. Поэтому launch не зависит от исходного каталога /workspace.
    configure_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution(
                [
                    FindPackageShare("rtk2026_description"),
                    "worlds",
                    "electrolysis_world_modular",
                    "models",
                ]
            ),
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )
    # ~ Пути к установленным ресурсам проекта
    diff_drive_xacro = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_description"),
            "urdf",
            "rtk2026_diff_drive_sim.urdf.xacro",
        ]
    )
    tracked_xacro = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_tracked_sim"),
            "urdf",
            "rtk2026_tracked.urdf.xacro",
        ]
    )
    tracked_sdf = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_tracked_sim"),
            "models",
            "rtk2026_tracked",
            "model.sdf",
        ]
    )
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_description"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rtabmap_settings = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_slam"),
            "config",
            "rtabmap_rgbd.ini",
        ]
    )
    gazebo_bridge_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_bringup"),
            "config",
            "gazebo_bridge.yaml",
        ]
    )

    # ~ Разворачиваем оба Xacro прямо при запуске.
    #
    # Условия ниже запускают только один robot_state_publisher. Для колёсной
    # модели то же описание передаётся Gazebo через /robot_description;
    # у гусеничной модели физика находится в отдельном SDF.
    diff_drive_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                diff_drive_xacro,
                " use_meshes:=",
                use_meshes,
            ]
        ),
        value_type=str,
    )
    tracked_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                tracked_xacro,
            ]
        ),
        value_type=str,
    )

    # ~ Один запуск Gazebo Harmonic для GUI и headless-режима.
    #
    # При use_gazebo_gui=false стандартный флаг -s отключает графический
    # клиент. Остальные параметры запуска и world в обоих режимах одинаковы.
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
            "gz_args": [
                "-r ",
                PythonExpression(
                    [
                        "'' if '",
                        use_gazebo_gui,
                        "'.lower() in ('true', '1', 'yes') else '-s '",
                    ]
                ),
                "-v 3 ",
                world,
            ],
        }.items(),
    )

    # ~ Статическая часть TF-дерева и описание выбранной модели.
    #
    # Обе модели создают одинаковые ветви:
    # base_footprint -> base_link -> imu_link;
    #                              -> lidar_link -> lidar_frame;
    #                              -> camera_link;
    #                                 -> camera_color_optical_frame;
    #                                 -> camera_depth_optical_frame;
    #                                 -> camera_accel/gyro_optical_frame.
    diff_drive_state_publisher = _robot_state_publisher(
        diff_drive_description,
        diff_drive_condition,
    )
    tracked_state_publisher = _robot_state_publisher(
        tracked_description,
        tracked_condition,
    )

    # ~ Создаём колёсную модель из /robot_description.
    #
    # Колёса касаются пола при z=0. Небольшой запас по высоте позволяет
    # Gazebo спокойно разрешить первые контакты без проникновения геометрии.
    spawn_diff_drive = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rtk2026_diff_drive",
        output="screen",
        arguments=[
            "-name",
            "rtk2026",
            "-topic",
            "/robot_description",
            "-allow_renaming",
            "false",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
        condition=diff_drive_condition,
    )

    # ~ Создаём гусеничную модель из SDF.
    #
    # SDF используется только там, где нужны TrackController и контактные
    # поверхности. TF и визуализацию RViz по-прежнему даёт Xacro выше.
    spawn_tracked = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rtk2026_tracked",
        output="screen",
        arguments=[
            "-name",
            "rtk2026_tracked",
            "-file",
            tracked_sdf,
            "-allow_renaming",
            "false",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
        condition=tracked_condition,
    )

    # ~ Общий мост Gazebo Transport -> ROS 2.
    #
    # Полный список и направления находятся в gazebo_bridge.yaml. Это штатный
    # YAML-интерфейс ros_gz_bridge; launch только передаёт путь к конфигу.
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        parameters=[
            {
                "config_file": gazebo_bridge_config,
                "use_sim_time": True,
            }
        ],
    )

    # ~ Дополнительный мост привода нужен только гусеничной SDF-модели.
    #
    # Колёсная модель получает /cmd_vel и публикует /wheel/odom напрямую
    # через ros2_control. TrackedVehicle работает в Gazebo Transport, поэтому
    # для него мостятся только эти два интерфейса.
    tracked_drive_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tracked_drive_bridge",
        output="screen",
        arguments=[
            (
                "/cmd_vel@geometry_msgs/msg/TwistStamped"
                "]gz.msgs.Twist"
            ),
            (
                "/wheel/odom@nav_msgs/msg/Odometry"
                "[gz.msgs.Odometry"
            ),
        ],
        parameters=[{"use_sim_time": True}],
        condition=tracked_condition,
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
        condition=diff_drive_condition,
    )

    # ~ Локальный EKF одометрии.
    #
    # EKF объединяет продольную скорость /wheel/odom с angular_velocity.z
    # основной IMU. Для diff_drive /wheel/odom рассчитывается штатным
    # diff_drive_controller по position state motor joints. У tracked-модели
    # этот же интерфейс пока предоставляет Gazebo TrackedVehicle, поскольку
    # её контактные борта не являются вращающимися joints.
    #
    # /ground_truth/odom в фильтр не подаётся и остаётся только эталоном
    # эксперимента. EKF публикует /odometry/filtered и является единственным
    # владельцем динамической трансформации odom -> base_footprint.
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
        condition=lidar_slam_condition,
    )

    # ~ RTAB-Map с внешней колёсно-инерциальной одометрией.
    #
    # Локальную одометрию продолжает рассчитывать EKF:
    #   /wheel/odom + /imu/data -> /odometry/filtered.
    #
    # RTAB-Map не создаёт вторую visual odometry, а использует RGB-D для
    # построения карты, распознавания мест и глобальной коррекции map -> odom.
    # Лидар и /scan в этом режиме не используются.
    visual_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rtabmap_launch"),
                    "launch",
                    "rtabmap.launch.py",
                ]
            )
        ),
        launch_arguments={
            # Все timestamps должны использовать Gazebo /clock.
            "use_sim_time": "true",

            # Основные frames единственного TF-дерева.
            "frame_id": "base_footprint",
            "map_frame_id": "map",
            "odom_frame_id": "",

            # EKF является источником непрерывной локальной одометрии.
            "odom_topic": "/odometry/filtered",
            "visual_odometry": "false",
            "icp_odometry": "false",
            "publish_tf_odom": "false",

            # RTAB-Map является единственным владельцем map -> odom.
            "publish_tf_map": "true",
            # Абсолютное имя совпадает с Map display в общем RViz-конфиге.
            # Без него namespace /rtabmap превращает выход в /rtabmap/map,
            # поэтому RTAB-Map не видит подписчика /map и не собирает grid.
            "map_topic": "/map",

            # RGB-D потоки симуляционной RealSense D435i.
            "rgb_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/color/camera_info",

            # RTAB-Map синхронизирует исходные RGB и depth сообщения.
            "depth": "true",
            "subscribe_rgbd": "false",
            "rgbd_sync": "false",
            "approx_sync": "true",
            "approx_sync_max_interval": "0.05",
            "topic_queue_size": "20",
            "sync_queue_size": "10",
            "qos": "2",

            # Лидар не участвует ни в регистрации, ни в occupancy grid.
            "subscribe_scan": "false",
            "subscribe_scan_cloud": "false",

            # /imu/data уже входит в /odometry/filtered.
            #
            # RTAB-Map принимает только готовую orientation из Imu, тогда как
            # наш EKF использует gyro Z. Поэтому не добавляем симуляционную
            # абсолютную ориентацию в граф повторно.
            "imu_topic": "/rtabmap/unused_imu",

            # Параметры алгоритма и сохраняемая база данных.
            "cfg": rtabmap_config,
            "database_path": rtabmap_database,
            "localization": rtabmap_localization,
            "rtabmap_args": rtabmap_args,

            # Используем существующий RViz из bringup.
            "rtabmap_viz": "false",
            "rviz": "false",
        }.items(),
        condition=visual_slam_condition,
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
                "robot_model",
                default_value="tracked",
                choices=[
                    "tracked",
                    "diff_drive",
                ],
                description=(
                    "Модель ходовой части: tracked или diff_drive."
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rtk2026_description"),
                        "worlds",
                        "electrolysis_world_modular",
                        "worlds",
                        "electrolysis.sdf",
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
                "use_gazebo_gui",
                default_value="false",
                description="Запустить графический клиент Gazebo.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Запустить RViz для локального X11/noVNC-дисплея.",
            ),
            DeclareLaunchArgument(
                "slam_mode",
                default_value="visual",
                choices=[
                    "lidar",
                    "visual",
                    "none",
                ],
                description=(
                    "SLAM backend: lidar=slam_toolbox, "
                    "visual=RTAB-Map RGB-D с внешней EKF-одометрией, "
                    "none=без SLAM."
                ),
            ),
            DeclareLaunchArgument(
                "rtabmap_config",
                default_value=rtabmap_settings,
                description="INI-конфигурация RGB-D RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "rtabmap_database",
                default_value="/workspace/records/rtabmap/rtk2026.db",
                description="База графа и данных RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "rtabmap_localization",
                default_value="false",
                description=(
                    "false — строить и дополнять карту; "
                    "true — локализоваться по существующей базе."
                ),
            ),
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value="",
                description=(
                    "Дополнительные аргументы RTAB-Map. "
                    "Для новой базы: --delete_db_on_start."
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
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.5",
                description="Начальная координата X модели, м.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="0.5",
                description="Начальная координата Y модели, м.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.081",
                description=(
                    "Начальная координата Z base_footprint, м. "
                    "Для платформы высотой 80 мм оставлен зазор 1 мм."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="1.57079632679",
                description=(
                    "Начальный поворот модели вокруг Z, рад. "
                    "Для движения вдоль прохода к задней стене: pi/2."
                ),
            ),
            configure_gz_resource_path,
            gazebo,
            diff_drive_state_publisher,
            tracked_state_publisher,
            spawn_diff_drive,
            spawn_tracked,
            gazebo_bridge,
            tracked_drive_bridge,
            delayed_controller_spawners,
            ekf,
            slam,
            visual_slam,
            rviz,
        ]
    )
