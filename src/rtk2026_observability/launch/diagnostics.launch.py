"""Запуск стандартной системной диагностики RTK2026."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Создать системные monitor-ноды и diagnostic aggregator."""

    use_gui = LaunchConfiguration("use_gui")
    diagnostics_display = LaunchConfiguration("diagnostics_display")
    localization_mode = LaunchConfiguration("localization_mode")
    slam_backend = LaunchConfiguration("slam_backend")
    records_path = LaunchConfiguration("records_path")
    use_city_nav = LaunchConfiguration("use_city_nav")
    use_nav2 = LaunchConfiguration("use_nav2")

    aggregator_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "diagnostic_aggregator.yaml",
        ]
    )
    topic_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "topics_sim.yaml",
        ]
    )
    localization_topic_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "topics_localization_sim.yaml",
        ]
    )
    lidar_topic_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "topics_lidar_slam.yaml",
        ]
    )
    visual_topic_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "topics_visual_slam.yaml",
        ]
    )
    node_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "nodes_sim.yaml",
        ]
    )
    localization_node_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "nodes_localization_sim.yaml",
        ]
    )
    lidar_node_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "nodes_lidar_slam.yaml",
        ]
    )
    visual_node_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "nodes_visual_slam.yaml",
        ]
    )
    tf_time_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "tf_time_sim.yaml",
        ]
    )
    sensor_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "sensors_sim.yaml",
        ]
    )
    localization_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "localization_sim.yaml",
        ]
    )
    city_nav_topic_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "topics_city_nav.yaml",
        ]
    )
    city_nav_node_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "nodes_city_nav.yaml",
        ]
    )
    nav2_topic_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "topics_nav2.yaml",
        ]
    )
    nav2_node_monitor_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "config",
            "nodes_nav2.yaml",
        ]
    )

    # ~ Загрузка процессора контейнера.
    cpu_monitor = Node(
        package="diagnostic_common_diagnostics",
        executable="cpu_monitor.py",
        output="screen",
        parameters=[
            {
                # Предупреждение, если хотя бы одно ядро превышает 90%.
                "warning_percentage": 90,

                # Усреднение последних пяти измерений.
                "window": 5,

                # Системные метрики измеряются по wall time.
                "use_sim_time": False,
            }
        ],
    )

    # ~ Использование оперативной памяти контейнера.
    ram_monitor = Node(
        package="diagnostic_common_diagnostics",
        executable="ram_monitor.py",
        output="screen",
        parameters=[
            {
                "warning_percentage": 90,
                "window": 5,
                "use_sim_time": False,
            }
        ],
    )

    # ~ Свободное место в каталоге записей.
    hd_monitor = Node(
        package="diagnostic_common_diagnostics",
        executable="hd_monitor.py",
        name="records_disk_monitor",
        output="screen",
        parameters=[
            {
                "path": records_path,

                # WARN при остатке менее 10%.
                "free_percent_low": 10,

                # ERROR при остатке менее 3%.
                "free_percent_crit": 3,

                "use_sim_time": False,
            }
        ],
    )

    # ~ Объединение отдельных DiagnosticStatus в дерево RTK2026.
    aggregator = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[
            aggregator_config,
            {"use_sim_time": False},
        ],
    )
    # ~ Общие topics робота не зависят от выбранного SLAM backend.
    topic_monitor = Node(
        package="rtk2026_observability",
        executable="topic_monitor.py",
        name="topic_monitor",
        output="screen",
        parameters=[
            {
                "config_file": topic_monitor_config,

                # Возраст получения измеряется по wall time. Timestamp источника
                # используется для частоты и отдельно не будит Python на /clock.
                "use_sim_time": False,

                # Период публикации /diagnostics.
                "diagnostic_updater.period": 1.0,
            }
        ],
    )

    # ~ Topics SLAM Toolbox: карта и событийная pose после scan matching.
    lidar_topic_monitor = Node(
        package="rtk2026_observability",
        executable="topic_monitor.py",
        name="slam_topic_monitor",
        output="screen",
        parameters=[
            {
                "config_file": lidar_topic_monitor_config,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    localization_mode,
                    "' == 'mapping' and '",
                    slam_backend,
                    "' == 'lidar'",
                ]
            )
        ),
    )

    # ~ RGB-D входы и карта визуального SLAM RTAB-Map.
    visual_topic_monitor = Node(
        package="rtk2026_observability",
        executable="topic_monitor.py",
        name="slam_topic_monitor",
        output="screen",
        parameters=[
            {
                "config_file": visual_topic_monitor_config,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    localization_mode,
                    "' == 'mapping' and '",
                    slam_backend,
                    "' == 'visual'",
                ]
            )
        ),
    )

    # ~ В AMCL-режиме карта и оценки позы являются событийными.
    localization_topic_monitor = Node(
        package="rtk2026_observability",
        executable="topic_monitor.py",
        name="localization_topic_monitor",
        output="screen",
        parameters=[
            {
                "config_file": localization_topic_monitor_config,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", localization_mode, "' == 'localization'"])
        ),
    )

    # ~ Общие ноды tracked-модели и диагностического backend.
    node_monitor = Node(
        package="rtk2026_observability",
        executable="node_monitor.py",
        name="node_monitor",
        output="screen",
        parameters=[
            {
                "config_file": node_monitor_config,

                # ROS graph и возраст логов проверяются по wall time.
                "use_sim_time": False,
                "graph_check_period": 1.0,
                "diagnostic_updater.period": 1.0,
            }
        ],
    )

    # ~ Lifecycle, сервисы и логи SLAM Toolbox.
    lidar_node_monitor = Node(
        package="rtk2026_observability",
        executable="node_monitor.py",
        name="slam_node_monitor",
        output="screen",
        parameters=[
            {
                "config_file": lidar_node_monitor_config,
                "use_sim_time": False,
                "graph_check_period": 1.0,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    localization_mode,
                    "' == 'mapping' and '",
                    slam_backend,
                    "' == 'lidar'",
                ]
            )
        ),
    )

    # ~ ROS graph и логи визуального SLAM RTAB-Map.
    visual_node_monitor = Node(
        package="rtk2026_observability",
        executable="node_monitor.py",
        name="slam_node_monitor",
        output="screen",
        parameters=[
            {
                "config_file": visual_node_monitor_config,
                "use_sim_time": False,
                "graph_check_period": 1.0,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    localization_mode,
                    "' == 'mapping' and '",
                    slam_backend,
                    "' == 'visual'",
                ]
            )
        ),
    )

    # ~ В режиме известной карты вместо SLAM обязательны Map Server и AMCL.
    localization_node_monitor = Node(
        package="rtk2026_observability",
        executable="node_monitor.py",
        name="localization_node_monitor",
        output="screen",
        parameters=[
            {
                "config_file": localization_node_monitor_config,
                "use_sim_time": False,
                "graph_check_period": 1.0,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", localization_mode, "' == 'localization'"])
        ),
    )

    # ~ Целостность TF-дерева и согласованность timestamp потоков.
    tf_time_monitor = Node(
        package="rtk2026_observability",
        executable="tf_time_monitor.py",
        name="tf_time_monitor",
        output="screen",
        parameters=[
            {
                "config_file": tf_time_monitor_config,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
    )

    # ~ Значения LaserScan, IMU и общей /wheel/odom.
    sensor_monitor = Node(
        package="rtk2026_observability",
        executable="sensor_monitor.py",
        name="sensor_monitor",
        output="screen",
        parameters=[
            {
                "config_file": sensor_monitor_config,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
    )

    # ~ Позы участков, детекции знаков и существование сервера Nav2.
    city_nav_topic_monitor = Node(
        package="rtk2026_observability",
        executable="topic_monitor.py",
        name="city_nav_topic_monitor",
        output="screen",
        parameters=[
            {
                "config_file": city_nav_topic_monitor_config,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(use_city_nav),
    )

    # ~ Присутствие city_nav и lifecycle стека навигации.
    city_nav_node_monitor = Node(
        package="rtk2026_observability",
        executable="node_monitor.py",
        name="city_nav_node_monitor",
        output="screen",
        parameters=[
            {
                "config_file": city_nav_node_monitor_config,
                "use_sim_time": False,
                "graph_check_period": 1.0,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(use_city_nav),
    )

    # ~ Есть ли кому исполнять цели и доходит ли исполнение до базы.
    nav2_topic_monitor = Node(
        package="rtk2026_observability",
        executable="topic_monitor.py",
        name="nav2_topic_monitor",
        output="screen",
        parameters=[
            {
                "config_file": nav2_topic_monitor_config,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(use_nav2),
    )

    # ~ Lifecycle серверов Nav2: без active стек цели не примет.
    nav2_node_monitor = Node(
        package="rtk2026_observability",
        executable="node_monitor.py",
        name="nav2_node_monitor",
        output="screen",
        parameters=[
            {
                "config_file": nav2_node_monitor_config,
                "use_sim_time": False,
                "graph_check_period": 1.0,
                "diagnostic_updater.period": 1.0,
            }
        ],
        condition=IfCondition(use_nav2),
    )

    # ~ Ковариации, согласованность EKF и состояние SLAM/AMCL.
    localization_monitor = Node(
        package="rtk2026_observability",
        executable="localization_monitor.py",
        name="localization_monitor",
        output="screen",
        parameters=[
            {
                "config_file": localization_monitor_config,
                "mode": localization_mode,
                "use_sim_time": False,
                "diagnostic_updater.period": 1.0,
            }
        ],
    )
    # ~ Неагрегированные статусы /diagnostics.
    runtime_monitor = Node(
        package="rqt_runtime_monitor",
        executable="rqt_runtime_monitor",
        name="diagnostics_runtime_monitor",
        output="screen",
        condition=IfCondition(use_gui),
        additional_env={"DISPLAY": diagnostics_display},
    )

    # ~ Иерархическое дерево /diagnostics_agg.
    robot_monitor = Node(
        package="rqt_robot_monitor",
        executable="rqt_robot_monitor",
        name="diagnostics_robot_monitor",
        output="screen",
        condition=IfCondition(use_gui),
        additional_env={"DISPLAY": diagnostics_display},
    )

    # ~ Графики исходных ROS topics и последующая загрузка rosbag2.
    plotjuggler = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="diagnostics_plotjuggler",
        output="screen",
        condition=IfCondition(use_gui),
        additional_env={"DISPLAY": diagnostics_display},
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="false",
                description="Запустить PlotJuggler и RQt на втором noVNC desktop.",
            ),
            DeclareLaunchArgument(
                "diagnostics_display",
                default_value=EnvironmentVariable(
                    "DIAGNOSTICS_DISPLAY",
                    default_value=":2",
                ),
                description="X11 DISPLAY диагностического noVNC desktop.",
            ),
            DeclareLaunchArgument(
                "localization_mode",
                default_value="mapping",
                description="Режим обязательных проверок: mapping или localization.",
            ),
            DeclareLaunchArgument(
                "slam_backend",
                default_value="visual",
                description="SLAM backend для mapping: lidar или visual.",
            ),
            DeclareLaunchArgument(
                "records_path",
                default_value="/workspace/records",
                description="Каталог записей для проверки свободного места.",
            ),
            DeclareLaunchArgument(
                "use_city_nav",
                default_value="false",
                description="Диагностировать ноду движения по городу.",
            ),
            DeclareLaunchArgument(
                "use_nav2",
                default_value="false",
                description="Диагностировать стек Nav2: lifecycle, цели, cmd_vel.",
            ),
            cpu_monitor,
            ram_monitor,
            hd_monitor,
            aggregator,
            topic_monitor,
            lidar_topic_monitor,
            visual_topic_monitor,
            localization_topic_monitor,
            node_monitor,
            lidar_node_monitor,
            visual_node_monitor,
            localization_node_monitor,
            tf_time_monitor,
            sensor_monitor,
            city_nav_topic_monitor,
            city_nav_node_monitor,
            nav2_topic_monitor,
            nav2_node_monitor,
            localization_monitor,
            runtime_monitor,
            robot_monitor,
            plotjuggler,
        ]
    )
