#!/usr/bin/env python3
"""Nav2 Route Tool: map_server + static TF + global_costmap; RViz: карта + costmap + граф (rtk2026_route_editor.rviz)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = Path(get_package_share_directory("rtk2026_route_nav"))
    vector_objects_pkg = Path(get_package_share_directory("rtk2026_vector_objects"))
    costmap_params = pkg / "config" / "route_editor_global_costmap.yaml"
    route_params = pkg / "config" / "nav2_route_params.yaml"
    nav2_execution_params = pkg / "config" / "nav2_execution_params.yaml"
    lane_manager_params = pkg / "config" / "lane_decision_manager_v3.yaml"
    graph_file = pkg / "config" / "graph.geojson"
    rviz_cfg = pkg / "rviz" / "rtk2026_route_editor.rviz"

    yaml_file = LaunchConfiguration("yaml_filename")
    use_sim_time = LaunchConfiguration("use_sim_time")
    graph_filepath = LaunchConfiguration("graph_filepath")
    costmap_params_file = LaunchConfiguration("costmap_params_file")
    use_vector_server = LaunchConfiguration("use_vector_server")
    zones_path = LaunchConfiguration("zones_path")
    lane_params_file = LaunchConfiguration("lane_params_file")
    lane_manager_executable = LaunchConfiguration("lane_manager_executable")
    lane_pose_topic = LaunchConfiguration("lane_pose_topic")
    lane_current_vertex = LaunchConfiguration("lane_current_vertex")
    lane_previous_vertex = LaunchConfiguration("lane_previous_vertex")
    lane_detected_sign_target_vertex = LaunchConfiguration("lane_detected_sign_target_vertex")
    lane_direction_mode = LaunchConfiguration("lane_direction_mode")
    lane_tick_rate_hz = LaunchConfiguration("lane_tick_rate_hz")
    lane_log_every_n_ticks = LaunchConfiguration("lane_log_every_n_ticks")
    publish_map_odom_static_tf = LaunchConfiguration("publish_map_odom_static_tf")
    nav2_execution_params_file = LaunchConfiguration("nav2_execution_params_file")
    start_rviz = LaunchConfiguration("start_rviz")
    enable_lane_manager = LaunchConfiguration("enable_lane_manager")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "yaml_filename",
                default_value="/workspace/maps/rtk2026_arena.yaml",
                description="Путь к yaml карты (рядом pgm).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Для офлайн-редактора обычно false.",
            ),
            DeclareLaunchArgument(
                "graph_filepath",
                default_value=str(graph_file),
                description="Путь к graph.geojson для автозагрузки в route_server.",
            ),
            DeclareLaunchArgument(
                "costmap_params_file",
                default_value=str(costmap_params),
                description="Путь к YAML параметров costmap.",
            ),
            DeclareLaunchArgument(
                "use_vector_server",
                default_value="false",
                description="Поднять vector_object_server + costmap_filter_info_server + keepout tool.",
            ),
            DeclareLaunchArgument(
                "zones_path",
                default_value="",
                description=(
                    "Файл keepout-зон. Пусто - зоны живут только до выхода. "
                    "Для правки карты задайте путь рядом с ней, например "
                    "/workspace/maps/polygon_5x5.zones.json."
                ),
            ),
            DeclareLaunchArgument(
                "lane_params_file",
                default_value=str(lane_manager_params),
                description="Путь к YAML параметров lane_decision_manager.",
            ),
            DeclareLaunchArgument(
                "lane_pose_topic",
                default_value="/goal_pose",
                description="Топик PoseStamped для выбора активного ребра.",
            ),
            DeclareLaunchArgument(
                "lane_manager_executable",
                default_value="lane_decision_manager_v3",
                description="Имя executable для lane manager: lane_decision_manager_v3 (NavigateThroughPoses).",
            ),
            DeclareLaunchArgument(
                "lane_current_vertex",
                default_value="5",
                description="Текущая логическая вершина (временный параметр каркаса).",
            ),
            DeclareLaunchArgument(
                "lane_previous_vertex",
                default_value="-1",
                description="Предыдущая вершина (для фаз перекрёстка entry/exit). -1 если неизвестно.",
            ),
            DeclareLaunchArgument(
                "lane_detected_sign_target_vertex",
                default_value="-1",
                description="Целевая вершина от детекции знака; -1 если знак отсутствует.",
            ),
            DeclareLaunchArgument(
                "lane_direction_mode",
                default_value="lane1",
                description="Полоса для v3: lane1/lane2.",
            ),
            DeclareLaunchArgument(
                "lane_tick_rate_hz",
                default_value="2.0",
                description="Частота цикла lane_decision_manager.",
            ),
            DeclareLaunchArgument(
                "lane_log_every_n_ticks",
                default_value="1",
                description="Логировать состояние каждые N тиков.",
            ),
            DeclareLaunchArgument(
                "publish_map_odom_static_tf",
                default_value="false",
                description="Публиковать статический TF map->odom (для локализации по готовой карте должен быть false).",
            ),
            DeclareLaunchArgument(
                "nav2_execution_params_file",
                default_value=str(nav2_execution_params),
                description="Путь к YAML параметров planner/controller/bt_navigator.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Запускать RViz в route_editor контейнере.",
            ),
            DeclareLaunchArgument(
                "enable_lane_manager",
                default_value="true",
                description="Поднимать lane_decision_manager (алгоритм езды). Для дебага инициализации выключить.",
            ),
            Node(
                condition=IfCondition(publish_map_odom_static_tf),
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
            ),
            # Офлайн-редактору робота никто не двигает, но костмапе нужна
            # поза: без odom -> base_footprint дерево распадается на две
            # части, и она встаёт с "map and base_footprint are not part of
            # the same tree". В бою эту связь даёт EKF, здесь её некому
            # дать, поэтому она статическая и живёт под тем же флагом.
            Node(
                condition=IfCondition(publish_map_odom_static_tf),
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "odom",
                    "--child-frame-id", "base_footprint",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0.1", "--y", "0", "--z", "0.15885",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "rtk2026/base_footprint/lidar",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "base_link",
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"yaml_filename": yaml_file, "use_sim_time": use_sim_time},
                ],
            ),
            # Отложенный старт: при одновременном подъёме десятков Nav2-узлов в Docker rmw иногда
            # не успевает доставить ответ map_server на change_state — карта не активируется (/map пуст).
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": True},
                            {"node_names": ["map_server"]},
                            {"service_timeout": 30.0},
                            {"bond_timeout": 0.0},
                        ],
                    ),
                ],
            ),
            # Исполняемый файл всегда поднимает LifecycleNode с именем «costmap» (см. nav2 costmap_2d_node.cpp).
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                name="costmap",
                output="screen",
                # use_sim_time отдельно от файла: в YAML он свой, а режимом
                # управляет аргумент launch. Без этой строки костмапа
                # оставалась на sim-времени при выключенном Gazebo, часы
                # ноды стояли на нуле.
                parameters=[costmap_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_costmap",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    # bond id на ноде — «costmap», у LM в node_names — FQN; без отключения bond — таймаут.
                    {"bond_timeout": 0.0},
                    {
                        "node_names": [
                            "costmap",
                        ]
                    },
                ],
            ),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d_markers",
                name="costmap_markers",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # vector_object_server + costmap_filter_info_server + lifecycle_manager_vector +
            # keepout_click_tool: вынесены в rtk2026_vector_objects, самодостаточный launch.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(vector_objects_pkg / "launch" / "vector_objects.launch.py")
                ),
                condition=IfCondition(use_vector_server),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    # Без этого keepout_click_tool работает только в
                    # памяти: сервис ~/save пишет в zones_path, и при
                    # пустом значении размеченные зоны теряются при
                    # выходе. Для редактора это и есть результат работы.
                    "zones_path": zones_path,
                }.items(),
            ),
            Node(
                package="nav2_route",
                executable="route_server",
                name="route_server",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    str(route_params),
                    {
                        # route_server в текущем окружении иногда падает на /clock callback
                        # (segfault внутри rcl_set_ros_time_override). Держим его на wall-time.
                        "use_sim_time": False,
                        "graph_filepath": graph_filepath,
                    },
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[
                    nav2_execution_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    nav2_execution_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[
                    nav2_execution_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    nav2_execution_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_route",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["route_server"]},
                ],
            ),
            # В docker-compose.sim.yml Gazebo ждёт sleep 8 перед spawn_robot; до этого нет joint_states/odom
            # и TF odom→base_footprint разорван — controller_server падает при activate, bt_navigator остаётся inactive.
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        parameters=[
                            nav2_execution_params_file,
                            {"use_sim_time": use_sim_time},
                            {"service_timeout": 60.0},
                            {"bond_timeout": 0.0},
                        ],
                    ),
                ],
            ),
            Node(
                condition=IfCondition(enable_lane_manager),
                package="rtk2026_route_nav",
                executable=lane_manager_executable,
                name="lane_decision_manager",
                output="screen",
                parameters=[
                    lane_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "graph_filepath": graph_filepath,
                        "nav2_goal_topic": lane_pose_topic,
                        "current_vertex": lane_current_vertex,
                        "previous_vertex": lane_previous_vertex,
                        "detected_sign_target_vertex": lane_detected_sign_target_vertex,
                        "direction_mode": lane_direction_mode,
                        "tick_rate_hz": lane_tick_rate_hz,
                        "log_every_n_ticks": lane_log_every_n_ticks,
                    },
                ],
            ),
            Node(
                condition=IfCondition(start_rviz),
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(rviz_cfg)],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
