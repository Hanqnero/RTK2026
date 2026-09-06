#!/usr/bin/env python3
"""Запуск стека Nav2.

Четыре сервера и менеджер жизненного цикла. По присланным позам ездит сам
Nav2 действием ``navigate_through_poses``, поэтому своей ноды-исполнителя
здесь нет.

Запретные зоны
--------------

Аргумент ``use_keepout`` включает в костмапах фильтр
``nav2_costmap_2d::KeepoutFilter``. Маску для него публикует
``vector_object_server`` из ``rtk2026_vector_objects``, который поднимается
своим лаунчем. Без него фильтр включать нельзя: он будет ждать несуществующую
информацию о фильтре и писать об этом в лог, ничего не запрещая.

Одометрия
---------

Стек ездит по фильтрованной одометрии EKF, а не по сырой колёсной: скорость
из неё задаёт lookahead контроллера и проверки в дереве поведения. Источник
задаётся аргументом ``odom_topic`` и попадает к потребителям двумя путями:
``bt_navigator`` берёт его параметром, а ``controller_server`` — ремапом,
потому что параметра для одометрии у него нет и имя топика в нём зашито.

Менеджер жизненного цикла
-------------------------

Поднимается с задержкой. Пока робот не заспавнен, нет ``joint_states``
и ``odom``, а значит разорвано TF ``odom -> base_footprint``:
``controller_server`` падает на активации, а ``bt_navigator`` остаётся
inactive. Задержка ждёт появления TF, поэтому её величина зависит от способа
запуска робота и задаётся аргументом.

Сонары
------

Аргумент ``use_sonars`` добавляет RangeSensorLayer в локальную костмапу
и ставит Collision Monitor между Nav2 и приводом. При этом серверы
движения пишут в ``/cmd_vel_nav``, а защищённая команда выходит
из Collision Monitor в штатный ``/cmd_vel``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

#: Серверы, которыми управляет менеджер жизненного цикла. Порядок значим:
#: bt_navigator активируется последним, когда всё, что он вызывает, готово.
LIFECYCLE_NODES = (
    ("nav2_planner", "planner_server"),
    ("nav2_controller", "controller_server"),
    ("nav2_behaviors", "behavior_server"),
    ("nav2_bt_navigator", "bt_navigator"),
)

def _stack(context, *_args, **_kwargs) -> list:
    """Серверы и менеджер. Собираются после разбора аргументов."""
    # Время и пути к деревьям перезаписываются на всех уровнях, включая
    # вложенные костмапы: подстановка отдельным словарём параметров до них
    # не доходит, потому что костмапа — вложенная нода со своим ключом.
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "default_nav_through_poses_bt_xml": LaunchConfiguration("through_poses_bt"),
            "default_nav_to_pose_bt_xml": LaunchConfiguration("to_pose_bt"),
        },
        convert_types=True,
    )

    # Фильтр запретных зон добавляется файлом поверх основного: ключ filters
    # должен либо отсутствовать, либо быть непустым, а значением его не
    # выключить — см. config/keepout_filter.yaml.
    use_keepout = LaunchConfiguration("use_keepout").perform(context).lower()
    use_sonars = LaunchConfiguration("use_sonars").perform(context).lower()
    params: list = [configured_params]
    if use_keepout in ("true", "1"):
        params.append(LaunchConfiguration("keepout_params_file"))
    if use_sonars in ("true", "1"):
        configured_sonar_params = RewrittenYaml(
            source_file=LaunchConfiguration("sonar_params_file"),
            root_key="",
            param_rewrites={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
            convert_types=True,
        )
        params.append(configured_sonar_params)

    # Ремапы по серверу. controller_server подписан на имя odom, менять его
    # параметром нельзя, поэтому одометрия приходит к нему только так.
    remappings = {
        "controller_server": [("odom", LaunchConfiguration("odom_topic"))],
    }

    lifecycle_nodes = list(LIFECYCLE_NODES)
    if use_sonars in ("true", "1"):
        # Любая команда Nav2 проходит через быструю защиту,
        # которая смотрит Range напрямую, не дожидаясь костмапы.
        remappings["controller_server"].append(("cmd_vel", "/cmd_vel_nav"))
        remappings["behavior_server"] = [("cmd_vel", "/cmd_vel_nav")]
        lifecycle_nodes.insert(
            -1,
            ("nav2_collision_monitor", "collision_monitor"),
        )

    servers = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=params,
            remappings=remappings.get(executable, []),
        )
        for package, executable in lifecycle_nodes
    ]

    manager = TimerAction(
        period=LaunchConfiguration("lifecycle_delay_s"),
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    configured_params,
                    {
                        "autostart": LaunchConfiguration("autostart"),
                        # Активация ждёт костмапы и TF, а не отвечает мгновенно.
                        "service_timeout": 60.0,
                        # Ноль отключает bond: серверы этого стека переживают
                        # короткие пропадания без перезапуска.
                        "bond_timeout": 0.0,
                        "node_names": [
                            executable for _, executable in lifecycle_nodes
                        ],
                    },
                ],
            )
        ],
    )

    return [*servers, manager]


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("rtk2026_nav2")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Брать время из /clock. Для реального робота false.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered",
                description=(
                    "Одометрия для стека: выход EKF, а не сырая колёсная. "
                    "Уходит параметром в bt_navigator и ремапом "
                    "в controller_server."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "nav2_params.yaml"]
                ),
                description="Параметры стека Nav2.",
            ),
            DeclareLaunchArgument(
                "through_poses_bt",
                default_value=PathJoinSubstitution(
                    [
                        share,
                        "behavior_trees",
                        "navigate_through_poses_static.xml",
                    ]
                ),
                description="Дерево поведения для navigate_through_poses.",
            ),
            DeclareLaunchArgument(
                "to_pose_bt",
                # Для одиночной цели берём штатное дерево Nav2: своего смысла
                # у неё в городском движении нет, а прибивать путь
                # к дистрибутиву незачем.
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("nav2_bt_navigator"),
                        "behavior_trees",
                        "navigate_w_replanning_only_if_goal_is_updated.xml",
                    ]
                ),
                description="Дерево поведения для navigate_to_pose.",
            ),
            DeclareLaunchArgument(
                "keepout_params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "keepout_filter.yaml"]
                ),
                description="Наложение с фильтром запретных зон.",
            ),
            DeclareLaunchArgument(
                "use_keepout",
                default_value="false",
                description=(
                    "Включить фильтр запретных зон в костмапах. Требует "
                    "запущенного vector_object_server, иначе фильтр будет "
                    "ждать информацию о фильтре и ничего не запретит."
                ),
            ),
            DeclareLaunchArgument(
                "sonar_params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "sonar_navigation.yaml"]
                ),
                description=(
                    "Наложение RangeSensorLayer и Collision Monitor "
                    "для шести сонаров."
                ),
            ),
            DeclareLaunchArgument(
                "use_sonars",
                default_value="false",
                description=(
                    "Подключить шесть Range-топиков к локальной "
                    "костмапе и пустить cmd_vel через Collision Monitor."
                ),
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Поднимать серверы в active сразу.",
            ),
            DeclareLaunchArgument(
                "lifecycle_delay_s",
                default_value="12.0",
                description=(
                    "Задержка перед активацией: ждём TF odom -> base_footprint. "
                    "В симуляции робот спавнится не сразу."
                ),
            ),
            OpaqueFunction(function=_stack),
        ]
    )
