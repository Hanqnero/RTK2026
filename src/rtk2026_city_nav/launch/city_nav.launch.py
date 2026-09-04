#!/usr/bin/env python3
"""Запуск ноды движения по городу.

Только сама нода. Стек Nav2, запретные зоны, карту, локализацию и
диагностику поднимают их собственные лаунчи: так каждый включается
и выключается отдельно, и режимы можно смешивать как нужно.

Четыре режима задаются двумя парами аргументов.

Позы участков:

* ``poses_path:=""`` — позы считаются на ходу из графа. Так проще: править
  нечего, менять граф можно свободно.
* ``poses_path:=<файл>`` — из файла берутся записи с пометкой ``manual``,
  остальные всё равно считаются. Нужно, когда часть точек пришлось отодвинуть
  руками. Файл создаёт ``ros2 run rtk2026_city_nav city_nav_poses``.

Знаки:

* ``use_sign_cache:=false`` — знаки читаются каждый проезд заново. Так видно,
  что перцепция находит сама по себе, без подстановок.
* ``use_sign_cache:=true`` без ``sign_cache_path`` — учится за прогон,
  следующий запуск начинает с нуля.
* ``use_sign_cache:=true`` с ``sign_cache_path`` — выученное переживает
  перезапуск. Память привязана к отпечатку графа: изменилась геометрия —
  файл отбрасывается.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("rtk2026_city_nav")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "city_nav.yaml"]
                ),
                description="Параметры движения по городу.",
            ),
            DeclareLaunchArgument(
                "graph_path",
                default_value="/workspace/maps/graph",
                description=(
                    "GeoJSON разметочной линии, сохранённый панелью Nav2 Route "
                    "Tool. По умолчанию берётся из рабочего каталога карт, а не "
                    "из config пакета: граф правится вместе с картой и живёт "
                    "рядом с ней, копия внутри пакета разъезжалась бы с ней. "
                    "Каталог maps/ смонтирован в /workspace/maps и на роботе, "
                    "и в симуляции."
                ),
            ),
            DeclareLaunchArgument(
                "poses_path",
                default_value="",
                description=(
                    "Файл поз. Пусто — позы считаются на ходу; иначе из файла "
                    "берутся записи с пометкой manual."
                ),
            ),
            DeclareLaunchArgument(
                "use_sign_cache",
                default_value="true",
                description=(
                    "Учитывать знаки, выученные на прошлых проездах. "
                    "false — читать заново каждый проезд."
                ),
            ),
            DeclareLaunchArgument(
                "sign_cache_path",
                default_value="",
                description=(
                    "Файл памяти о знаках. Пусто — память живёт только "
                    "до конца прогона."
                ),
            ),
            DeclareLaunchArgument(
                "start_previous_vertex",
                default_value="-1",
                description="Вершина, откуда приехали. Зависит от постановки робота.",
            ),
            DeclareLaunchArgument(
                "start_current_vertex",
                default_value="-1",
                description="Вершина, где робот стоит перед прогоном.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Брать время из /clock. Для реального робота false.",
            ),
            Node(
                package="rtk2026_city_nav",
                executable="city_nav_node",
                name="city_nav",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        # Только то, чего в файле нет: пути, которые в YAML
                        # не записать без привязки к раскладке рабочего
                        # пространства, положение робота перед прогоном и
                        # выбор режима. У каждого параметра один источник,
                        # иначе значение по умолчанию отсюда затирало бы
                        # настроенное в файле.
                        "graph_path": LaunchConfiguration("graph_path"),
                        "poses_path": LaunchConfiguration("poses_path"),
                        "sign_cache_path": LaunchConfiguration("sign_cache_path"),
                        "use_sign_cache": LaunchConfiguration("use_sign_cache"),
                        "start_previous_vertex": LaunchConfiguration(
                            "start_previous_vertex"
                        ),
                        "start_current_vertex": LaunchConfiguration(
                            "start_current_vertex"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
        ]
    )