"""Единая таблица параметров пакета.

Параметры объявляют три исполняемых файла: нода движения и два инструмента
проверки. Держать имена и типы в трёх местах значило бы рано или поздно их
разойтись, поэтому объявление одно, а каждый берёт нужное подмножество.

Значений по умолчанию нет: параметры объявляются типами, значения лежат в
``config/city_nav.yaml``, пути к файлам приходят из лаунча. Отсутствие
параметра — отказ при запуске, а не молчаливо подставленное число.

Описания видны в ``ros2 param describe``, поэтому смысл параметра не
приходится искать по исходникам.
"""

from __future__ import annotations

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter

_TEXT = Parameter.Type.STRING
_REAL = Parameter.Type.DOUBLE
_WHOLE = Parameter.Type.INTEGER
_FLAG = Parameter.Type.BOOL

#: Имя параметра — тип и описание.
SPEC: dict[str, tuple[Parameter.Type, str]] = {
    "graph_path": (_TEXT, "GeoJSON разметочной линии"),
    "poses_path": (
        _TEXT,
        "Файл поз; нода берёт из него только записи с пометкой manual. "
        "Пусто — все позы считаются на ходу",
    ),
    "frame_id": (_TEXT, "Система координат целей Nav2"),
    "lane_offset_m": (_REAL, "Смещение центра полосы от разметочной линии"),
    "pose_step_m": (_REAL, "Шаг поз; 0 — только точки полилинии"),
    "traffic_side": (_WHOLE, "1 правостороннее, -1 левостороннее"),
    "miter_limit": (_REAL, "Предел выноса позы в изломе"),
    "straight_tolerance_deg": (
        _REAL,
        "Полураствор классов прямо и разворот",
    ),
    "max_retries": (_WHOLE, "Повторов участка до остановки"),
    "default_stop_duration_s": (_REAL, "Остановка, если знак её не задал"),
    "control_period_s": (_REAL, "Период автомата исполнителя"),
    "halt_on_validation_error": (_FLAG, "Не ехать при ошибках проверки графа"),
    "start_previous_vertex": (
        _WHOLE,
        "Откуда приехали; отрицательное — не задано",
    ),
    "start_current_vertex": (_WHOLE, "Где находимся; отрицательное — не задано"),
    "detection_topic": (_TEXT, "Топик детекций знаков"),
    "min_box_area_px": (
        _REAL,
        "Порог принадлежности знака к ближайшей точке решения; "
        "0 — не откалиброван, ехать нельзя",
    ),
    "min_confidence": (_REAL, "Порог уверенности детекции"),
    "nav2_action_name": (_TEXT, "Имя действия Nav2"),
    "nav2_server_timeout_s": (_REAL, "Ожидание сервера действия"),
    "diagnostic_period_s": (_REAL, "Период публикации /diagnostics"),
}

#: Что нужно, чтобы прочитать граф и построить таблицу маневров. Нужно всем.
GRAPH = ("graph_path", "straight_tolerance_deg")

#: Где робот в маршруте. Нужно ноде и проверке достижимости.
START = ("start_previous_vertex", "start_current_vertex")

#: Геометрия полосы. Нужна ноде и инструменту поз: считают они одно и то же.
LANE = ("lane_offset_m", "pose_step_m", "traffic_side", "miter_limit")


def declare(node: Node, *names: str) -> None:
    """Объявить перечисленные параметры на ноде.

    :raises KeyError: имени нет в :data:`SPEC`. Опечатка в имени должна
        ломать запуск, а не тихо создавать параметр, которого никто не задаёт.
    """
    for name in names:
        kind, description = SPEC[name]
        node.declare_parameter(name, kind, ParameterDescriptor(description=description))
