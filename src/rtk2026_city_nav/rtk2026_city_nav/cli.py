"""Проверки до выезда: пригодность графа и файл поз.

Обе — обычные ноды ROS 2, разово выполняющие работу и завершающиеся. Они
берут те же параметры из того же ``config/city_nav.yaml``, что и нода
движения, поэтому проверять и ехать заведомо будут по одинаковым числам.

Вывод идёт через ``get_logger``, то есть попадает в ``/rosout`` и виден
теми же средствами, что и всё остальное.

Код выхода ненулевой, когда что-то требует внимания. Это позволяет ставить
проверку в скрипт запуска, а не надеяться, что кто-то прочитает вывод.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

from rtk2026_city_nav import parameters
from rtk2026_city_nav.planner import ManeuverTable, RouteState
from rtk2026_city_nav.poses_io import generate, load, merge, save
from rtk2026_city_nav.topology import build_topology
from rtk2026_city_nav.validate import Severity, validate
from rtk2026_pose_graph import load_geojson_path


class _GraphNode(Node):
    """Общее для обеих проверок: прочитать граф, построить таблицу маневров."""

    def __init__(self, name: str, *extra: str) -> None:
        super().__init__(name)
        parameters.declare(self, *parameters.GRAPH, *extra)

        path = str(self.get_parameter("graph_path").value)
        self.graph = load_geojson_path(path)
        self.topology = build_topology(self.graph)
        self.table = ManeuverTable(
            self.topology,
            straight_tolerance_rad=math.radians(
                float(self.get_parameter("straight_tolerance_deg").value)
            ),
        )

        self.get_logger().info(
            f"граф {path}: вершин {len(self.graph.nodes)}, "
            f"рёбер {len(self.graph.edges)}, "
            f"точек решений {len(self.topology.decision_points)}, "
            f"цепочек {len(self.topology.chains)}, "
            f"состояний {len(self.table.states)}"
        )


class CheckNode(_GraphNode):
    """Проверка пригодности графа для движения."""

    def __init__(self) -> None:
        super().__init__("city_nav_check", *parameters.START)

    def run(self) -> int:
        """:returns: ноль, если ошибок нет."""
        previous = int(self.get_parameter("start_previous_vertex").value)
        current = int(self.get_parameter("start_current_vertex").value)

        start = None
        if previous >= 0 and current >= 0:
            start = RouteState(previous=previous, current=current)
        else:
            self.get_logger().warn(
                "начальное состояние не задано: достижимость не проверялась"
            )

        report = validate(self.topology, self.table, start=start)

        logger = self.get_logger()
        logger.info(report.summary())
        for finding in report.findings:
            text = str(finding)
            if finding.severity is Severity.ERROR:
                logger.error(text)
            else:
                logger.warn(text)

        for state in sorted(
            report.uturn_exceptions, key=lambda s: (s.previous, s.current)
        ):
            logger.warn(
                f"{state.previous} -> {state.current}: тупик, "
                "запрет разворота здесь обязан не действовать"
            )

        if not report.errors:
            logger.info("граф пригоден: можно ехать")

        return 1 if report.errors else 0


class PosesNode(_GraphNode):
    """Создание и обновление файла поз."""

    def __init__(self) -> None:
        super().__init__("city_nav_poses", "poses_path", *parameters.LANE)

    def run(self) -> int:
        """:returns: ненулевой код, если есть устаревшие ручные правки."""
        logger = self.get_logger()

        raw = str(self.get_parameter("poses_path").value).strip()
        if not raw:
            logger.error("poses_path не задан: некуда писать")
            return 2
        path = Path(raw)

        fresh = generate(
            self.topology,
            lane_offset_m=float(self.get_parameter("lane_offset_m").value),
            pose_step_m=float(self.get_parameter("pose_step_m").value),
            traffic_side=int(self.get_parameter("traffic_side").value),
            miter_limit=float(self.get_parameter("miter_limit").value),
        )
        poses_total = sum(len(leg.poses) for leg in fresh.legs)

        if not path.exists():
            save(path, fresh)
            logger.info(
                f"создан {path}: участков {len(fresh.legs)}, поз {poses_total}, "
                f"отпечаток графа {fresh.graph_fingerprint}"
            )
            return 0

        merged, report = merge(load(path), fresh)
        save(path, merged)

        logger.info(f"обновлён {path}: {report.summary()}")
        for line in report.details():
            if "проверить" in line:
                logger.warn(line)
            else:
                logger.info(line)

        return 1 if report.stale_manual else 0


def _run(factory) -> None:
    """Поднять ноду, выполнить работу, завершиться её кодом."""
    rclpy.init()
    code = 2
    node = None
    try:
        node = factory()
        code = node.run()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    sys.exit(code)


def main_check() -> None:
    _run(CheckNode)


def main_poses() -> None:
    _run(PosesNode)
