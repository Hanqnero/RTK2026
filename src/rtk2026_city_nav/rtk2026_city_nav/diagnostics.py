"""Задачи диагностики движения по городу.

Публикуются штатным :mod:`diagnostic_updater` в ``/diagnostics`` с именами
``CityNav/<ключ>``, как это делают мониторы ``rtk2026_observability``; их
подхватывает анализатор ``CityNav`` в ``diagnostic_aggregator.yaml``.

Здесь только то, что видно изнутри: выбранный маневр и его основание,
счётчики посещений, эффективная скорость последнего участка, попытки, уходы
в остановку, выученные знаки, отброшенные по дальности детекции. Всё, что
видно снаружи — присутствие ноды, lifecycle Nav2, активность топиков —
снимают штатные ``node_monitor`` и ``topic_monitor``, и дублировать это
здесь незачем.

Каждая задача зависит только от того, что описывает: так видно, откуда
берётся каждое число, и задачу можно завести, не поднимая остального.
"""

from __future__ import annotations

from collections.abc import Callable

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask
from rclpy.node import Node

from rtk2026_city_nav.controller import Controller, ControllerState
from rtk2026_city_nav.detections import Latch
from rtk2026_city_nav.manual_poses import ManualPoses
from rtk2026_city_nav.nav2_client import Nav2Goals
from rtk2026_city_nav.planner import ManeuverTable
from rtk2026_city_nav.sign_memory import SignMemory
from rtk2026_city_nav.topology import Topology
from rtk2026_city_nav.validate import Report


class GraphTask(DiagnosticTask):
    """Пригодность графа. Считается один раз при загрузке.

    Причина отказа выехать приходит замыканием, а не значением: она
    выясняется после того, как задачи уже заведены, чтобы диагностика
    работала и у ноды, которая ехать отказалась.
    """

    def __init__(
        self,
        topology: Topology,
        table: ManeuverTable,
        report: Report,
        manual: ManualPoses,
        blocked: Callable[[], str],
    ) -> None:
        super().__init__("CityNav/graph")
        self._topology = topology
        self._table = table
        self._report = report
        self._manual = manual
        self._blocked = blocked

    def run(self, status):
        status.add("Точек решений", str(len(self._topology.decision_points)))
        status.add("Цепочек", str(len(self._topology.chains)))
        status.add("Состояний в таблице", str(len(self._table.states)))
        status.add("Ошибок проверки", str(len(self._report.errors)))
        status.add("Предупреждений проверки", str(len(self._report.warnings)))
        status.add("Ручных правок поз", str(len(self._manual)))

        blocked = self._blocked()
        if blocked:
            status.summary(DiagnosticStatus.ERROR, blocked)
        elif self._report.errors:
            status.summary(DiagnosticStatus.ERROR, self._report.summary())
        elif self._report.warnings:
            status.summary(DiagnosticStatus.WARN, self._report.summary())
        else:
            status.summary(DiagnosticStatus.OK, "граф пригоден для движения")

        return status


class RouteTask(DiagnosticTask):
    """Где робот в маршруте и как прошёл последний участок."""

    def __init__(
        self, controller: Controller, goals: Nav2Goals, manual: ManualPoses
    ) -> None:
        super().__init__("CityNav/route")
        self._controller = controller
        self._goals = goals
        self._manual = manual

    def run(self, status):
        controller = self._controller
        route = controller.route

        status.add("Состояние", controller.state.value)
        status.add("Приехал из", str(route.previous))
        status.add("Находится в", str(route.current))
        status.add("Попытка", str(controller.attempt))
        status.add("Уходов в остановку", str(controller.recover_count))
        status.add("Отклонённых целей Nav2", str(self._goals.rejected))
        status.add("Участков по ручным позам", str(self._manual.used))

        visits = controller.planner.visits
        status.add("Посещено точек решений", str(len(visits)))
        status.add(
            "Максимум посещений одной точки",
            str(max(visits.values())) if visits else "0",
        )

        last = controller.last_leg
        if last is not None:
            status.add("Последний участок, м", f"{last.length_m:.2f}")
            speed = last.speed_mps
            status.add(
                "Скорость на последнем участке, м/с",
                "нет данных" if speed is None else f"{speed:.3f}",
            )
            duration = last.duration_s
            status.add(
                "Время последнего участка, с",
                "нет данных" if duration is None else f"{duration:.2f}",
            )
            status.add("Выбор и построение поз, с", f"{last.planning_s:.6f}")
            status.add(
                "Исход последнего участка",
                "нет данных" if last.outcome is None else last.outcome.value,
            )

        if controller.state is ControllerState.RECOVER:
            status.summary(
                DiagnosticStatus.ERROR, f"движение остановлено: {controller.reason}"
            )
        elif controller.state is ControllerState.WAIT:
            status.summary(DiagnosticStatus.OK, f"стоим: {controller.reason}")
        elif controller.attempt > 0:
            status.summary(
                DiagnosticStatus.WARN, f"повтор участка, попытка {controller.attempt}"
            )
        else:
            status.summary(DiagnosticStatus.OK, "движение идёт")

        return status


class DecisionTask(DiagnosticTask):
    """Последний выбор маневра и что было на выбор."""

    def __init__(self, controller: Controller) -> None:
        super().__init__("CityNav/decision")
        self._controller = controller

    def run(self, status):
        record = self._controller.leg or self._controller.last_leg

        if record is None:
            status.summary(DiagnosticStatus.OK, "выбор ещё не делался")
            return status

        decision = record.decision
        status.add("Маневр", decision.maneuver.value)
        status.add("Основание", decision.source)
        status.add("Цель", str(decision.target))
        status.add(
            "Было на выбор",
            ", ".join(
                f"{c.maneuver.value}->{c.target} ({c.turn_deg:+.0f})"
                for c in decision.candidates
            ),
        )
        status.add(
            "Вычеркнуто знаком",
            ", ".join(sorted(m.value for m in decision.forbidden)) or "ничего",
        )

        if decision.prohibition_ignored:
            status.summary(
                DiagnosticStatus.WARN,
                "запрещающий знак не оставил ни одного маневра и был "
                "проигнорирован: знак противоречит графу",
            )
        else:
            status.summary(
                DiagnosticStatus.OK,
                f"{decision.maneuver.value} по {decision.source}",
            )

        return status


class SignMemoryTask(DiagnosticTask):
    """Что выучено о знаках и согласуется ли это с наблюдениями."""

    def __init__(self, memory: SignMemory, table: ManeuverTable) -> None:
        super().__init__("CityNav/sign_memory")
        self._memory = memory
        self._table = table

    def run(self, status):
        cache = self._memory.cache

        if cache is None:
            status.add("Память", "выключена")
            status.summary(DiagnosticStatus.OK, "знаки читаются каждый проезд заново")
            return status

        total = len(self._table.states)
        path = self._memory.path

        status.add("Файл", str(path) if path is not None else "нет")
        status.add("Изучено состояний", str(cache.known_states))
        status.add("Всего состояний в графе", str(total))
        status.add("Из них со знаком", str(cache.constrained_states))
        status.add("Решений по памяти", str(cache.hits))
        status.add("Поправок", str(cache.corrections))
        status.add("Расхождений", str(cache.conflicts))

        disputed = cache.disputed_states
        status.add(
            "Состояния с разными знаками",
            ", ".join(f"{a}->{b}" for a, b in disputed) or "нет",
        )

        if disputed:
            # Знаки статичны: разные прочтения в одном состоянии означают,
            # что ошибается перцепция, а не меняется трасса.
            status.summary(
                DiagnosticStatus.WARN,
                f"в {len(disputed)} состояниях читались разные знаки: "
                "перцепция расходится сама с собой",
            )
        elif cache.known_states < total:
            status.summary(
                DiagnosticStatus.OK, f"изучено {cache.known_states} из {total}"
            )
        else:
            status.summary(DiagnosticStatus.OK, "трасса изучена полностью")

        return status


class DetectionTask(DiagnosticTask):
    """Что накоплено к предстоящему выбору и есть ли кому публиковать знаки."""

    def __init__(
        self,
        node: Node,
        latch: Latch,
        topic: str,
        *,
        last_message_age_s: Callable[[], float | None],
        timeout_s: float,
    ) -> None:
        super().__init__("CityNav/detections")
        self._node = node
        self._latch = latch
        self._topic = topic
        self._last_message_age_s = last_message_age_s
        self._timeout_s = max(0.0, float(timeout_s))

    def run(self, status):
        latch = self._latch

        if latch.min_box_area_px <= 0.0:
            status.add("Порог принадлежности, пикс2", "не задан")
            status.summary(
                DiagnosticStatus.OK,
                "знаки не учитываются: маршрут по покрытию",
            )
            return status

        publishers = self._node.count_publishers(self._topic)
        message_age_s = self._last_message_age_s()

        status.add("Topic", self._topic)
        status.add("Publishers", str(publishers))
        status.add(
            "Возраст сообщения, с",
            "нет сообщений" if message_age_s is None else f"{message_age_s:.2f}",
        )
        status.add("Порог принадлежности, пикс2", f"{latch.min_box_area_px:.0f}")
        status.add("Порог уверенности", f"{latch.min_confidence:.2f}")
        status.add("Накопленная команда", latch.route_command or "нет")
        status.add("Площадь её рамки, пикс2", f"{latch.route_box_area_px:.0f}")

        stop = latch.stop_request()
        status.add("Требование остановки", "нет" if stop is None else stop.reason)
        status.add("Отброшено как далёкие", str(latch.too_far_count))

        if publishers == 0:
            # Ехать можно и по покрытию, но знаки при этом не действуют,
            # и узнать об этом лучше здесь, а не по итогам прогона.
            status.summary(
                DiagnosticStatus.WARN,
                "знаки никто не публикует: маршрут пойдёт по покрытию",
            )
        elif message_age_s is None:
            status.summary(
                DiagnosticStatus.WARN,
                "издатель есть, но детекции ещё не приходили",
            )
        elif message_age_s > self._timeout_s:
            status.summary(
                DiagnosticStatus.WARN,
                f"детекции не приходили {message_age_s:.1f} с",
            )
        else:
            status.summary(DiagnosticStatus.OK, "детекции принимаются")

        return status
