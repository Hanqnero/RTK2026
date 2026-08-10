"""Нода движения по городу: сборка частей и цикл автомата.

Нода — переводчик, а не алгоритм. Решения принимает
:class:`~rtk2026_city_nav.controller.Controller`, и здесь нет ни одной ветки,
которая выбирала бы маневр или считала геометрию.

Из чего собрана
---------------

:class:`~rtk2026_city_nav.nav2_client.Nav2Goals`
    Всё про действие Nav2: цель, отмена, перевод поз в сообщения.

:class:`~rtk2026_city_nav.sign_memory.SignMemory`
    Файл памяти о знаках: включение, загрузка, запись при изменении.

:class:`~rtk2026_city_nav.manual_poses.ManualPoses`
    Позы, правленные руками, вместо расчётных.

:mod:`rtk2026_city_nav.diagnostics`
    Задачи ``/diagnostics``, каждая от того, что описывает.

Ноде остаётся собрать это, перекладывать детекции в накопитель и вести цикл.

Обратные вызовы автомат не двигают
----------------------------------

Результат цели Nav2, запрос возобновления и детекции приходят каждый из
своего обратного вызова. Они только записывают событие, а автомат двигается
в таймере — в одном месте и за один проход.

Иначе обработка результата цели немедленно отправляла бы следующую, и то же
делал бы её обратный вызов: цепочка «событие — команда — событие» уходила бы
в рекурсию тем глубже, чем длиннее маршрут.

Параметры
---------

Значений по умолчанию в коде нет: параметры объявлены типами в
:mod:`rtk2026_city_nav.parameters`, значения лежат в ``config/city_nav.yaml``,
пути к файлам приходят из лаунча. Отсутствие параметра — отказ при запуске,
а не молчаливо подставленное число, с которым робот поедет не туда.
"""

from __future__ import annotations

import math

import rclpy
from diagnostic_updater import Updater
from rclpy.node import Node
from rtk2026_interfaces.msg import DrivingDetection
from std_srvs.srv import Trigger

from rtk2026_city_nav import parameters
from rtk2026_city_nav.controller import (
    Command,
    Controller,
    ControllerConfig,
    ControllerState,
    FollowPoses,
    Halt,
    Wait,
)
from rtk2026_city_nav.detections import Latch
from rtk2026_city_nav.diagnostics import (
    DecisionTask,
    DetectionTask,
    GraphTask,
    RouteTask,
    SignMemoryTask,
)
from rtk2026_city_nav.manual_poses import ManualPoses
from rtk2026_city_nav.nav2_client import Nav2Goals
from rtk2026_city_nav.planner import ManeuverTable, Planner, RouteState
from rtk2026_city_nav.poses_io import graph_fingerprint
from rtk2026_city_nav.sign_memory import SignMemory
from rtk2026_city_nav.topology import build_topology
from rtk2026_city_nav.validate import Severity, validate
from rtk2026_pose_graph import load_geojson_path

#: События, которые обратные вызовы оставляют для таймера.
_ARRIVED = "arrived"
_FAILED = "failed"
_RESUME = "resume"


class CityNavNode(Node):
    """Движение по городу: граф, знаки, Nav2."""

    def __init__(self) -> None:
        super().__init__("city_nav")

        parameters.declare(self, *parameters.SPEC)
        logger = self.get_logger()

        self._graph = load_geojson_path(str(self._value("graph_path")))
        self._topology = build_topology(self._graph)
        self._table = ManeuverTable(
            self._topology,
            straight_tolerance_rad=math.radians(
                float(self._value("straight_tolerance_deg"))
            ),
        )
        fingerprint = graph_fingerprint(self._graph)

        self._manual = ManualPoses(
            logger,
            path=str(self._value("poses_path")),
            graph_fingerprint=fingerprint,
        )
        self._memory = SignMemory(
            logger,
            enabled=bool(self._value("use_sign_cache")),
            path=str(self._value("sign_cache_path")),
            graph_fingerprint=fingerprint,
        )
        self._latch = Latch(
            min_box_area_px=float(self._value("min_box_area_px")),
            min_confidence=float(self._value("min_confidence")),
        )

        self._start = RouteState(
            previous=int(self._value("start_previous_vertex")),
            current=int(self._value("start_current_vertex")),
        )
        self._report = validate(self._topology, self._table, start=self._start)
        self._log_report()

        self._controller = Controller(
            planner=Planner(self._table),
            config=ControllerConfig(
                lane_offset_m=float(self._value("lane_offset_m")),
                pose_step_m=float(self._value("pose_step_m")),
                traffic_side=int(self._value("traffic_side")),
                miter_limit=float(self._value("miter_limit")),
                max_retries=int(self._value("max_retries")),
                default_stop_duration_s=float(
                    self._value("default_stop_duration_s")
                ),
            ),
            route=self._start,
            latch=self._latch,
            cache=self._memory.cache,
        )

        #: События от обратных вызовов, ожидающие обработки в таймере.
        self._pending: list[tuple[str, str]] = []
        #: Что помешало начать движение. Пусто, если всё в порядке.
        self._blocked = self._startup_obstacle()

        self._goals = Nav2Goals(
            self,
            action_name=str(self._value("nav2_action_name")),
            frame_id=str(self._value("frame_id")),
            server_timeout_s=float(self._value("nav2_server_timeout_s")),
            on_arrived=lambda: self._pending.append((_ARRIVED, "")),
            on_failed=lambda reason: self._pending.append((_FAILED, reason)),
        )
        self._resume_service = self.create_service(
            Trigger, "~/resume", self._on_resume_request
        )

        # Порог принадлежности отличает знак ближайшей точки решения от знака
        # следующей. Без него далёкий знак приписался бы не туда, поэтому при
        # нуле детекции не читаются вовсе, и маршрут идёт по покрытию.
        detection_topic = str(self._value("detection_topic"))
        self._detections = (
            self.create_subscription(
                DrivingDetection, detection_topic, self._on_detection, 10
            )
            if self._latch.min_box_area_px > 0.0
            else None
        )
        if self._detections is None:
            logger.warn(
                "min_box_area_px не задан: знаки не учитываются, "
                "маршрут пойдёт по покрытию"
            )

        self._updater = Updater(self, period=float(self._value("diagnostic_period_s")))
        self._updater.setHardwareID("rtk2026_city_nav")
        self._updater.add(
            GraphTask(
                self._topology,
                self._table,
                self._report,
                self._manual,
                lambda: self._blocked,
            )
        )
        self._updater.add(RouteTask(self._controller, self._goals, self._manual))
        self._updater.add(DecisionTask(self._controller))
        self._updater.add(SignMemoryTask(self._memory, self._table))
        self._updater.add(DetectionTask(self, self._latch, detection_topic))

        if self._blocked:
            logger.error(f"движение не начато: {self._blocked}")
            return

        self._timer = self.create_timer(
            float(self._value("control_period_s")), self._tick
        )
        logger.info(
            f"старт {self._start.previous} -> {self._start.current}, "
            f"точек решений {len(self._topology.decision_points)}, "
            f"цепочек {len(self._topology.chains)}"
        )

    def _value(self, name: str):
        return self.get_parameter(name).value

    def _startup_obstacle(self) -> str:
        """Что мешает начать движение. Пусто, если ничего."""
        if self._start.previous < 0 or self._start.current < 0:
            return (
                "не задано начальное состояние: нужны start_previous_vertex "
                "и start_current_vertex"
            )

        if not self._table.candidates(self._start):
            return (
                f"из состояния {self._start.previous} -> {self._start.current} "
                "нет ни одного маневра: это должны быть смежные точки решений"
            )

        if self._report.errors and bool(self._value("halt_on_validation_error")):
            return (
                f"проверка графа нашла ошибок {len(self._report.errors)}: "
                "исправьте граф либо снимите halt_on_validation_error"
            )

        return ""

    def _log_report(self) -> None:
        logger = self.get_logger()
        logger.info(self._report.summary())
        for finding in self._report.findings:
            text = str(finding)
            if finding.severity is Severity.ERROR:
                logger.error(text)
            else:
                logger.warn(text)

    # -- Цикл --------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        """Единственное место, где двигается автомат.

        События забираются целиком, а не по одному: команда может породить
        новое событие — например, недоступный сервер Nav2 даёт отказ сразу
        при отправке, — и обрабатывать его надо следующим тиком, а не в этом
        же проходе.
        """
        now = self._now_s()
        events, self._pending = self._pending, []

        for event, reason in events:
            if event == _ARRIVED:
                self._execute(self._controller.on_arrived(now))
            elif event == _FAILED:
                self._execute(self._controller.on_failed(now, reason=reason))
            elif event == _RESUME:
                self._execute(self._controller.on_resume(now))

        self._execute(self._controller.poll(now))
        self._memory.save_if_changed()

    def _execute(self, commands: tuple[Command, ...]) -> None:
        """Выполнить то, что попросил автомат."""
        for command in commands:
            if isinstance(command, FollowPoses):
                self._follow(command)
            elif isinstance(command, Wait):
                self.get_logger().info(
                    f"стоим {command.duration_s:.1f} с: {command.reason}"
                )
            elif isinstance(command, Halt):
                self.get_logger().error(f"остановка: {command.reason}")
                self._goals.cancel()

    def _follow(self, command: FollowPoses) -> None:
        decision = command.decision
        poses = self._manual.resolve(
            decision.state.current, decision.target, command.poses
        )

        self.get_logger().info(
            f"участок {decision.state.current} -> {decision.target}: "
            f"{decision.maneuver.value} по {decision.source}, поз {len(poses)}"
        )
        self._goals.send(poses)

    def _on_resume_request(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Возобновить движение после остановки."""
        state = self._controller.state
        if state is not ControllerState.RECOVER:
            response.success = False
            response.message = f"состояние {state.value}, возобновлять нечего"
            return response

        self._pending.append((_RESUME, ""))
        response.success = True
        response.message = "движение будет возобновлено следующим тиком"
        return response

    def _on_detection(self, message) -> None:
        """Переложить поля сообщения в накопитель.

        Порог принадлежности и выбор лучшей детекции — забота накопителя;
        здесь только распаковка.
        """
        if message.route_command:
            self._latch.observe_route(
                message.route_command,
                confidence=float(message.route_confidence),
                box_area_px=float(message.route_box_area),
            )

        if message.stop_action:
            self._latch.observe_stop(
                message.stop_action,
                duration_s=float(message.stop_duration_sec),
                confidence=float(message.stop_confidence),
                box_area_px=float(message.stop_box_area),
            )

        if message.bus_detected:
            self._latch.observe_bus(
                box_area_px=float(message.bus_box_area),
                confidence=float(message.bus_confidence),
            )

    def save_sign_memory(self) -> None:
        """Дописать память о знаках. Вызывается при остановке ноды."""
        self._memory.save_if_changed()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CityNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Прогон обычно останавливают с клавиатуры, и выученное на последнем
        # круге иначе не дописалось бы.
        node.save_sign_memory()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()