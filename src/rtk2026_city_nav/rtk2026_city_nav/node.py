"""Нода движения по городу.

Нода — переводчик, а не алгоритм. Наружу от неё идут цели Nav2, внутрь —
детекции знаков и результаты целей. Решения принимает
:class:`~rtk2026_city_nav.controller.Controller`, и здесь нет ни одной
ветки, которая выбирала бы маневр или считала геометрию.

Обратные вызовы автомат не двигают
----------------------------------

Результат цели Nav2, запрос возобновления и детекции приходят каждый из
своего обратного вызова. Они только записывают событие, а автомат
двигается в таймере — в одном месте и за один проход.

Иначе обработка результата цели немедленно отправляла бы следующую, и то
же самое делал бы её обратный вызов: цепочка «событие — команда — событие»
уходила бы в рекурсию тем глубже, чем длиннее маршрут.

Параметры
---------

Значений по умолчанию в коде нет: параметры объявлены типами, значения
лежат в ``config/city_nav.yaml``, пути к графу и файлу поз приходят из
лаунча. Отсутствие параметра — отказ при запуске, а не молчаливо
подставленное число, с которым робот поедет не туда.

Диагностика
-----------

Задачи публикуются штатным :mod:`diagnostic_updater` в ``/diagnostics``
с именами ``CityNav/<ключ>``, как это делают мониторы
``rtk2026_observability``; их подхватывает анализатор ``CityNav``
в ``diagnostic_aggregator.yaml``.

Здесь только то, что видно изнутри: выбранный маневр и его основание,
счётчики посещений, эффективная скорость последнего участка, попытки,
уходы в остановку, отброшенные по дальности детекции, проигнорированный
запрет. Всё, что видно снаружи — присутствие ноды, lifecycle Nav2,
активность топиков — снимают штатные ``node_monitor`` и ``topic_monitor``
по ``nodes_city_nav.yaml`` и ``topics_city_nav.yaml``, и дублировать это
здесь незачем.
"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Goals
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
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
from rtk2026_city_nav.lane import LanePose
from rtk2026_city_nav.planner import ManeuverTable, Planner, RouteState
from rtk2026_city_nav.poses_io import graph_fingerprint
from rtk2026_city_nav.poses_io import load as load_poses
from rtk2026_city_nav.topology import build_topology
from rtk2026_city_nav.validate import Severity, validate
from rtk2026_pose_graph import load_geojson_path

#: События, которые обратные вызовы оставляют для таймера.
_ARRIVED = "arrived"
_FAILED = "failed"
_RESUME = "resume"


class _Task(DiagnosticTask):
    """Задача диагностики, заполняемая методом ноды.

    Один класс на все задачи: они отличаются только тем, что пишут
    в статус, и заводить под каждую свой тип было бы шумом.
    """

    def __init__(self, name: str, fill) -> None:
        super().__init__(name)
        self._fill = fill

    def run(self, status):
        self._fill(status)
        return status


class CityNavNode(Node):
    """Движение по городу: граф, знаки, Nav2."""

    def __init__(self) -> None:
        super().__init__("city_nav")

        self._declare_parameters()

        self._frame_id = str(self.get_parameter("frame_id").value)

        self._graph = load_geojson_path(str(self.get_parameter("graph_path").value))
        self._topology = build_topology(self._graph)
        self._table = ManeuverTable(
            self._topology,
            straight_tolerance_rad=math.radians(
                float(self.get_parameter("straight_tolerance_deg").value)
            ),
        )

        self._latch = Latch(
            min_box_area_px=float(self.get_parameter("min_box_area_px").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
        )
        self._manual_poses = self._load_manual_poses()

        self._start = RouteState(
            previous=int(self.get_parameter("start_previous_vertex").value),
            current=int(self.get_parameter("start_current_vertex").value),
        )
        self._report = validate(self._topology, self._table, start=self._start)
        self._log_report()

        self._controller = Controller(
            planner=Planner(self._table),
            config=ControllerConfig(
                lane_offset_m=float(self.get_parameter("lane_offset_m").value),
                pose_step_m=float(self.get_parameter("pose_step_m").value),
                traffic_side=int(self.get_parameter("traffic_side").value),
                miter_limit=float(self.get_parameter("miter_limit").value),
                max_retries=int(self.get_parameter("max_retries").value),
                default_stop_duration_s=float(
                    self.get_parameter("default_stop_duration_s").value
                ),
            ),
            route=self._start,
            latch=self._latch,
        )

        #: События от обратных вызовов, ожидающие обработки в таймере.
        self._pending: list[tuple[str, str]] = []
        #: Что помешало начать движение. Пусто, если всё в порядке.
        self._blocked = self._startup_obstacle()
        #: Сколько целей Nav2 отклонил или потерял за прогон.
        self._rejected_goals = 0
        #: Сколько участков прошли по позам из файла, а не по расчётным.
        self._manual_legs_used = 0
        self._goal: ClientGoalHandle | None = None

        self._nav_timeout_s = float(
            self.get_parameter("nav2_server_timeout_s").value
        )
        self._nav = ActionClient(
            self,
            NavigateThroughPoses,
            str(self.get_parameter("nav2_action_name").value),
        )
        self._path_publisher = self.create_publisher(PathMsg, "~/leg_path", 1)
        self._resume_service = self.create_service(
            Trigger, "~/resume", self._on_resume_request
        )
        self._detection_topic = str(self.get_parameter("detection_topic").value)
        self._detections = self.create_subscription(
            DrivingDetection, self._detection_topic, self._on_detection, 10
        )

        self._updater = Updater(
            self, period=float(self.get_parameter("diagnostic_period_s").value)
        )
        self._updater.setHardwareID("rtk2026_city_nav")
        self._updater.add(_Task("CityNav/graph", self._diag_graph))
        self._updater.add(_Task("CityNav/route", self._diag_route))
        self._updater.add(_Task("CityNav/decision", self._diag_decision))
        self._updater.add(_Task("CityNav/detections", self._diag_detections))

        if self._blocked:
            self.get_logger().error(f"движение не начато: {self._blocked}")
            return

        self._timer = self.create_timer(
            float(self.get_parameter("control_period_s").value), self._tick
        )
        self.get_logger().info(
            f"старт {self._start.previous} -> {self._start.current}, "
            f"точек решений {len(self._topology.decision_points)}, "
            f"цепочек {len(self._topology.chains)}"
        )

    # -- Параметры ---------------------------------------------------------

    def _declare_parameters(self) -> None:
        """Объявить все параметры пакета.

        Таблица имён и типов одна на пакет, см. :mod:`parameters`: ноде нужны
        все, инструментам проверки — подмножества, и разойтись они не могут.
        """
        parameters.declare(self, *parameters.SPEC)

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

        if float(self.get_parameter("min_box_area_px").value) <= 0.0:
            return (
                "min_box_area_px не задан: без порога принадлежности далёкий "
                "знак будет отнесён к текущей точке решения"
            )

        if self._report.errors and bool(
            self.get_parameter("halt_on_validation_error").value
        ):
            return (
                f"проверка графа нашла ошибок {len(self._report.errors)}: "
                "исправьте граф либо снимите halt_on_validation_error"
            )

        return ""

    def _load_manual_poses(self) -> dict[tuple[int, int], tuple[LanePose, ...]]:
        """Прочитать ручные правки поз, если файл задан.

        Берутся только записи с пометкой ``manual``. Остальные — результат
        того же расчёта, что делает исполнитель, и подменять их файлом
        значило бы держать в репозитории копию вычислимого.
        """
        raw = str(self.get_parameter("poses_path").value).strip()
        if not raw:
            return {}

        path = Path(raw)
        if not path.is_file():
            self.get_logger().warn(f"файла поз нет: {path}")
            return {}

        try:
            stored = load_poses(path)
        except (OSError, ValueError) as error:
            self.get_logger().error(f"файл поз не прочитан: {error}")
            return {}

        actual = graph_fingerprint(self._graph)
        if stored.graph_fingerprint != actual:
            self.get_logger().warn(
                "файл поз собран под другой граф "
                f"({stored.graph_fingerprint} вместо {actual}): "
                "ручные правки могли относиться к прежней геометрии"
            )

        manual = {leg.key: leg.poses for leg in stored.legs if leg.manual}
        self.get_logger().info(f"ручных правок поз загружено: {len(manual)}")
        return manual

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
                self._halt(command)

    def _follow(self, command: FollowPoses) -> None:
        decision = command.decision
        poses = self._manual_poses.get(
            (decision.state.current, decision.target), command.poses
        )
        if poses is not command.poses:
            self._manual_legs_used += 1

        self.get_logger().info(
            f"участок {decision.state.current} -> {decision.target}: "
            f"{decision.maneuver.value} по {decision.source}, поз {len(poses)}"
        )

        stamped = tuple(self._to_pose_stamped(pose) for pose in poses)
        self._publish_path(stamped)

        if not self._nav.wait_for_server(timeout_sec=self._nav_timeout_s):
            self.get_logger().warn("сервер navigate_through_poses недоступен")
            self._pending.append((_FAILED, "nav2_unavailable"))
            return

        goals = Goals()
        goals.header.frame_id = self._frame_id
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.goals = list(stamped)

        goal = NavigateThroughPoses.Goal()
        goal.poses = goals

        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _halt(self, command: Halt) -> None:
        self.get_logger().error(f"остановка: {command.reason}")
        if self._goal is not None:
            self._goal.cancel_goal_async()
            self._goal = None

    # -- Nav2 --------------------------------------------------------------

    def _on_goal_response(self, future) -> None:
        handle: ClientGoalHandle | None = future.result()

        if handle is None or not handle.accepted:
            self._rejected_goals += 1
            self.get_logger().warn("Nav2 отклонил цель")
            self._pending.append((_FAILED, "goal_rejected"))
            return

        self._goal = handle
        handle.get_result_async().add_done_callback(
            lambda done, owner=handle: self._on_goal_result(done, owner)
        )

    def _on_goal_result(self, future, owner: ClientGoalHandle) -> None:
        # Результат отменённой цели приходит и после того, как отправлена
        # новая. Своя она или чужая, видно по владельцу.
        if self._goal is not owner:
            return

        self._goal = None
        result = future.result()

        if result is None:
            self._rejected_goals += 1
            self._pending.append((_FAILED, "no_result"))
            return

        status = int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._pending.append((_ARRIVED, ""))
            return

        self._pending.append((_FAILED, f"nav2_status_{status}"))

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

    # -- Детекции ----------------------------------------------------------

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

    # -- Публикации --------------------------------------------------------

    def _to_pose_stamped(self, pose: LanePose) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(pose.x)
        message.pose.position.y = float(pose.y)
        message.pose.orientation.z = math.sin(0.5 * float(pose.yaw))
        message.pose.orientation.w = math.cos(0.5 * float(pose.yaw))
        return message

    def _publish_path(self, poses: tuple[PoseStamped, ...]) -> None:
        message = PathMsg()
        message.header.frame_id = self._frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = list(poses)
        self._path_publisher.publish(message)

    # -- Диагностика -------------------------------------------------------

    def _log_report(self) -> None:
        self.get_logger().info(self._report.summary())
        for finding in self._report.findings:
            text = str(finding)
            if finding.severity is Severity.ERROR:
                self.get_logger().error(text)
            else:
                self.get_logger().warn(text)

    def _diag_graph(self, status) -> None:
        """Итог проверки графа. Считается один раз при загрузке."""
        status.add("Точек решений", str(len(self._topology.decision_points)))
        status.add("Цепочек", str(len(self._topology.chains)))
        status.add("Состояний в таблице", str(len(self._table.states)))
        status.add("Ошибок проверки", str(len(self._report.errors)))
        status.add("Предупреждений проверки", str(len(self._report.warnings)))
        status.add("Ручных правок поз", str(len(self._manual_poses)))

        if self._blocked:
            status.summary(DiagnosticStatus.ERROR, self._blocked)
        elif self._report.errors:
            status.summary(DiagnosticStatus.ERROR, self._report.summary())
        elif self._report.warnings:
            status.summary(DiagnosticStatus.WARN, self._report.summary())
        else:
            status.summary(DiagnosticStatus.OK, "граф пригоден для движения")

    def _diag_route(self, status) -> None:
        """Где робот в маршруте и как быстро прошёл последний участок."""
        controller = self._controller
        route = controller.route

        status.add("Состояние", controller.state.value)
        status.add("Приехал из", str(route.previous))
        status.add("Находится в", str(route.current))
        status.add("Попытка", str(controller.attempt))
        status.add("Уходов в остановку", str(controller.recover_count))
        status.add("Отклонённых целей Nav2", str(self._rejected_goals))
        status.add("Участков по ручным позам", str(self._manual_legs_used))

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
                DiagnosticStatus.ERROR,
                f"движение остановлено: {controller.reason}",
            )
        elif controller.state is ControllerState.WAIT:
            status.summary(DiagnosticStatus.OK, f"стоим: {controller.reason}")
        elif controller.attempt > 0:
            status.summary(
                DiagnosticStatus.WARN,
                f"повтор участка, попытка {controller.attempt}",
            )
        else:
            status.summary(DiagnosticStatus.OK, "движение идёт")

    def _diag_decision(self, status) -> None:
        """Последний выбор маневра и что было на выбор."""
        controller = self._controller
        record = controller.leg or controller.last_leg

        if record is None:
            status.summary(DiagnosticStatus.OK, "выбор ещё не делался")
            return

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

    def _diag_detections(self, status) -> None:
        """Что накоплено к предстоящему выбору."""
        latch = self._latch

        publishers = self.count_publishers(self._detection_topic)

        status.add("Topic", self._detection_topic)
        status.add("Publishers", str(publishers))
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
        else:
            status.summary(DiagnosticStatus.OK, "детекции принимаются")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CityNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
