"""Сквозной прогон: граф, знаки, выбор, автомат, позы — вместе.

Модульные тесты проверяют каждую часть на маленьких искусственных графах.
Здесь проверяется то, что видно только на длинном прогоне и на настоящем
графе трассы: что робот не встаёт, не зацикливается на двух вершинах, что
знак действует на следующий выбор, а не на текущий участок, и что у каждого
участка есть позы.

Роль исполнителя играет функция, отвечающая «доехал» либо «не смог».
Ответ попадает в очередь событий и разбирается следующим шагом — так же,
как это делает нода: команда может породить событие, и обрабатывать его
в том же проходе значило бы уходить в рекурсию на всю длину маршрута.
"""

from __future__ import annotations

import collections
import math
from pathlib import Path

import pytest

from rtk2026_city_nav.controller import (
    Command,
    Controller,
    ControllerConfig,
    ControllerState,
    FollowPoses,
    Halt,
    Wait,
)
from rtk2026_city_nav.detections import STOP_SIGN, Latch
from rtk2026_city_nav.lane import resample_along_chain
from rtk2026_city_nav.maneuver import Maneuver
from rtk2026_city_nav.planner import DecisionSource, ManeuverTable, Planner, RouteState
from rtk2026_city_nav.topology import build_topology
from rtk2026_pose_graph import load_geojson_path
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph

#: Граф трассы проекта. Лежит в пакете, который ещё не переписан.
REAL_GRAPH = (
    Path(__file__).resolve().parents[2] / "rtk2026_route_nav" / "config" / "graph.geojson"
)

#: Порог принадлежности знака. В прогоне без перцепции величина не важна,
#: важно что он задан: без него накопитель детекции не примет.
BOX_AREA_PX = 1000.0

ARRIVED = "arrived"
FAILED = "failed"

#: Смещение полосы во всех прогонах.
LANE_OFFSET_M = 0.2


def _controller(graph: RoadGraph, start: RouteState, **overrides) -> Controller:
    params = {"lane_offset_m": LANE_OFFSET_M, "pose_step_m": 0.4}
    params.update(overrides)
    return Controller(
        planner=Planner(ManeuverTable(build_topology(graph))),
        config=ControllerConfig(**params),
        route=start,
        latch=Latch(min_box_area_px=BOX_AREA_PX),
    )


class Run:
    """Прогон автомата с подставным исполнителем.

    Время идёт по секунде на шаг: от настоящих часов тест не зависит, а на
    выводы длительность участка не влияет.
    """

    def __init__(self, controller: Controller, *, succeeds: bool = True) -> None:
        self.controller = controller
        self.reply = ARRIVED if succeeds else FAILED
        self.now_s = 0.0
        self.pending: list[str] = []
        #: Участки в порядке отправки: откуда, куда, маневр, основание.
        self.legs: list[tuple[int, int, Maneuver, str]] = []
        self.pose_counts: list[int] = []
        self.waits: list[str] = []
        self.halts: list[str] = []

    def step(self) -> None:
        """Шаг: разобрать накопленные события, затем продвинуть автомат."""
        self.now_s += 1.0
        events, self.pending = self.pending, []

        for event in events:
            if event == ARRIVED:
                self._execute(self.controller.on_arrived(self.now_s))
            elif event == FAILED:
                self._execute(self.controller.on_failed(self.now_s, reason="nav2"))

        self._execute(self.controller.poll(self.now_s))

    def steps(self, count: int) -> None:
        for _ in range(count):
            self.step()

    def wait_out(self, seconds: float) -> None:
        """Переждать остановку, не тратя шагов на ожидание."""
        self.now_s += seconds

    def drive(self, legs: int) -> None:
        """Пройти заданное число участков."""
        # Участок занимает два шага: отправка и разбор ответа.
        limit = legs * 4 + 20
        for _ in range(limit):
            if len(self.legs) >= legs:
                return
            self.step()

        raise AssertionError(
            f"автомат встал: участков {len(self.legs)} из {legs}, "
            f"состояние {self.controller.state.value}, остановки {self.halts}"
        )

    def _execute(self, commands: tuple[Command, ...]) -> None:
        for command in commands:
            if isinstance(command, FollowPoses):
                decision = command.decision
                self.legs.append(
                    (
                        decision.state.current,
                        decision.target,
                        decision.maneuver,
                        decision.source,
                    )
                )
                self.pose_counts.append(len(command.poses))
                self.pending.append(self.reply)
            elif isinstance(command, Wait):
                self.waits.append(command.reason)
            elif isinstance(command, Halt):
                self.halts.append(command.reason)


# -- Настоящий граф трассы ------------------------------------------------


@pytest.fixture(scope="module")
def real_graph() -> RoadGraph:
    if not REAL_GRAPH.is_file():
        pytest.skip(f"графа трассы нет: {REAL_GRAPH}")
    return load_geojson_path(str(REAL_GRAPH))


@pytest.fixture(scope="module")
def real_states(real_graph: RoadGraph) -> tuple[RouteState, ...]:
    return ManeuverTable(build_topology(real_graph)).states


def test_every_state_of_the_real_graph_can_start_a_long_run(
    real_graph: RoadGraph, real_states: tuple[RouteState, ...]
) -> None:
    """Из любого состояния трассы робот едет и не встаёт.

    Проверяется каждое состояние, а не одно выбранное: начальное положение
    зависит от того, где робота поставили перед прогоном.
    """
    assert real_states, "в таблице маневров нет ни одного состояния"

    for start in real_states:
        run = Run(_controller(real_graph, start))
        run.drive(40)

        assert run.halts == [], f"из {start} остановка: {run.halts}"
        assert all(count > 0 for count in run.pose_counts)


def test_consecutive_legs_of_the_real_graph_join_end_to_end(
    real_graph: RoadGraph, real_states: tuple[RouteState, ...]
) -> None:
    """Каждый участок начинается там, где кончился предыдущий."""
    run = Run(_controller(real_graph, real_states[0]))
    run.drive(60)

    for (_, target, _, _), (start, _, _, _) in zip(run.legs, run.legs[1:]):
        assert start == target


def test_the_real_graph_does_not_trap_the_robot_on_two_vertices(
    real_graph: RoadGraph, real_states: tuple[RouteState, ...]
) -> None:
    """Выбор по покрытию разводит маршрут, а не гоняет туда-обратно.

    Без учёта посещений робот застрял бы на паре вершин: маршрут по условию
    задачи недетерминированный, и повторно попадая в ту же точку выбора он
    обязан идти другим путём.
    """
    run = Run(_controller(real_graph, real_states[0]))
    run.drive(120)

    visited = collections.Counter(target for _, target, _, _ in run.legs)
    decision_points = build_topology(real_graph).decision_points

    assert set(visited) == decision_points, (
        "за 120 участков посещены не все точки решений: пропущены "
        f"{sorted(decision_points - set(visited))}"
    )
    assert max(visited.values()) <= 3 * min(visited.values()), (
        f"посещения разошлись: {dict(sorted(visited.items()))}"
    )


def test_uturns_on_the_real_graph_are_only_ever_a_last_resort(
    real_graph: RoadGraph, real_states: tuple[RouteState, ...]
) -> None:
    """Разворот выбирается только там, где больше нечего выбрать."""
    run = Run(_controller(real_graph, real_states[0]))
    run.drive(120)

    for start, target, maneuver, source in run.legs:
        if maneuver is Maneuver.UTURN:
            assert source == DecisionSource.UTURN_FALLBACK.value, (
                f"разворот {start} -> {target} выбран не как безальтернативный"
            )


def test_opposite_directions_of_the_real_graph_get_opposite_lanes(
    real_graph: RoadGraph,
) -> None:
    """Позы участка и участка в обратную сторону лежат по разные стороны.

    Это и есть правило движения: сторона выводится из порядка пары вершин,
    отдельной переменной полосы нет.
    """
    topology = build_topology(real_graph)
    by_ends = {(c.start, c.end): c for c in topology.chains}

    checked = 0
    for (start, end), chain in by_ends.items():
        backward = by_ends.get((end, start))
        if backward is None:
            continue

        forward_poses = resample_along_chain(
            chain.polyline_xy, lane_offset_m=LANE_OFFSET_M, step_m=0.0
        )
        backward_poses = resample_along_chain(
            backward.polyline_xy, lane_offset_m=LANE_OFFSET_M, step_m=0.0
        )
        assert forward_poses and backward_poses

        # Первая поза одного направления и последняя другого стоят у одной
        # вершины, но на разных полосах: между ними двойное смещение.
        gap = math.dist(
            (forward_poses[0].x, forward_poses[0].y),
            (backward_poses[-1].x, backward_poses[-1].y),
        )
        assert gap == pytest.approx(2 * LANE_OFFSET_M, abs=0.05), (
            f"цепочка {start} -> {end}: полосы не разошлись, {gap:.3f} м"
        )
        checked += 1

    assert checked > 0


# -- Знаки в сквозном прогоне ---------------------------------------------


def _cross() -> RoadGraph:
    """Перекрёсток с четырьмя отводами: из центра можно во все стороны."""
    nodes = {
        0: (0.0, 0.0),
        10: (2.0, 0.0),
        20: (0.0, 2.0),
        30: (0.0, -2.0),
        40: (-2.0, 0.0),
    }
    return RoadGraph(
        nodes={nid: Node(node_id=nid, x=xy[0], y=xy[1]) for nid, xy in nodes.items()},
        edges={
            eid: OrientedEdge(
                edge_id=eid,
                start_id=0,
                end_id=other,
                polyline_xy=(nodes[0], nodes[other]),
            )
            for eid, other in enumerate((10, 20, 30, 40), start=1)
        },
    )


def test_a_sign_seen_while_driving_applies_to_the_next_choice() -> None:
    """Знак действует на выбор в приближающейся вершине, не на текущий участок.

    Геометрия участка, по которому робот уже едет, от знака не меняется —
    меняется то, куда он поедет из вершины.
    """
    controller = _controller(_cross(), RouteState(previous=0, current=30))
    run = Run(controller)

    run.step()
    # Из тупика 30 выехать можно только назад в 0.
    assert run.legs[-1][:2] == (30, 0)
    poses_of_current_leg = run.pose_counts[-1]

    # Знак замечен уже в движении, до прибытия в 0.
    assert controller.latch.observe_route("right_only", box_area_px=BOX_AREA_PX)

    run.step()

    assert run.pose_counts[0] == poses_of_current_leg, (
        "геометрия участка, по которому уже едут, изменилась"
    )
    start, _, maneuver, source = run.legs[-1]
    assert start == 0
    assert maneuver is Maneuver.RIGHT
    assert source == DecisionSource.SIGN.value


def test_a_maneuver_sign_survives_the_wait_at_a_stop_line() -> None:
    """Остановка потребляется у вершины, знак маневра — при выборе.

    Иначе знак, замеченный вместе со стоп-линией, терялся бы за время
    ожидания, и робот поехал бы куда попало.
    """
    controller = _controller(_cross(), RouteState(previous=0, current=30))
    run = Run(controller)

    run.step()
    assert run.legs[-1][:2] == (30, 0)

    # Оба знака попадают в кадр на подъезде к вершине: и стоп-линия,
    # и предписание маневра. Выставить их раньше нельзя — знак относится
    # к ближайшему выбору, и первый же выбор его бы потребил.
    controller.latch.observe_route("left_only", box_area_px=BOX_AREA_PX)
    controller.latch.observe_stop(STOP_SIGN, duration_s=3.0, box_area_px=BOX_AREA_PX)

    # Прибытие в 0: встаём по стоп-знаку.
    run.step()
    assert run.waits == [STOP_SIGN]
    assert controller.state is ControllerState.WAIT

    legs_while_standing = len(run.legs)
    run.step()
    assert len(run.legs) == legs_while_standing, "поехали, не выждав срок"

    run.wait_out(5.0)
    run.step()

    _, _, maneuver, source = run.legs[-1]
    assert maneuver is Maneuver.LEFT
    assert source == DecisionSource.SIGN.value


def test_a_prohibition_only_crosses_out_and_choice_goes_on_by_coverage() -> None:
    """Запрещающий знак вычёркивает вариант, ничего не предлагая взамен."""
    controller = _controller(_cross(), RouteState(previous=0, current=30))
    run = Run(controller)
    run.step()

    controller.latch.observe_route("no_straight", box_area_px=BOX_AREA_PX)
    run.step()

    start, _, maneuver, source = run.legs[-1]
    assert start == 0
    assert maneuver is not Maneuver.STRAIGHT
    # После запрета выбор идёт обычным порядком, а не «по знаку».
    assert source == DecisionSource.COVERAGE.value


def test_retries_are_spent_before_the_robot_stops() -> None:
    """Отказ Nav2 повторяется, и только потом остановка."""
    controller = _controller(
        _cross(), RouteState(previous=0, current=30), max_retries=2
    )
    run = Run(controller, succeeds=False)

    run.steps(10)

    # Первая отправка плюс две попытки, затем остановка.
    assert len(run.legs) == 3, f"попыток {len(run.legs)}, ожидалось 3"
    assert run.halts == ["nav2"], f"остановок {run.halts}, ожидалась одна"
    assert controller.state is ControllerState.RECOVER
    assert controller.recover_count == 1


def test_resume_after_a_stop_puts_the_robot_back_on_the_route() -> None:
    """Возобновление возвращает автомат к планированию."""
    controller = _controller(
        _cross(), RouteState(previous=0, current=30), max_retries=0
    )
    run = Run(controller, succeeds=False)
    run.steps(4)
    assert controller.state is ControllerState.RECOVER

    # Теперь исполнитель отвечает успехом.
    run.reply = ARRIVED
    run.now_s += 1.0
    run._execute(controller.on_resume(run.now_s))

    assert controller.state is ControllerState.TRACK
    assert controller.attempt == 0
