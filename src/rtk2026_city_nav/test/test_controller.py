import math

import pytest

from rtk2026_city_nav.controller import (
    Controller,
    ControllerConfig,
    ControllerState,
    FollowPoses,
    Halt,
    LegOutcome,
    Wait,
)
from rtk2026_city_nav.detections import BUS_STOP, STOP_SIGN, Latch
from rtk2026_city_nav.maneuver import Maneuver
from rtk2026_city_nav.planner import ManeuverTable, Planner, RouteState
from rtk2026_city_nav.topology import build_topology
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph

TOL = math.radians(30.0)

#: Две развязки, соединённые между собой, у каждой по три отвода.
#:
#:         21            22
#:          |             |
#:   40 -- 10 ---------- 20 -- 41
#:          |             |
#:         31            32
NODES: dict[int, tuple[float, float]] = {
    10: (0.0, 0.0),
    20: (2.0, 0.0),
    21: (0.0, 1.0),
    31: (0.0, -1.0),
    40: (-1.0, 0.0),
    22: (2.0, 1.0),
    32: (2.0, -1.0),
    41: (3.0, 0.0),
}
EDGES: list[tuple[int, int, int]] = [
    (1, 10, 20),
    (2, 10, 21),
    (3, 10, 31),
    (4, 10, 40),
    (5, 20, 22),
    (6, 20, 32),
    (7, 20, 41),
]


def _controller(**overrides: object) -> Controller:
    graph = RoadGraph(
        nodes={nid: Node(node_id=nid, x=xy[0], y=xy[1]) for nid, xy in NODES.items()},
        edges={
            eid: OrientedEdge(
                edge_id=eid, start_id=a, end_id=b, polyline_xy=(NODES[a], NODES[b])
            )
            for eid, a, b in EDGES
        },
    )
    config = ControllerConfig(lane_offset_m=0.2, **overrides)  # type: ignore[arg-type]
    return Controller(
        planner=Planner(ManeuverTable(build_topology(graph), straight_tolerance_rad=TOL)),
        config=config,
        route=RouteState(previous=40, current=10),
        latch=Latch(min_box_area_px=0.0),
    )


def _only(commands: tuple[object, ...]) -> object:
    assert len(commands) == 1, f"ожидалась одна команда, получено {commands}"
    return commands[0]


def test_starts_in_plan_and_first_poll_publishes_poses() -> None:
    controller = _controller()

    assert controller.state is ControllerState.PLAN

    command = _only(controller.poll(now_s=0.0))

    assert isinstance(command, FollowPoses)
    assert controller.state is ControllerState.TRACK
    assert command.poses


def test_poses_are_offset_into_the_lane() -> None:
    controller = _controller()

    command = _only(controller.poll(now_s=0.0))
    assert isinstance(command, FollowPoses)

    # Едем на восток по линии y = 0, полоса справа значит южнее.
    assert all(pose.y == pytest.approx(-0.2) for pose in command.poses)


def test_track_ignores_further_polls() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)

    # Пока едем, автомат ничего не делает: он ждёт события.
    assert controller.poll(now_s=1.0) == ()
    assert controller.state is ControllerState.TRACK


def test_sign_during_track_does_not_touch_the_current_leg() -> None:
    """Знак относится к выбору в приближающейся вершине, не к текущему участку."""
    controller = _controller()
    first = _only(controller.poll(now_s=0.0))
    assert isinstance(first, FollowPoses)

    controller.latch.observe_route("left_only", box_area_px=1000.0)

    # Ни новых команд, ни смены состояния: геометрия участка не меняется.
    assert controller.poll(now_s=0.5) == ()
    assert controller.state is ControllerState.TRACK


def test_sign_during_track_applies_at_the_next_vertex() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)
    assert controller.leg is not None
    assert controller.leg.decision.target == 20

    controller.latch.observe_route("left_only", box_area_px=1000.0)

    following = _only(controller.on_arrived(now_s=5.0))

    assert isinstance(following, FollowPoses)
    # В вершине 20 знак сработал: налево это отвод 22.
    assert following.decision.maneuver is Maneuver.LEFT
    assert following.decision.target == 22


def test_arrival_shifts_the_pair_and_counts_the_visit() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)

    controller.on_arrived(now_s=4.0)

    assert controller.route.previous == 10
    assert controller.route.current == 20
    assert controller.planner.visits[20] == 1


def test_arrival_clears_the_latch() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)
    controller.latch.observe_route("left_only", box_area_px=1000.0)

    controller.on_arrived(now_s=4.0)

    # Знак применён и забыт: к следующей вершине он не относится.
    assert controller.latch.advice().is_empty


def test_leg_record_holds_timing_and_speed() -> None:
    controller = _controller()
    controller.poll(now_s=10.0)
    controller.on_arrived(now_s=14.0)

    leg = controller.last_leg
    assert leg is not None
    assert leg.outcome is LegOutcome.ARRIVED
    assert leg.length_m == pytest.approx(2.0)
    assert leg.duration_s == pytest.approx(4.0)
    assert leg.speed_mps == pytest.approx(0.5)


def test_unfinished_leg_has_no_duration() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)

    assert controller.leg is not None
    assert controller.leg.duration_s is None
    assert controller.leg.speed_mps is None


def test_stop_sign_makes_the_robot_wait_at_the_vertex() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)
    controller.latch.observe_stop("stop", duration_s=3.0, box_area_px=1000.0)

    command = _only(controller.on_arrived(now_s=4.0))

    assert isinstance(command, Wait)
    assert command.duration_s == pytest.approx(3.0)
    assert command.reason == STOP_SIGN
    assert controller.state is ControllerState.WAIT


def test_wait_holds_until_the_deadline_then_plans() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)
    controller.latch.observe_stop("stop", duration_s=3.0, box_area_px=1000.0)
    controller.on_arrived(now_s=4.0)

    # Срок не вышел: стоим.
    assert controller.poll(now_s=6.0) == ()
    assert controller.state is ControllerState.WAIT

    command = _only(controller.poll(now_s=7.0))

    assert isinstance(command, FollowPoses)
    assert controller.state is ControllerState.TRACK


def test_bus_stop_uses_the_default_duration() -> None:
    """У автобусной остановки своей длительности нет."""
    controller = _controller(default_stop_duration_s=5.0)
    controller.poll(now_s=0.0)
    controller.latch.observe_bus(box_area_px=1000.0)

    command = _only(controller.on_arrived(now_s=1.0))

    assert isinstance(command, Wait)
    assert command.reason == BUS_STOP
    assert command.duration_s == pytest.approx(5.0)


def test_failure_retries_the_leg() -> None:
    controller = _controller(max_retries=2)
    controller.poll(now_s=0.0)

    command = _only(controller.on_failed(now_s=1.0))

    assert isinstance(command, FollowPoses)
    assert controller.state is ControllerState.TRACK
    assert controller.attempt == 1
    # Счётчики не менялись, поэтому выбор тот же.
    assert command.decision.target == 20


def test_retries_run_out_and_halt() -> None:
    controller = _controller(max_retries=2)
    controller.poll(now_s=0.0)

    controller.on_failed(now_s=1.0)
    controller.on_failed(now_s=2.0)
    command = _only(controller.on_failed(now_s=3.0))

    assert isinstance(command, Halt)
    assert controller.state is ControllerState.RECOVER
    assert controller.recover_count == 1
    assert controller.last_leg is not None
    assert controller.last_leg.outcome is LegOutcome.FAILED


def test_each_attempt_gets_its_own_record() -> None:
    controller = _controller(max_retries=2)
    controller.poll(now_s=0.0)
    assert controller.leg is not None and controller.leg.attempt == 0

    controller.on_failed(now_s=1.0)

    assert controller.leg is not None and controller.leg.attempt == 1


def test_resume_from_recover_replans() -> None:
    controller = _controller(max_retries=0)
    controller.poll(now_s=0.0)
    controller.on_failed(now_s=1.0)
    assert controller.state is ControllerState.RECOVER

    command = _only(controller.on_resume(now_s=2.0))

    assert isinstance(command, FollowPoses)
    assert controller.state is ControllerState.TRACK
    assert controller.attempt == 0


def test_arrival_resets_the_attempt_counter() -> None:
    controller = _controller(max_retries=3)
    controller.poll(now_s=0.0)
    controller.on_failed(now_s=1.0)
    assert controller.attempt == 1

    controller.on_arrived(now_s=2.0)

    assert controller.attempt == 0


def test_events_outside_track_are_ignored() -> None:
    controller = _controller()

    # В PLAN ещё ничего не едет.
    assert controller.on_arrived(now_s=0.0) == ()
    assert controller.on_failed(now_s=0.0) == ()

    controller.poll(now_s=0.0)
    controller.on_arrived(now_s=1.0)
    # Уже уехали на следующий участок: повторное прибытие не считается.
    state_before = controller.route
    controller.on_failed(now_s=1.5)
    controller.on_arrived(now_s=2.0)
    assert controller.route != state_before or controller.state is ControllerState.TRACK


def test_resume_outside_recover_is_ignored() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)

    assert controller.on_resume(now_s=1.0) == ()
    assert controller.state is ControllerState.TRACK


def test_dead_end_state_without_maneuvers_halts() -> None:
    """Состояние, которого нет в таблице: ехать некуда."""
    controller = _controller()
    controller.route = RouteState(previous=999, current=888)

    command = _only(controller.poll(now_s=0.0))

    assert isinstance(command, Halt)
    assert controller.state is ControllerState.RECOVER
    assert controller.recover_count == 1


def test_pose_step_densifies_the_leg() -> None:
    sparse = _controller()
    dense = _controller(pose_step_m=0.5)

    sparse_command = _only(sparse.poll(now_s=0.0))
    dense_command = _only(dense.poll(now_s=0.0))
    assert isinstance(sparse_command, FollowPoses)
    assert isinstance(dense_command, FollowPoses)

    assert len(dense_command.poses) > len(sparse_command.poses)


def test_left_hand_traffic_mirrors_the_lane() -> None:
    controller = _controller(traffic_side=-1)

    command = _only(controller.poll(now_s=0.0))
    assert isinstance(command, FollowPoses)

    assert all(pose.y == pytest.approx(+0.2) for pose in command.poses)


def test_route_sign_survives_the_stop_and_applies_after_it() -> None:
    """Стоп-знак и знак маневра приезжают вместе, потребляются в разное время."""
    controller = _controller(default_stop_duration_s=2.0)
    controller.poll(now_s=0.0)

    controller.latch.observe_route("left_only", box_area_px=1000.0)
    controller.latch.observe_stop("stop", duration_s=2.0, box_area_px=1000.0)

    # Прибытие исполняет остановку.
    waiting = _only(controller.on_arrived(now_s=4.0))
    assert isinstance(waiting, Wait)
    # Знак маневра при этом ещё действует.
    assert controller.latch.advice().prefer is Maneuver.LEFT

    # Ожидание кончилось - знак применяется к выбору.
    following = _only(controller.poll(now_s=6.0))
    assert isinstance(following, FollowPoses)
    assert following.decision.maneuver is Maneuver.LEFT
    assert following.decision.target == 22

    # И только теперь забыт.
    assert controller.latch.advice().is_empty


def test_route_sign_is_consumed_by_the_decision_it_influenced() -> None:
    controller = _controller()
    controller.poll(now_s=0.0)
    controller.latch.observe_route("left_only", box_area_px=1000.0)

    controller.on_arrived(now_s=1.0)

    # Знак сработал один раз и больше не действует на последующие вершины.
    assert controller.latch.advice().is_empty
