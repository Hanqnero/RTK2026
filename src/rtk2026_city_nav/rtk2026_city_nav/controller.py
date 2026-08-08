"""Автомат исполнителя: события снаружи, команды наружу.

Модуль не знает ни про ROS, ни про Nav2, ни про часы: время приходит
параметром, а команды возвращаются значениями. Поэтому автомат целиком
проверяется прогоном последовательности событий, без робота.

Состояния
---------

``PLAN``
    Пара вершин известна, надо выбрать маневр и построить позы.

``TRACK``
    Позы отданы исполнителю, робот едет. Детекции знаков только копятся:
    знак относится к выбору в приближающейся вершине, а не к текущему
    участку, поэтому геометрия уже опубликованного участка не меняется.

``WAIT``
    Робот стоит у вершины по знаку остановки или у автобусной.

``RECOVER``
    Участок не исполнился и попытки кончились. Дальше нужно вмешательство.

Состояния ``ADVANCE`` из описания алгоритма здесь нет: из него всегда
уходят немедленно, то есть это ветка кода, а не состояние. Сдвиг пары
вершин и счётчика посещений происходит при обработке прибытия.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from rtk2026_city_nav.detections import Latch, StopRequest
from rtk2026_city_nav.lane import LanePose, RIGHT_HAND_TRAFFIC, resample_along_chain
from rtk2026_city_nav.planner import Decision, NoManeuverAvailable, Planner, RouteState
from rtk2026_pose_graph.geometry import polyline_length_m


class ControllerState(str, Enum):
    """Состояния автомата."""

    PLAN = "plan"
    TRACK = "track"
    WAIT = "wait"
    RECOVER = "recover"


class LegOutcome(str, Enum):
    """Чем закончился участок."""

    ARRIVED = "arrived"
    FAILED = "failed"


@dataclass(frozen=True)
class FollowPoses:
    """Ехать по этим позам. Нода превращает это в цель Nav2."""

    poses: tuple[LanePose, ...]
    decision: Decision


@dataclass(frozen=True)
class Wait:
    """Стоять указанное время."""

    duration_s: float
    #: Причина из :mod:`rtk2026_city_nav.detections`.
    reason: str


@dataclass(frozen=True)
class Halt:
    """Прекратить движение: дальше без вмешательства нельзя."""

    reason: str


#: Что автомат может попросить сделать.
Command = FollowPoses | Wait | Halt


@dataclass
class ControllerConfig:
    """Настройки исполнителя. Свойства робота и политики, не трассы."""

    #: Смещение центра полосы от разметочной линии, метры.
    lane_offset_m: float
    #: Шаг постановки поз вдоль цепочки, метры. Ноль — только точки полилинии.
    pose_step_m: float = 0.0
    #: Правостороннее либо левостороннее движение.
    traffic_side: int = RIGHT_HAND_TRAFFIC
    #: Предел выноса позы в остром изломе, в долях смещения.
    miter_limit: float = 2.0
    #: Сколько раз повторить участок, прежде чем уйти в RECOVER.
    #: Отказы Nav2 бывают преходящими, и сразу останавливаться незачем.
    max_retries: int = 2
    #: Длительность остановки, если знак её не задал (например, автобусная).
    default_stop_duration_s: float = 2.0


@dataclass
class LegRecord:
    """Засечки времени по участку. Отсюда берётся диагностика.

    Реальный расход времени здесь один — исполнение участка. Выбор маневра
    и построение поз занимают микросекунды, но засекаются тоже: если они
    вдруг вырастут, это будет видно, а не останется догадкой.
    """

    decision: Decision
    #: Когда решение принято.
    decided_at_s: float
    #: Длина цепочки по полилинии, метры.
    length_m: float
    #: Сколько времени заняли выбор и построение поз.
    planning_s: float
    #: Какая это попытка для данного участка, начиная с нуля.
    attempt: int
    finished_at_s: float | None = None
    outcome: LegOutcome | None = None

    @property
    def duration_s(self) -> float | None:
        """Время исполнения участка; ``None``, пока не закончен."""
        if self.finished_at_s is None:
            return None
        return self.finished_at_s - self.decided_at_s

    @property
    def speed_mps(self) -> float | None:
        """Эффективная скорость: длина участка на время его прохождения."""
        duration = self.duration_s
        if duration is None or duration <= 0.0:
            return None
        return self.length_m / duration


@dataclass
class Controller:
    """Автомат исполнителя.

    Нода вызывает :meth:`poll` в своём цикле и сообщает о событиях
    методами ``on_*``. Всё, что автомат хочет сделать, возвращается
    списком команд — сам он ничего не выполняет.
    """

    planner: Planner
    config: ControllerConfig
    route: RouteState
    latch: Latch

    state: ControllerState = ControllerState.PLAN
    #: Текущий участок, пока он не закончен.
    leg: LegRecord | None = field(default=None, init=False)
    #: Последний законченный участок. Нужен диагностике.
    last_leg: LegRecord | None = field(default=None, init=False)
    #: Сколько раз подряд повторяли текущий участок.
    attempt: int = field(default=0, init=False)
    #: До какого времени стоять в WAIT.
    wait_until_s: float | None = field(default=None, init=False)
    #: Почему стоим либо почему остановились.
    reason: str = field(default="", init=False)
    #: Сколько раз уходили в RECOVER за прогон.
    recover_count: int = field(default=0, init=False)

    def poll(self, now_s: float) -> tuple[Command, ...]:
        """Продвинуть автомат по времени.

        В ``PLAN`` выбирает маневр и отдаёт позы, в ``WAIT`` ждёт срока.
        В ``TRACK`` и ``RECOVER`` ничего не делает: там ждут события.
        """
        if self.state is ControllerState.PLAN:
            return self._plan(now_s)

        if self.state is ControllerState.WAIT:
            if self.wait_until_s is not None and now_s >= self.wait_until_s:
                self.wait_until_s = None
                self.reason = ""
                self.state = ControllerState.PLAN
                return self._plan(now_s)

        return ()

    def on_arrived(self, now_s: float) -> tuple[Command, ...]:
        """Робот доехал до цели участка.

        Здесь происходит сдвиг: цель становится текущей вершиной, счётчик
        её посещений растёт.

        Требование остановки исполняется тут же и на этом потребляется.
        Знак маневра, наоборот, остаётся: он относится к выбору, который
        ещё предстоит, и обязан пережить ожидание у знака остановки.
        """
        if self.state is not ControllerState.TRACK or self.leg is None:
            return ()

        decision = self.leg.decision
        self._finish_leg(now_s, LegOutcome.ARRIVED)
        self.route = self.planner.commit(decision)
        self.attempt = 0

        # Остановка исполняется здесь, у вершины, и на этом потребляется.
        # Знак маневра не трогаем: он нужен предстоящему выбору и должен
        # пережить ожидание.
        stop = self.latch.stop_request()
        self.latch.clear_stop()

        if stop is not None:
            return self._begin_wait(now_s, stop)

        self.state = ControllerState.PLAN
        return self.poll(now_s)

    def on_failed(self, now_s: float, *, reason: str = "nav2") -> tuple[Command, ...]:
        """Исполнитель не смог пройти участок.

        Попытка повторяется до :attr:`ControllerConfig.max_retries`:
        отказы Nav2 бывают преходящими, и уходить в остановку с первого
        раза незачем.

        Повтор проходит через обычный выбор заново. Счётчики посещений при
        отказе не менялись, поэтому выбор выходит тот же — если только за
        это время не был распознан знак, и тогда смена маневра оправдана.
        Каждая попытка попадает в свою запись участка, так что в
        диагностике видно и число попыток, и что менялось между ними.
        """
        if self.state is not ControllerState.TRACK or self.leg is None:
            return ()

        self._finish_leg(now_s, LegOutcome.FAILED)

        if self.attempt < self.config.max_retries:
            self.attempt += 1
            self.state = ControllerState.PLAN
            return self.poll(now_s)

        self.recover_count += 1
        self.reason = reason
        self.state = ControllerState.RECOVER
        return (Halt(reason=reason),)

    def on_resume(self, now_s: float) -> tuple[Command, ...]:
        """Возобновить после ``RECOVER``. Счётчик попыток сбрасывается."""
        if self.state is not ControllerState.RECOVER:
            return ()

        self.attempt = 0
        self.reason = ""
        self.state = ControllerState.PLAN
        return self.poll(now_s)

    @property
    def leg_decision(self) -> Decision:
        """Решение текущего либо последнего участка."""
        record = self.leg or self.last_leg
        if record is None:
            raise RuntimeError("участок ещё не планировался")
        return record.decision

    def _plan(self, now_s: float) -> tuple[Command, ...]:
        """Выбрать маневр и построить позы."""
        started_s = now_s

        try:
            decision = self.planner.decide(self.route, advice=self.latch.advice())
        except NoManeuverAvailable as error:
            self.recover_count += 1
            self.reason = str(error)
            self.state = ControllerState.RECOVER
            return (Halt(reason=self.reason),)

        poses = resample_along_chain(
            decision.chain.polyline_xy,
            lane_offset_m=self.config.lane_offset_m,
            step_m=self.config.pose_step_m,
            traffic_side=self.config.traffic_side,
            miter_limit=self.config.miter_limit,
        )

        if not poses:
            self.recover_count += 1
            self.reason = (
                f"цепочка {decision.state.current} -> {decision.target} "
                "не дала ни одной позы"
            )
            self.state = ControllerState.RECOVER
            return (Halt(reason=self.reason),)

        self.leg = LegRecord(
            decision=decision,
            decided_at_s=now_s,
            length_m=polyline_length_m(decision.chain.polyline_xy),
            planning_s=now_s - started_s,
            attempt=self.attempt,
        )
        self.state = ControllerState.TRACK
        # Знак маневра использован в решении и больше не действует.
        self.latch.clear_route()

        return (FollowPoses(poses=poses, decision=decision),)

    def _begin_wait(self, now_s: float, stop: StopRequest) -> tuple[Command, ...]:
        """Встать у вершины по требованию знака."""
        duration = stop.duration_s
        if duration <= 0.0:
            duration = self.config.default_stop_duration_s

        self.wait_until_s = now_s + duration
        self.reason = stop.reason
        self.state = ControllerState.WAIT

        return (Wait(duration_s=duration, reason=stop.reason),)

    def _finish_leg(self, now_s: float, outcome: LegOutcome) -> None:
        if self.leg is None:
            return
        self.leg.finished_at_s = now_s
        self.leg.outcome = outcome
        self.last_leg = self.leg
        self.leg = None
