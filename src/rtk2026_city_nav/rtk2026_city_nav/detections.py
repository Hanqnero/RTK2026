"""Обработка детекций знаков: что из них следует для выбора и для остановок.

Модуль намеренно не знает про ROS: на вход идут обычные значения, а не
сообщение. Нода превращает ``DrivingDetection`` в вызовы ``observe_*``.
Так логика отбора тестируется без установленного ROS и без перцепции.

Детекции копятся до момента выбора, а не применяются сразу: знак, увиденный
по пути к точке решения, относится к выбору **в ней**, а не к участку,
по которому робот едет сейчас.

К какой точке решения относится знак
-------------------------------------

Знак впереди может относиться не к ближайшему перекрёстку, а к следующему
за ним. Отличить их можно только по видимому размеру: чем ближе знак, тем
крупнее его рамка. Поэтому ``min_box_area_px`` — не фильтр шума, а порог
принадлежности: пока рамка меньше, знак считается относящимся к чему-то
дальше и в выбор не идёт.

Значения по умолчанию у порога нет: он зависит от разрешения камеры,
объектива и физического размера знака. Калибруется так — подъехать к
перекрёстку, писать ``box_area`` из ``/perception/driving_detection`` и
взять то значение, которое рамка принимает на дистанции, с которой знак
уже должен учитываться.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from rtk2026_city_nav.maneuver import Maneuver, SignAdvice

#: Предписывающие команды: указывают маневр.
PRESCRIBE: dict[str, Maneuver] = {
    "straight_only": Maneuver.STRAIGHT,
    "left_only": Maneuver.LEFT,
    "right_only": Maneuver.RIGHT,
}

#: Запрещающие команды: вычёркивают маневр, ничего не предлагая взамен.
PROHIBIT: dict[str, Maneuver] = {
    "no_straight": Maneuver.STRAIGHT,
    "no_left_turn": Maneuver.LEFT,
    "no_right_turn": Maneuver.RIGHT,
}

#: Причины остановки.
STOP_SIGN = "stop_sign"
BUS_STOP = "bus_stop"

#: Порог уверенности по умолчанию. Совпадает с порогом адаптера перцепции,
#: чтобы здесь не отсеивалось то, что там уже прошло.
DEFAULT_MIN_CONFIDENCE = 0.25


@dataclass(frozen=True)
class StopRequest:
    """Требование остановиться."""

    duration_s: float
    #: :data:`STOP_SIGN` или :data:`BUS_STOP`.
    reason: str


@dataclass(frozen=True)
class Observation:
    """Одна принятая детекция с признаками, по которым выбирается лучшая."""

    command: str
    confidence: float
    #: Площадь рамки в пикселях: прокси близости знака.
    box_area_px: float

    def closer_than(self, other: Observation | None) -> bool:
        """Ближе, а при равной близости — уверенней."""
        if other is None:
            return True
        if self.box_area_px != other.box_area_px:
            return self.box_area_px > other.box_area_px
        return self.confidence > other.confidence


@dataclass
class Latch:
    """Накопитель детекций между двумя точками решений.

    Хранит по одной лучшей детекции каждого рода. Опустошается вызовом
    :meth:`clear` после того, как выбор сделан: знаки относятся к одной
    конкретной точке решения и на следующую не переносятся.
    """

    #: Порог принадлежности к ближайшей точке решения, пиксели в квадрате.
    #: Значения по умолчанию нет намеренно, см. модульную документацию.
    min_box_area_px: float
    min_confidence: float = DEFAULT_MIN_CONFIDENCE

    _route: Observation | None = field(default=None, init=False, repr=False)
    _stop: Observation | None = field(default=None, init=False, repr=False)
    _stop_duration_s: float = field(default=0.0, init=False, repr=False)
    _bus_seen: bool = field(default=False, init=False, repr=False)
    #: Сколько детекций отброшено как относящиеся к чему-то дальше.
    too_far_count: int = field(default=0, init=False)

    def observe_route(
        self,
        command: str,
        *,
        confidence: float = 1.0,
        box_area_px: float,
    ) -> bool:
        """Учесть знак маршрута. Возвращает, обновилась ли лучшая детекция."""
        candidate = self._accept(command, confidence, box_area_px)
        if candidate is None or not candidate.closer_than(self._route):
            return False
        self._route = candidate
        return True

    def observe_stop(
        self,
        action: str,
        *,
        duration_s: float,
        confidence: float = 1.0,
        box_area_px: float,
    ) -> bool:
        """Учесть знак остановки."""
        candidate = self._accept(action, confidence, box_area_px)
        if candidate is None or not candidate.closer_than(self._stop):
            return False
        self._stop = candidate
        self._stop_duration_s = max(0.0, float(duration_s))
        return True

    def observe_bus(self, *, box_area_px: float, confidence: float = 1.0) -> bool:
        """Отметить автобусную остановку.

        Порог принадлежности применяется и к ней: далёкая остановка
        относится к следующему участку, а не к текущему.
        """
        candidate = self._accept(BUS_STOP, confidence, box_area_px)
        if candidate is None:
            return False
        self._bus_seen = True
        return True

    def advice(self) -> SignAdvice:
        """Что накопленные знаки говорят о выборе маневра.

        Неизвестная команда даёт пустой совет: ехать своим порядком
        безопаснее, чем угадывать смысл нераспознанного знака.
        """
        if self._route is None:
            return SignAdvice()

        command = self._route.command
        forbidden = PROHIBIT.get(command)

        return SignAdvice(
            prefer=PRESCRIBE.get(command),
            forbid=frozenset({forbidden}) if forbidden is not None else frozenset(),
        ).resolved()

    def stop_request(self) -> StopRequest | None:
        """Требование остановиться, если знак остановки был.

        Стоп-знак приоритетнее автобусной: его длительность задана явно,
        а автобусная сама по себе длительности не несёт.
        """
        if self._stop is not None:
            return StopRequest(duration_s=self._stop_duration_s, reason=STOP_SIGN)
        if self._bus_seen:
            return StopRequest(duration_s=0.0, reason=BUS_STOP)
        return None

    @property
    def route_command(self) -> str:
        """Лучшая накопленная команда маршрута; пусто, если её не было."""
        return "" if self._route is None else self._route.command

    @property
    def route_box_area_px(self) -> float:
        """Площадь рамки лучшей команды маршрута. Нужна для диагностики."""
        return 0.0 if self._route is None else self._route.box_area_px

    def clear(self) -> None:
        """Забыть накопленное: выбор сделан, знаки к следующему не относятся."""
        self._route = None
        self._stop = None
        self._stop_duration_s = 0.0
        self._bus_seen = False

    def _accept(
        self,
        command: str,
        confidence: float,
        box_area_px: float,
    ) -> Observation | None:
        """Отсеять пустое, неуверенное и слишком далёкое."""
        text = str(command).strip()
        if not text:
            return None
        if float(confidence) < self.min_confidence:
            return None
        if float(box_area_px) < self.min_box_area_px:
            self.too_far_count += 1
            return None
        return Observation(
            command=text,
            confidence=float(confidence),
            box_area_px=float(box_area_px),
        )
