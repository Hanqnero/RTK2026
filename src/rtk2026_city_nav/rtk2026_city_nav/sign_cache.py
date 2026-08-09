"""Память о знаках: правила движения, доученные проездом.

Зачем
-----

Таблица маневров говорит, что из состояния *возможно*. Знаки говорят, что
из него *разрешено*. Первое считается из графа заранее, второе до сих пор
приходилось выяснять заново на каждом проезде — хотя знаки на трассе стоят
неподвижно и граф неподвижен, то есть «состояние → ограничение» такая же
функция, как и таблица маневров, и доучивается она за один проезд.

Доучив её, робот перестаёт зависеть от того, попал ли знак в кадр.

Ключ
----

Ключ — пара вершин «откуда приехал, где находится», та же, что у таблицы
маневров. Не одна вершина: знак стоит на подъезде и его команда задана
относительно направления прибытия. К одной вершине с разных сторон
относятся разные знаки, и ``left_only`` с одного подъезда означает не тот
маневр, что с другого.

Когда запись закрывается
------------------------

Окно наблюдения состояния ``(A, B)`` — весь проезд от ``A`` к ``B`` плюс
момент прибытия в ``B``: знак виден и по дороге, и уже на вершине, в том
числе пока робот стоит у стоп-линии. Окно закрывается ровно тогда, когда
выбирается следующий маневр, — больше об этом состоянии узнать нечего,
и запись фиксируется тогда же.

Отсутствие знака — такой же результат
-------------------------------------

Если за всё окно ничего не предписало и не запретило, состояние
запоминается как свободное. Это не то же самое, что «ещё не проезжали»:
свободное состояние изучено, и знаки в нём больше не нужны.

Присутствие сильнее отсутствия
------------------------------

Единственная асимметрия, и она вынужденная. Не увидеть знак — штатный
отказ перцепции: не попал в кадр, засветка, угол. Увидеть несуществующий
знак — отказ куда более редкий. Поэтому:

* память говорит знак, детекции молчат — действует память, в этом весь смысл;
* память говорит знак, детекции говорят другой — действует память,
  расхождение считается: один плохой кадр не должен менять решение;
* память говорит «свободно», детекции говорят знак — действует знак
  и попадает в память: отсутствие было слабым утверждением;
* записи нет — действуют детекции, и запись закрывается по ним.

Чего память не исправляет
-------------------------

Если знак отнесён не к той точке решения — например, дальний знак прошёл
порог принадлежности, — запись закрепится под неверным ключом. Нового рода
отказа память не добавляет, но и этот не лечит: разбираться надо с порогом,
см. :mod:`rtk2026_city_nav.detections`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rtk2026_city_nav.detections import StopRequest, advice_for
from rtk2026_city_nav.maneuver import SignAdvice
from rtk2026_city_nav.planner import RouteState

#: Версия формата файла памяти.
FORMAT_VERSION = 1

#: Ключ записи: пара вершин.
Key = tuple[int, int]

#: Наблюдение «ничего не видели». Хранится наравне с командами, потому что
#: свободное состояние — такой же результат, как и состояние со знаком.
FREE = ""


def _key(state: RouteState) -> Key:
    return (state.previous, state.current)


@dataclass
class Entry:
    """Что наблюдалось в одном состоянии за все проезды.

    Счётчик, а не последнее значение: одна ошибка распознавания не должна
    перевешивать несколько согласных наблюдений.
    """

    counts: dict[str, int] = field(default_factory=dict[str, int])
    #: Последнее наблюдение. Разрешает равенство счётчиков.
    last: str = FREE

    def observe(self, command: str) -> bool:
        """Учесть наблюдение. Возвращает, разошлось ли оно с прежним выводом.

        Расхождение — это два разных знака в одном состоянии, и только оно:
        по нему судят о неисправности перцепции. Не увидеть запомненный знак
        расхождением не считается, иначе счётчик забили бы обычные проезды —
        знак не в каждом кадре. Не считается им и знак там, где раньше было
        свободно: это поправка, и она считается отдельно.
        """
        conclusion = self.best
        conflicts = bool(command) and bool(conclusion) and command != conclusion

        self.counts[command] = self.counts.get(command, 0) + 1
        self.last = command
        return conflicts

    @property
    def best(self) -> str:
        """Вывод по всем наблюдениям.

        Среди команд побеждает самая частая; при равенстве — последняя
        наблюдённая. :data:`FREE` возвращается только если знака здесь
        не видели ни разу: отсутствие слабее присутствия и вытесняется им,
        сколько бы раз ни наблюдалось.
        """
        commands = {c: n for c, n in self.counts.items() if c != FREE}
        if not commands:
            return FREE

        top = max(commands.values())
        candidates = sorted(c for c, n in commands.items() if n == top)
        if len(candidates) == 1:
            return candidates[0]
        return self.last if self.last in candidates else candidates[0]

    @property
    def free(self) -> bool:
        """Состояние изучено и знаков в нём нет."""
        return self.best == FREE

    @property
    def disputed(self) -> bool:
        """Читались разные команды: неисправность перцепции."""
        return len({c for c in self.counts if c != FREE}) > 1

    @property
    def observations(self) -> int:
        return sum(self.counts.values())


@dataclass
class StopEntry:
    """Запомненное требование остановки."""

    entry: Entry = field(default_factory=Entry)
    #: Длительность последней наблюдённой остановки, секунды.
    duration_s: float = 0.0


@dataclass
class SignCache:
    """Память о знаках, привязанная к конкретному графу.

    Отпечаток графа хранится вместе с записями: сдвинулась геометрия —
    сдвинулись подъезды, и запомненное может относиться уже не к тем
    состояниям.
    """

    graph_fingerprint: str = ""

    route: dict[Key, Entry] = field(default_factory=dict[Key, Entry])
    stop: dict[Key, StopEntry] = field(default_factory=dict[Key, StopEntry])

    #: Решений, принятых по памяти без живой детекции.
    hits: int = 0
    #: Записей, закрытых впервые.
    learned: int = 0
    #: Наблюдений, разошедшихся с уже выученным.
    conflicts: int = 0
    #: Сколько раз живой знак вытеснил запомненное «свободно».
    corrections: int = 0

    # -- Маневры -----------------------------------------------------------

    def resolve_route(self, state: RouteState, live_command: str) -> SignAdvice:
        """Закрыть окно наблюдения и дать совет для выбора маневра.

        Вызывается один раз на выбор: он же и есть момент закрытия окна.

        :param live_command: команда, накопленная за этот проезд; пусто,
            если знака не видели.
        """
        key = _key(state)
        entry = self.route.get(key)

        if entry is None:
            entry = Entry()
            self.route[key] = entry
            entry.observe(live_command)
            self.learned += 1
            return advice_for(live_command)

        was_free = entry.free
        if entry.observe(live_command):
            self.conflicts += 1

        if live_command and was_free:
            # Отсутствие было слабым утверждением, живой знак его вытесняет.
            self.corrections += 1
            return advice_for(live_command)

        remembered = entry.best
        if not remembered:
            return SignAdvice()

        advice = advice_for(remembered)
        if advice.is_empty:
            # Команда запомнена, но смысла для выбора не несёт: нераспознанное.
            # Считать это решением по памяти нельзя, иначе счётчик врёт.
            return advice

        if not live_command:
            self.hits += 1

        return advice.remembered()

    # -- Остановки ---------------------------------------------------------

    def resolve_stop(
        self, state: RouteState, live: StopRequest | None
    ) -> StopRequest | None:
        """Закрыть окно наблюдения и дать требование остановки.

        Запомненная остановка исполняется, даже если знака сейчас не видно:
        лишняя остановка стоит времени, пропущенная — нарушение правил.
        """
        key = _key(state)
        observed = live.reason if live is not None else FREE

        record = self.stop.get(key)
        if record is None:
            record = StopEntry()
            self.stop[key] = record
            record.entry.observe(observed)
            self.learned += 1
            if live is not None:
                record.duration_s = live.duration_s
            return live

        was_free = record.entry.free
        if record.entry.observe(observed):
            self.conflicts += 1
        if live is not None and live.duration_s > 0.0:
            record.duration_s = live.duration_s

        if live is not None:
            if was_free:
                self.corrections += 1
            return live

        remembered = record.entry.best
        if not remembered:
            return None

        self.hits += 1
        return StopRequest(duration_s=record.duration_s, reason=remembered)

    # -- Состояние ---------------------------------------------------------

    @property
    def known_states(self) -> int:
        """Сколько состояний изучено, со знаком или свободных."""
        return len(set(self.route) | set(self.stop))

    @property
    def constrained_states(self) -> int:
        """Сколько состояний ограничено знаком."""
        return len(
            {key for key, entry in self.route.items() if not entry.free}
            | {key for key, record in self.stop.items() if not record.entry.free}
        )

    @property
    def disputed_states(self) -> tuple[Key, ...]:
        """Состояния, где читались разные команды."""
        return tuple(
            sorted(
                {key for key, entry in self.route.items() if entry.disputed}
                | {
                    key
                    for key, record in self.stop.items()
                    if record.entry.disputed
                }
            )
        )

    def summary(self) -> str:
        return (
            f"изучено состояний {self.known_states}, "
            f"из них со знаком {self.constrained_states}, "
            f"решений по памяти {self.hits}, "
            f"поправок {self.corrections}, "
            f"расхождений {self.conflicts}"
        )

    def clear(self) -> None:
        self.route.clear()
        self.stop.clear()


# -- Файл -----------------------------------------------------------------


def to_dict(cache: SignCache) -> dict[str, Any]:
    """Представление для записи в JSON."""
    return {
        "version": FORMAT_VERSION,
        "graph_fingerprint": cache.graph_fingerprint,
        "route": [
            {
                "from": key[0],
                "to": key[1],
                "sign": entry.best,
                "counts": dict(sorted(entry.counts.items())),
                "last": entry.last,
            }
            for key, entry in sorted(cache.route.items())
        ],
        "stop": [
            {
                "from": key[0],
                "to": key[1],
                "sign": record.entry.best,
                "counts": dict(sorted(record.entry.counts.items())),
                "last": record.entry.last,
                "duration_sec": round(record.duration_s, 3),
            }
            for key, record in sorted(cache.stop.items())
        ],
    }


def from_dict(data: dict[str, Any]) -> SignCache:
    """Разобрать файл памяти.

    :raises ValueError: версия формата не та.
    """
    version = int(data.get("version", 0))
    if version != FORMAT_VERSION:
        raise ValueError(
            f"версия формата памяти {version}, ожидается {FORMAT_VERSION}"
        )

    cache = SignCache(graph_fingerprint=str(data.get("graph_fingerprint", "")))

    for item in data.get("route") or []:
        parsed = _entry(item)
        if parsed is not None:
            cache.route[(int(item["from"]), int(item["to"]))] = parsed

    for item in data.get("stop") or []:
        parsed = _entry(item)
        if parsed is not None:
            cache.stop[(int(item["from"]), int(item["to"]))] = StopEntry(
                entry=parsed,
                duration_s=float(item.get("duration_sec", 0.0)),
            )

    return cache


def _entry(item: Any) -> Entry | None:
    """Разобрать одну запись; ``None``, если она непригодна."""
    if not isinstance(item, dict) or "from" not in item or "to" not in item:
        return None

    raw = item.get("counts")
    if not isinstance(raw, dict):
        return None

    # Ключ "" здесь осмыслен: это наблюдение «знака нет».
    counts = {
        str(command): int(number)
        for command, number in raw.items()
        if int(number) > 0
    }
    if not counts:
        return None

    return Entry(counts=counts, last=str(item.get("last", FREE)))


def load(path: str | Path, *, graph_fingerprint: str) -> tuple[SignCache, str]:
    """Прочитать память и проверить, тому ли графу она принадлежит.

    :returns: память и причина, по которой она пуста; пустая строка
        означает, что всё в порядке.

    Отсутствующий файл — не ошибка: до первого прогона его и не должно быть.
    Память от другого графа отбрасывается: запомненное привязано к подъездам,
    а подъезды задаёт геометрия.
    """
    file = Path(path)
    if not file.is_file():
        return SignCache(graph_fingerprint=graph_fingerprint), "файла памяти нет"

    try:
        with file.open(encoding="utf-8") as stream:
            cache = from_dict(json.load(stream))
    except (OSError, ValueError, TypeError) as error:
        return (
            SignCache(graph_fingerprint=graph_fingerprint),
            f"память не прочитана: {error}",
        )

    if cache.graph_fingerprint != graph_fingerprint:
        return (
            SignCache(graph_fingerprint=graph_fingerprint),
            "память собрана под другой граф и отброшена: "
            f"{cache.graph_fingerprint} вместо {graph_fingerprint}",
        )

    return cache, ""


def save(path: str | Path, cache: SignCache) -> None:
    """Записать память."""
    file = Path(path)
    file.parent.mkdir(parents=True, exist_ok=True)
    with file.open("w", encoding="utf-8") as stream:
        json.dump(to_dict(cache), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
