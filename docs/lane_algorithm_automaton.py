#!/usr/bin/env python3
"""Схема автомата исполнителя из docs/lane_algorithm_spec.md.

Запуск:

    python3 docs/lane_algorithm_automaton.py [движок]

Пишет docs/images/lane_algorithm_automaton.png и .svg. Требует graphviz:
бинарь ``dot`` в PATH и пакет ``graphviz`` в окружении.

Расстановка вершин и рёбер полностью на движке компоновки. Здесь задаются
только состояния, переходы и стиль по типу перехода: ни координат, ни
рангов, ни подпорок вида constraint или minlen.

Движки: ``dot`` — иерархический, для автоматов подходит лучше всего;
``neato`` и ``fdp`` — силовые, ``circo`` — по окружности.
"""

from __future__ import annotations

import sys
from pathlib import Path

import graphviz

OUT_DIR = Path(__file__).resolve().parent / "images"
OUT_NAME = "lane_algorithm_automaton"

FONT = "Helvetica"
INK = "#1A1F24"
ACCENT = "#0A6A6A"
FAIL = "#A03E29"

#: Переходы автомата: откуда, куда, событие, тип.
#:
#: ``sign`` — реакция на дорожный знак. Различие между этими двумя рёбрами
#: и есть смысл состояния COMMIT: до барьера знак перепланирует маневр,
#: после барьера не делает ничего.
#: ``fail`` — уход в восстановление.
TRANSITIONS: tuple[tuple[str, str, str, str], ...] = (
    ("LOCALIZE", "PLAN", "поза сопоставлена", "plain"),
    ("LOCALIZE", "LOCALIZE", "не сопоставлена", "plain"),
    ("PLAN", "TRACK", "позы построены", "plain"),
    ("TRACK", "COMMIT", "ℓ ≤ ℓc", "plain"),
    ("COMMIT", "ADVANCE", "вершина достигнута", "plain"),
    ("ADVANCE", "PLAN", "p := c,  c := v", "plain"),
    ("TRACK", "PLAN", "знак σ: перепланировать", "sign"),
    ("COMMIT", "COMMIT", "знак σ: игнорируется", "sign"),
    ("PLAN", "RECOVER", "нет маневров (C2)", "fail"),
    ("TRACK", "RECOVER", "сторона не та, отказ Nav2", "fail"),
    ("COMMIT", "RECOVER", "отказ Nav2", "fail"),
    ("RECOVER", "LOCALIZE", "сброс цепочки", "plain"),
)

EDGE_STYLE: dict[str, dict[str, str]] = {
    "plain": {},
    "sign": {"color": ACCENT, "fontcolor": ACCENT, "penwidth": "1.7"},
    "fail": {"color": FAIL, "fontcolor": FAIL, "style": "dashed"},
}

#: Заливка состояний, отличающихся ролью. Остальные — по умолчанию.
NODE_STYLE: dict[str, dict[str, str]] = {
    "COMMIT": {"fillcolor": "#DCEBEA", "color": ACCENT, "penwidth": "1.8"},
    "RECOVER": {"fillcolor": "#F6ECEA", "color": FAIL},
}

INITIAL_STATE = "LOCALIZE"


def build(engine: str = "dot") -> graphviz.Digraph:
    g = graphviz.Digraph("controller", engine=engine)

    g.attr(rankdir="LR", bgcolor="white", fontname=FONT, pad="0.3")
    g.attr(
        "node",
        shape="box",
        style="rounded,filled",
        fillcolor="#F4F6F7",
        color=INK,
        fontname=FONT,
        fontsize="13",
        fontcolor=INK,
        margin="0.18,0.10",
    )
    g.attr("edge", fontname=FONT, fontsize="10", color=INK, fontcolor=INK, arrowsize="0.75")

    # Порядок объявления, а не алфавитный: движок компоновки разрешает
    # равные ранги в порядке поступления, поэтому подача вершин в порядке
    # переходов даёт чтение слева направо. Координат это не задаёт.
    states: list[str] = []
    for source, target, _event, _kind in TRANSITIONS:
        for state in (source, target):
            if state not in states:
                states.append(state)

    for state in states:
        g.node(state, state, **NODE_STYLE.get(state, {}))

    # Маркер начального состояния.
    g.node("__start__", "", shape="point", width="0.12", color=INK, fillcolor=INK)
    g.edge("__start__", INITIAL_STATE)

    for source, target, event, kind in TRANSITIONS:
        g.edge(source, target, label=event, **EDGE_STYLE[kind])

    return g


def main() -> None:
    engine = sys.argv[1] if len(sys.argv) > 1 else "dot"
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    g = build(engine)
    for fmt in ("png", "svg"):
        g.format = fmt
        path = g.render(filename=OUT_NAME, directory=OUT_DIR, cleanup=True)
        print(Path(path).relative_to(OUT_DIR.parent.parent))


if __name__ == "__main__":
    main()
