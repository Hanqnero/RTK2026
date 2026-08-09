import json

import pytest

from rtk2026_city_nav.detections import BUS_STOP, STOP_SIGN, StopRequest
from rtk2026_city_nav.maneuver import Maneuver
from rtk2026_city_nav.planner import RouteState
from rtk2026_city_nav.sign_cache import (
    FORMAT_VERSION,
    FREE,
    Entry,
    SignCache,
    from_dict,
    load,
    save,
    to_dict,
)

S = RouteState(previous=1, current=2)
OTHER = RouteState(previous=3, current=2)


def _cache() -> SignCache:
    return SignCache(graph_fingerprint="sha256:aaaa")


# -- Вывод по наблюдениям --------------------------------------------------


def test_a_state_never_seen_has_no_entry() -> None:
    assert _cache().route == {}


def test_seeing_nothing_still_closes_the_entry() -> None:
    """Отсутствие знака — такой же результат, как и знак."""
    cache = _cache()

    advice = cache.resolve_route(S, "")

    assert advice.is_empty
    assert cache.route[(1, 2)].free
    assert cache.learned == 1
    # Состояние изучено, хотя знака в нём нет.
    assert cache.known_states == 1
    assert cache.constrained_states == 0


def test_a_remembered_sign_acts_when_detections_are_silent() -> None:
    """В этом и весь смысл памяти."""
    cache = _cache()
    cache.resolve_route(S, "left_only")

    advice = cache.resolve_route(S, "")

    assert advice.prefer is Maneuver.LEFT
    assert advice.from_memory is True
    assert cache.hits == 1


def test_a_live_sign_is_not_marked_as_remembered() -> None:
    """Иначе диагностика не отличит увиденное от запомненного."""
    cache = _cache()

    advice = cache.resolve_route(S, "left_only")

    assert advice.prefer is Maneuver.LEFT
    assert advice.from_memory is False
    assert cache.hits == 0


def test_a_live_sign_displaces_a_remembered_free_state() -> None:
    """Отсутствие — слабое утверждение: знак могли просто не увидеть."""
    cache = _cache()
    cache.resolve_route(S, "")
    assert cache.route[(1, 2)].free

    advice = cache.resolve_route(S, "right_only")

    assert advice.prefer is Maneuver.RIGHT
    assert advice.from_memory is False, "живой знак не должен считаться памятью"
    assert cache.corrections == 1
    assert not cache.route[(1, 2)].free


def test_repeated_absence_never_outweighs_one_sighting() -> None:
    """Сколько бы раз знак ни пропустили, он остаётся выученным."""
    cache = _cache()
    cache.resolve_route(S, "left_only")

    for _ in range(10):
        advice = cache.resolve_route(S, "")

    assert advice.prefer is Maneuver.LEFT
    assert cache.route[(1, 2)].counts == {"left_only": 1, FREE: 10}
    assert not cache.route[(1, 2)].free


def test_not_seeing_a_remembered_sign_is_not_a_disagreement() -> None:
    """Знак попадает в кадр не каждый проезд — это обычное дело.

    Иначе счётчик расхождений забили бы штатные проезды, и по нему нельзя
    было бы судить о неисправности перцепции, ради чего он и нужен.
    """
    cache = _cache()
    cache.resolve_route(S, "left_only")
    cache.resolve_stop(S, StopRequest(duration_s=3.0, reason=STOP_SIGN))

    for _ in range(10):
        cache.resolve_route(S, "")
        cache.resolve_stop(S, None)

    assert cache.conflicts == 0
    assert cache.disputed_states == ()


def test_a_sign_where_it_was_free_is_a_correction_not_a_disagreement() -> None:
    """Поправка считается отдельно: смысл у неё другой."""
    cache = _cache()
    cache.resolve_route(S, "")

    cache.resolve_route(S, "left_only")

    assert cache.corrections == 1
    assert cache.conflicts == 0
    assert cache.disputed_states == ()


def test_a_different_live_command_does_not_override_memory() -> None:
    """Один плохой кадр не должен менять решение."""
    cache = _cache()
    cache.resolve_route(S, "left_only")
    cache.resolve_route(S, "left_only")

    advice = cache.resolve_route(S, "right_only")

    assert advice.prefer is Maneuver.LEFT, "победил один кадр против двух"
    assert advice.from_memory is True
    assert cache.conflicts == 1
    assert cache.disputed_states == ((1, 2),)


def test_persistent_disagreement_eventually_flips_the_entry() -> None:
    """Устойчивое расхождение всё же должно переучивать: это не догма."""
    cache = _cache()
    cache.resolve_route(S, "left_only")

    for _ in range(3):
        advice = cache.resolve_route(S, "right_only")

    assert advice.prefer is Maneuver.RIGHT
    assert cache.route[(1, 2)].counts == {"left_only": 1, "right_only": 3}


def test_states_are_kept_apart_by_approach_direction() -> None:
    """К одной вершине с разных сторон относятся разные знаки."""
    cache = _cache()
    cache.resolve_route(S, "left_only")

    advice = cache.resolve_route(OTHER, "")

    assert advice.is_empty, "память подъезда 1->2 применилась к подъезду 3->2"
    assert set(cache.route) == {(1, 2), (3, 2)}


def test_an_unknown_command_is_remembered_but_advises_nothing() -> None:
    cache = _cache()

    first = cache.resolve_route(S, "нечто_нераспознанное")
    second = cache.resolve_route(S, "")

    assert first.is_empty
    assert second.is_empty
    # Записана, но советовать по ней нечего, поэтому решением по памяти
    # это не считается.
    assert cache.hits == 0
    # И расхождением тоже: разошлись бы два разных знака, а не знак с тишиной.
    assert cache.conflicts == 0


def test_a_prohibition_is_remembered_as_a_prohibition() -> None:
    cache = _cache()
    cache.resolve_route(S, "no_left_turn")

    advice = cache.resolve_route(S, "")

    assert advice.prefer is None
    assert advice.forbid == frozenset({Maneuver.LEFT})
    assert advice.from_memory is True


def test_tie_between_commands_goes_to_the_most_recent() -> None:
    entry = Entry()
    entry.observe("left_only")
    entry.observe("right_only")

    assert entry.best == "right_only"
    assert entry.disputed is True


# -- Остановки -------------------------------------------------------------


def test_a_remembered_stop_is_executed_without_seeing_the_sign() -> None:
    """Лишняя остановка стоит времени, пропущенная — нарушение."""
    cache = _cache()
    cache.resolve_stop(S, StopRequest(duration_s=3.0, reason=STOP_SIGN))

    stop = cache.resolve_stop(S, None)

    assert stop is not None
    assert stop.reason == STOP_SIGN
    assert stop.duration_s == pytest.approx(3.0)
    assert cache.hits == 1


def test_a_state_learned_as_free_does_not_invent_a_stop() -> None:
    cache = _cache()
    cache.resolve_stop(S, None)

    assert cache.resolve_stop(S, None) is None
    assert cache.hits == 0


def test_a_live_stop_displaces_a_state_learned_as_free() -> None:
    cache = _cache()
    cache.resolve_stop(S, None)

    stop = cache.resolve_stop(S, StopRequest(duration_s=2.0, reason=BUS_STOP))

    assert stop is not None and stop.reason == BUS_STOP
    assert cache.corrections == 1


def test_a_remembered_stop_keeps_the_duration_it_was_given() -> None:
    """У автобусной остановки своей длительности нет: её задаёт настройка."""
    cache = _cache()
    cache.resolve_stop(S, StopRequest(duration_s=0.0, reason=BUS_STOP))

    stop = cache.resolve_stop(S, None)

    assert stop is not None
    assert stop.duration_s == pytest.approx(0.0)


def test_a_live_stop_wins_over_a_remembered_one() -> None:
    cache = _cache()
    cache.resolve_stop(S, StopRequest(duration_s=3.0, reason=STOP_SIGN))

    live = StopRequest(duration_s=5.0, reason=STOP_SIGN)
    stop = cache.resolve_stop(S, live)

    assert stop is live
    assert cache.stop[(1, 2)].duration_s == pytest.approx(5.0)


# -- Файл ------------------------------------------------------------------


def test_roundtrip_keeps_signs_free_states_and_counts() -> None:
    cache = _cache()
    cache.resolve_route(S, "left_only")
    cache.resolve_route(S, "")
    cache.resolve_route(OTHER, "")
    cache.resolve_stop(S, StopRequest(duration_s=3.0, reason=STOP_SIGN))

    restored = from_dict(json.loads(json.dumps(to_dict(cache))))

    assert restored.graph_fingerprint == cache.graph_fingerprint
    assert restored.route[(1, 2)].counts == {"left_only": 1, FREE: 1}
    # Свободное состояние обязано пережить запись: иначе его пришлось бы
    # изучать заново, а в этом и был смысл его запоминать.
    assert (3, 2) in restored.route
    assert restored.route[(3, 2)].free
    assert restored.stop[(1, 2)].duration_s == pytest.approx(3.0)


def test_the_saved_file_shows_the_conclusion_not_only_the_counts(tmp_path) -> None:
    """Файл читают руками, поэтому вывод должен быть виден сразу."""
    cache = _cache()
    cache.resolve_route(S, "left_only")
    path = tmp_path / "signs.json"
    save(path, cache)

    data = json.loads(path.read_text(encoding="utf-8"))

    assert data["route"][0]["sign"] == "left_only"
    assert data["route"][0]["from"] == 1
    assert path.read_text(encoding="utf-8").endswith("\n")


def test_a_missing_file_is_not_an_error(tmp_path) -> None:
    """До первого прогона файла и не должно быть."""
    cache, reason = load(tmp_path / "нет.json", graph_fingerprint="sha256:aaaa")

    assert cache.route == {}
    assert cache.graph_fingerprint == "sha256:aaaa"
    assert "нет" in reason


def test_memory_from_another_graph_is_discarded(tmp_path) -> None:
    """Подъезды задаёт геометрия: сдвинулась она — запомненное не про то."""
    cache = _cache()
    cache.resolve_route(S, "left_only")
    path = tmp_path / "signs.json"
    save(path, cache)

    restored, reason = load(path, graph_fingerprint="sha256:bbbb")

    assert restored.route == {}
    assert "другой граф" in reason
    assert restored.graph_fingerprint == "sha256:bbbb"


def test_a_damaged_file_yields_empty_memory_not_a_crash(tmp_path) -> None:
    path = tmp_path / "signs.json"
    path.write_text("{ это не json", encoding="utf-8")

    cache, reason = load(path, graph_fingerprint="sha256:aaaa")

    assert cache.route == {}
    assert "не прочитана" in reason


def test_a_future_format_version_is_refused() -> None:
    with pytest.raises(ValueError, match="версия формата"):
        from_dict({"version": FORMAT_VERSION + 1})


def test_entries_without_endpoints_are_skipped() -> None:
    cache = from_dict(
        {
            "version": FORMAT_VERSION,
            "route": [
                {"counts": {"left_only": 1}},
                {"from": 1, "to": 2, "counts": {"left_only": 2}},
                {"from": 9, "to": 9, "counts": {}},
            ],
        }
    )

    assert set(cache.route) == {(1, 2)}


def test_summary_reports_what_was_learned() -> None:
    cache = _cache()
    cache.resolve_route(S, "left_only")
    cache.resolve_route(OTHER, "")
    cache.resolve_route(S, "")

    text = cache.summary()

    assert "изучено состояний 2" in text
    assert "со знаком 1" in text
    assert "решений по памяти 1" in text
