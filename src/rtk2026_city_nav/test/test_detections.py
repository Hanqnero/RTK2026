import pytest

from rtk2026_city_nav.detections import BUS_STOP, STOP_SIGN, Latch
from rtk2026_city_nav.maneuver import Maneuver, SignAdvice

#: Порог принадлежности к ближайшей точке решения, пиксели в квадрате.
NEAR = 900.0
FAR = 100.0


def _latch() -> Latch:
    return Latch(min_box_area_px=500.0, min_confidence=0.25)


def test_no_detections_gives_empty_advice() -> None:
    latch = _latch()

    assert latch.advice().is_empty
    assert latch.stop_request() is None
    assert latch.route_command == ""


def test_prescribing_sign_becomes_preference() -> None:
    latch = _latch()

    assert latch.observe_route("left_only", box_area_px=NEAR) is True

    assert latch.advice() == SignAdvice(prefer=Maneuver.LEFT)


def test_prohibiting_sign_becomes_forbid_without_preference() -> None:
    latch = _latch()

    latch.observe_route("no_left_turn", box_area_px=NEAR)

    advice = latch.advice()
    assert advice.prefer is None
    assert advice.forbid == frozenset({Maneuver.LEFT})


def test_far_sign_belongs_to_a_later_junction_and_is_dropped() -> None:
    """Мелкая рамка означает, что знак относится к чему-то дальше."""
    latch = _latch()

    assert latch.observe_route("left_only", box_area_px=FAR) is False

    assert latch.advice().is_empty
    assert latch.too_far_count == 1


def test_sign_starts_counting_once_close_enough() -> None:
    latch = _latch()

    latch.observe_route("right_only", box_area_px=FAR)
    assert latch.advice().is_empty

    # Подъехали: та же табличка стала крупнее и теперь учитывается.
    latch.observe_route("right_only", box_area_px=NEAR)
    assert latch.advice() == SignAdvice(prefer=Maneuver.RIGHT)


def test_closer_sign_replaces_further_one() -> None:
    latch = _latch()

    latch.observe_route("left_only", box_area_px=NEAR)
    assert latch.observe_route("right_only", box_area_px=NEAR * 2) is True

    # Ближе - значит относится к этой точке решения, а не предыдущая.
    assert latch.advice() == SignAdvice(prefer=Maneuver.RIGHT)


def test_further_sign_does_not_replace_closer_one() -> None:
    latch = _latch()

    latch.observe_route("left_only", box_area_px=NEAR * 2)
    assert latch.observe_route("right_only", box_area_px=NEAR) is False

    assert latch.advice() == SignAdvice(prefer=Maneuver.LEFT)


def test_equal_area_resolved_by_confidence() -> None:
    latch = _latch()

    latch.observe_route("left_only", confidence=0.5, box_area_px=NEAR)
    assert latch.observe_route("right_only", confidence=0.9, box_area_px=NEAR) is True

    assert latch.advice() == SignAdvice(prefer=Maneuver.RIGHT)


def test_low_confidence_is_dropped() -> None:
    latch = _latch()

    assert latch.observe_route("left_only", confidence=0.1, box_area_px=NEAR) is False

    assert latch.advice().is_empty
    # Это не «слишком далеко», а недостаточная уверенность.
    assert latch.too_far_count == 0


def test_empty_command_is_ignored() -> None:
    latch = _latch()

    assert latch.observe_route("", box_area_px=NEAR) is False
    assert latch.observe_route("   ", box_area_px=NEAR) is False


def test_unknown_command_gives_empty_advice() -> None:
    """Нераспознанный знак не должен превращаться в догадку."""
    latch = _latch()

    latch.observe_route("some_new_sign", box_area_px=NEAR)

    assert latch.advice().is_empty
    # Но сама команда сохранена: увидим её в диагностике.
    assert latch.route_command == "some_new_sign"


def test_stop_sign_becomes_stop_request_with_duration() -> None:
    latch = _latch()

    latch.observe_stop("stop", duration_s=3.0, box_area_px=NEAR)

    request = latch.stop_request()
    assert request is not None
    assert request.reason == STOP_SIGN
    assert request.duration_s == pytest.approx(3.0)


def test_negative_duration_is_clamped() -> None:
    latch = _latch()

    latch.observe_stop("stop", duration_s=-5.0, box_area_px=NEAR)

    request = latch.stop_request()
    assert request is not None
    assert request.duration_s == 0.0


def test_bus_stop_becomes_stop_request_without_duration() -> None:
    latch = _latch()

    assert latch.observe_bus(box_area_px=NEAR) is True

    request = latch.stop_request()
    assert request is not None
    assert request.reason == BUS_STOP
    assert request.duration_s == 0.0


def test_far_bus_stop_is_dropped_too() -> None:
    latch = _latch()

    assert latch.observe_bus(box_area_px=FAR) is False

    assert latch.stop_request() is None


def test_stop_sign_wins_over_bus_stop() -> None:
    latch = _latch()

    latch.observe_bus(box_area_px=NEAR)
    latch.observe_stop("stop", duration_s=2.0, box_area_px=NEAR)

    request = latch.stop_request()
    assert request is not None
    assert request.reason == STOP_SIGN


def test_route_and_stop_are_independent() -> None:
    latch = _latch()

    latch.observe_route("left_only", box_area_px=NEAR)
    latch.observe_stop("stop", duration_s=1.0, box_area_px=NEAR)

    assert latch.advice() == SignAdvice(prefer=Maneuver.LEFT)
    assert latch.stop_request() is not None


def test_clear_forgets_everything() -> None:
    latch = _latch()

    latch.observe_route("left_only", box_area_px=NEAR)
    latch.observe_stop("stop", duration_s=1.0, box_area_px=NEAR)
    latch.observe_bus(box_area_px=NEAR)

    latch.clear()

    assert latch.advice().is_empty
    assert latch.stop_request() is None
    assert latch.route_command == ""


def test_box_area_is_exposed_for_diagnostics() -> None:
    latch = _latch()

    latch.observe_route("left_only", box_area_px=1234.0)

    assert latch.route_box_area_px == pytest.approx(1234.0)
