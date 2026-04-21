"""Имена полос нового v2-пайплайна (без legacy alias)."""

from __future__ import annotations

LANE1 = "lane1"
LANE2 = "lane2"
LANE_ANY = "any"
KNOWN_LANES: frozenset[str] = frozenset({LANE1, LANE2})


def normalize_lane_mode(mode: str) -> str:
    """Проверяет и нормализует имя полосы v2."""
    key = str(mode).strip().lower()
    if key == LANE_ANY:
        return LANE_ANY
    if key in KNOWN_LANES:
        return key
    raise ValueError(f"unknown lane mode: {mode!r}; expected one of: {sorted(KNOWN_LANES)}")


def opposite_lane_mode(mode: str) -> str:
    """Возвращает противоположную полосу."""
    canonical = normalize_lane_mode(mode)
    if canonical == LANE1:
        return LANE2
    if canonical == LANE2:
        return LANE1
    raise ValueError(f"cannot get opposite for lane mode: {mode!r}")
