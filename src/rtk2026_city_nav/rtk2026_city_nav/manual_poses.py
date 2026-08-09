"""Ручные правки поз: загрузка и подмена расчётных.

Из файла поз берутся только записи с пометкой ``manual``. Остальные —
результат того же расчёта, что делает исполнитель, и подменять их файлом
значило бы держать в репозитории копию вычислимого.

Файл создаётся и обновляется инструментом ``city_nav_poses``, см.
:mod:`rtk2026_city_nav.cli`.
"""

from __future__ import annotations

from pathlib import Path

from rclpy.impl.rcutils_logger import RcutilsLogger

from rtk2026_city_nav.lane import LanePose
from rtk2026_city_nav.poses_io import load


class ManualPoses:
    """Правленные руками позы участков, если файл задан."""

    def __init__(
        self,
        logger: RcutilsLogger,
        *,
        path: str,
        graph_fingerprint: str,
    ) -> None:
        self._logger = logger
        self._legs: dict[tuple[int, int], tuple[LanePose, ...]] = {}
        #: Сколько участков прошли по позам из файла, а не по расчётным.
        self.used = 0

        raw = path.strip()
        if not raw:
            return

        file = Path(raw)
        if not file.is_file():
            logger.warn(f"файла поз нет: {file}")
            return

        try:
            stored = load(file)
        except (OSError, ValueError) as error:
            logger.error(f"файл поз не прочитан: {error}")
            return

        if stored.graph_fingerprint != graph_fingerprint:
            logger.warn(
                "файл поз собран под другой граф "
                f"({stored.graph_fingerprint} вместо {graph_fingerprint}): "
                "ручные правки могли относиться к прежней геометрии"
            )

        self._legs = {leg.key: leg.poses for leg in stored.legs if leg.manual}
        logger.info(f"ручных правок поз загружено: {len(self._legs)}")

    def __len__(self) -> int:
        return len(self._legs)

    def resolve(
        self, start: int, end: int, computed: tuple[LanePose, ...]
    ) -> tuple[LanePose, ...]:
        """Позы участка: правленные руками, если они есть, иначе расчётные."""
        manual = self._legs.get((start, end))
        if manual is None:
            return computed

        self.used += 1
        return manual