"""Файл памяти о знаках: включение, загрузка, запись при изменении.

Сама память — :class:`~rtk2026_city_nav.sign_cache.SignCache`, она про ROS
не знает. Здесь только то, что вокруг неё: включена ли она, откуда читается,
когда записывается и что об этом сказать в лог.

Три режима, и все три нужны:

* выключена — знаки читаются каждый проезд заново, видно, что находит
  перцепция сама по себе;
* без файла — учится за прогон, следующий запуск начинает с нуля;
* с файлом — выученное переживает перезапуск.
"""

from __future__ import annotations

from pathlib import Path

from rclpy.impl.rcutils_logger import RcutilsLogger

from rtk2026_city_nav.sign_cache import SignCache, load, save


class SignMemory:
    """Обёртка над памятью о знаках, отвечающая за её файл."""

    def __init__(
        self,
        logger: RcutilsLogger,
        *,
        enabled: bool,
        path: str,
        graph_fingerprint: str,
    ) -> None:
        self._logger = logger
        self._path: Path | None = None
        self._cache: SignCache | None = None
        self._saved = (0, 0, 0)

        if not enabled:
            logger.info("память о знаках выключена: каждый проезд читаем заново")
            return

        raw = path.strip()
        if not raw:
            logger.info(
                "память о знаках только в памяти процесса: файл не задан, "
                "следующий запуск начнёт учиться заново"
            )
            self._cache = SignCache(graph_fingerprint=graph_fingerprint)
            return

        self._path = Path(raw)
        cache, reason = load(self._path, graph_fingerprint=graph_fingerprint)
        if reason:
            logger.warn(f"память о знаках пуста: {reason}")
        else:
            logger.info(f"память о знаках из {self._path}: {cache.summary()}")

        self._cache = cache
        self._saved = self._state()

    @property
    def cache(self) -> SignCache | None:
        """Память либо ``None``, если она выключена."""
        return self._cache

    @property
    def path(self) -> Path | None:
        """Файл либо ``None``, если память не сохраняется."""
        return self._path

    def save_if_changed(self) -> None:
        """Записать память, если она изменилась с прошлой записи.

        Пишется по ходу прогона, а не на завершении: прогон может кончиться
        не по-хорошему, а выученное за круг терять незачем. Файл небольшой,
        меняется несколько раз за круг, так что запись ничего не стоит.
        """
        if self._cache is None or self._path is None:
            return

        state = self._state()
        if state == self._saved:
            return

        try:
            save(self._path, self._cache)
        except OSError as error:
            # Не повод останавливать движение: в процессе память работает.
            self._logger.warn(f"память о знаках не записана: {error}")
            return

        self._saved = state

    def _state(self) -> tuple[int, int, int]:
        """Счётчики, по которым видно, что память изменилась."""
        if self._cache is None:
            return (0, 0, 0)
        return (self._cache.learned, self._cache.corrections, self._cache.conflicts)