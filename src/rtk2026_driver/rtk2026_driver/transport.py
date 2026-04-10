"""
Транспортный уровень: открытие serial порта и handshake с Arduino.

Отвечает за:
  - открытие порта без DTR reset (dsrdtr=False)
  - ожидание первого валидного фрейма телеметрии от Arduino
  - закрытие порта
"""

import time
from typing import Optional

import serial

from rtk2026_driver.protocol import parse_telemetry, TelemetryFrame, InvalidChecksumError


class HandshakeTimeoutError(Exception):
    """Arduino did not send a valid telemetry frame within the timeout period."""


class SerialTransport:
    """
    Обёртка над serial.Serial с handshake логикой.

    Все таймауты и интервалы задаются снаружи через конструктор —
    конкретные значения живут в arduino_bridge.yaml.

    Использование:
        transport = SerialTransport("/dev/ttyUSB0", 115200,
                                    handshake_timeout_sec=5.0,
                                    handshake_poll_interval_sec=0.01)
        transport.open()   # открывает порт и дожидается готовности Arduino
        transport.write(pack_command(0, 0))
        data = transport.read()
        transport.close()
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        handshake_timeout_sec: float,
        handshake_poll_interval_sec: float,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._handshake_timeout = handshake_timeout_sec
        self._handshake_poll_interval = handshake_poll_interval_sec
        self._ser: Optional[serial.Serial] = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> TelemetryFrame:
        """
        Открыть serial порт и выполнить handshake с Arduino.

        dsrdtr=False — не дёргаем DTR при открытии, Arduino не ресетится.

        Возвращает первый валидный фрейм телеметрии — Arduino готов к работе.
        Выбрасывает HandshakeTimeoutError если Arduino не ответил за handshake_timeout_sec.
        """
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=0,       # неблокирующее чтение
            dsrdtr=False,    # не ресетим Arduino через DTR
            xonxoff=False,
            rtscts=False,
        )
        return self._wait_for_handshake()

    def _wait_for_handshake(self) -> TelemetryFrame:
        """
        Ждём первый валидный фрейм телеметрии от Arduino.

        Первый фрейм означает что Arduino:
          - инициализировал энкодеры и моторы
          - начал цикл отправки телеметрии
          - протокол согласован (заголовок и checksum верны)
        """
        buf = bytearray()
        t_start = time.monotonic()

        while time.monotonic() - t_start < self._handshake_timeout:
            raw = self._ser.read(256)
            if raw:
                buf.extend(raw)
                try:
                    frame = parse_telemetry(buf)
                    if frame is not None:
                        # первый валидный фрейм — Arduino готов
                        return frame
                except InvalidChecksumError:
                    # битый фрейм в начале — Arduino возможно ещё инициализируется,
                    # чистим буфер и ждём следующий фрейм
                    buf.clear()

            time.sleep(self._handshake_poll_interval)

        raise HandshakeTimeoutError(
            f"Arduino on port {self._port} did not respond within {self._handshake_timeout}s"
        )

    def write(self, data: bytes) -> None:
        """Отправить байты на Arduino."""
        if self._ser and self._ser.is_open:
            self._ser.write(data)

    def read(self, size: int = 256) -> bytes:
        """Прочитать доступные байты (неблокирующее)."""
        if self._ser and self._ser.is_open:
            return self._ser.read(size)
        return b""

    def close(self) -> None:
        """Закрыть serial порт."""
        if self._ser and self._ser.is_open:
            self._ser.close()
