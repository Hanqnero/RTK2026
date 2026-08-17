"""
! Низкоуровневый serial-транспорт.

Класс ничего не знает о ROS2, TwistStamped, Odometry или TF. Его задачи:

    1. открыть serial-порт;
    2. подождать перезагрузку Arduino после открытия USB;
    3. читать доступные байты без длительной блокировки;
    4. писать готовые бинарные пакеты;
    5. корректно закрыть порт.

Разделение транспорта и ROS2-ноды полезно тем, что бинарный обмен можно тестировать отдельно от ROS.
"""

from __future__ import annotations

import threading
import time

import serial


class SerialTransport:
    """
    Потокобезопасная обёртка над pyserial.Serial.

    Lock нужен на случай, если позднее нода будет запущена через
    MultiThreadedExecutor и callbacks чтения и записи смогут выполняться
    параллельно.

    При обычном SingleThreadedExecutor callbacks выполняются последовательно,
    но наличие блокировки делает класс независимым от типа executor.
    """

    def __init__(
        self,
        port: str,  # Имя порта
        baudrate: int, # Скорость передачи данных в бодах
        reset_wait_sec: float, # Время ожидания после сброса устройства (в секундах)
        write_timeout_sec: float = 0.1, # Таймаут для операций записи (в секундах)
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._reset_wait_sec = reset_wait_sec
        self._write_timeout_sec = write_timeout_sec

        self._serial: serial.Serial | None = None  # Объект для работы с портом
        self._lock = threading.Lock() # Блокировка для синхронизации доступа к общему ресурсу

    @property
    def port(self) -> str:
        """Имя serial-устройства, например /dev/serial/by-id/..."""
        return self._port

    @property
    def is_open(self) -> bool:
        """Открыт ли serial-порт в данный момент."""
        return self._serial is not None and self._serial.is_open

    def open(self) -> None:
        """
        ? Открыть порт и подготовить его к работе.

        На Arduino Mega открытие USB serial часто вызывает аппаратный reset
        через линию DTR. Поэтому сразу читать порт нельзя: Arduino должна
        выполнить setup() и начать отправлять телеметрию.

        ~ timeout=0:
            чтение неблокирующее. Если байтов нет, read() сразу возвращает b"".

        ~ write_timeout:
            ограничивает ожидание при переполненном выходном буфере.
        """

        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0.0,
                write_timeout=self._write_timeout_sec,
            )

        # Не держим lock во время sleep: порт уже создан, но другие callbacks
        # ещё не запущены, поскольку open вызывается из конструктора ноды.
        time.sleep(self._reset_wait_sec)

        with self._lock:
            if self._serial is None or not self._serial.is_open:
                raise serial.SerialException(
                    "Serial port was closed during Arduino reset wait"
                )

            # Удаляем возможные байты загрузчика и старую телеметрию,
            # накопившуюся за время перезагрузки.
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

    def read_available(self, max_bytes: int = 512) -> bytes:
        """
        Прочитать уже находящиеся в системном serial-буфере байты.

        Функция не ждёт появления новых данных. Это важно для ROS2 callback:
        callback не должен блокировать executor на десятки миллисекунд.
        """

        with self._lock:
            if self._serial is None or not self._serial.is_open:
                raise serial.SerialException("Attempt to read closed serial port")

            waiting = int(self._serial.in_waiting)
            if waiting <= 0:
                return b""

            read_size = min(waiting, max_bytes)
            return bytes(self._serial.read(read_size))

    def write(self, data: bytes) -> None:
        """
        Записать один готовый бинарный пакет.

        Транспорт намеренно не проверяет семантику данных и не знает
        про кадрирование: сборка кадра и CRC — обязанность protocol.py.
        """

        if not data:
            return

        with self._lock:
            if self._serial is None or not self._serial.is_open:
                raise serial.SerialException("Attempt to write closed serial port")

            written = self._serial.write(data)

            if written != len(data):
                raise serial.SerialTimeoutException(
                    f"Partial serial write: sent {written} of {len(data)} bytes"
                )

    def close(self) -> None:
        """Закрыть serial-порт. Повторный вызов безопасен."""

        with self._lock:
            if self._serial is not None:
                try:
                    if self._serial.is_open:
                        self._serial.close()
                finally:
                    self._serial = None
