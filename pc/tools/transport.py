#!/usr/bin/env python3
"""Транспорт до платы: локальный USB или сеть.

Зачем два транспорта
--------------------

На стенде плата подключена к тому же компьютеру, где запущены инструменты,
и достаточно serial-порта. На роботе плата висит на Raspberry Pi, а смотреть
графики удобнее с ноутбука: у Pi нет ни экрана, ни смысла тащить туда
графическую подсистему.

Поэтому на Pi поднимается ``link_server.py``, который владеет портом
и ретранслирует байты по TCP, а инструменты подключаются к нему так же,
как к порту. Кодек, разбор кадров и вся логика при этом не меняются:
транспорт отдаёт те же байты.

Выбор транспорта
----------------

Адрес разбирается автоматически:

``/dev/ttyUSB0``, ``COM3``
    Локальный serial-порт.

``192.168.1.50:5555``, ``pi.local:5555``
    TCP-подключение к ``link_server.py``.
"""

from __future__ import annotations

import socket
import time
from typing import Protocol

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Требуется pyserial. Установка: pip install pyserial"
    ) from exc

#: Порт TCP по умолчанию, если в адресе указан только хост.
DEFAULT_TCP_PORT = 5555



class Transport(Protocol):
    """Общий интерфейс: столько же методов, сколько реально нужно кодеку."""

    @property
    def in_waiting(self) -> int:
        ...


    def read(self, size: int) -> bytes:
        ...



    def write(self, data: bytes) -> None:
        ...



    def reset_input_buffer(self) -> None:
        ...



    def close(self) -> None:
        ...





class SerialTransport:
    """Локальный serial-порт.

    ``in_waiting`` у pyserial опрашивает драйвер напрямую и на части
    USB-serial адаптеров под macOS возвращает ноль при непустом буфере.
    Поэтому транспорт не спрашивает драйвер "сколько ждёт", а как и
    ``TcpTransport``, сам вычитывает доступное во внутренний буфер
    и отдаёт из него.

    Порт открыт неблокирующим (``timeout=0``), и это не деталь, а
    условие работоспособности. Плата шлёт телеметрию непрерывно, поэтому
    любой ненулевой таймаут превращает чтение в ожидание: ``read(n)``
    у pyserial ждёт либо ``n`` байт, либо истечения таймаута, а пока
    поток не прерывается, цикл "читать до пустоты" не заканчивается.
    Замерено на этой машине: с ``timeout=0.01`` опрос занимал 188 мс
    в медиане и до 760 мс, команды уходили раз в 155 мс вместо 20 мс
    и пробивали дедмен прошивки в 500 мс, то есть привод глох на ходу.
    С ``timeout=0`` тот же опрос стоит 0.04 мс.
    """

    def __init__(self, device: str, baud: int, reset_wait_s: float = 2.0) -> None:
        self.description = f"serial {device} @ {baud}"

        try:
            self._port = serial.Serial(device, baud, timeout=0)
        except serial.SerialException as exc:
            raise SystemExit(f"не удалось открыть {device}: {exc}") from exc

        # Открытие USB обычно перезагружает Mega: до конца setup() читать нечего.
        time.sleep(reset_wait_s)
        self._port.reset_input_buffer()

        self._buffer = bytearray()

    def _drain_serial(self) -> None:
        """Забрать накопившееся в буфере драйвера, без обмана in_waiting.

        Ровно одно чтение: неблокирующий ``read`` отдаёт всё, что есть,
        и сразу возвращается. Цикла здесь быть не должно - см. класс.
        """

        chunk = self._port.read(65536)
        if chunk:
            self._buffer.extend(chunk)

    @property
    def in_waiting(self) -> int:
        self._drain_serial()
        return len(self._buffer)


    def read(self, size: int) -> bytes:
        self._drain_serial()
        chunk = bytes(self._buffer[:size])
        del self._buffer[:size]
        return chunk



    def write(self, data: bytes) -> None:
        self._port.write(data)



    def reset_input_buffer(self) -> None:
        self._port.reset_input_buffer()
        self._buffer.clear()



    def close(self) -> None:
        self._port.close()





class TcpTransport:
    """Подключение к ``link_server.py`` на Raspberry Pi.

    Поток байт тот же, что в serial-порту: сервер ничего не разбирает
    и не переупаковывает, поэтому кадрирование и CRC работают из конца
    в конец, а не до сервера и обратно.
    """

    def __init__(self, host: str, port: int, connect_timeout_s: float = 5.0) -> None:
        self.description = f"tcp {host}:{port}"

        try:
            self._socket = socket.create_connection((host, port), connect_timeout_s)
        except OSError as exc:
            raise SystemExit(
                f"не удалось подключиться к {host}:{port}: {exc}\n"
                "проверьте, что на Raspberry Pi запущен link_server.py"
            ) from exc

        # Неблокирующее чтение: инструменты опрашивают транспорт в своём темпе
        # и не должны застревать в ожидании пакета.
        self._socket.setblocking(False)

        # Отключаем алгоритм Нейгла: команды короткие и должны уходить сразу,
        # иначе к задержке управления добавились бы десятки миллисекунд.
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self._buffer = bytearray()

    def _drain_socket(self) -> None:
        """Забрать всё, что пришло, не блокируясь."""

        while True:
            try:
                chunk = self._socket.recv(65536)
            except BlockingIOError:
                return
            except OSError as exc:
                raise SystemExit(f"соединение потеряно: {exc}") from exc

            if not chunk:
                raise SystemExit("соединение закрыто сервером")

            self._buffer.extend(chunk)

    @property
    def in_waiting(self) -> int:
        self._drain_socket()
        return len(self._buffer)


    def read(self, size: int) -> bytes:
        self._drain_socket()
        chunk = bytes(self._buffer[:size])
        del self._buffer[:size]
        return chunk



    def write(self, data: bytes) -> None:
        try:
            self._socket.sendall(data)
        except OSError as exc:
            raise SystemExit(f"не удалось отправить команду: {exc}") from exc



    def reset_input_buffer(self) -> None:
        self._drain_socket()
        self._buffer.clear()



    def close(self) -> None:
        try:
            self._socket.close()
        except OSError:
            pass





def is_network_target(target: str) -> bool:
    """Отличить сетевой адрес от пути к устройству.

    Путь к устройству всегда начинается со слэша на POSIX или содержит
    ``COM`` на Windows. Сетевой адрес содержит двоеточие и не начинается
    со слэша.
    """

    if target.startswith("/") or target.upper().startswith("COM"):
        return False

    return ":" in target or target.replace(".", "").replace("-", "").isalnum()




def parse_network_target(target: str) -> tuple[str, int]:
    """Разобрать ``host:port`` или ``host`` в пару."""

    if ":" not in target:
        return target, DEFAULT_TCP_PORT

    host, _, port_text = target.rpartition(":")

    try:
        return host, int(port_text)
    except ValueError as exc:
        raise SystemExit(f"неверный порт в адресе {target!r}") from exc




def open_transport(
    target: str,
    baud: int = 115200,
    reset_wait_s: float = 2.0,
) -> Transport:
    """Открыть транспорт по адресу.

    :param target: путь к устройству или ``host:port`` сервера.
    :param baud: скорость serial-порта; для TCP игнорируется, потому что
        скоростью владеет сервер на Raspberry Pi.
    :param reset_wait_s: пауза после открытия USB на перезагрузку платы;
        для TCP не нужна, так как плата уже работает.
    """

    if is_network_target(target):
        host, port = parse_network_target(target)
        return TcpTransport(host, port)

    return SerialTransport(target, baud, reset_wait_s)

