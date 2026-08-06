#!/usr/bin/env python3
"""Ретранслятор serial-порта Arduino в TCP. Запускается на Raspberry Pi.

Зачем
-----

Плата подключена к Raspberry Pi, а графики удобнее смотреть с ноутбука:
у Pi нет экрана, и тащить туда графическую подсистему незачем. Сервер владеет
портом и ретранслирует байты в обе стороны, а инструменты подключаются к нему
так же, как к порту.

Сервер ничего не разбирает и не переупаковывает. Он не знает ни про кадры,
ни про CRC: это важно, потому что тогда кадрирование и контрольные суммы
работают из конца в конец, от прошивки до инструмента, а не до сервера
и обратно. Любое повреждение по дороге будет замечено кодеком на хосте.

Несколько клиентов
------------------

Телеметрия рассылается всем подключённым, поэтому панель состояния и
инструмент настройки могут работать одновременно.

Команды принимаются от всех клиентов и сливаются в один поток. Два
одновременно командующих инструмента будут мешать друг другу: прошивка
исполнит ту команду, что пришла последней. Ограничения на это сервер
не накладывает - он не может отличить осмысленную одновременную работу
от ошибки оператора, - но считает, сколько клиентов писали в порт,
и печатает это в статистике.

Примеры::

    python3 link_server.py --device /dev/ttyUSB0
    python3 link_server.py --device /dev/ttyUSB0 --listen 0.0.0.0 --port 5555
"""

from __future__ import annotations

import argparse
import selectors
import socket
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Требуется pyserial. Установка: pip install pyserial"
    ) from exc

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

DEFAULT_PORT = 5555

#: Период печати статистики.
STATUS_PERIOD_S = 10.0



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ретранслировать serial-порт Arduino в TCP"
    )
    parser.add_argument(
        "--device",
        default="/dev/ttyUSB0",
        help="Путь к serial-устройству Arduino (по умолчанию /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Скорость порта (по умолчанию 115200)"
    )
    parser.add_argument(
        "--listen",
        default="0.0.0.0",
        help="Адрес прослушивания (по умолчанию все интерфейсы)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"TCP-порт (по умолчанию {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--reset-wait",
        type=float,
        default=2.0,
        help="Пауза после открытия порта на перезагрузку платы, секунды",
    )
    parser.add_argument(
        "--quiet", action="store_true", help="Не печатать периодическую статистику"
    )
    return parser.parse_args()




class LinkServer:
    """Двунаправленный ретранслятор между портом и набором клиентов."""

    def __init__(self, device: str, baud: int, reset_wait_s: float) -> None:
        try:
            self.port = serial.Serial(device, baud, timeout=0.0)
        except serial.SerialException as exc:
            raise SystemExit(f"не удалось открыть {device}: {exc}") from exc

        print(f"device = {device}")
        print(f"baud = {baud}")

        # Открытие USB перезагружает Mega: до конца setup() читать нечего.
        time.sleep(reset_wait_s)
        self.port.reset_input_buffer()

        self.selector = selectors.DefaultSelector()
        self.clients: dict[socket.socket, str] = {}

        self.bytes_from_board = 0
        self.bytes_to_board = 0
        self.writers: set[str] = set()


    def listen(self, address: str, tcp_port: int) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((address, tcp_port))
        listener.listen(8)
        listener.setblocking(False)

        self.selector.register(listener, selectors.EVENT_READ, self._accept)
        self.selector.register(self.port, selectors.EVENT_READ, self._read_board)

        print(f"listen = {address}:{tcp_port}")
        print("Ctrl+C для остановки")


    def _accept(self, listener: socket.socket) -> None:
        connection, address = listener.accept()
        connection.setblocking(False)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        name = f"{address[0]}:{address[1]}"
        self.clients[connection] = name
        self.selector.register(connection, selectors.EVENT_READ, self._read_client)

        print(f"client_connected = {name}, clients = {len(self.clients)}")

    def _drop(self, connection: socket.socket) -> None:
        name = self.clients.pop(connection, "?")

        try:
            self.selector.unregister(connection)
        except KeyError:
            pass

        connection.close()
        print(f"client_disconnected = {name}, clients = {len(self.clients)}")

    def _read_board(self, _port) -> None:
        """Прочитать байты платы и разослать всем клиентам."""

        waiting = self.port.in_waiting
        if not waiting:
            return

        data = self.port.read(waiting)
        if not data:
            return

        self.bytes_from_board += len(data)

        for connection in list(self.clients):
            try:
                connection.sendall(data)
            except OSError:
                # Отвалившийся клиент не должен ронять ретрансляцию остальным.
                self._drop(connection)

    def _read_client(self, connection: socket.socket) -> None:
        """Прочитать команду клиента и отправить её плате."""

        try:
            data = connection.recv(4096)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            self._drop(connection)
            return

        if not data:
            self._drop(connection)
            return

        self.bytes_to_board += len(data)
        self.writers.add(self.clients.get(connection, "?"))

        try:
            self.port.write(data)
        except serial.SerialException as exc:
            raise SystemExit(f"порт отвалился при записи: {exc}") from exc


    def serve(self, quiet: bool) -> None:
        last_status = time.monotonic()

        while True:
            for key, _ in self.selector.select(timeout=0.2):
                key.data(key.fileobj)

            now = time.monotonic()
            if not quiet and now - last_status >= STATUS_PERIOD_S:
                last_status = now
                print(
                    f"clients = {len(self.clients)}  "
                    f"bytes_from_board = {self.bytes_from_board}  "
                    f"bytes_to_board = {self.bytes_to_board}  "
                    f"writers = {len(self.writers)}"
                )



    def close(self) -> None:
        for connection in list(self.clients):
            self._drop(connection)

        self.selector.close()

        # Приводы глушим при остановке сервера: иначе робот уедет,
        # если инструмент отвалился раньше.
        try:
            from rtk_link import pack_velocity_command

            self.port.write(pack_velocity_command(0.0, 0.0))
            time.sleep(0.1)
        except Exception:
            pass

        self.port.close()





def main() -> int:
    args = parse_args()

    server = LinkServer(args.device, args.baud, args.reset_wait)

    try:
        server.listen(args.listen, args.port)
        server.serve(args.quiet)
    except KeyboardInterrupt:
        print("\nостановлено")
        return 0
    finally:
        server.close()



if __name__ == "__main__":
    raise SystemExit(main())
