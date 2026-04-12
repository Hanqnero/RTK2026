#!/usr/bin/env python3
"""
serial_test.py — тест протокола Arduino через Serial без ROS.

TX → Arduino : velocity packet, 7 байт
  [0xA5, 0x5A, left_tps_lo, left_tps_hi, right_tps_lo, right_tps_hi, checksum]
  left_tps / right_tps : int16, тиков/сек, little-endian

RX ← Arduino : telemetry packet, 19 байт
  [0x5A, 0xA5, left_speed(i32), left_cnt(i32), right_speed(i32), right_cnt(i32), checksum]

Использование:
  python3 tools/serial_test.py
  python3 tools/serial_test.py --port /dev/tty.usbserial-XXXX
  python3 tools/serial_test.py --demo
"""

import argparse
import struct
import sys
import time
import threading

import serial

# ── Протокол ──────────────────────────────────────────────────────────────
VEL_HEADER  = b"\xA5\x5A"
TEL_HEADER  = b"\x5A\xA5"
VEL_SIZE    = 7
TEL_SIZE    = 19


def make_vel_packet(left_tps: int, right_tps: int) -> bytes:
    """Упаковать velocity пакет. tps: тиков/сек, -32768..32767"""
    left_tps  = max(-32767, min(32767, left_tps))
    right_tps = max(-32767, min(32767, right_tps))
    body = VEL_HEADER + struct.pack("<hh", left_tps, right_tps)
    checksum = sum(body) & 0xFF
    return body + bytes([checksum])


def parse_telemetry(raw: bytes):
    """Вернуть (left_speed, left_cnt, right_speed, right_cnt) или None."""
    if len(raw) != TEL_SIZE or raw[:2] != TEL_HEADER:
        return None
    if (sum(raw[:-1]) & 0xFF) != raw[-1]:
        return None
    return struct.unpack_from("<iiii", raw, 2)


# ── Фоновый поток чтения ──────────────────────────────────────────────────
_last_telem = None
_telem_lock = threading.Lock()
_stop_evt   = threading.Event()


def _reader(ser: serial.Serial):
    buf = bytearray()
    while not _stop_evt.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except Exception:
            break
        if not chunk:
            continue
        buf.extend(chunk)
        while len(buf) >= TEL_SIZE:
            idx = buf.find(TEL_HEADER)
            if idx == -1:
                buf.clear()
                break
            if idx > 0:
                buf = buf[idx:]
            if len(buf) < TEL_SIZE:
                break
            parsed = parse_telemetry(bytes(buf[:TEL_SIZE]))
            buf = buf[TEL_SIZE:]
            if parsed:
                with _telem_lock:
                    global _last_telem
                    _last_telem = parsed


def get_telem():
    with _telem_lock:
        return _last_telem


# ── Отправка / вывод ──────────────────────────────────────────────────────
def send_vel(ser: serial.Serial, left_tps: int, right_tps: int):
    ser.write(make_vel_packet(left_tps, right_tps))


def print_telem(tag: str = ""):
    t = get_telem()
    if t:
        l_spd, l_cnt, r_spd, r_cnt = t
        print(f"  {tag:14s}  L_cnt={l_cnt:8d}  R_cnt={r_cnt:8d}"
              f"  L_spd={l_spd:6d} tks/s  R_spd={r_spd:6d} tks/s")
    else:
        print(f"  {tag:14s}  (нет данных от Arduino)")


# ── Демо-сценарий ──────────────────────────────────────────────────────────
def run_demo(ser: serial.Serial):
    # (название, left_tps, right_tps, длительность сек)
    steps = [
        ("ВПЕРЁД",     300,  300, 2.0),
        ("СТОП",          0,    0, 1.0),
        ("НАЗАД",      -300, -300, 2.0),
        ("СТОП",          0,    0, 1.0),
        ("ПОВОРОТ Л",  -200,  200, 1.0),
        ("СТОП",          0,    0, 0.5),
        ("ПОВОРОТ П",   200, -200, 1.0),
        ("СТОП",          0,    0, 1.0),
    ]
    print("\n=== DEMO START ===")
    for name, l, r, dur in steps:
        print(f"\n>> {name}  (L={l:+d} R={r:+d} tks/s, {dur:.1f} с)")
        send_vel(ser, l, r)
        t0 = time.monotonic()
        last_send = time.monotonic()
        while time.monotonic() - t0 < dur:
            if time.monotonic() - last_send >= 0.1:
                send_vel(ser, l, r)
                last_send = time.monotonic()
            time.sleep(0.05)
            print_telem()
    send_vel(ser, 0, 0)
    print("\n=== DEMO DONE ===")
    print_telem("ФИНАЛ")


# ── Интерактив ────────────────────────────────────────────────────────────
HELP = """
Команды (скорости в тиках/сек):
  f <tps>           вперёд оба мотора
  b <tps>           назад  (tps > 0)
  l <left> <right>  задать tps каждому (-32767..32767)
  s                 стоп
  t                 показать последнюю телеметрию
  demo              авто-сценарий
  q                 выход
"""


def run_interactive(ser: serial.Serial):
    print(HELP)
    while True:
        try:
            line = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            break
        elif cmd == "s":
            send_vel(ser, 0, 0)
            print("  стоп")
        elif cmd == "t":
            print_telem()
        elif cmd == "demo":
            run_demo(ser)
        elif cmd == "f" and len(parts) == 2:
            tps = int(parts[1])
            send_vel(ser, tps, tps)
            print(f"  вперёд {tps} tks/s")
        elif cmd == "b" and len(parts) == 2:
            tps = int(parts[1])
            send_vel(ser, -tps, -tps)
            print(f"  назад {tps} tks/s")
        elif cmd == "l" and len(parts) == 3:
            l, r = int(parts[1]), int(parts[2])
            send_vel(ser, l, r)
            print(f"  L={l:+d}  R={r:+d} tks/s")
        else:
            print("  ?  введи help для списка команд")


# ── Main ──────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true")
    args = ap.parse_args()

    port = args.port
    if port is None:
        from serial.tools import list_ports
        candidates = [
            p.device for p in list_ports.comports()
            if any(k in (p.description or "") for k in ("CH340", "USB Serial", "Arduino"))
            or any(p.device.endswith(s) for s in ("ttyUSB0", "ttyUSB1", "ttyACM0", "ttyACM1"))
        ]
        if not candidates:
            print("Порт не найден. Укажи --port /dev/ttyXXX")
            sys.exit(1)
        port = candidates[0]
        print(f"Авто-выбран порт: {port}")

    print(f"Открываю {port} @ {args.baud}...")
    ser = serial.Serial(port, args.baud, timeout=0.1, write_timeout=0.5)
    print("Жду загрузки Arduino (3 с)...")
    time.sleep(3.0)
    ser.reset_input_buffer()
    print("Готово.\n")

    reader = threading.Thread(target=_reader, args=(ser,), daemon=True)
    reader.start()

    try:
        if args.demo:
            run_demo(ser)
        else:
            run_interactive(ser)
    finally:
        _stop_evt.set()
        send_vel(ser, 0, 0)
        time.sleep(0.1)
        ser.close()
        print("Порт закрыт.")


if __name__ == "__main__":
    main()
