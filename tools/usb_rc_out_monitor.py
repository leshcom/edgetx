#!/usr/bin/env python3
"""
USB RC OUT monitor — перевірка потоку каналів з EdgeTX (режим "USB RC OUT").

Пакет, 39 байт:
  A5 5A | ver=1 | seq | CH1..CH16 int16 LE | flags | CRC16-CCITT LE
  CRC: poly 0x1021, init 0xFFFF, по байтах 0..36.

Використання:
  pip install pyserial
  python usb_rc_out_monitor.py            # автопошук порту (VID 0483 / PID 5740)
  python usb_rc_out_monitor.py COM7       # вказати порт вручну
  python usb_rc_out_monitor.py --raw      # друкувати кожен пакет рядком (без "живого" екрана)
  python usb_rc_out_monitor.py --list     # показати всі COM-порти
"""

import argparse
import os
import struct
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("Потрібен pyserial:  pip install pyserial")

MAGIC = b"\xA5\x5A"
PACKET_SIZE = 39
CHANNELS = 16
PROTO_VERSION = 1
STM_VID, CDC_PID = 0x0483, 0x5740
RANGE = 1536


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


class Parser:
    """Шукає пакети в потоці байтів, рахує статистику."""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.crc_errors = 0
        self.bad_version = 0
        self.lost = 0
        self.resync_bytes = 0
        self.last_seq = None

    def feed(self, data: bytes):
        """Повертає список (seq, channels, flags) для кожного валідного пакета."""
        self.buf += data
        out = []
        while True:
            idx = self.buf.find(MAGIC)
            if idx < 0:
                # залишаємо останній байт — він може бути початком A5
                keep = 1 if self.buf[-1:] == MAGIC[:1] else 0
                self.resync_bytes += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                break
            if idx > 0:
                self.resync_bytes += idx
                del self.buf[:idx]
            if len(self.buf) < PACKET_SIZE:
                break

            pkt = bytes(self.buf[:PACKET_SIZE])
            crc_rx = struct.unpack_from("<H", pkt, 37)[0]
            if crc16_ccitt(pkt[:37]) != crc_rx:
                # хибний заголовок або пошкоджений пакет — зсуваємось на 1 байт
                self.crc_errors += 1
                self.resync_bytes += 1
                del self.buf[:1]
                continue

            del self.buf[:PACKET_SIZE]
            if pkt[2] != PROTO_VERSION:
                self.bad_version += 1
                continue

            seq = pkt[3]
            if self.last_seq is not None:
                gap = (seq - self.last_seq - 1) & 0xFF
                self.lost += gap
            self.last_seq = seq

            channels = struct.unpack_from("<16h", pkt, 4)
            flags = pkt[36]
            self.ok += 1
            out.append((seq, channels, flags))
        return out


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == STM_VID and p.pid == CDC_PID:
            return p.device
    return None


def bar(value, width=32):
    norm = (max(-RANGE, min(RANGE, value)) + RANGE) / (2 * RANGE)
    pos = round(norm * width)
    center = width // 2
    cells = [" "] * width
    lo, hi = sorted((pos, center))
    for i in range(lo, hi):
        cells[i] = "#"
    cells[min(center, width - 1)] = "|"
    return "[" + "".join(cells) + "]"


def draw(port, parser, last, rate):
    lines = [
        f"USB RC OUT monitor  —  {port}   (Ctrl+C — вихід)",
        f"пакетів: {parser.ok}   частота: {rate:5.1f} Гц   "
        f"втрачено (seq): {parser.lost}   CRC-помилок: {parser.crc_errors}   "
        f"сміття байт: {parser.resync_bytes}",
        "",
    ]
    if last is None:
        lines.append("чекаю на пакети... (на пульті має бути вибрано USB RC OUT)")
    else:
        seq, ch, flags = last
        lines.append(f"seq={seq:3d}  flags=0x{flags:02X}")
        for i in range(8):
            a, b = ch[i], ch[i + 8]
            lines.append(
                f"CH{i+1:02d} {a:6d} {bar(a, 24)}    CH{i+9:02d} {b:6d} {bar(b, 24)}"
            )
    # курсор у кут + очистити до кінця екрана
    sys.stdout.write("\x1b[H" + "\n".join(l + "\x1b[K" for l in lines) + "\n\x1b[J")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Перевірка потоку USB RC OUT з EdgeTX")
    ap.add_argument("port", nargs="?", help="COM-порт, напр. COM7 (за замовчуванням — автопошук)")
    ap.add_argument("--raw", action="store_true", help="друкувати кожен пакет окремим рядком")
    ap.add_argument("--list", action="store_true", help="показати COM-порти і вийти")
    args = ap.parse_args()

    if args.list:
        for p in serial.tools.list_ports.comports():
            vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "----:----"
            print(f"{p.device:8s} {vidpid}  {p.description}")
        return

    port = args.port or find_port()
    if not port:
        sys.exit("Не знайшов пульт (VID 0483 / PID 5740). Вкажи порт вручну або подивись --list.")

    if os.name == "nt":
        os.system("")  # вмикає ANSI-коди в консолі Windows

    ser = serial.Serial(port, 115200, timeout=0.05)
    ser.dtr = True  # деякі хости/драйвери не віддають дані без DTR
    parser = Parser()
    last = None
    t_start = time.monotonic()
    window_ok, window_t = 0, t_start
    rate = 0.0
    next_draw = 0.0

    if not args.raw:
        sys.stdout.write("\x1b[2J")

    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                for pkt in parser.feed(data):
                    last = pkt
                    if args.raw:
                        seq, ch, flags = pkt
                        print(f"{time.monotonic() - t_start:8.3f}s seq={seq:3d} "
                              + " ".join(f"{v:5d}" for v in ch))

            now = time.monotonic()
            if now - window_t >= 1.0:
                rate = (parser.ok - window_ok) / (now - window_t)
                window_ok, window_t = parser.ok, now
            if not args.raw and now >= next_draw:
                draw(port, parser, last, rate)
                next_draw = now + 0.1
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        print(f"\nПомилка порту: {e} (кабель витягли або вибрали інший режим USB?)")
    finally:
        ser.close()
        elapsed = time.monotonic() - t_start
        print(f"\nПідсумок за {elapsed:.1f} с: пакетів {parser.ok}, "
              f"середня частота {parser.ok / elapsed if elapsed else 0:.1f} Гц, "
              f"втрачено {parser.lost}, CRC-помилок {parser.crc_errors}, "
              f"невірна версія {parser.bad_version}")


if __name__ == "__main__":
    main()
