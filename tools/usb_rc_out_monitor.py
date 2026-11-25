#!/usr/bin/env python3
"""
USB RC OUT monitor — перевірка потоку CRSF-каналів з EdgeTX (режим "USB RC OUT").

Формат — стандартні кадри CRSF RC_CHANNELS_PACKED:
  C8 18 16 | 22 байти: 16 каналів по 11 біт | CRC8 DVB-S2 (poly 0xD5) по [2..24]
  мкс = v * 5 / 8 + 880  (172 → 987, 992 → 1500, 1811 → 2011)

Використання:
  pip install pyserial
  python usb_rc_out_monitor.py            # автопошук порту (VID 0483 / PID 5740)
  python usb_rc_out_monitor.py COM7       # вказати порт вручну
  python usb_rc_out_monitor.py --raw      # друкувати кожен кадр рядком (без "живого" екрана)
  python usb_rc_out_monitor.py --us       # показувати мікросекунди замість сирих значень CRSF
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

CRSF_SYNC = (0xC8, 0xEE, 0xEA, 0xEC)
CRSF_TYPE_RC = 0x16
CRSF_MAX_FRAME = 64
CHANNELS = 16
STM_VID, CDC_PID = 0x0483, 0x5740
RANGE_LO, RANGE_HI = 172, 1811  # CRSF для ±1024 (≈988..2012 мкс)


def crc8_dvb_s2(data) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crsf_to_us(v: int) -> int:
    return (v * 5) // 8 + 880


def unpack_channels(payload: bytes):
    v = int.from_bytes(payload[:22], "little")
    return [(v >> (11 * i)) & 0x7FF for i in range(CHANNELS)]


class Parser:
    """Потоковий розбір CRSF — той самий алгоритм, що й у плагіні Mission Planner."""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.other = 0          # валідні кадри CRSF інших типів
        self.crc_errors = 0
        self.resync_bytes = 0
        self._out = []

    def feed(self, data: bytes):
        """Повертає список (channels_crsf, frame_type) для кожного кадру RC_CHANNELS."""
        self._out = []
        for b in data:
            self._push(b)
        return self._out

    def _push(self, b):
        buf = self.buf
        if len(buf) == 0:
            if b in CRSF_SYNC:
                buf.append(b)
            else:
                self.resync_bytes += 1
            return
        if len(buf) == 1:
            if b < 2 or b > CRSF_MAX_FRAME - 2:
                self.resync_bytes += 1
                buf.clear()
                if b in CRSF_SYNC:
                    buf.append(b)
                return
            buf.append(b)
            return
        buf.append(b)
        frame_len = buf[1] + 2
        if len(buf) < frame_len:
            return
        frame = bytes(buf)
        buf.clear()
        if crc8_dvb_s2(frame[2:frame_len - 1]) == frame[frame_len - 1]:
            if frame[2] == CRSF_TYPE_RC and frame[1] == 24:
                self.ok += 1
                self._out.append((unpack_channels(frame[3:25]), frame[0]))
            else:
                self.other += 1
        else:
            self.crc_errors += 1
            # шукаємо наступний sync-байт усередині відкинутого кадру
            for i in range(1, frame_len):
                if frame[i] in CRSF_SYNC:
                    self.resync_bytes += i
                    for x in frame[i:]:
                        self._push(x)
                    break
            else:
                self.resync_bytes += frame_len


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == STM_VID and p.pid == CDC_PID:
            return p.device
    return None


def bar(v, width=24):
    norm = (max(RANGE_LO, min(RANGE_HI, v)) - RANGE_LO) / (RANGE_HI - RANGE_LO)
    pos = round(norm * width)
    center = width // 2
    cells = [" "] * width
    lo, hi = sorted((pos, center))
    for i in range(lo, hi):
        cells[i] = "#"
    cells[min(center, width - 1)] = "|"
    return "[" + "".join(cells) + "]"


def draw(port, parser, last, rate, as_us):
    lines = [
        f"USB RC OUT (CRSF) monitor  —  {port}   (Ctrl+C — вихід)",
        f"кадрів RC: {parser.ok}   частота: {rate:5.1f} Гц   CRC-помилок: {parser.crc_errors}   "
        f"інших кадрів: {parser.other}   сміття байт: {parser.resync_bytes}",
        "",
    ]
    if last is None:
        lines.append("чекаю на кадри... (на пульті має бути вибрано USB RC OUT)")
    else:
        ch, addr = last
        lines.append(f"адреса 0x{addr:02X}   значення: {'мкс' if as_us else 'CRSF (172..1811, центр 992)'}")
        for i in range(8):
            a, b = ch[i], ch[i + 8]
            va, vb = (crsf_to_us(a), crsf_to_us(b)) if as_us else (a, b)
            lines.append(f"CH{i+1:02d} {va:5d} {bar(a)}    CH{i+9:02d} {vb:5d} {bar(b)}")
    sys.stdout.write("\x1b[H" + "\n".join(l + "\x1b[K" for l in lines) + "\n\x1b[J")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Перевірка потоку USB RC OUT з EdgeTX")
    ap.add_argument("port", nargs="?", help="COM-порт, напр. COM7 (за замовчуванням — автопошук)")
    ap.add_argument("--raw", action="store_true", help="друкувати кожен пакет окремим рядком")
    ap.add_argument("--list", action="store_true", help="показати COM-порти і вийти")
    ap.add_argument("--us", action="store_true", help="показувати мікросекунди")
    ap.add_argument("--baud", type=int, default=420000,
                    help="швидкість (для USB VCP пульта не важлива; для USB-UART з приймачем — 420000)")
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

    ser = serial.Serial(port, args.baud, timeout=0.05)
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
                        ch, _ = pkt
                        vals = [crsf_to_us(v) for v in ch] if args.us else ch
                        print(f"{time.monotonic() - t_start:8.3f}s " + " ".join(f"{v:5d}" for v in vals))

            now = time.monotonic()
            if now - window_t >= 1.0:
                rate = (parser.ok - window_ok) / (now - window_t)
                window_ok, window_t = parser.ok, now
            if not args.raw and now >= next_draw:
                draw(port, parser, last, rate, args.us)
                next_draw = now + 0.1
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        print(f"\nПомилка порту: {e} (кабель витягли або вибрали інший режим USB?)")
    finally:
        ser.close()
        elapsed = time.monotonic() - t_start
        print(f"\nПідсумок за {elapsed:.1f} с: кадрів RC {parser.ok}, "
              f"середня частота {parser.ok / elapsed if elapsed else 0:.1f} Гц, "
              f"CRC-помилок {parser.crc_errors}, інших кадрів {parser.other}")


if __name__ == "__main__":
    main()
