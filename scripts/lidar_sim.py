#!/usr/bin/env python3
"""Симулятор USB-лидара на псевдотерминале: стенд без железа.

Скрипт создаёт pty и сыплет в него настоящие кадры лидара (какие понимает
приложение): LDS-01/LDS-02 (TurtleBot3 Burger, 0xFA, 42 байта), LD06/LD19
(0x54 0x2C, 47 байт) или RPLIDAR (ответ на SCAN + узлы 5 байт). Рядом с
симулятором можно запускать пульт и бортовой экран — скан будет идти точно
так же, как с настоящего лидара:

    .venv/bin/python scripts/lidar_sim.py --protocol lds01 --symlink /tmp/lidar_sim

а в другом терминале:

    RC_LIDAR_PORT=/tmp/lidar_sim RC_LIDAR_SOURCE=serial ./scripts/start.sh

Симулятор проверяет весь путь «кабель → подбор параметров → скан → API →
экран» и полезен, когда нужно убедиться, что стенд исправен, а настоящий
лидар отключён. Он не трогает настоящие /dev/ttyUSB* — только свой pty.

Что отдаётся:

  * ``lds01`` (по умолчанию, лидар робота) — 60 пакетов по 6 точек = 360 точек
    за виток, 230400 бод, контрольная сумма ROBOTIS; команда ``b`` запускает
    поток, ``e`` останавливает (как прошивка LDS-01);
  * ``ld06`` — пакеты с 12 точками и XOR-контрольной суммой, 230400 бод;
  * ``rplidar`` — дескриптор A5 5A 05 00 00 40 81 на команду SCAN и узлы
    Standard Scan; мотор считается крутящимся всегда.

Дальности меняются по углу как в комнате (см. ``room_distance``): на карте
видно «помещение», а не идеальный круг — так проще убедиться, что картинка
живая.
"""

from __future__ import annotations

import argparse
import math
import os
import pty
import select
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_control.lidar import (  # noqa: E402
    LDS01_CMD_START,
    LDS01_CMD_STOP,
    RPLIDAR_CMD_SCAN,
    RPLIDAR_CMD_STOP,
    lds01_checksum,
)

#: Байт в секунду на скорости N бод (10 бит на байт: старт + 8 данных + стоп).
def bytes_per_second(baud: int) -> float:
    return baud / 10.0


def room_distance(angle_deg: float, base: float) -> float:
    """Дальность «комнаты» по углу: гранёный контур вместо идеального круга."""
    rad = math.radians(angle_deg)
    dist = base * (1.0 + 0.35 * math.sin(3.0 * rad) + 0.12 * math.sin(7.0 * rad + 0.6))
    return max(0.25, min(8.0, dist))


# --------------------------------------------------------------------------- #
# Кодировщики кадров (обратные к parse_lds01_packet / parse_ld06_packet /
# parse_rplidar_point в robot_control/lidar.py)
# --------------------------------------------------------------------------- #


def encode_lds01_packet(index: int, dist_m: float, *, rpm: int = 300) -> bytes:
    """42-байтовый пакет LDS-01: номер пакета 0…59, 6 точек с дальностью dist_m."""
    pkt = bytearray(42)
    pkt[0] = 0xFA
    pkt[1] = 0xA0 + (index % 60)
    pkt[2], pkt[3] = rpm & 0xFF, (rpm >> 8) & 0xFF
    for i in range(6):
        off = 4 + i * 6
        angle_deg = 359.0 - float((index % 60) * 6 + i)
        dist = room_distance(angle_deg, dist_m)
        dist_mm = int(round(dist * 1000.0)) & 0x3FFF
        pkt[off], pkt[off + 1] = 200, 0                      # интенсивность
        pkt[off + 2], pkt[off + 3] = dist_mm & 0xFF, (dist_mm >> 8) & 0xFF
    chk = lds01_checksum(bytes(pkt[:40]))
    pkt[40], pkt[41] = chk & 0xFF, (chk >> 8) & 0xFF
    return bytes(pkt)


def encode_ld06_packet(start_deg: float, dist_m: float) -> bytes:
    """47-байтовый пакет LD06/LD19: 12 точек от start_deg, XOR в конце."""
    pkt = bytearray(47)
    pkt[0], pkt[1] = 0x54, 0x2C
    pkt[2], pkt[3] = 0xE8, 0x03                               # 1000 °/с
    start = int(round((start_deg % 360.0) * 100.0)) & 0xFFFF
    pkt[4], pkt[5] = start & 0xFF, start >> 8
    for i in range(12):
        angle_deg = (start_deg + i * 2.25) % 360.0
        dist = room_distance(angle_deg, dist_m)
        dist_mm = int(round(dist * 1000.0))
        idx = 6 + i * 3
        pkt[idx] = dist_mm & 0xFF
        pkt[idx + 1] = (dist_mm >> 8) & 0xFF
        pkt[idx + 2] = 200                                    # достоверность
    end = int(round(((start_deg + 27.0) % 360.0) * 100.0)) & 0xFFFF
    pkt[42], pkt[43] = end & 0xFF, end >> 8
    crc = 0
    for byte in pkt[:46]:
        crc ^= byte
    pkt[46] = crc
    return bytes(pkt)


def encode_rplidar_node(angle_deg: float, dist_m: float, *, start: bool = False) -> bytes:
    """5-байтовый узел RPLIDAR Standard Scan."""
    angle_q6 = int(round((angle_deg % 360.0) * 64.0)) & 0x7FFF
    dist_q2 = int(round(dist_m * 1000.0 * 4.0)) & 0x3FFF
    b0 = (10 << 2) | (0x01 if start else 0x02)                # качество + sync
    b1 = ((angle_q6 & 0x7F) << 1) | 0x01
    b2 = (angle_q6 >> 7) & 0xFF
    b3 = dist_q2 & 0xFF
    b4 = (dist_q2 >> 8) & 0xFF
    return bytes([b0, b1, b2, b3, b4])


#: Дескриптор ответа RPLIDAR на SCAN.
RPLIDAR_ANSWER_DESCRIPTOR = bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])


#: Реальные частоты вращения: LDS-01 — 300 об/мин (5 Гц), LD06/RPLIDAR — ~10 Гц.
#: Симулятор держит темп байтами, как настоящий UART, но не быстрее реального
#: лидара: иначе первый виток собирался бы из «забуференного» потока.
REAL_RATE_BYTES = {"lds01": 5 * 2520, "ld06": 10 * 30 * 47, "rplidar": 5 * 360 * 5}


class Simulator:
    """Поток кадров выбранного протокола в master-сторону pty."""

    def __init__(self, master_fd: int, protocol: str, baud: int, dist: float) -> None:
        self.fd = master_fd
        self.protocol = protocol
        self.baud = baud
        self.dist = dist
        self.sent_bytes = 0
        self.packets = 0
        self.revolutions = 0.0
        #: RPLIDAR: поток идёт после SCAN и до STOP; LDS-01 — после b или сразу.
        self.scanning = protocol != "rplidar"
        self._started = time.monotonic()
        self._emitted = 0          # байт, «отправленных» с учётом темпа порта
        self._pending = bytearray()  # кадры, ещё не принятые ядром pty
        self._rate = min(bytes_per_second(baud), float(REAL_RATE_BYTES[protocol]))

    # -- команды от приложения -- #

    def feed_app_write(self, data: bytes) -> None:
        if self.protocol == "rplidar":
            if RPLIDAR_CMD_SCAN in data:
                self.scanning = True
                self._pending.extend(RPLIDAR_ANSWER_DESCRIPTOR)
            elif RPLIDAR_CMD_STOP in data:
                self.scanning = False
        elif self.protocol == "lds01":
            if LDS01_CMD_START in data:
                self.scanning = True
            elif LDS01_CMD_STOP in data:
                self.scanning = False

    # -- генерация кадров -- #

    def _make_frames(self, budget_bytes: int) -> bytes:
        out = bytearray()
        if self.protocol == "lds01":
            while len(out) < budget_bytes:
                index = self.packets % 60
                out.extend(encode_lds01_packet(index, self.dist))
                self.packets += 1
                if self.packets % 60 == 0:
                    self.revolutions += 1
        elif self.protocol == "ld06":
            while len(out) < budget_bytes:
                start = (self.packets % 30) * 12.0
                out.extend(encode_ld06_packet(start, self.dist))
                self.packets += 1
                if self.packets % 30 == 0:
                    self.revolutions += 1
        else:  # rplidar
            points = 360
            while len(out) < budget_bytes:
                for i in range(points):
                    angle = i * 360.0 / points
                    dist = room_distance(-angle, self.dist)  # угол по часовой
                    out.extend(encode_rplidar_node(angle, dist, start=(i == 0)))
                self.revolutions += 1
        return bytes(out)

    def step(self) -> None:
        """Одна итерация: вычитать команды, дописать кадры в порт."""
        try:
            readable, _, _ = select.select([self.fd], [], [], 0)
            if readable:
                data = os.read(self.fd, 4096)
                if data:
                    self.feed_app_write(data)
        except OSError:
            return
        if not self.scanning:
            return
        # Темп как у настоящего UART (бод/10), но не быстрее реального лидара.
        # +2048 — разовый запас на старте: первый виток начинает собираться сразу,
        # а не после нескольких крошечных порций (запас не растёт со временем).
        allowed = int((time.monotonic() - self._started) * self._rate) + 2048
        budget = allowed - self._emitted
        if budget <= 0:
            return
        if len(self._pending) < budget:
            self._pending.extend(self._make_frames(budget - len(self._pending)))
        if len(self._pending) > 65536:
            # Никто не читает порт (приложение не открыло его): как настоящий
            # лидар, сбрасываем буфер и идём дальше, а не копим байты вечно.
            del self._pending[:-2048]
            self._emitted = max(self._emitted, allowed - 2048)
            budget = min(budget, 2048)
        chunk = bytes(self._pending[:budget])
        try:
            written = os.write(self.fd, chunk)
        except BlockingIOError:
            return
        except OSError:
            return
        del self._pending[:written]
        self._emitted += written
        self.sent_bytes += written


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Симулятор USB-лидара на псевдотерминале (стенд без железа)",
    )
    parser.add_argument("--protocol", choices=("lds01", "ld06", "rplidar"),
                        default="lds01", help="какой лидар эмулировать (по умолчанию lds01)")
    parser.add_argument("--baud", type=int, default=0,
                        help="скорость порта (по умолчанию: 230400 у lds01/ld06, 115200 у rplidar)")
    parser.add_argument("--dist", type=float, default=1.5,
                        help="базовая дальность до стен, м (по умолчанию 1.5)")
    parser.add_argument("--symlink", default="",
                        help="постоянное имя порта (символическая ссылка на pty), "
                             "например /tmp/lidar_sim — удобно подставить в RC_LIDAR_PORT")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="остановиться через N секунд (по умолчанию — работать до Ctrl+C)")
    parser.add_argument("--quiet", action="store_true", help="не печатать ход работы")
    args = parser.parse_args(argv)

    baud = args.baud or (115200 if args.protocol == "rplidar" else 230400)
    master, slave = pty.openpty()
    os.set_blocking(master, False)  # как настоящий порт: данные теряются, если никто не читает
    port = os.ttyname(slave)

    if args.symlink:
        link = Path(args.symlink)
        try:
            link.unlink()
        except OSError:
            pass
        link.parent.mkdir(parents=True, exist_ok=True)
        link.symlink_to(port)

    sim = Simulator(master, args.protocol, baud, args.dist)
    stop = {"flag": False}

    def finish(*_):
        stop["flag"] = True

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)

    if not args.quiet:
        print("симулятор лидара: %s, %d бод" % (args.protocol, baud))
        print("порт: %s" % port)
        if args.symlink:
            print("ссылка: %s -> %s" % (args.symlink, port))
        print("проверка без остановки пульта:")
        print("  .venv/bin/python scripts/lidar_check.py --port %s" % (args.symlink or port))
        print("или настроить приложение:")
        print("  RC_LIDAR_PORT=%s RC_LIDAR_SOURCE=serial ./scripts/start.sh" % (args.symlink or port))
        print("остановка: Ctrl+C")

    deadline = time.monotonic() + args.seconds if args.seconds else None
    last_report = time.monotonic()
    try:
        while not stop["flag"]:
            if deadline and time.monotonic() >= deadline:
                break
            sim.step()
            time.sleep(0.004)
            now = time.monotonic()
            if not args.quiet and now - last_report >= 2.0:
                last_report = now
                print("  отправлено %.1f КБ, витков %.1f" %
                      (sim.sent_bytes / 1024.0, sim.revolutions), flush=True)
    finally:
        os.close(slave)
        os.close(master)
        if args.symlink:
            try:
                Path(args.symlink).unlink()
            except OSError:
                pass
    if not args.quiet:
        print("симулятор остановлен (витков: %.1f)" % sim.revolutions)
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
