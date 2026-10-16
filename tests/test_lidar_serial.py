"""Проверки чтения USB/serial лидара: подбор порта, протокола и скорости.

Железа в тестах нет, поэтому pyserial подменяется виртуальными устройствами:
«RPLIDAR» (отвечает на SCAN и отдаёт точки, пока крутится мотор), «LD06/LD19»
(односторонний поток пакетов 0x54 0x2C) и «LDS-01/LDS-02» — лидар TurtleBot3
Burger, который сыплет кадры 0xFA по 42 байта. Это позволяет проверить ровно те
ситуации, из-за которых на экране было «скана лидара нет»: мотор не включён
линией DTR/RTS, другая скорость порта (A3/S1 — 256000), лидар не той модели
(кадры 0xFA против 0x54 на одной и той же скорости 230400), порт занят вторым
процессом робота.
"""

from __future__ import annotations

import importlib.util
import json
import logging
import math
import os
import pathlib
import random
import sys
import threading
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from robot_control import lidar
from robot_control.lidar import (
    Lds01Stream,
    Ld06Stream,
    LidarManager,
    RplidarStream,
    ScanAccumulator,
    discover_ports,
    ld06_crc_ok,
    lds01_checksum,
    lds01_packet_crc_ok,
    lidar_profiles,
    normalize_angle,
    parse_lds01_packet,
    parse_rplidar_point,
    probe_port,
    scan_looks_like_rotation,
)

# ---------------------------------------------------------------------------
# Кодирование потоков «устройств»
# ---------------------------------------------------------------------------


def encode_rplidar_node(angle_deg: float, dist_m: float, *, start: bool = False, quality: int = 10) -> bytes:
    """5-байтовый пакет RPLIDAR (обратный к parse_rplidar_point)."""
    angle_q6 = int(round((angle_deg % 360.0) * 64.0)) & 0x7FFF
    dist_q2 = int(round(dist_m * 1000.0 * 4.0)) & 0x3FFF
    b0 = ((quality & 0x3F) << 2) | (0x01 if start else 0x02)
    b1 = ((angle_q6 & 0x7F) << 1) | 0x01
    b2 = (angle_q6 >> 7) & 0xFF
    b3 = dist_q2 & 0xFF
    b4 = (dist_q2 >> 8) & 0xFF
    return bytes([b0, b1, b2, b3, b4])


def encode_ld06_packet(start_deg: float, dist_m: float = 1.0, *, crc_ok: bool = True) -> bytes:
    """47-байтовый пакет LD06/LD19 с 12 точками."""
    pkt = bytearray(47)
    pkt[0], pkt[1] = 0x54, 0x2C
    pkt[2], pkt[3] = 0xE8, 0x03          # скорость, °/с
    start = int(round((start_deg % 360.0) * 100.0)) & 0xFFFF
    pkt[4], pkt[5] = start & 0xFF, start >> 8
    for i in range(12):
        idx = 6 + i * 3
        dist_mm = int(round(dist_m * 1000.0))
        pkt[idx] = dist_mm & 0xFF
        pkt[idx + 1] = (dist_mm >> 8) & 0xFF
        pkt[idx + 2] = 200                # достоверность
    end = int(round(((start_deg + 27.0) % 360.0) * 100.0)) & 0xFFFF
    pkt[42], pkt[43] = end & 0xFF, end >> 8
    crc = 0
    for byte in pkt[:46]:
        crc ^= byte
    pkt[46] = crc if crc_ok else (crc ^ 0xFF)
    return bytes(pkt)


def encode_lds01_packet(
    index: int,
    dist_m: float = 1.5,
    *,
    rpm: int = 300,
    crc_ok: bool = True,
    invalid: bool = False,
) -> bytes:
    """42-байтовый пакет LDS-01/LDS-02 (обратный к parse_lds01_packet)."""
    pkt = bytearray(42)
    pkt[0] = 0xFA
    pkt[1] = 0xA0 + (index % 60)
    pkt[2], pkt[3] = rpm & 0xFF, (rpm >> 8) & 0xFF
    for i in range(6):
        off = 4 + i * 6
        dist_mm = int(round(dist_m * 1000.0)) + i
        pkt[off], pkt[off + 1] = 200, 0                     # интенсивность
        word = dist_mm | (0x8000 if invalid else 0)
        pkt[off + 2], pkt[off + 3] = word & 0xFF, (word >> 8) & 0xFF
    chk = lds01_checksum(bytes(pkt[:40]))
    pkt[40], pkt[41] = chk & 0xFF, (chk >> 8) & 0xFF
    if not crc_ok:
        pkt[41] ^= 0xFF
    return bytes(pkt)


# ---------------------------------------------------------------------------
# Виртуальные устройства и модуль serial
# ---------------------------------------------------------------------------


class FakeDevice:
    """Базовое устройство: умеет отдавать байты и запоминать запись."""

    def __init__(self, *, baud: int) -> None:
        self.baud = baud
        self.lines = {"dtr": True, "rts": True}
        self.buffer = bytearray()
        self.written: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if port.baudrate != self.baud:
            return b""  # устройство живёт на своей скорости
        if not self.buffer:
            return b""
        chunk = bytes(self.buffer[:size])
        del self.buffer[:size]
        return chunk

    def flush_input(self) -> None:
        pass


class FakeRplidar(FakeDevice):
    """Виртуальный RPLIDAR: мотор включается линией USB-переходника."""

    def __init__(
        self,
        *,
        baud: int = 115200,
        motor_line: str = "dtr",
        motor_level: bool = False,
        points_per_rev: int = 30,
        revolutions: int = 3,
        respond: bool = True,
    ) -> None:
        super().__init__(baud=baud)
        self.motor_line = motor_line
        self.motor_level = motor_level
        self.points_per_rev = points_per_rev
        self.revolutions = revolutions
        self.respond = respond
        self.scanning = False
        self.emitted = 0
        self.scan_commands = 0

    @property
    def motor_on(self) -> bool:
        return bool(self.lines.get(self.motor_line)) is bool(self.motor_level)

    def write(self, data: bytes) -> int:
        size = super().write(data)
        if bytes(data) == lidar.RPLIDAR_CMD_SCAN:
            self.scan_commands += 1
            if self.respond:
                self.scanning = True
                self.buffer.extend(bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81]))
        elif bytes(data) == lidar.RPLIDAR_CMD_STOP:
            self.scanning = False
        return size

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if port.baudrate != self.baud:
            return b""
        if self.scanning and self.motor_on and self.emitted < self.revolutions:
            self._emit_revolution()
        return super().read(size, port)

    def _emit_revolution(self) -> None:
        self.emitted += 1
        for i in range(self.points_per_rev):
            self.buffer.extend(
                encode_rplidar_node(
                    angle_deg=i * 360.0 / self.points_per_rev,
                    dist_m=1.0 + i * 0.01,
                    start=(i == 0),
                )
            )
        # точка с битом начала закрывает текущий виток у читателя
        self.buffer.extend(encode_rplidar_node(0.0, 2.0, start=True))


class FakeLd06(FakeDevice):
    """Виртуальный LD06/LD19: поток пакетов без команд управления.

    По умолчанию поток длинный (100 витков), чтобы перебор на одной скорости
    (lds01 230400 vs ld06 230400) не съедал весь запас первой неудачной попыткой.
    Для бесконечного потока передай revolutions=None.
    """

    def __init__(
        self,
        *,
        baud: int = 230400,
        revolutions: int | None = 100,
        packets_per_rev: int = 12,
        crc_ok: bool = True,
    ) -> None:
        super().__init__(baud=baud)
        self.revolutions = revolutions
        self.packets_per_rev = packets_per_rev
        self.crc_ok = crc_ok
        self.sent = 0
        self._started = time.monotonic()
        self._emitted = 0

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if port.baudrate != self.baud:
            return b""
        # Ограничиваем скорость реальным бодрейтом (как FakeNoise), но с
        # начальным запасом, чтобы первый read не вернул пусто.
        elapsed = time.monotonic() - self._started
        allowed = int(max(1024, elapsed * self.baud / 10.0))
        while len(self.buffer) < size and self._emitted < allowed and (
            self.revolutions is None or self.sent < self.revolutions * self.packets_per_rev
        ):
            step = 360.0 / self.packets_per_rev
            start_deg = (self.sent % self.packets_per_rev) * step
            self.buffer.extend(
                encode_ld06_packet(start_deg, dist_m=1.0 + 0.01 * self.sent, crc_ok=self.crc_ok)
            )
            self.sent += 1
            self._emitted += 47
        return super().read(size, port)


class FakeLds01(FakeDevice):
    """Виртуальный LDS-01/LDS-02 — лидар TurtleBot3 Burger (HLS-LFCD2).

    Отдаёт кадры 0xFA по 42 байта (60 пакетов = 360 точек). ``needs_start=True``
    повторяет прошивки до 2017.10: без команды ``'b'`` мотор не крутится и данных
    нет. ``needs_start=False`` — новый firmware, поток идёт сразу.
    """

    def __init__(
        self,
        *,
        baud: int = 230400,
        frames: int | None = None,
        needs_start: bool = True,
        dist_m: float = 1.5,
        crc_ok: bool = True,
        ignore_baud: bool = False,
    ) -> None:
        super().__init__(baud=baud)
        self.frames = frames        # None — поток не кончается, как у настоящего лидара
        self.needs_start = needs_start
        self.dist_m = dist_m
        self.crc_ok = crc_ok
        #: Отдавать кадры на любой скорости — так проверяем, что чужой поток не
        #: сходит за RPLIDAR/LD06, когда скорость порта угадана неверно.
        self.ignore_baud = ignore_baud
        self.scanning = not needs_start
        self.sent = 0             # сколько пакетов уже выдано
        self.started = False      # приходила ли команда запуска
        self.stopped = False
        self._baud_started = time.monotonic()
        self._baud_emitted = 0

    @property
    def total_packets(self) -> int | None:
        return None if self.frames is None else self.frames * 60

    def write(self, data: bytes) -> int:
        size = super().write(data)
        if bytes(data) == lidar.LDS01_CMD_START:
            self.started = True
            self.scanning = True
        elif bytes(data) == lidar.LDS01_CMD_STOP:
            self.stopped = True
            self.scanning = False
        return size

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if (port.baudrate != self.baud and not self.ignore_baud) or not self.scanning:
            return b""
        elapsed = time.monotonic() - self._baud_started
        allowed = int(max(1024, elapsed * self.baud / 10.0))
        while len(self.buffer) < size and self._baud_emitted < allowed and (
            self.total_packets is None or self.sent < self.total_packets
        ):
            index = self.sent % 60
            self.buffer.extend(
                encode_lds01_packet(
                    index,
                    self.dist_m + 0.001 * (index % 7),
                    crc_ok=self.crc_ok,
                )
            )
            self.sent += 1
            self._baud_emitted += 42
        return super().read(size, port)


class FakeNoise(FakeDevice):
    """Устройство, которое сыплет случайными байтами: это не лидар.

    Раньше такой поток мог сойти за RPLIDAR, если дескриптор ответа на SCAN
    принимался «любыми семью байтами после синхрослова» — на экран шёл мусор.
    Байты выдаются не быстрее скорости порта: иначе «устройство» нагенерировало
    бы за секунду больше, чем настоящий лидар за минуты, и проверка перебора
    перестала бы быть похожей на реальность.
    """

    def __init__(self, *, baud: int = 230400, seed: int = 1) -> None:
        super().__init__(baud=baud)
        self.rng = random.Random(seed)
        self.started = time.monotonic()
        self.emitted = 0

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if port.baudrate != self.baud:
            return b""
        allowed = int((time.monotonic() - self.started) * self.baud / 10.0)  # 10 бит на байт
        while self.emitted < allowed and len(self.buffer) < size:
            self.buffer.append(self.rng.randrange(256))
            self.emitted += 1
        return super().read(size, port)


class FakeSerialError(Exception):
    """Аналог serial.SerialException."""


class FakeSerial:
    """Минимальный порт: только то, что использует robot_control.lidar."""

    devices: dict[str, FakeDevice] = {}
    #: Все попытки открыть порт — по ним видно, что перебор прошёл весь список.
    opened: list[str] = []

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2, exclusive: bool = True) -> None:
        FakeSerial.opened.append(port)
        device = FakeSerial.devices.get(port)
        if device is None:
            raise FakeSerialError("could not open port %s" % port)
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.exclusive = exclusive
        self.device = device
        self.closed = False

    @property
    def dtr(self) -> bool:
        return bool(self.device.lines["dtr"])

    @dtr.setter
    def dtr(self, value: bool) -> None:
        self.device.lines["dtr"] = bool(value)

    @property
    def rts(self) -> bool:
        return bool(self.device.lines["rts"])

    @rts.setter
    def rts(self, value: bool) -> None:
        self.device.lines["rts"] = bool(value)

    def read(self, size: int = 1) -> bytes:
        return self.device.read(size, self)

    def write(self, data: bytes) -> int:
        return self.device.write(bytes(data))

    def reset_input_buffer(self) -> None:
        self.device.flush_input()

    def close(self) -> None:
        self.closed = True

    def __enter__(self) -> "FakeSerial":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


@pytest.fixture
def fake_serial(monkeypatch):
    """Подменяет pyserial и даёт чистую таблицу устройств."""
    module = types.ModuleType("serial")
    module.Serial = FakeSerial
    module.SerialException = FakeSerialError
    monkeypatch.setitem(sys.modules, "serial", module)
    FakeSerial.devices = {}
    FakeSerial.opened = []
    yield module
    FakeSerial.devices = {}
    FakeSerial.opened = []


@pytest.fixture
def fake_usb(tmp_path, monkeypatch):
    """Подменяет /dev/ttyUSB* на существующие файлы во временной папке.

    Возвращает функцию, которая «подключает» очередной USB-порт и отдаёт его
    путь — так проверяется перебор нескольких портов без железа.
    """
    paths: list[str] = []

    def make(name: str) -> str:
        path = tmp_path / name
        path.write_text("")
        paths.append(str(path))
        return str(path)

    def fake_glob(pattern, **_):
        if "ttyUSB" in pattern:
            return sorted(p for p in paths if "ttyUSB" in p)
        if "ttyACM" in pattern:
            return sorted(p for p in paths if "ttyACM" in p)
        return []

    monkeypatch.setattr(lidar.glob, "glob", fake_glob)
    return make


@pytest.fixture
def fake_port(tmp_path):
    """Путь порта, который существует в файловой системе (иначе его не найти)."""
    path = tmp_path / "ttyUSB7"
    path.write_text("")
    return str(path)


@pytest.fixture(autouse=True)
def fast_lidar(monkeypatch):
    """Короткие таймауты и никаких соседних процессов по умолчанию."""
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")
    monkeypatch.setattr(lidar, "LIDAR_PROBE_TIMEOUT_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_DATA_TIMEOUT_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_RESCAN_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_ATTEMPT_MAX_S", 0.25)
    yield


def wait_for(predicate, timeout: float = 3.0, pause: float = 0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(pause)
    return False


# ---------------------------------------------------------------------------
# Разбор потоков
# ---------------------------------------------------------------------------


def test_rplidar_node_round_trip():
    point = parse_rplidar_point(encode_rplidar_node(90.0, 2.0, start=True))
    assert point is not None
    assert point["r"] == 2.0
    assert abs(point["a"] + math.pi / 2) < 0.001  # угол по часовой → против часовой


def test_rplidar_stream_joins_split_packets():
    """Пакет, разрезанный между чтениями порта, собирается обратно."""
    stream = RplidarStream()
    node = encode_rplidar_node(45.0, 3.25, start=True)
    assert stream.feed(node[:2]) == []
    points = stream.feed(node[2:])
    assert len(points) == 1
    assert points[0][0]["r"] == 3.25 and points[0][1] is True


def test_rplidar_stream_recovers_after_garbage():
    """После мусора поток продолжает отдавать настоящие точки витка."""
    stream = RplidarStream()
    nodes = b"".join(
        encode_rplidar_node(i * 30.0, 1.0 + i * 0.1, start=(i == 0)) for i in range(12)
    )
    points = stream.feed(b"\xf7\x00\x9a" + nodes)
    radii = [p["r"] for p, _ in points]
    # на стыке с мусором один пакет может «съесться» ложным совпадением (так же,
    # как у любого драйвера RPLIDAR), но виток восстанавливается и точек хватает
    assert len([r for r in radii if r <= 2.2]) >= 11
    assert len(points) >= 11


def test_ld06_crc_and_revolution_detection():
    good = encode_ld06_packet(0.0)
    assert ld06_crc_ok(good) is True
    assert ld06_crc_ok(encode_ld06_packet(0.0, crc_ok=False)) is False

    stream = Ld06Stream()
    packets = []
    for start_deg in (0.0, 30.0, 60.0, 0.0, 30.0):  # второй виток начинается с 0°
        packets.extend(stream.feed(encode_ld06_packet(start_deg)))
    assert len(packets) == 5
    assert [revolution for _, revolution in packets] == [False, False, False, True, False]
    assert all(len(pts) == 12 for pts, _ in packets)


def test_ld06_stream_tolerates_broken_crc():
    stream = Ld06Stream(check_crc=True, crc_bad_limit=3)
    for _ in range(4):
        stream.feed(encode_ld06_packet(0.0, crc_ok=False))
    assert stream.check_crc is False           # перешли в режим без CRC
    packets = stream.feed(encode_ld06_packet(90.0, crc_ok=False))
    assert len(packets) == 1 and len(packets[0][0]) == 12


def test_lds01_packet_angles_like_robotis_driver():
    """Углы LDS-01 считаются как в драйвере ROBOTIS: 359° − номер точки.

    Нумерация важна: карта должна совпадать с ROS-режимом TurtleBot3, где этот
    лидар работает штатно.
    """
    head = parse_lds01_packet(encode_lds01_packet(0, 1.0))
    assert len(head) == 6
    assert abs(math.degrees(head[0]["a"]) + 1.0) < 0.05     # 359° == −1° (чуть справа от носа)
    assert abs(math.degrees(head[5]["a"]) + 6.0) < 0.05
    assert head[0]["r"] == 1.0

    tail = parse_lds01_packet(encode_lds01_packet(59, 1.0))
    assert abs(math.degrees(tail[-1]["a"])) < 0.05          # последняя точка витка — «вперёд»
    assert math.degrees(tail[0]["a"]) > 0                   # 5° — слева


def test_lds01_packet_flags_and_range():
    """Бит «нет измерения» выбрасывает точку, остальные биты слова — маска."""
    pkt = bytearray(encode_lds01_packet(10, 2.0))
    pkt[6] |= 0x80                     # младший байт первого слова дальности
    pkt[7] |= 0x80                     # старший байт: бит 15 (недостоверно)
    chk = lds01_checksum(bytes(pkt[:40]))
    pkt[40], pkt[41] = chk & 0xFF, (chk >> 8) & 0xFF
    pts = parse_lds01_packet(bytes(pkt))
    assert len(pts) == 5               # первое измерение выброшено

    # Предупреждение о слабом сигнале (бит 14) измерению не мешает: биты маскируются
    pkt = bytearray(encode_lds01_packet(10, 2.0))
    pkt[7] |= 0x40
    chk = lds01_checksum(bytes(pkt[:40]))
    pkt[40], pkt[41] = chk & 0xFF, (chk >> 8) & 0xFF
    pts = parse_lds01_packet(bytes(pkt))
    assert pts[0]["r"] == 2.0


def test_lds01_checksum_matches_driver_algorithm():
    pkt = encode_lds01_packet(7, 1.0)
    assert lds01_packet_crc_ok(pkt) is True
    assert lds01_packet_crc_ok(encode_lds01_packet(7, 1.0, crc_ok=False)) is False
    assert lds01_checksum(bytes(pkt[:40])) == (pkt[40] | (pkt[41] << 8))
    assert 0 <= lds01_checksum(b"\xff" * 40) <= 0x7FFF      # складывается в 15 бит


def test_lds01_stream_joins_split_packets_and_marks_revolution():
    stream = Lds01Stream()
    frame = b"".join(encode_lds01_packet(i) for i in range(60))
    assert stream.feed(frame[:20]) == []                     # пакет разрезан чтением
    packets = stream.feed(frame[20:])
    assert len(packets) == 60
    assert packets[0][1] is True                             # виток начинается пакетом 0xA0
    assert packets[-1][1] is False
    assert sum(1 for _, rev in packets if rev) == 1
    assert sum(len(pts) for pts, _ in packets) == 360
    assert stream.crc_ok == 60 and stream.crc_bad == 0
    assert all(pts for pts, _ in packets)


def test_lds01_stream_drops_packet_with_bad_checksum():
    stream = Lds01Stream()
    assert stream.feed(encode_lds01_packet(0, crc_ok=False)) == []
    assert stream.crc_bad > 0 and stream.packets == 0
    packets = stream.feed(encode_lds01_packet(1))
    assert len(packets) == 1 and stream.packets == 1


def test_lds01_stream_without_crc_confirms_stream_by_packet_numbers():
    """Без контрольной суммы поток подтверждается серией номеров пакетов."""
    stream = Lds01Stream(check_crc=False)
    for index in range(lidar.LDS01_WARMUP_PACKETS - 1):
        assert stream.feed(encode_lds01_packet(index, crc_ok=False)) == []
    assert len(stream.feed(encode_lds01_packet(lidar.LDS01_WARMUP_PACKETS - 1, crc_ok=False))) == 1
    # поток подтверждён — дальше пакеты идут как обычно
    assert len(stream.feed(encode_lds01_packet(lidar.LDS01_WARMUP_PACKETS, crc_ok=False))) == 1


def test_lds01_stream_without_crc_rejects_garbage():
    """«Пакеты» из случайных байт не набирают серию по порядку номеров."""
    rng = random.Random(7)
    stream = Lds01Stream(check_crc=False)
    produced = []
    for _ in range(400):
        produced.extend(stream.feed(bytes([0xFA]) + bytes(rng.randrange(256) for _ in range(63))))
    assert produced == []
    assert stream.crc_ok == 0


def test_lds01_stream_switches_off_crc_for_clones_with_other_formula():
    """Если КС не сходится ни на одном пакете — читаем поток без неё (как у LD06)."""
    stream = Lds01Stream(crc_bad_limit=2)
    for index in range(2):
        stream.feed(encode_lds01_packet(index, crc_ok=False))
    assert stream.check_crc is False
    assert stream.crc_ok == 0 and stream.crc_bad >= 2


def test_lds01_full_scan_publishes_all_360_points():
    published = []
    stream = Lds01Stream()
    acc = ScanAccumulator(published.append, min_points=lidar.LIDAR_MIN_POINTS)
    data = b"".join(encode_lds01_packet(i) for i in range(60)) + encode_lds01_packet(0)
    for points, revolution in stream.feed(data):
        acc.add(points, revolution_done=revolution)
    assert len(published) == 1
    scan = published[0]
    assert len(scan) == 360
    # Все 360 направлений разные и обходятся по кругу (шаг 1°)
    assert len({round(math.degrees(p["a"]), 1) for p in scan}) == 360
    assert scan[100]["a"] < scan[50]["a"]


def test_scan_looks_like_rotation_separates_scan_from_noise():
    """Проверка витка: настоящий скан идёт размеренно и закрывает весь круг."""
    real = [{"a": normalize_angle(math.radians(-1.0 - i)), "r": 2.0} for i in range(360)]
    assert scan_looks_like_rotation(real) is True

    noise = [{"a": a, "r": 1.0} for a in (
        0.1, 2.9, -1.4, 2.2, -0.3, 1.9, -2.7, 0.7, 2.5, -1.9,
        3.0, -0.9, -2.2, 1.1, -2.9, 0.4, 2.0, -1.1, 2.7, -0.1,
    )]
    assert scan_looks_like_rotation(noise) is False
    assert scan_looks_like_rotation([]) is False
    assert scan_looks_like_rotation([{"a": 0.0, "r": 1.0}, {"a": 0.1, "r": 1.0}]) is False

    # Мусор может разбираться «в упор»: шаг нулевой, но все точки в одну сторону.
    # Так выглядит поток чужого протокола, который случайно разобрался ровно.
    stuck = [{"a": -1.558, "r": 2.0} for _ in range(50)]
    assert scan_looks_like_rotation(stuck) is False
    # Дуга в полкруга — ещё не виток: у настоящего скана разрыв между соседями
    # не больше пары градусов, здесь же дыра в 180°.
    half = [{"a": normalize_angle(math.radians(-1.0 - i)), "r": 2.0} for i in range(180)]
    assert scan_looks_like_rotation(half) is False


def test_lds01_stream_never_looks_like_rplidar_scan():
    """Поток LDS-01, разобранный как RPLIDAR, не должен сойти за виток.

    Именно так выглядит ложная находка при неверной скорости: байты 0xFA-кадра
    разбираются «в упор», углы у «точек» почти одинаковые, шаг нулевой — и
    раньше такой виток считался настоящим, а на экран шёл мусор. Устройства
    здесь нет вовсе: проверяем сам разбор.
    """
    data = b"".join(encode_lds01_packet(i) for i in range(120))
    stream = RplidarStream()
    published = []
    acc = ScanAccumulator(published.append, min_points=lidar.LIDAR_MIN_POINTS, gap_s=0.0)
    for start in range(0, len(data), 64):
        for point, is_start in stream.feed(data[start:start + 64]):
            acc.add([point], revolution_done=is_start)
    acc.flush()

    assert published, "поток LDS-01 вообще не разобрался как RPLIDAR — проверять нечего"
    assert all(scan_looks_like_rotation(scan) is False for scan in published)


def test_scan_accumulator_publishes_revolution_and_gap():
    published = []
    acc = ScanAccumulator(published.append, min_points=3, gap_s=0.0)
    acc.add([{"a": 0.0, "r": 1.0}, {"a": 0.1, "r": 1.1}])
    assert published == []
    acc.add([{"a": 0.2, "r": 1.2}], revolution_done=True)
    assert len(published) == 1 and len(published[0]) == 3
    # виток без бита начала: отдаём накопленное по паузе
    acc.add([{"a": 0.3, "r": 1.3}, {"a": 0.4, "r": 1.4}, {"a": 0.5, "r": 1.5}])
    assert len(published) == 2


# ---------------------------------------------------------------------------
# Поиск портов и профилей
# ---------------------------------------------------------------------------


def test_lidar_profiles_order_and_overrides():
    # LDS-01 (TurtleBot3 Burger) — основной лидар робота, поэтому он первый
    assert lidar_profiles("auto")[0] == ("lds01", 230400)
    assert ("ld06", 230400) in lidar_profiles("auto")
    assert ("rplidar", 115200) in lidar_profiles("auto")
    assert lidar_profiles("ld06") == [("ld06", 230400)]
    assert lidar_profiles("auto", 230400)[0] == ("lds01", 230400)
    assert lidar_profiles("rplidar", 256000)[0] == ("rplidar", 256000)


def test_lidar_profiles_know_lds01_turtlebot():
    """LDS-01 (TurtleBot3 Burger) — кадры 0xFA на 230400, как и LD06/LD19."""
    assert lidar_profiles("lds01") == [("lds01", 230400)]
    assert ("lds01", 230400) in lidar_profiles("auto")
    assert lidar_profiles("hls-lfcd2") == [("lds01", 230400)]      # синоним названия
    assert lidar_profiles("turtlebot3") == [("lds01", 230400)]
    # При явной скорости на 230400 проверяются оба потока, которые на ней живут
    assert ("lds01", 230400) in lidar_profiles("auto", 230400)
    # Неизвестное значение — это не «ничего», а обычный перебор
    assert lidar_profiles("что-то своё") == lidar_profiles("auto")


def test_discover_ports_configured_glob_and_exclusions(tmp_path, monkeypatch):
    new = tmp_path / "ttyUSB7"
    other = tmp_path / "ttyUSB9"
    new.write_text("")
    other.write_text("")
    ports = discover_ports(configured=str(tmp_path / "ttyUSB*"))
    assert [p for p in ports if p.endswith(("ttyUSB7", "ttyUSB9"))] == [str(new), str(other)]

    # порт мастера шасси лидару не отдаём
    ports = discover_ports(configured=str(tmp_path / "ttyUSB*"), excluded={str(new)})
    assert str(new) not in ports and str(other) in ports


def test_discover_ports_reads_master_and_modules_from_env(tmp_path, monkeypatch):
    master = tmp_path / "ttyACM0"
    module = tmp_path / "ttyUSB1"
    for path in (master, module):
        path.write_text("")
    monkeypatch.setenv("RC_SLAM_PORT", str(master))
    monkeypatch.setenv("RC_SLAM_PORTS", "%s,%s" % (module, tmp_path / "ttyUSB2"))
    excluded = lidar.excluded_ports()
    assert str(master) in excluded
    assert str(module) in excluded


def test_discover_ports_skips_autoprobe(tmp_path, monkeypatch):
    port = tmp_path / "ttyUSB3"
    port.write_text("")
    monkeypatch.setattr(lidar.glob, "glob", lambda pattern, **_: [str(port)] if "ttyUSB" in pattern else [])
    assert discover_ports(autoprobe=False) == []
    assert str(port) in discover_ports(autoprobe=True)


# ---------------------------------------------------------------------------
# Подбор параметров на порту
# ---------------------------------------------------------------------------


def test_probe_port_finds_rplidar_hidden_behind_motor_line(fake_serial, fake_port):
    """Мотор включается только линией rts-low — перебор обязан до неё дойти."""
    device = FakeRplidar(motor_line="rts", motor_level=False)
    FakeSerial.devices[fake_port] = device
    published = []
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=published.append)
    assert found is not None
    assert found["protocol"] == "rplidar"
    assert found["baud"] == 115200
    assert found["motor"] == "rts-low"          # DTR-варианты не помогли
    assert published and len(published[0]) >= lidar.LIDAR_MIN_POINTS
    assert device.motor_on is True


def test_probe_port_leaves_motor_running(fake_serial, fake_port):
    """RPLIDAR A1/A2: мотор включается DTR=низкий, и линия НЕ глушится в конце.

    Старый код в конце выставлял DTR=True (сброс после открытия порта) и тем
    самым останавливал мотор — скан так и не появлялся.
    """
    device = FakeRplidar(motor_line="dtr", motor_level=False)
    FakeSerial.devices[fake_port] = device
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05)
    assert found is not None and found["motor"] == "dtr-low"
    assert device.lines["dtr"] is False
    assert device.motor_on is True


def test_probe_port_keeps_trying_bauds_for_rplidar_a3(fake_serial, fake_port):
    """RPLIDAR A3 отвечает только на 256000 — как на USB-адаптере с переключателем."""
    FakeSerial.devices[fake_port] = FakeRplidar(baud=256000)
    published = []
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=published.append)
    assert found is not None and found["baud"] == 256000
    assert published


def test_probe_port_finds_ld06(fake_serial, fake_port):
    FakeSerial.devices[fake_port] = FakeLd06(revolutions=None)
    published = []
    stop = {"flag": False}

    def publish(points):
        published.append(points)
        stop["flag"] = True

    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=publish,
                           stop=lambda: stop["flag"])
    assert found is not None
    assert found["protocol"] == "ld06"
    assert found["baud"] == 230400
    assert published and len(published[0]) >= lidar.LIDAR_MIN_POINTS


def test_probe_port_finds_lds01_turtlebot(fake_serial, fake_port):
    """Лидар TurtleBot3 Burger (LDS-01): кадры 0xFA на 230400, мотор — командой 'b'.

    На той же скорости 230400 живёт и LD06/LD19 с кадрами 0x54, поэтому перебор
    обязан отличить один от другого и не выдать мусор за скан.
    """
    device = FakeLds01(needs_start=True)
    FakeSerial.devices[fake_port] = device
    published = []
    stop = {"flag": False}

    def publish(points):
        published.append(points)
        stop["flag"] = True          # как в scripts/lidar_check.py: одного витка достаточно

    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=publish,
                           stop=lambda: stop["flag"])
    assert found is not None
    assert found["protocol"] == "lds01"
    assert found["baud"] == 230400
    assert found["motor"] == "none"              # линии USB-переходника не трогаем
    assert device.started is True                # команда запуска отправлена
    assert published and len(published[0]) == 360
    assert lidar.scan_looks_like_rotation(published[0]) is True


def test_probe_port_finds_lds01_without_start_command(fake_serial, fake_port):
    """Прошивка с 2017.10 крутится сама: команда 'b' не обязательна для данных.

    Поток идёт непрерывно, как у настоящего лидара, и до LDS-01 перебор обязан
    пройти LD06/LD19 на той же скорости, не приняв его кадры за свои.
    """
    FakeSerial.devices[fake_port] = FakeLds01(needs_start=False)
    stop = {"flag": False}

    def publish(points):
        stop["flag"] = True

    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=publish,
                           stop=lambda: stop["flag"])
    assert found is not None and found["protocol"] == "lds01"


def test_probe_port_does_not_take_foreign_stream_for_rplidar(fake_serial, fake_port):
    """Поток не той скорости не должен сойти за RPLIDAR/LD06.

    Устройство отдаёт кадры LDS-01 на любой скорости — так выглядит порт, когда
    скорость угадана неверно: перебор сначала пробует RPLIDAR на 115200, и
    раньше мусорные «витки» из чужих байт могли сойти за настоящий скан (на
    экран шла карта из мусора). Теперь такие витки отбраковываются, а лидар
    находится на своей скорости и со своим протоколом.
    """
    FakeSerial.devices[fake_port] = FakeLds01(needs_start=False, ignore_baud=True)
    published = []
    stop = {"flag": False}

    def publish(points):
        published.append(points)
        stop["flag"] = True      # как в scripts/lidar_check.py: одного витка достаточно

    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=publish,
                           stop=lambda: stop["flag"])
    assert found is not None
    assert found["protocol"] == "lds01"
    assert found["baud"] == 230400
    assert found["motor"] == "none"
    # Виток, на котором лидар найден, — настоящий (мусор отбраковывается).
    assert lidar.scan_looks_like_rotation(published[0]) is True


def test_probe_port_leaves_lds01_motor_running(fake_serial, fake_port):
    """Порт закрывается без команды остановки: иначе мотор каждый раз раскручивать заново."""
    device = FakeLds01()
    FakeSerial.devices[fake_port] = device
    stop = {"flag": False}

    def publish(points):
        stop["flag"] = True

    with FakeSerial(fake_port) as ser:
        assert probe_port(ser, fake_port, timeout=0.05, publish=publish,
                          stop=lambda: stop["flag"]) is not None
    assert device.stopped is False
    assert lidar.LDS01_CMD_STOP not in device.written


def test_probe_port_does_not_take_random_bytes_for_a_lidar(fake_serial, fake_port):
    """Поток случайных байт — не скан: лидар не «находится», и перебор не застревает.

    Раньше на порту с чужим устройством, которое сыплет данными непрерывно,
    чтение шло без ограничения по времени: перебор висел на этом порту вечно и
    до лидара на соседнем USB не доходил.
    """
    FakeSerial.devices[fake_port] = FakeNoise(baud=230400)
    published = []
    started = time.monotonic()
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=published.append)
    assert found is None
    assert published == []
    assert time.monotonic() - started < 30.0


def test_probe_port_publishes_nothing_without_real_points(fake_serial, fake_port):
    """Ответ на SCAN есть, но мотор стоит: «скана» из чужих байт быть не должно."""
    FakeSerial.devices[fake_port] = FakeRplidar(motor_line="dtr", motor_level=False)
    published = []
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, motor="none", publish=published.append)
    assert found is None
    assert published == []


def test_probe_port_ignores_port_without_answer(fake_serial, fake_port):
    FakeSerial.devices[fake_port] = FakeDevice(baud=9600)
    with FakeSerial(fake_port) as ser:
        assert probe_port(ser, fake_port, timeout=0.05) is None


# ---------------------------------------------------------------------------
# Менеджер: рабочий поток, статус, замок порта, второй процесс
# ---------------------------------------------------------------------------


def test_manager_reads_fake_rplidar_and_reports_detected(fake_serial, fake_port, monkeypatch):
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    FakeSerial.devices[fake_port] = FakeRplidar()
    manager = LidarManager()
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan()))
        status = manager.get_status()
        assert status["connected"] is True
        assert status["detected"]["port"] == fake_port
        assert status["detected"]["motor"] == "dtr-low"
        assert status["reason"].startswith("скан идёт")
        assert manager.last_error == ""
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_reads_fake_lds01_and_reports_detected(fake_serial, fake_port, monkeypatch):
    """Порт с лидаром TurtleBot3: менеджер сам находит LDS-01 и отдаёт настоящий скан."""
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    device = FakeLds01()
    FakeSerial.devices[fake_port] = device
    manager = LidarManager()
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan()))
        status = manager.get_status()
        assert status["connected"] is True
        assert status["detected"]["port"] == fake_port
        assert status["detected"]["protocol"] == "lds01"
        assert status["detected"]["baud"] == 230400
        assert status["reason"].startswith("скан идёт")
        assert device.started is True
        scan = manager.get_scan()
        assert len({round(math.degrees(p["a"]), 1) for p in scan}) == 360
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_protocol_env_understands_lds01_aliases(fake_serial, fake_port, monkeypatch):
    """RC_LIDAR_PROTOCOL=lds01 (и синонимы) ограничивает перебор этим протоколом."""
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_PROTOCOL", "turtlebot3")
    FakeSerial.devices[fake_port] = FakeLds01(needs_start=False, frames=2)
    manager = LidarManager()
    assert manager.protocol == "lds01"
    assert lidar.lidar_profiles(manager.protocol) == [("lds01", 230400)]
    assert manager._read_port(fake_serial, fake_port) is True
    assert manager.get_status()["detected"]["protocol"] == "lds01"


def test_manager_reports_missing_pyserial(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", None)  # import serial → ImportError
    manager = LidarManager()
    manager._serial_worker()                          # один проход и выход
    status = manager.get_status()
    assert "pyserial" in status["reason"]
    assert "pyserial" in status["hint"]


def test_manager_reports_unanswered_port(fake_serial, fake_port, monkeypatch):
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    FakeSerial.devices[fake_port] = FakeDevice(baud=9600)
    manager = LidarManager()
    assert manager._read_port(fake_serial, fake_port) is False
    status = manager.get_status()
    assert status["connected"] is False
    assert "не отвечает" in status["reason"]
    assert "256000" in status["hint"]      # подсказка про скорость модели
    assert status["detected"] is None


def test_manager_report_has_help_hint_when_no_port(fake_serial, monkeypatch):
    """Порта нет — в статусе должна быть причина и подсказка, а не тишина."""
    monkeypatch.setenv("RC_LIDAR_PORT", "/dev/ttyNOPE")
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    manager = LidarManager()
    assert manager.candidate_ports() == []
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: "не найден" in manager.get_status()["reason"])
        status = manager.get_status()
        assert "RC_LIDAR_PORT" in status["hint"]
        assert "lidar_check" in status["hint"]
        assert status["candidates"] == []
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_skips_port_of_another_robot_process(fake_serial, fake_port, monkeypatch):
    """Занятый вторым процессом порт не перехватываем, а просим скан у соседа."""
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "http://127.0.0.1:1/api/lidar")  # соседа нет
    FakeSerial.devices[fake_port] = FakeRplidar()
    owner = LidarManager()
    manager = LidarManager()
    assert owner._acquire_port_lock(fake_port) is True
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: "занят" in manager.get_status()["reason"])
        assert manager.get_scan() is None
    finally:
        manager.stop()
        owner._release_port_lock()
        worker.join(timeout=2.0)


def test_port_lock_between_processes(fake_serial, monkeypatch):
    """Порт лидара открывается один раз: второй процесс робота замок не получает."""
    monkeypatch.setenv("RC_LIDAR_PORT", "/dev/ttyTEST")
    FakeSerial.devices["/dev/ttyTEST"] = FakeRplidar()
    first, second = LidarManager(), LidarManager()
    try:
        assert first._acquire_port_lock("/dev/ttyTEST") is True
        assert second._acquire_port_lock("/dev/ttyTEST") is False
        first._release_port_lock()
        assert second._acquire_port_lock("/dev/ttyTEST") is True
    finally:
        first._release_port_lock()
        second._release_port_lock()


def test_second_process_takes_scan_over_http(fake_serial, monkeypatch):
    """Второй процесс получает скан по HTTP — экран не остаётся пустым."""
    payload = {
        "scan": [{"a": 0.5, "r": 1.5}, {"a": -0.5, "r": 2.5}],
        "detected": {"port": "/dev/ttyTEST", "protocol": "rplidar", "baud": 115200, "motor": "dtr-low"},
    }

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):                                     # noqa: N802
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):                         # тишина в тестах
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        monkeypatch.setenv("RC_LIDAR_PEER_URLS", "http://127.0.0.1:%d/api/lidar" % server.server_port)
        manager = LidarManager()
        assert manager._peer_cycle() is True
        assert manager.get_scan() == payload["scan"]
        assert manager.detected["baud"] == 115200
        assert "второго процесса" in manager.reason
        assert manager.get_status()["source"] == "peer"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_manager_goes_through_every_usb_port(fake_serial, fake_usb, monkeypatch):
    """Лидар переподключили на соседний порт — перебор обязан пройти все.

    Раньше поиск начинался всегда с одного и того же порта, и после
    переподключения лидара на другой /dev/ttyUSBx приложение могло упереться
    в пустой порт и никогда не дойти до остальных.
    """
    ports = [fake_usb("ttyUSB0"), fake_usb("ttyUSB1"), fake_usb("ttyUSB2")]
    FakeSerial.devices[ports[2]] = FakeRplidar()       # лидар — на третьем порту
    manager = LidarManager()
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan()))
        assert manager.detected["port"] == ports[2]
        assert set(ports) <= set(FakeSerial.opened), "перебор не дошёл до всех портов"
        status = manager.get_status()
        assert status["connected"] is True
        assert status["detected"]["port"] == ports[2]
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_finds_lidar_on_other_port_when_port_is_pinned(fake_serial, fake_usb, monkeypatch):
    """RC_LIDAR_PORT — только первый в очереди, а не единственный порт.

    С RC_LIDAR_AUTOPROBE=0 поиск сначала идёт по закреплённому порту, но если
    данных там нет, следующий круг всё равно перебирает все USB-порты.
    """
    pinned = fake_usb("ttyUSB0")                       # порт из .env — пустой
    moved = fake_usb("ttyUSB1")                        # лидар переехал сюда
    FakeSerial.devices[moved] = FakeRplidar()
    monkeypatch.setenv("RC_LIDAR_PORT", pinned)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    manager = LidarManager()
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan()))
        assert manager.detected["port"] == moved
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_reports_every_port_it_checked(fake_serial, fake_usb, monkeypatch):
    """Мёртвый порт не сбивает перебор: порты проверяются по очереди, результат виден в статусе."""
    ports = [fake_usb("ttyUSB0"), fake_usb("ttyUSB1")]
    FakeSerial.devices[ports[1]] = FakeLd06()          # лидар — на втором порту
    manager = LidarManager()
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan()))
        assert FakeSerial.opened[:2] == ports, "первый круг должен пройти порты по очереди"
        status = manager.get_status()
        assert status["detected"]["port"] == ports[1]
        assert status["tried"], "в статусе должен быть список проверенных портов"
        assert all(set(item) == {"port", "ok", "note"} for item in status["tried"])
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_logs_one_result_per_round_not_every_cycle(fake_serial, fake_usb, caplog):
    """Итог перебора идёт в журнал один раз, а не каждые 2 секунды.

    Раньше причина менялась от круга к кругу (порядок портов вращался), и
    journalctl на роботе без лидара забивался одинаковыми строками.
    """
    fake_usb("ttyUSB0")
    fake_usb("ttyUSB1")
    manager = LidarManager()
    # Своё имя потока: в общем прогоне рядом могут работать менеджеры других
    # тестов, и их записи в общий журнал не должны попадать в счёт.
    worker = threading.Thread(target=manager._serial_worker, daemon=True, name="lidar-rounds-test")

    def ours(record):
        return record.levelno >= logging.WARNING and record.threadName == worker.name

    with caplog.at_level(logging.WARNING, logger="robot_control.lidar"):
        worker.start()
        try:
            assert wait_for(lambda: any(ours(r) for r in caplog.records))
            time.sleep(0.3)          # за это время (RESCAN=0.05) проходит ещё несколько кругов
        finally:
            manager.stop()
            worker.join(timeout=2.0)
    warnings = [r.getMessage() for r in caplog.records if ours(r)]
    assert len(warnings) == 1, warnings


def test_candidate_ports_rotate_so_search_does_not_stick_to_one_usb(fake_usb, monkeypatch):
    """Каждый круг перебора начинается со следующего порта — очередь идёт по кругу."""
    ports = [fake_usb("ttyUSB0"), fake_usb("ttyUSB1"), fake_usb("ttyUSB2")]
    manager = LidarManager()
    assert manager.candidate_ports() == ports
    assert manager.candidate_ports() == ports[1:] + ports[:1]
    assert manager.candidate_ports() == ports[2:] + ports[:2]
    # круг замкнулся: каждый порт успел побыть первым
    assert manager.candidate_ports() == ports


def test_discover_ports_counts_one_attempt_per_device(tmp_path, monkeypatch):
    """Один и тот же порт по symlink'у и по /dev/ttyUSBx — одна попытка, а не две."""
    real = tmp_path / "ttyUSB4"
    real.write_text("")
    link = tmp_path / "rplidar"
    link.symlink_to(real)

    def fake_glob(pattern, **_):
        if "rplidar" in pattern:
            return [str(link)]
        if "ttyUSB" in pattern:
            return [str(real)]
        return []

    monkeypatch.setattr(lidar.glob, "glob", fake_glob)
    ports = discover_ports(configured=str(link), autoprobe=True)
    assert ports == [str(link)]


# ---------------------------------------------------------------------------
# Проверка на настоящем tty (псевдотерминал) с настоящим pyserial
# ---------------------------------------------------------------------------


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="нужен POSIX-псевдотерминал")
def test_real_pyserial_reads_lds01_from_tty(monkeypatch):
    """Настоящий pyserial, настоящий tty: порт открывается и скан читается.

    Виртуальные устройства подменяют модуль ``serial`` целиком; здесь проверяется
    сам путь pyserial (открытие порта, смена скорости, таймауты) — на
    псевдотерминале, куда «лидар» пишет кадры LDS-01 в темпе порта. Это самое
    близкое к роботу, что можно проверить без железа.
    """
    serial = pytest.importorskip("serial")
    pty = pytest.importorskip("pty")

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    stop = threading.Event()
    written = {"count": 0}

    def device():
        """«Железо»: кадры LDS-01 в темпе 230400 бод, пока не остановят."""
        started = time.monotonic()
        while not stop.is_set():
            allowed = int((time.monotonic() - started) * 23040.0)     # 10 бит на байт
            while written["count"] < allowed:
                os.write(master, encode_lds01_packet(written["count"] % 60, 1.5))
                written["count"] += 1
            time.sleep(0.003)

    thread = threading.Thread(target=device, daemon=True)
    thread.start()
    try:
        monkeypatch.setenv("RC_LIDAR_PORT", port)
        monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
        monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")

        # 1. Подбор параметров на порту настоящим pyserial
        ser = lidar.open_serial(serial, port, 115200)
        published = []
        flag = {"stop": False}

        def publish(points):
            published.append(points)
            flag["stop"] = True

        try:
            found = lidar.probe_port(ser, port, timeout=0.3, publish=publish,
                                     stop=lambda: flag["stop"])
        finally:
            ser.close()
        assert found is not None and found["protocol"] == "lds01"
        assert found["baud"] == 230400
        # «Железо» в тесте пишет быстрее настоящего лидара (5 Гц), поэтому первый
        # опубликованный виток может быть собран по паузе и содержать чуть больше
        # точек, чем один оборот. Важно, что точки реальные и их не меньше порога.
        assert published and 20 <= len(published[0]) <= 400

        # 2. Менеджер целиком — как на роботе
        manager = LidarManager()
        worker = threading.Thread(target=manager._serial_worker, daemon=True)
        worker.start()
        try:
            # Первый виток после подключения может быть неполным (порт открыли
            # в середине кадра) — ждём полный виток, как на экране робота
            assert wait_for(
                lambda: len(manager.get_scan() or []) == 360, timeout=8.0
            )
            status = manager.get_status()
            assert status["connected"] is True
            assert status["detected"]["protocol"] == "lds01"
            scan = manager.get_scan()
            assert len({round(math.degrees(p["a"]), 1) for p in scan}) == 360
            assert min(p["r"] for p in scan) == 1.5
        finally:
            manager.stop()
            worker.join(timeout=3.0)
    finally:
        stop.set()
        thread.join(timeout=2.0)
        os.close(slave)
        os.close(master)


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="нужен POSIX-псевдотерминал")
def test_command_to_a_stalled_port_does_not_hang_the_search():
    """Вставший передатчик не останавливает поиск лидара навсегда.

    ``write`` в pyserial без ``write_timeout`` ждёт готовности порта в ``select()``
    без ограничения времени. На вставшем адаптере (в тесте это pty, сторону
    «устройства» у которого никто не читает) команда SCAN не уходила, и поток
    поиска лидара замирал навсегда — лидар переставал искать порт, а на экране
    оставалась старая причина. Поэтому у порта есть ``write_timeout``.
    """
    serial = pytest.importorskip("serial")
    pty = pytest.importorskip("pty")

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    try:
        ser = lidar.open_serial(serial, port, 115200)
        try:
            started = time.monotonic()
            with pytest.raises(serial.SerialTimeoutException):
                for _ in range(400):                 # переполняем буфер передачи
                    ser.write(b"\xa5\x20" * 64)      # команда SCAN, как у RPLIDAR
            assert time.monotonic() - started < 5.0, "write ждал порт бесконечно"
        finally:
            ser.close()
    finally:
        os.close(slave)
        os.close(master)


# ---------------------------------------------------------------------------
# Диагностический скрипт scripts/lidar_check.py
# ---------------------------------------------------------------------------

CLI_SCRIPT = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "lidar_check.py"


def load_lidar_check():
    """Загружает скрипт как модуль (он рассчитан на запуск с робота)."""
    spec = importlib.util.spec_from_file_location("lidar_check_cli", CLI_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_lidar_check_cli_lists_ports(fake_serial, fake_port, monkeypatch, capsys):
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    module = load_lidar_check()
    assert module.main(["--list"]) == 0
    assert fake_port in capsys.readouterr().out


def test_lidar_check_cli_finds_rplidar(fake_serial, fake_port, capsys):
    FakeSerial.devices[fake_port] = FakeRplidar(baud=256000)
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--seconds", "0.2"]) == 0
    out = capsys.readouterr().out
    assert "256000" in out and "RC_LIDAR_PORT" in out


def test_lidar_check_cli_finds_lds01_turtlebot(fake_serial, fake_port, capsys):
    FakeSerial.devices[fake_port] = FakeLds01()   # поток не кончается, как у настоящего лидара
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--seconds", "0.2"]) == 0
    out = capsys.readouterr().out
    assert "lds01" in out and "TurtleBot3" in out
    assert "230400" in out


def test_lidar_check_cli_reports_dead_port(fake_serial, fake_port, capsys):
    FakeSerial.devices[fake_port] = FakeDevice(baud=9600)
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--seconds", "0.1"]) == 1
    out = capsys.readouterr().out
    assert "не найден" in out
    assert "115200" in out          # подсказка про скорости моделей


def test_lidar_check_cli_json_for_scripts(fake_serial, fake_port, capsys):
    FakeSerial.devices[fake_port] = FakeLd06(revolutions=None)
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--json", "--seconds", "0.2"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["ok"] is True
    assert report["reports"][0]["protocol"] == "ld06"
    assert report["reports"][0]["baud"] == 230400


def test_usb_diagnostics_no_ports(monkeypatch):
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: [])
    monkeypatch.setattr(lidar, "excluded_ports", lambda: set())
    result = lidar.usb_diagnostics()
    assert result["ports"] == []
    assert "не обнаружены" in result["summary"]


@pytest.mark.parametrize("accessible,excluded,expected", [
    (False, False, "Нет прав"),
    (True, False, "доступны"),
    (True, True, "исключены"),
])
def test_usb_diagnostics_access_and_exclusion(monkeypatch, accessible, excluded, expected):
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: ["/dev/ttyUSB0"])
    monkeypatch.setattr(lidar, "excluded_ports", lambda: {"/dev/ttyUSB0"} if excluded else set())
    monkeypatch.setattr(lidar.os, "access", lambda *args: accessible)
    result = lidar.usb_diagnostics()
    assert expected in result["summary"]
    assert result["ports"][0]["accessible"] is accessible
    assert result["ports"][0]["excluded"] is excluded


# ---------------------------------------------------------------------------
# Где физически искать лидар: роли портов, by-path и режим «найти лидар»
# ---------------------------------------------------------------------------


@pytest.fixture
def fake_dev(tmp_path, monkeypatch):
    """Поддельный /dev: tty-устройства и symlink'и by-id/by-path во временной папке.

    ``glob`` подменяется так, что шаблоны ``/dev/ttyUSB*``, ``/dev/ttyACM*`` и
    ``/dev/serial/by-*`` смотрят в неё — путь открытия тот же, что на роботе.
    """
    dev = tmp_path / "dev"
    (dev / "serial" / "by-id").mkdir(parents=True)
    (dev / "serial" / "by-path").mkdir(parents=True)

    def add(name: str, by_id: str = "", by_path: str = "") -> str:
        target = dev / name
        target.write_text("")
        if by_id:
            (dev / "serial" / "by-id" / by_id).symlink_to(target)
        if by_path:
            (dev / "serial" / "by-path" / by_path).symlink_to(target)
        return str(target)

    real_glob = lidar.glob.glob

    def fake_glob(pattern, **kwargs):
        if pattern.startswith("/dev/"):
            pattern = str(dev) + pattern[len("/dev"):]
        return real_glob(pattern, **kwargs)

    monkeypatch.setattr(lidar.glob, "glob", fake_glob)
    for name in ("RC_LIDAR_PORT", "RC_SLAM_PORT", "RC_SLAM_MASTER_PORT", "RC_SLAM_PORTS",
                 "RC_LIDAR_EXCLUDE", "RC_BMS_PORT", "RC_LIDAR_PLACE_NOTE"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(lidar, "chassis_module_ports", lambda: set())
    add.dev = dev  # type: ignore[attr-defined]
    return add


def test_port_inventory_names_roles_and_physical_paths(fake_dev, monkeypatch):
    """Каждый порт получает роль по факту и оба постоянных имени (by-id, by-path)."""
    master = fake_dev("ttyUSB0", by_path="pci-0000:01:00.0-usb-0:1.1:1.0-port0")
    cand = fake_dev("ttyUSB1", by_id="usb-Silicon_Labs_CP2102-if00-port0",
                    by_path="pci-0000:01:00.0-usb-0:1.3:1.0-port0")
    module = fake_dev("ttyACM0")
    busy = fake_dev("ttyUSB2", by_path="pci-0000:01:00.0-usb-0:1.4:1.0-port0")
    monkeypatch.setenv("RC_SLAM_MASTER_PORT", master)
    monkeypatch.setattr(lidar, "chassis_module_ports", lambda: {module})

    rows = {row["device"]: row for row in lidar.port_inventory(
        tried=[{"port": busy, "ok": False, "note": "занят"},
               {"port": cand, "ok": False, "note": "нет данных"}],
        candidates=[cand, busy],
    )}
    assert set(rows) == {master, cand, module, busy}

    assert rows[master]["role"] == "master"
    assert rows[master]["excluded"] is True
    assert rows[master]["excluded_by"] == "RC_SLAM_MASTER_PORT"
    assert "RC_SLAM_MASTER_PORT" in rows[master]["why"]
    assert rows[master]["by_path"] == [str(fake_dev.dev / "serial/by-path/pci-0000:01:00.0-usb-0:1.1:1.0-port0")]

    assert rows[cand]["role"] == "no_response"      # открылся, но лидар не ответил
    assert rows[cand]["candidate"] is True
    assert rows[cand]["by_id"] == [str(fake_dev.dev / "serial/by-id/usb-Silicon_Labs_CP2102-if00-port0")]
    assert rows[cand]["port"] == rows[cand]["by_id"][0]

    assert rows[module]["role"] == "chassis_module"
    assert rows[module]["chassis_module"] is True
    assert rows[module]["by_path"] == [] and rows[module]["by_id"] == []

    assert rows[busy]["role"] == "busy"
    assert "другой программой" in rows[busy]["why"]
    # Подписи ролей — ровно те, что просят на экране.
    labels = {rows[p]["role_label"] for p in rows}
    assert {"мастер руля", "модуль шасси", "занят другим процессом",
            "не отвечает на протокол лидара"} <= labels


def test_port_inventory_untried_port_is_a_candidate(fake_dev):
    port = fake_dev("ttyUSB5")
    (row,) = lidar.port_inventory(candidates=[port])
    assert row["role"] == "candidate"
    assert row["role_label"] == "кандидат на лидар"


def test_port_inventory_marks_chassis_ports_and_bms(fake_dev, monkeypatch):
    chassis = fake_dev("ttyUSB0")
    bms = fake_dev("ttyUSB1")
    monkeypatch.setenv("RC_SLAM_PORTS", chassis)
    monkeypatch.setenv("RC_BMS_PORT", bms)
    rows = {row["device"]: row for row in lidar.port_inventory()}
    assert rows[chassis]["role"] == "chassis" and rows[chassis]["excluded_by"] == "RC_SLAM_PORTS"
    assert rows[bms]["role"] == "bms" and rows[bms]["excluded_by"] == "RC_BMS_PORT"


def test_port_inventory_matches_exclusion_by_symlink(fake_dev, monkeypatch):
    """RC_SLAM_PORT задан symlink'ом by-id — исключение действует на устройство."""
    device = fake_dev("ttyUSB0", by_id="usb-FTDI_FT232R-if00-port0")
    monkeypatch.setenv("RC_SLAM_PORT", str(fake_dev.dev / "serial/by-id/usb-FTDI_FT232R-if00-port0"))
    (row,) = lidar.port_inventory()
    assert row["device"] == device
    assert row["role"] == "master"


def test_port_inventory_reports_configured_port_that_is_missing(fake_dev):
    fake_dev("ttyUSB0")
    rows = lidar.port_inventory(configured="/dev/ttyACM0")
    missing = [row for row in rows if row["role"] == "missing"]
    assert [row["port"] for row in missing] == ["/dev/ttyACM0"]
    assert "RC_LIDAR_PORT" in missing[0]["why"]


def test_port_inventory_connected_lidar_is_marked(fake_dev):
    port = fake_dev("ttyUSB1")
    (row,) = lidar.port_inventory(detected={"port": port}, connected=True)
    assert row["role"] == "lidar"


def test_port_watch_sees_cable_unplugged_and_plugged_back(fake_dev, monkeypatch):
    """Режим «найти лидар»: исчезновение и появление порта — событие с его by-path."""
    fake_dev("ttyUSB0")
    lidar_port = fake_dev("ttyUSB1", by_path="pci-0000:01:00.0-usb-0:1.3:1.0-port0")
    by_path = str(fake_dev.dev / "serial/by-path/pci-0000:01:00.0-usb-0:1.3:1.0-port0")
    watch = lidar.PortWatch()
    status = watch.start()
    assert status["active"] is True
    assert status["events"] == [] and status["highlight"] is None
    assert sorted(status["baseline"]) == sorted(lidar.list_serial_devices())

    watch.poll()
    assert watch.status()["events"] == []        # ничего не менялось — ничего не выдумано

    os.unlink(by_path)
    os.unlink(lidar_port)
    watch.poll()
    status = watch.status()
    assert [e["kind"] for e in status["events"]] == ["disappeared"]
    assert status["events"][0]["device"] == lidar_port
    assert status["events"][0]["by_path"] == [by_path]
    assert status["highlight"] == lidar_port and status["highlight_kind"] == "disappeared"

    fake_dev("ttyUSB1", by_path="pci-0000:01:00.0-usb-0:1.3:1.0-port0")
    watch.poll()
    status = watch.status()
    assert [e["kind"] for e in status["events"]] == ["disappeared", "appeared"]
    assert status["highlight_kind"] == "appeared"

    assert watch.stop()["active"] is False
    os.unlink(lidar_port)
    watch.poll()
    assert len(watch.status()["events"]) == 2    # после остановки не следим


def test_port_watch_expires(fake_dev, monkeypatch):
    watch = lidar.PortWatch()
    watch.start()
    monkeypatch.setattr(lidar, "PORT_WATCH_TTL_S", 0.0)
    watch.poll()
    status = watch.status()
    assert status["active"] is False
    assert "время вышло" in status["stopped_reason"]


def test_ports_overview_in_manager_status(fake_dev, monkeypatch):
    """Статус менеджера несёт блок ports: экран берёт его из /api/kiosk_state."""
    port = fake_dev("ttyUSB1", by_path="pci-usb-0:1.3:1.0-port0")
    monkeypatch.setenv("RC_LIDAR_PLACE_NOTE", "разъём 1.3 — хаб под крышкой")
    manager = lidar.LidarManager()
    try:
        status = manager.get_status()
    finally:
        manager.stop()
    overview = status["ports"]
    assert [row["device"] for row in overview["ports"]] == [port]
    assert overview["find"]["active"] in (False, True)
    assert overview["place_note"] == "разъём 1.3 — хаб под крышкой"
    assert isinstance(overview["usb_devices"], list)
    # Прежние поля никуда не делись: исключение портов мастера/шасси не ослаблено.
    assert "candidates" in status and "chassis_ports" in status and "configured_port" in status


def test_usb_devices_reads_sysfs_without_hubs(tmp_path, monkeypatch):
    root = tmp_path / "usb"
    for name, attrs in {
        "1-1": {"idVendor": "2109", "idProduct": "3431", "bDeviceClass": "09", "product": "USB2.0 Hub"},
        "1-1.3": {"idVendor": "10c4", "idProduct": "ea60", "bDeviceClass": "00",
                  "manufacturer": "Silicon Labs", "product": "CP2102 USB to UART Bridge Controller"},
        "1-1.3:1.0": {"idVendor": "10c4", "idProduct": "ea60"},   # интерфейс, не устройство
        "usb1": {"idVendor": "1d6b", "idProduct": "0002", "bDeviceClass": "09"},
    }.items():
        (root / name).mkdir(parents=True)
        for attr, value in attrs.items():
            (root / name / attr).write_text(value + "\n")
    real_listdir = os.listdir
    monkeypatch.setattr(lidar.os, "listdir", lambda p: real_listdir(root) if p == "/sys/bus/usb/devices" else real_listdir(p))
    real_join = os.path.join
    monkeypatch.setattr(lidar.os.path, "join",
                        lambda a, *rest: real_join(str(root), *rest) if a == "/sys/bus/usb/devices" else real_join(a, *rest))
    devices = lidar.usb_devices()
    assert devices == [{"bus_path": "1-1.3", "id": "10c4:ea60", "manufacturer": "Silicon Labs",
                        "product": "CP2102 USB to UART Bridge Controller"}]


def test_ports_overview_without_ports_reports_real_tty_and_lsusb(fake_dev, monkeypatch):
    """Портов нет — в блоке реальный результат ls /dev/tty* и lsusb, не выдумка."""
    monkeypatch.setattr(lidar, "lsusb_output",
                        lambda: {"available": False, "lines": [], "error": "lsusb не установлен (пакет usbutils)"})
    overview = lidar.ports_overview(None)
    assert overview["ports"] == []
    assert overview["tty"]["usb_like"] == []
    assert overview["tty"]["count"] == len(overview["tty"]["names"])
    assert overview["lsusb"]["available"] is False
    assert "lsusb" in overview["lsusb"]["error"]
    # С портами листинг tty не нужен — его нет (экран не показывает лишнего).
    fake_dev("ttyUSB0")
    assert "tty" not in lidar.ports_overview(None)


def test_lsusb_output_is_honest_when_tool_is_missing(monkeypatch):
    import shutil
    monkeypatch.setattr(shutil, "which", lambda name: None)
    result = lidar.lsusb_output()
    assert result == {"available": False, "lines": [], "error": "lsusb не установлен (пакет usbutils)"}


def test_ports_block_is_real_only(fake_dev, monkeypatch):
    """Блок «Порты на роботе» — только реальное состояние, даже при RC_SLAM_SOURCE=sim."""
    monkeypatch.setenv("RC_SLAM_SOURCE", "sim")
    monkeypatch.setenv("RC_LIDAR_SOURCE", "serial")
    port = fake_dev("ttyUSB3", by_path="pci-usb-0:1.2:1.0-port0")
    overview = lidar.ports_overview(None)
    assert overview["real_only"] is True
    assert [row["device"] for row in overview["ports"]] == [port]     # ровно то, что есть в системе
    blob = json.dumps(overview, ensure_ascii=False).lower()
    for word in ("demo", "демо", "sim", "симул", "example", "пример"):
        assert word not in blob, word
    for row in overview["ports"]:
        assert row["by_path"] and all(os.path.islink(p) for p in row["by_path"])
