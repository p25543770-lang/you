"""Модуль работы с 2D лидаром 360° (RPLIDAR, LDS-01, LD06/LD19, ROS 2 /scan, HTTP).

Поддерживает:
1. Прямое чтение с USB/serial лидара: RPLIDAR (A1/A2M8 — 115200,
   A2M7/A2M12/A3/S1 — 256000, C1 — 460800), LD06/LD19 (230400) и
   LDS-01/LDS-02 — лидар TurtleBot3 Burger (HLS-LFCD2, кадры 0xFA, 230400).
   Модель и скорость обычно неизвестны заранее, поэтому параметры подбираются
   перебором, а признак успеха — реальные точки скана, а не ответ устройства.
2. Подписку на топик ROS 2 /scan (sensor_msgs/LaserScan), если установлен rclpy.
3. Приём скана через HTTP API (POST /api/lidar или POST /api/scan).
4. Демо-генератор скана (стены склада с препятствиями) для отладки без железа.

Порт можно закрепить вручную: ``RC_LIDAR_PORT=/dev/ttyUSB1`` (можно список или
glob). Остальные настройки: ``RC_LIDAR_BAUD``, ``RC_LIDAR_PROTOCOL``
(auto | rplidar | ld06 | lds01), ``RC_LIDAR_MOTOR`` (auto | none | dtr-low |
dtr-high | rts-low | rts-high), ``RC_LIDAR_AUTOPROBE``, ``RC_LIDAR_EXCLUDE``.

Почему столько осторожности: на роботе одновременно работают два процесса с этим
менеджером (пульт оператора на 8080 и бортовой экран на 8090), а USB-порт лидара
можно открыть только один раз. Поэтому порт защищён межпроцессным замком, а
второй процесс берёт готовый скан у первого по HTTP — иначе половина экранов
навсегда показывала бы «нет скана» без единой строчки в журнале.

Диагностика: ``python3 scripts/lidar_check.py`` печатает, что видно на портах;
причина отсутствия скана лежит в ``GET /api/lidar`` (поле ``reason``) и на
экране робота в подписи «скана лидара нет».
"""

from __future__ import annotations

import glob
import json
import logging
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence, Tuple

try:  # POSIX (робот на Ubuntu): межпроцессный замок на порт лидара.
    import fcntl
except ImportError:  # pragma: no cover - не-POSIX платформы
    fcntl = None  # type: ignore[assignment]

log = logging.getLogger("robot_control.lidar")

LIDAR_MAX_AGE_S = 1.0  # скан старше 1 с считается устаревшим
LIDAR_DEFAULT_RANGE_M = 10.0

#: Таймаут одного чтения порта, с: от него зависят отклик на stop() и переподключение.
SERIAL_READ_TIMEOUT_S = 0.2
#: Сколько ждём реальные точки, прежде чем считать набор параметров неудачным.
LIDAR_PROBE_TIMEOUT_S = 1.4
#: Сколько ждём точки, прежде чем считать, что поток данных оборвался.
LIDAR_DATA_TIMEOUT_S = 1.6
#: Предел одной попытки на порту, пока витка так и не появилось. Нужен потому,
#: что на порту может оказаться чужое устройство с непрерывным потоком: без
#: предела перебор висел на нём вечно и до лидара на соседнем USB не доходил.
LIDAR_ATTEMPT_MAX_S = 4.0
#: Виток с меньшим числом точек не публикуем: это мусор или чужой протокол.
LIDAR_MIN_POINTS = 20
#: Если полный виток не закрывается, публикуем накопленное по паузе.
LIDAR_PUBLISH_GAP_S = 0.35
#: Пауза перед новым кругом поиска порта.
LIDAR_RESCAN_S = 2.0

#: Скорости RPLIDAR по моделям (у A3 на USB-адаптере есть переключатель 115200/256000).
RPLIDAR_BAUDS = (115200, 256000, 460800)
#: LD06/LD19 — односторонний поток на 230400, команд не принимает.
LD06_BAUD = 230400
#: LDS-01/LDS-02 (TurtleBot3 Burger, он же HLS-LFCD2) — тоже 230400, кадры 0xFA.
LDS01_BAUD = 230400

#: Профили поиска в порядке частоты моделей: (протокол, скорость).
LIDAR_PROFILES: Tuple[Tuple[str, int], ...] = (
    ("rplidar", 115200),      # RPLIDAR A1 / A2M8
    ("ld06", LD06_BAUD),      # LD06 / LD19 (комплекты D500 и клоны)
    ("lds01", LDS01_BAUD),    # LDS-01 / LDS-02 (TurtleBot3 Burger, HLS-LFCD2)
    ("rplidar", 256000),      # RPLIDAR A2M7 / A2M12 / A3 / S1
    ("rplidar", 460800),      # RPLIDAR C1
)

#: Как в RC_LIDAR_PROTOCOL можно назвать протоколы (синонимы — в _PROTOCOL_ALIASES).
LIDAR_PROTOCOLS: Tuple[str, ...] = ("auto", "rplidar", "ld06", "lds01")
_PROTOCOL_ALIASES = {
    "lds": "lds01", "lds-01": "lds01", "lds_01": "lds01", "lds1": "lds01",
    "lds02": "lds01", "lds-02": "lds01", "lds2": "lds01",
    "hls": "lds01", "hls-lfcd2": "lds01", "hls_lfcd2": "lds01", "lfcd2": "lds01",
    "turtlebot": "lds01", "turtlebot3": "lds01",
}


#: Варианты включения мотора RPLIDAR через линии USB-переходника (MOTOCTL).
#: У адаптеров A1/A2 это DTR (низкий уровень — мотор включён), у части плат — RTS
#: и с обратной полярностью; последний вариант — линии не трогаем (у многих
#: клонов мотор питается постоянно). Перебираем, пока не пойдут точки.
RPLIDAR_MOTOR_VARIANTS: Tuple[Tuple[Optional[str], Optional[bool]], ...] = (
    ("dtr", False),
    ("rts", False),
    ("dtr", True),
    ("rts", True),
    (None, None),
)

#: Настройка RC_LIDAR_MOTOR → варианты линий (сколько вариантов — столько попыток).
MOTOR_MODES: dict[str, Tuple[Tuple[Optional[str], Optional[bool]], ...]] = {
    "auto": RPLIDAR_MOTOR_VARIANTS,
    "none": ((None, None),),
    "off": ((None, None),),
    "dtr-low": (("dtr", False),),
    "dtr-high": (("dtr", True),),
    "rts-low": (("rts", False),),
    "rts-high": (("rts", True),),
}

RPLIDAR_CMD_STOP = b"\xa5\x25"
RPLIDAR_CMD_SCAN = b"\xa5\x20"
RPLIDAR_SYNC = (0xA5, 0x5A)
RPLIDAR_ANSWER_SYNC = b"\xa5\x5a"
#: Стандартный дескриптор ответа RPLIDAR на SCAN: A5 5A 05 00 00 40 81.
RPLIDAR_ANSWER_SIZE = 0x05
RPLIDAR_ANSWER_TYPES = (0x81, 0x82)

#: Настоящий виток идёт монотонно (шаг ~1° у RPLIDAR и LDS-01, ~3° у LD06), а
#: «точки» из чужих байт прыгают по всему кругу: медиана шага угла между
#: соседними точками отделяет одно от другого с большим запасом.
SCAN_MAX_MEDIAN_STEP_RAD = math.radians(45.0)

#: Мало того, что виток идёт размеренно — он ещё и закрывает весь круг. Поток
#: чужого протокола (LDS-01 на скорости RPLIDAR, LD06 как RPLIDAR и т. п.) может
#: разбираться «в упор»: точки тогда выходят с крошечным шагом, но все в одном
#: направлении — медиана шага у такого мусора нулевая, и раньше он сходил за
#: настоящий скан. Настоящий виток оставляет разрыв не больше пары градусов,
#: так что запас в 90° огромный.
SCAN_MAX_GAP_RAD = math.radians(90.0)

LD06_SYNC = b"\x54\x2c"
LD06_PACKET_LEN = 47

#: LDS-01 / LDS-02 (TurtleBot3 Burger, HLS-LFCD2): кадр 42 байта, 60 пакетов по
#: 6 измерений — 360 точек на виток, скорость 230400.
LDS01_HEADER = 0xFA
LDS01_PACKET_LEN = 42
LDS01_INDEX_BASE = 0xA0
LDS01_PACKETS_PER_SCAN = 60
LDS01_READINGS_PER_PACKET = 6
LDS01_CMD_START = b"b"  # запуск вращения (нужен прошивкам до 2017.10)
LDS01_CMD_STOP = b"e"   # остановка вращения (мы её не шлём: пусть мотор крутится)
LDS01_INVALID_FLAG = 0x8000  # старший бит слова дальности: измерение недостоверно
LDS01_RANGE_MASK = 0x3FFF    # дальность занимает младшие 14 бит
LDS01_RANGE_MIN_M = 0.05
LDS01_RANGE_MAX_M = 16.0
#: Сколько пакетов с неверной КС ждём, прежде чем читать поток без проверки.
LDS01_CRC_BAD_LIMIT = 60
#: Сколько пакетов подряд должно прийти строго по порядку номеров, чтобы поверить
#: потоку без контрольной суммы (у настоящего лидара номера идут по кругу).
LDS01_WARMUP_PACKETS = 12

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on", "да"}


def _env_int(name: str) -> Optional[int]:
    raw = (os.environ.get(name) or "").strip()
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError:
        log.warning("Лидар: %s=%r — не число, игнорирую", name, raw)
        return None


def normalize_protocol(value: str) -> str:
    """Приводит RC_LIDAR_PROTOCOL к одному из ``LIDAR_PROTOCOLS``.

    Понимает синонимы: ``lds-01``, ``hls-lfcd2``, ``turtlebot3`` → ``lds01`` —
    чтобы лидар TurtleBot3 можно было назвать так, как он написан на корпусе.
    """
    proto = (value or "").strip().lower().replace(" ", "")
    proto = _PROTOCOL_ALIASES.get(proto, proto)
    return proto if proto in LIDAR_PROTOCOLS else "auto"


def normalize_angle(rad: float) -> float:
    """Нормализует угол в диапазон [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def scan_from_ranges(
    ranges: list[float],
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float = 0.05,
    range_max: float = 12.0,
) -> list[dict[str, float]]:
    """Преобразует массив дальностей в список точек [{a, r}].

    a — угол в радианах [-pi, pi], 0 — вперёд.
    r — дальность в метрах.
    """
    points: list[dict[str, float]] = []
    r_min = float(range_min)
    r_max = float(range_max)
    inc = float(angle_increment)
    a_start = float(angle_min)

    for i, r in enumerate(ranges):
        try:
            dist = float(r)
        except (ValueError, TypeError):
            continue
        if not math.isfinite(dist) or dist < r_min or dist > r_max:
            continue
        a = a_start + i * inc
        points.append({"a": round(normalize_angle(a), 4), "r": round(dist, 3)})
    return points


def parse_rplidar_point(b: bytes) -> Optional[dict[str, float]]:
    """Разбирает 5-байтовый пакет RPLIDAR (Standard Scan).

    b[0]: sync, quality
    b[1]: check bit, angle_q6[0:6]
    b[2]: angle_q6[7:14]
    b[3]: distance_q2[0:7]
    b[4]: distance_q2[8:15]
    """
    if len(b) < 5:
        return None
    # проверка sync/inv_sync и check_bit
    s = b[0] & 0x01
    inv_s = (b[0] >> 1) & 0x01
    if s == inv_s:  # должны быть инверсны
        return None
    if not (b[1] & 0x01):  # check bit должен быть 1
        return None

    angle_q6 = (b[1] >> 1) | (b[2] << 7)
    angle_deg = angle_q6 / 64.0
    dist_q2 = b[3] | (b[4] << 8)
    dist_m = (dist_q2 / 4.0) / 1000.0

    if dist_m < 0.05 or dist_m > 16.0:
        return None

    # В RPLIDAR угол по часовой стрелке, в RUS SLAM против часовой от оси X
    rad = normalize_angle(-math.radians(angle_deg))
    return {"a": round(rad, 4), "r": round(dist_m, 3)}


def lds01_checksum(data: bytes) -> int:
    """Контрольная сумма LDS-01 (HLS-LFCD2) по первым 40 байтам пакета.

    Алгоритм из драйвера ROBOTIS/HLDS: идём по данным 16-битными словами
    (младший байт первым), каждый раз сдвигая накопленное влево на бит, затем
    складываем младшие 15 бит со всем остальным — получается 15-битное число.
    """
    chk32 = 0
    limit = len(data) - (len(data) % 2)  # неполный хвост в расчёт не идёт
    for i in range(0, limit, 2):
        chk32 = (chk32 << 1) + (data[i] | (data[i + 1] << 8))
    return ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF


def lds01_packet_crc_ok(pkt: bytes) -> bool:
    """Совпадает ли контрольная сумма пакета LDS-01 с записанной в нём."""
    if len(pkt) < LDS01_PACKET_LEN:
        return False
    expected = pkt[40] | (pkt[41] << 8)
    return lds01_checksum(pkt[: LDS01_PACKET_LEN - 2]) == expected


def parse_lds01_packet(pkt: bytes) -> list[dict[str, float]]:
    """Разбирает 42-байтовый пакет LDS-01 / LDS-02 (TurtleBot3 Burger).

    Раскладка пакета:

    ============ ==========================================================
    байт 0       0xFA — заголовок
    байт 1       0xA0 + номер пакета (0…59), метр: 6° на пакет
    байты 2..3   скорость вращения, младший байт первым
    байты 4..39  шесть измерений по 6 байт: интенсивность (u16),
                 дальность в мм (u16), резерв (u16)
    байты 40..41 контрольная сумма (см. :func:`lds01_checksum`)
    ============ ==========================================================

    Угол точки считается как у драйвера ROBOTIS (``hls_lfcd_lds_driver``):
    359 − (номер пакета·6 + i) градусов против часовой стрелки от направления
    вперёд. Нумерацию и направление отсчёта не меняем — иначе карта повернулась
    бы относительно ROS-режима, где этот лидар работает штатно.

    В слове дальности старший бит означает недостоверное измерение (такие точки
    выбрасываются), а сама дальность занимает младшие 14 бит. Контрольная сумма
    здесь не проверяется (см. :func:`lds01_packet_crc_ok` и ``Lds01Stream``).
    """
    if len(pkt) < LDS01_PACKET_LEN or pkt[0] != LDS01_HEADER:
        return []
    index = pkt[1] - LDS01_INDEX_BASE
    if not 0 <= index < LDS01_PACKETS_PER_SCAN:
        return []

    pts: list[dict[str, float]] = []
    for i in range(LDS01_READINGS_PER_PACKET):
        offset = 4 + i * 6
        raw = pkt[offset + 2] | (pkt[offset + 3] << 8)
        if raw & LDS01_INVALID_FLAG:
            continue
        dist_m = (raw & LDS01_RANGE_MASK) / 1000.0
        if dist_m < LDS01_RANGE_MIN_M or dist_m > LDS01_RANGE_MAX_M:
            continue
        angle_deg = 359.0 - float(index * LDS01_READINGS_PER_PACKET + i)
        rad = normalize_angle(math.radians(angle_deg))
        pts.append({"a": round(rad, 4), "r": round(dist_m, 3)})
    return pts


def ld06_crc_ok(pkt: bytes) -> bool:
    """Контрольная сумма пакета LD06/LD19: XOR всех байт, кроме последнего."""
    if len(pkt) < LD06_PACKET_LEN:
        return False
    crc = 0
    for byte in pkt[: LD06_PACKET_LEN - 1]:
        crc ^= byte
    return crc == pkt[LD06_PACKET_LEN - 1]


def parse_ld06_packet(pkt: bytes) -> list[dict[str, float]]:
    """Разбирает 47-байтовый пакет LD06 / LD19.

    Заголовок: 0x54, 0x2C.
    12 точек измерений на пакет.
    Контрольная сумма здесь не проверяется (см. ``ld06_crc_ok``): у части
    клонов она не считается, а поток всё равно несёт корректные данные.
    """
    if len(pkt) < 47 or pkt[0] != 0x54 or pkt[1] != 0x2C:
        return []
    start_angle = (pkt[4] | (pkt[5] << 8)) * 0.01
    end_angle = (pkt[42] | (pkt[43] << 8)) * 0.01
    if end_angle < start_angle:
        end_angle += 360.0

    step = (end_angle - start_angle) / 11.0 if end_angle != start_angle else 0.0
    pts: list[dict[str, float]] = []

    for i in range(12):
        idx = 6 + i * 3
        dist_mm = pkt[idx] | (pkt[idx + 1] << 8)
        conf = pkt[idx + 2]
        if conf < 10 or dist_mm < 50 or dist_mm > 15000:
            continue
        angle_deg = (start_angle + i * step) % 360.0
        rad = normalize_angle(-math.radians(angle_deg))
        pts.append({"a": round(rad, 4), "r": round(dist_mm / 1000.0, 3)})
    return pts


def generate_demo_scan(t: Optional[float] = None, speed: float = 0.5) -> list[dict[str, float]]:
    """Генерирует скан помещения склада со стенами и столбиками."""
    now = time.time() if t is None else t
    pts: list[dict[str, float]] = []
    num_rays = 180

    for i in range(num_rays):
        a = (i / num_rays) * math.pi * 2.0 - math.pi
        # Базовый прямоугольный контур помещения ~5×4 м
        cos_a = math.cos(a)
        sin_a = math.sin(a)
        r_x = abs(3.5 / cos_a) if abs(cos_a) > 1e-4 else 12.0
        r_y = abs(2.5 / sin_a) if abs(sin_a) > 1e-4 else 12.0
        r = min(r_x, r_y)

        # Небольшая динамическая волна и неровность стен
        r += 0.08 * math.sin(a * 8.0 + now * 0.5) + 0.04 * math.cos(a * 15.0)

        # Столбик 1 (угол ~35°)
        if abs(a - 0.6) < 0.08:
            r = min(r, 1.8 + 0.05 * math.sin(now))
        # Столбик 2 (угол ~-50°)
        if abs(a + 0.9) < 0.07:
            r = min(r, 2.4 + 0.04 * math.cos(now * 1.2))
        # Стеллаж сбоку (угол ~110°)
        if 1.7 < a < 2.2:
            r = min(r, 2.1)

        # Движение робота влияет на скан
        r += 0.06 * math.sin(now * speed * 2.0 + a * 3.0)
        r = max(0.2, min(9.8, r))

        pts.append({"a": round(a, 4), "r": round(r, 3)})

    return pts


# ============================================================================
# Разбор потока данных лидара
# ============================================================================
class ScanAccumulator:
    """Собирает точки в виток и публикует его.

    Лидару нужно время на оборот (RPLIDAR A1 — 5…10 Гц, LD06/LD19 — 10…15 Гц),
    поэтому публикуем не каждую точку, а готовый виток; если виток не
    закрывается (клоны без достоверного бита начала), отдаём накопленное по
    паузе, чтобы на экране не было пустой карты.
    """

    def __init__(
        self,
        publish: Callable[[list[dict[str, float]]], None],
        *,
        min_points: int = LIDAR_MIN_POINTS,
        gap_s: float = LIDAR_PUBLISH_GAP_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._publish = publish
        self._min_points = max(1, int(min_points))
        self._gap_s = float(gap_s)
        self._clock = clock
        self._points: list[dict[str, float]] = []
        self._published_at = clock()
        self.published = 0

    @property
    def pending(self) -> int:
        return len(self._points)

    def add(self, points: Sequence[dict[str, float]], revolution_done: bool = False) -> bool:
        """Добавляет точки. True — опубликовали виток (или накопленное по паузе)."""
        published = False
        if revolution_done:
            published = self.flush()
        if points:
            self._points.extend(points)
        if len(self._points) >= self._min_points and self._clock() - self._published_at >= self._gap_s:
            published = self.flush() or published
        return published

    def flush(self) -> bool:
        """Публикует накопленное, если точек хватает. True — опубликовали."""
        points = self._points
        if len(points) < self._min_points:
            return False
        self._points = []
        self._published_at = self._clock()
        self.published += 1
        self._publish(points)
        return True


class RplidarStream:
    """Разбор стандартного протокола RPLIDAR: точки по 5 байт, бит начала витка."""

    def __init__(self, *, max_buffer: int = 4096) -> None:
        self.buffer = bytearray()
        self.max_buffer = max_buffer
        self.points = 0

    def feed(self, chunk: bytes) -> list[Tuple[dict[str, float], bool]]:
        """Добавляет байты потока → [(точка, начало_витка), ...]."""
        out: list[Tuple[dict[str, float], bool]] = []
        if chunk:
            self.buffer.extend(chunk)
            if len(self.buffer) > self.max_buffer:
                del self.buffer[: len(self.buffer) - 5]
        while len(self.buffer) >= 5:
            point = parse_rplidar_point(bytes(self.buffer[:5]))
            if point is None:
                del self.buffer[:1]  # синхронизация потеряна — ищем заново
                continue
            is_start = bool(self.buffer[0] & 0x01)
            del self.buffer[:5]
            self.points += 1
            out.append((point, is_start))
        return out


class Ld06Stream:
    """Разбор потока LD06/LD19: пакеты 0x54 0x2C по 12 точек."""

    def __init__(
        self,
        *,
        check_crc: bool = True,
        crc_bad_limit: int = 40,
        max_buffer: int = 8192,
    ) -> None:
        self.buffer = bytearray()
        self.check_crc = check_crc
        self.crc_bad_limit = crc_bad_limit
        self.crc_ok = 0
        self.crc_bad = 0
        self.packets = 0
        self.max_buffer = max_buffer
        self._prev_start: Optional[float] = None

    def feed(self, chunk: bytes) -> list[Tuple[list[dict[str, float]], bool]]:
        """Добавляет байты потока → [(точки пакета, виток_завершён), ...]."""
        out: list[Tuple[list[dict[str, float]], bool]] = []
        if chunk:
            self.buffer.extend(chunk)
            if len(self.buffer) > self.max_buffer:
                del self.buffer[: len(self.buffer) - LD06_PACKET_LEN]
        while True:
            packet = self._next_packet()
            if packet is None:
                return out
            pts = parse_ld06_packet(packet)
            if not pts:
                continue
            self.packets += 1
            start_angle = (packet[4] | (packet[5] << 8)) * 0.01
            revolution = self._prev_start is not None and start_angle + 1e-9 < self._prev_start
            self._prev_start = start_angle
            out.append((pts, revolution))

    def _next_packet(self) -> Optional[bytes]:
        while True:
            start = self.buffer.find(LD06_SYNC)
            if start < 0:
                # заголовок мог разрезаться между чтениями — держим последний байт
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return None
            if start:
                del self.buffer[:start]
            if len(self.buffer) < LD06_PACKET_LEN:
                return None
            packet = bytes(self.buffer[:LD06_PACKET_LEN])
            if self.check_crc and not ld06_crc_ok(packet):
                self.crc_bad += 1
                if self.crc_bad >= self.crc_bad_limit and self.crc_ok == 0:
                    # у части клонов CRC не считается — читаем поток без проверки
                    self.check_crc = False
                    log.info("Лидар: у LD06/LD19 не сходится CRC — читаю поток без проверки")
                del self.buffer[:1]
                continue
            del self.buffer[:LD06_PACKET_LEN]
            self.crc_ok += 1
            return packet


class Lds01Stream:
    """Разбор потока LDS-01 / LDS-02 (TurtleBot3 Burger): кадры 0xFA по 42 байта.

    Виток — 60 пакетов (360 точек); границей витка служит пакет с индексом 0xA0,
    как и в драйвере ROBOTIS. Контрольная сумма проверяется, но если у устройства
    она не сходится ни на одном пакете (встречаются прошивки с другой формулой),
    поток читается без неё — тогда единственной защитой остаётся строгий порядок
    номеров пакетов: у настоящего лидара он идёт по кругу 0xA0…0xDB без пропусков,
    и точкам верим лишь после серии из ``LDS01_WARMUP_PACKETS`` пакетов подряд.
    """

    def __init__(
        self,
        *,
        check_crc: bool = True,
        crc_bad_limit: int = LDS01_CRC_BAD_LIMIT,
        max_buffer: int = 8192,
    ) -> None:
        self.buffer = bytearray()
        self.check_crc = check_crc
        self.crc_bad_limit = crc_bad_limit
        self.crc_ok = 0            # пакетов принято с верной контрольной суммой
        self.crc_bad = 0           # пакетов забраковано по контрольной сумме
        self.packets = 0           # пакетов отдано
        self.max_buffer = max_buffer
        self._expected_index: Optional[int] = None
        self._run = 0             # сколько пакетов подряд идут по порядку номеров
        self._confirmed = False   # потоку без контрольной суммы уже можно верить

    def feed(self, chunk: bytes) -> list[Tuple[list[dict[str, float]], bool]]:
        """Добавляет байты потока → [(точки пакета, виток_завершён), ...]."""
        out: list[Tuple[list[dict[str, float]], bool]] = []
        if chunk:
            self.buffer.extend(chunk)
            if len(self.buffer) > self.max_buffer:
                del self.buffer[: len(self.buffer) - LDS01_PACKET_LEN]
        while True:
            packet = self._next_packet()
            if packet is None:
                return out
            index = packet[1] - LDS01_INDEX_BASE
            self.packets += 1
            # Пакет с индексом 0xA0 начинает новый виток: к этому моменту в
            # накопителе лежат точки предыдущего (359…0°), и он закрывается.
            out.append((parse_lds01_packet(packet), index == 0))

    def _next_packet(self) -> Optional[bytes]:
        """Ищет в буфере очередной пакет; None — данных пока не хватает."""
        while True:
            start = self._find_start()
            if start < 0:
                # заголовок мог разрезаться между чтениями — держим последний байт
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return None
            if start:
                del self.buffer[:start]
            if len(self.buffer) < LDS01_PACKET_LEN:
                return None
            packet = bytes(self.buffer[:LDS01_PACKET_LEN])

            if self.check_crc and not lds01_packet_crc_ok(packet):
                self.crc_bad += 1
                if self.crc_bad >= self.crc_bad_limit and self.crc_ok == 0:
                    self.check_crc = False
                    log.info(
                        "Лидар: у LDS-01/LDS-02 не сходится контрольная сумма — "
                        "читаю поток без проверки"
                    )
                del self.buffer[:1]  # контрольная сумма не та — ищем заголовок дальше
                continue

            index = packet[1] - LDS01_INDEX_BASE
            if not self.check_crc and not self._confirmed:
                # Без контрольной суммы верить пакету сразу нельзя: «пакеты» из
                # чужих байт тоже начинаются с 0xFA и допустимого номера. Зато
                # у настоящего лидара номера идут строго по кругу, поэтому поток
                # признаём своим только после серии пакетов подряд по порядку.
                self._run = self._run + 1 if index == self._expected_index else 1
                self._expected_index = (index + 1) % LDS01_PACKETS_PER_SCAN
                del self.buffer[:LDS01_PACKET_LEN]
                if self._run < LDS01_WARMUP_PACKETS:
                    continue
                self._confirmed = True
                log.info(
                    "Лидар: LDS-01/LDS-02 читается без контрольной суммы — "
                    "поток подтверждён по порядку номеров пакетов"
                )
                return packet

            del self.buffer[:LDS01_PACKET_LEN]
            self._expected_index = (index + 1) % LDS01_PACKETS_PER_SCAN
            if self.check_crc:
                self.crc_ok += 1
            return packet

    def _find_start(self) -> int:
        """Индекс первого байта кадра (0xFA + допустимый номер пакета) или −1."""
        last = LDS01_INDEX_BASE + LDS01_PACKETS_PER_SCAN - 1
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == LDS01_HEADER and LDS01_INDEX_BASE <= self.buffer[i + 1] <= last:
                return i
        return -1


def read_rplidar_stream(
    ser,
    publish: Callable[[list[dict[str, float]]], None],
    stop: Callable[[], bool],
    *,
    timeout: float = LIDAR_DATA_TIMEOUT_S,
    min_points: int = LIDAR_MIN_POINTS,
) -> int:
    """Читает точки RPLIDAR до тишины и публикует витки. Возвращает число точек."""
    stream = RplidarStream()
    acc = ScanAccumulator(publish, min_points=min_points)
    last_point = time.monotonic()
    while not stop():
        chunk = ser.read(64)
        for point, is_start in stream.feed(chunk):
            last_point = time.monotonic()
            acc.add([point], revolution_done=is_start)
        if not chunk and time.monotonic() - last_point > timeout:
            break
    acc.flush()  # хвост витка — чтобы последний скан не пропал
    return stream.points


def read_ld06_stream(
    ser,
    publish: Callable[[list[dict[str, float]]], None],
    stop: Callable[[], bool],
    *,
    timeout: float = LIDAR_DATA_TIMEOUT_S,
    min_points: int = LIDAR_MIN_POINTS,
    check_crc: bool = True,
) -> int:
    """Читает поток LD06/LD19 до тишины и публикует витки. Возвращает число точек."""
    stream = Ld06Stream(check_crc=check_crc)
    acc = ScanAccumulator(publish, min_points=min_points)
    last_point = time.monotonic()
    total = 0
    while not stop():
        chunk = ser.read(256)
        for points, revolution in stream.feed(chunk):
            last_point = time.monotonic()
            total += len(points)
            acc.add(points, revolution_done=revolution)
        if not chunk and time.monotonic() - last_point > timeout:
            break
    acc.flush()
    return total


def lds01_start_scan(ser) -> None:
    """Просит LDS-01 начать вращение (``b``).

    Нужно прошивкам до 2017.10, новым не мешает. Команду остановки (``e``) не
    отправляем: мотор должен крутиться и после закрытия порта — иначе при каждом
    переподключении лидар тратил бы секунды на раскрутку и перебор его не находил.
    """
    try:
        ser.write(LDS01_CMD_START)
    except Exception as exc:  # noqa: BLE001 — порт мог пропасть
        log.debug("Лидар: не удалось отправить команду запуска LDS-01: %s", exc)


def read_lds01_stream(
    ser,
    publish: Callable[[list[dict[str, float]]], None],
    stop: Callable[[], bool],
    *,
    timeout: float = LIDAR_DATA_TIMEOUT_S,
    min_points: int = LIDAR_MIN_POINTS,
    check_crc: bool = True,
) -> int:
    """Читает поток LDS-01/LDS-02 до тишины и публикует витки. Возвращает число точек."""
    stream = Lds01Stream(check_crc=check_crc)
    acc = ScanAccumulator(publish, min_points=min_points)
    last_point = time.monotonic()
    total = 0
    while not stop():
        chunk = ser.read(256)
        for points, revolution in stream.feed(chunk):
            last_point = time.monotonic()
            total += len(points)
            acc.add(points, revolution_done=revolution)
        if not chunk and time.monotonic() - last_point > timeout:
            break
    acc.flush()
    return total


def scan_looks_like_rotation(
    points: Sequence[dict[str, float]],
    max_median_step: float = SCAN_MAX_MEDIAN_STEP_RAD,
    max_gap: float = SCAN_MAX_GAP_RAD,
) -> bool:
    """Похож ли набор точек на настоящий виток лидара (луч идёт размеренно)?

    Настоящий виток RPLIDAR/LDS-01/LD06 идёт монотонно — шаг между соседними
    точками около градуса, и точки закрывают весь круг. Если скорость порта
    угадана неверно (например, поток LDS-01 читают как RPLIDAR на 115200) или
    выбран чужой протокол, «точки» получаются из случайных байт: они либо
    прыгают по всему кругу, либо, наоборот, разбираются «в упор» — встают в одно
    направление с нулевым шагом. Поэтому проверяем оба признака: медиана шага
    (мусор с прыжками) и наибольший разрыв между соседними углами (мусор,
    слипшийся в одну сторону). На переборе такой виток не считается удачей:
    иначе лидар «находился» бы, а карта на экране была из мусора.
    """
    if len(points) < 3:
        return False
    steps = sorted(
        abs(normalize_angle(cur["a"] - prev["a"]))
        for prev, cur in zip(points, points[1:])
    )
    if steps[len(steps) // 2] >= max_median_step:
        return False
    angles = sorted(normalize_angle(p["a"]) % math.tau for p in points)
    gaps = [cur - prev for prev, cur in zip(angles, angles[1:])]
    gaps.append(angles[0] + math.tau - angles[-1])
    return max(gaps) <= max_gap


# ============================================================================
# Подбор порта и параметров лидара
# ============================================================================
def _natural_key(path: str) -> Tuple[str, int]:
    """Сортировка /dev/ttyUSB2 < /dev/ttyUSB10 (обычный sorted даёт наоборот)."""
    digits = re.sub(r"\D", "", path)
    return (path.rstrip("0123456789"), int(digits) if digits else 0)


def excluded_ports() -> set[str]:
    """Порты, которые лидару трогать нельзя: мастер и модули шасси."""
    ports: set[str] = set()
    raw = os.environ.get("RC_LIDAR_EXCLUDE") or ""
    for name in ("RC_SLAM_PORT", "RC_SLAM_MASTER_PORT"):
        value = os.environ.get(name)
        if value:
            raw = raw + "," + value
    raw = raw + "," + (os.environ.get("RC_SLAM_PORTS") or "")
    for chunk in raw.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        ports.add(chunk)
        ports.add(os.path.realpath(chunk))
    return ports


def discover_ports(
    *,
    configured: Optional[str] = None,
    autoprobe: bool = True,
    excluded: Iterable[str] = (),
) -> list[str]:
    """Порты-кандидаты для лидара: сначала заданные вручную, потом symlink'и, потом tty.

    ``configured`` может быть списком через запятую и поддерживает glob
    (``RC_LIDAR_PORT=/dev/ttyUSB1`` или ``/dev/serial/by-id/*lidar*``).

    Порт из настройки — только **первый в очереди**, а не единственный:
    лидар переподключают, и после перезагрузки он оказывается на другом
    ``/dev/ttyUSBx``. Поэтому ``autoprobe=True`` (по умолчанию) добавляет в
    список все USB-порты машины, и один и тот же порт, попавший и по
    symlink'у, и по имени tty, добавляется один раз (сравнение по realpath) —
    иначе перебор тратил бы на него двойное время.
    """
    blocked = {p for p in excluded if p}
    blocked |= {os.path.realpath(p) for p in blocked}
    found: list[str] = []
    seen_real: set[str] = set()

    def add(path: str) -> None:
        if not path or not os.path.exists(path):
            return
        real = os.path.realpath(path)
        if path in blocked or real in blocked:
            return
        if path in found or real in seen_real:
            return
        found.append(path)
        seen_real.add(real)

    patterns: list[str] = []
    for chunk in (configured or "").replace(";", ",").split(","):
        chunk = chunk.strip()
        if chunk:
            patterns.append(chunk)
    for pattern in patterns:
        for path in sorted(glob.glob(pattern), key=_natural_key):
            add(path)

    # Именованные устройства: правило udev (см. README) или постоянные symlink'и.
    for pattern in (
        "/dev/rplidar*",
        "/dev/lidar*",
        "/dev/ldlidar*",
        "/dev/serial/by-id/*lidar*",
        "/dev/serial/by-id/*rplidar*",
        "/dev/serial/by-id/*lds*",
        "/dev/serial/by-id/*ld19*",
        "/dev/serial/by-id/*ld06*",
    ):
        for path in sorted(glob.glob(pattern), key=_natural_key):
            add(path)

    # Остальные USB-порты — по умолчанию (``autoprobe``): перебор обязан
    # пройти их все, даже если RC_LIDAR_PORT задан одним портом. Иначе после
    # переподключения лидара на соседний порт приложение навсегда оставалось
    # без скана («обращается только к одному USB»).
    if autoprobe:
        for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
            for path in sorted(glob.glob(pattern), key=_natural_key):
                add(path)

    return found


def lidar_profiles(protocol: str = "auto", baud: Optional[int] = None) -> list[Tuple[str, int]]:
    """Список (протокол, скорость) для подбора: от частых моделей к редким.

    На 230400 работают сразу два протокола — LD06/LD19 (кадры 0x54) и
    LDS-01/LDS-02 от TurtleBot3 (кадры 0xFA), поэтому при ``auto`` проверяются
    оба: сначала тот, у кого пакет защищён контрольной суммой.
    """
    proto = normalize_protocol(protocol)
    profiles = list(LIDAR_PROFILES)
    if proto != "auto":
        only_baud = {"ld06": LD06_BAUD, "lds01": LDS01_BAUD}.get(proto, RPLIDAR_BAUDS[0])
        profiles = [p for p in profiles if p[0] == proto] or [(proto, only_baud)]
    if baud:
        rate = int(baud)
        preferred = [p for p in profiles if p[1] == rate]
        rest = [p for p in profiles if p[1] != rate]
        names = (proto,) if proto != "auto" else ("rplidar", "ld06", "lds01")
        extra = [(name, rate) for name in names]
        profiles = preferred + [p for p in extra if p[1] == rate] + rest

    seen: set[Tuple[str, int]] = set()
    out: list[Tuple[str, int]] = []
    for item in profiles:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


def motor_variants(motor: str = "auto") -> Tuple[Tuple[Optional[str], Optional[bool]], ...]:
    """Варианты управления мотором RPLIDAR из настройки RC_LIDAR_MOTOR."""
    mode = (motor or "auto").strip().lower()
    if mode in MOTOR_MODES:
        return MOTOR_MODES[mode]
    log.warning("Лидар: неизвестное RC_LIDAR_MOTOR=%r — беру auto", motor)
    return RPLIDAR_MOTOR_VARIANTS


def motor_variant_name(variant: Tuple[Optional[str], Optional[bool]]) -> str:
    line, level = variant
    if line is None:
        return "none"
    return "%s-%s" % (line, "low" if level is False else "high")


def apply_motor_variant(ser, variant: Tuple[Optional[str], Optional[bool]]) -> None:
    """Выставляет линии USB-переходника так, чтобы мотор RPLIDAR крутился."""
    line, level = variant
    if line is None or level is None:
        return
    try:
        # DTR у части драйверов дёргает устройство при открытии порта — ставим явно.
        if line == "dtr":
            ser.dtr = bool(level)
        else:
            ser.rts = bool(level)
    except Exception as exc:  # noqa: BLE001 — драйвер может не уметь линии
        log.debug("Лидар: не удалось выставить %s=%s: %s", line, level, exc)


def _flush_input(ser) -> None:
    try:
        ser.reset_input_buffer()
    except Exception:  # noqa: BLE001 — у самодельных «портов» метода может не быть
        pass


def rplidar_handshake(ser, variant: Tuple[Optional[str], Optional[bool]], *,
                      timeout: float = 0.6, stop: Optional[Callable[[], bool]] = None) -> bool:
    """Включает мотор, посылает SCAN и ждёт стандартный дескриптор ответа.

    Ответ RPLIDAR на SCAN — ``A5 5A 05 00 00 40 81`` (длина 5, тип «измерение»).
    Ждём именно его: раньше за ответ принимались любые семь байт после
    синхрослова, и чужой поток на этой скорости (например, LDS-01, читаемый не
    на своей скорости) иногда «отвечал» сам — лидар находился там, где его нет,
    а на экран шёл мусор. Нестандартный дескриптор тоже принимаем, но только как
    запасной вариант, если стандартного за отведённое время не было: у клонов он
    бывает свой, а от мусорного скана дальше защищает проверка витка
    (:func:`scan_looks_like_rotation`).
    """
    stop = stop or (lambda: False)
    apply_motor_variant(ser, variant)
    time.sleep(0.05)  # мотору нужно тронуться, а лидару — сбросить буфер
    _flush_input(ser)
    try:
        ser.write(RPLIDAR_CMD_STOP)
        time.sleep(0.02)
        ser.write(RPLIDAR_CMD_SCAN)
    except Exception as exc:  # noqa: BLE001 — порт мог пропасть
        log.debug("Лидар: запись в порт не удалась: %s", exc)
        return False

    deadline = time.monotonic() + max(0.1, float(timeout))
    buf = bytearray()
    loose = False
    while time.monotonic() < deadline and not stop():
        chunk = ser.read(7)
        if not chunk:
            continue
        buf.extend(chunk)
        idx = buf.find(RPLIDAR_ANSWER_SYNC)
        if idx >= 0 and len(buf) - idx >= 7:
            header = bytes(buf[idx: idx + 7])
            if header[2] == RPLIDAR_ANSWER_SIZE and header[6] in RPLIDAR_ANSWER_TYPES:
                return True
            if not loose:
                loose = True
                log.debug("Лидар: нестандартный дескриптор ответа на SCAN: %s", header.hex(" "))
            del buf[idx]  # ищем стандартный ответ дальше, синхрослово отброшено
            continue
        if len(buf) > 64:
            del buf[:-1]
    if loose:
        log.warning("Лидар: ответ на SCAN нестандартный — сужу о лидаре по точкам, а не по дескриптору")
    return loose


def probe_port(
    ser,
    port: str,
    *,
    protocol: str = "auto",
    baud: Optional[int] = None,
    motor: str = "auto",
    timeout: float = LIDAR_PROBE_TIMEOUT_S,
    stop: Optional[Callable[[], bool]] = None,
    publish: Optional[Callable[[list[dict[str, float]]], None]] = None,
    on_found: Optional[Callable[[dict[str, Any]], None]] = None,
    min_points: int = LIDAR_MIN_POINTS,
    attempt_max: Optional[float] = None,
) -> Optional[dict[str, Any]]:
    """Подбирает параметры лидара на уже открытом порту.

    Возвращает {"port", "protocol", "baud", "motor", "points"} или None.
    ``publish(точки)`` вызывается на каждом готовом витке; первый вызов означает,
    что данные реально пошли (``on_found`` срабатывает на нём же). Пока данные не
    пошли, витков не публикуем — иначе «скан» из случайных байт чужого протокола
    попадал бы на экран.
    """
    stop = stop or (lambda: False)
    budget = LIDAR_ATTEMPT_MAX_S if attempt_max is None else float(attempt_max)
    state: dict[str, Any] = {"found": False, "probe": {"port": port}, "noise": 0}

    def on_scan(points: list[dict[str, float]]) -> None:
        if not state["found"]:
            # Первый виток — доказательство, что на порту настоящий лидар. Если
            # скорость или протокол угаданы неверно, точки получаются из чужих
            # байт и разбросаны по всему кругу: такой виток не считаем находкой
            # и не отдаём на экран (иначе «лидар» находился бы на любом мусоре).
            if not scan_looks_like_rotation(points):
                if not state["noise"]:
                    log.debug(
                        "Лидар: %s — виток не похож на настоящий скан (%d точек), "
                        "продолжаю перебор", port, len(points),
                    )
                state["noise"] += 1
                return
            state["found"] = True
            if on_found is not None:
                on_found(dict(state["probe"]))
        if publish is not None:
            publish(points)

    for proto, rate in lidar_profiles(protocol, baud):
        if stop() or state["found"]:
            break
        variants = motor_variants(motor) if proto == "rplidar" else ((None, None),)
        for variant in variants:
            if stop() or state["found"]:
                break
            try:
                ser.baudrate = rate
            except Exception as exc:  # noqa: BLE001
                log.debug("Лидар: %s: не переключить скорость %d: %s", port, rate, exc)
                return None
            _flush_input(ser)
            state["probe"].update(
                {
                    "protocol": proto,
                    "baud": rate,
                    "motor": motor_variant_name(variant),
                }
            )

            # Одна попытка не может длиться бесконечно: чужое устройство на порту
            # (телеметрия модуля, GPS, лидар другой модели) может сыпать данными
            # непрерывно, и перебор застревал бы на нём навсегда. Как только виток
            # найден, читаем поток дальше без ограничения — иначе на каждом круге
            # скана появлялась бы пауза на переоткрытие порта.
            attempt_deadline = time.monotonic() + budget

            def attempt_stop() -> bool:
                if stop():
                    return True
                return not state["found"] and time.monotonic() >= attempt_deadline

            if proto == "rplidar":
                if not rplidar_handshake(
                    ser, variant, timeout=min(0.6, max(0.2, timeout / 2)), stop=stop
                ):
                    break  # нет ответа на SCAN — на этой скорости устройства нет
                got = read_rplidar_stream(
                    ser, on_scan, attempt_stop, timeout=timeout, min_points=min_points
                )
            elif proto == "lds01":
                # LDS-01 сыплет кадры сам, но прошивки до 2017.10 не крутят мотор
                # без команды 'b' — она же полезна после чужой команды 'e'.
                lds01_start_scan(ser)
                got = read_lds01_stream(
                    ser, on_scan, attempt_stop, timeout=timeout, min_points=min_points
                )
            else:
                got = read_ld06_stream(
                    ser, on_scan, attempt_stop, timeout=timeout, min_points=min_points
                )

            if state["found"]:
                state["probe"]["points"] = got
                return state["probe"]
            if proto == "rplidar" and got:
                log.info(
                    "Лидар: %s ответил на SCAN (%d бод), но точек нет "
                    "(мотор: %s) — пробую другую линию мотора",
                    port, rate, motor_variant_name(variant),
                )
    return None


def open_serial(serial_module, port: str, baud: int):
    """Открывает порт лидара с нашим таймаутом (и эксклюзивно, если умеет pyserial)."""
    try:
        return serial_module.Serial(port, baud, timeout=SERIAL_READ_TIMEOUT_S, exclusive=True)
    except TypeError:  # pyserial < 3.3 — параметра exclusive ещё нет
        return serial_module.Serial(port, baud, timeout=SERIAL_READ_TIMEOUT_S)


# ============================================================================
# Менеджер лидара
# ============================================================================
class LidarManager:
    """Менеджер лидара: собирает данные из ROS 2, USB/serial или HTTP API."""

    _instance: Optional["LidarManager"] = None

    def __init__(self, port: Optional[str] = None):
        self.port = port or os.environ.get("RC_LIDAR_PORT") or None
        self.baud = _env_int("RC_LIDAR_BAUD")
        raw_protocol = (os.environ.get("RC_LIDAR_PROTOCOL") or "auto").strip().lower()
        self.protocol = normalize_protocol(raw_protocol)
        if raw_protocol and self.protocol == "auto" and raw_protocol != "auto":
            log.warning(
                "Лидар: неизвестный RC_LIDAR_PROTOCOL=%r — беру auto (%s)",
                raw_protocol, " | ".join(LIDAR_PROTOCOLS),
            )
        raw_motor = (os.environ.get("RC_LIDAR_MOTOR") or "auto").strip().lower()
        self.motor = raw_motor if raw_motor in MOTOR_MODES else "auto"
        self.autoprobe = _env_bool("RC_LIDAR_AUTOPROBE", True)
        self.excluded = excluded_ports()
        self.peer_urls = peer_urls()

        self.lock = threading.Lock()
        self.scan: list[dict[str, float]] = []
        self.scan_at: float = 0.0
        self.source_type: str = "none"
        self.reason: str = "лидар: идёт поиск порта"
        self.hint: str = ""
        self.detected: Optional[dict[str, Any]] = None
        self.last_error: str = ""
        self.candidates: list[str] = []

        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self._ros_started = False
        self._lock_fd: Optional[int] = None
        self._peer_misses = 0
        self._peer_reason = ""
        # С какого порта начинать очередной круг перебора и что нашли в
        # прошлом круге (видно в GET /api/lidar → tried).
        self._rotate = 0
        self._fallback_all = False
        self.tried: list[dict[str, Any]] = []
        self._logged_ports: tuple[str, ...] = ()
        self._logged_reason = ""
        self._logged_hint = ""

    # --------------------------- жизненный цикл --------------------------- #

    @classmethod
    def get_instance(cls, port: Optional[str] = None) -> "LidarManager":
        if cls._instance is None:
            cls._instance = LidarManager(port)
            cls._instance.start()
        return cls._instance

    def start(self) -> None:
        """Запускает фоновый опрос serial-лидара и ROS 2 (если доступно)."""
        if self._thread and self._thread.is_alive():
            return
        self._stop = False
        self._try_start_ros()
        self._thread = threading.Thread(target=self._serial_worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        self._release_port_lock()

    # ----------------------------- приём данных ---------------------------- #

    def feed_points(self, points: list[dict[str, float]], source_label: str = "api") -> int:
        """Принимает готовый список точек [{'a': ..., 'r': ...}]."""
        valid: list[dict[str, float]] = []
        for p in points:
            if not isinstance(p, dict):
                continue
            a = p.get("a")
            r = p.get("r")
            if a is None or r is None:
                continue
            try:
                fa = float(a)
                fr = float(r)
                if math.isfinite(fa) and math.isfinite(fr) and fr > 0.0:
                    valid.append({"a": round(normalize_angle(fa), 4), "r": round(fr, 3)})
            except (ValueError, TypeError):
                continue

        with self.lock:
            self.scan = valid
            self.scan_at = time.time()
            self.source_type = source_label
        return len(valid)

    def feed_scan_msg(
        self,
        ranges: list[float],
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float = 0.05,
        range_max: float = 12.0,
        source_label: str = "ros",
    ) -> int:
        """Принимает параметры LaserScan и сохраняет точки."""
        pts = scan_from_ranges(ranges, angle_min, angle_max, angle_increment, range_min, range_max)
        with self.lock:
            self.scan = pts
            self.scan_at = time.time()
            self.source_type = source_label
        return len(pts)

    def get_scan(
        self, max_age: float = LIDAR_MAX_AGE_S, allow_demo: bool = False
    ) -> Optional[list[dict[str, float]]]:
        """Возвращает свежий скан или None (если устарел)."""
        now = time.time()
        with self.lock:
            if self.scan and (now - self.scan_at) < max_age:
                return list(self.scan)
        if allow_demo:
            return generate_demo_scan()
        return None

    def get_status(self) -> dict[str, Any]:
        """Статус работы лидара для API /api/lidar и подписи на экране."""
        now = time.time()
        with self.lock:
            scan = list(self.scan)
            scan_at = self.scan_at
            src = self.source_type

        is_fresh = bool(scan and (now - scan_at) < LIDAR_MAX_AGE_S)
        near = min((p["r"] for p in scan), default=None) if scan else None

        if is_fresh:
            reason = "скан идёт: %s" % src
        elif scan:
            reason = "скан устарел (%.1f с без точек)" % (now - scan_at)
        else:
            reason = self.reason

        return {
            "ok": True,
            "connected": is_fresh,
            "source": src if is_fresh else "no_signal",
            "points_count": len(scan) if is_fresh else 0,
            "scan_at": scan_at,
            "age_s": round(now - scan_at, 2) if scan_at > 0 else None,
            "nearest_m": near if is_fresh else None,
            "configured_port": self.port,
            "reason": reason,
            "hint": self.hint,
            "detected": dict(self.detected) if self.detected else None,
            "candidates": list(self.candidates),
            # Результат последнего круга перебора: видно, что проверены все
            # порты, а не один (``ok`` — на нём пошли данные).
            "tried": [dict(item) for item in self.tried],
            "last_error": self.last_error or None,
        }

    # --------------------------- поиск и чтение --------------------------- #

    def candidate_ports(self, *, wide: bool = True) -> list[str]:
        """Порты для очередного круга перебора, начиная со следующего за прошлым.

        Перебор идёт по кругу: если лидар стоял на одном порту, а потом его
        переподключили (или порт заняла другая программа), круг начинается не с
        него, а со следующего кандидата — «залипания» на одном USB нет.
        ``wide=False`` (``RC_LIDAR_AUTOPROBE=0``) — узкий список: только порт из
        ``RC_LIDAR_PORT`` и именованные symlink'и; полный перебор в этом случае
        делает :meth:`_serial_worker` на следующем круге.
        """
        ports = discover_ports(
            configured=self.port, autoprobe=wide, excluded=self.excluded
        )
        if ports:
            step = self._rotate % len(ports)
            if step:
                ports = ports[step:] + ports[:step]
            self._rotate = step + 1
        self.candidates = ports
        return ports

    def _excluded_note(self) -> str:
        """Подсказка про порты, которые лидару запрещено трогать (мастер/шасси)."""
        if not self.excluded:
            return ""
        ports = sorted(p for p in self.excluded if p)
        return ("исключены порты мастера/модулей шасси: %s — если лидар подключён "
                "туда, убери порт из RC_SLAM_PORT/RC_SLAM_PORTS или из RC_LIDAR_EXCLUDE"
                % ", ".join(ports))

    def _set_status(self, reason: str, hint: str = "", *, warn: bool = False,
                    quiet: bool = False) -> None:
        """Меняет причину «нет скана»; в журнал пишет только при изменении.

        ``quiet`` — для частых сообщений о ходе перебора портов: на экране они
        видны (это и есть ответ «что происходит»), а журнал не засоряется.
        Итог круга перебора в журнал повторно не пишется, даже если между
        кругами статус успел пробежать по портам (``quiet``-сообщения).
        """
        if reason == self.reason and hint == self.hint:
            return
        self.reason = reason
        self.hint = hint
        if quiet:
            return
        if reason == self._logged_reason and hint == self._logged_hint:
            return
        self._logged_reason = reason
        self._logged_hint = hint
        text = reason + ((" (" + hint + ")") if hint else "")
        if warn:
            log.warning("Лидар: %s", text)
        else:
            log.info("Лидар: %s", text)

    def _store_scan(self, points: list[dict[str, float]], source_label: str) -> None:
        with self.lock:
            self.scan = list(points)
            self.scan_at = time.time()
            self.source_type = source_label

    def _ros_owns_lidar(self) -> bool:
        """ROS 2 уже отдаёт /scan — не отбираем у него USB-порт."""
        if self.port or self.baud or self.protocol != "auto":
            return False
        return self._ros_started

    def _serial_worker(self) -> None:
        """Поток поиска порта лидара и чтения скана."""
        try:
            import serial  # pyserial
        except ImportError:
            self._set_status(
                "нет библиотеки pyserial — USB-лидар не читается",
                hint="поставь пакет: .venv/bin/pip install pyserial (или ./scripts/ensure_deps.sh)",
                warn=True,
            )
            return

        while not self._stop:
            if self._ros_owns_lidar():
                self._set_status("скан идёт из ROS 2 (/scan) — USB-порт не трогаю")
                time.sleep(LIDAR_RESCAN_S)
                continue

            wide = self.autoprobe or self._fallback_all
            ports = self.candidate_ports(wide=wide)
            if not ports:
                hint = ("подключи лидар или задай RC_LIDAR_PORT (список портов: "
                        "ls /dev/ttyUSB* /dev/serial/by-id/; проверка: scripts/lidar_check.py)")
                note = self._excluded_note()
                if note:
                    hint = hint + "; " + note
                self._set_status("порт лидара не найден", hint=hint)
                time.sleep(LIDAR_RESCAN_S)
                continue

            # Перебор всех портов: у лидара нет постоянного имени, поэтому
            # каждая попытка явно попадает и в статус, и в журнал — иначе
            # кажется, что приложение «обращается только к одному USB».
            # Список портов печатаем, только когда он изменился: ротация
            # каждые 2 с одного и того же набора портов — это не новость.
            if tuple(sorted(ports)) != self._logged_ports:
                self._logged_ports = tuple(sorted(ports))
                log.info(
                    "Лидар: перебираю %d порт(ов)%s: %s",
                    len(ports),
                    "" if wide else " (узкий список: RC_LIDAR_AUTOPROBE=0)",
                    ", ".join(ports),
                )
            self.tried = []
            progressed = False
            for index, port in enumerate(ports, start=1):
                if self._stop:
                    return
                self._set_status(
                    "перебираю порты: %d из %d — %s" % (index, len(ports), os.path.basename(port)),
                    quiet=True,
                )
                if not self._acquire_port_lock(port):
                    # Порт читает второй процесс робота: пульт (8080) и бортовой
                    # экран (8090) поднимают этот менеджер одновременно, а порт
                    # открывается только один раз.
                    if self._peer_cycle():
                        progressed = True
                        self._fallback_all = False
                        self.tried.append({"port": port, "ok": True,
                                           "note": "скан берётся у второго процесса робота"})
                        break
                    peer_reason = ("; сосед сообщает: " + self._peer_reason) if self._peer_reason else ""
                    self._set_status(
                        "порт %s занят другой программой" % port,
                        hint="его читает второй процесс пульта, rplidar_ros или Serial Monitor; "
                             "останови лишнее или задай RC_LIDAR_PORT" + peer_reason,
                        quiet=True,
                    )
                    self.tried.append({"port": port, "ok": False, "note": "занят"})
                    continue
                try:
                    ok = self._read_port(serial, port)
                finally:
                    self._release_port_lock()
                self.tried.append({
                    "port": port,
                    "ok": ok,
                    "note": "данные пошли" if ok else "нет данных",
                })
                if ok:
                    progressed = True
                    self._fallback_all = False
                    break

            if not progressed:
                # Ни один порт не ответил — говорим об этом прямо:
                # видно, что перебор был по всем портам, а не по одному.
                if self.tried and all(item["note"] == "занят" for item in self.tried):
                    # Все кандидаты кем-то открыты: советы про кабель и скорость
                    # здесь бесполезны, причина — чужая программа на порту.
                    reason = ("лидар не найден: все %d порт(ов) заняты другой программой"
                              % len(self.tried))
                    hint = ("порты читает второй процесс пульта, rplidar_ros или Serial Monitor; "
                            "останови лишнее или задай RC_LIDAR_PORT")
                else:
                    # Порты в сообщении — по имени, а не в порядке обхода:
                    # иначе из-за ротации причина менялась бы каждый круг и
                    # один и тот же итог сыпался бы в журнал каждые 2 с.
                    items = sorted(self.tried, key=lambda item: os.path.basename(item["port"]))
                    checked = ", ".join(
                        "%s — %s" % (os.path.basename(item["port"]), item["note"])
                        for item in items[:5]
                    )
                    if len(items) > 5:
                        checked += " …"
                    reason = "лидар не ответил ни на одном из %d портов (%s)" % (
                        len(items), checked)
                    hint = ("проверь питание и кабель лидара; скорость зависит от модели: "
                            "115200 (A1/A2M8), 256000 (A2M7/A2M12/A3/S1), 460800 (C1), "
                            "230400 (LD06/LD19 и LDS-01/LDS-02 от TurtleBot3); "
                            "полный разбор — .venv/bin/python scripts/lidar_check.py")
                note = self._excluded_note()
                if note:
                    hint = hint + "; " + note
                self._set_status(reason, hint=hint, warn=True)
                if not self.autoprobe and not self._fallback_all:
                    # RC_LIDAR_AUTOPROBE=0 — узкий список. Он только первым, а
                    # не единственным: со следующего круга перебираем все USB.
                    self._fallback_all = True
                    log.info("Лидар: на закреплённых портах данных нет — "
                             "следующий круг перебирает все USB-порты")
                time.sleep(LIDAR_RESCAN_S)

    def _read_port(self, serial, port: str) -> bool:
        """Подбирает параметры и читает скан. True — данные шли (до обрыва)."""
        self._set_status("подбираю параметры порта %s…" % port, quiet=True)
        try:
            ser = open_serial(serial, port, RPLIDAR_BAUDS[0])
        except Exception as exc:  # noqa: BLE001 — порт занят/пропал/нет прав
            self.last_error = "%s: %s" % (port, exc)
            hint = ""
            if isinstance(exc, PermissionError) or "permission" in str(exc).lower():
                hint = "добавь пользователя в группу dialout: sudo usermod -aG dialout $USER (и перезайди)"
            elif getattr(serial, "SerialException", None) and isinstance(exc, serial.SerialException):
                hint = "порт занят другой программой (второй пульт, rplidar_ros, Serial Monitor)"
            self._set_status("порт %s не открылся" % port, hint=hint, quiet=True)
            return False

        try:
            found = probe_port(
                ser,
                port,
                protocol=self.protocol,
                baud=self.baud,
                motor=self.motor,
                timeout=LIDAR_PROBE_TIMEOUT_S,
                stop=lambda: self._stop,
                publish=lambda pts: self._store_scan(pts, "serial:%s" % os.path.basename(port)),
                on_found=self._mark_found,
            )
        except Exception as exc:  # noqa: BLE001 — кабель выдернули на ходу
            self.last_error = "%s: %s" % (port, exc)
            self._set_status("обрыв связи с портом %s" % port, hint=str(exc), quiet=True)
            return False
        finally:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

        if not found:
            self.detected = None
            self._set_status(
                "лидар на %s не отвечает" % port,
                hint="проверь скорость (A3/S1 — 256000, C1 — 460800, LD06/LD19 и "
                     "LDS-01/LDS-02 — 230400) и переключатель на USB-адаптере; "
                     "полный прогон: scripts/lidar_check.py",
                quiet=True,
            )
            return False

        self.last_error = ""
        if self.detected is None:  # данные шли, но поток уже оборвался
            self._mark_found(found)
        return True

    def _mark_found(self, probe: dict[str, Any]) -> None:
        """Запоминает рабочие параметры порта (и пишет об этом в журнал один раз)."""
        port = str(probe.get("port") or "")
        self.detected = {
            "port": port,
            "protocol": probe.get("protocol"),
            "baud": probe.get("baud"),
            "motor": probe.get("motor"),
        }
        self.last_error = ""
        self._set_status(
            "лидар на связи: %s, %s бод, мотор %s"
            % (os.path.basename(port), probe.get("baud"), probe.get("motor"))
        )
        log.info(
            "Лидар: найден на %s (%s %s бод, мотор: %s)",
            port, probe.get("protocol"), probe.get("baud"), probe.get("motor"),
        )

    # ------------------ межпроцессный замок и обмен сканом ------------------ #

    def _lock_path(self, port: str) -> Path:
        name = re.sub(r"[^A-Za-z0-9_.-]", "_", port.strip("/").replace("/", "-")) or "port"
        return DATA_DIR / ("lidar-%s.lock" % name)

    def _acquire_port_lock(self, port: str) -> bool:
        """Пытается занять порт. False — его уже читает другой процесс робота."""
        if fcntl is None or not port:  # не-POSIX: замки недоступны, читаем как читали
            return True
        try:
            path = self._lock_path(port)
            path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(str(path), os.O_RDWR | os.O_CREAT, 0o644)
        except OSError as exc:  # нет каталога data/ и т.п. — не мешаем чтению
            log.debug("Лидар: замок порта %s недоступен: %s", port, exc)
            return True
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            return False
        self._release_port_lock()
        self._lock_fd = fd
        return True

    def _release_port_lock(self) -> None:
        fd, self._lock_fd = self._lock_fd, None
        if fd is None:
            return
        try:
            if fcntl is not None:
                fcntl.flock(fd, fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            os.close(fd)
        except OSError:
            pass

    def _peer_cycle(self) -> bool:
        """Берёт готовый скан у второго процесса робота (у него занят порт)."""
        for url in self.peer_urls:
            data = _fetch_json(url)
            if not isinstance(data, dict):
                continue
            scan = data.get("scan")
            if isinstance(scan, list) and scan:
                count = self.feed_points(scan, source_label="peer")
                detected = data.get("detected")
                if isinstance(detected, dict):
                    self.detected = {
                        "port": detected.get("port"),
                        "protocol": detected.get("protocol"),
                        "baud": detected.get("baud"),
                        "motor": detected.get("motor"),
                    }
                self._peer_reason = ""
                self._set_status(
                    "скан берётся у второго процесса робота (%d точек)" % count,
                    hint="этот процесс сам порт лидара не открывает",
                )
                time.sleep(0.25)  # сосед обновляет скан со скоростью лидара (10 Гц)
                return True
            reason = str(data.get("reason") or "")
            if reason:
                self._peer_reason = reason
        self._peer_misses += 1
        time.sleep(0.4)
        return False

    # ----------------------- ROS 2 интеграция ----------------------- #

    def _try_start_ros(self) -> None:
        if self._ros_started:
            return
        try:
            import rclpy
            from sensor_msgs.msg import LaserScan
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
        except ImportError:
            return

        manager = self

        class LidarBridgeNode(Node):
            def __init__(self):
                super().__init__("lidar_bridge_node")
                self.sub = self.create_subscription(
                    LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
                )

            def _on_scan(self, msg):
                try:
                    manager.feed_scan_msg(
                        list(msg.ranges),
                        float(msg.angle_min),
                        float(msg.angle_max),
                        float(msg.angle_increment),
                        float(msg.range_min),
                        float(msg.range_max),
                        source_label="ros",
                    )
                except Exception as exc:  # noqa: BLE001
                    log.debug("ошибка парсинга ROS scan: %s", exc)

        def spin():
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
                node = LidarBridgeNode()
                manager._ros_started = True  # noqa: SLF001
                manager._set_status("скан идёт из ROS 2 (/scan)")  # noqa: SLF001
                log.info("Лидар: подключён топик ROS 2 /scan")
                rclpy.spin(node)
            except Exception as exc:  # noqa: BLE001
                log.debug("ROS лидар spin завершён: %s", exc)

        threading.Thread(target=spin, daemon=True).start()


def peer_urls() -> list[str]:
    """Адреса второго процесса робота, у которого можно взять готовый скан."""
    raw = os.environ.get("RC_LIDAR_PEER_URLS")
    if raw is not None:
        return [u.strip() for u in raw.split(",") if u.strip()]
    return ["http://127.0.0.1:8090/api/lidar", "http://127.0.0.1:8080/api/lidar"]


def _fetch_json(url: str, timeout: float = 1.0) -> Optional[dict]:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:  # noqa: S310
            if response.status != 200:
                return None
            return json.loads(response.read().decode("utf-8", "replace"))
    except (urllib.error.URLError, OSError, ValueError, json.JSONDecodeError):
        return None
    except Exception:  # noqa: BLE001 — соседний процесс может ответить чем угодно
        return None
