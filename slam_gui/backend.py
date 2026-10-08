#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
backend.py — сервер борта RUS SLAM: отдаёт экраны и данные любому браузеру.

Ничего не открывает и не показывает сам: робот лишь отвечает по сети, а
страницы смотрят в браузере — на дисплее робота, планшете или ноутбуке.
Один процесс отдаёт оба экрана и данные к ним:

    /                     → основной экран робота (main.html, киоск)
    /index.html, /console → инженерный пульт (index.html)
    /api/state            → состояние: двигатели, АКБ, груз, замок, связь, ИИ
    /api/map              → клетки карты цеха (строкой, по смене версии)
    /api/lock/open        → открыть грузовой отсек по PIN-коду
    /api/lock/close       → закрыть отсек
    /api/lock/pin         → сменить PIN-код
    /api/audit            → журнал доступа (аудит с хеш-цепочкой)
    /api/health           → проверка живости

Источники данных (выбираются автоматически или флагом):
    --source sim      демонстрационная физика (стенд без робота) — по умолчанию;
    --source serial   реальные модули: кадры телеметрии 16 Б по /dev/ttyUSB*
                      (протокол: docs/SERIAL_PROTOCOL.md, CRC16/Modbus);
    --source ros      ROS 2 (rclpy): /modules/state, /battery, /cargo/lock.

Замок: PIN-код хранится как SHA-256(salt + pin) в gui/state/lock.json,
5 неудачных попыток → блокировка 30 с, каждое событие — в журнал аудита.
Реальный замок вызывается внешней командой:

    RUS_SLAM_LOCK_CMD='gpio write 7 1'   (open)   /  RUS_SLAM_LOCK_CMD_CLOSE  (close)

Примеры запуска:
    python3 gui/backend.py                          # только сервер (штатный режим)
    python3 gui/backend.py --source serial --ports /dev/ttyUSB0,/dev/ttyUSB1,/dev/ttyUSB2,/dev/ttyUSB3
    python3 gui/backend.py --source ros --pin 2580
    python3 gui/backend.py --kiosk                   # опция: дисплей на борту, браузер без рамок
    python3 gui/backend.py --open                    # опция: открыть страницу в браузере робота

Кто где смотрит:
    на роботе     http://127.0.0.1:8080/            основной экран
    планшет/ПК    http://<ip-робота>:8080/          основной экран (оператор, получатель)
    инженер       http://<ip-робота>:8080/console   инженерный пульт, удалённо
"""

import argparse
import hashlib
import json
import math
import os
import re
import secrets
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.abspath(__file__))

# ИИ-водитель лежит рядом (gui/ai_driver.py). Пакетом gui не является — это
# набор файлов, поэтому кладём каталог в путь импорта, а не тянем пакет.
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
import ai_driver                                                   # noqa: E402
STATE_DIR = os.path.join(ROOT, "state")
LOCK_FILE = os.path.join(STATE_DIR, "lock.json")
MAIN_PAGE = "/main.html"
CONSOLE_PAGE = "/index.html"

DEFAULT_PIN = "2580"
MAX_ATTEMPTS = 5
LOCK_MS = 30_000

# --- Протокол (docs/SERIAL_PROTOCOL.md) -------------------------------------
CMD_SYNC = (0xAA, 0x55)
TLM_SYNC = (0xBB, 0x44)
CMD_LEN, TLM_LEN = 10, 16
STATUS_ENABLED, STATUS_HOMED, STATUS_FAULT = 0x01, 0x02, 0x04
MODULE_NAMES = {1: "FL", 2: "FR", 3: "RL", 4: "RR"}
MODULE_TITLES = {
    "FL": "передний левый", "FR": "передний правый",
    "RL": "задний левый", "RR": "задний правый",
}
WHEEL_R_M = 0.127            # радиус колеса Xiaomi M365 Pro, м

#: Скорость рулевого модуля, °/с — колесо не перескакивает мгновенно.
DEMO_STEER_RATE = 110.0

#: Задания стенда: куда ехать. Робот едет к точке, а как встать колёсами —
#: решает ИИ (gui/ai_driver.py): у машины четыре рулевых модуля, поэтому она
#: ездит боком и разворачивается на месте, как настоящая 4WIS.
DEMO_GOALS = (
    {"label": "проход у стеллажа", "x": 3.80, "y": 0.45},
    {"label": "крабом вбок", "x": 3.80, "y": 1.05, "crab": True},
    {"label": "разворот на месте", "turn": 90.0},     # курс: на север, как в цехе
    {"label": "коридор вдоль стеллажа", "x": 3.80, "y": 2.55},
    {"label": "площадка Б", "x": 4.05, "y": 2.60},
    {"label": "спуск к столу", "x": 3.80, "y": 1.60},
    {"label": "низ цеха", "x": 3.80, "y": 0.45},
    {"label": "площадка А", "x": 0.55, "y": 0.60},
    {"label": "база / зарядка", "x": 2.40, "y": 0.40},
)

# --- Карта цеха: мир, сканирование, упаковка в строку -----------------------
WORLD_W, WORLD_H = 4.8, 3.2          # м — размер цеха на стенде
MAP_W, MAP_H = 96, 64                # клеток
MAP_RES = WORLD_W / MAP_W            # 0,05 м на клетку
UNKNOWN, FREE, OCCUPIED = 0, 1, 2

#: Стены и стеллажи: (x0, y0, x1, y1) в метрах. По ним же считаются препятствия
#: для дальномера — мир и карта не расходятся.
WORLD_BLOCKS = (
    (0.00, 0.00, 4.80, 0.06),        # стены цеха
    (0.00, 3.14, 4.80, 3.20),
    (0.00, 0.00, 0.06, 3.20),
    (4.74, 0.00, 4.80, 3.20),
    (1.20, 1.00, 1.36, 2.30),        # стеллажи
    (1.95, 1.00, 2.11, 2.30),
    (3.30, 0.60, 3.46, 1.90),
    (2.05, 1.85, 2.90, 2.00),        # стол посреди цеха
)

#: Площадки на карте: (x, y, подпись).
WORLD_PADS = ((0.55, 0.60, "А"), (4.05, 2.60, "Б"), (2.40, 0.40, "база"))

#: Клеток карты для честного процента: за вычетом стен и стеллажей.
MAP_AREA = MAP_W * MAP_H


class RoomMap:
    """Карта цеха: клетки размечает дальномер, пока робот едет.

    Значения клеток: 0 — не разведано, 1 — свободно, 2 — препятствие. Карта
    копится с версией: экран забирает клетки только когда версия изменилась,
    поэтому опрос состояния остаётся лёгким.
    """

    def __init__(self):
        self.cells = bytearray(MAP_AREA)
        self.version = 1
        self.scanned = 0

    # --- мир -------------------------------------------------------------
    @staticmethod
    def blocked(x: float, y: float) -> bool:
        for x0, y0, x1, y1 in WORLD_BLOCKS:
            if x0 <= x <= x1 and y0 <= y <= y1:
                return True
        return False

    @staticmethod
    def cast(x: float, y: float, angle: float, max_range: float):
        """Луч дальномера: (до препятствия, м; точка попадания)."""
        step = MAP_RES * 0.8
        dist = 0.0
        while dist < max_range:
            dist += step
            px = x + math.cos(angle) * dist
            py = y + math.sin(angle) * dist
            if not (0.0 <= px <= WORLD_W and 0.0 <= py <= WORLD_H):
                return dist, (px, py)
            if RoomMap.blocked(px, py):
                return dist, (px, py)
        return max_range, (x + math.cos(angle) * max_range, y + math.sin(angle) * max_range)

    # --- разметка --------------------------------------------------------
    def _mark(self, cx: int, cy: int, value: int) -> bool:
        if not (0 <= cx < MAP_W and 0 <= cy < MAP_H):
            return False
        idx = cy * MAP_W + cx
        if self.cells[idx] == value:
            return False
        if self.cells[idx] == UNKNOWN and value != UNKNOWN:
            self.scanned += 1
        self.cells[idx] = value
        return True

    def scan(self, x: float, y: float, heading: float, rays: int = 72, max_range: float = 3.0) -> bool:
        """Круговой обзор: свободное по пути луча, препятствие на попадании."""
        changed = False
        for i in range(rays):
            angle = heading + (2 * math.pi * i / rays)
            dist, (hx, hy) = self.cast(x, y, angle, max_range)
            for step in range(1, int(dist / MAP_RES) + 1):
                px = x + math.cos(angle) * step * MAP_RES
                py = y + math.sin(angle) * step * MAP_RES
                if RoomMap.blocked(px, py):
                    break
                cx, cy = int(px / MAP_RES), int(py / MAP_RES)
                if abs(px - x) > 0.12 or abs(py - y) > 0.12:
                    changed |= self._mark(cx, cy, FREE)
            changed |= self._mark(int(hx / MAP_RES), int(hy / MAP_RES), OCCUPIED)
        if changed:
            self.version += 1
        return changed

    def percent(self) -> int:
        """Доля разведанной площади — по ней на экране видно, как идёт съёмка."""
        return int(round(100.0 * self.scanned / MAP_AREA))

    def rle(self) -> str:
        """Клетки одной строкой: «значение×количество» через запятую."""
        parts = []
        prev, count = None, 0
        for value in self.cells:
            if value == prev:
                count += 1
                continue
            if prev is not None:
                parts.append(f"{prev}*{count}")
            prev, count = value, 1
        if prev is not None:
            parts.append(f"{prev}*{count}")
        return ",".join(parts)


def decode_rle(text: str, expect: int = MAP_AREA) -> bytes:
    """Обратная к :meth:`RoomMap.rle` — нужна тестам и отладке."""
    out = bytearray()
    for chunk in text.split(","):
        value, _, count = chunk.partition("*")
        out.extend([int(value)] * int(count))
    if len(out) != expect:
        raise ValueError(f"в строке {len(out)} клеток, ожидалось {expect}")
    return bytes(out)


def _clampf(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# --- АКБ 12S3P LiFePO4 (совпадает с gui/console-core.js) --------------------
PACK = {
    "series": 12, "parallel": 3, "cellAh": 6.2,
    "nominalV": 38.4, "fullV": 43.8, "emptyV": 30.0,
    "capacityAh": 18.6, "capacityWh": 714.2,
    "internalR": 0.075, "bmsA": 60, "fuseA": 25,
    "lowV": 35.5, "criticalV": 33.5, "tempWarnC": 45, "tempMaxC": 60,
    "ocv": [(2.50, 0), (2.80, 3), (3.00, 7), (3.10, 10), (3.20, 15), (3.25, 30),
            (3.30, 50), (3.33, 70), (3.35, 85), (3.40, 95), (3.45, 98), (3.65, 100)],
}


# ============================================================================
# 1. Протокол обмена
# ============================================================================
def crc16(data):
    """CRC16/Modbus (0xA001, init 0xFFFF). Эталон: b'123456789' → 0x4B37."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def build_command(module_id, steer_cdeg=0, pwm=0, enable=False, home=False):
    """Кадр команды ПК → модуль, 10 байт."""
    steer = max(-32767, min(32767, int(round(steer_cdeg))))
    duty = max(-1000, min(1000, int(round(pwm))))
    flags = (0x01 if enable else 0) | (0x02 if home else 0)
    body = bytes([CMD_SYNC[0], CMD_SYNC[1], module_id & 0xFF]) + \
        struct.pack(">hh", steer, duty) + bytes([flags])
    crc = crc16(body)
    return body + struct.pack(">H", crc)


def parse_telemetry(frame):
    """Кадр телеметрии 16 Б → dict, либо None (длина/синхро/CRC)."""
    if len(frame) < TLM_LEN:
        return None
    if frame[0] != TLM_SYNC[0] or frame[1] != TLM_SYNC[1]:
        return None
    if crc16(frame[:14]) != struct.unpack(">H", frame[14:16])[0]:
        return None
    steer_cdeg, enc_delta, pwm_actual = struct.unpack(">hhh", frame[4:10])
    vbat_cv, current_ma = struct.unpack(">HH", frame[10:14])
    status = frame[3]
    return {
        "moduleId": frame[2],
        "id": MODULE_NAMES.get(frame[2], "M%d" % frame[2]),
        "status": status,
        "enabled": bool(status & STATUS_ENABLED),
        "homed": bool(status & STATUS_HOMED),
        "fault": bool(status & STATUS_FAULT),
        "steerDeg": steer_cdeg / 100.0,
        "encDelta": enc_delta,
        "pwm": pwm_actual,
        "volts": vbat_cv / 100.0,
        "amps": current_ma / 1000.0,
    }


def pick_telemetry(buf):
    """Ищет в буфере корректный кадр. → (кадр | None, остаток буфера)."""
    while len(buf) >= TLM_LEN:
        start = buf.find(bytes(TLM_SYNC))
        if start < 0:
            return None, buf[-1:]          # синхробайтов нет — держим хвост
        if start:
            buf = buf[start:]
        if len(buf) < TLM_LEN:
            return None, buf
        frame = buf[:TLM_LEN]
        parsed = parse_telemetry(frame)
        if parsed:
            return parsed, buf[TLM_LEN:]
        buf = buf[1:]                      # CRC не сошёлся — сдвигаемся на байт
    return None, buf


# ============================================================================
# 2. Модель АКБ 12S3P
# ============================================================================
def _interp(table, x, col):
    key, val = (0, 1) if col == 0 else (1, 0)
    if x <= table[0][key]:
        return table[0][val]
    if x >= table[-1][key]:
        return table[-1][val]
    for i in range(1, len(table)):
        a, b = table[i - 1], table[i]
        if x <= b[key]:
            k = (x - a[key]) / (b[key] - a[key] or 1)
            return a[val] + k * (b[val] - a[val])
    return table[-1][val]


def soc_from_voltage(pack_v):
    """SOC, % — по кривой OCV на элемент."""
    return max(0.0, min(100.0, _interp(PACK["ocv"], (pack_v or 0) / PACK["series"], 0)))


def voltage_from_soc(soc):
    return _interp(PACK["ocv"], max(0.0, min(100.0, soc)), 1) * PACK["series"]


def pack_state(pack_v, pack_a, soc=None, temp_c=28.0):
    """Сводка по АКБ для экрана: %, напряжение, ток, запас хода, состояние."""
    soc = soc_from_voltage(pack_v) if soc is None else max(0.0, min(100.0, soc))
    remaining_wh = PACK["capacityWh"] * soc / 100.0
    wh_per_km = PACK["capacityWh"] / 22.0          # паспортные ~22 км на полный заряд
    if pack_v >= PACK["lowV"]:
        level = "НОРМА"
    elif pack_v >= PACK["criticalV"]:
        level = "НИЗКИЙ"
    else:
        level = "КРИТИЧЕСКИЙ"
    return {
        "soc": round(soc, 1),
        "volts": round(pack_v, 2),
        "amps": round(abs(pack_a), 2),
        "watts": round(pack_v * abs(pack_a), 1),
        "remainingWh": round(remaining_wh, 1),
        "rangeKm": round(remaining_wh / wh_per_km, 1),
        "state": "заряд" if pack_a < -0.2 else ("разряд" if pack_a > 0.2 else "покой"),
        "level": level,
        "tempC": round(temp_c, 1),
        "cellMin": round(pack_v / PACK["series"] - 0.011, 3),
        "cellMax": round(pack_v / PACK["series"] + 0.013, 3),
        "thresholds": {"lowV": PACK["lowV"], "criticalV": PACK["criticalV"]},
    }


# ============================================================================
# 3. Источники данных
# ============================================================================
class SimSource:
    """Стенд: ИИ ведёт робота по заданиям, дальномер размечает карту.

    Манёвры больше не сценарий: сеть на 32 нейрона (gui/ai_driver.py) решает
    на каждом такте, как встать колёсами, и учится на ходу, повторяя
    геометрического учителя. Те же скорости уходят в ROS, если рядом rclpy.
    """

    name = "sim"

    def __init__(self, ai_ros=None):
        self.t0 = time.time()
        self.soc = 78.0
        self.last = time.time()
        self.frames_ok = 0
        self.motors = [{"id": mid, "title": MODULE_TITLES[mid], "angle": 0.0,
                        "rpm": 0.0, "temp": 36.0, "homed": True} for mid in MODULE_NAMES.values()]

        # --- водитель и его связь с ROS ---
        if ai_ros is None:
            ai_ros = os.environ.get("RC_AI_ROS", "0").strip().lower() in {"1", "true", "yes", "on"}
        self.driver = ai_driver.NeuralDriver()
        self.pretrain_loss = self.driver.pretrain()      # урок перед выездом
        self.sink = ai_driver.RosCommandSink(enabled=ai_ros)

        # --- цех: место робота, карта, след ---
        self.map = RoomMap()
        self.x, self.y, self.th = WORLD_PADS[2][0], WORLD_PADS[2][1], 0.0
        self.trail = [[round(self.x, 2), round(self.y, 2)]]
        self.goal_index = 0
        self.goal_entered = time.time()
        self.goal_hold_until = 0.0
        self.turn_target = None
        self.turn_start = 0.0

        self.target_vec = None            # цель обучения, сглаженная по тактам
        self.throttle = 0.0               # тяга после привода (фильтр)
        self.last_inputs = {}
        self.last_target = {}
        self.last_command = {"label": "—", "mode": "—", "steer": "прямо", "angles": {},
                             "throttle": 0.0, "speed": 0.0}
        self.command_log = []
        self.vx = self.vy = self.wz = 0.0
        self.stuck_s = 0.0                            # стоим, упёршись в препятствие
        self.stall_s = 0.0                            # стоим, не выполняя задание

    # --- обмен ------------------------------------------------------------
    def bus(self):
        """Счётчики обмена для экрана диагностики.

        У демонстрационного источника кадров на линии нет, поэтому счётчики
        помечены ``simulated``: панель диагностики не должна выдавать модель
        за реальный обмен с модулями.
        """
        return {
            "simulated": True,
            "framesOk": self.frames_ok,
            "discarded": 0,
            "ageMs": int((time.time() - self.last) * 1000),
            "online": len(MODULE_NAMES),
            "modules": sorted(MODULE_NAMES),
        }

    # --- задания ----------------------------------------------------------
    def _goal(self):
        return DEMO_GOALS[self.goal_index % len(DEMO_GOALS)]

    def _goal_error(self, goal):
        """Задание в системе робота: (dx, dy) в метрах и рассогласование курса."""
        if goal.get("turn") is not None:
            if self.turn_target is None:                  # курс задания, ° от оси x
                self.turn_target = math.radians(goal["turn"])
            dth = self.turn_target - self.th
            while dth > math.pi:
                dth -= 2 * math.pi
            while dth <= -math.pi:
                dth += 2 * math.pi
            return 0.0, 0.0, dth
        gx, gy = float(goal["x"]) - self.x, float(goal["y"]) - self.y
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)
        dx = gx * cos_t + gy * sin_t                  # вперёд
        dy = -gx * sin_t + gy * cos_t                 # влево
        if goal.get("crab"):
            return dx, dy, 0.0                        # боком: курс держим, не доворачиваем
        dth = math.atan2(gy, gx) - self.th
        while dth > math.pi:
            dth -= 2 * math.pi
        while dth <= -math.pi:
            dth += 2 * math.pi
        return dx, dy, dth

    def _goal_reached(self, goal, dx, dy, dth, now) -> bool:
        if now < self.goal_hold_until:
            return False
        if (now - self.goal_entered) > 8.0:           # задание зависло — идём дальше
            return True
        if self.stall_s > 2.5:                        # робот встал: задание не идёт
            return True
        if goal.get("turn") is not None:
            return abs(math.degrees(dth)) < 6.0
        if goal.get("crab"):
            return math.hypot(dx, dy) < 0.18
        return math.hypot(dx, dy) < 0.22

    def _next_goal(self, now):
        self.goal_index = (self.goal_index + 1) % len(DEMO_GOALS)
        self.goal_entered = now
        self.goal_hold_until = now + 0.5              # пауза: робот осматривается
        self.turn_target = None

    @staticmethod
    def _free(x: float, y: float) -> bool:
        """Место свободно для центра машины (с запасом от стен)."""
        return (not RoomMap.blocked(x, y)) and 0.09 <= x <= WORLD_W - 0.09 and 0.09 <= y <= WORLD_H - 0.09

    def _obstacles(self):
        """Близость препятствий слева/по центру/справа — входы дальномера."""
        out = []
        for offset in (math.radians(-35), 0.0, math.radians(35)):
            dist, _ = RoomMap.cast(self.x, self.y, self.th + offset, 1.6)
            out.append(round(max(0.0, 1.0 - dist / 1.6), 3))
        return out

    def _inputs(self, goal, dx, dy, dth, obst):
        data = ai_driver.scenario_inputs(dx, dy, dth, bool(goal.get("crab")), obst,
                                         soc=self.soc / 100.0,
                                         speed=abs(self.vx) / ai_driver.V_MAX)
        self.last_inputs = data
        return [data[name] for name in ai_driver.INPUTS]

    # --- такт -------------------------------------------------------------
    def read(self):
        now = time.time()
        dt = max(0.001, min(0.5, now - self.last))
        self.last = now
        t = now - self.t0

        goal = self._goal()
        dx, dy, dth = self._goal_error(goal)
        obst = self._obstacles()
        vector = self._inputs(goal, dx, dy, dth, obst)
        # краб разрешён только на заданиях с пометкой «crab»: там машина идёт
        # боком, а на остальных доворачивается и едет — так манёвры не хлопают
        target = ai_driver.geometric_target(dx, dy, dth, crab=bool(goal.get("crab")), obst=obst)
        wanted = list(target["vector"])
        if self.target_vec is None:
            self.target_vec = wanted
        else:
            k = min(1.0, dt * 3.0)                    # сглаживание цели, ~0,3 с
            self.target_vec = [p + (w - p) * k for p, w in zip(self.target_vec, wanted)]
        self.last_target = {"mode": target["mode"]}

        out = self.driver.step(vector, self.target_vec)
        # тяга проходит через привод: фильтр сглаживает дрожь выходов сети
        raw = ai_driver.command_from_outputs(out["y"])
        k = min(1.0, dt * 3.0)
        self.throttle = self.throttle + (raw["throttle"] - self.throttle) * k
        cmd = ai_driver.command_from_outputs(out["y"])
        cmd["throttle"] = self.throttle
        cmd["label"] = cmd["label"].rsplit(" · ", 1)[0] + " · " + \
            ("%.1f м/с" % (abs(self.throttle) * ai_driver.V_MAX)).replace(".", ",")
        self.last_command = cmd

        # рулевые модули перекладываются с ограниченной скоростью, как настоящие
        step = DEMO_STEER_RATE * dt
        actual = {}
        for i, m in enumerate(self.motors):
            want = float(cmd["angles"].get(m["id"], 0.0))
            diff = want - m["angle"]
            m["angle"] = want if abs(diff) <= step else m["angle"] + math.copysign(step, diff)
            actual[m["id"]] = m["angle"]
            target_rpm = cmd["throttle"] * 240.0
            m["rpm"] += (target_rpm - m["rpm"]) * min(1.0, dt * 2.0)
            m["temp"] = 34.0 + abs(m["rpm"]) / 40.0 + math.sin(t / 5 + i) * 1.4

        # движение: та же модель, что уходит в ROS
        self.vx, self.vy, self.wz = ai_driver.body_velocity(actual, cmd["throttle"])
        cos_t, sin_t = math.cos(self.th), math.sin(self.th)
        nx = self.x + (self.vx * cos_t - self.vy * sin_t) * dt
        ny = self.y + (self.vx * sin_t + self.vy * cos_t) * dt
        if not self._free(nx, ny):
            if self._free(nx, self.y):          # задели препятствие — скользим вдоль него
                ny = self.y
            elif self._free(self.x, ny):
                nx = self.x
            else:
                nx, ny, self.stuck_s = self.x, self.y, self.stuck_s + dt
        moved = math.hypot(nx - self.x, ny - self.y)
        turned = abs(self.wz * dt)
        self.x, self.y = nx, ny
        self.th = (self.th + self.wz * dt) % (2 * math.pi)
        # «стоим» — когда почти не двигаемся и не разворачиваемся: задание не
        # выполняется, и киоск не должен смотреть в замершего робота
        self.stall_s = self.stall_s + dt if (moved < 0.02 and turned < 0.01) else 0.0 % (2 * math.pi)

        if moved:
            last_pt = self.trail[-1]
            if math.hypot(self.x - last_pt[0], self.y - last_pt[1]) >= 0.12:
                self.trail.append([round(self.x, 2), round(self.y, 2)])
                del self.trail[:-200]

        self.map.scan(self.x, self.y, self.th)
        self.sink.publish(self.vx, self.vy, self.wz)      # ROS 2: /cmd_vel

        if self._goal_reached(goal, dx, dy, dth, now):
            self._next_goal(now)

        speed = math.hypot(self.vx, self.vy)
        amps = 4.0 + 16.0 * abs(cmd["throttle"]) + 3.0 * abs(self.wz)
        self.soc = max(4.0, self.soc - amps * dt / 3600.0 * 100.0 / PACK["capacityAh"])
        volts = voltage_from_soc(self.soc) - amps * PACK["internalR"]
        return self._payload(self.motors, volts, amps, speed, cmd["label"])

    # --- данные наружу ----------------------------------------------------
    def _payload(self, motors, volts, amps, speed, drive_mode=None):
        # 4 модуля × 1 кадр телеметрии за выборку — как на реальной шине.
        self.frames_ok += len(MODULE_NAMES)
        battery = pack_state(volts, amps, self.soc)
        goal = self._goal()
        dx, dy, dth = self._goal_error(goal)
        self._log_command()
        return {
            "motors": motors,
            "battery": battery,
            "cargo": {"kg": 80, "closed": True},
            "mode": "АВТОНОМНЫЙ РЕЖИМ",
            "driveMode": drive_mode,
            "route": "склад → зона выгрузки",
            "speedMps": round(speed, 2),
            "powerKw": round(battery["watts"] / 1000.0, 2),
            "linkOk": True,
            "ai": self.ai_state(goal, dx, dy, dth),
            "map": self.map_state(goal),
        }

    def _log_command(self):
        """Журнал команд: пишем, когда меняется манёвр или сторона поворота."""
        cmd = self.last_command
        key = (cmd.get("mode"), cmd.get("steer"))
        if self.command_log and self.command_log[-1].get("key") == key:
            return
        self.command_log.append({
            "key": key,
            "time": time.strftime("%H:%M:%S"),
            "label": cmd.get("label", "—"),
            "mode": cmd.get("mode", "—"),
        })
        del self.command_log[:-12]

    def ai_state(self, goal, dx, dy, dth):
        cmd = self.last_command
        return {
            "available": True,
            "neurons": self.driver.neurons,
            "layers": self.driver.layers,
            "command": cmd.get("label", "—"),
            "mode": cmd.get("mode", "—"),
            "steer": cmd.get("steer", "прямо"),
            "angles": {k: round(v, 1) for k, v in (cmd.get("angles") or {}).items()},
            "throttle": round(float(cmd.get("throttle", 0.0)), 3),
            "speed": round(abs(float(cmd.get("throttle", 0.0))) * ai_driver.V_MAX, 2),
            "loss": round(float(self.driver.loss), 4),
            "pretrainLoss": round(float(self.pretrain_loss), 5),
            "lossAvg": round(float(self.driver.loss_avg or 0.0), 4),
            "steps": self.driver.steps,
            "activations": self.driver.activations(),
            "inputs": dict(self.last_inputs),
            "teacher": self.last_target.get("mode", "—"),
            "goal": {
                "label": goal["label"],
                "dist": round(math.hypot(dx, dy), 2),
                "dth": round(math.degrees(dth), 1),
            },
            "log": list(self.command_log[-6:]),
            "ros": self.sink.status(),
            "stuckSec": round(self.stuck_s, 1),
            "stallSec": round(self.stall_s, 1),
        }

    def map_state(self, goal=None):
        goal = goal or self._goal()
        return {
            "ok": True,
            "w": MAP_W, "h": MAP_H, "res": MAP_RES,
            "world": {"w": WORLD_W, "h": WORLD_H},
            "version": self.map.version,
            "scanPct": self.map.percent(),
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3), "th": round(self.th, 4)},
            "trail": self.trail[-160:],
            "pads": [{"x": p[0], "y": p[1], "label": p[2]} for p in WORLD_PADS],
            "goal": {"x": goal.get("x"), "y": goal.get("y"), "label": goal["label"]},
            "source": "sim",
        }


class SerialSource:
    """Реальные модули: 4 UART, кадры телеметрии 16 Б (20 Гц)."""

    name = "serial"

    def __init__(self, ports, baud=115200):
        self.ports = ports
        self.baud = baud
        self.lock = threading.Lock()
        self.frames = {}                 # id модуля → последний кадр
        self.cargo = {"kg": 80, "closed": True}
        self.seen = 0.0
        self.frames_ok = 0               # разобрано корректных кадров
        self.discarded = 0               # байт выброшено при поиске синхрослова/сбое CRC
        self._stop = False
        for idx, port in enumerate(ports):
            threading.Thread(target=self._reader, args=(port, idx + 1), daemon=True).start()

    def bus(self):
        """Реальные счётчики линии: кадры, отброшенные байты, возраст данных."""
        with self.lock:
            seen = self.seen
            frames_ok = self.frames_ok
            discarded = self.discarded
            present = sorted(self.frames)
        return {
            "simulated": False,
            "framesOk": frames_ok,
            "discarded": discarded,
            "ageMs": int((time.time() - seen) * 1000) if seen else None,
            "online": len(present),
            "modules": present,
            "ports": list(self.ports),
        }

    def _reader(self, port, default_id):
        try:
            import serial                    # pyserial, только для реального железа
        except ImportError:
            print("! serial: pyserial не установлен (pip install pyserial) — модуль %s выключен" % port)
            return
        buf = b""
        while not self._stop:
            try:
                with serial.Serial(port, self.baud, timeout=0.2) as dev:
                    print("· serial: %s открыт (модуль %d)" % (port, default_id))
                    while not self._stop:
                        chunk = dev.read(64)
                        if not chunk:
                            continue
                        buf += chunk
                        while True:
                            before = len(buf)
                            frame, buf = pick_telemetry(buf)
                            if not frame:
                                break
                            mid = frame.get("moduleId") or default_id
                            with self.lock:
                                self.frames[mid] = frame
                                self.seen = time.time()
                                self.frames_ok += 1
                                # Всё, что ушло из буфера сверх длины кадра, —
                                # мусор до синхрослова или отброшенный кадр с
                                # несошедшимся CRC (pick_telemetry сдвигает буфер
                                # на байт). Это и есть счётчик ошибок линии.
                                self.discarded += max(0, before - len(buf) - TLM_LEN)
            except Exception as exc:          # noqa: BLE001 — порт может пропасть
                print("! serial %s: %s — повтор через 2 с" % (port, exc))
                time.sleep(2.0)

    def read(self):
        with self.lock:
            frames = dict(self.frames)
            seen = self.seen
        motors = []
        volts_all, amps_all = [], []
        for num, name in MODULE_NAMES.items():
            f = frames.get(num)
            if not f:
                motors.append({"id": name, "title": MODULE_TITLES[name], "angle": 0.0,
                               "rpm": 0.0, "temp": 0.0, "homed": False, "online": False})
                continue
            rpm = f["encDelta"] * 6 * 60 / 360.0 * 10            # 6 имп/об·10 → об/мин (при 10 Гц опросе)
            motors.append({"id": name, "title": MODULE_TITLES[name], "angle": f["steerDeg"],
                           "rpm": rpm, "temp": 0.0, "homed": f["homed"], "online": True,
                           "fault": f["fault"], "amps": f["amps"], "pwm": f["pwm"]})
            volts_all.append(f["volts"])
            amps_all.append(f["amps"])
        volts = sum(volts_all) / len(volts_all) if volts_all else 0.0
        amps = sum(amps_all) if amps_all else 0.0
        link = (time.time() - seen) < 1.0 if seen else False
        battery = pack_state(volts or PACK["nominalV"], amps)
        return {
            "motors": motors,
            "battery": battery,
            "cargo": dict(self.cargo),
            "mode": "АВТОНОМНЫЙ РЕЖИМ" if link else "НЕТ СВЯЗИ С МОДУЛЯМИ",
            "driveMode": None,
            "ai": {
                "available": False,
                "reason": "реальные модули: ИИ ведёт только стенд (—source sim)",
            },
            "map": {
                "ok": False,
                "reason": "карту даёт SLAM: на стенде — дальномер, на роботе — ROS (/map)",
            },
            "route": "—",
            "speedMps": round(sum(abs(m.get("rpm", 0)) for m in motors) / 4 * 2 * math.pi * WHEEL_R_M / 60, 2),
            "powerKw": round(battery["watts"] / 1000.0, 2),
            "linkOk": link,
        }


class RosSource:
    """ROS 2 (rclpy): /modules/state, /battery. Если rclpy нет — падаем в sim."""

    name = "ros"

    def __init__(self):
        self.ok = False
        self.data = {}
        self.frames_ok = 0
        self.last_msg = 0.0
        try:
            import rclpy                                            # noqa: F401
            from rclpy.node import Node
        except ImportError:
            print("! ros: rclpy не найден — используется демонстрационный источник")
            self.fallback = SimSource()
            return
        self.fallback = None
        threading.Thread(target=self._spin_node, daemon=True).start()

    def bus(self):
        """Счётчики моста ROS 2: сколько сообщений принято и как давно."""
        if self.fallback:
            return self.fallback.bus()
        return {
            "simulated": False,
            "transport": "ros2",
            "framesOk": self.frames_ok,
            "discarded": 0,
            "ageMs": int((time.time() - self.last_msg) * 1000) if self.last_msg else None,
            "online": 1 if self.ok else 0,
            "modules": [],
        }

    def _touch(self):
        self.frames_ok += 1
        self.last_msg = time.time()

    def _spin_node(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32, String

        source = self

        class Bridge(Node):
            def __init__(self):
                super().__init__("rus_slam_screen_bridge")
                self.create_subscription(String, "/screen/state", source._on_state, 10)
                self.create_subscription(Float32, "/battery/voltage", source._on_volts, 10)
                self.create_subscription(Float32, "/battery/current", source._on_amps, 10)
                self.create_subscription(String, "/screen/motors", source._on_motors, 10)

        rclpy.init(args=None)
        node = Bridge()
        self.ok = True
        print("· ros: мост экрана запущен (/screen/state, /battery/*, /screen/motors)")
        try:
            rclpy.spin(node)
        except Exception:                                            # noqa: BLE001
            pass

    def _on_state(self, msg):
        self._touch()
        try:
            self.data.update(json.loads(msg.data))
        except Exception:                                            # noqa: BLE001
            pass

    def _on_volts(self, msg):
        self._touch()
        self.data.setdefault("battery", {})["volts"] = float(msg.data)

    def _on_amps(self, msg):
        self._touch()
        self.data.setdefault("battery", {})["amps"] = float(msg.data)

    def _on_motors(self, msg):
        self._touch()
        try:
            self.data["motors"] = json.loads(msg.data)
        except Exception:                                            # noqa: BLE001
            pass

    def read(self):
        if self.fallback:
            # rclpy не поднялся: честно работаем стендом, ИИ и карта — стендовые
            return self.fallback.read()
        d = self.data
        volts = float(d.get("battery", {}).get("volts") or 0.0)
        amps = float(d.get("battery", {}).get("amps") or 0.0)
        return {
            "motors": d.get("motors") or [],
            "battery": pack_state(volts or PACK["nominalV"], amps),
            "cargo": d.get("cargo") or {"kg": 0, "closed": True},
            "mode": d.get("mode") or ("АВТОНОМНЫЙ РЕЖИМ" if self.ok else "ОЖИДАНИЕ ROS"),
            "route": d.get("route") or "—",
            "speedMps": d.get("speedMps") or 0.0,
            "powerKw": round((volts * abs(amps)) / 1000.0, 2),
            "linkOk": self.ok,
        }


# ============================================================================
# 4. Замок грузового отсека (PIN, блокировки, аудит)
# ============================================================================
class Vault:
    """PIN-хранилище: SHA-256 + соль, счётчик попыток, аудит с хеш-цепочкой."""

    def __init__(self, path=LOCK_FILE, default_pin=DEFAULT_PIN,
                 max_attempts=MAX_ATTEMPTS, lock_ms=LOCK_MS, clock=time.time):
        self.path = path
        self.default_pin = default_pin
        self.max_attempts = max_attempts
        self.lock_ms = lock_ms
        self.clock = clock
        self.lock = threading.Lock()
        self.data = self._load()

    # --- файл ---
    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                raw = json.load(fh)
            if isinstance(raw, dict) and raw.get("pinHash"):
                raw.setdefault("audit", [])
                raw.setdefault("fails", 0)
                raw.setdefault("lockUntil", 0)
                return raw
        except (OSError, ValueError):
            pass
        return self._fresh()

    def _fresh(self):
        salt = secrets.token_hex(16)
        return {
            "version": 1, "salt": salt,
            "pinHash": self._hash(salt, self.default_pin),
            "fails": 0, "lockUntil": 0, "audit": [], "changedAt": time.time(),
        }

    def _persist(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(self.data, fh, ensure_ascii=False, indent=2)
            os.replace(tmp, self.path)
        except OSError as exc:
            print("! не удалось сохранить %s: %s" % (self.path, exc))

    # --- крипто ---
    @staticmethod
    def _hash(salt, pin):
        return hashlib.sha256((salt + ":" + str(pin)).encode("utf-8")).hexdigest()

    def _audit(self, action, ok, detail=""):
        prev = self.data["audit"][-1]["hash"] if self.data["audit"] else "genesis"
        ts = self.clock()
        payload = "%s|%.0f|%s|%d|%s" % (prev, ts * 1000, action, 1 if ok else 0, detail)
        entry = {"ts": ts, "action": action, "ok": bool(ok), "detail": detail,
                 "prev": prev, "hash": hashlib.sha256(payload.encode("utf-8")).hexdigest()}
        self.data["audit"].append(entry)
        del self.data["audit"][:-200]

    def verify_audit(self):
        prev = "genesis"
        for i, e in enumerate(self.data["audit"]):
            if e.get("prev") != prev:
                return {"ok": False, "brokenAt": i}
            payload = "%s|%.0f|%s|%d|%s" % (prev, e["ts"] * 1000, e["action"],
                                            1 if e["ok"] else 0, e.get("detail", ""))
            if hashlib.sha256(payload.encode("utf-8")).hexdigest() != e.get("hash"):
                return {"ok": False, "brokenAt": i}
            prev = e["hash"]
        return {"ok": True, "brokenAt": -1}

    # --- логика ---
    def lock_remaining_ms(self, now=None):
        now = self.clock() if now is None else now
        return max(0, int((self.data.get("lockUntil", 0) - now) * 1000))

    def state(self, open_flag=False):
        rem = self.lock_remaining_ms()
        return {
            "open": bool(open_flag),
            "blocked": rem > 0,
            "remainingMs": rem,
            "attemptsLeft": max(0, self.max_attempts - int(self.data.get("fails", 0))),
            "changedAt": self.data.get("changedAt"),
        }

    def verify(self, pin):
        with self.lock:
            now = self.clock()
            if self.lock_remaining_ms(now) > 0:
                self._audit("unlock", False, "ввод заблокирован")
                self._persist()
                return {"ok": False, "reason": "blocked",
                        "remainingMs": self.lock_remaining_ms(now),
                        "attemptsLeft": 0}
            pin = re.sub(r"\D", "", str(pin or ""))
            if not 4 <= len(pin) <= 8:
                return {"ok": False, "reason": "format", "attemptsLeft": self.state()["attemptsLeft"]}
            if self._hash(self.data["salt"], pin) == self.data["pinHash"]:
                self.data["fails"] = 0
                self.data["lockUntil"] = 0
                self.data["lastOpenAt"] = now
                self._audit("unlock", True, "доступ разрешён")
                self._persist()
                return {"ok": True, "reason": "ok", "attemptsLeft": self.max_attempts}
            if self.data.get("lockUntil") and self.data["lockUntil"] <= now:
                self.data["fails"] = 0
                self.data["lockUntil"] = 0
            self.data["fails"] = int(self.data.get("fails", 0)) + 1
            if self.data["fails"] >= self.max_attempts:
                self.data["fails"] = 0
                self.data["lockUntil"] = now + self.lock_ms / 1000.0
                self._audit("unlock", False, "блокировка после %d попыток" % self.max_attempts)
                self._persist()
                return {"ok": False, "reason": "blocked", "remainingMs": self.lock_ms, "attemptsLeft": 0}
            self._audit("unlock", False, "неверный PIN")
            self._persist()
            return {"ok": False, "reason": "wrong",
                    "attemptsLeft": max(0, self.max_attempts - self.data["fails"])}

    def set_pin(self, current, new):
        res = self.verify(current)
        if not res.get("ok"):
            return {"ok": False, "reason": res.get("reason"), "detail": "текущий PIN неверен"}
        new = re.sub(r"\D", "", str(new or ""))
        if not 4 <= len(new) <= 8:
            return {"ok": False, "reason": "format", "detail": "новый PIN — 4…8 цифр"}
        with self.lock:
            self.data["salt"] = secrets.token_hex(16)
            self.data["pinHash"] = self._hash(self.data["salt"], new)
            self.data["changedAt"] = self.clock()
            self._audit("pin_change", True, "PIN изменён")
            self._persist()
        return {"ok": True}

    def audit(self, limit=20):
        items = self.data["audit"][-int(limit):][::-1]
        return [{k: v for k, v in e.items() if k != "hash"} for e in items]


def hardware_lock(action):
    """Реальный замок: внешняя команда из окружения (реле/соленоид/GPIO)."""
    cmd = os.environ.get("RUS_SLAM_LOCK_CMD_CLOSE" if action == "close" else "RUS_SLAM_LOCK_CMD")
    if not cmd:
        return "software"
    try:
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return "hardware"
    except OSError as exc:
        print("! замок: команда не выполнена (%s)" % exc)
        return "error"


# ============================================================================
# 5. Состояние приложения
# ============================================================================
class App:
    def __init__(self, source, vault, cargo_nominal_kg=80):
        self.source = source
        self.vault = vault
        self.cargo_nominal_kg = cargo_nominal_kg
        self.lock_open = False
        self.started = time.time()

    def snapshot(self):
        data = self.source.read()
        lock_state = self.vault.state(self.lock_open)
        cargo = dict(data.get("cargo") or {})
        cargo["closed"] = not self.lock_open
        cargo.setdefault("kg", self.cargo_nominal_kg)
        out = dict(data)
        out["cargo"] = cargo
        out["lock"] = lock_state
        out["source"] = getattr(self.source, "name", "unknown")
        bus = getattr(self.source, "bus", None)
        if callable(bus):
            try:
                out["bus"] = bus()
            except Exception:                                        # noqa: BLE001
                pass
        out["uptimeSec"] = int(time.time() - self.started)
        out["ts"] = time.time()
        return out

    def map_cells(self):
        """Клетки карты строкой: экран забирает их, когда сменилась версия."""
        source = self.source
        room = getattr(source, "map", None)
        if room is None:
            return {"ok": False, "reason": "у источника данных нет карты"}
        return {
            "ok": True,
            "version": room.version,
            "w": MAP_W,
            "h": MAP_H,
            "res": MAP_RES,
            "cells": room.rle(),
        }

    def open_lock(self, pin):
        res = self.vault.verify(pin)
        if res.get("ok"):
            self.lock_open = True
            res["hardware"] = hardware_lock("open")
            res["lock"] = self.vault.state(True)
        else:
            res["lock"] = self.vault.state(self.lock_open)
        return res

    def close_lock(self):
        if not self.lock_open:
            return {"ok": True, "lock": self.vault.state(False)}
        self.lock_open = False
        return {"ok": True, "hardware": hardware_lock("close"), "lock": self.vault.state(False)}


# ============================================================================
# 6. HTTP: статика + JSON API
# ============================================================================
class ApiHandler(SimpleHTTPRequestHandler):
    app = None                      # проставляется в main()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    # --- утилиты ---
    def _json(self, payload, code=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b"{}"
            return json.loads(raw.decode("utf-8") or "{}")
        except (ValueError, UnicodeDecodeError, OSError):
            return {}

    # --- маршруты ---
    def do_GET(self):                                            # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/api/state":
            return self._json({"ok": True, "data": self.app.snapshot()})
        if path == "/api/map":
            # клетки отдельно от состояния: 6 КБ на каждый опрос в 300 мс ни к чему
            return self._json(self.app.map_cells())
        if path == "/api/audit":
            limit = 20
            m = re.search(r"[?&]limit=(\d+)", self.path)
            if m:
                limit = max(1, min(200, int(m.group(1))))
            return self._json({"ok": True, "audit": self.app.vault.audit(limit),
                               "integrity": self.app.vault.verify_audit()})
        if path == "/api/health":
            return self._json({"ok": True, "source": getattr(self.app.source, "name", "?"),
                               "uptimeSec": int(time.time() - self.app.started),
                               "pinDefault": self.app.vault.default_pin})
        if path in ("/", "/index.htm", "/default.html"):
            self.send_response(302)
            self.send_header("Location", MAIN_PAGE)
            self.end_headers()
            return
        if path == "/console":
            self.send_response(302)
            self.send_header("Location", CONSOLE_PAGE)
            self.end_headers()
            return
        super().do_GET()

    def do_POST(self):                                           # noqa: N802
        path = self.path.split("?", 1)[0]
        body = self._body()
        if path == "/api/lock/open":
            res = self.app.open_lock(body.get("pin", ""))
            return self._json(res, 200 if res.get("ok") else 403)
        if path == "/api/lock/close":
            return self._json(self.app.close_lock())
        if path == "/api/lock/pin":
            res = self.app.vault.set_pin(body.get("current", ""), body.get("new", ""))
            if res.get("ok"):
                self.app.close_lock()
            return self._json(res, 200 if res.get("ok") else 403)
        return self._json({"ok": False, "error": "неизвестный метод"}, 404)

    # --- служебное ---
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        super().end_headers()

    def log_message(self, fmt, *args):
        status = str(args[1]) if len(args) > 1 else ""
        if status.startswith(("4", "5")):
            super().log_message(fmt, *args)


# ============================================================================
# 7. Запуск
# ============================================================================
KIOSK_CANDIDATES = ["chromium-browser", "chromium", "google-chrome",
                    "google-chrome-stable", "microsoft-edge", "firefox"]


def local_ips():
    try:
        return [ip for ip in socket.gethostbyname_ex(socket.gethostname())[2]
                if not ip.startswith("127.")]
    except OSError:
        return []


def launch_kiosk(url):
    exe = next((shutil.which(c) for c in KIOSK_CANDIDATES if shutil.which(c)), None)
    if not exe:
        print("! браузер не найден — откройте вручную:", url)
        return
    base = os.path.basename(exe)
    args = ([exe, "--kiosk", url] if "firefox" in base
            else [exe, "--kiosk", "--app=" + url, "--noerrdialogs", "--disable-infobars",
                  "--autoplay-policy=no-user-gesture-required"])
    try:
        subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("· браузер открыт в киоск-режиме:", base)
    except OSError as exc:
        print("! не удалось открыть браузер:", exc)


def make_source(kind, ports):
    if kind == "serial":
        return SerialSource(ports or ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"])
    if kind == "ros":
        return RosSource()
    if kind == "auto":
        src = RosSource()
        return src if not getattr(src, "fallback", None) else src
    return SimSource()


def build_app(args):
    source = make_source(args.source, args.ports)
    vault = Vault(args.lock_file or LOCK_FILE, args.pin or DEFAULT_PIN,
                  max_attempts=args.max_attempts, lock_ms=int(args.lock_sec * 1000))
    return App(source, vault)


def main(argv=None):
    ap = argparse.ArgumentParser(description="RUS SLAM: основной экран робота и API")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--source", choices=["sim", "serial", "ros", "auto"], default="sim",
                    help="источник данных (по умолчанию sim — стенд)")
    ap.add_argument("--ports", default="", help="UART-порты для --source serial, через запятую")
    ap.add_argument("--pin", default=DEFAULT_PIN, help="заводской PIN отсека (по умолчанию 2580)")
    ap.add_argument("--lock-file", default="", help="файл состояния замка (по умолчанию gui/state/lock.json)")
    ap.add_argument("--max-attempts", type=int, default=MAX_ATTEMPTS)
    ap.add_argument("--lock-sec", type=float, default=LOCK_MS / 1000.0)
    ap.add_argument("--kiosk", action="store_true",
                    help="опция: открыть основной экран браузером на дисплее робота (без рамок)")
    ap.add_argument("--open", action="store_true",
                    help="опция: открыть основной экран в браузере робота (с рамками)")
    ap.add_argument("--delay", type=float, default=1.2)
    ap.add_argument("--reset-pin", action="store_true", help="сбросить PIN к заводскому")
    args = ap.parse_args(argv)

    lock_path = args.lock_file or LOCK_FILE
    if args.reset_pin:
        try:
            os.remove(lock_path)
            print("· PIN сброшен к заводскому (%s)" % args.pin)
        except OSError:
            pass

    app = build_app(args)
    if not os.path.exists(lock_path):
        app.vault._persist()                                  # noqa: SLF001 — первичная запись
    ApiHandler.app = app

    server = ThreadingHTTPServer((args.host, args.port),
                                 partial(ApiHandler))
    url = "http://127.0.0.1:%d%s" % (args.port, MAIN_PAGE)

    print("RUS SLAM · сервер борта: страницы и API (сам окон не открывает)")
    print("  источник данных : %s" % getattr(app.source, "name", "?"))
    print("  на роботе       : %s" % url)
    for ip in local_ips():
        print("  с планшета/ПК   : http://%s:%d%s" % (ip, args.port, MAIN_PAGE))
    print("  пульт (удалённо): http://127.0.0.1:%d%s" % (args.port, CONSOLE_PAGE))
    print("  API             : /api/state · /api/lock/open · /api/lock/close · /api/audit · /api/health")
    print("  дисплей робота  : страницу показывает deploy/rus-slam-display.service")
    print("                    (разово — флаг --kiosk); сервер только отдаёт данные")
    print("  Ctrl+C — остановить\n")

    if args.kiosk or args.open:
        def opener():
            time.sleep(max(0.0, args.delay))
            launch_kiosk(url) if args.kiosk else webbrowser.open(url)
        threading.Thread(target=opener, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n· остановлено оператором")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
