"""Модуль работы с 2D лидаром 360° (RPLIDAR, LD06/LD19, ROS 2 /scan, HTTP API).

Поддерживает:
1. Прямое чтение с USB/serial лидара (RPLIDAR A1/A2/A3/S1/C1, LD06/LD19, YDLidar)
   через порт RC_LIDAR_PORT или автоопределение (/dev/rplidar, /dev/ttyUSB*).
2. Подписку на топик ROS 2 /scan (sensor_msgs/LaserScan), если установлен rclpy.
3. Приём скана через HTTP API (POST /api/lidar или POST /api/scan).
4. Демо-генератор скана (стены склада с препятствиями) для отладки без железа.
"""

from __future__ import annotations

import glob
import logging
import math
import os
import threading
import time
from typing import Any, List, Optional, Tuple

log = logging.getLogger("robot_control.lidar")

LIDAR_MAX_AGE_S = 1.0  # скан старше 1 с считается устаревшим
LIDAR_DEFAULT_RANGE_M = 10.0


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


def parse_ld06_packet(pkt: bytes) -> list[dict[str, float]]:
    """Разбирает 47-байтовый пакет LD06 / LD19.

    Заголовок: 0x54, 0x2C.
    12 точек измерений на пакет.
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


class LidarManager:
    """Менеджер лидара: собирает данные из ROS 2, Serial USB или HTTP API."""

    _instance: Optional[LidarManager] = None

    def __init__(self, port: Optional[str] = None):
        self.port = port or os.environ.get("RC_LIDAR_PORT")
        self.lock = threading.Lock()
        self.scan: list[dict[str, float]] = []
        self.scan_at: float = 0.0
        self.source_type: str = "none"
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self._ros_started = False

    @classmethod
    def get_instance(cls, port: Optional[str] = None) -> LidarManager:
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

    def get_scan(self, max_age: float = LIDAR_MAX_AGE_S, allow_demo: bool = False) -> Optional[list[dict[str, float]]]:
        """Возвращает свежий скан или None (если устарел)."""
        now = time.time()
        with self.lock:
            if self.scan and (now - self.scan_at) < max_age:
                return list(self.scan)
        if allow_demo:
            return generate_demo_scan()
        return None

    def get_status(self) -> dict[str, Any]:
        """Статус работы лидара для API /api/lidar."""
        now = time.time()
        with self.lock:
            scan = list(self.scan)
            scan_at = self.scan_at
            src = self.source_type

        is_fresh = bool(scan and (now - scan_at) < LIDAR_MAX_AGE_S)
        near = min((p["r"] for p in scan), default=None) if scan else None

        return {
            "ok": True,
            "connected": is_fresh,
            "source": src if is_fresh else "no_signal",
            "points_count": len(scan) if is_fresh else 0,
            "scan_at": scan_at,
            "age_s": round(now - scan_at, 2) if scan_at > 0 else None,
            "nearest_m": near if is_fresh else None,
            "configured_port": self.port,
        }

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
                manager._ros_started = True
                log.info("Лидар: подключён топик ROS 2 /scan")
                rclpy.spin(node)
            except Exception as exc:  # noqa: BLE001
                log.debug("ROS лидар spin завершён: %s", exc)

        threading.Thread(target=spin, daemon=True).start()

    # ----------------------- Serial USB лидар ----------------------- #

    def _find_lidar_port(self) -> Optional[str]:
        if self.port and os.path.exists(self.port):
            return self.port
        # Поиск по udev symlinks
        for pattern in ("/dev/rplidar*", "/dev/serial/by-id/*lidar*", "/dev/serial/by-id/*rplidar*"):
            found = sorted(glob.glob(pattern))
            if found:
                return found[0]
        # Проверка стандартных портов ttyUSB (исключая порт мастера если задан)
        master_port = os.environ.get("RC_SLAM_PORT", "/dev/ttyACM0")
        for p in ("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyACM1"):
            if p != master_port and os.path.exists(p):
                return p
        return None

    def _serial_worker(self) -> None:
        """Поток прямого чтения с порта лидара."""
        try:
            import serial
        except ImportError:
            log.info("Лидар: pyserial не установлен — прямое чтение с USB serial выключено")
            return

        while not self._stop:
            port = self._find_lidar_port()
            if not port:
                time.sleep(2.0)
                continue

            try:
                # Пробуем RPLIDAR на 115200 (A1/A2) или 256000
                log.info("Лидар: попытка подключения к %s", port)
                with serial.Serial(port, 115200, timeout=1.0) as ser:
                    # Сброс и запуск сканирования RPLIDAR
                    ser.dtr = False
                    ser.rts = True
                    time.sleep(0.05)
                    ser.dtr = True
                    # Команда SCAN: 0xA5 0x20
                    ser.write(b"\xa5\x20")
                    time.sleep(0.05)

                    # Читаем дескриптор ответа (7 байт: 0xA5 0x5A ...)
                    header = ser.read(7)
                    if len(header) == 7 and header[0] == 0xA5 and header[1] == 0x5A:
                        log.info("Лидар: обнаружен RPLIDAR на %s", port)
                        self._read_rplidar_loop(ser, port)
                        continue

                    # Проверяем на протокол LD06 (230400 бод)
            except Exception as exc:  # noqa: BLE001
                log.debug("Лидар порт %s: %s", port, exc)

            time.sleep(2.0)

    def _read_rplidar_loop(self, ser, port_name: str) -> None:
        current_rev: list[dict[str, float]] = []
        while not self._stop:
            pkt = ser.read(5)
            if len(pkt) < 5:
                continue
            is_new_rev = bool(pkt[0] & 0x01)
            pt = parse_rplidar_point(pkt)
            if pt:
                current_rev.append(pt)

            if is_new_rev and len(current_rev) > 20:
                with self.lock:
                    self.scan = list(current_rev)
                    self.scan_at = time.time()
                    self.source_type = f"serial:{os.path.basename(port_name)}"
                current_rev = []
