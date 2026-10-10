"""Физика и сенсоры: склад 14×9 м, стеллажи, 4-колёсная рулёжка, лидар.

Модель — велосипед с рулём на передней оси (как у SimSource: предел 30°,
скорость рулевого привода 35°/с). Препятствия — отрезки стен и стеллажей.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

ARENA_W, ARENA_H = 14.0, 9.0
WHEELBASE_M = 0.6
ROBOT_RADIUS_M = 0.35
V_MAX = 0.7                  # м/с, вперёд
V_REV = 0.25                 # м/с, назад (для выхода из тупика)
STEER_LIMIT_DEG = 30.0
STEER_RATE_DEG_S = 35.0
LIDAR_BEAMS = 16
LIDAR_RANGE_M = 4.0

Segment = tuple[float, float, float, float]

# Стеллажи: прямоугольники (x0, y0, x1, y1). Базовая раскладка склада.
BASE_SHELVES = [
    (3.0, 1.5, 4.0, 4.0), (3.0, 5.0, 4.0, 7.5),
    (6.5, 1.5, 7.5, 4.0), (6.5, 5.0, 7.5, 7.5),
    (10.0, 1.5, 11.0, 4.0), (10.0, 5.0, 11.0, 7.5),
]
BASE_WAYPOINTS = [(12.5, 4.5), (1.5, 7.5), (12.5, 1.5)]
START_POSE = (1.2, 1.2, 0.0)


def rect_segments(x0, y0, x1, y1) -> list[Segment]:
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def arena_segments() -> list[Segment]:
    return rect_segments(0.0, 0.0, ARENA_W, ARENA_H)


def shelves_to_segments(shelves) -> list[Segment]:
    segs: list[Segment] = []
    for rect in shelves:
        segs.extend(rect_segments(*rect))
    return segs


def random_layout(seed: int) -> tuple[list, list]:
    """Случайная раскладка стеллажей и точек маршрута (для обучения/проверки)."""
    rng = random.Random(seed)
    shelves = []
    for cx in (3.5, 7.0, 10.5):
        for y0 in (1.5, 5.0):
            jx = rng.uniform(-0.4, 0.4)
            jy = rng.uniform(-0.3, 0.3)
            w = rng.uniform(0.9, 1.3)
            h = rng.uniform(2.2, 2.8)
            x0 = cx + jx - w / 2
            y = y0 + jy
            shelves.append((x0, y, x0 + w, y + h))
    waypoints = []
    for _ in range(3):
        while True:
            x, y = rng.uniform(1.0, ARENA_W - 1.0), rng.uniform(1.0, ARENA_H - 1.0)
            if all(not (r[0] - 0.6 < x < r[2] + 0.6 and r[1] - 0.6 < y < r[3] + 0.6)
                   for r in shelves):
                waypoints.append((x, y))
                break
    return shelves, waypoints


def _ray_segment(ox, oy, dx, dy, seg) -> float | None:
    """Расстояние по лучу (ox,oy)+t(dx,dy) до отрезка seg, либо None."""
    x1, y1, x2, y2 = seg
    ex, ey = x2 - x1, y2 - y1
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    wx, wy = x1 - ox, y1 - oy
    t = (wx * ey - wy * ex) / denom
    u = (wx * dy - wy * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def point_segment_dist(px, py, seg) -> float:
    x1, y1, x2, y2 = seg
    ex, ey = x2 - x1, y2 - y1
    L2 = ex * ex + ey * ey or 1e-12
    t = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / L2))
    cx, cy = x1 + t * ex, y1 + t * ey
    return math.hypot(px - cx, py - cy)


@dataclass
class Robot:
    x: float
    y: float
    theta: float
    v: float = 0.0            # фактическая скорость, м/с
    steer: float = 0.0        # фактический угол руля, градусы
    odometer: float = 0.0
    collisions: int = 0       # число событий касания (не шагов)
    touching: bool = False
    path: list = field(default_factory=list)

    def step(self, v_cmd: float, steer_cmd_deg: float, dt: float, segments) -> bool:
        """Один шаг физики. Возвращает True при столкновении (шаг отменяется)."""
        v_cmd = max(-V_REV, min(V_MAX, v_cmd))
        steer_cmd_deg = max(-STEER_LIMIT_DEG, min(STEER_LIMIT_DEG, steer_cmd_deg))
        rate = STEER_RATE_DEG_S * dt
        self.steer += max(-rate, min(rate, steer_cmd_deg - self.steer))
        self.v += max(-1.0 * dt, min(1.0 * dt, v_cmd - self.v))   # разгон ~1 м/с²

        delta = math.radians(self.steer)
        ox, oy, oth = self.x, self.y, self.theta
        self.theta = math.atan2(math.sin(self.theta + self.v / WHEELBASE_M * math.tan(delta) * dt),
                                math.cos(self.theta + self.v / WHEELBASE_M * math.tan(delta) * dt))
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        if any(point_segment_dist(self.x, self.y, s) < ROBOT_RADIUS_M for s in segments):
            self.x, self.y, self.theta = ox, oy, oth
            self.v = 0.0
            if not self.touching:
                self.collisions += 1
            self.touching = True
            return True
        self.touching = False
        self.odometer += math.hypot(self.x - ox, self.y - oy)
        return False

    def lidar(self, segments, noise: float = 0.0, rng: random.Random | None = None) -> list[float]:
        """16 лучей на 360°, относительно курса робота, в метрах."""
        out = []
        for i in range(LIDAR_BEAMS):
            a = self.theta + 2.0 * math.pi * i / LIDAR_BEAMS
            dx, dy = math.cos(a), math.sin(a)
            best = LIDAR_RANGE_M
            for seg in segments:
                t = _ray_segment(self.x, self.y, dx, dy, seg)
                if t is not None and t < best:
                    best = t
            if noise and rng is not None:
                best = max(0.0, best + rng.gauss(0.0, noise))
            out.append(round(min(best, LIDAR_RANGE_M), 3))
        return out
