"""Регулятор движения по плану: pure pursuit с ограничением скорости по кривизне.

Почему не нейросеть: планировщик A* даёт проходимый путь, а задача контроллера —
отработать его стабильно на любой длине. Pure pursuit детерминирован, его
поведение предсказуемо и не зависит от весов. Нейросеть остаётся в рантайме
как опция (BRAIN_CONTROLLER=neural), но основной режим — этот регулятор.

Законы:
  * рулевой угол: δ = atan2(2·L·sin α, ℓ), где α — угол на опережающую точку,
    ℓ — расстояние до неё, L — колёсная база;
  * если точка сзади (|α| > 80°): стоим на месте и поворачиваем на максимальный руль;
  * скорость: убывает с ростом |δ| и приближением к цели, не ниже V_MIN.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .world import STEER_LIMIT_DEG, V_MAX, WHEELBASE_M

V_MIN = 0.18          # м/с: минимальный ход, чтобы не застревать
V_TURN = 0.12         # м/с: поворот на месте, когда цель сзади
BEHIND_RAD = math.radians(80.0)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class PursuitOutput:
    v: float
    steer_deg: float


class PurePursuit:
    def __init__(self, wheelbase: float = WHEELBASE_M, v_max: float = V_MAX,
                 steer_limit_deg: float = STEER_LIMIT_DEG):
        self.L = wheelbase
        self.v_max = v_max
        self.steer_limit = steer_limit_deg

    def command(self, x: float, y: float, theta: float, target, goal_dist: float,
                direction: int = 1) -> PursuitOutput:
        """direction=+1 — ехать вперёд, -1 — назад (курс разворачивается на π)."""
        tx, ty = target
        dx, dy = tx - x, ty - y
        ld = max(1e-3, math.hypot(dx, dy))
        heading = theta if direction > 0 else theta + math.pi
        alpha = _wrap(math.atan2(dy, dx) - heading)

        if abs(alpha) > BEHIND_RAD:
            # цель в стороне или сзади относительно курса движения: поворот на месте
            return PursuitOutput(direction * V_TURN, math.copysign(self.steer_limit, alpha) * direction)

        steer = math.degrees(math.atan2(2.0 * self.L * math.sin(alpha), ld))
        steer = max(-self.steer_limit, min(self.steer_limit, steer))
        # скорость: меньше на крутых поворотах и вблизи цели
        turn_factor = max(0.3, math.cos(math.radians(abs(steer)) * 0.9))
        v = self.v_max * turn_factor
        v = min(v, 0.25 + 0.6 * goal_dist)         # плавная остановка у цели
        v = max(V_MIN, min(self.v_max, v))
        return PursuitOutput(direction * v, steer * direction)


class PathTracker:
    """Выбор цели на пути из Hybrid A*: опережение 0.9 м, но не дальше точки смены
    направления (разворота). У разворота робот доезжает до точки, останавливается и
    только затем едет в обратную сторону. Индекс на пути только растёт."""

    LOOKAHEAD_M = 0.9
    CUSP_ARRIVE_M = 0.25
    SEARCH_AHEAD = 12

    def __init__(self, path):
        self.path = list(path)
        self.idx = 0

    def _advance(self, x: float, y: float):
        n = len(self.path)
        best, best_d = self.idx, math.inf
        for k in range(self.idx, min(n, self.idx + self.SEARCH_AHEAD)):
            d = math.hypot(self.path[k].x - x, self.path[k].y - y)
            if d < best_d:
                best, best_d = k, d
        self.idx = best

    def target(self, x: float, y: float):
        """-> ((tx, ty), direction, distance_to_end)."""
        p = self.path
        n = len(p)
        self._advance(x, y)
        if self.idx >= n - 1:
            return (p[-1].x, p[-1].y), p[-1].direction, math.hypot(p[-1].x - x, p[-1].y - y)
        seg_dir = p[self.idx + 1].direction
        k = self.idx
        acc = 0.0
        while k < n - 1 and p[k + 1].direction == seg_dir and acc < self.LOOKAHEAD_M:
            acc += math.hypot(p[k + 1].x - p[k].x, p[k + 1].y - p[k].y)
            k += 1
        # если текущий отрезок закончился разворотом и мы у точки разворота — переходим дальше
        if k == self.idx:
            k = self.idx + 1
        end = p[k]
        remain = math.hypot(p[-1].x - x, p[-1].y - y)
        return (end.x, end.y), end.direction, remain
