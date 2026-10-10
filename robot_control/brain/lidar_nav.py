"""Езда по лидару без заранее известной карты (чистый Python, без ROS).

Стек:
  1. Одометрия: робот знает только свои приращения пути и курса, они с шумом.
  2. Локализация: сопоставление скана лидара с картой (поиск по сетке поз
     вокруг одометрической оценки, штраф за отход от неё).
  3. Картография: карта занятости (log-odds) по лучам лидара, обновляется
     в оценённой позе.
  4. Навигация: Hybrid A* по текущей карте (неизвестное считаем свободным —
     оптимистичный план), трекинг pure pursuit с задним ходом, перепланирование
     при отходе от плана и откат после касания.

Физика и лидар — те же, что в world.py. Робот физически двигается по истинной
позе, а управление и план используют только оценённую позу.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field

from . import hybrid as H
from . import planner as P
from .controller import PathTracker, PurePursuit
from .missions import GOAL_RADIUS_M, STUCK_S, STUCK_V, BACK_STEPS, BACK_V, REPLAN_DEV_M, hard_layout
from .world import (ARENA_H, ARENA_W, LIDAR_BEAMS, LIDAR_RANGE_M, ROBOT_RADIUS_M, START_POSE,
                    Robot, arena_segments, shelves_to_segments)

DT = 0.05
SCAN_EVERY = 2            # скан каждые 2 шага (10 Hz при DT 0.05)
CELL = P.CELL_M

L_OCC = 0.9               # приращение log-odds при попадании
L_FREE = -0.5             # при прохождении луча
L_CLAMP = 3.0

# одометрия (шум на каждом шаге)
ODO_DIST_NOISE = 0.02     # относительный шум пути
ODO_HEAD_NOISE = 0.003    # рад на шаг

# сопоставление скана
MATCH_DXY = [-0.12, -0.09, -0.06, -0.03, 0.0, 0.03, 0.06, 0.09, 0.12]
MATCH_DTH = [math.radians(d) for d in (-4, -2, 0, 2, 4)]
PRIOR_XY_W = 300.0        # штраф за отход от одометрии по позиции (на м²)
PRIOR_TH_W = 2000.0       # штраф за отход от одометрии по курсу (на рад²)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class OccMap:
    """Карта занятости. Клетки — как у планировщика (CELL_M)."""

    def __init__(self, width: float = ARENA_W, height: float = ARENA_H):
        self.width, self.height = width, height
        self.nx = int(round(width / CELL))
        self.ny = int(round(height / CELL))
        self.l = [0.0] * (self.nx * self.ny)
        self.occ: set[int] = set()        # клетки с log-odds > 0
        self._near: set[int] | None = None  # окрестность занятых (для сопоставления)

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (min(self.nx - 1, max(0, int(x / CELL))), min(self.ny - 1, max(0, int(y / CELL))))

    def known(self, i: int, j: int) -> bool:
        return self.l[j * self.nx + i] != 0.0

    def _add(self, i: int, j: int, dl: float):
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return
        k = j * self.nx + i
        v = max(-L_CLAMP, min(L_CLAMP, self.l[k] + dl))
        was = self.l[k] > 0.0
        self.l[k] = v
        now = v > 0.0
        if now != was:
            if now:
                self.occ.add(k)
            else:
                self.occ.discard(k)
            self._near = None

    @staticmethod
    def _line(i0, j0, i1, j1):
        """Клетки отрезка от (i0,j0) до (i1,j1) включительно (Bresenham)."""
        out = []
        dx, dy = abs(i1 - i0), -abs(j1 - j0)
        sx = 1 if i0 < i1 else -1
        sy = 1 if j0 < j1 else -1
        err = dx + dy
        i, j = i0, j0
        while True:
            out.append((i, j))
            if i == i1 and j == j1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                i += sx
            if e2 <= dx:
                err += dx
                j += sy
        return out

    def integrate(self, x: float, y: float, theta: float, ranges) -> None:
        i0, j0 = self.cell(x, y)
        for b, r in enumerate(ranges):
            a = theta + 2.0 * math.pi * b / LIDAR_BEAMS
            hit = r < LIDAR_RANGE_M - 1e-6
            rr = r if hit else LIDAR_RANGE_M
            i1, j1 = self.cell(x + rr * math.cos(a), y + rr * math.sin(a))
            cells = self._line(i0, j0, i1, j1)
            last = len(cells) - 1
            for n, (i, j) in enumerate(cells):
                if n == last and hit:
                    self._add(i, j, L_OCC)
                elif n < last or not hit:
                    self._add(i, j, L_FREE)

    def dist_field(self, max_d: int = 4) -> dict[int, int]:
        """Расстояние в клетках до ближайшего занятого (до max_d); кэш до изменения карты."""
        if self._near is None:
            dist = {k: 0 for k in self.occ}
            frontier = list(self.occ)
            for d in range(1, max_d + 1):
                nxt = []
                for k in frontier:
                    i, j = k % self.nx, k // self.nx
                    for di in (-1, 0, 1):
                        for dj in (-1, 0, 1):
                            ii, jj = i + di, j + dj
                            if 0 <= ii < self.nx and 0 <= jj < self.ny:
                                kk = jj * self.nx + ii
                                if kk not in dist:
                                    dist[kk] = d
                                    nxt.append(kk)
                frontier = nxt
            self._near = dist
        return self._near

    def to_grid(self, inflate: float, borders: bool = True) -> P.Grid:
        """Сетка для планировщика: занятые клетки раздуты на inflate; границы склада известны."""
        blocked = bytearray(self.nx * self.ny)
        r_cells = int(math.ceil(inflate / CELL))
        disc = [(di, dj) for di in range(-r_cells, r_cells + 1) for dj in range(-r_cells, r_cells + 1)
                if math.hypot(di, dj) * CELL <= inflate]
        for k in self.occ:
            i, j = k % self.nx, k // self.nx
            for di, dj in disc:
                ii, jj = i + di, j + dj
                if 0 <= ii < self.nx and 0 <= jj < self.ny:
                    blocked[jj * self.nx + ii] = 1
        if borders:
            for j in range(self.ny):
                for i in range(self.nx):
                    x, y = (i + 0.5) * CELL, (j + 0.5) * CELL
                    if x < inflate or y < inflate or x > self.width - inflate or y > self.height - inflate:
                        blocked[j * self.nx + i] = 1
        return P.Grid(self.width, self.height, self.nx, self.ny, blocked)


class ScanMatcher:
    """Локализация: поиск позы вокруг одометрической оценки, максимум совпадения скана с картой."""

    def match(self, mp: OccMap, prior, ranges):
        px, py, pth = prior
        dist = mp.dist_field()
        if not dist:
            return prior
        hits = [(b, r) for b, r in enumerate(ranges) if r < LIDAR_RANGE_M - 1e-6]
        if not hits:
            return prior
        best, best_s = prior, -math.inf
        for dth in MATCH_DTH:
            th = pth + dth
            for dx in MATCH_DXY:
                for dy in MATCH_DXY:
                    x, y = px + dx, py + dy
                    s = 0.0
                    for b, r in hits:
                        a = th + 2.0 * math.pi * b / LIDAR_BEAMS
                        i, j = mp.cell(x + r * math.cos(a), y + r * math.sin(a))
                        d = dist.get(j * mp.nx + i)
                        if d is not None:
                            s += (4 - d) / 4.0          # 1 — в занятой клетке, 0 — дальше 4 клеток
                    s -= PRIOR_XY_W * (dx * dx + dy * dy) + PRIOR_TH_W * dth * dth
                    if s > best_s:
                        best_s, best = s, (x, y, th)
        return best


class Odometry:
    """Шумная одометрия: приращение в собственной системе координат робота."""

    def __init__(self, rng: random.Random):
        self.rng = rng
        self.prev = None

    def delta(self, x, y, theta):
        """Возвращает (ds, dth) — путь вдоль курса и поворот, с шумом."""
        if self.prev is None:
            self.prev = (x, y, theta)
            return 0.0, 0.0
        px, py, pth = self.prev
        self.prev = (x, y, theta)
        dx, dy = x - px, y - py
        ds = dx * math.cos(pth) + dy * math.sin(pth)          # знак — по направлению движения
        dth = _wrap(theta - pth)
        ds *= 1.0 + self.rng.gauss(0.0, ODO_DIST_NOISE)
        dth += self.rng.gauss(0.0, ODO_HEAD_NOISE)
        return ds, dth


@dataclass
class NavResult:
    completed: bool = False
    reached: int = 0
    touches: int = 0
    time_s: float = 0.0
    travelled_m: float = 0.0
    replans: int = 0
    plan_failures: int = 0
    stuck_events: int = 0
    loc_err_mean: float = 0.0
    loc_err_max: float = 0.0
    map_cells_known: int = 0
    plan_ms_max: float = 0.0
    failures: list = field(default_factory=list)


def run_lidar_mission(seed: int, n_waypoints: int = 8, lidar_noise: float = 0.02,
                      time_factor: float = 3.0, use_true_pose: bool = False) -> NavResult:
    """Миссия по неизвестной карте: мапим на ходу, едем по точкам из hard_layout(seed)."""
    shelves, waypoints = hard_layout(seed, n_waypoints)
    segs = arena_segments() + shelves_to_segments(shelves)
    rng = random.Random(seed * 7919 + 1)
    robot = Robot(*START_POSE)
    mp = OccMap()
    matcher = ScanMatcher()
    odo = Odometry(rng)
    ctrl = PurePursuit()
    res = NavResult()

    est = [START_POSE[0], START_POSE[1], START_POSE[2]]     # оценённая поза
    # оценка длины — по истинной карте (только для лимита времени)
    true_grid = P.build_grid(ARENA_W, ARENA_H, shelves, inflate=H.COLLISION_INFLATE_M)
    pos = START_POSE[:2]
    planned = 0.0
    for wp in waypoints:
        cp = P.plan(true_grid, pos, wp)
        planned += P.path_length(cp) if cp else 0.0
        pos = wp
    limit = time_factor * planned / 0.4 + 30.0

    t = 0.0
    step = 0
    idx = 0
    path = None
    tracker = None
    back = 0
    low_since = None
    plan_ms = []
    errs = []
    prev_xy = (robot.x, robot.y)

    def sense_and_update():
        nonlocal est
        ranges = robot.lidar(segs, noise=lidar_noise, rng=rng)
        if use_true_pose:
            est = [robot.x, robot.y, robot.theta]
        else:
            ds, dth = odo.delta(robot.x, robot.y, robot.theta)
            prior = (est[0] + ds * math.cos(est[2]), est[1] + ds * math.sin(est[2]), _wrap(est[2] + dth))
            if mp.occ:
                mx, my, mth = matcher.match(mp, prior, ranges)
                est = [mx, my, _wrap(mth)]
            else:
                est = [prior[0], prior[1], prior[2]]
        mp.integrate(est[0], est[1], est[2], ranges)

    sense_and_update()
    while idx < len(waypoints) and t < limit:
        gx, gy = waypoints[idx]
        d = math.hypot(gx - est[0], gy - est[1])
        if math.hypot(gx - robot.x, gy - robot.y) < GOAL_RADIUS_M:
            res.reached += 1
            idx += 1
            path = None
            low_since = None
            continue
        if back > 0:
            back -= 1
            robot.step(BACK_V, 0.0, DT, segs)
            t += DT
            step += 1
            if back == 0:
                path = None
            if step % SCAN_EVERY == 0:
                sense_and_update()
            continue
        if path is not None:
            dev = min(math.hypot(p.x - est[0], p.y - est[1]) for p in path)
            if dev > REPLAN_DEV_M:
                path = None
        if path is None:
            tp = time.perf_counter()
            grid = mp.to_grid(H.COLLISION_INFLATE_M)
            pose = H.Pose(est[0], est[1], est[2], 1)
            path = H.plan_hybrid(grid, pose, (gx, gy))
            if path is None:
                res.failures.append((round(t, 1), idx))
                grid_t = mp.to_grid(ROBOT_RADIUS_M + 0.01)
                ci, cj = grid_t.cell_of(est[0], est[1])
                free = P._nearest_free(grid_t, ci, cj)
                ex, ey = grid_t.center(*free) if free else (gx, gy)
                path = H.plan_hybrid(grid_t, pose, (ex, ey))
            plan_ms.append((time.perf_counter() - tp) * 1000.0)
            res.replans += 1
            if path is None:
                res.plan_failures += 1
                back = BACK_STEPS
                res.stuck_events += 1
                continue
            tracker = PathTracker(path)
        tgt, direction, rem = tracker.target(est[0], est[1])
        out = ctrl.command(est[0], est[1], est[2], tgt, rem, direction)
        if abs(robot.v) < STUCK_V:
            low_since = t if low_since is None else low_since
            if t - low_since >= STUCK_S:
                res.stuck_events += 1
                back = BACK_STEPS
                low_since = None
                continue
        else:
            low_since = None
        if robot.step(out.v, out.steer_deg, DT, segs):
            back = BACK_STEPS
            path = None
        t += DT
        step += 1
        if step % SCAN_EVERY == 0:
            sense_and_update()
            errs.append(math.hypot(est[0] - robot.x, est[1] - robot.y))

    res.time_s = round(t, 2)
    res.completed = idx >= len(waypoints)
    res.touches = robot.collisions
    res.travelled_m = round(robot.odometer, 2)
    res.loc_err_mean = round(sum(errs) / len(errs), 3) if errs else 0.0
    res.loc_err_max = round(max(errs), 3) if errs else 0.0
    res.map_cells_known = sum(1 for v in mp.l if v != 0.0)
    res.plan_ms_max = round(max(plan_ms) if plan_ms else 0.0, 1)
    return res
