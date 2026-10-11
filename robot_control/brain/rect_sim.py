"""Симуляция с прямоугольным корпусом 9:16 и хитбоксом. Робот едет только по лидару.

Корпус: прямоугольник 0.60 × 0.75 м (ширина × длина), центр в точке (x, y).
Столкновение — пересечение прямоугольника корпуса со стенами и стеллажами (отрезки).
Лидар, одометрия, карта, локализация (частичный фильтр), планирование (Hybrid A*) и трекинг —
те же модули, что в lidar_nav.py. Ничего из этого не читает истинную позу, кроме физики
и записи кадров для визуализации.

Результат миссии — список кадров (для render_sim.py) и итоговые метрики.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from . import hybrid as H
from . import planner as P
from .controller import PathTracker, PurePursuit
from .lidar_nav import OccMap, Odometry, ParticleLocalizer, _wrap
from .missions import BACK_STEPS, BACK_V, GOAL_RADIUS_M, REPLAN_DEV_M, STUCK_S, STUCK_V
import random as _random

from .world import (LIDAR_BEAMS, LIDAR_RANGE_M, START_POSE, STEER_LIMIT_DEG, STEER_RATE_DEG_S,
                    V_MAX, WHEELBASE_M, Robot, rect_segments, shelves_to_segments)

BODY_W = 0.60             # м, ширина корпуса
BODY_L = 0.75             # м, длина корпуса
HALF_W = BODY_W / 2
HALF_L = BODY_L / 2
CIRCUMRADIUS = math.hypot(HALF_W, HALF_L)     # радиус описанной окружности корпуса
BIG_W, BIG_H = 24.0, 14.0                       # большая карта склада, м
# Безопасность: центр не ближе описанной окружности корпуса (+ допуск) к препятствию —
# корпус тогда заведомо не касается стены. Это жёсткое ограничение планировщика.
SAFE_INFLATE_M = CIRCUMRADIUS + 0.03
# Зазор 30 см — мягкое требование: близость к стене штрафуется в стоимости дуги.
CLEAR_TARGET_M = 0.30
PROX_COST_K = 0.6                              # вес штрафа за близость
PLAN_INFLATE_M = SAFE_INFLATE_M
PLAN_TIGHT_M = SAFE_INFLATE_M
V_CRUISE = 0.4                                 # м/с, аккуратная езда
STEER_SMOOTH = 0.5                             # доля новой команды руля за шаг (мягче поворот)
DT = 0.05
SCAN_EVERY = 2
LIDAR_NOISE = 0.02


def body_corners(x: float, y: float, th: float):
    c, s = math.cos(th), math.sin(th)
    out = []
    for lx, ly in ((HALF_L, HALF_W), (HALF_L, -HALF_W), (-HALF_L, -HALF_W), (-HALF_L, HALF_W)):
        out.append((x + lx * c - ly * s, y + lx * s + ly * c))
    return out


def _orient(ax, ay, bx, by, cx, cy):
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def _seg_cross(p1, p2, q1, q2) -> bool:
    d1 = _orient(*q1, *q2, *p1)
    d2 = _orient(*q1, *q2, *p2)
    d3 = _orient(*p1, *p2, *q1)
    d4 = _orient(*p1, *p2, *q2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _point_in_body(px, py, x, y, th) -> bool:
    c, s = math.cos(th), math.sin(th)
    dx, dy = px - x, py - y
    lx = dx * c + dy * s
    ly = -dx * s + dy * c
    return abs(lx) <= HALF_L and abs(ly) <= HALF_W


def body_touches(x: float, y: float, th: float, segs) -> list:
    """Список отрезков, которые касаются (пересекают) корпус. Пусто — столкновения нет."""
    corners = body_corners(x, y, th)
    edges = [(corners[k], corners[(k + 1) % 4]) for k in range(4)]
    hit = []
    for seg in segs:
        a = (seg[0], seg[1])
        b = (seg[2], seg[3])
        touch = _point_in_body(a[0], a[1], x, y, th) or _point_in_body(b[0], b[1], x, y, th)
        if not touch:
            for p, q in edges:
                if _seg_cross(p, q, a, b):
                    touch = True
                    break
        if touch:
            hit.append(seg)
    return hit


@dataclass
class RectRobot:
    x: float
    y: float
    theta: float
    v: float = 0.0
    steer: float = 0.0
    odometer: float = 0.0
    collisions: int = 0
    touching: bool = False

    def step(self, v_cmd: float, steer_cmd_deg: float, dt: float, segs) -> bool:
        v_cmd = max(-0.25, min(V_MAX, v_cmd))
        steer_cmd_deg = max(-STEER_LIMIT_DEG, min(STEER_LIMIT_DEG, steer_cmd_deg))
        rate = STEER_RATE_DEG_S * dt
        self.steer += max(-rate, min(rate, steer_cmd_deg - self.steer))
        self.v += max(-dt, min(dt, v_cmd - self.v))
        delta = math.radians(self.steer)
        ox, oy, oth = self.x, self.y, self.theta
        self.theta = _wrap(self.theta + self.v / WHEELBASE_M * math.tan(delta) * dt)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        if body_touches(self.x, self.y, self.theta, segs):
            self.x, self.y, self.theta = ox, oy, oth
            self.v = 0.0
            if not self.touching:
                self.collisions += 1
            self.touching = True
            return True
        self.touching = False
        self.odometer += math.hypot(self.x - ox, self.y - oy)
        return False

    def lidar(self, segs, rng):
        ranges = Robot(self.x, self.y, self.theta).lidar(segs, noise=LIDAR_NOISE, rng=rng)
        return ranges


@dataclass
class RectResult:
    seed: int
    completed: bool = False
    reached: int = 0
    touches: int = 0
    time_s: float = 0.0
    travelled_m: float = 0.0
    replans: int = 0
    plan_failures: int = 0
    loc_err_mean: float = 0.0
    loc_err_max: float = 0.0
    frames: list = field(default_factory=list)
    paths: dict = field(default_factory=dict)
    waypoints: list = field(default_factory=list)
    shelves: list = field(default_factory=list)
    map_occ: list = field(default_factory=list)
    collision_events: list = field(default_factory=list)
    arena: list = field(default_factory=list)


def big_layout(seed: int, n_waypoints: int = 10):
    """Большой склад 24×14 м: колонны стеллажей с проходами и поперечные стеллажи-перегородки.
    Точки маршрута достижимы из предыдущей по сетке с запасом планировщика."""
    rng = _random.Random(seed)
    shelves = []
    for k in range(6):
        x0 = 2.5 + k * 3.6
        y = 1.2 + rng.uniform(0.0, 0.6)
        while y < BIG_H - 2.2:
            if rng.random() < 0.8:                       # 20% проходов поперёк
                h = rng.uniform(1.8, 2.8)
                w = rng.uniform(0.9, 1.3)
                xx = x0 + rng.uniform(-0.25, 0.25)
                shelves.append((xx, y, xx + w, min(BIG_H - 1.0, y + h)))
            y += rng.uniform(3.2, 3.8) if rng.random() < 0.5 else rng.uniform(2.0, 2.6)
    for _ in range(4):                                   # поперечные перегородки
        x0 = rng.uniform(4.0, BIG_W - 6.0)
        y0 = rng.uniform(1.5, BIG_H - 3.0)
        shelves.append((x0, y0, x0 + rng.uniform(1.5, 2.5), y0 + 0.9))
    shelves = [(max(1.0, a), max(1.0, b), min(BIG_W - 1.0, c), min(BIG_H - 1.0, d))
               for a, b, c, d in shelves if c - a > 0.5 and d - b > 0.5]
    grid = P.build_grid(BIG_W, BIG_H, shelves, inflate=PLAN_INFLATE_M)
    waypoints = []
    prev = START_POSE[:2]
    tries = 0
    while len(waypoints) < n_waypoints and tries < 20000:
        tries += 1
        cand = (rng.uniform(1.0, BIG_W - 1.0), rng.uniform(1.0, BIG_H - 1.0))
        gi, gj = grid.cell_of(*cand)
        if not grid.free(gi, gj) or math.hypot(cand[0] - prev[0], cand[1] - prev[1]) < 4.0:
            continue
        if P.plan(grid, prev, cand) is None:
            continue
        waypoints.append(cand)
        prev = cand
    return shelves, waypoints


def plan_explore(mp: OccMap, pose: H.Pose, goal, robot_xy):
    """План по известной карте: неизвестное — занято. Если пути к цели нет, идём к границе
    известного (клетка рядом с неизвестным), ближайшей к цели. Так робот исследует склад,
    не врезаясь в ещё не замеченные препятствия."""
    grid = mp.to_grid(SAFE_INFLATE_M, unknown_blocked=True)
    dist = mp.dist_field(10)

    def prox(x, y):
        # расстояние центра до препятствия; штраф растёт, когда зазор корпуса < 30 см
        dd = dist.get(mp.cell(x, y)[1] * mp.nx + mp.cell(x, y)[0])
        dm = (dd * 0.1) if dd is not None else 1.0
        gap = (CIRCUMRADIUS + CLEAR_TARGET_M) - dm
        return PROX_COST_K * max(0.0, gap) / CLEAR_TARGET_M

    path = H.plan_hybrid(grid, pose, goal, extra_cost=prox)
    if path is not None:
        return path
    # граница известного: свободная клетка с соседом неизвестным
    fr = []
    for j in range(grid.ny):
        for i in range(grid.nx):
            k = j * grid.nx + i
            if grid.blocked[k] or mp.l[k] == 0.0:
                continue
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ii, jj = i + di, j + dj
                if 0 <= ii < mp.nx and 0 <= jj < mp.ny and mp.l[jj * mp.nx + ii] == 0.0:
                    cx, cy = grid.center(i, j)
                    fr.append((math.hypot(cx - goal[0], cy - goal[1]) + 0.3 * math.hypot(cx - robot_xy[0], cy - robot_xy[1]), cx, cy))
                    break
    fr.sort()
    for _, cx, cy in fr[:6]:
        path = H.plan_hybrid(grid, pose, (cx, cy), extra_cost=prox)
        if path is not None:
            return path
    return None


def run_rect_mission(seed: int, n_waypoints: int = 10, time_factor: float = 3.0,
                     use_true_pose: bool = False) -> RectResult:
    shelves, waypoints = big_layout(seed, n_waypoints)
    segs = rect_segments(0.0, 0.0, BIG_W, BIG_H) + shelves_to_segments(shelves)
    rng = random.Random(seed * 7919 + 3)
    robot = RectRobot(*START_POSE)
    mp = OccMap(BIG_W, BIG_H)
    odo = Odometry(rng)
    pf = ParticleLocalizer(START_POSE, rng)
    ctrl = PurePursuit()
    res = RectResult(seed=seed, waypoints=[list(w) for w in waypoints], shelves=[list(s) for s in shelves],
                     arena=[BIG_W, BIG_H])

    planned = 0.0
    true_grid = P.build_grid(BIG_W, BIG_H, shelves, inflate=PLAN_INFLATE_M)
    pos = START_POSE[:2]
    for wp in waypoints:
        cp = P.plan(true_grid, pos, wp)
        planned += P.path_length(cp) if cp else 0.0
        pos = wp
    limit = time_factor * planned / 0.4 + 30.0

    est = [START_POSE[0], START_POSE[1], START_POSE[2]]
    t = 0.0
    step = 0
    idx = 0
    path = None
    path_id = 0
    tracker = None
    back = 0
    low_since = None
    errs = []
    prev_steer = 0.0

    def sense():
        nonlocal est
        ranges = robot.lidar(segs, rng)
        ds, dth = odo.delta(robot.x, robot.y, robot.theta)
        pf.motion(ds, dth)
        if mp.occ:
            pf.correct(mp, ranges)
        est = list(pf.estimate())
        if use_true_pose:                       # диагностика: идеальная локализация
            est = [robot.x, robot.y, robot.theta]
        mp.integrate(est[0], est[1], est[2], ranges)
        return ranges

    ranges = sense()
    while idx < len(waypoints) and t < limit:
        gx, gy = waypoints[idx]
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
            continue
        if path is not None and min(math.hypot(p.x - est[0], p.y - est[1]) for p in path) > REPLAN_DEV_M:
            path = None
        if path is None:
            pose = H.Pose(est[0], est[1], est[2], 1)
            path = plan_explore(mp, pose, (gx, gy), (robot.x, robot.y))
            if path is None:
                res.plan_failures += 1
                grid_t = mp.to_grid(PLAN_TIGHT_M)
                ci, cj = grid_t.cell_of(est[0], est[1])
                free = P._nearest_free(grid_t, ci, cj)
                ex, ey = grid_t.center(*free) if free else (gx, gy)
                path = H.plan_hybrid(grid_t, pose, (ex, ey))
            res.replans += 1
            if path is None:
                back = BACK_STEPS
                continue
            path_id += 1
            res.paths[path_id] = [[round(p.x, 3), round(p.y, 3)] for p in path]
            tracker = PathTracker(path)
            res.frames.append({"ev": "plan", "t": round(t, 2), "path": path_id})
        tgt, direction, rem = tracker.target(est[0], est[1])
        out = ctrl.command(est[0], est[1], est[2], tgt, rem, direction)
        if abs(robot.v) < STUCK_V:
            low_since = t if low_since is None else low_since
            if t - low_since >= STUCK_S:
                back = BACK_STEPS
                low_since = None
                continue
        else:
            low_since = None
        steer_cmd = prev_steer + STEER_SMOOTH * (out.steer_deg - prev_steer)
        prev_steer = steer_cmd
        ok_move = not robot.step(min(out.v, V_CRUISE), steer_cmd, DT, segs)
        if not ok_move:
            back = BACK_STEPS
            path = None
            res.collision_events.append([round(robot.x, 3), round(robot.y, 3), round(robot.theta, 3), round(t, 2)])
        t += DT
        step += 1
        if step % SCAN_EVERY == 0:
            ranges = sense()
            errs.append(math.hypot(est[0] - robot.x, est[1] - robot.y))
            res.frames.append({
                "t": round(t, 2),
                "tx": round(robot.x, 3), "ty": round(robot.y, 3), "tth": round(robot.theta, 3),
                "ex": round(est[0], 3), "ey": round(est[1], 3), "eth": round(est[2], 3),
                "ok": ok_move, "path": path_id if path is not None else 0,
                "lidar": ranges, "wp": idx,
            })

    res.time_s = round(t, 2)
    res.completed = idx >= len(waypoints)
    res.touches = robot.collisions
    res.travelled_m = round(robot.odometer, 2)
    res.loc_err_mean = round(sum(errs) / len(errs), 3) if errs else 0.0
    res.loc_err_max = round(max(errs), 3) if errs else 0.0
    res.map_occ = [[k % mp.nx, k // mp.nx] for k in mp.occ]
    return res
