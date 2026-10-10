"""Сложные и длинные миссии: генератор раскладок и полный прогон миссии.

Генератор строит плотный склад (сетка стеллажей с перекрытиями проходов) и
выбирает несколько точек маршрута, каждая из которых достижима планировщиком
из предыдущей. Прогон миссии — Hybrid A* (hybrid.py) + pure pursuit с задним ходом (controller.py);
физика та же, что в симуляции (world.py).
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field

from . import hybrid as H
from . import planner as P
from .controller import PathTracker, PurePursuit
from .world import ARENA_H, ARENA_W, ROBOT_RADIUS_M, START_POSE, V_MAX, Robot, arena_segments, shelves_to_segments

GOAL_RADIUS_M = 0.6
DT = 0.05
STUCK_V = 0.05
STUCK_S = 3.0
REPLAN_DEV_M = 0.25      # отклонение от плана, при котором перепланируем
BACK_STEPS = 20           # откат после касания: 20 × 0.05 с = 1 с
BACK_V = -0.15            # м/с при откате


def hard_layout(seed: int, n_waypoints: int = 8):
    """Плотная раскладка и n_waypoints точек, каждая достижима из предыдущей."""
    rng = random.Random(seed)
    shelves = []
    for cx in (2.6, 4.9, 7.2, 9.5, 11.8):
        for y0 in (1.6, 4.4, 6.9):
            if rng.random() < 0.25:
                continue                       # проход поперёк
            w = rng.uniform(0.8, 1.2)
            h = rng.uniform(1.4, 2.0)
            x0 = cx + rng.uniform(-0.3, 0.3) - w / 2
            y = y0 + rng.uniform(-0.25, 0.25)
            x0 = max(1.2, min(ARENA_W - 1.2 - w, x0))
            y = max(1.2, min(ARENA_H - 1.2 - h, y))
            shelves.append((x0, y, x0 + w, y + h))
    grid = P.build_grid(ARENA_W, ARENA_H, shelves, inflate=H.COLLISION_INFLATE_M)
    waypoints = []
    prev = START_POSE[:2]
    tries = 0
    while len(waypoints) < n_waypoints and tries < 5000:
        tries += 1
        cand = (rng.uniform(0.8, ARENA_W - 0.8), rng.uniform(0.8, ARENA_H - 0.8))
        gi, gj = grid.cell_of(*cand)
        if not grid.free(gi, gj):
            continue
        if math.hypot(cand[0] - prev[0], cand[1] - prev[1]) < 3.0:
            continue
        if P.plan(grid, prev, cand) is None:  # грубая проверка; точная — в run_mission
            continue
        waypoints.append(cand)
        prev = cand
    return shelves, waypoints


@dataclass
class MissionResult:
    completed: bool = False
    reached: int = 0
    collisions: int = 0
    time_s: float = 0.0
    planned_m: float = 0.0
    travelled_m: float = 0.0
    replans: int = 0
    stuck_events: int = 0
    plan_ms_max: float = 0.0
    failures: list = field(default_factory=list)


def run_mission(seed: int, n_waypoints: int = 8, controller=None, time_factor: float = 3.0) -> MissionResult:
    """Полный прогон миссии. Лимит времени — time_factor × (длина пути / 0.4 м/с) + 30 с.

    Цикл: план Hybrid A* до точки → трекинг (PathTracker) → перепланирование при
    отклонении > REPLAN_DEV_M → откат назад после касания или застревания.
    """
    shelves, waypoints = hard_layout(seed, n_waypoints)
    segs = arena_segments() + shelves_to_segments(shelves)
    grid = P.build_grid(ARENA_W, ARENA_H, shelves, inflate=H.COLLISION_INFLATE_M)
    # запасная сетка: только радиус робота — нужна, когда робот прижат к стенке
    # и его клетка попадает в раздутую зону основной сетки
    grid_tight = P.build_grid(ARENA_W, ARENA_H, shelves, inflate=ROBOT_RADIUS_M + 0.01)
    ctrl = controller or PurePursuit()
    robot = Robot(*START_POSE)
    res = MissionResult()

    pos = START_POSE[:2]
    for wp in waypoints:
        cp = P.plan(grid, pos, wp)          # оценка длины: дешёвый A* по сетке
        res.planned_m += P.path_length(cp) if cp else 0.0
        pos = wp
    limit = time_factor * res.planned_m / 0.4 + 30.0

    idx = 0
    t = 0.0
    path = None
    tracker = None
    back = 0
    low_since = None
    plan_ms = []
    while idx < len(waypoints) and t < limit:
        gx, gy = waypoints[idx]
        d = math.hypot(gx - robot.x, gy - robot.y)
        if d < GOAL_RADIUS_M:
            res.reached += 1
            idx += 1
            path = None
            low_since = None
            continue
        if back > 0:
            back -= 1
            robot.step(BACK_V, 0.0, DT, segs)
            t += DT
            if back == 0:
                path = None
            continue
        if path is not None and min(math.hypot(p.x - robot.x, p.y - robot.y) for p in path) > REPLAN_DEV_M:
            path = None
        if path is None:
            tp = time.perf_counter()
            pose = H.Pose(robot.x, robot.y, robot.theta, 1)
            path = H.plan_hybrid(grid, pose, (gx, gy))
            if path is None:
                # робот прижат к стенке/стеллажу (его клетка в раздутой зоне): сначала
                # выходим на ближайшую свободную клетку запасной сетки, затем — цель
                res.failures.append((round(t, 1), idx))
                ci, cj = grid.cell_of(robot.x, robot.y)
                free = P._nearest_free(grid, ci, cj)
                ex, ey = grid.center(*free) if free else (gx, gy)
                path = H.plan_hybrid(grid_tight, pose, (ex, ey))
            plan_ms.append((time.perf_counter() - tp) * 1000.0)
            res.replans += 1
            if path is None:
                back = BACK_STEPS
                res.stuck_events += 1
                continue
            tracker = PathTracker(path)
        target, direction, rem = tracker.target(robot.x, robot.y)
        out = ctrl.command(robot.x, robot.y, robot.theta, target, rem, direction)
        # страховка: почти стоим слишком долго — откат и перепланирование
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
            back = BACK_STEPS          # касание: откат, затем перепланирование
        t += DT
    res.time_s = round(t, 2)
    res.completed = idx >= len(waypoints)
    res.collisions = robot.collisions
    res.travelled_m = round(robot.odometer, 2)
    res.plan_ms_max = round(max(plan_ms) if plan_ms else 0.0, 2)
    return res
