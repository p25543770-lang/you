"""Один эпизод миссии: робот проходит точки маршрута, ИИ рулит.

Общий код для обучения (train.py) и проверки; в рантайме то же самое
делают узлы на шине ROS-подобного графа (runtime.py).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .network import Brain, features
from .world import (
    BASE_SHELVES, BASE_WAYPOINTS, START_POSE, Robot, arena_segments,
    random_layout, shelves_to_segments,
)

GOAL_RADIUS_M = 0.6
CONTROL_DT = 0.05          # 20 Гц, как узел BrainNode
SENSOR_NOISE_M = 0.02


@dataclass
class EpisodeResult:
    reached: int = 0
    collisions: int = 0
    final_dist: float = 0.0
    progress_m: float = 0.0
    near_steps: int = 0        # шаги с препятствием ближе 0.5 м
    score: float = 0.0
    time_s: float = 0.0
    path: list = field(default_factory=list)


def layout(seed: int | None):
    """seed None → базовый склад; иначе случайная раскладка с этим seed."""
    if seed is None:
        shelves, waypoints = BASE_SHELVES, BASE_WAYPOINTS
    else:
        shelves, waypoints = random_layout(seed)
    return shelves, waypoints, arena_segments() + shelves_to_segments(shelves)


def run_episode(brain: Brain, seed: int | None, duration_s: float = 45.0,
                noise_seed: int = 0, keep_path: bool = False) -> EpisodeResult:
    shelves, waypoints, segs = layout(seed)
    rng = random.Random(noise_seed)
    robot = Robot(*START_POSE)
    brain.reset()
    res = EpisodeResult()
    idx = 0
    progress_m = 0.0           # сумма продвижения к текущей точке, м
    prev_dist = None
    steps = int(duration_s / CONTROL_DT)
    for step in range(steps):
        if idx >= len(waypoints):
            break
        gx, gy = waypoints[idx]
        dx, dy = gx - robot.x, gy - robot.y
        dist = math.hypot(dx, dy)
        if dist < GOAL_RADIUS_M:
            res.reached += 1
            idx += 1
            prev_dist = None
            continue
        if prev_dist is not None:
            progress_m += max(0.0, prev_dist - dist)   # вознаграждаем только приближение
        prev_dist = dist
        rel = math.atan2(math.sin(math.atan2(dy, dx) - robot.theta),
                         math.cos(math.atan2(dy, dx) - robot.theta))
        lidar = robot.lidar(segs, SENSOR_NOISE_M, rng)
        if min(lidar) < 0.5:
            res.near_steps += 1
        v, steer = brain.step(features(lidar, rel, dist, robot.v))
        robot.step(v, steer, CONTROL_DT, segs)
        res.time_s += CONTROL_DT
        if keep_path and step % 4 == 0:
            res.path.append((round(robot.x, 2), round(robot.y, 2)))
    res.collisions = robot.collisions
    if idx < len(waypoints):
        gx, gy = waypoints[idx]
        res.final_dist = math.hypot(gx - robot.x, gy - robot.y)
    res.progress_m = progress_m
    # плотная награда: за каждый метр к цели, за точку — бонус, за касание — штраф
    res.score = (100.0 * res.reached + 10.0 * progress_m
                 - 20.0 * res.collisions - 0.02 * res.near_steps)
    return res
