"""Hybrid A*: планирование с учётом кинематики робота (минимальный радиус поворота).

Обычный A* по клеткам выдаёт острые углы, которые робот физически не пройдёт
(минимальный радиус ≈ L / tg 30° ≈ 1.04 м). Hybrid A* ищет в пространстве
состояний (x, y, курс): из каждого состояния робот едет дугой ARC_M вперёд или
назад с одним из пяти углов руля. Путь всегда выполним регулятором.

Задний ход нужен: в тесных местах (например, старт между стеллажами) только так
меняется курс на 90°+ (манёвр «назад-вперёд»). Эвристика — точная стоимость пути
на сетке без учёта курса (Dijkstra от цели). Столкновения — по сетке с запасом.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from . import planner as P
from .world import WHEELBASE_M

HEADINGS = 24                      # 15° на бин
HEADING_STEP = 2.0 * math.pi / HEADINGS
ARC_M = 0.25                       # длина одного примитива, м
SUBSTEPS = 5
STEERS_DEG = (-30.0, -15.0, 0.0, 15.0, 30.0)
GOAL_TOL_M = 0.5
COLLISION_INFLATE_M = 0.42         # радиус робота 0.35 + запас 0.07
MAX_EXPANSIONS = 150_000
HEUR_WEIGHT = 1.0               # >1 — взвешенный A*: быстрее, путь немного длиннее
STEER_CHANGE_PENALTY = 0.05        # за смену руля
REVERSE_PENALTY = 1.5              # за движение назад (множитель длины дуги)
SWITCH_PENALTY = 1.0               # за смену направления «вперёд ↔ назад»


@dataclass
class Pose:
    x: float
    y: float
    theta: float
    direction: int = 1             # +1 вперёд, -1 назад


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _bin(theta: float) -> int:
    return int(round(_wrap(theta) / HEADING_STEP)) % HEADINGS


def dijkstra_costs(grid: P.Grid, goal_cell: tuple[int, int]) -> list[float]:
    """Стоимость (м) от каждой клетки до цели по сетке (8 соседей, без курса)."""
    cost = [math.inf] * (grid.nx * grid.ny)
    gi, gj = goal_cell
    cost[gj * grid.nx + gi] = 0.0
    heap = [(0.0, gi, gj)]
    while heap:
        c, i, j = heapq.heappop(heap)
        if c > cost[j * grid.nx + i]:
            continue
        for di, dj, step in P.NEIGHBORS:
            ni, nj = i + di, j + dj
            if not grid.free(ni, nj):
                continue
            nc = c + step * P.CELL_M
            k = nj * grid.nx + ni
            if nc < cost[k]:
                cost[k] = nc
                heapq.heappush(heap, (nc, ni, nj))
    return cost


def _arc(grid: P.Grid, x: float, y: float, theta: float, steer_deg: float, direction: int):
    """Дуга ARC_M вперёд (direction=+1) или назад (-1). None при столкновении."""
    delta = math.radians(steer_deg)
    ds = direction * ARC_M / SUBSTEPS
    for _ in range(SUBSTEPS):
        theta = theta + ds / WHEELBASE_M * math.tan(delta)
        x = x + ds * math.cos(theta)
        y = y + ds * math.sin(theta)
        if not grid.free(*grid.cell_of(x, y)):
            return None
    return x, y, _wrap(theta)


def plan_hybrid(grid: P.Grid, start: Pose, goal_xy, max_expansions: int = MAX_EXPANSIONS,
                weight: float = HEUR_WEIGHT):
    """Путь из позы start к точке goal_xy: список Pose (первая — старт) или None."""
    goal_cell = P._nearest_free(grid, *grid.cell_of(*goal_xy))
    if goal_cell is None:
        return None
    gx, gy = grid.center(*goal_cell)
    if math.hypot(gx - start.x, gy - start.y) < GOAL_TOL_M:
        return [start, Pose(gx, gy, start.theta, start.direction)]
    h_cost = dijkstra_costs(grid, goal_cell)

    def heur(x, y):
        i, j = grid.cell_of(x, y)
        c = h_cost[j * grid.nx + i]
        return max(math.hypot(x - gx, y - gy), c) if math.isfinite(c) else math.inf

    # узел: (x, y, theta, g, parent, steer, direction)
    nodes = [(start.x, start.y, _wrap(start.theta), 0.0, -1, 0.0, start.direction)]
    best_g: dict[tuple[int, int, int, int], float] = {}
    heap = [(weight * heur(start.x, start.y), 0.0, 0)]
    expansions = 0
    while heap and expansions < max_expansions:
        _, g, idx = heapq.heappop(heap)
        x, y, th, g_node, _, steer_prev, dir_prev = nodes[idx]
        if g > g_node + 1e-9:
            continue
        if math.hypot(x - gx, y - gy) < GOAL_TOL_M:
            return _reconstruct(nodes, idx, start)
        key = (*grid.cell_of(x, y), _bin(th), dir_prev)
        if best_g.get(key, math.inf) + 1e-9 < g_node:
            continue
        best_g[key] = g_node
        expansions += 1
        for direction in (1, -1):
            for steer in STEERS_DEG:
                nxt = _arc(grid, x, y, th, steer, direction)
                if nxt is None:
                    continue
                nx_, ny_, nth = nxt
                if not math.isfinite(heur(nx_, ny_)):
                    continue
                cost = ARC_M * (REVERSE_PENALTY if direction < 0 else 1.0)
                cost += STEER_CHANGE_PENALTY * abs(steer - steer_prev)
                if direction != dir_prev:
                    cost += SWITCH_PENALTY
                ng = g_node + cost
                nkey = (*grid.cell_of(nx_, ny_), _bin(nth), direction)
                if ng + 1e-9 >= best_g.get(nkey, math.inf):
                    continue
                nodes.append((nx_, ny_, nth, ng, idx, steer, direction))
                heapq.heappush(heap, (ng + weight * heur(nx_, ny_), ng, len(nodes) - 1))
    return None


def _reconstruct(nodes, idx, start: Pose):
    out = []
    while idx >= 0:
        x, y, th, _, parent, _, direction = nodes[idx]
        out.append(Pose(x, y, th, direction))
        idx = parent
    out.reverse()
    out[0] = start
    return out


def path_length(poses) -> float:
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))
