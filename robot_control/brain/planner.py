"""Планировщик пути: A* по сетке занятости + сглаживание + опережающая точка.

Задача нейросети (network.py) — рулить на ближайшую точку пути, а не на
далёкую цель через стеллажи. Тогда локальные ловушки, в которых застревал
реактивный контроллер, обходит планировщик, а сеть отвечает за движение.

Устройство:
  * карта: прямоугольные стеллажи и границы склада; клетка 0.10 м;
    препятствие «раздувается» на радиус робота + запас, чтобы путь был
    проходим с запасом;
  * поиск: A* по 8 соседям с октильной эвристикой, без срезания углов;
  * сглаживание: «натягивание нити» — точки пути убираются, если между
    соседями прямая видимость по сетке;
  * опережающая точка: на расстоянии LOOKAHEAD_M вдоль пути от ближайшей
    к роботу точки — это и есть /local_goal для нейросети.

Вычисления — чистый Python, без numpy: на борту нет интернета, а pip-колёса
в vendor/ ограничены Flask и gunicorn. Замеры — в scripts/bench_planner.py.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

CELL_M = 0.10
ROBOT_RADIUS_M = 0.35
CLEARANCE_M = 0.10            # дополнительный запас от стен и стеллажей
LOOKAHEAD_M = 0.9
SQRT2 = math.sqrt(2.0)
NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


@dataclass
class Grid:
    width: float
    height: float
    nx: int
    ny: int
    blocked: bytearray = field(repr=False)

    def idx(self, i: int, j: int) -> int:
        return j * self.nx + i

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        return (min(self.nx - 1, max(0, int(x / CELL_M))),
                min(self.ny - 1, max(0, int(y / CELL_M))))

    def center(self, i: int, j: int) -> tuple[float, float]:
        return ((i + 0.5) * CELL_M, (j + 0.5) * CELL_M)

    def free(self, i: int, j: int) -> bool:
        return 0 <= i < self.nx and 0 <= j < self.ny and not self.blocked[j * self.nx + i]


def build_grid(width: float, height: float, shelves, inflate: float | None = None) -> Grid:
    """Сетка занятости: граница склада и стеллажи, раздутые на inflate м."""
    r = ROBOT_RADIUS_M + CLEARANCE_M if inflate is None else inflate
    nx, ny = int(round(width / CELL_M)), int(round(height / CELL_M))
    blocked = bytearray(nx * ny)
    for j in range(ny):
        y = (j + 0.5) * CELL_M
        for i in range(nx):
            x = (i + 0.5) * CELL_M
            bad = x < r or y < r or x > width - r or y > height - r
            if not bad:
                for (x0, y0, x1, y1) in shelves:
                    if x0 - r < x < x1 + r and y0 - r < y < y1 + r:
                        bad = True
                        break
            if bad:
                blocked[j * nx + i] = 1
    return Grid(width, height, nx, ny, blocked)


def _octile(i0, j0, i1, j1):
    dx, dy = abs(i0 - i1), abs(j0 - j1)
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def _nearest_free(grid: Grid, i: int, j: int, max_r: int = 15) -> tuple[int, int] | None:
    if grid.free(i, j):
        return i, j
    for r in range(1, max_r + 1):
        for di in range(-r, r + 1):
            for dj in (-r, r):
                if grid.free(i + di, j + dj):
                    return i + di, j + dj
        for dj in range(-r + 1, r):
            for di in (-r, r):
                if grid.free(i + di, j + dj):
                    return i + di, j + dj
    return None


def astar(grid: Grid, start: tuple[int, int], goal: tuple[int, int]):
    """Путь в клетках (включая старт и цель) или None."""
    if not grid.free(*start) or not grid.free(*goal):
        return None
    g = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    heap = [(_octile(*start, *goal), 0.0, start)]
    closed = set()
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path
        closed.add(cur)
        ci, cj = cur
        for di, dj, cost in NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if not grid.free(ni, nj):
                continue
            # без срезания углов: диагональ только если оба соседа свободны
            if di and dj and not (grid.free(ci + di, cj) and grid.free(ci, cj + dj)):
                continue
            nxt = (ni, nj)
            ng = gc + cost
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(heap, (ng + _octile(ni, nj, *goal), ng, nxt))
    return None


def _line_free(grid: Grid, a, b) -> bool:
    """Прямая видимость по клеткам (шаг — полклетки)."""
    (x0, y0), (x1, y1) = a, b
    steps = max(abs(x1 - x0), abs(y1 - y0)) * 2
    for k in range(steps + 1):
        t = k / steps if steps else 0.0
        if not grid.free(int(round(x0 + (x1 - x0) * t)), int(round(y0 + (y1 - y0) * t))):
            return False
    return True


def smooth(grid: Grid, cells):
    """Натягивание нити: оставляем точки, где путь меняет направление."""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    anchor = 0
    k = 1
    while k < len(cells):
        far = k
        for m in range(len(cells) - 1, k - 1, -1):
            if _line_free(grid, cells[anchor], cells[m]):
                far = m
                break
        out.append(cells[far])
        anchor = far
        k = far + 1
    return out


def plan(grid: Grid, start_xy, goal_xy):
    """Полный план: список точек (x, y) в метрах от старта до цели, либо None."""
    s = _nearest_free(grid, *grid.cell_of(*start_xy))
    g = _nearest_free(grid, *grid.cell_of(*goal_xy))
    if s is None or g is None:
        return None
    cells = astar(grid, s, g)
    if cells is None:
        return None
    cells = smooth(grid, cells)
    pts = [grid.center(i, j) for (i, j) in cells]
    pts[0] = tuple(start_xy)
    pts[-1] = tuple(goal_xy)
    return pts


def lookahead_point(path, pos, lookahead: float = LOOKAHEAD_M):
    """Точка на пути на расстоянии lookahead вперёд от ближайшей к роботу."""
    if not path:
        return tuple(pos)
    best_k, best_d = 0, math.inf
    for k, (x, y) in enumerate(path):
        d = math.hypot(x - pos[0], y - pos[1])
        if d < best_d:
            best_k, best_d = k, d
    remaining = lookahead
    prev = tuple(pos) if best_k == 0 else path[best_k]
    for k in range(best_k + 1, len(path)):
        nxt = path[k]
        seg = math.hypot(nxt[0] - prev[0], nxt[1] - prev[1])
        if seg >= remaining and seg > 0:
            t = remaining / seg
            return (prev[0] + (nxt[0] - prev[0]) * t, prev[1] + (nxt[1] - prev[1]) * t)
        remaining -= seg
        prev = nxt
    return tuple(path[-1])


def path_length(path) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def cross_track(path, pos) -> float:
    """Расстояние от робота до ближайшей точки пути (м)."""
    if not path:
        return 0.0
    return min(math.hypot(x - pos[0], y - pos[1]) for x, y in path)


class PathFollower:
    """Ведёт робота по плану: перепланирует при смене цели, раз в секунду
    (по времени симуляции) и при уходе с пути дальше deviate_m."""

    def __init__(self, grid: Grid, replan_s: float = 1.0, deviate_m: float = 0.8,
                 lookahead: float = LOOKAHEAD_M):
        self.grid = grid
        self.replan_s = replan_s
        self.deviate_m = deviate_m
        self.lookahead = lookahead
        self.path = None
        self.goal = None
        self.last_plan_t = -math.inf
        self.replans = 0
        self.plan_ms: list[float] = []
        self.failures = 0

    def update(self, pos, goal, t: float):
        """Возвращает опережающую точку (x, y). Если плана нет — прямо к цели."""
        import time

        goal = tuple(goal)
        need = (self.path is None or self.goal != goal
                or t - self.last_plan_t >= self.replan_s
                or cross_track(self.path, pos) > self.deviate_m)
        if need:
            t0 = time.perf_counter()
            path = plan(self.grid, pos, goal)
            self.plan_ms.append((time.perf_counter() - t0) * 1000.0)
            self.replans += 1
            self.last_plan_t = t
            self.goal = goal
            if path is None:
                self.failures += 1
                self.path = [tuple(pos), goal]
            else:
                self.path = path
        return lookahead_point(self.path, pos, self.lookahead)

