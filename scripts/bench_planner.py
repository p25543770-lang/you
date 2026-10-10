#!/usr/bin/env python3
"""Замер планировщика пути на целевом железе (Intel Core i5-13400, 16 ГБ ОЗУ).

    .venv/bin/python scripts/bench_planner.py            # 200 планов на случайных раскладках
    .venv/bin/python scripts/bench_planner.py --plans 1000

Печатает: время построения сетки, среднее/p95/максимум времени плана (мс),
пиковую память процесса (МБ) и сколько планов не нашлось.
"""

from __future__ import annotations

import argparse
import os
import platform
import random
import resource
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robot_control.brain import planner as P  # noqa: E402
from robot_control.brain.world import (  # noqa: E402
    ARENA_H, ARENA_W, random_layout,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plans", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    print(f"процессор: {platform.processor() or platform.machine()}, ядер: {os.cpu_count()}, "
          f"python {platform.python_version()}")
    rng = random.Random(a.seed)
    times, fails = [], 0
    build_ms = []
    for k in range(a.plans):
        shelves, _ = random_layout(a.seed * 1000 + k)
        t0 = time.perf_counter()
        grid = P.build_grid(ARENA_W, ARENA_H, shelves)
        build_ms.append((time.perf_counter() - t0) * 1000)
        start = (rng.uniform(1, ARENA_W - 1), rng.uniform(1, ARENA_H - 1))
        goal = (rng.uniform(1, ARENA_W - 1), rng.uniform(1, ARENA_H - 1))
        t0 = time.perf_counter()
        path = P.plan(grid, start, goal)
        times.append((time.perf_counter() - t0) * 1000)
        if path is None:
            fails += 1
    times.sort()
    p95 = times[int(0.95 * (len(times) - 1))]
    peak_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    print(f"сетка 0.10 м ({P.build_grid(ARENA_W, ARENA_H, []).nx}×{P.build_grid(ARENA_W, ARENA_H, []).ny} клеток): "
          f"построение {statistics.mean(build_ms):.1f} мс")
    print(f"план A*: среднее {statistics.mean(times):.1f} мс, p95 {p95:.1f} мс, "
          f"максимум {times[-1]:.1f} мс, не найдено {fails} из {a.plans}")
    print(f"пиковая память процесса: {peak_mb:.0f} МБ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
