"""Замер сложных миссий: Hybrid A* + pure pursuit с задним ходом.

Запуск: .venv/bin/python scripts/bench_missions.py [seed_from seed_to]
Печатает построчно результат каждой миссии и итоговую сводку.
"""
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_control.brain.missions import run_mission  # noqa: E402


def main() -> None:
    a = int(sys.argv[1]) if len(sys.argv) > 2 else 0
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    rows = []
    t0 = time.time()
    for seed in range(a, b):
        r = run_mission(seed)
        rows.append(r)
        print(f"seed {seed:3d} completed={r.completed} reached={r.reached}/8 "
              f"touches={r.collisions} time={r.time_s}s replans={r.replans} "
              f"plan_ms_max={r.plan_ms_max}", flush=True)
    done = sum(r.completed for r in rows)
    print(f"SUMMARY seeds={len(rows)} completed={done} "
          f"touches_mean={statistics.mean(r.collisions for r in rows):.2f} "
          f"time_mean={statistics.mean(r.time_s for r in rows):.1f}s "
          f"wall={time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
