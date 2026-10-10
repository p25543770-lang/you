"""Замер езды по лидару без известной карты: одометрия + сопоставление скана + Hybrid A*.

Запуск: .venv/bin/python scripts/bench_lidar_nav.py [seed_from seed_to]
"""
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_control.brain.lidar_nav import run_lidar_mission  # noqa: E402


def main() -> None:
    a = int(sys.argv[1]) if len(sys.argv) > 2 else 0
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    rows = []
    t0 = time.time()
    for seed in range(a, b):
        r = run_lidar_mission(seed)
        rows.append(r)
        print(f"seed {seed:3d} completed={r.completed} reached={r.reached}/8 touches={r.touches} "
              f"time={r.time_s}s loc_err_mean={r.loc_err_mean} loc_err_max={r.loc_err_max} "
              f"plan_fail={r.plan_failures} stuck={r.stuck_events} plan_ms_max={r.plan_ms_max}", flush=True)
    print(f"SUMMARY seeds={len(rows)} completed={sum(r.completed for r in rows)} "
          f"touches_mean={statistics.mean(r.touches for r in rows):.1f} "
          f"loc_err_mean={statistics.mean(r.loc_err_mean for r in rows):.2f} "
          f"wall={time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
