"""Перебор seed для езды по лидару. Пишет по одной JSON-строке на seed.

Запуск: .venv/bin/python scripts/sweep_lidar.py OUT.jsonl SEED_FROM SEED_TO
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_control.brain.lidar_nav import run_lidar_mission  # noqa: E402


def main() -> None:
    out, a, b = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    with open(out, "a", encoding="utf-8") as f:
        for seed in range(a, b):
            t0 = time.time()
            r = run_lidar_mission(seed)
            row = dict(seed=seed, completed=r.completed, reached=r.reached, touches=r.touches,
                       sim_s=r.time_s, loc_err_mean=r.loc_err_mean, loc_err_max=r.loc_err_max,
                       plan_failures=r.plan_failures, stuck=r.stuck_events, replans=r.replans,
                       plan_ms_max=r.plan_ms_max, wall_s=round(time.time() - t0))
            f.write(json.dumps(row) + "\n")
            f.flush()


if __name__ == "__main__":
    main()
