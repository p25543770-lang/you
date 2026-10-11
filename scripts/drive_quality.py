"""Качество езды: зазор до стен/стеллажей, доля времени у стен, плавность руля, прохождение миссий.

Запуск: .venv/bin/python scripts/drive_quality.py [seed_from seed_to] [--baseline]
--baseline: прежняя настройка (запас 0.42 м, без сглаживания руля) — для сравнения.
"""
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_control.brain import missions as M  # noqa: E402


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    a, b = (int(args[0]), int(args[1])) if len(args) >= 2 else (0, 4)
    if "--baseline" in sys.argv:
        M.GRID_TIERS_M = (0.42,)
        M.STEER_SMOOTH = 1.0
    rows = []
    for seed in range(a, b):
        r = M.run_mission(seed)
        rows.append(r)
        print(f"seed {seed:3d} completed={r.completed} reached={r.reached}/8 touches={r.collisions} "
              f"min_clear={r.min_clear_m} close_frac={r.close_frac} steer_rate_mean={r.steer_rate_mean} "
              f"steer_rate_max={r.steer_rate_max} time={r.time_s}", flush=True)
    print(f"SUMMARY n={len(rows)} completed={sum(r.completed for r in rows)} "
          f"touches_mean={statistics.mean(r.collisions for r in rows):.1f} "
          f"min_clear_mean={statistics.mean(r.min_clear_m for r in rows):.3f} "
          f"close_frac_mean={statistics.mean(r.close_frac for r in rows):.3f} "
          f"steer_rate_mean={statistics.mean(r.steer_rate_mean for r in rows):.1f}", flush=True)


if __name__ == "__main__":
    main()
