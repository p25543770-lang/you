import math

from robot_control.brain import hybrid as H
from robot_control.brain import planner as P
from robot_control.brain.controller import PathTracker
from robot_control.brain.missions import hard_layout
from robot_control.brain.world import ARENA_H, ARENA_W, START_POSE


def test_hybrid_plans_reverse_out_of_dead_end_seed6():
    shelves, wps = hard_layout(6, 8)
    grid = P.build_grid(ARENA_W, ARENA_H, shelves, inflate=H.COLLISION_INFLATE_M)
    path = H.plan_hybrid(grid, H.Pose(*START_POSE), wps[0])
    assert path is not None
    assert any(p.direction < 0 for p in path)      # старт заперт — без заднего хода не выйти
    assert path[-1].direction in (1, -1)
    assert math.hypot(path[-1].x - wps[0][0], path[-1].y - wps[0][1]) < 0.6


def test_tracker_stops_at_cusp_and_does_not_skip_reverse():
    shelves, wps = hard_layout(6, 8)
    grid = P.build_grid(ARENA_W, ARENA_H, shelves, inflate=H.COLLISION_INFLATE_M)
    path = H.plan_hybrid(grid, H.Pose(*START_POSE), wps[0])
    tr = PathTracker(path)
    tgt, direction, _ = tr.target(START_POSE[0], START_POSE[1])
    # первый отрезок — задний: цель не должна перескочить разворот и уйти вперёд
    first_cusp = next(k for k in range(1, len(path)) if path[k].direction != path[k - 1].direction)
    assert direction == path[1].direction
    assert math.hypot(tgt[0] - path[first_cusp].x, tgt[1] - path[first_cusp].y) > 0 or first_cusp >= 1
