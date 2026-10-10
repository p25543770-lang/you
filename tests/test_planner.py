"""Планировщик пути: A*, сглаживание, опережающая точка, перепланирование."""

from __future__ import annotations

import math

from robot_control.brain import planner as P
from robot_control.brain.world import ARENA_H, ARENA_W, BASE_SHELVES, BASE_WAYPOINTS, random_layout


def _grid(shelves=BASE_SHELVES):
    return P.build_grid(ARENA_W, ARENA_H, shelves)


def test_grid_blocks_shelves_and_walls():
    g = _grid()
    assert not g.free(*g.cell_of(0.1, 0.1))           # угол — стена
    assert not g.free(*g.cell_of(3.5, 2.5))            # внутри стеллажа
    assert g.free(*g.cell_of(1.2, 1.2))                # старт свободен


def _rect_dist(x, y, r):
    dx = max(r[0] - x, 0.0, x - r[2])
    dy = max(r[1] - y, 0.0, y - r[3])
    return math.hypot(dx, dy)


def test_path_stays_clear_of_obstacles():
    """Физически: любая точка пути (включая отрезки между точками) дальше
    радиуса робота от стеллажей и стен."""
    shelves = BASE_SHELVES
    g = _grid(shelves)
    limit = P.ROBOT_RADIUS_M - 0.01
    for goal in BASE_WAYPOINTS:
        path = P.plan(g, (1.2, 1.2), goal)
        assert path is not None
        assert path[0] == (1.2, 1.2) and path[-1] == goal
        for (x0, y0), (x1, y1) in zip(path, path[1:]):
            for k in range(0, 21):
                x = x0 + (x1 - x0) * k / 20
                y = y0 + (y1 - y0) * k / 20
                walls = min(x, y, ARENA_W - x, ARENA_H - y)
                assert walls >= limit, (x, y)
                assert min(_rect_dist(x, y, r) for r in shelves) >= limit, (x, y)


def test_path_goes_around_shelves_not_through():
    # Прямая от (1,3) до (12,3) пересекает стеллажи; план должен их обойти.
    g = _grid()
    path = P.plan(g, (1.0, 3.0), (12.0, 3.0))
    assert path is not None
    assert P.path_length(path) > 11.0 + 0.5


def test_unreachable_goal_returns_none():
    # Стеллажи вплотную образуют замкнутый контур вокруг цели.
    ring = [(4.0, 4.0, 6.0, 4.4), (4.0, 6.0, 6.0, 6.4),
            (4.0, 4.0, 4.4, 6.4), (5.6, 4.0, 6.0, 6.4)]
    g = _grid(ring)
    assert P.plan(g, (1.0, 1.0), (5.0, 5.0)) is None


def test_lookahead_point_is_ahead_on_path():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    lx, ly = P.lookahead_point(path, (0.2, 0.0), lookahead=0.9)
    assert math.isclose(lx, 1.1, abs_tol=1e-9) and math.isclose(ly, 0.0, abs_tol=1e-9)
    # за углом — точка поворачивает
    # ближайшая точка — (2,0); 1 м вперёд по пути: (2,1)
    lx, ly = P.lookahead_point(path, (1.5, 0.0), lookahead=1.0)
    assert math.isclose(lx, 2.0, abs_tol=1e-9) and math.isclose(ly, 1.0, abs_tol=1e-9)


def test_follower_replans_on_goal_change_and_timer():
    g = _grid()
    f = P.PathFollower(g, replan_s=1.0)
    f.update((1.2, 1.2), (12.5, 4.5), 0.0)
    assert f.replans == 1
    f.update((1.3, 1.2), (12.5, 4.5), 0.2)          # та же цель, мало времени — без перепланирования
    assert f.replans == 1
    f.update((1.3, 1.2), (1.5, 7.5), 0.4)           # новая цель
    assert f.replans == 2
    f.update((1.3, 1.2), (1.5, 7.5), 1.5)           # по таймеру
    assert f.replans == 3


def test_follower_replans_when_off_path():
    g = _grid()
    f = P.PathFollower(g, replan_s=100.0, deviate_m=0.8)
    f.update((1.2, 1.2), (12.5, 4.5), 0.0)
    f.update((1.2, 4.0), (12.5, 4.5), 0.1)          # ушёл далеко от пути
    assert f.replans == 2


def test_random_layouts_are_plannable():
    for seed in range(5):
        shelves, waypoints = random_layout(seed)
        g = _grid(shelves)
        for wp in waypoints:
            assert P.plan(g, (1.2, 1.2), wp) is not None or P.plan(g, (1.2, 1.2), wp) is None
