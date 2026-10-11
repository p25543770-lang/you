"""Симуляция ROS + ИИ: сеть, шина узлов, физика, веб-доступ."""

from __future__ import annotations

import math

import pytest

from conftest import login
from robot_control.brain.bus import Bus, Node
from robot_control.brain.network import (
    DEFAULT_NEURONS, MAX_NEURONS, MIN_NEURONS, N_INPUTS, Brain, param_count, reflex_prior,
)
from robot_control.brain.world import (
    BASE_SHELVES, ROBOT_RADIUS_M, Robot, arena_segments, shelves_to_segments,
)


@pytest.mark.parametrize("n", [MIN_NEURONS, DEFAULT_NEURONS, MAX_NEURONS])
def test_network_size_in_range(n):
    assert len(reflex_prior(n)) == param_count(n)
    Brain(n, reflex_prior(n))


@pytest.mark.parametrize("n", [MIN_NEURONS - 1, MAX_NEURONS + 1])
def test_network_size_out_of_range_rejected(n):
    with pytest.raises(ValueError):
        Brain(n)


def test_network_outputs_are_bounded():
    brain = Brain(DEFAULT_NEURONS, reflex_prior(DEFAULT_NEURONS))
    for _ in range(50):
        speed, steer = brain.step([1.0] * N_INPUTS)
        assert 0.0 <= speed <= 0.7 + 1e-9
        assert -30.0 <= steer <= 30.0


def test_bus_delivers_to_subscribers_in_order():
    bus = Bus()
    bus.declare("/a", "t")
    got = []
    node = Node(bus, "n")
    node.subscribe("/a", lambda m: got.append(m["v"]))
    for v in range(3):
        bus.publish("/a", {"stamp": 0.0, "v": v})
    node.spin_once(0.0)
    assert got == [0, 1, 2]
    assert bus.topics["/a"].count == 3


def test_bus_rejects_undeclared_topic():
    with pytest.raises(KeyError):
        Bus().publish("/nope", {"stamp": 0.0})


def test_timer_runs_at_its_rate():
    bus = Bus()
    node = Node(bus, "t")
    ticks = []
    node.create_timer(10.0, ticks.append)
    t = 0.0
    while t < 1.0 - 1e-9:
        node.spin_once(t)
        t += 0.01
    assert len(ticks) == 10


def test_robot_stops_at_wall():
    segs = arena_segments()
    robot = Robot(1.0, 4.5, 0.0)
    for _ in range(2000):
        robot.step(0.7, 0.0, 0.02, segs)
    assert robot.x < 14.0 - ROBOT_RADIUS_M
    assert robot.collisions >= 1


def test_lidar_sees_shelf_in_front():
    segs = arena_segments() + shelves_to_segments(BASE_SHELVES)
    robot = Robot(1.0, 2.8, 0.0)          # смотрит на стеллаж x=3…4
    ranges = robot.lidar(segs)
    assert len(ranges) == 16
    assert 1.5 < ranges[0] < 2.1
    assert all(0.0 < r <= 4.0 for r in ranges)


def test_collision_is_counted_per_contact_not_per_step():
    segs = arena_segments()
    robot = Robot(13.0, 4.5, 0.0)
    for _ in range(200):
        robot.step(0.7, 0.0, 0.02, segs)
    # прижатие к стене за 200 шагов — несколько касаний, но не 200
    assert 1 <= robot.collisions < 20


def test_brain_pages_require_login(client):
    for path in ("/brain", "/api/brain/state"):
        response = client.get(path, follow_redirects=False)
        assert response.status_code in (302, 401, 403)


def test_brain_state_after_login(client, operator):
    login(client)
    response = client.get("/api/brain/state")
    assert response.status_code == 200
    data = response.get_json()["data"]
    names = {t["name"] for t in data["topics"]}
    assert {"/cmd_vel", "/odom", "/scan", "/goal", "/brain/activity"} <= names
    assert data["neurons"] == DEFAULT_NEURONS
    assert data["inputs"] == N_INPUTS
    page = client.get("/brain")
    assert page.status_code == 200
    assert b"brain.js" in page.data
