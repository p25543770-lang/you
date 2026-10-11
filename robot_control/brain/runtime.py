"""Живая симуляция: граф ROS-подобных узлов + ИИ, работает в реальном времени.

Топики (как в ROS 2):
    /cmd_vel        ← BrainNode  (geometry_msgs/Twist: скорость, угол руля)
    /odom           → WorldNode  (поза, скорость, пройденный путь, столкновения)  20 Гц
    /scan           → WorldNode  (16 лучей лидара, м)                           10 Гц
    /goal           ← MissionNode (текущая точка маршрута)                       2 Гц
    /brain/activity ← BrainNode  (активности 40 нейронов и выходы)               20 Гц
    /modules/state  ← ModulesNode (углы и обороты 4 модулей, как на экране RUS SLAM) 5 Гц

Физика шагает с фиксированным шагом 50 Гц и привязана к часам (1 с симуляции =
1 с реального времени), поэтому поведение можно наблюдать вживую.
"""

from __future__ import annotations

import json
import logging
import math
import random
import threading
import time
from pathlib import Path

from .bus import Bus, Node
from .episode import GOAL_RADIUS_M, SENSOR_NOISE_M
from .network import DEFAULT_NEURONS, N_INPUTS, WEIGHTS_FILE, Brain, features, load_weights
from .planner import PathFollower, build_grid, path_length
from .world import (
    ARENA_H, ARENA_W, BASE_SHELVES, BASE_WAYPOINTS, START_POSE, V_MAX,
    Robot, arena_segments, shelves_to_segments,
)

log = logging.getLogger("robot_control.brain")

PHYSICS_DT = 0.02          # 50 Гц
TRAIL_LEN = 400


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class WorldNode(Node):
    """Физика и сенсоры: двигает робота, публикует /odom и /scan."""

    def __init__(self, bus: Bus, sim: "BrainSim"):
        super().__init__(bus, "world")
        self.sim = sim
        self.cmd = (0.0, 0.0)
        self.subscribe("/cmd_vel", self._on_cmd)
        self.create_timer(20.0, self._publish_odom)
        self.create_timer(10.0, self._publish_scan)

    def _on_cmd(self, msg):
        self.cmd = (msg["linear"], msg["steer_deg"])

    def physics(self, t: float) -> None:
        self.sim.robot.step(self.cmd[0], self.cmd[1], PHYSICS_DT, self.sim.segments)

    def _publish_odom(self, t: float) -> None:
        r = self.sim.robot
        self.publish("/odom", {
            "stamp": t, "x": round(r.x, 3), "y": round(r.y, 3),
            "theta": round(_wrap(r.theta), 4), "v": round(r.v, 3),
            "odometer": round(r.odometer, 2), "collisions": r.collisions,
        })

    def _publish_scan(self, t: float) -> None:
        r = self.sim.robot
        ranges = r.lidar(self.sim.segments, SENSOR_NOISE_M, self.sim.rng)
        self.sim.last_lidar = ranges
        self.publish("/scan", {"stamp": t, "ranges": ranges})


class MissionNode(Node):
    """Маршрут: публикует текущую точку /goal и засчитывает достижения."""

    def __init__(self, bus: Bus, sim: "BrainSim"):
        super().__init__(bus, "mission")
        self.sim = sim
        self.subscribe("/odom", self._on_odom)
        self.create_timer(2.0, self._publish_goal)

    def _on_odom(self, msg):
        s = self.sim
        gx, gy = s.waypoints[s.goal_idx]
        if math.hypot(gx - msg["x"], gy - msg["y"]) < GOAL_RADIUS_M:
            s.reached_total += 1
            s.goal_idx += 1
            if s.goal_idx >= len(s.waypoints):
                s.goal_idx = 0
                s.cycles += 1
            self._publish_goal(msg["stamp"])

    def _publish_goal(self, t: float) -> None:
        s = self.sim
        gx, gy = s.waypoints[s.goal_idx]
        self.publish("/goal", {"stamp": t, "index": s.goal_idx, "x": gx, "y": gy})


class PlannerNode(Node):
    """Планировщик: /odom + /goal → /local_goal (опережающая точка), /path (план)."""

    def __init__(self, bus: Bus, sim: "BrainSim"):
        super().__init__(bus, "planner")
        self.sim = sim
        self.pose = None
        self.goal = None
        self.follower = PathFollower(build_grid(ARENA_W, ARENA_H, sim.shelves))
        self.subscribe("/odom", lambda m: setattr(self, "pose", m))
        self.subscribe("/goal", self._on_goal)
        self.create_timer(5.0, self._tick)

    def _on_goal(self, msg):
        self.goal = msg

    def _tick(self, t: float) -> None:
        if not (self.pose and self.goal):
            return
        replans_before = self.follower.replans
        x, y = self.follower.update((self.pose["x"], self.pose["y"]),
                                    (self.goal["x"], self.goal["y"]), t)
        self.publish("/local_goal", {"stamp": t, "x": round(x, 3), "y": round(y, 3)})
        if self.follower.replans != replans_before:
            self.publish("/path", {"stamp": t, "poses": [
                {"x": round(px, 3), "y": round(py, 3)} for px, py in self.follower.path]})

    def stats(self) -> dict:
        ms = self.follower.plan_ms
        return {
            "replans": self.follower.replans,
            "failures": self.follower.failures,
            "planMsMean": round(sum(ms) / len(ms), 2) if ms else 0.0,
            "planMsMax": round(max(ms), 2) if ms else 0.0,
            "pathPoints": len(self.follower.path or []),
            "pathLengthM": round(path_length(self.follower.path or []), 2),
        }


class BrainNode(Node):
    """ИИ-контроллер: /scan + /odom + /local_goal (от планировщика) → /cmd_vel."""

    def __init__(self, bus: Bus, sim: "BrainSim", brain: Brain):
        super().__init__(bus, "brain")
        self.sim = sim
        self.brain = brain
        self.scan = None
        self.odom = None
        self.goal = None
        self.last = {"speed": 0.0, "steer": 0.0}
        self.last_inputs = [0.0] * N_INPUTS
        self.subscribe("/scan", lambda m: setattr(self, "scan", m["ranges"]))
        self.subscribe("/odom", lambda m: setattr(self, "odom", m))
        self.subscribe("/local_goal", lambda m: setattr(self, "goal", m))
        self.create_timer(20.0, self._tick)

    def _tick(self, t: float) -> None:
        if not (self.scan and self.odom and self.goal):
            return
        dx = self.goal["x"] - self.odom["x"]
        dy = self.goal["y"] - self.odom["y"]
        dist = math.hypot(dx, dy)
        rel = _wrap(math.atan2(dy, dx) - self.odom["theta"])
        x = features(self.scan, rel, dist, self.odom["v"])
        self.last_inputs = [round(v, 3) for v in x]
        speed, steer = self.brain.step(x)
        self.last = {"speed": round(speed, 3), "steer": round(steer, 2)}
        self.publish("/cmd_vel", {"stamp": t, "linear": speed, "steer_deg": steer})
        self.publish("/brain/activity", {
            "stamp": t,
            "hidden": [round(h, 3) for h in self.brain.h],
            "outputs": [round(o, 3) for o in self.brain.out],
            "inputs": [round(v, 3) for v in x],
        })


class ModulesNode(Node):
    """Четыре модуля руления/хода — тот же вид данных, что на экране RUS SLAM."""

    NAMES = [("FL", "передний левый", 1), ("FR", "передний правый", 1),
             ("RL", "задний левый", -1), ("RR", "задний правый", -1)]  # RL/RR — руль задний, противофаза

    def __init__(self, bus: Bus, sim: "BrainSim"):
        super().__init__(bus, "modules")
        self.sim = sim
        self.create_timer(5.0, self._publish)

    def _publish(self, t: float) -> None:
        r = self.sim.robot
        wheel_rpm = r.v / (2.0 * math.pi * 0.127) * 60.0
        mods = []
        for mid, title, _ in self.NAMES:
            mods.append({
                "id": mid, "title": title,
                "angle": round(-r.steer if mid[0] == "R" else r.steer, 1),
                "rpm": round(abs(wheel_rpm), 1),
            })
        self.publish("/modules/state", {"stamp": t, "modules": mods})


class BrainSim:
    """Собирает граф узлов, крутит его в реальном времени, отдаёт снимок."""

    def __init__(self, weights_path: Path = WEIGHTS_FILE, seed: int = 7):
        payload = load_weights(weights_path)
        self.validation = payload.get("validation")
        n = int(payload.get("neurons", DEFAULT_NEURONS))
        if payload.get("inputs", N_INPUTS) != N_INPUTS:
            raise ValueError("веса обучены для другого числа входов — переобучите сеть")
        self.n_neurons = n
        self.weights = payload["weights"]
        self.rng = random.Random(seed)
        self.lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.started_wall = None
        self.reset()

    # ---------------------------- состояние мира ---------------------------- #
    def reset(self) -> None:
        with self.lock:
            self.shelves = BASE_SHELVES
            self.waypoints = BASE_WAYPOINTS
            self.segments = arena_segments() + shelves_to_segments(self.shelves)
            self.robot = Robot(*START_POSE)
            self.bus = Bus()
            for name, typ in [("/cmd_vel", "geometry_msgs/Twist"),
                              ("/odom", "nav_msgs/Odometry"),
                              ("/scan", "sensor_msgs/LaserScan"),
                              ("/goal", "geometry_msgs/PoseStamped"),
                              ("/path", "nav_msgs/Path"),
                              ("/local_goal", "geometry_msgs/PointStamped"),
                              ("/brain/activity", "std_msgs/Float32MultiArray"),
                              ("/modules/state", "rus_slam/ModuleState")]:
                self.bus.declare(name, typ)
            self.goal_idx = 0
            self.reached_total = 0
            self.cycles = 0
            self.last_lidar = [4.0] * 16
            self.sim_t = 0.0
            self.trail: list = []
            self.world = WorldNode(self.bus, self)
            self.mission = MissionNode(self.bus, self)
            self.planner_node = PlannerNode(self.bus, self)
            self.brain_node = BrainNode(self.bus, self, Brain(self.n_neurons, self.weights))
            self.modules = ModulesNode(self.bus, self)
            self.nodes = [self.world, self.mission, self.planner_node, self.brain_node, self.modules]

    def _tick(self) -> None:
        self.sim_t += PHYSICS_DT
        self.world.physics(self.sim_t)
        self.trail.append((round(self.robot.x, 3), round(self.robot.y, 3)))
        if len(self.trail) > TRAIL_LEN:
            del self.trail[:len(self.trail) - TRAIL_LEN]
        for node in self.nodes:
            node.spin_once(self.sim_t)

    def _loop(self) -> None:
        wall0 = time.monotonic()
        sim0 = self.sim_t
        while not self._stop.is_set():
            with self.lock:
                self._tick()
            target = wall0 + (self.sim_t - sim0)
            delay = target - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            elif delay < -1.0:      # отстали больше секунды (например, пауза) — пересинхронизация
                wall0 = time.monotonic()
                sim0 = self.sim_t

    def ensure_running(self) -> None:
        with self.lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop.clear()
            self.started_wall = time.time()
            self._thread = threading.Thread(target=self._loop, name="brain-sim", daemon=True)
            self._thread.start()
            log.info("симуляция ROS + ИИ запущена (%d нейронов)", self.n_neurons)

    def stop(self) -> None:
        self._stop.set()

    # ------------------------------- снимок -------------------------------- #
    def snapshot(self) -> dict:
        with self.lock:
            r = self.robot
            hidden = self.brain_node.brain.h
            gx, gy = self.waypoints[self.goal_idx]
            topic_rates = self.bus.rates(self.sim_t)
            return {
                "simTime": round(self.sim_t, 2),
                "neurons": self.n_neurons,
                "inputs": N_INPUTS,
                "arena": [ARENA_W, ARENA_H],
                "shelves": [list(s) for s in self.shelves],
                "waypoints": [list(w) for w in self.waypoints],
                "robot": {
                    "x": round(r.x, 3), "y": round(r.y, 3),
                    "thetaDeg": round(math.degrees(_wrap(r.theta)), 1),
                    "v": round(r.v, 3), "vMax": V_MAX,
                    "steerDeg": round(r.steer, 2),
                    "odometer": round(r.odometer, 2),
                    "collisions": r.collisions,
                },
                "lidar": self.last_lidar,
                "trail": self.trail[-TRAIL_LEN:],
                "path": [list(p) for p in ((self.planner_node.follower.path or []))],
                "localGoal": self.bus.topics["/local_goal"].last,
                "planner": self.planner_node.stats(),
                "goal": {"index": self.goal_idx, "x": gx, "y": gy},
                "mission": {"reached": self.reached_total, "cycles": self.cycles},
                "activity": {
                    "hidden": [round(h, 3) for h in hidden],
                    "outputs": [round(o, 3) for o in self.brain_node.brain.out],
                    "inputs": self.brain_node.last_inputs,
                    "speed": self.brain_node.last["speed"],
                    "steer": self.brain_node.last["steer"],
                },
                "modules": [
                    {"id": m[0], "title": m[1]} for m in ModulesNode.NAMES
                ],
                "topics": [
                    {"name": t.name, "type": t.msg_type, "hz": topic_rates.get(t.name, 0.0),
                     "count": t.count}
                    for t in self.bus.topics.values()
                ],
                "nodes": [n.name for n in self.nodes],

                "validation": self.validation,
                "running": bool(self._thread and self._thread.is_alive()),
            }
