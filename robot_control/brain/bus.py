"""Минимальная ROS-подобная шина: топики, издатели, подписчики, таймеры.

Настоящего ROS 2 (rclpy) на борту и в песочнице нет, поэтому граф узлов
повторяет его модель: узлы публикуют сообщения в именованные топики
(``/scan``, ``/odom``, ``/cmd_vel`` ...), подписчики получают их очередью, а
исполнитель (executor) по очереди «крутит» узлы и вызывает их таймеры.
Сообщения — обычные dict с полем ``stamp`` (время симуляции, с).
"""

from __future__ import annotations

import collections
import threading
from dataclasses import dataclass, field
from typing import Callable

RATE_WINDOW_S = 2.0


@dataclass
class Topic:
    name: str
    msg_type: str
    last: dict | None = None
    count: int = 0
    stamps: collections.deque = field(default_factory=lambda: collections.deque(maxlen=200))


class Bus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.topics: dict[str, Topic] = {}
        self._subs: dict[str, list[Node]] = collections.defaultdict(list)

    def declare(self, name: str, msg_type: str) -> None:
        with self._lock:
            self.topics.setdefault(name, Topic(name, msg_type))

    def publish(self, name: str, msg: dict) -> None:
        with self._lock:
            topic = self.topics.get(name)
            if topic is None:
                raise KeyError(f"топик {name} не объявлен")
            topic.last = msg
            topic.count += 1
            topic.stamps.append(msg.get("stamp", 0.0))
            subscribers = list(self._subs[name])
        for node in subscribers:
            node._inbox.append((name, msg))

    def subscribe(self, node: "Node", name: str) -> None:
        with self._lock:
            self._subs[name].append(node)

    def rates(self, now: float) -> dict[str, float]:
        """Частота публикаций по топикам за последние RATE_WINDOW_S секунд."""
        out = {}
        with self._lock:
            for name, topic in self.topics.items():
                recent = [s for s in topic.stamps if now - s <= RATE_WINDOW_S]
                out[name] = round(len(recent) / RATE_WINDOW_S, 1)
        return out


class Node:
    """Базовый узел: входящая очередь, таймеры и обработчики подписок."""

    def __init__(self, bus: Bus, name: str) -> None:
        self.bus = bus
        self.name = name
        self._inbox: collections.deque = collections.deque()
        self._handlers: dict[str, Callable[[dict], None]] = {}
        self._timers: list[list] = []   # [period, next_due, callback]

    def subscribe(self, topic: str, handler: Callable[[dict], None]) -> None:
        self._handlers[topic] = handler
        self.bus.subscribe(self, topic)

    def create_timer(self, rate_hz: float, callback: Callable[[float], None]) -> None:
        self._timers.append([1.0 / rate_hz, 0.0, callback])

    def publish(self, topic: str, msg: dict) -> None:
        self.bus.publish(topic, msg)

    def spin_once(self, now: float) -> None:
        """Обработать входящие сообщения и сработавшие таймеры (как executor)."""
        while self._inbox:
            topic, msg = self._inbox.popleft()
            handler = self._handlers.get(topic)
            if handler:
                handler(msg)
        for timer in self._timers:
            period, due, callback = timer
            if now + 1e-9 >= due:
                callback(now)
                timer[1] = max(due + period, now)  # без «догоняющих» залпов
