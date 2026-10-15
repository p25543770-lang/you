"""Приём скана лидара из ROS 2: подписка на ``/scan`` и понятный статус.

Раньше ROS 2 был лишь одной из веток внутри ``LidarManager``: топик читался
только если в ``.env`` не был задан USB-порт, о найденном дистрибутиве
оператор ничего не видел, а «ROS 2 есть, а скана нет» выглядело как обычное
«лидар не найден». Теперь это отдельный, проверяемый модуль:

  * подписка на топик скана (``sensor_msgs/LaserScan``, по умолчанию ``/scan``)
    и на состояние супервизора драйвера (``/zapret/lidar/status``, JSON из
    пакета ``zapret_lidar``);
  * подсчёт частоты и свежести: ``hz``, ``age_s``, ``points``;
  * причина и подсказка словами: нет ROS 2 → «поставь ROS 2»; ROS 2 есть, но
    топик никто не публикует → «запусти драйвер»; драйвер публикует, а скана
    нет → «смотри состояние супервизора»;
  * кнопка «Пересканировать» в панели «Лидар» уходит в сервис ``~/rescan``
    супервизора: он заново подбирает драйвер, а не только порт.

Сам узел ROS 2 создаётся в фоновом потоке (:meth:`RosLidar.start`), а вся
логика состояния — чистые методы, поэтому проверяется тестами без ROS 2.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from typing import Any, Callable, Optional

from . import ros_env

log = logging.getLogger("robot_control.ros_lidar")

#: Топик состояния супервизора драйверов (пакет ``zapret_lidar``).
DEFAULT_STATUS_TOPIC = ros_env.SUPERVISOR_STATUS_TOPIC

#: Через сколько секунд без точек скан считается оборвавшимся.
SCAN_TIMEOUT_S = 2.0

#: Имя сервиса «пересканировать» у супервизора (``~/rescan`` от узла
#: ``lidar_supervisor``).
RESCAN_SERVICE = "/lidar_supervisor/rescan"


def _hz(stamps: list[float]) -> float:
    """Частота по отметкам времени сообщений (0.0, если их мало)."""
    if len(stamps) < 2:
        return 0.0
    span = stamps[-1] - stamps[0]
    if span <= 0:
        return 0.0
    return round((len(stamps) - 1) / span, 1)


class RosLidar:
    """Подписка на скан в ROS 2 и состояние для веб-панели.

    ``on_scan`` вызывается при каждом сообщении (им ``LidarManager`` складывает
    точки в общий скан), ``topic`` — топик скана, ``status_topic`` — топик
    состояния супервизора.
    """

    def __init__(
        self,
        on_scan: Optional[Callable[..., int]] = None,
        *,
        topic: Optional[str] = None,
        status_topic: str = DEFAULT_STATUS_TOPIC,
        frame: str = "lens",
    ) -> None:
        self.topic = topic or os.environ.get("RC_LIDAR_ROS_TOPIC") or ros_env.DEFAULT_SCAN_TOPIC
        self.status_topic = status_topic or os.environ.get("RC_LIDAR_ROS_STATUS_TOPIC") \
            or DEFAULT_STATUS_TOPIC
        self.on_scan = on_scan
        self.available = False          # ROS 2 есть и rclpy импортируется
        self.started = False            # узел поднят и крутится
        self.error = ""                 # последняя ошибка запуска узла
        self.scan_at = 0.0              # время последнего сообщения скана
        self.points = 0                 # точек в последнем скане
        self.stamps: list[float] = []   # отметки времени сканов (для частоты)
        self.publishers = 0             # сколько узлов публикуют топик скана
        self.publisher_names: list[str] = []
        self.supervisor: dict[str, Any] = {}
        self.supervisor_at = 0.0
        self.rescan_requested_at = 0.0
        self.rescan_result = ""
        self._lock = threading.Lock()
        self._node: Any = None
        self._rescan_client: Any = None
        self._thread: Optional[threading.Thread] = None
        self._frame = frame

    # ------------------------------ состояние ------------------------------ #

    def age(self) -> Optional[float]:
        """Сколько секунд назад пришёл последний скан (None — скана не было)."""
        if not self.scan_at:
            return None
        return round(time.time() - self.scan_at, 2)

    def fresh(self, timeout: float = SCAN_TIMEOUT_S) -> bool:
        """Идёт ли скан прямо сейчас."""
        age = self.age()
        return age is not None and age < timeout

    def hz(self) -> float:
        with self._lock:
            stamps = list(self.stamps[-30:])
        return _hz(stamps)

    def feed_laserscan(
        self,
        ranges: list[float],
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float = 0.05,
        range_max: float = 12.0,
    ) -> int:
        """Принимает поля ``sensor_msgs/LaserScan`` и отдаёт их менеджеру."""
        count = 0
        if self.on_scan is not None:
            count = self.on_scan(
                ranges=ranges,
                angle_min=angle_min,
                angle_max=angle_max,
                angle_increment=angle_increment,
                range_min=range_min,
                range_max=range_max,
                source_label="ros",
            )
        else:
            count = sum(
                1 for value in ranges
                if value is not None and math.isfinite(value) and range_min <= value <= range_max
            )
        now = time.time()
        with self._lock:
            self.scan_at = now
            self.points = count
            self.stamps.append(now)
            if len(self.stamps) > 60:
                del self.stamps[:-60]
        return count

    def feed_supervisor_json(self, text: str) -> bool:
        """Разбирает состояние супервизора (``std_msgs/String`` с JSON из пакета)."""
        if not text:
            return False
        try:
            data = json.loads(text)
        except (ValueError, TypeError):
            return False
        if not isinstance(data, dict) or data.get("backend") != "zapret_lidar":
            # Чужой JSON в этом топике: не выдаём его за состояние своего лидара.
            return False
        self.supervisor = data
        self.supervisor_at = time.time()
        return True

    def note_publishers(self, count: int, names: Optional[list[str]] = None) -> None:
        """Сколько узлов публикуют топик скана (видно из rclpy ``count_publishers``)."""
        self.publishers = int(count)
        self.publisher_names = list(names or [])[:8]

    def supervisor_fresh(self, timeout: float = 5.0) -> bool:
        """Свежее ли состояние супервизора (он публикует его раз в секунду)."""
        return bool(self.supervisor_at) and (time.time() - self.supervisor_at) < timeout

    # -------------------------------- причины ------------------------------ #

    def summary(self) -> str:
        """Короткая причина для панели и подписи под картой."""
        if self.fresh():
            return "скан идёт из ROS 2 %s: %d точек, %.1f Гц" % (
                self.topic, self.points, self.hz() or 0.0)
        if not self.available:
            return "ROS 2 недоступен: %s" % (self.error or ros_env.missing_reason())
        if not self.started:
            return "ROS 2 есть, но подписка на %s не поднялась%s" % (
                self.topic, (": " + self.error) if self.error else "")
        if self.supervisor_fresh():
            supervisor = self.supervisor
            state = str(supervisor.get("state") or "")
            reason = str(supervisor.get("reason") or "")
            driver = supervisor.get("driver_title") or supervisor.get("driver") or ""
            port = supervisor.get("port") or ""
            details = ", ".join(part for part in (driver, port) if part)
            text = "драйвер ROS 2: %s" % (reason or state or "состояние неизвестно")
            if details:
                text += " (%s)" % details
            return "скан ещё не идёт — " + text
        if self.age() is not None:
            # Скан уже приходил: «оборвался» точнее, чем «драйвер не видит лидар» —
            # оператор должен искать причину в драйвере и порту, а не в настройках.
            return ("ROS 2: скан в %s оборвался %.1f с назад — драйвер лидара остановился"
                    % (self.topic, self.age() or 0.0))
        if self.publishers:
            return ("ROS 2: %d узел(ов) публикуют %s, но точек нет — драйвер не видит лидар"
                    % (self.publishers, self.topic))
        return ("ROS 2 есть, но %s никто не публикует — драйвер лидара не запущен" % self.topic)

    def hint(self) -> str:
        """Что делать: команда запуска, установка пакета или адрес сервиса."""
        if self.fresh():
            return ""
        if not self.available:
            return ros_env.missing_reason()
        supervisor = self.supervisor if self.supervisor_fresh() else {}
        if supervisor:
            hint = str(supervisor.get("hint") or "")
            if hint:
                return hint
            tried = supervisor.get("tried") or []
            if isinstance(tried, list) and tried:
                notes = "; ".join(
                    "%s %s — %s" % (item.get("driver"), item.get("port") or "", item.get("note"))
                    for item in tried[-3:] if isinstance(item, dict)
                )
                if notes:
                    return "попытки запуска: " + notes
        if self.publishers:
            return ("драйвер публикует топик, но без точек: проверь порт лидара "
                    "(права dialout, занятость порта) и запусти %s"
                    % ros_env.LAUNCH_COMMAND)
        return ("драйвер лидара не запущен: он стартует сам с ярлыком «Запустить робот»; "
                "вручную — %s (пакеты ROS 2 — %s)"
                % (ros_env.LAUNCH_COMMAND, ros_env.SETUP_COMMAND))

    def status(self) -> dict[str, Any]:
        """Блок ``ros`` для ``GET /api/lidar``: состояние, топик, супервизор."""
        supervisor = dict(self.supervisor) if self.supervisor_fresh() else {}
        return {
            "available": bool(self.available),
            "started": bool(self.started),
            "topic": self.topic,
            "status_topic": self.status_topic,
            "connected": self.fresh(),
            "points": self.points,
            "hz": self.hz(),
            "age_s": self.age(),
            "publishers": self.publishers,
            "publisher_names": list(self.publisher_names),
            "supervisor": supervisor or None,
            "supervisor_age_s": (round(time.time() - self.supervisor_at, 1)
                                 if self.supervisor_at else None),
            "rescan_service": RESCAN_SERVICE,
            "launch": ros_env.launch_command(
                model=os.environ.get("RC_LIDAR_PROTOCOL") or "auto",
                port=os.environ.get("RC_LIDAR_PORT") or "",
                baud=os.environ.get("RC_LIDAR_BAUD") or "",
            ),
            "error": self.error or None,
            "reason": self.summary(),
            "hint": self.hint(),
        }

    # --------------------------- узел ROS 2 (rclpy) ------------------------ #

    def start(self) -> bool:
        """Поднимает подписку в фоновом потоке. False — ROS 2 недоступен.

        Вызывать можно несколько раз: повторный вызов ничего не делает, пока
        поток жив. Ошибки не пробрасываются: приложение должно работать и
        показывать причину, а не падать без лидара.
        """
        if self._thread is not None and self._thread.is_alive():
            return self.started
        try:
            ros_env.import_rclpy()
        except ros_env.RosUnavailable as exc:
            self.error = str(exc)
            self.available = False
            log.info("Лидар: ROS 2 недоступен — %s", exc)
            return False
        self.available = True
        self.error = ""
        self._thread = threading.Thread(target=self._spin, daemon=True,
                                        name="lidar-ros-bridge")
        self._thread.start()
        return True

    def _spin(self) -> None:
        """Создаёт узел, подписки и крутит spin до остановки процесса."""
        try:
            rclpy = ros_env.import_rclpy()
            # Сначала init, потом узел: Node() без инициализированного контекста
            # падает («context is invalid») — подписка не поднималась никогда.
            if not rclpy.ok():
                rclpy.init(args=None)
            node = self._build_node(rclpy)
            self._node = node
            self.started = True
            log.info("Лидар: подписан на ROS 2 %s (состояние драйвера — %s)",
                     self.topic, self.status_topic)
            rclpy.spin(node)
        except Exception as exc:  # noqa: BLE001 — rclpy может упасть по десятку причин
            self.started = False
            self.error = "%s: %s" % (type(exc).__name__, exc)
            log.warning("Лидар: подписка на ROS 2 %s не удалась: %s", self.topic, exc)
        finally:
            self.started = False

    def _build_node(self, rclpy: Any):
        """Собирает узел ROS 2 с подписками на скан, состояние и таймером."""
        from rclpy.node import Node
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String

        bridge = self

        # Скан публикуется как sensor data: BEST_EFFORT — так же, как у драйверов
        # лидаров (надёжная доставка точек не нужна, свежесть важнее).
        qos_scan = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        class ZapretLidarBridge(Node):  # pragma: no cover — нужен установленный rclpy
            def __init__(self) -> None:
                super().__init__("zapret_lidar_bridge")
                self.subscription = self.create_subscription(
                    LaserScan, bridge.topic, self._on_scan, qos_scan)
                self.create_subscription(String, bridge.status_topic,
                                         self._on_status, 1)
                self.create_timer(1.0, self._on_tick)

            def _on_scan(self, msg) -> None:
                try:
                    bridge.feed_laserscan(
                        list(msg.ranges), float(msg.angle_min), float(msg.angle_max),
                        float(msg.angle_increment), float(msg.range_min),
                        float(msg.range_max))
                except Exception as exc:  # noqa: BLE001 — одно плохое сообщение не повод падать
                    log.debug("Лидар: сообщение /scan не разобрано: %s", exc)

            def _on_status(self, msg) -> None:
                bridge.feed_supervisor_json(getattr(msg, "data", ""))

            def _on_tick(self) -> None:
                try:
                    count = self.count_publishers(bridge.topic)
                    names = [str(info.node_name) for info in
                             self.get_publishers_info_by_topic(bridge.topic)]
                    bridge.note_publishers(count, names)
                except Exception:  # noqa: BLE001 — счётчики могут быть недоступны
                    pass

        node = ZapretLidarBridge()
        try:
            from std_srvs.srv import Trigger
            self._rescan_client = node.create_client(Trigger, RESCAN_SERVICE)
        except Exception:  # noqa: BLE001 — без std_srvs просто нет кнопки «пересканировать»
            self._rescan_client = None
        return node

    def request_rescan(self) -> bool:
        """Просит супервизор заново подобрать драйвер (кнопка «Пересканировать»)."""
        self.rescan_requested_at = time.time()
        client = self._rescan_client
        if client is None:
            self.rescan_result = "супервизор ROS 2 не найден (запусти %s)" % ros_env.LAUNCH_COMMAND
            return False
        try:
            from std_srvs.srv import Trigger
            if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.5):
                self.rescan_result = "супервизор ROS 2 не отвечает (запусти %s)" % ros_env.LAUNCH_COMMAND
                return False
            future = client.call_async(Trigger.Request())
            # Ответ ждём коротко: перебор драйверов занимает секунды, а панель
            # не должна висеть на кнопке.
            deadline = time.time() + 1.5
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done() and future.result() is not None:
                self.rescan_result = str(future.result().message)
                return True
            self.rescan_result = "супервизор принял запрос на пересканирование"
            return True
        except Exception as exc:  # noqa: BLE001 — сервис чужой/недоступен
            self.rescan_result = "не удалось вызвать сервис: %s" % exc
            return False

    def stop(self) -> None:
        """Останавливает узел (нужен только тестам и перезапуску подписки)."""
        node, self._node = self._node, None
        self.started = False
        self._rescan_client = None
        if node is None:
            return
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
