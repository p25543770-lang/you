# Copyright 2026 Zapret robot control
#
# Супервизор драйвера лидара: сам выбирает ROS 2-пакет под подключённый лидар,
# запускает его, следит за свежестью /scan и публикует состояние для веб-панели.
"""Узел ROS 2 ``lidar_supervisor``: запуск и присмотр за драйвером лидара."""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import drivers as drv
from .probe import probe_ports, scores_for_plan

#: Как часто пересматривать план, если ничего не получается (с).
RETRY_PAUSE_S = 5.0
#: Сколько ждать сообщение /scan после запуска драйвера, прежде чем считать
#: попытку неудачной (с). LDS-01 раскручивается ~1–2 с, RPLIDAR — быстрее.
DEFAULT_PROBE_TIMEOUT_S = 8.0
#: Через сколько секунд без точек считать драйвер вставшим (с).
DEFAULT_SCAN_TIMEOUT_S = 3.0


def now() -> float:
    return time.time()


class ScanWatch:
    """Свежесть скана: сколько точек, когда последний раз и с какой частотой."""

    def __init__(self) -> None:
        self.at = 0.0
        self.points = 0
        self.stamps: list[float] = []

    def feed(self, points: int) -> None:
        self.at = now()
        self.points = points
        self.stamps.append(self.at)
        if len(self.stamps) > 30:
            del self.stamps[:-30]

    def hz(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        if span <= 0:
            return 0.0
        return round((len(self.stamps) - 1) / span, 1)

    def age(self) -> Optional[float]:
        return None if not self.at else round(now() - self.at, 2)


class LidarSupervisor(Node):
    """Держит один драйвер лидара работающим и рассказывает о нём всем желающим."""

    def __init__(self) -> None:
        super().__init__("lidar_supervisor")
        self.declare_parameter("model", "auto")
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 0)
        self.declare_parameter("scan_topic", drv.DEFAULT_SCAN_TOPIC)
        self.declare_parameter("status_topic", drv.DEFAULT_STATUS_TOPIC)
        self.declare_parameter("frame_id", "base_scan")
        self.declare_parameter("probe_timeout", DEFAULT_PROBE_TIMEOUT_S)
        self.declare_parameter("scan_timeout", DEFAULT_SCAN_TIMEOUT_S)
        self.declare_parameter("autostart", True)
        self.declare_parameter("autoprobe", True)
        self.declare_parameter("tick", 1.0)

        self.model = str(self.get_parameter("model").value or "auto")
        self.port_spec = str(self.get_parameter("port").value or "")
        self.baud_override = int(self.get_parameter("baud").value or 0)
        self.scan_topic = str(self.get_parameter("scan_topic").value or "/scan")
        self.status_topic = str(self.get_parameter("status_topic").value
                                or drv.DEFAULT_STATUS_TOPIC)
        self.frame_id = str(self.get_parameter("frame_id").value or "base_scan")
        self.probe_timeout = float(self.get_parameter("probe_timeout").value or 8.0)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value or 3.0)
        self.autostart = bool(self.get_parameter("autostart").value)
        self.autoprobe = bool(self.get_parameter("autoprobe").value)
        tick = float(self.get_parameter("tick").value or 1.0)

        self.watch = ScanWatch()
        self.plan: list[drv.PlanItem] = []
        self.plan_index = 0
        self.state = "starting"
        self.reason = "запускаюсь"
        self.hint = ""
        self.tried: list[dict[str, Any]] = []
        self.probes: dict[str, dict[str, Any]] = {}
        self.installed = drv.installed_packages(self._prefixes())
        self.process: Optional[subprocess.Popen] = None
        self.driver: Optional[drv.Driver] = None
        self.driver_port = ""
        self.driver_baud = 0
        self.started_at = 0.0
        self.restarts = 0
        self._next_retry = 0.0
        self._lock = threading.Lock()
        self._stop = False

        qos_scan = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_scan)
        self.publisher = self.create_publisher(String, self.status_topic, 10)
        self.create_service(Trigger, "~/rescan", self._on_rescan)
        self.create_timer(max(0.2, tick), self._tick)
        self.get_logger().info(
            "супервизор лидара: топик %s, модель %s, порты: %s"
            % (self.scan_topic, self.model, ", ".join(self._ports()) or "автопоиск")
        )

    # ------------------------------ утилиты -------------------------------- #

    def _prefixes(self) -> list[Path]:
        out: list[Path] = []
        for value in (os.environ.get("RC_ROS_PREFIX"),
                      os.environ.get("ROS_PREFIX"),
                      os.environ.get("AMENT_PREFIX_PATH")):
            for item in (value or "").split(os.pathsep):
                if item.strip():
                    out.append(Path(item.strip()))
        for distro in (os.environ.get("ROS_DISTRO"), "humble", "jazzy"):
            if distro:
                out.append(Path("/opt/ros") / distro)
        try:
            out.extend(sorted(path for path in Path("/opt/ros").iterdir() if path.is_dir()))
        except OSError:
            pass
        seen: list[Path] = []
        for path in out:
            if path not in seen and path.is_dir():
                seen.append(path)
        return seen

    def _ros_prefix(self) -> Optional[Path]:
        prefixes = self._prefixes()
        # Сам ROS 2 — тот префикс, где есть rclpy: после source ros2_ws первыми в
        # AMENT_PREFIX_PATH идут пакеты рабочего пространства (zapret_lidar и
        # драйверы из исходников), и «дистрибутивом» оказывался zapret_lidar.
        for path in prefixes:
            if (path / "share" / "ament_index" / "resource_index" / "packages" / "rclpy").is_file():
                return path
        for path in prefixes:
            if (path / "setup.bash").is_file() or (path / "share").is_dir():
                return path
        return None

    def _install_prefix(self) -> Optional[Path]:
        raw = os.environ.get("RC_ROS_WS_INSTALL") or ""
        path = Path(raw) if raw else None
        if path and path.is_dir():
            return path
        for entry in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
            candidate = Path(entry or ".")
            if (candidate / "lib").is_dir() and candidate.name != "ros":
                return candidate
        return None

    def _ports(self) -> list[str]:
        return drv.candidate_ports(self.port_spec or None)

    def ros2_cli(self) -> Optional[str]:
        return shutil.which("ros2")

    # --------------------------- план и запуск ----------------------------- #

    def refresh_plan(self) -> None:
        """Пересобирает план: порты, установленные пакеты, быстрая проверка порта."""
        ports = self._ports()
        self.installed = drv.installed_packages(self._prefixes())
        scores: dict[str, float] = {}
        if self.autoprobe and ports:
            # Проверяем порты, только если лидар не задан вручную: на роботе это
            # доли секунды, но порт занимается на время проверки.
            self.probes = probe_ports(ports, limit=4)
            scores = scores_for_plan(self.probes)
        self.plan = drv.build_plan(model=self.model, ports=ports,
                                   installed=self.installed, scores=scores)
        self.plan_index = 0

    def start_next(self) -> None:
        """Запускает следующую попытку из плана (или сообщает, что нечего)."""
        self.stop_process()
        while self.plan_index < len(self.plan):
            item = self.plan[self.plan_index]
            self.plan_index += 1
            if not item.installed:
                self.tried.append({"driver": item.driver.name, "port": item.port,
                                   "note": "пакет %s не установлен" % item.driver.package})
                continue
            if item.driver.needs_port and not item.port:
                self.tried.append({"driver": item.driver.name, "port": "",
                                   "note": "порт не найден"})
                continue
            self.launch(item)
            return
        self.state = "failed"
        self.reason = self._failure_reason()
        self.hint = self._failure_hint()

    def launch(self, item: drv.PlanItem) -> None:
        """Запускает драйвер попытки и переводит состояние в «проверяю»."""
        prefix = self._ros_prefix()
        install = self._install_prefix()
        executable = drv.find_executable(item.driver, prefix=prefix, install=install)
        if executable is None:
            # Драйвер, скомпилированный из исходников, лежит в своём каталоге
            # ros2_ws/install/<пакет>: ищем во всех префиксах AMENT_PREFIX_PATH.
            for base in self._prefixes():
                executable = drv.find_executable(item.driver, install=base)
                if executable is not None:
                    break
        if executable is None and self.ros2_cli() is None:
            self.tried.append({"driver": item.driver.name, "port": item.port,
                               "note": "нет ни %s в lib/, ни команды ros2" % item.driver.executable})
            return
        argv = drv.driver_command(item.driver, port=item.port, baud=item.baud,
                                  topic=self.scan_topic, frame_id=self.frame_id,
                                  prefix=prefix, install=install, executable=executable)
        env = dict(os.environ)
        env.setdefault("TURTLEBOT3_MODEL", "burger")
        try:
            self.process = subprocess.Popen(  # noqa: S603 — argv собран из таблицы
                argv, env=env, start_new_session=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
            )
        except OSError as exc:
            self.tried.append({"driver": item.driver.name, "port": item.port,
                               "note": "не запустился: %s" % exc})
            self.process = None
            return
        self.driver = item.driver
        self.driver_port = item.port
        self.driver_baud = item.baud
        self.started_at = now()
        self.state = "probing"
        self.reason = "запускаю %s" % item.driver.title
        self.hint = ""
        threading.Thread(target=self._drain, args=(self.process,), daemon=True).start()
        self.get_logger().info(
            "пробую %s на %s (%s бод): %s"
            % (item.driver.title, item.port or "—", item.baud or "по умолчанию", " ".join(argv))
        )

    def _drain(self, process: Optional[subprocess.Popen]) -> None:
        """Читает вывод драйвера в журнал супервизора (иначе он копится в трубе)."""
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            text = line.rstrip()
            if text:
                self.get_logger().info("[%s] %s" % (self.driver.package if self.driver else "driver",
                                                    text))

    def stop_process(self) -> None:
        """Останавливает драйвер (вместе с группой процессов: он мог spawn'ить детей)."""
        process, self.process = self.process, None
        if process is None:
            return
        try:
            if process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        finally:
            try:
                if process.stdout is not None:
                    process.stdout.close()
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------ события -------------------------------- #

    def _on_scan(self, msg: LaserScan) -> None:
        points = sum(1 for value in msg.ranges if value and value == value and value < 1e30)
        self.watch.feed(points)
        if self.state != "running":
            self.state = "running"
            self.driver_port = self.driver_port or ""
            self.get_logger().info(
                "скан пошёл: %d точек в %s%s" % (
                    points, self.scan_topic,
                    (" (драйвер %s)" % self.driver.package) if self.driver else ""))
        self.reason = "скан идёт: %d точек, %.1f Гц" % (self.watch.points, self.watch.hz())

    def _on_rescan(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Сервис ``~/rescan``: кнопка «Пересканировать» в веб-панели."""
        self.stop_process()
        self.watch = ScanWatch()
        self.tried = []
        self.refresh_plan()
        self.start_next()
        response.success = True
        response.message = "перебор драйверов перезапущен (состояние: %s)" % self.state
        return response

    def _tick(self) -> None:
        """Раз в тик: присмотр за драйвером, переключение попыток, публикация статуса."""
        if self._stop:
            return
        age = self.watch.age()
        if self.state in ("probing", "running"):
            dead = self.process is None or self.process.poll() is not None
            stale = age is None or age > self.scan_timeout
            elapsed = now() - self.started_at
            if not dead and not stale:
                self.state = "running"
            elif self.state == "probing" and not dead and elapsed < self.probe_timeout:
                # Драйвер работает, но /scan ещё не пошёл: лидару нужно время на
                # раскрутку (LDS-01 — до пары секунд), поэтому ждём таймаут.
                pass
            else:
                note = ("процесс драйвера завершился" if dead
                        else "нет точек %.1f с" % (age if age is not None else 0.0))
                if self.state == "running":
                    # Драйвер работал и встал: это уже не подбор, а сбой —
                    # считаем перезапуски и начинаем попытки заново.
                    self.restarts += 1
                    self.state = "restarting"
                    self.reason = "драйвер встал: %s" % note
                    self.refresh_plan()
                else:
                    self.tried.append({"driver": self.driver.name if self.driver else "?",
                                       "port": self.driver_port,
                                       "note": "нет %s за %.0f с (%s)"
                                               % (self.scan_topic, self.probe_timeout, note)})
                self.stop_process()
                self.watch = ScanWatch()
                if self.plan_index >= len(self.plan):
                    self.refresh_plan()
                self.start_next()
        elif self.state in ("failed", "idle") and now() >= self._next_retry:
            # Ничего не вышло: раз в RETRY_PAUSE_S пересобираем план — лидар могли
            # воткнуть в другой порт, а пакет драйвера — доустановить.
            self._next_retry = now() + RETRY_PAUSE_S
            self.refresh_plan()
            self.start_next()
        self.publish_status()

    # ------------------------------- статус -------------------------------- #

    def status(self) -> dict[str, Any]:
        """Состояние для веб-панели и ``ros2 topic echo /zapret/lidar/status``."""
        prefix = self._ros_prefix()
        missing = sorted({item.driver.package for item in self.plan if not item.installed})
        return {
            "backend": "zapret_lidar",
            "state": self.state,
            "reason": self.reason,
            "hint": self.hint or self._missing_hint(missing),
            "driver": self.driver.name if self.driver else None,
            "driver_package": self.driver.package if self.driver else None,
            "driver_title": self.driver.title if self.driver else None,
            "port": self.driver_port or None,
            "baud": self.driver_baud or None,
            "scan_topic": self.scan_topic,
            "points": self.watch.points,
            "hz": self.watch.hz(),
            "age_s": self.watch.age(),
            "pid": self.process.pid if self.process and self.process.poll() is None else None,
            "restarts": self.restarts,
            "distro": prefix.name if prefix else None,
            "ros_prefix": str(prefix) if prefix else None,
            "installed": sorted(self.installed),
            "missing": missing,
            "tried": self.tried[-8:],
            "probes": self.probes,
            "plan": [item.as_dict() for item in self.plan[:8]],
            "at": now(),
        }

    def _missing_hint(self, missing: list[str]) -> str:
        """Подсказка, если нужного пакета нет: что именно поставить."""
        if not missing:
            return ""
        prefix = self._ros_prefix()
        distro = prefix.name if prefix else "humble"
        lines: list[str] = []
        for package in missing:
            for name, driver in drv.DRIVERS.items():
                if driver.package != package:
                    continue
                commands = drv.install_lines(driver, distro)
                if commands:
                    lines.append(" · ".join(commands))
                break
        return ("нет ROS 2-пакетов драйверов лидара: " + "; ".join(lines)) if lines else ""

    def _failure_reason(self) -> str:
        """Почему не вышло: коротко и по делу, без «лидар не найден» на всё."""
        if not self.installed:
            return "в этом ROS 2 нет ни одного пакета драйвера лидара"
        if not self._ports() and any(not item.installed for item in self.plan):
            return "нет портов-кандидатов и не все пакеты драйверов установлены"
        tried = ", ".join(
            "%s@%s" % (item["driver"], item["port"] or "-") for item in self.tried[-5:]
        )
        return "ни один драйвер не отдал %s (%s)" % (self.scan_topic, tried or "попыток не было")

    def _failure_hint(self) -> str:
        missing = sorted({item.driver.package for item in self.plan if not item.installed})
        hint = self._missing_hint(missing)
        if hint:
            return hint
        extra = ("проверь, что лидар подключён и не занят другой программой "
                 "(в том числе прошлым запуском драйвера), и запусти "
                 "`ros2 topic hz %s` для проверки" % self.scan_topic)
        if self.probes and not any(report.get("protocol") for report in self.probes.values()):
            extra = ("на портах %s не видно ни одного известного лидара: %s"
                     % (", ".join(self.probes), extra))
        return extra

    def publish_status(self) -> None:
        payload = json.dumps(self.status(), ensure_ascii=False)
        message = String()
        message.data = payload
        self.publisher.publish(message)


def _terminate(node: LidarSupervisor, signum: int, _frame: Any) -> None:
    node.get_logger().info("останавливаюсь по сигналу %s" % signum)
    node._stop = True  # noqa: SLF001 — внутренний флаг выхода из spin


def main(argv: Optional[list[str]] = None) -> int:
    """Точка входа узла (``ros2 run zapret_lidar supervisor_node``)."""
    rclpy.init(args=argv)
    node = LidarSupervisor()
    signal.signal(signal.SIGTERM, lambda signum, frame: _terminate(node, signum, frame))
    signal.signal(signal.SIGINT, lambda signum, frame: _terminate(node, signum, frame))
    try:
        if node.autostart:
            node.refresh_plan()
            node.start_next()
        else:
            node.state = "idle"
            node.reason = "автозапуск выключен (autostart:=false) — жду сервис ~/rescan"
        while rclpy.ok() and not node._stop:  # noqa: SLF001
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.stop_process()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001 — второй shutdown в процессе
            pass
    return 0
