# Copyright 2026 Zapret robot control
#
# Таблица ROS 2-драйверов лидара и планирование запуска: какие пакеты умеют
# отдавать sensor_msgs/LaserScan, как их запускать и в каком порядке пробовать.
#
# Модуль намеренно без зависимостей от rclpy: он собирает план (и печатает его
# командой ``ros2 run zapret_lidar supervisor_node --print-plan``), поэтому его
# можно проверить тестами и на машине без ROS 2.
"""Драйверы лидара и план запуска."""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional

#: Топик скана: так его называют все драйверы лидаров (sensor_msgs/LaserScan).
DEFAULT_SCAN_TOPIC = "/scan"

#: Топик состояния супервизора (std_msgs/String, JSON): по нему веб-панель
#: показывает, что именно происходит с драйвером.
DEFAULT_STATUS_TOPIC = "/zapret/lidar/status"


@dataclass(frozen=True)
class Driver:
    """Один ROS 2-драйвер лидара: пакет, исполняемый файл, параметры."""

    name: str                       # наш ключ: rplidar, lds01, ld06, opencr…
    title: str                      # человекочитаемое имя устройства
    package: str                    # ROS 2-пакет (проверяется в ament-индексе)
    executable: str                 # имя узла в lib/<package>/
    protocols: tuple[str, ...]      # каким RC_LIDAR_PROTOCOL соответствует
    params: dict[str, Any] = field(default_factory=dict)   # --ros-args -p key:=value
    args: tuple[str, ...] = ()      # позиционные аргументы (например -i /dev/ttyACM0)
    baud: int = 0                   # скорость порта (0 — у драйвера своя)
    needs_port: bool = True         # нужен ли драйверу последовательный порт
    alt_executables: tuple[str, ...] = ()   # прошлые/новые имена того же узла
    param_file: str = ""            # yaml с параметрами, если драйвер так устроен
    note: str = ""                  # подсказка оператору
    apt: str = ""                   # apt-имя пакета (ros-{d}-…)
    source: str = ""                # откуда брать, если пакета нет в apt


#: Порядок перебора при ``model:=auto``. LDS-01 (лидар TurtleBot3 Burger) идёт
#: первым: на этом роботе он основной, и он же чаще всего остаётся без скана при
#: подборе «вручную» по USB. ``opencr`` — вариант «лидар подключён к плате
#: OpenCR»: тогда отдельного USB-устройства лидара нет вовсе, и скан отдаёт
#: turtlebot3_node. ``sllidar`` — альтернативный драйвер RPLIDAR (Slamtec),
#: если основной rplidar_ros не завёлся.
DRIVER_ORDER: tuple[str, ...] = (
    "lds01", "rplidar", "ld06", "ld19", "stl27l", "opencr", "sllidar",
)

DRIVERS: dict[str, Driver] = {
    "lds01": Driver(
        name="lds01",
        title="LDS-01/LDS-02 (TurtleBot3 Burger, HLS-LFCD2)",
        package="hls_lfcd_lds_driver",
        executable="hlds_laser_publisher",
        protocols=("lds01",),
        params={"port": "{port}", "frame_id": "{frame_id}"},
        baud=230400,
        apt="ros-{d}-hls-lfcd-lds-driver",
        source="https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver",
        note="кадры 0xFA, 230400 бод; драйвер сам запускает вращение",
    ),
    "rplidar": Driver(
        name="rplidar",
        title="RPLIDAR (A1/A2/A3/S1/C1)",
        package="rplidar_ros",
        executable="rplidar_composition",
        alt_executables=("rplidar_node",),
        protocols=("rplidar",),
        params={
            "serial_port": "{port}",
            "serial_baudrate": "{baud}",
            "frame_id": "{frame_id}",
            "angle_compensate": True,
        },
        apt="ros-{d}-rplidar-ros",
        source="https://github.com/Slamtec/rplidar_ros",
        note="скорость зависит от модели: 115200 (A1/A2M8), 256000 (A3/S1), 460800 (C1)",
    ),
    "sllidar": Driver(
        name="sllidar",
        title="RPLIDAR через sllidar_ros2",
        package="sllidar_ros2",
        executable="sllidar_node",
        protocols=("rplidar",),
        params={
            "serial_port": "{port}",
            "serial_baudrate": "{baud}",
            "frame_id": "{frame_id}",
            "angle_compensate": True,
        },
        # В rosdistro sllidar_ros2 нет — пакета ros-<d>-sllidar-ros2 не существует.
        apt="",
        source="https://github.com/Slamtec/sllidar_ros2",
        note="альтернативный драйвер Slamtec: полезен, если rplidar_ros не заводится",
    ),
    "ld06": Driver(
        name="ld06",
        title="LD06/LD19 (D500 и клоны)",
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        protocols=("ld06", "ld19"),
        params={
            "product_name": "LDLiDAR_LD06",
            "port_name": "{port}",
            "port_baudrate": 230400,
            "topic_name": "{topic}",
            "frame_id": "{frame_id}",
        },
        baud=230400,
        source="https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2",
        note="односторонний поток кадров 0x54 0x2C на 230400, команд не принимает",
    ),
    "ld19": Driver(
        name="ld19",
        title="LD19/LD06 (та же плата, другое имя модели)",
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        protocols=("ld19", "ld06"),
        params={
            "product_name": "LDLiDAR_LD19",
            "port_name": "{port}",
            "port_baudrate": 230400,
            "topic_name": "{topic}",
            "frame_id": "{frame_id}",
        },
        baud=230400,
        source="https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2",
    ),
    "stl27l": Driver(
        name="stl27l",
        title="STL-27L",
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        protocols=("stl27l", "ld06"),
        params={
            "product_name": "LDLiDAR_STL27L",
            "port_name": "{port}",
            "port_baudrate": 921600,
            "topic_name": "{topic}",
            "frame_id": "{frame_id}",
        },
        baud=921600,
        source="https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2",
    ),
    "opencr": Driver(
        name="opencr",
        title="LDS-01 через плату OpenCR (TurtleBot3) без отдельного USB-лидара",
        package="turtlebot3_node",
        executable="turtlebot3_ros",
        protocols=("lds01", "opencr"),
        args=("-i", "{port}"),
        params={},
        param_file="{prefix}/share/turtlebot3_bringup/param/{turtlebot3_model}.yaml",
        apt="ros-{d}-turtlebot3-node",
        source="https://github.com/ROBOTIS-GIT/turtlebot3",
        note="скан отдаёт сама плата робота (OpenCR): отдельного устройства лидара "
             "в /dev/ttyUSB* нет, порт — это OpenCR (/dev/ttyACM0)",
    ),
}

#: Пакеты, которых нет в apt: их собирают из исходников (scripts/ros2_setup.sh
#: клонирует и компилирует их сам).
SOURCE_ONLY: tuple[str, ...] = ("ldlidar_stl_ros2", "sllidar_ros2")

#: Драйверы, которые scripts/ros2_setup.sh ставит без дополнительных настроек
#: (остальные — по RC_ROS_DRIVERS или выбранной модели лидара).
SETUP_DEFAULT: tuple[str, ...] = ("hls_lfcd_lds_driver", "rplidar_ros", "ldlidar_stl_ros2",
                                  "turtlebot3_node")

#: Синонимы протоколов из RC_LIDAR_PROTOCOL/--model → ключ драйвера.
MODEL_ALIASES: dict[str, str] = {
    "auto": "auto",
    "rplidar": "rplidar", "rplidar_ros": "rplidar", "sllidar": "sllidar",
    "lds": "lds01", "lds-01": "lds01", "lds_01": "lds01", "lds1": "lds01",
    "lds02": "lds01", "lds-02": "lds01", "lds2": "lds01",
    "hls": "lds01", "hls-lfcd2": "lds01", "hls_lfcd2": "lds01", "lfcd2": "lds01",
    "turtlebot": "lds01", "turtlebot3": "lds01", "lds01": "lds01",
    "ld06": "ld06", "ld-06": "ld06", "d500": "ld06", "ld19": "ld19", "ld-19": "ld19",
    "stl27l": "stl27l", "stl-27l": "stl27l",
    "opencr": "opencr", "burger": "opencr", "waffle": "opencr",
}

#: Порт платы OpenCR у TurtleBot3 по умолчанию: лидар подключён к ней.
OPENC_R_PORT = "/dev/ttyACM0"


def driver_names() -> tuple[str, ...]:
    """Ключи драйверов в порядке перебора."""
    return DRIVER_ORDER


def get_driver(name: str) -> Optional[Driver]:
    """Драйвер по имени или синониму протокола (``lds-01``, ``turtlebot3``…)."""
    key = MODEL_ALIASES.get((name or "").strip().lower(), (name or "").strip().lower())
    return DRIVERS.get(key)


def drivers_for_protocol(protocol: str) -> list[Driver]:
    """Все драйверы, умеющие этот протокол (для RC_LIDAR_PROTOCOL=rplidar и т. п.)."""
    key = (protocol or "auto").strip().lower()
    if key in ("", "auto", "opencr"):
        return [DRIVERS[name] for name in DRIVER_ORDER]
    wanted = MODEL_ALIASES.get(key, key)
    out: list[Driver] = []
    for name in DRIVER_ORDER:
        driver = DRIVERS[name]
        if name == wanted or wanted in driver.protocols:
            out.append(driver)
    return out


def default_port_globs() -> tuple[str, ...]:
    """Где искать лидар, если порт не задан: постоянные имена, затем USB-порты."""
    return (
        "/dev/serial/by-id/*lidar*",
        "/dev/serial/by-id/*Lidar*",
        "/dev/rplidar*",
        "/dev/serial/by-id/*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    )


def parse_ports(spec: str | Iterable[str] | None) -> list[str]:
    """Разворачивает строку/список портов (можно glob) в существующие пути."""
    if spec is None:
        chunks: list[str] = []
    elif isinstance(spec, str):
        chunks = [item.strip() for item in spec.replace(";", ",").split(",")]
    else:
        chunks = [str(item).strip() for item in spec]
    chunks = [item for item in chunks if item]
    if not chunks:
        chunks = list(default_port_globs())
    out: list[str] = []
    for chunk in chunks:
        matches = sorted(glob.glob(chunk)) if any(ch in chunk for ch in "*?[") else [chunk]
        for match in matches:
            if os.path.exists(match) and match not in out:
                out.append(match)
    return out


def reserved_ports() -> set[str]:
    """Порты, которые лидару отдавать нельзя: мастер руления, шасси, BMS.

    Те же переменные, что и в веб-приложении (``robot_control/lidar.py``):
    лидар не должен перебирать порт мастера или аккумулятора.
    """
    names = ("RC_SLAM_PORT", "RC_BMS_PORT")
    values: list[str] = []
    for name in names:
        raw = (os.environ.get(name) or "").strip()
        if raw:
            values.append(raw)
    for name in ("RC_SLAM_PORTS", "RC_LIDAR_EXCLUDE"):
        raw = os.environ.get(name) or ""
        values.extend(item.strip() for item in raw.replace(";", ",").split(","))
    out: set[str] = set()
    for value in values:
        if not value:
            continue
        for path in parse_ports(value) or [value]:
            out.add(path)
            try:
                out.add(os.path.realpath(path))
            except OSError:
                pass
    return out


def candidate_ports(spec: str | Iterable[str] | None = None) -> list[str]:
    """Порты-кандидаты для лидара: без портов мастера/шасси/BMS и дублей."""
    blocked = reserved_ports()
    out: list[str] = []
    for port in parse_ports(spec):
        if port in blocked or os.path.realpath(port) in blocked:
            continue
        out.append(port)
    return out


def installed_packages(under: Iterable[Path]) -> set[str]:
    """Пакеты ROS 2 из ament-индекса перечисленных префиксов."""
    names: set[str] = set()
    for base in under:
        index = Path(base) / "share" / "ament_index" / "resource_index" / "packages"
        try:
            for entry in index.iterdir():
                if entry.is_file():
                    names.add(entry.name)
        except OSError:
            continue
    return names


@dataclass
class PlanItem:
    """Одна попытка: какой драйвер на каком порту запускать и почему."""

    driver: Driver
    port: str
    baud: int
    score: float = 0.0
    note: str = ""
    installed: bool = True

    @property
    def key(self) -> str:
        return "%s@%s" % (self.driver.name, self.port or "-")

    def as_dict(self) -> dict[str, Any]:
        return {
            "driver": self.driver.name,
            "title": self.driver.title,
            "package": self.driver.package,
            "port": self.port,
            "baud": self.baud,
            "installed": self.installed,
            "score": round(self.score, 3),
            "note": self.note or self.driver.note,
        }


def build_plan(
    *,
    model: str = "auto",
    ports: str | Iterable[str] | None = None,
    installed: Optional[set[str]] = None,
    scores: Optional[dict[str, float]] = None,
    only_first_port: bool = False,
) -> list[PlanItem]:
    """Порядок попыток запуска драйверов лидара.

    ``model`` — ``auto`` или имя драйвера/синоним протокола (``lds01``,
    ``rplidar``, ``turtlebot3``…). ``installed`` — набор ROS 2-пакетов: драйверы
    без своего пакета всё равно попадают в план, но помечаются ``installed:
    False`` — оператор видит, чего не хватает, а не «лидар не найден».
    ``scores`` — результат быстрой проверки порта (:mod:`zapret_lidar.frames`):
    чем выше, тем раньше драйвер попробуют.
    """
    scores = scores or {}
    port_list = candidate_ports(ports)
    # Порт OpenCR — не лидар: если отдельного USB-устройства лидара нет, а порт
    # робота есть, вариант «лидар на плате» проверяется тоже.
    opencr_ports = [port for port in port_list if port.startswith("/dev/ttyACM")]
    plan: list[PlanItem] = []
    for driver in drivers_for_protocol(model):
        is_installed = True if installed is None else driver.package in installed
        driver_ports: list[str]
        if driver.needs_port:
            driver_ports = list(port_list)
            if driver.name == "opencr":
                # Драйвер платы робота: свой порт у него есть и без лидара.
                extra = [port for port in opencr_ports if port not in driver_ports]
                primary = [OPENC_R_PORT] if os.path.exists(OPENC_R_PORT) else []
                driver_ports = list(dict.fromkeys(primary + driver_ports + extra))
        else:
            driver_ports = [""]
        for port in driver_ports:
            baud = driver.baud
            score = 0.0
            for protocol in driver.protocols:
                score = max(score, scores.get("%s@%s" % (protocol, port), 0.0))
            if not is_installed:
                # Пакета нет — такая попытка не сработает, но её место в плане
                # сохраняем: в статусе видно, что именно надо доустановить.
                score -= 100.0
            plan.append(PlanItem(driver=driver, port=port, baud=baud,
                                 score=score + _static_priority(driver),
                                 installed=is_installed))
        if only_first_port and driver.needs_port and driver_ports:
            break
    plan.sort(key=lambda item: -item.score)
    return plan


def _static_priority(driver: Driver) -> float:
    """Небольшая добавка к оценке: порядок моделей по частоте на роботах."""
    try:
        index = DRIVER_ORDER.index(driver.name)
    except ValueError:
        return 0.0
    return (len(DRIVER_ORDER) - index) * 0.01


def driver_command(
    driver: Driver,
    *,
    port: str = "",
    baud: int = 0,
    topic: str = DEFAULT_SCAN_TOPIC,
    frame_id: str = "base_scan",
    prefix: Optional[Path] = None,
    install: Optional[Path] = None,
    executable: Optional[Path] = None,
) -> list[str]:
    """Команда запуска драйвера: путь к исполняемому файлу или ``ros2 run``.

    Прямой путь (``lib/<package>/<executable>``) предпочтительнее: не нужен
    ``ros2`` CLI, а вывод драйвера сразу попадает в журнал супервизора.
    """
    rate = baud or driver.baud
    file_path = str(executable) if executable is not None else ""
    if file_path:
        argv = [file_path]
    else:
        argv = ["ros2", "run", driver.package, driver.executable]
    for arg in driver.args:
        argv.append(_substitute(arg, port=port, baud=rate, topic=topic, frame_id=frame_id,
                                prefix=prefix, driver=driver))
    pairs: list[tuple[str, str]] = []
    for key, value in driver.params.items():
        pairs.append((key, _substitute(str(value), port=port, baud=rate, topic=topic,
                                       frame_id=frame_id, prefix=prefix, driver=driver)))
    param_file = driver.param_file
    if param_file:
        path = _substitute(param_file, port=port, baud=rate, topic=topic,
                           frame_id=frame_id, prefix=prefix, driver=driver)
        if os.path.exists(path):
            # Узел платы робота (turtlebot3_node) читает параметры из yaml.
            argv += ["--ros-args", "--params-file", path]
    if pairs and param_file and os.path.exists(
            _substitute(param_file, port=port, baud=rate, topic=topic,
                        frame_id=frame_id, prefix=prefix, driver=driver)):
        return argv
    if pairs:
        argv.append("--ros-args")
        for key, value in pairs:
            argv += ["-p", "%s:=%s" % (key, value)]
    return argv


def _substitute(text: str, **context: Any) -> str:
    """Подставляет ``{port}``, ``{baud}``, ``{topic}``, ``{frame_id}``, ``{prefix}``."""
    prefix = context.get("prefix")
    driver = context.get("driver")
    mapping = {
        "port": context.get("port") or "",
        "baud": str(context.get("baud") or ""),
        "topic": context.get("topic") or DEFAULT_SCAN_TOPIC,
        "frame_id": context.get("frame_id") or "base_scan",
        "prefix": str(prefix) if prefix else "/opt/ros",
        "turtlebot3_model": (os.environ.get("TURTLEBOT3_MODEL") or "burger"),
    }
    if driver is not None and not mapping["port"]:
        mapping["port"] = "/dev/ttyACM0" if driver.name == "opencr" else "/dev/ttyUSB0"
    try:
        return text.format(**mapping)
    except (KeyError, IndexError):
        return text


def find_executable(
    driver: Driver,
    *,
    prefix: Optional[Path] = None,
    install: Optional[Path] = None,
) -> Optional[Path]:
    """Ищет исполняемый файл драйвера в собранном ws и в ROS 2.

    Возвращает ``None``, если файла нет — тогда запуск идёт через ``ros2 run``.
    """
    names = (driver.executable, *driver.alt_executables)
    bases = [item for item in (install, prefix) if item is not None]
    for base in bases:
        for name in names:
            for tail in (Path("lib") / driver.package / name,
                         Path("lib") / driver.package / ("%s.py" % name)):
                candidate = base / tail
                if candidate.is_file() and os.access(candidate, os.X_OK):
                    return candidate
    return None


def install_lines(driver: Driver, distro: str = "humble") -> list[str]:
    """Готовые команды установки драйвера (apt или сборка из исходников)."""
    lines: list[str] = []
    if driver.apt:
        lines.append("sudo apt install %s" % driver.apt.format(d=distro))
    elif driver.source:
        setup = "./scripts/ros2_setup.sh"
        if driver.package not in SETUP_DEFAULT:
            setup = "RC_ROS_DRIVERS=%s %s" % (driver.package, setup)
        lines.append("%s (сам скачает %s и скомпилирует colcon-ом)" % (setup, driver.package))
        lines.append("вручную: git clone %s ros2_ws/src/%s && colcon build --symlink-install "
                     "--packages-select %s" % (driver.source, driver.package, driver.package))
    return lines
