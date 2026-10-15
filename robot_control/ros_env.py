"""Мост к ROS 2: rclpy из ``.venv`` проекта и проверка установленных пакетов.

Зачем это отдельным модулем. Лидар на роботе теперь берётся из ROS 2: драйвер
(``rplidar_ros``, ``hls_lfcd_lds_driver``, ``ldlidar_stl_ros2`` или
``turtlebot3_node``) публикует ``sensor_msgs/LaserScan`` в ``/scan``, а пульт и
бортовой экран читают топик. Но приложение запускается интерпретатором из
``.venv`` (там Flask и pyserial, см. ``scripts/ensure_deps.sh``), а ROS 2 ставится
системно в ``/opt/ros/<дистрибутив>``. Обычный ``.venv`` не видит системные
пакеты, поэтому ``import rclpy`` в нём молча падал и ROS-режим лидара **никогда**
не включался — ни на пульте, ни на борту, хотя ``ros2 topic echo /scan`` в
терминале работал. Здесь этот разрыв закрывается: пути ROS 2 добавляются в
``sys.path`` и переменные окружения процесса, после чего ``rclpy`` импортируется
как обычно.

Но ``sys.path`` — только половина: ``rclpy`` — C-расширение, его библиотеки
(``librcl.so`` и др.) загрузчик ищет по ``LD_LIBRARY_PATH``, а его glibc читает
лишь при старте процесса. Поэтому точки входа (``run.py``, ``run_kiosk.py``)
в самом начале зовут :func:`reexec_with_ros_env`: если ROS 2 установлен, а
процесс запущен без его окружения, он перезапускается с ``source setup.bash``
(ровно как в терминале оператора) — и ``import rclpy`` работает по-настоящему.

Второе назначение — диагностика. ``GET /api/lidar`` отдаёт блок ``ros``
(:func:`describe`): какой дистрибутив найден, доступен ли ``rclpy``, какие
пакеты драйверов лидара установлены, идёт ли сейчас автоустановка ROS 2
(``scripts/ros2_setup.sh`` пишет ``var/ros2_setup.json``) и что делать, если
чего-то нет. Оператор видит не «скана нет», а причину с командой.

Настройки (``.env``):

  * ``RC_LIDAR_SOURCE`` — ``auto`` (по умолчанию), ``ros`` или ``serial``;
  * ``RC_LIDAR_ROS_TOPIC`` — топик скана, по умолчанию ``/scan``;
  * ``RC_ROS_PREFIX`` — префикс ROS 2, если он не в ``/opt/ros/<distro>``;
  * ``RC_ROS_REEXEC=0`` — не перезапускать процесс с окружением ROS 2;
  * ``ROS_DISTRO`` — обычная переменная ROS 2, читается как есть.
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import subprocess
import sys
import sysconfig
import threading
import time
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger("robot_control.ros_env")

#: Дистрибутивы ROS 2, которые имеет смысл искать на роботе (от новых к старым).
ROS_DISTROS = ("lyrical", "kilted", "jazzy", "iron", "humble", "rolling", "galactic", "foxy")

#: Где обычно лежит ROS 2, если он поставлен пакетами Ubuntu.
DEFAULT_PREFIX = Path("/opt/ros")

#: Рабочее пространство ROS 2 проекта (пакет ``zapret_lidar``).
WORKSPACE = Path(__file__).resolve().parent.parent / "ros2_ws"

#: Топик скана по умолчанию: так его называют все драйверы лидаров.
DEFAULT_SCAN_TOPIC = "/scan"

#: Топик со состоянием супервизора драйверов (пакет ``zapret_lidar``).
SUPERVISOR_STATUS_TOPIC = "/zapret/lidar/status"

#: Состояние автоустановки ROS 2: его пишет ``scripts/ros2_setup.sh``.
SETUP_STATE_FILE = WORKSPACE.parent / "var" / "ros2_setup.json"

#: Метка «ROS 2 из исходников ещё компилируется» в префиксе (ros2_setup.sh):
#: colcon кладёт setup.bash и при неудачной сборке, такой префикс — не ROS 2.
INCOMPLETE_MARK = ".zapret_build_incomplete"

#: Процесс уже запущен с окружением этого префикса ROS 2 (защита от
#: бесконечного перезапуска в :func:`reexec_with_ros_env`).
ENV_READY_FLAG = "RC_ROS_ENV_READY"

#: ROS-пакеты, которые умеют отдавать ``/scan`` для конкретного лидара.
#: Порядок = порядок предпочтения. Значения сверяются с таблицей драйверов
#: ``ros2_ws/src/zapret_lidar/zapret_lidar/drivers.py`` (это проверяет тест).
LIDAR_DRIVER_PACKAGES: dict[str, tuple[str, ...]] = {
    "lds01": ("hls_lfcd_lds_driver",),
    "rplidar": ("rplidar_ros", "sllidar_ros2"),
    "ld06": ("ldlidar_stl_ros2",),
    "ld19": ("ldlidar_stl_ros2",),
    "stl27l": ("ldlidar_stl_ros2",),
    "opencr": ("turtlebot3_node",),
}

#: apt-имена пакетов драйверов (``{d}`` — дистрибутив: humble, jazzy, …).
#: sllidar_ros2 в apt нет ни для одного дистрибутива (его нет в rosdistro).
APT_PACKAGES: dict[str, str] = {
    "hls_lfcd_lds_driver": "ros-{d}-hls-lfcd-lds-driver",
    "rplidar_ros": "ros-{d}-rplidar-ros",
    "turtlebot3_node": "ros-{d}-turtlebot3-node",
}

#: Пакеты, которых нет в apt: их собирают из исходников в ``ros2_ws/src``
#: (``scripts/ros2_setup.sh`` клонирует и компилирует их сам).
GIT_SOURCES: dict[str, str] = {
    "ldlidar_stl_ros2": "https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2",
    "sllidar_ros2": "https://github.com/Slamtec/sllidar_ros2",
}

#: Подсказка оператору: чем поднять драйвер, если скана нет.
SETUP_COMMAND = "./scripts/ros2_setup.sh"
LAUNCH_COMMAND = "./scripts/ros2_lidar.sh"

#: Сколько секунд держится кэш :func:`describe`. Статус лидара читают оба
#: экрана несколько раз в секунду, а разбор ament-индекса — это обход каталога;
#: раз в 3 с достаточно, чтобы оператор видел изменения сразу после установки
#: пакета.
DESCRIBE_TTL_S = 3.0

_lock = threading.Lock()
_rclpy: Optional[Any] = None
_rclpy_error: str = ""
_prepared_prefix: Optional[Path] = None
_describe_cache: tuple[float, str, dict[str, Any]] = (0.0, "", {})


class RosUnavailable(RuntimeError):
    """ROS 2 или ``rclpy`` недоступны; текст исключения — подсказка оператору."""


# --------------------------------------------------------------------------- #
# Поиск установленного ROS 2
# --------------------------------------------------------------------------- #


def _is_prefix(path: Path) -> bool:
    """Похож ли каталог на префикс ROS 2 (есть setup.bash или библиотеки)."""
    try:
        if not path.is_dir() or (path / INCOMPLETE_MARK).exists():
            return False
    except OSError:
        return False
    return (path / "setup.bash").is_file() or (path / "lib").is_dir() and (
        path / "share" / "ament_index"
    ).is_dir()


def candidate_prefixes() -> list[Path]:
    """Префиксы ROS 2 в порядке проверки: явный, ``ROS_DISTRO``, ``/opt/ros/*``.

    Явный путь (``RC_ROS_PREFIX``) уважается первым: на роботах ROS 2 иногда
    распакован не в ``/opt/ros``, а, например, в домашний каталог.
    """
    out: list[Path] = []

    def add(value: Any) -> None:
        if not value:
            return
        path = Path(str(value)).expanduser()
        if path not in out:
            out.append(path)

    add(os.environ.get("RC_ROS_PREFIX"))
    distro = (os.environ.get("ROS_DISTRO") or "").strip()
    if distro:
        add(DEFAULT_PREFIX / distro)
    for name in ROS_DISTROS:
        add(DEFAULT_PREFIX / name)
    try:
        for path in sorted(DEFAULT_PREFIX.iterdir()):
            if path.is_dir():
                add(path)
    except OSError:
        pass
    for entry in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        add(entry)
    # Собранный ros2_ws — не ROS 2: ensure_python_path() добавляет его в
    # AMENT_PREFIX_PATH, и без этого фильтра он «находился» как префикс ROS 2
    # (дистрибутив «install»), когда самого ROS 2 ещё нет.
    return [path for path in out if not _inside_workspace(path)]


def _inside_workspace(path: Path) -> bool:
    try:
        path.resolve().relative_to(WORKSPACE.resolve())
    except (ValueError, OSError):
        return False
    return True


def prefix() -> Optional[Path]:
    """Первый существующий префикс ROS 2 или ``None``, если ROS 2 не установлен."""
    for path in candidate_prefixes():
        if _is_prefix(path):
            return path
    return None


def install_prefix() -> Optional[Path]:
    """Каталог ``install`` собранного рабочего пространства ``ros2_ws`` (если есть)."""
    for name in ("install",):
        candidate = WORKSPACE / name
        if (candidate / "setup.bash").is_file() or (candidate / "local_setup.bash").is_file():
            return candidate
    return None


def distro() -> Optional[str]:
    """Имя дистрибутива (``humble``, ``jazzy``…): из ``ROS_DISTRO`` или из пути."""
    env = (os.environ.get("ROS_DISTRO") or "").strip()
    if env:
        return env
    path = prefix()
    return path.name if path else None


def site_packages_dirs(root: Path) -> list[Path]:
    """Каталоги с python-пакетами ROS 2 внутри префикса (``rclpy`` и соседи)."""
    version = "%d.%d" % sys.version_info[:2]
    out: list[Path] = []
    for tail in (
        Path("lib") / ("python%s" % version) / "site-packages",
        Path("lib") / "python3" / "dist-packages",
        Path("lib") / "python3" / "site-packages",
    ):
        candidate = root / tail
        if candidate.is_dir() and candidate not in out:
            out.append(candidate)
    return out


def python_path_entries(root: Optional[Path] = None) -> list[str]:
    """Пути, которые нужно добавить в ``sys.path``, чтобы ``import rclpy`` сработал."""
    entries: list[str] = []
    for base in (root, install_prefix()):
        if base is None:
            continue
        for directory in site_packages_dirs(base):
            text = str(directory)
            if text not in entries:
                entries.append(text)
    # Системные пакеты Ubuntu: часть зависимостей ROS 2 ставится apt-ом в python3.
    for extra in ("/usr/lib/python3/dist-packages",):
        if Path(extra).is_dir() and extra not in entries:
            entries.append(extra)
    return entries


def library_dirs(base: Optional[Path]) -> list[str]:
    """Каталоги библиотек префикса: ``lib`` и ``lib/<multiarch>`` (x86_64, aarch64…)."""
    if base is None:
        return []
    tails = ["lib"]
    multiarch = sysconfig.get_config_var("MULTIARCH") or ""
    for name in (multiarch, "x86_64-linux-gnu", "aarch64-linux-gnu", "arm-linux-gnueabihf"):
        if name and "lib/" + name not in tails:
            tails.append("lib/" + name)
    out: list[str] = []
    for tail in tails:
        path = base / tail
        if path.is_dir() and str(path) not in out:
            out.append(str(path))
    return out


def _prepend_env(name: str, values: list[str]) -> None:
    """Добавляет значения в переменную окружения, не дублируя существующие."""
    if not values:
        return
    current = [item for item in (os.environ.get(name) or "").split(os.pathsep) if item]
    merged = [item for item in values if item] + [item for item in current if item not in values]
    os.environ[name] = os.pathsep.join(merged)


def ensure_python_path(root: Optional[Path] = None) -> bool:
    """Готовит процесс к импорту ``rclpy``. True — что-то изменилось.

    Кроме ``sys.path`` выставляются переменные, которые нужны уже загруженным
    библиотекам ROS 2: ``AMENT_PREFIX_PATH`` (поиск пакетов), ``LD_LIBRARY_PATH``
    (``librcl.so`` и типы сообщений), ``ROS_DISTRO`` и ``PYTHONPATH`` для
    дочерних процессов. Без них ``import rclpy`` либо падает, либо работает до
    первого создания узла.
    """
    global _prepared_prefix
    base = root or prefix()
    entries = python_path_entries(base)
    changed = False
    for entry in entries:
        if entry not in sys.path:
            sys.path.insert(0, entry)
            changed = True
    _prepend_env("PYTHONPATH", entries)

    prefixes: list[str] = []
    for item in (base, install_prefix()):
        if item is not None and str(item) not in prefixes:
            prefixes.append(str(item))
    if prefixes:
        _prepend_env("AMENT_PREFIX_PATH", prefixes)
        libs: list[str] = []
        for item in (base, install_prefix()):
            libs.extend(path for path in library_dirs(item) if path not in libs)
        _prepend_env("LD_LIBRARY_PATH", libs)
        if base is not None and not os.environ.get("ROS_DISTRO"):
            os.environ["ROS_DISTRO"] = base.name
    if changed or base != _prepared_prefix:
        _prepared_prefix = base
        changed = True
    return changed


# --------------------------------------------------------------------------- #
# Окружение ROS 2 для нового процесса (LD_LIBRARY_PATH читается только при старте)
# --------------------------------------------------------------------------- #


def sourced_environment(base: Optional[Path] = None, *,
                        timeout: float = 20.0) -> Optional[dict[str, str]]:
    """Окружение после ``source <префикс>/setup.bash`` (и ``ros2_ws``, если собран).

    Ровно то, что делает оператор в терминале: переменные ROS 2 выставляют его
    же скрипты (``PYTHONPATH``, ``LD_LIBRARY_PATH``, ``AMENT_PREFIX_PATH``,
    ``ROS_DISTRO``…), угадывать их по каталогам не нужно. ``set +u`` обязателен:
    скрипты colcon читают неопределённые переменные. ``None`` — нет bash или
    скриптов, либо source не удался.
    """
    base = base or prefix()
    if base is None:
        return None
    install = install_prefix()
    scripts = [path for path in (base / "setup.bash",
                                 (install / "setup.bash") if install else None)
               if path is not None and path.is_file()]
    bash = shutil.which("bash")
    if not scripts or not bash:
        return None
    command = 'set +u; for f in "$@"; do . "$f" >/dev/null 2>&1 || true; done; exec env -0'
    try:
        result = subprocess.run(
            [bash, "-c", command, "ros_env", *[str(path) for path in scripts]],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            timeout=timeout, env=dict(os.environ), check=False)
    except (OSError, subprocess.SubprocessError) as exc:
        log.info("ROS 2: source %s не выполнился: %s", scripts[0], exc)
        return None
    if result.returncode != 0 or not result.stdout:
        return None
    env: dict[str, str] = {}
    for chunk in result.stdout.split(b"\0"):
        key, sep, value = chunk.partition(b"=")
        if not sep or not key:
            continue
        name = key.decode("utf-8", "replace")
        if name in ("_", "SHLVL", "PWD", "OLDPWD") or name.startswith("BASH_FUNC_"):
            continue
        env[name] = value.decode("utf-8", "replace")
    return env or None


def _computed_environment(base: Path) -> dict[str, str]:
    """Запасной путь без bash: те же пути, что выставил бы setup.bash."""
    env = dict(os.environ)

    def prepend(name: str, values: list[str]) -> None:
        current = [item for item in (env.get(name) or "").split(os.pathsep) if item]
        merged = [item for item in values if item] + [item for item in current if item not in values]
        if merged:
            env[name] = os.pathsep.join(merged)

    install = install_prefix()
    prepend("PYTHONPATH", python_path_entries(base))
    prepend("AMENT_PREFIX_PATH", [str(item) for item in (install, base) if item is not None])
    libs: list[str] = []
    for item in (install, base):
        libs.extend(path for path in library_dirs(item) if path not in libs)
    prepend("LD_LIBRARY_PATH", libs)
    if (base / "bin").is_dir():
        prepend("PATH", [str(base / "bin")])
    env.setdefault("ROS_DISTRO", base.name)
    env.setdefault("ROS_VERSION", "2")
    env.setdefault("ROS_PYTHON_VERSION", "3")
    return env


def process_env(base: Optional[Path] = None) -> Optional[dict[str, str]]:
    """Окружение для нового процесса приложения с ROS 2 (``None`` — ROS 2 нет)."""
    base = base or prefix()
    if base is None:
        return None
    env = sourced_environment(base) or _computed_environment(base)
    env[ENV_READY_FLAG] = str(base)
    return env


def reexec_needed() -> bool:
    """Нужно ли перезапустить этот процесс с окружением ROS 2."""
    if (os.environ.get("RC_ROS_REEXEC") or "1").strip().lower() in ("0", "no", "false", "off"):
        return False
    base = prefix()
    if base is None:
        return False
    return os.environ.get(ENV_READY_FLAG) != str(base)


def reexec_with_ros_env(argv: Optional[list[str]] = None) -> bool:
    """Перезапускает текущий процесс Python с окружением ROS 2 — один раз.

    Зовут точки входа (``run.py``, ``run_kiosk.py``) в самом начале, до
    тяжёлых импортов. ROS 2 нет, окружение уже готово или ``RC_ROS_REEXEC=0``
    — ничего не происходит (``False``). Иначе процесс заменяется копией себя
    (``os.execve``) и сюда управление не возвращается. Ошибка перезапуска не
    фатальна: приложение работает дальше, лидар — по USB.
    """
    if not reexec_needed():
        return False
    env = process_env()
    if env is None:
        return False
    args = [sys.executable, *(argv if argv is not None else sys.argv)]
    log.info("ROS 2: перезапускаю процесс с окружением %s", env.get(ENV_READY_FLAG))
    try:
        sys.stdout.flush()
        sys.stderr.flush()
        os.execve(sys.executable, args, env)
    except OSError as exc:  # pragma: no cover — execve почти не падает
        log.warning("ROS 2: перезапуск с окружением не удался: %s", exc)
    return False


# --------------------------------------------------------------------------- #
# Автоустановка (scripts/ros2_setup.sh)
# --------------------------------------------------------------------------- #


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False
    return True


def setup_state() -> dict[str, Any]:
    """Что делает (или чем закончилась) автоустановка ROS 2.

    Читает ``var/ros2_setup.json`` (``RC_ROS_SETUP_STATE`` — другой путь).
    Пустой словарь — установка ещё не запускалась. ``running`` с мёртвым
    процессом превращается в ``interrupted``: установку прервали.
    """
    path = Path(os.environ.get("RC_ROS_SETUP_STATE") or SETUP_STATE_FILE)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    pid = data.get("pid")
    if data.get("state") == "running" and isinstance(pid, int) and pid > 0 and not _pid_alive(pid):
        data["state"] = "interrupted"
    return data


def setup_note(state: Optional[dict[str, Any]] = None) -> str:
    """Одна фраза об автоустановке для подсказки оператору (пусто — нечего сказать)."""
    state = setup_state() if state is None else state
    kind = str(state.get("state") or "")
    text = str(state.get("message") or state.get("phase") or "").strip()
    if kind in ("running", "background"):
        return "сейчас идёт автоустановка ROS 2: %s (ход: %s --status)" % (text or "работает", SETUP_COMMAND)
    if kind == "need_root":
        return ("автоустановке ROS 2 нужен пароль администратора: запусти ярлык "
                "«Запустить робот» (откроет терминал) или %s" % SETUP_COMMAND)
    if kind == "failed":
        return "прошлая автоустановка ROS 2 не удалась: %s" % (text or "см. var/ros2_setup.log")
    if kind == "interrupted":
        return "прошлую автоустановку ROS 2 прервали — она продолжится при следующем запуске робота"
    return ""


def workspace_scripts_dir() -> Optional[Path]:
    """``lib`` собранного рабочего пространства: там лежат исполняемые файлы пакета."""
    install = install_prefix()
    if install is None:
        return None
    for tail in ("lib/zapret_lidar", "lib"):
        path = install / tail
        if path.is_dir():
            return path
    return None


def launch_command(model: str = "auto", port: str = "", baud: str = "") -> str:
    """Команда запуска супервизора драйверов лидара — её показывает статус."""
    parts = ["%s" % LAUNCH_COMMAND]
    if model and model not in ("auto", ""):
        parts.append("--model %s" % model)
    if port:
        parts.append("--port %s" % port)
    if baud:
        parts.append("--baud %s" % baud)
    return " ".join(parts)


# --------------------------------------------------------------------------- #
# rclpy
# --------------------------------------------------------------------------- #


def rclpy_available() -> bool:
    """Импортируется ли ``rclpy`` в этом процессе (после правки путей)."""
    try:
        import_rclpy()
    except RosUnavailable:
        return False
    return True


def import_rclpy():
    """Импортирует ``rclpy``, подготавливая окружение ROS 2.

    Держит модуль в кэше: ``import rclpy`` тяжёлый, а спрашивают его часто
    (статус, старт моста, ``auto``-источник). При отсутствии ROS 2 поднимается
    :class:`RosUnavailable` с готовой подсказкой оператору.
    """
    global _rclpy, _rclpy_error
    if _rclpy is not None:
        return _rclpy
    ensure_python_path()
    try:
        import rclpy  # noqa: PLC0415 — импорт нужен именно в момент вызова
    except Exception as exc:  # noqa: BLE001 — нет rclpy / несовместимая версия python
        _rclpy_error = "%s: %s" % (type(exc).__name__, exc)
        raise RosUnavailable(missing_reason()) from exc
    _rclpy = rclpy
    _rclpy_error = ""
    return rclpy


def rclpy_error() -> str:
    """Текст последней ошибки импорта ``rclpy`` (пусто, если её не было)."""
    return _rclpy_error


def missing_reason(root: Optional[Path] = None) -> str:
    """Подсказка, почему ROS 2 недоступен и что с этим делать."""
    base = root if root is not None else prefix()
    note = setup_note()
    if base is None:
        text = ("ROS 2 не установлен (нет /opt/ros/<дистрибутив>): он докачается и "
                "скомпилируется сам при запуске робота ярлыком «Запустить робот» "
                "или командой %s; лидар пока читается по USB" % SETUP_COMMAND)
        return text + ("; " + note if note else "")
    where = site_packages_dirs(base)
    if not where:
        return ("в префиксе ROS 2 %s нет python-пакетов (lib/python3.*/site-packages): "
                "похоже, поставлен только C++-часть — нужен пакет ros-%s-rclpy "
                "или полный ros-base (%s)" % (base, base.name, SETUP_COMMAND))
    if os.environ.get(ENV_READY_FLAG) != str(base) and _rclpy_error:
        # Процесс стартовал без окружения ROS 2 (например, ROS 2 поставили уже
        # после запуска): библиотеки rclpy загрузчик здесь не найдёт.
        return ("ROS 2 %s есть, но это приложение запущено без его окружения "
                "(LD_LIBRARY_PATH): перезапусти робота ярлыком «Запустить робот» "
                "или ./scripts/stop.sh и снова; последняя ошибка: %s (%s)"
                % (base, _rclpy_error, SETUP_COMMAND))
    return ("rclpy не импортируется из %s (python %s): проверь поставку ROS 2 "
            "(%s%s)" % (base, "%d.%d" % sys.version_info[:2], SETUP_COMMAND,
                        ("; последняя ошибка: " + _rclpy_error) if _rclpy_error else ""))


def reset_cache() -> None:
    """Сбрасывает кэши: нужны тестам и переключению дистрибутива/пакетов."""
    global _rclpy, _rclpy_error, _prepared_prefix, _describe_cache
    with _lock:
        _rclpy = None
        _rclpy_error = ""
        _prepared_prefix = None
        _describe_cache = (0.0, "", {})


# --------------------------------------------------------------------------- #
# Диагностика для веб-панели
# --------------------------------------------------------------------------- #


def installed_packages(root: Optional[Path] = None) -> set[str]:
    """Пакеты ROS 2, видимые в ament-индексе префиксов (по одному файлу на пакет).

    Так видно, поставлен ли пакет драйвера лидара, не запуская его: панель сама
    говорит «нет ros-humble-rplidar-ros», вместо того чтобы оператор искал
    причину в журнале.
    """
    bases = [item for item in (root, install_prefix(), prefix()) if item is not None]
    names: set[str] = set()
    for base in bases:
        index = base / "share" / "ament_index" / "resource_index" / "packages"
        try:
            for entry in index.iterdir():
                if entry.is_file():
                    names.add(entry.name)
        except OSError:
            continue
    return names


#: Синонимы названий лидаров (как их пишут на корпусе и в .env) → наш ключ.
_LIDAR_ALIASES = {
    "lds": "lds01", "lds-01": "lds01", "lds_01": "lds01", "lds1": "lds01",
    "lds02": "lds01", "lds-02": "lds01", "lds2": "lds01",
    "hls": "lds01", "hls-lfcd2": "lds01", "hls_lfcd2": "lds01", "lfcd2": "lds01",
    "turtlebot": "lds01", "turtlebot3": "lds01", "burger": "opencr",
    "rplidar_ros": "rplidar", "sllidar": "rplidar", "ld-06": "ld06",
    "ld-19": "ld19", "d500": "ld06", "stl-27l": "stl27l",
}


def driver_packages(protocol: str) -> tuple[str, ...]:
    """ROS-пакеты, которые умеют отдавать ``/scan`` для этого лидара.

    Понимает синонимы: ``lds-01``, ``hls-lfcd2``, ``turtlebot3`` → ``lds01``;
    ``rplidar_ros`` → ``rplidar``. Так подсказка «что поставить» не зависает на
    пустом ответе, когда лидар назван как в .env.
    """
    key = (protocol or "").strip().lower()
    key = _LIDAR_ALIASES.get(key, key)
    return LIDAR_DRIVER_PACKAGES.get(key, ())


def install_hint(protocol: str = "auto", *, distro_name: Optional[str] = None) -> str:
    """Что поставить, чтобы у лидара этого типа появился ROS 2-драйвер."""
    name = distro_name or distro() or "humble"
    packages = driver_packages(protocol)
    if not packages and protocol in ("", "auto"):
        packages = tuple(dict.fromkeys(
            package for group in LIDAR_DRIVER_PACKAGES.values() for package in group
        ))
    apt: list[str] = []
    git: list[str] = []
    for package in packages:
        template = APT_PACKAGES.get(package)
        if template:
            apt.append(template.format(d=name))
        elif package in GIT_SOURCES:
            git.append(GIT_SOURCES[package])
    chunks: list[str] = []
    if apt:
        chunks.append("apt: sudo apt install " + " ".join(sorted(set(apt))))
    if git:
        chunks.append("из исходников: " + ", ".join(sorted(set(git))))
    chunks.append("затем: %s (сборка пакета zapret_lidar и запуск)" % SETUP_COMMAND)
    return "; ".join(chunks)


def describe(topic: Optional[str] = None) -> dict[str, Any]:
    """Блок ``ros`` для ``GET /api/lidar``: что есть и чего не хватает.

    Никаких догадок: ``available`` — это результат реального ``import rclpy``,
    ``packages`` — содержимое ament-индекса. Он дешёвый (список каталогов), но
    всё равно вызывается раз в секунду, а не на каждый запрос скана.
    """
    scan_topic = topic or os.environ.get("RC_LIDAR_ROS_TOPIC") or DEFAULT_SCAN_TOPIC
    global _describe_cache
    cached_at, cached_topic, cached = _describe_cache
    if cached and cached_topic == scan_topic and (time.time() - cached_at) < DESCRIBE_TTL_S:
        return dict(cached)
    base = prefix()
    packages = installed_packages()
    available = rclpy_available()
    drivers = {
        name: [package for package in group if package in packages]
        for name, group in LIDAR_DRIVER_PACKAGES.items()
    }
    any_driver = any(drivers.values())
    if not available:
        hint = missing_reason(base)
    elif not any_driver:
        hint = ("ROS 2 есть, пакетов драйверов лидара нет: " + install_hint("auto"))
    else:
        hint = ""
    info = {
        "available": available,
        "distro": distro(),
        "prefix": str(base) if base else None,
        "topic": scan_topic,
        "status_topic": SUPERVISOR_STATUS_TOPIC,
        "packages": sorted(packages) if packages else [],
        "drivers": drivers,
        "workspace": str(install_prefix()) if install_prefix() else None,
        "launch": launch_command(),
        "error": rclpy_error() or None,
        "hint": hint,
        "setup": setup_state(),
    }
    _describe_cache = (time.time(), scan_topic, dict(info))
    return info


def summary_line(topic: Optional[str] = None) -> str:
    """Одна строка для журнала запуска: с чем лидар будет работать."""
    info = describe(topic)
    if info["available"]:
        drivers = ", ".join(sorted(name for name, found in info["drivers"].items() if found))
        return ("ROS 2 %s: rclpy доступен, топик %s, драйверы: %s"
                % (info["distro"] or "?", info["topic"], drivers or "не найдены"))
    return "ROS 2: недоступен — %s" % (info["hint"] or missing_reason())
