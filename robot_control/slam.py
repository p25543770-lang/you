"""Веб-интерфейс RUS SLAM внутри пульта оператора.

Интерфейс взят из https://github.com/danilka-revin/rus_slam (каталог ``gui/``)
и лежит в репозитории в ``slam_gui/`` — робот работает без интернета, поэтому
подтягивать его откуда-то при запуске нельзя.

Здесь мы:
  * импортируем ``slam_gui/backend.py`` как модуль (он написан только на
    стандартной библиотеке, pyserial и ROS подключаются по желанию);
  * выбираем источник данных: ROS 2 → UART-модули → симуляция;
  * отдаём страницы и JSON API через Flask, под общей авторизацией пульта.

Страницы:  ``/`` и ``/main.html`` — основной экран робота,
           ``/console`` и ``/index.html`` — инженерный пульт.
JS обращается к API относительными путями (``fetch('api/state')``), поэтому
консоль живёт строго на ``/console`` без завершающего слэша.
"""

from __future__ import annotations

import glob
import importlib.util
import logging
import os
import re
import time
from pathlib import Path

from flask import Blueprint, jsonify, request, send_from_directory

log = logging.getLogger("robot_control.slam")

GUI_DIR = Path(__file__).resolve().parent.parent / "slam_gui"
BACKEND_PY = GUI_DIR / "backend.py"

MAIN_PAGE = "main.html"
CONSOLE_PAGE = "index.html"

#: Статика интерфейса: имя файла → endpoint. Отдаётся только этот список.
ASSETS = {
    "main.css": "main_css",
    "main.js": "main_js",
    "styles.css": "styles_css",
    "app.js": "app_js",
    "console.js": "console_js",
    "console-core.js": "console_core_js",
    "vision.js": "vision_js",
    # наша приборная оснастка: частота опроса, тренды, диагностика обмена
    "instrument.js": "instrument_js",
    # наш скрипт: пробрасывает токен сессии в подзапросы (нужен в iframe без cookie)
    "slam_auth.js": "slam_auth_js",
}

#: Порты, которые проверяем в режиме ``auto``, если список не задан явно.
DEFAULT_PORTS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]

_backend = None


def load_backend():
    """Загружает vendored ``backend.py`` (один раз на процесс)."""
    global _backend
    if _backend is not None:
        return _backend
    if not BACKEND_PY.is_file():
        raise RuntimeError(f"не найден {BACKEND_PY} — интерфейс RUS SLAM не установлен")
    spec = importlib.util.spec_from_file_location("rus_slam_backend", str(BACKEND_PY))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _backend = module
    return module


def _have(module_name: str) -> bool:
    try:
        __import__(module_name)
    except ImportError:
        return False
    return True


def _present_ports(ports):
    return [p for p in ports if os.path.exists(p)]


def _autodetect_ports():
    """Реально существующие UART-адаптеры робота."""
    found = _present_ports(DEFAULT_PORTS)
    if found:
        return found
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found
    return []


def make_source(mode: str = "auto", ports=None):
    """Источник данных робота.

    ``auto``: ROS 2, если есть rclpy; иначе UART-модули, если есть pyserial и
    живой порт; иначе симуляция — интерфейс работает и на пустом столе.
    """
    mod = load_backend()
    mode = (mode or "auto").strip().lower()

    if mode == "sim":
        return mod.SimSource()
    if mode == "serial":
        return mod.SerialSource(list(ports or DEFAULT_PORTS))
    if mode == "ros":
        return mod.RosSource()

    if mode != "auto":
        log.warning("неизвестный RC_SLAM_SOURCE=%r — использую auto", mode)

    if _have("rclpy"):
        log.info("источник данных: ROS 2")
        return mod.RosSource()

    live = _present_ports(list(ports or [])) or _autodetect_ports()
    if _have("serial") and live:
        log.info("источник данных: UART-модули %s", ", ".join(live))
        return mod.SerialSource(live)

    log.info("источник данных: симуляция (железо не найдено)")
    return mod.SimSource()


def build_state(mode="auto", ports=None, lock_file=None, pin=None,
                max_attempts=None, lock_sec=None):
    """Собирает ``App`` из backend.py: источник данных + PIN-хранилище."""
    mod = load_backend()
    source = make_source(mode, ports)
    path = str(lock_file) if lock_file else str(GUI_DIR / "state" / "lock.json")
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    vault = mod.Vault(
        path,
        pin or mod.DEFAULT_PIN,
        max_attempts=max_attempts or mod.MAX_ATTEMPTS,
        lock_ms=int((lock_sec if lock_sec is not None else mod.LOCK_MS / 1000.0) * 1000),
    )
    return mod.App(source, vault)


def _send(name: str):
    response = send_from_directory(GUI_DIR, name)
    response.headers["Cache-Control"] = "no-store"
    return response


def create_blueprint(state, guard=None) -> Blueprint:
    """Маршруты интерфейса RUS SLAM.

    ``guard`` — декоратор авторизации пульта; без него маршруты открыты
    (используется в тестах самого интерфейса).
    """
    bp = Blueprint("slam", __name__)
    guard = guard or (lambda view: view)

    # ------------------------------ страницы ----------------------------- #
    @bp.get("/", endpoint="main")
    @guard
    def main_screen():
        """Основной экран робота."""
        return _send(MAIN_PAGE)

    bp.add_url_rule("/main.html", endpoint="main_page", view_func=guard(main_screen))

    @bp.get("/console", endpoint="console")
    @guard
    def console():
        """Инженерный пульт."""
        return _send(CONSOLE_PAGE)

    bp.add_url_rule("/index.html", endpoint="console_page", view_func=guard(console))

    # -------------------------------- API -------------------------------- #
    @bp.get("/api/state")
    @guard
    def api_state():
        return jsonify({"ok": True, "data": state.snapshot()})

    @bp.get("/api/audit")
    @guard
    def api_audit():
        raw = request.args.get("limit", "20") or ""
        match = re.search(r"\d+", raw)
        limit = max(1, min(200, int(match.group()))) if match else 20
        return jsonify(
            {
                "ok": True,
                "audit": state.vault.audit(limit),
                "integrity": state.vault.verify_audit(),
            }
        )

    @bp.get("/api/health")
    @guard
    def api_health():
        return jsonify(
            {
                "ok": True,
                "source": getattr(state.source, "name", "?"),
                "uptimeSec": int(time.time() - state.started),
                "pinDefault": state.vault.default_pin,
            }
        )

    @bp.post("/api/lock/open")
    @guard
    def api_lock_open():
        body = request.get_json(silent=True) or {}
        result = state.open_lock(str(body.get("pin", "")))
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/lock/close")
    @guard
    def api_lock_close():
        return jsonify(state.close_lock())

    @bp.post("/api/lock/pin")
    @guard
    def api_lock_pin():
        body = request.get_json(silent=True) or {}
        result = state.vault.set_pin(str(body.get("current", "")), str(body.get("new", "")))
        if result.get("ok"):
            state.close_lock()
        return jsonify(result), 200 if result.get("ok") else 403

    # ------------------------------- статика ------------------------------ #
    # Без авторизации намеренно: в средах без cookie (iframe-предпросмотр)
    # подзапросы не могут передать сессию, и страница приезжала сломанной.
    # Секретов в CSS/JS нет; данные и страницы по-прежнему закрыты.
    for filename, endpoint in ASSETS.items():
        bp.add_url_rule(
            f"/{filename}",
            endpoint=endpoint,
            view_func=lambda name=filename: _send(name),
        )

    return bp
