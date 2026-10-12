"""Отдельное приложение для дисплея на борту робота (без авторизации).

Корень / и /main.html сразу отдают экран киоска с АКБ, шасси и лидаром.
Никакого логина, никакого пульта оператора, никаких эндпоинтов управления.

Порт по умолчанию — 8090 (переменная ``RC_KIOSK_PORT``). Запуск одной
командой — ``./run_kiosk.py`` или ``./scripts/kiosk.sh``: сервер поднимается на
8090 и сам открывает экран в браузере в киоск-режиме (без рамок, на весь
экран). Если браузер закрыли — экран открывается заново, чтобы на дисплее
робота всегда была телеметрия.
"""
from __future__ import annotations

import argparse
import logging
import os
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import replace
from pathlib import Path
from typing import Mapping, Sequence

from flask import Flask, jsonify, redirect
from werkzeug.serving import make_server

from robot_control.config import PROJECT_ROOT, Config
from robot_control.slam import (
    GUI_DIR,
    KIOSK_PAGE,
    KIOSK_URL_PATH,
    build_state,
)

log = logging.getLogger("robot_control.kiosk")

#: Порт бортового экрана. 8080 занят пультом оператора, поэтому киоск живёт рядом.
DEFAULT_PORT = 8090

#: Браузеры, которые умеют киоск-режим (первый найденный и используется).
BROWSER_CANDIDATES = (
    "chromium-browser",
    "chromium",
    "google-chrome",
    "google-chrome-stable",
    "microsoft-edge",
    "firefox",
)

#: Сколько секунд ждать, пока сервер ответит, прежде чем открывать браузер.
HEALTH_TIMEOUT_SEC = 20.0
#: Пауза перед повторной попыткой, если браузер не найден.
BROWSER_RETRY_SEC = 30.0
#: Пауза перед повторным открытием закрытого браузера.
BROWSER_REOPEN_SEC = 2.0


# ---------------------------------------------------------------- окружение --- #
def _env_flag(name: str, default: bool) -> bool:
    """Флаг из окружения: 1/true/yes/on → True, 0/false/no/off → False."""
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on", "да"}


def kiosk_port() -> int:
    """Порт бортового экрана (``RC_KIOSK_PORT``, по умолчанию 8090)."""
    raw = os.environ.get("RC_KIOSK_PORT", "").strip()
    try:
        return int(raw) if raw else DEFAULT_PORT
    except ValueError:
        log.warning("RC_KIOSK_PORT=%r не число — беру %s", raw, DEFAULT_PORT)
        return DEFAULT_PORT


def display_available(env: Mapping[str, str] | None = None) -> bool:
    """Есть ли графическая сессия: только тогда браузер имеет смысл открывать."""
    env = os.environ if env is None else env
    return bool(env.get("WAYLAND_DISPLAY") or env.get("DISPLAY"))


def find_browser(explicit: str | None = None) -> str | None:
    """Путь к браузеру для киоск-режима (``RC_KIOSK_BROWSER`` переопределяет)."""
    candidate = (explicit or os.environ.get("RC_KIOSK_BROWSER") or "").strip()
    if candidate:
        path = Path(candidate).expanduser()
        if path.is_file() and os.access(path, os.X_OK):
            return str(path)
        found = shutil.which(candidate)
        if found:
            return found
        # Ошиблись в имени — экран всё равно должен открыться: ищем сами.
        log.warning("RC_KIOSK_BROWSER=%s не найден — ищу браузер сам", candidate)
    for name in BROWSER_CANDIDATES:
        found = shutil.which(name)
        if found:
            return found
    return None


def browser_command(exe: str, url: str) -> list[str]:
    """Команда запуска браузера без рамок и адресной строки (киоск-режим)."""
    base = os.path.basename(exe).lower()
    if "firefox" in base:
        return [exe, "--kiosk", url]
    return [
        exe,
        "--kiosk",
        "--app=" + url,
        "--noerrdialogs",
        "--disable-infobars",
        "--autoplay-policy=no-user-gesture-required",
    ]


# ------------------------------------------------------------------- сервер -- #
def create_kiosk_app(config: Config | None = None) -> Flask:
    config = config or Config.from_env()
    app = Flask(__name__)
    app.config["SECRET_KEY"] = config.secret_key or secrets.token_urlsafe(48)

    # Только источник данных — никакой авторизации, никаких операторов.
    try:
        state = build_state(
            mode=config.slam_source,
            ports=config.slam_ports,
            lock_file=config.slam_lock_file,
            pin=config.slam_pin,
            max_attempts=config.slam_max_attempts,
            lock_sec=config.slam_lock_seconds,
        )
        log.info("киоск: интерфейс подключён (источник: %s)",
                 getattr(state.source, "name", "?"))
    except Exception as exc:  # noqa: BLE001
        log.error("киоск: не удалось подключить источник: %s", exc)
        state = None

    def _send(name: str):
        from flask import send_from_directory
        response = send_from_directory(GUI_DIR, name)
        response.headers["Cache-Control"] = "no-store"
        return response

    # ---------- маршруты ----------
    @app.get("/")
    def root():
        return redirect(KIOSK_URL_PATH)

    @app.get(KIOSK_URL_PATH)
    def kiosk():
        return _send(KIOSK_PAGE)

    @app.get("/main.html")
    @app.get("/robot.html")
    def kiosk_alt():
        return _send(KIOSK_PAGE)

    @app.get("/api/state")
    @app.get("/api/kiosk_state")
    def api_state():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        snap = state.snapshot()
        # Только публичные показатели — АКБ, шасси, лидар, скорость/мощность/связь.
        return jsonify({"ok": True, "data": {
            "battery": snap.get("battery"),
            "motors": snap.get("motors"),
            "lidar": snap.get("lidar"),
            "speedMps": snap.get("speedMps"),
            "powerKw": snap.get("powerKw"),
            "mode": snap.get("mode"),
            "linkOk": snap.get("linkOk"),
            "ts": snap.get("ts"),
        }})

    @app.get("/healthz")
    def healthz():
        return jsonify({
            "ok": True,
            "mode": "kiosk",
            "source": getattr(getattr(state, "source", None), "name", None) if state else None,
        })

    # статика (robot.css, robot.js)
    for filename in ("robot.css", "robot.js"):
        # нужен closure, чтобы цикл не переиспользовал последнее значение
        def _view(name=filename):
            return _send(name)
        endpoint = f"asset_{filename.replace('.', '_')}"
        app.add_url_rule(f"/{filename}", endpoint=endpoint, view_func=_view)

    # заголовки безопасности (разрешены iframe для превью)
    @app.after_request
    def _headers(response):
        response.headers.setdefault("X-Content-Type-Options", "nosniff")
        response.headers.setdefault("Referrer-Policy", "no-referrer")
        response.headers.setdefault("X-Robots-Tag", "noindex, nofollow")
        response.headers.setdefault("Cache-Control", "no-store")
        return response

    return app


# ------------------------------------------------------------ открытие экрана -- #
def kiosk_url(port: int | None = None, host: str = "127.0.0.1") -> str:
    """Адрес бортового экрана (для браузера на самом роботе — 127.0.0.1)."""
    return f"http://{host}:{port or kiosk_port()}{KIOSK_URL_PATH}"


def port_in_use(port: int, host: str = "127.0.0.1", timeout: float = 0.5) -> bool:
    """Слушает ли кто-то порт уже сейчас (тогда сервер поднимать не надо)."""
    with socket.socket() as sock:
        sock.settimeout(timeout)
        try:
            sock.connect((host, port))
        except OSError:
            return False
    return True


def wait_for_server(base_url: str, timeout: float = HEALTH_TIMEOUT_SEC) -> bool:
    """Ждёт, пока сервер ответит на /healthz (браузеру нельзя открываться раньше)."""
    deadline = time.monotonic() + max(0.0, timeout)
    while True:
        try:
            with urllib.request.urlopen(base_url + "/healthz", timeout=1.0) as response:
                if response.status == 200:
                    return True
        except (urllib.error.URLError, OSError):
            pass
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.25)


def _display_worker(url: str, base_url: str, watch: bool,
                    stop_event: threading.Event) -> None:
    """Открывает экран в браузере и, если ``watch``, держит его открытым."""
    if not wait_for_server(base_url):
        log.warning("сервер не ответил за %.0f с — открываю %s как есть",
                    HEALTH_TIMEOUT_SEC, url)
    while not stop_event.is_set():
        browser = find_browser()
        if browser is None:
            log.warning(
                "браузер не найден — откройте %s вручную "
                "(поставить: sudo apt install chromium-browser)", url)
            if not watch or stop_event.wait(BROWSER_RETRY_SEC):
                return
            continue
        try:
            proc = subprocess.Popen(
                browser_command(browser, url),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            log.error("не удалось открыть браузер %s: %s", browser, exc)
            return
        log.info("экран робота открыт в браузере (%s): %s", os.path.basename(browser), url)
        if not watch:
            return
        # Ждём закрытия браузера и открываем заново: экран всегда на месте.
        while not stop_event.wait(1.0):
            if proc.poll() is not None:
                log.info("браузер закрыт — открываю экран заново")
                if stop_event.wait(BROWSER_REOPEN_SEC):
                    return
                break
        else:
            return


def open_kiosk_screen(port: int | None = None, *, watch: bool = True,
                      stop_event: threading.Event | None = None) -> threading.Thread:
    """Открыть бортовой экран в отдельном потоке (сервер не блокируется)."""
    port = port or kiosk_port()
    base_url = f"http://127.0.0.1:{port}"
    thread = threading.Thread(
        target=_display_worker,
        args=(kiosk_url(port), base_url, watch, stop_event or threading.Event()),
        name="kiosk-display",
        daemon=True,
    )
    thread.start()
    return thread


# -------------------------------------------------------------- точка входа --- #
def _pid_file() -> Path:
    return PROJECT_ROOT / "var" / "kiosk.pid"


def _write_pid_file() -> None:
    try:
        path = _pid_file()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{os.getpid()}\n", encoding="utf-8")
    except OSError as exc:  # не критично: файл нужен только для stop.sh
        log.debug("не удалось записать %s: %s", _pid_file(), exc)


def _remove_pid_file() -> None:
    try:
        _pid_file().unlink()
    except OSError:
        pass


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="run_kiosk.py",
        description="Бортовой экран робота (порт 8090): телеметрия без авторизации",
    )
    parser.add_argument("--host", default=None,
                        help="адрес прослушивания (по умолчанию RC_BIND_HOST или 0.0.0.0)")
    parser.add_argument("--port", type=int, default=None,
                        help=f"порт экрана (по умолчанию RC_KIOSK_PORT или {DEFAULT_PORT})")
    parser.add_argument("--source", choices=["auto", "sim", "serial", "ros"], default=None,
                        help="источник данных: auto (ROS → UART → sim), sim, serial, ros")
    parser.add_argument("--open", dest="open_screen", action="store_true", default=None,
                        help="открыть экран в браузере (по умолчанию — если есть дисплей)")
    parser.add_argument("--no-open", dest="open_screen", action="store_false",
                        help="только сервер, браузер не открывать")
    parser.add_argument("--watch", dest="watch", action="store_true", default=None,
                        help="снова открывать экран, если браузер закрыли (по умолчанию — да)")
    parser.add_argument("--no-watch", dest="watch", action="store_false",
                        help="не переоткрывать браузер")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    # Настройки (заодно подхватываем .env — например RC_KIOSK_OPEN).
    # По умолчанию — реальные данные (auto: ROS → UART → мастер).
    # Симуляция включается явно: RC_SLAM_SOURCE=sim ./run_kiosk.py
    cfg = Config.from_env()
    if args.source:
        cfg = replace(cfg, slam_source=args.source)

    host = args.host or cfg.bind_host
    port = args.port or kiosk_port()
    url = kiosk_url(port)
    base_url = f"http://127.0.0.1:{port}"

    open_screen = args.open_screen
    if open_screen is None:
        open_screen = _env_flag("RC_KIOSK_OPEN", display_available())
    watch = args.watch
    if watch is None:
        watch = open_screen and _env_flag("RC_KIOSK_WATCHDOG", True)
    if open_screen and not display_available():
        log.warning("дисплей не найден (нет DISPLAY/WAYLAND_DISPLAY) — "
                    "экран будет доступен по адресу %s", url)

    stop_event = threading.Event()

    # Сервер поднимаем сами, если порт свободен: так сначала связываем сокет,
    # а браузер открываем уже по живому адресу.
    server = None
    if port_in_use(port):
        log.info("порт %s уже обслуживается (сервер уже запущен) — сервер не поднимаю", port)
    else:
        try:
            server = make_server(host, port, create_kiosk_app(cfg), threaded=True)
        except OSError as exc:
            log.error("не удалось занять порт %s: %s", port, exc)
            return 1

    if server is None:
        # Порт уже занят: экран отдаёт кто-то другой (служба robot-kiosk или
        # прошлый запуск). Наша задача — просто показать его на дисплее.
        print(f"Экран робота: {url} (сервер уже запущен отдельно)")
        if not open_screen:
            return 0
        open_kiosk_screen(port, watch=watch, stop_event=stop_event)
        try:
            while not stop_event.wait(1.0):
                pass
        except KeyboardInterrupt:
            pass
        stop_event.set()
        return 0

    if open_screen:
        open_kiosk_screen(port, watch=watch, stop_event=stop_event)

    _write_pid_file()
    log.info("экран робота (киоск) слушает http://%s:%s/ · источник: %s",
             host, port, cfg.slam_source)
    log.info("на роботе: http://127.0.0.1:%s%s · без авторизации", port, KIOSK_URL_PATH)
    log.info("Ctrl+C — остановить сервер")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("остановлено")
    finally:
        stop_event.set()
        server.server_close()
        _remove_pid_file()
    return 0


if __name__ == "__main__":
    sys.exit(main())
