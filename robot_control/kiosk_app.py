"""Отдельное приложение для дисплея на борту робота.

Корень / и /main.html сразу отдают экран киоска с АКБ, шасси и лидаром.
Телеметрия публична; ручной поворот рулевых модулей защищён PIN и доступен
только источнику MasterSource. Команды движения/тяги этот экран не отправляет.

Порт по умолчанию — 8090 (переменная ``RC_KIOSK_PORT``). Запуск одной
командой — ``./run_kiosk.py`` или ``./scripts/kiosk.sh``: сервер поднимается на
8090 и сам открывает экран в браузере в киоск-режиме (без рамок, на весь
экран). Если браузер закрыли — экран открывается заново, чтобы на дисплее
робота всегда была телеметрия.

Экран при этом открывается строго в одном экземпляре, сколько бы раз ни
запустили команду (автозапуск рабочего стола, ярлык, вручную): повторный
запуск видит, что окно киоска уже открыто, и просто завершается. Для этого
держат flock-замок на время жизни окна, а браузеру дают отдельный профиль —
иначе Chromium просто передаёт адрес уже запущенной копии и сразу выходит,
а сторожевой таймер принимал бы это за «браузер закрыли» и плодил окна.

Firefox с ``--profile`` каталог сам не создаёт. На Ubuntu он ещё и snap:
скрытый ``~/.cache`` и чужой ``/tmp`` ему недоступны. И то и другое
заканчивается окном «Your Firefox profile cannot be loaded. It may be
missing or inaccessible.» Перед запуском профиль создаётся там, где
браузер его действительно прочитает.
"""
from __future__ import annotations

import argparse
import logging
import os
import re
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

try:
    import fcntl  # POSIX (Ubuntu на роботе): межпроцессный замок экрана.
except ImportError:  # pragma: no cover - не-POSIX платформы
    fcntl = None  # type: ignore[assignment]

from flask import Flask, jsonify, redirect, request
from werkzeug.serving import make_server

from robot_control.config import PROJECT_ROOT, Config
from robot_control.slam import (
    GUI_DIR,
    KIOSK_PAGE,
    KIOSK_URL_PATH,
    STEERING_COMMAND_HEADER,
    STEERING_COMMAND_HEADER_VALUE,
    build_state,
    dispatch_steering_command,
    grant_steering_access,
    revoke_steering_access,
    steering_access_valid,
    steering_control_info,
)

log = logging.getLogger("robot_control.kiosk")

#: Порт бортового экрана. 8080 занят пультом оператора, поэтому киоск живёт рядом.
DEFAULT_PORT = 8090

#: Имя каталога профиля киоска — и в кэше, и в ~/snap/<имя>/common.
PROFILE_DIR_NAME = "robot-kiosk-browser"

# Обёртка Ubuntu («/usr/bin/firefox» → snap) и прямой бинарник snap.
_SNAP_BIN_RE = re.compile(rb"/snap/bin/([A-Za-z0-9][A-Za-z0-9._-]*)")
_SNAP_RUN_RE = re.compile(
    rb"\bsnap\s+run\s+(?:--\S+\s+)*([A-Za-z0-9][A-Za-z0-9._-]*)"
)
_FIREFOX_LOCK_PID_RE = re.compile(r"\+(\d+)\s*$")

# Глушит окна первого запуска, которые иначе закрывают телеметрию.
# Пишется только если своего user.js ещё нет — ручные правки не затираем.
_FIREFOX_USER_JS = """\
// Профиль бортового экрана робота. Firefox читает файл при каждом старте.
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("browser.aboutwelcome.enabled", false);
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("startup.homepage_welcome_url", "");
user_pref("startup.homepage_welcome_url.additional", "");
user_pref("browser.startup.firstrunSkipsHomepage", true);
user_pref("trailhead.firstrun.didSeeAboutWelcome", true);
user_pref("datareporting.policy.dataSubmissionEnabled", false);
user_pref("datareporting.healthreport.uploadEnabled", false);
user_pref("toolkit.telemetry.reportingpolicy.firstRun", false);
user_pref("app.normandy.enabled", false);
user_pref("app.normandy.first_run", false);
user_pref("browser.messaging-system.whatsNewPanel.enabled", false);
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("toolkit.startup.max_resumed_crashes", -1);
"""

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
#: Браузер, проживший меньше этого, не «закрыт пользователем», а скорее всего
#: сразу вышел (передал адрес другой копии/упал): пауза перед повтором растёт.
BROWSER_QUICK_EXIT_SEC = 10.0
#: Верхняя граница растущей паузы между быстрыми перезапусками браузера.
BROWSER_REOPEN_MAX_SEC = 30.0
#: Сколько ждать освобождения замка экрана, прежде чем решить, что окно киоска
#: уже открыто другим запуском (небольшая пауза — на случай смены владельца:
#: старый показ как раз завершается, когда стартует новый).
DISPLAY_CLAIM_TIMEOUT_SEC = 3.0


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


def kiosk_profile_dir(env: Mapping[str, str] | None = None) -> Path:
    """Отдельный профиль браузера киоска (``RC_KIOSK_PROFILE_DIR``).

    Свой профиль жизненно важен, чтобы экран не дублировался: без него
    Chromium/Chrome при повторном запуске не открывает новое окно сам, а
    передаёт адрес уже работающей копии браузера и тут же завершается.
    Сторожевой таймер видел «процесс вышел» → «браузер закрыли» → открывал
    экран заново — и так без конца, плодя окна. С отдельным профилем
    запущенный процесс и есть окно киоска: живёт, пока окно открыто.

    Это желаемый путь. Перед запуском ``prepare_browser_profile`` создаёт
    каталог и, если браузер — snap, переносит профиль из скрытого
    ``~/.cache`` в ``~/snap/<имя>/common``: иначе Firefox не может его
    прочитать.
    """
    env = os.environ if env is None else env
    explicit = (env.get("RC_KIOSK_PROFILE_DIR") or "").strip()
    if explicit:
        return Path(explicit).expanduser()
    cache = (env.get("XDG_CACHE_HOME") or "").strip()
    base_dir = Path(cache).expanduser() if cache else Path.home() / ".cache"
    return base_dir / PROFILE_DIR_NAME


def _absolute(path: Path | str) -> Path:
    """Абсолютный путь без «~» и без хвостовых ссылок.

    Firefox с относительным ``--profile`` ищет каталог от своего cwd и,
    не найдя его, показывает то же окно «profile cannot be loaded».
    """
    expanded = os.path.expanduser(str(path))
    try:
        return Path(os.path.realpath(expanded))
    except OSError:
        return Path(os.path.abspath(expanded))


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def snap_name(exe: str) -> str | None:
    """Имя snap-пакета браузера или None, если это обычная сборка.

    На Ubuntu ``firefox`` в PATH — это ``/usr/bin/firefox``: скрипт, который
    запускает ``/snap/bin/firefox`` (или ``snap run firefox``). По одному
    пути этого не видно, а snap не читает скрытые каталоги домашнего
    каталога. Тот же приём у ``chromium-browser``.
    """
    raw = os.path.expanduser(exe)
    candidates = [raw]
    try:
        resolved = os.path.realpath(raw)
    except OSError:
        resolved = raw
    if resolved not in candidates:
        candidates.append(resolved)
    for candidate in candidates:
        name = _snap_name_from_path(candidate)
        if name:
            return name
    try:
        with open(raw, "rb") as handle:
            head = handle.read(8192)
    except OSError:
        return None
    if not head.startswith(b"#!"):
        return None
    match = _SNAP_BIN_RE.search(head) or _SNAP_RUN_RE.search(head)
    if not match:
        return None
    token = match.group(1).decode("ascii", errors="ignore").split(".", 1)[0]
    return token or None


def _snap_name_from_path(path: str) -> str | None:
    parts = path.split("/")
    # ['', 'snap', 'bin', 'firefox'] или ['', 'snap', 'firefox', 'current', ...]
    if len(parts) >= 4 and parts[1] == "snap":
        if parts[2] == "bin" and parts[3]:
            return parts[3].split(".", 1)[0]
        if parts[2] not in {"bin", "core", "snapd"}:
            return parts[2]
    return None


def _snap_can_read(path: Path, snap: str) -> bool:
    """Видит ли строго ограниченный snap этот каталог.

    Домашний интерфейс snap не открывает скрытые каталоги (``.cache``,
    ``.local``) и чужой ``/tmp`` (у snap свой). Гарантированно доступен
    ``~/snap/<имя>/``; обычные нескрытые пути в $HOME — тоже, если подключён
    интерфейс home. Чужой ``~/snap/<другой>/`` — нет.
    """
    home = _absolute(Path.home())
    path = _absolute(path)
    own = home / "snap" / snap
    if _is_relative_to(path, own):
        return True
    if not _is_relative_to(path, home):
        return False
    rel = path.relative_to(home)
    if not rel.parts or rel.parts[0] == "snap":
        return False
    return all(not part.startswith(".") for part in rel.parts)


def _writable_profile(path: Path) -> Path | None:
    """Создать каталог профиля с правами только владельца. None — не вышло."""
    if path.exists() and not path.is_dir():
        log.error("профиль %s существует, но это не каталог — браузер его не откроет", path)
        return None
    try:
        path.mkdir(parents=True, exist_ok=True)
        os.chmod(path, 0o700)
    except OSError as exc:
        log.warning("не удалось создать профиль %s: %s", path, exc)
        return None
    if not os.access(path, os.R_OK | os.W_OK | os.X_OK):
        log.warning("профиль %s недоступен на запись", path)
        return None
    return path


def prepare_browser_profile(exe: str, profile_dir: Path | str | None = None) -> Path:
    """Каталог профиля, который этот браузер сможет открыть. Каталог создаётся.

    Без этого шага Firefox показывает: «Your Firefox profile cannot be
    loaded. It may be missing or inaccessible.» Каталог ``--profile`` должен
    уже существовать, а у snap-сборки он ещё и не может лежать в ``~/.cache``
    или в ``/tmp``.
    """
    requested = _absolute(profile_dir if profile_dir is not None else kiosk_profile_dir())
    snap = snap_name(exe)
    candidates = [requested]
    if snap and not _snap_can_read(requested, snap):
        # Сначала каталог самого snap (виден даже без интерфейса home),
        # затем обычная нескрытая папка в $HOME — её home-интерфейс читает.
        candidates = [
            _absolute(Path.home() / "snap" / snap / "common" / PROFILE_DIR_NAME),
            _absolute(Path.home() / PROFILE_DIR_NAME),
        ]
    chosen: Path | None = None
    for candidate in candidates:
        chosen = _writable_profile(candidate)
        if chosen is not None:
            break
    if chosen is None:
        log.error(
            "не удалось подготовить профиль %s — Firefox покажет: "
            "Your Firefox profile cannot be loaded. It may be missing or inaccessible.",
            requested,
        )
        return candidates[0]
    if chosen != requested:
        log.warning(
            "профиль %s этому браузеру недоступен — беру %s. "
            "Иначе Firefox пишет: Your Firefox profile cannot be loaded. "
            "It may be missing or inaccessible.",
            requested, chosen,
        )
    if "firefox" in os.path.basename(exe).lower():
        _ensure_firefox_profile(chosen)
    return chosen


def _ensure_firefox_profile(profile: Path) -> None:
    """Минимальный профиль: без мастера первого запуска и без мёртвого замка."""
    user_js = profile / "user.js"
    if not user_js.exists():
        try:
            user_js.write_text(_FIREFOX_USER_JS, encoding="utf-8")
            os.chmod(user_js, 0o600)
        except OSError as exc:
            log.warning("не удалось записать %s: %s", user_js, exc)
    _clear_stale_firefox_lock(profile)


def _firefox_lock_pid(lock: Path) -> int | None:
    try:
        raw = os.readlink(lock) if lock.is_symlink() else lock.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    match = _FIREFOX_LOCK_PID_RE.search(raw.strip())
    if not match:
        return None
    return int(match.group(1))


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # процесс есть, просто чужой
    except OSError:
        return False
    return True


def _clear_stale_firefox_lock(profile: Path) -> None:
    """Снять замок упавшего Firefox, чтобы киоск не упёрся в «already running».

    Живой процесс не трогаем: замок с его pid остаётся.
    """
    lock = profile / "lock"
    pid = _firefox_lock_pid(lock)
    if pid is None or _pid_alive(pid):
        return
    for name in ("lock", ".parentlock", "parent.lock"):
        try:
            (profile / name).unlink()
        except FileNotFoundError:
            pass
        except OSError as exc:
            log.debug("не удалось снять замок %s: %s", profile / name, exc)
    log.info("снят устаревший замок профиля Firefox (процесс %s уже не жив)", pid)


def browser_command(exe: str, url: str, profile_dir: Path | str | None = None) -> list[str]:
    """Команда запуска браузера без рамок и адресной строки (киоск-режим).

    Каталог профиля должен уже существовать и быть браузеру виден — это
    делает ``prepare_browser_profile`` (Firefox сам каталог не создаёт).
    """
    profile = str(profile_dir) if profile_dir is not None else str(kiosk_profile_dir())
    base = os.path.basename(exe).lower()
    if "firefox" in base:
        # --no-remote + свой профиль: не цепляться к чужому окну Firefox
        # (иначе процесс-лаунчер сразу выходит, как и у Chromium).
        return [exe, "--kiosk", "--no-remote", "--profile", profile, url]
    return [
        exe,
        "--kiosk",
        "--app=" + url,
        f"--user-data-dir={profile}",
        "--no-first-run",
        "--noerrdialogs",
        "--disable-infobars",
        "--autoplay-policy=no-user-gesture-required",
    ]


# ------------------------------------------------------------------- сервер -- #
def create_kiosk_app(config: Config | None = None) -> Flask:
    config = config or Config.from_env()
    app = Flask(__name__)
    app.config.update(
        SECRET_KEY=config.secret_key or secrets.token_urlsafe(48),
        SESSION_COOKIE_HTTPONLY=True,
        SESSION_COOKIE_SAMESITE=config.cookie_samesite,
        SESSION_COOKIE_SECURE=config.cookie_secure,
    )

    # Телеметрия открыта без логина; поворот модулей доступен только по PIN.
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
    app.extensions["rc_slam"] = state

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
        # Публичны только телеметрия и признак поддержки; команда доступна отдельно по PIN.
        return jsonify({"ok": True, "data": {
            "battery": snap.get("battery"),
            "motors": snap.get("motors"),
            "lidar": snap.get("lidar"),
            "speedMps": snap.get("speedMps"),
            "powerKw": snap.get("powerKw"),
            "mode": snap.get("mode"),
            "linkOk": snap.get("linkOk"),
            "ts": snap.get("ts"),
            "source": snap.get("source"),
            "steeringControl": steering_control_info(state.source),
        }})

    @app.get("/api/kiosk_lock")
    def api_kiosk_lock():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        return jsonify({"ok": True, "lock": state.vault.state(state.lock_open),
                        "steeringAuthorized": steering_access_valid()})

    @app.post("/api/kiosk_lock/verify")
    def api_kiosk_lock_verify():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        body = request.get_json(silent=True) or {}
        pin = str(body.get("pin", ""))
        result = state.vault.verify(pin)
        if result.get("ok"):
            grant_steering_access()
        else:
            revoke_steering_access()
        result["lock"] = state.vault.state(state.lock_open)
        result["steeringAuthorized"] = steering_access_valid() if result.get("ok") else False
        return jsonify(result), 200 if result.get("ok") else 403

    @app.post("/api/kiosk_lock/session/end")
    def api_kiosk_steering_logout():
        revoke_steering_access()
        return jsonify({"ok": True, "steeringAuthorized": False})

    @app.post("/api/kiosk_lock/open")
    def api_kiosk_lock_open():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        body = request.get_json(silent=True) or {}
        pin = str(body.get("pin", ""))
        result = state.open_lock(pin)
        return jsonify(result), 200 if result.get("ok") else 403

    @app.post("/api/kiosk_lock/close")
    def api_kiosk_lock_close():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        return jsonify(state.close_lock())

    @app.post("/api/kiosk_steer")
    def api_kiosk_steer():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_command(state.source, request.get_json(silent=True))
        return jsonify(result), status

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


# ------------------------------------------------------ единственный экран -- #
# Экран должен быть один, сколько бы запусков ни случилось одновременно:
# автозапуск рабочего стола после входа, клик по ярлыку, ручной kiosk.sh.
# Поэтому показ экрана держит межпроцессный замок (flock) на весь срок жизни
# окна; второй запуск замок не получает и браузер не открывает вовсе.
_display_ports_lock = threading.Lock()
_display_ports: set[int] = set()  # порты, чей экран уже показывает этот процесс


def _display_lock_file(port: int) -> Path:
    return PROJECT_ROOT / "var" / f"kiosk-display-{port}.lock"


class _DisplayClaim:
    """Занятый показ экрана: держит flock, пока окно киоска открыто."""

    def __init__(self, port: int, handle) -> None:
        self.port = port
        self._handle = handle  # открытый файл замка (None — замка нет, не POSIX)

    def release(self) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            try:
                if fcntl is not None:
                    fcntl.flock(handle, fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                handle.close()
            except OSError:
                pass
        with _display_ports_lock:
            _display_ports.discard(self.port)


def _claim_display(port: int) -> _DisplayClaim | None:
    """Занять показ экрана на ``port``; None — экран уже показывает кто-то ещё.

    Замок один на порт, поэтому повторный вызов из этого же процесса тоже
    получит отказ — второй сторожевой поток нам не нужен.
    """
    with _display_ports_lock:
        if port in _display_ports:
            return None
        _display_ports.add(port)
    handle = None
    if fcntl is not None:
        try:
            path = _display_lock_file(port)
            path.parent.mkdir(parents=True, exist_ok=True)
            handle = path.open("a+")
        except OSError as exc:  # без файла замка живём на внутрипроцессной защите
            log.debug("не удалось открыть %s: %s", _display_lock_file(port), exc)
            handle = None
    if handle is not None:
        deadline = time.monotonic() + DISPLAY_CLAIM_TIMEOUT_SEC
        while True:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    handle.close()
                    with _display_ports_lock:
                        _display_ports.discard(port)
                    return None
                time.sleep(0.1)
    return _DisplayClaim(port, handle)


def display_claimed(port: int | None = None) -> bool:
    """Показывает ли кто-нибудь экран этого порта прямо сейчас (для тестов)."""
    port = port or kiosk_port()
    with _display_ports_lock:
        if port in _display_ports:
            return True
    if fcntl is None:
        return False
    try:
        path = _display_lock_file(port)
        if not path.exists():
            return False
        with path.open("a+") as handle:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                return True
            fcntl.flock(handle, fcntl.LOCK_UN)
    except OSError:
        return False
    return False


def _reopen_delay(quick_exits: int) -> float:
    """Пауза перед переоткрытием: растёт, если браузер выходит почти сразу."""
    if quick_exits <= 1:
        return BROWSER_REOPEN_SEC
    return min(BROWSER_REOPEN_SEC * 2 ** (quick_exits - 1), BROWSER_REOPEN_MAX_SEC)


def _display_worker(url: str, base_url: str, watch: bool,
                    stop_event: threading.Event, claim: _DisplayClaim) -> None:
    """Открывает экран в браузере и, если ``watch``, держит его открытым."""
    try:
        if not wait_for_server(base_url):
            log.warning("сервер не ответил за %.0f с — открываю %s как есть",
                        HEALTH_TIMEOUT_SEC, url)
        quick_exits = 0
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
                # Каталог создаём до запуска: иначе Firefox рисует
                # «Your Firefox profile cannot be loaded…» и экран пустой.
                profile = prepare_browser_profile(browser)
                proc = subprocess.Popen(
                    browser_command(browser, url, profile_dir=profile),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            except OSError as exc:
                log.error("не удалось открыть браузер %s: %s", browser, exc)
                return
            log.info(
                "экран робота открыт в браузере (%s), профиль %s: %s",
                os.path.basename(browser), profile, url,
            )
            if not watch:
                # Не переоткрываем, но замок показа держим, пока окно живо:
                # иначе следующий запуск решит, что экран никто не показывает.
                while not stop_event.wait(0.5):
                    if proc.poll() is not None:
                        return
                if proc.poll() is None:
                    proc.terminate()
                return
            # Ждём закрытия браузера и открываем заново: экран всегда на месте.
            # Но если процесс вышел почти сразу, это не «окно закрыли», а
            # лаунчер передал адрес другой копии/браузер упал: паузу между
            # попытками увеличиваем, чтобы не плодить окна очередями.
            started = time.monotonic()
            try:
                while not stop_event.wait(0.5):
                    if proc.poll() is not None:
                        break
                else:
                    return
            finally:
                if stop_event.is_set() and proc.poll() is None:
                    proc.terminate()  # показ остановлен — своё окно закрываем
            lived = time.monotonic() - started
            quick_exits = quick_exits + 1 if lived < BROWSER_QUICK_EXIT_SEC else 0
            delay = _reopen_delay(quick_exits)
            log.info("браузер закрыт — открываю экран заново через %.0f с", delay)
            if stop_event.wait(delay):
                return
    finally:
        claim.release()


def open_kiosk_screen(port: int | None = None, *, watch: bool = True,
                      stop_event: threading.Event | None = None) -> threading.Thread | None:
    """Открыть бортовой экран в отдельном потоке (сервер не блокируется).

    Возвращает поток показа или None, если экран этого порта уже открыт
    (этим или другим запуском киоска): второй браузер не запускается никогда.
    """
    port = port or kiosk_port()
    claim = _claim_display(port)
    if claim is None:
        log.info("экран робота (порт %s) уже открыт другим запуском — "
                 "второй браузер не открываю", port)
        return None
    base_url = f"http://127.0.0.1:{port}"
    thread = threading.Thread(
        target=_display_worker,
        args=(kiosk_url(port), base_url, watch, stop_event or threading.Event(), claim),
        name="kiosk-display",
        daemon=True,
    )
    try:
        thread.start()
    except Exception:
        claim.release()
        raise
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
        description="Бортовой экран (8090): публичная телеметрия, поворот модулей после PIN",
    )
    parser.add_argument("--host", default=None,
                        help="адрес прослушивания (по умолчанию RC_BIND_HOST или 0.0.0.0)")
    parser.add_argument("--port", type=int, default=None,
                        help=f"порт экрана (по умолчанию RC_KIOSK_PORT или {DEFAULT_PORT})")
    parser.add_argument("--source", choices=["auto", "sim", "serial", "ros", "master"], default=None,
                        help="источник данных: auto, sim, serial, ros или master (адресное управление углом)")
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
    # По умолчанию ищем ROS/UART; мастер выбирается явно как RC_SLAM_SOURCE=master.
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
        display = open_kiosk_screen(port, watch=watch, stop_event=stop_event)
        if display is None:
            # Экран уже на дисплее (автозапуск/прошлый запуск его держит).
            # Мы — лишний дублирующий запуск: не висим и окна не плодим.
            print("Экран уже открыт другим запуском — выхожу, чтобы не дублировать окно.")
            return 0
        try:
            while display.is_alive():
                display.join(timeout=0.5)
        except KeyboardInterrupt:
            pass
        stop_event.set()
        return 0

    if open_screen:
        # None (экран уже открыт) — не беда: сервер всё равно должен работать,
        # а открытое окно показывает именно этот адрес.
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
