"""Веб-интерфейс RUS SLAM внутри пульта оператора.

Интерфейс взят из https://github.com/danilka-revin/rus_slam (каталог ``gui/``)
и лежит в репозитории в ``slam_gui/`` — робот работает без интернета, поэтому
подтягивать его откуда-то при запуске нельзя.

Здесь мы:
  * импортируем ``slam_gui/backend.py`` как модуль (он написан только на
    стандартной библиотеке, pyserial и ROS подключаются по желанию);
  * выбираем источник данных: ROS 2, UART-модули, мастер по одному UART или симуляция;
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
import math
import re
import time
from pathlib import Path

from flask import Blueprint, Response, jsonify, request, send_from_directory, session

from robot_control.camera import CameraManager
from robot_control.lidar import (
    LidarManager,
    lidar_holds_port,
    lidar_reserved_ports,
    set_self_port,
    startup_summary,
    tag_module_port,
    try_lock_port,
    unlock_port,
    untag_module_port,
)

log = logging.getLogger("robot_control.slam")

GUI_DIR = Path(__file__).resolve().parent.parent / "slam_gui"
BACKEND_PY = GUI_DIR / "backend.py"

MAIN_PAGE = "main.html"
CONSOLE_PAGE = "index.html"
KIOSK_PAGE = "robot.html"
#: Путь бортового экрана: его же открывает браузер на дисплее робота.
KIOSK_URL_PATH = "/kiosk"

#: Ввод PIN открывает короткую сессию только для команд поворота рулевых модулей.
STEERING_SESSION_KEY = "kiosk_steering_expires"
STEERING_SESSION_TTL_S = 5 * 60
STEERING_ANGLE_LIMIT_DEG = 32.0
STEERING_COMMAND_HEADER = "X-Robot-Control"
STEERING_COMMAND_HEADER_VALUE = "steering"


def grant_steering_access(now=None):
    """Разрешить ручной поворот модулей на короткое время после проверки PIN."""
    session[STEERING_SESSION_KEY] = (time.time() if now is None else now) + STEERING_SESSION_TTL_S


def revoke_steering_access():
    session.pop(STEERING_SESSION_KEY, None)


def steering_access_valid(now=None):
    """Проверить и очистить истёкший доступ к рулевым моторам."""
    now = time.time() if now is None else now
    expires = session.get(STEERING_SESSION_KEY, 0)
    if not isinstance(expires, (int, float)) or expires <= now:
        revoke_steering_access()
        return False
    return True


def steering_control_info(source):
    """Публичная возможность управления: только мастер с адресной командой."""
    return {
        "available": bool(getattr(source, "supports_individual_steer", False)),
        "source": getattr(source, "name", "unknown"),
    }


def dispatch_steering_command(source, body):
    """Проверить узкий payload и отправить один целевой угол без команд тяги."""
    if not isinstance(body, dict) or set(body) != {"module", "angle"}:
        return {"ok": False, "reason": "payload"}, 400
    module = body.get("module")
    if not isinstance(module, str) or module.upper() not in {"FL", "FR", "RL", "RR"}:
        return {"ok": False, "reason": "invalid_module"}, 400
    module = module.upper()
    raw_angle = body.get("angle")
    if isinstance(raw_angle, bool):
        return {"ok": False, "reason": "invalid_angle"}, 400
    try:
        angle = float(raw_angle)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "invalid_angle"}, 400
    import math
    if not math.isfinite(angle):
        return {"ok": False, "reason": "invalid_angle"}, 400
    if abs(angle) > STEERING_ANGLE_LIMIT_DEG:
        return {"ok": False, "reason": "angle_limit", "limit": STEERING_ANGLE_LIMIT_DEG}, 400

    if not getattr(source, "supports_individual_steer", False):
        return {"ok": False, "reason": "control_not_supported",
                "source": getattr(source, "name", "unknown")}, 409
    command = getattr(source, "steer_module", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(module, angle)
    if result.get("ok"):
        return {"ok": True, "queued": True, "module": module,
                "angle": result.get("angle", round(angle, 1))}, 200

    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason in {"invalid_module", "invalid_angle", "angle_limit"}:
        status = 400
    return {"ok": False, "reason": reason}, status


#: Статика интерфейса: имя файла → endpoint. Отдаётся только этот список.
ASSETS = {
    "main.css": "main_css",
    "main.js": "main_js",
    "styles.css": "styles_css",
    "app.js": "app_js",
    "console.js": "console_js",
    "console-core.js": "console_core_js",
    "vision.js": "vision_js",
    # наш скрипт: пробрасывает токен сессии в подзапросы (нужен в iframe без cookie)
    "slam_auth.js": "slam_auth_js",
    # страница киоска на борту робота (без авторизации)
    "robot.css": "robot_css",
    "robot.js": "robot_js",
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


def chassis_ports(ports):
    """Порты модулей шасси без тех, что отданы лидару.

    USB-порт можно читать одному: если источник данных шасси в режиме ``auto``
    сам найдёт все ``/dev/ttyUSB*`` и откроет порт лидара, лидар останется без
    данных — именно так «скана нет» в веб-интерфейсе при живом железе (скрипт
    ``scripts/lidar_check.py`` при этом работает: его запускают, когда пульт
    остановлен). Порт из ``RC_LIDAR_PORT`` сравнивается по realpath, потому что
    на роботе это обычно ``/dev/serial/by-id/…`` → ``/dev/ttyUSB1``.
    """
    reserved = lidar_reserved_ports()
    if not reserved:
        return list(ports)
    kept, skipped = [], []
    for port in ports:
        try:
            real = os.path.realpath(port)
        except OSError:
            real = port
        if port in reserved or real in reserved:
            skipped.append(port)
        else:
            kept.append(port)
    if skipped:
        log.info(
            "шасси: порт(ы) %s отданы лидару (RC_LIDAR_PORT) — модули их не открывают",
            ", ".join(skipped),
        )
    return kept


def _autodetect_ports():
    """Реально существующие UART-адаптеры робота (кроме порта лидара)."""
    found = chassis_ports(_present_ports(DEFAULT_PORTS))
    if found:
        return found
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        found = chassis_ports(sorted(glob.glob(pattern)))
        if found:
            return found
    return []


def chassis_source(mod, ports):
    """Источник данных шасси на этих портах — под тем же замком, что у лидара.

    Замок порта (``try_lock_port``/``unlock_port``) общий на всё приложение:
    пока порт не доказал, что он — модуль шасси, его читает только один процесс
    робота, а лидар (он берёт порт монопольно, ``exclusive`` в pyserial) всегда
    получает его свободным. Без этого пульт и бортовой экран читали один USB-порт
    вдвоём — «device reports readiness to read but returned no data (… multiple
    access on port?)» — и лидар не мог найти себя ни на одном порту, хотя порты
    держало это же приложение, а на экране появлялся совет «проверь кабель».

    Замка мало: он говорит только «занят / свободен» и не различает, кто держит
    порт. Поэтому здесь же два механизма из ``robot_control.lidar``:

      * ``yield_to_lidar`` (:func:`lidar_holds_port`) — пока лидар ищет свой порт
        (окно ``RC_LIDAR_PRIORITY_S``, 20 с) или уже читает этот порт, модули не
        занимают его своей проверкой. Иначе на роботе с двумя адаптерами оба
        перебора сталкивались, и лидар сообщал «все 2 порт(ов) заняты другой
        программой» про порты, которые держало это же приложение;
      * ``tag_module``/``untag_module`` (:func:`tag_module_port`) — отметка в
        ``data/`` «на этом порту подтвердились модули». Её видит лидар и не
        перебирает такой порт: он всё равно занят, а совет «проверь кабель»
        уводит в сторону.
    """
    return mod.SerialSource(
        ports,
        claim=try_lock_port,
        release=unlock_port,
        yield_to_lidar=lidar_holds_port,
        tag_module=tag_module_port,
        untag_module=untag_module_port,
    )


def make_source(mode: str = "auto", ports=None):
    """Источник данных робота.

    ``master``: один UART мастер-Arduino (строки "@TLM", см. firmware/motor_master).
    ``auto``: ROS 2, если есть rclpy; иначе UART-модули, если есть pyserial и
    живой порт; иначе пустой источник (без выдуманных данных).
    """
    mod = load_backend()
    mode = (mode or "auto").strip().lower()

    if mode == "sim":
        return mod.SimSource()
    if mode == "serial":
        # Явно заданные RC_SLAM_PORTS — воля оператора: их лидар и так не
        # трогает (они попадают в excluded_ports()). Если список пуст, ищем
        # сами — но порт лидара не занимаем (см. chassis_ports).
        chosen = list(ports) if ports else (_autodetect_ports() or chassis_ports(DEFAULT_PORTS))
        if not chosen:
            log.info("шасси: живых UART-портов нет (порт лидара не в счёт) — значений не будет")
        return chassis_source(mod, chosen)
    if mode == "ros":
        return mod.RosSource()
    if mode == "master":
        # мастер-Arduino на одном UART (RC_SLAM_PORT, по умолчанию /dev/ttyACM0)
        return mod.MasterSource(os.environ.get("RC_SLAM_PORT", "/dev/ttyACM0").strip())

    if mode != "auto":
        log.warning("неизвестный RC_SLAM_SOURCE=%r — использую auto", mode)

    if _have("rclpy"):
        log.info("источник данных: ROS 2")
        return mod.RosSource()

    live = _present_ports(list(ports or [])) or _autodetect_ports()
    if _have("serial") and live:
        log.info("источник данных: UART-модули %s", ", ".join(live))
        return chassis_source(mod, live)

    log.info("источник данных: робот не подключён — значения пустые (симуляция только по RC_SLAM_SOURCE=sim)")
    return mod.NoSource()


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


def lidar_for_screen(manager, snap):
    """Точки лидара для экрана и признак, что это демо.

    Реальные точки приходят только из менеджера лидара: USB-порт (RPLIDAR,
    LD06/LD19, LDS-01/LDS-02), ROS 2 ``/scan``, HTTP ``POST /api/lidar`` или
    соседний процесс робота. Всё, что лежит в ``snap["lidar"]`` сверх этого, —
    данные источника-симуляции (``RC_SLAM_SOURCE=sim``): без пометки они
    выглядели бы на экране как «в реальном времени · 180 точек», то есть демо
    выдавалось бы за работающий лидар.
    """
    scan = manager.get_scan()
    if scan:
        return scan, False
    fallback = snap.get("lidar") if isinstance(snap, dict) else None
    if isinstance(fallback, list) and fallback:
        return fallback, True
    return None, False


def lidar_status_payload(manager, points, demo):
    """Единый статус лидара для ``/api/lidar``, ``/api/state`` и киоска.

    Одинаковый во всех ответах: соседний процесс робота берёт по нему скан, а
    экран — причину «скана нет». Флаг ``demo`` не даёт выдуманному скану
    разъехаться по обоим экранам как реальному.
    """
    status = manager.get_status()
    status["demo"] = bool(demo)
    if demo and not status["connected"]:
        status["reason"] = "демо-скан источника данных (реального лидара нет): " + status["reason"]
    status["scan"] = points
    status["count"] = len(points) if points else 0
    return status


def start_lidar(*, testing: bool = False, port: int | None = None, force: bool = False):
    """Запускает чтение лидара при старте веб-приложения.

    Без этого менеджер создавался только по первому запросу к API: до открытия
    страницы USB-порт мог занять источник данных шасси, а пульт и бортовой
    экран начинали читать лидар в разное время. ``.env`` к этому моменту уже
    прочитан (``Config.from_env``), поэтому настройки лидара берутся из него,
    а в журнал пишется, каким интерпретатором запущен процесс и что он видит.

    В тестах не запускается (``PYTEST_CURRENT_TEST``/``testing``): поток
    перебора портов там не нужен. ``RC_LIDAR_AUTOSTART=0`` отключает ранний
    старт и на роботе.
    """
    if (os.environ.get("RC_LIDAR_AUTOSTART") or "1").strip().lower() in {
        "0", "false", "no", "off", "нет",
    }:
        return None
    if not force and (testing or os.environ.get("PYTEST_CURRENT_TEST")):
        return None
    if port is not None:
        set_self_port(port)
    manager = LidarManager.get_instance()
    for line in startup_summary():
        log.info("Лидар: %s", line)
    return manager


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

    # --------- экран киоска на борту робота: без авторизации -------- #
    # Открывается локально в киоск-браузере на роботе; телеметрия публична,
    # а адресный поворот рулевых модулей открывается отдельной PIN-сессией.
    @bp.get(KIOSK_URL_PATH, endpoint="kiosk")
    def kiosk_screen():
        """Экран, который показывается на дисплее самого робота."""
        return _send(KIOSK_PAGE)

    bp.add_url_rule("/robot.html", endpoint="kiosk_page", view_func=kiosk_screen)

    @bp.get("/api/kiosk_state")
    def api_kiosk_state():
        """Публичная телеметрия и признак, поддерживает ли источник адресный поворот."""
        manager = LidarManager.get_instance()
        snap = state.snapshot()
        points, demo = lidar_for_screen(manager, snap)
        public = {
            "battery": snap.get("battery"),
            "motors": snap.get("motors"),
            "lidar": points,
            # Демо (стенд без железа) помечено: экран пишет «демо», а не
            # «в реальном времени».
            "lidarDemo": demo,
            # Причина «скана нет» нужна и на этом экране: robot.js берёт её
            # отсюда, а раньше в ответе её не было вовсе.
            "lidarStatus": lidar_status_payload(manager, points, demo),
            "speedMps": snap.get("speedMps"),
            "powerKw": snap.get("powerKw"),
            "mode": snap.get("mode"),
            "linkOk": snap.get("linkOk"),
            "ts": snap.get("ts"),
            "source": snap.get("source"),
            "steeringControl": steering_control_info(state.source),
        }
        return jsonify({"ok": True, "data": public})

    # PIN-оверлей киоска: телеметрия публична, команды рулевым моторам — только
    # после отдельной проверки PIN и с короткоживущей серверной сессией.
    @bp.get("/api/kiosk_lock")
    def api_kiosk_lock():
        return jsonify({"ok": True, "lock": state.vault.state(state.lock_open),
                        "steeringAuthorized": steering_access_valid()})

    @bp.post("/api/kiosk_lock/verify")
    def api_kiosk_lock_verify():
        body = request.get_json(silent=True) or {}
        result = state.vault.verify(str(body.get("pin", "")))
        if result.get("ok"):
            grant_steering_access()
        else:
            revoke_steering_access()
        result["lock"] = state.vault.state(state.lock_open)
        result["steeringAuthorized"] = steering_access_valid() if result.get("ok") else False
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/kiosk_lock/session/end")
    def api_kiosk_steering_logout():
        revoke_steering_access()
        return jsonify({"ok": True, "steeringAuthorized": False})

    @bp.post("/api/kiosk_lock/open")
    def api_kiosk_lock_open():
        body = request.get_json(silent=True) or {}
        result = state.open_lock(str(body.get("pin", "")))
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/kiosk_lock/close")
    def api_kiosk_lock_close():
        return jsonify(state.close_lock())

    @bp.post("/api/kiosk_steer")
    def api_kiosk_steer():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_command(state.source, request.get_json(silent=True))
        return jsonify(result), status

    # -------------------------------- API -------------------------------- #
    @bp.get("/api/state")
    @guard
    def api_state():
        snap = state.snapshot()
        manager = LidarManager.get_instance()
        points, demo = lidar_for_screen(manager, snap)
        snap["lidar"] = points
        snap["lidarDemo"] = demo
        # Причина, если скана нет: экран пишет её под картой (порт лидара не
        # найден, не та скорость, порт занят вторым процессом робота и т. д.).
        snap["lidarStatus"] = lidar_status_payload(manager, points, demo)
        snap["cameras"] = CameraManager.get_instance().list_cameras()
        return jsonify({"ok": True, "data": snap})

    # ----------------------------- ЛИДАР --------------------------------- #
    @bp.get("/api/lidar")
    @bp.get("/api/scan")
    def api_lidar_get():
        """Скан и статус лидара.

        Ответ открыт без авторизации намеренно: по нему второй процесс робота
        (пульт 8080 ↔ бортовой экран 8090) берёт скан, когда USB-порт занят
        соседом. Точки источника-симуляции отдаются только с флагом ``demo`` —
        сосед их не принимает, а экран показывает как демо.
        """
        mgr = LidarManager.get_instance()
        scan = mgr.get_scan()
        demo = False
        if scan is None and hasattr(state, "source"):
            try:
                src_data = getattr(state.source, "read", lambda: {})()
                scan = src_data.get("lidar")
                demo = bool(scan)
            except Exception:  # noqa: BLE001 — источник может не уметь read()
                scan = None
        return jsonify(lidar_status_payload(mgr, scan, demo))

    @bp.post("/api/lidar")
    @bp.post("/api/scan")
    def api_lidar_post():
        body = request.get_json(silent=True) or {}
        mgr = LidarManager.get_instance()
        if "points" in body or "scan" in body:
            pts = body.get("points") or body.get("scan") or []
            count = mgr.feed_points(pts, source_label="api")
            return jsonify({"ok": True, "count": count})
        elif "ranges" in body:
            count = mgr.feed_scan_msg(
                ranges=body.get("ranges", []),
                angle_min=float(body.get("angle_min", -math.pi)),
                angle_max=float(body.get("angle_max", math.pi)),
                angle_increment=float(body.get("angle_increment", 0.0174)),
                range_min=float(body.get("range_min", 0.05)),
                range_max=float(body.get("range_max", 12.0)),
                source_label="api_ranges",
            )
            return jsonify({"ok": True, "count": count})
        return jsonify({"ok": False, "error": "expected 'points' or 'ranges'"}), 400

    # ---------------------------- КАМЕРЫ --------------------------------- #
    @bp.get("/api/cameras")
    def api_cameras():
        mgr = CameraManager.get_instance()
        return jsonify({"ok": True, "cameras": mgr.list_cameras()})

    @bp.get("/api/camera/<int:cam_id>/stream")
    def api_camera_stream(cam_id: int):
        mgr = CameraManager.get_instance()
        return Response(
            mgr.generate_mjpeg_stream(cam_id),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @bp.get("/api/camera/front/stream")
    def api_camera_front_stream():
        return api_camera_stream(0)

    @bp.get("/api/camera/rear/stream")
    def api_camera_rear_stream():
        return api_camera_stream(1)

    @bp.get("/api/camera/<int:cam_id>/frame")
    def api_camera_frame(cam_id: int):
        mgr = CameraManager.get_instance()
        frame = mgr.get_frame(cam_id)
        if frame:
            return Response(frame, mimetype="image/jpeg")
        return Response(status=404)

    @bp.get("/api/camera/front/frame")
    def api_camera_front_frame():
        return api_camera_frame(0)

    @bp.get("/api/camera/rear/frame")
    def api_camera_rear_frame():
        return api_camera_frame(1)

    @bp.post("/api/camera/<int:cam_id>/frame")
    def api_camera_feed(cam_id: int):
        mgr = CameraManager.get_instance()
        data = request.get_data()
        if data:
            mgr.feed_frame(cam_id, data)
            return jsonify({"ok": True})
        return jsonify({"ok": False, "error": "empty data"}), 400

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
