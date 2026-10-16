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
import json
import logging
import os
import math
import re
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from flask import Blueprint, Response, jsonify, request, send_from_directory, session

from robot_control import eventlog
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

#: Ввод PIN открывает короткую сессию для руления и удерживаемой тяги.
STEERING_SESSION_KEY = "kiosk_steering_expires"
STEERING_SESSION_TTL_S = 5 * 60
STEERING_ANGLE_LIMIT_DEG = 32.0
STEERING_COMMAND_HEADER = "X-Robot-Control"
STEERING_COMMAND_HEADER_VALUE = "steering"
DRIVE_COMMAND_HEADER_VALUE = "drive"
MAX_DRIVE_PERCENT = 20


def grant_steering_access(now=None):
    """Разрешить ручной поворот модулей на короткое время после проверки PIN."""
    session[STEERING_SESSION_KEY] = (time.time() if now is None else now) + STEERING_SESSION_TTL_S


def revoke_steering_access():
    session.pop(STEERING_SESSION_KEY, None)


def steering_access_valid(now=None):
    """Проверить и очистить истёкший доступ к рулению и тяге."""
    now = time.time() if now is None else now
    expires = session.get(STEERING_SESSION_KEY, 0)
    if not isinstance(expires, (int, float)) or expires <= now:
        # Истёкшая сессия — настоящее событие: на экране видно, почему руление
        # снова требует PIN. Сессии, которой не было, в журнал не попадает:
        # иначе запись появлялась бы на каждом опросе состояния.
        if expires:
            eventlog.record(
                "warn",
                "PIN-сессия руления истекла (%.0f мин) — команды снова требуют PIN"
                % (STEERING_SESSION_TTL_S / 60.0),
                source="руление",
            )
        revoke_steering_access()
        return False
    return True


#: Русский текст причины отказа проверки PIN (сам код в журнал не пишется).
PIN_REASON_TEXT = {
    "wrong": "неверный PIN",
    "format": "PIN должен быть 4…8 цифр",
    "blocked": "ввод заблокирован после серии ошибок",
    "ok": "PIN принят",
}


def note_pin_result(result, *, where="киоск"):
    """Записать в журнал настоящий итог проверки PIN (подробности без цифр кода)."""
    reason = str((result or {}).get("reason", ""))
    text = PIN_REASON_TEXT.get(reason, reason or "проверка не пройдена")
    if (result or {}).get("ok"):
        eventlog.record(
            "ok",
            "PIN принят (%s): руление и тяга разрешены на %.0f мин"
            % (where, STEERING_SESSION_TTL_S / 60.0),
            source="pin",
        )
    else:
        eventlog.record("warn", "PIN не принят (%s): %s" % (where, text), source="pin")


#: Последняя записанная ступень тяги: heartbeat ручки не должен забивать журнал.
_last_drive_percent: float | None = None
_last_drive_lock = threading.Lock()


def note_drive_command(percent: float) -> None:
    """Записать ступень тяги (0 — тормоз), а не каждый heartbeat удержания.

    Кнопку держат, и команда уходит 5 раз в секунду; в журнале отражается
    смена ступени, то есть реальное действие оператора. Повторы внутри
    ``eventlog`` сворачиваются в счётчик ``×N``.
    """
    global _last_drive_percent
    step = round(float(percent))
    with _last_drive_lock:
        if _last_drive_percent == step:
            return
        _last_drive_percent = step
    if step == 0:
        eventlog.record("info", "тяга 0% — тормоз BLDC", source="тяга")
    else:
        eventlog.record(
            "ok",
            "тяга %+d%% (%s)" % (step, "вперёд" if step > 0 else "назад"),
            source="тяга",
        )


#: Русский текст причины отказа команды — для журнала экрана.
COMMAND_REASON_TEXT = {
    "payload": "неверный состав полей",
    "invalid_module": "неизвестный модуль",
    "invalid_angle": "неверный угол",
    "angle_limit": "угол вне допуска ±%.0f°" % STEERING_ANGLE_LIMIT_DEG,
    "invalid_drive": "неверная величина тяги",
    "drive_limit": "тяга вне допуска ±%d%%" % MAX_DRIVE_PERCENT,
    "control_not_supported": "источник данных не умеет адресное управление",
    "module_offline": "модуль не на связи",
    "serial_unavailable": "нет связи по UART",
    "command_queue_full": "очередь команд занята",
    "command_failed": "команда не выполнена",
    "pin_required": "нужен PIN руления",
    "request_header_required": "запрос без заголовка X-Robot-Control",
    "module_busy": "модуль занят (поворачивается, калибруется или включена тяга)",
    "not_homed": "у модуля нет нуля — откалибруйте по датчику или задайте 0°",
}


def note_pin_required(action: str) -> None:
    """Команда без PIN-сессии: на экране видно, чего именно не хватает."""
    note_command_rejected(action, "pin_required")


def note_command_rejected(action: str, reason: str) -> None:
    """Отказ команды — тоже настоящее событие: на экране видно, почему её нет."""
    text = COMMAND_REASON_TEXT.get(reason, reason or "причина неизвестна")
    eventlog.record("warn", "%s отклонено: %s" % (action, text), source="управление")


def note_steering_command(module: str, angle: float) -> None:
    """Записать настоящую команду поворота одного модуля."""
    eventlog.record("ok", "руление %s → %+.1f°" % (module, float(angle)), source="руление")


def note_steering_zero(module: str) -> None:
    """Записать ручной программный ноль: без движения, только отсчёт от колеса."""
    eventlog.record(
        "info", "программный ноль %s выставлен вручную (без движения)" % module,
        source="руление",
    )


def note_steering_calibration(module: str) -> None:
    """Записать запуск калибровки по оптодатчику D18 (колесо будет двигаться)."""
    target = "всех колёс" if module == "ALL" else module
    eventlog.record(
        "ok", "калибровка руля %s по оптодатчику запущена" % target, source="руление",
    )


#: Слова в сообщении прошивки, после которых запись в журнале — предупреждение.
_MASTER_WARN_WORDS = ("ОШИБКА", "нет связи", "не доставлена", "нет команды", "отклонена",
                      "заблокирован", "Неизв.")


def note_master_message(text: str) -> None:
    """Сообщение прошивки мастера («# FR: Калибровка ЗАВЕРШЕНА!») — в журнал экрана."""
    text = str(text or "").strip()
    if not text:
        return
    level = "warn" if any(word in text for word in _MASTER_WARN_WORDS) else "info"
    eventlog.record(level, "прошивка " + text, source="мастер")


def reset_journal_notes() -> None:
    """Забыть, о чём журнал уже писал: повторная инициализация и тесты.

    Без этого после очистки журнала тот же статус лидара считался бы «уже
    записанным» и на экране осталась бы пустота.
    """
    global _lidar_note, _last_drive_percent
    with _lidar_note_lock:
        _lidar_note = None
    with _last_drive_lock:
        _last_drive_percent = None


#: Повторяющийся «лидар:» внутри причины статуса — префикс нужен один.
_LIDAR_PREFIX_RE = re.compile(r"(?i)(?:^|(?<=[\s(]))лидар:\s*")

#: Последнее состояние лидара, о котором уже написано в журнал.
_lidar_note: tuple | None = None
_lidar_note_lock = threading.Lock()


def is_stop_request(body) -> bool:
    """Команда СТОП: тяга ровно 0 %. Она только гасит PWM и включает тормоз.

    Такая команда не даёт движения, поэтому её не блокирует PIN-сессия:
    иначе после истечения PIN оператор не мог бы остановить шасси с пульта.
    Заголовок ``X-Robot-Control`` и вход оператора по-прежнему обязательны.
    Любая ненулевая тяга, лимит 20 % и PIN для неё не меняются.
    """
    if not isinstance(body, dict) or set(body) != {"percent"}:
        return False
    value = body.get("percent")
    return not isinstance(value, bool) and isinstance(value, (int, float)) and value == 0


def stop_url_for(peer_url: str) -> str:
    """Адрес команды тяги бортового экрана по его адресу телеметрии."""
    base = str(peer_url or "").split("/api/", 1)[0].rstrip("/")
    return base + "/api/kiosk_drive"


def relay_stop(peer_url: str, timeout: float = 1.5):
    """Переслать СТОП на бортовой экран (владелец порта мастера). Возвращает (тело, статус)."""
    url = stop_url_for(peer_url)
    request_obj = urllib.request.Request(
        url, data=b'{"percent": 0}', method="POST",
        headers={"Content-Type": "application/json", "X-Robot-Control": DRIVE_COMMAND_HEADER_VALUE},
    )
    try:
        with urllib.request.urlopen(request_obj, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8", "replace") or "{}")
            return body, 200 if body.get("ok") else 409
    except urllib.error.HTTPError as exc:
        try:
            body = json.loads(exc.read().decode("utf-8", "replace") or "{}")
        except (ValueError, OSError):
            body = {}
        body.setdefault("ok", False)
        body.setdefault("reason", "kiosk_refused")
        return body, exc.code or 502
    except (urllib.error.URLError, OSError, ValueError):
        return {"ok": False, "reason": "kiosk_unreachable"}, 503


#: Русский текст причины отказа при остановке тяги (для журнала).
STOP_REASON_TEXT = {
    "kiosk_unreachable": "бортовой экран не отвечает",
    "kiosk_refused": "бортовой экран отказал",
    "serial_unavailable": "UART мастера недоступен",
    "module_offline": "модуль не на связи",
    "command_queue_full": "очередь команд переполнена",
    "control_not_supported": "этот пульт не управляет шасси",
    "command_failed": "команда не выполнена",
}


def note_stop_result(result, *, where: str) -> None:
    """Записать в журнал, подтверждена ли остановка тяги сервером (а не только экраном)."""
    if (result or {}).get("ok"):
        eventlog.record("info", "СТОП тяги подтверждён (%s): PWM 0, тормоз" % where, source="тяга")
        return
    reason = str((result or {}).get("reason", "") or "")
    text = STOP_REASON_TEXT.get(reason, reason or "неизвестная причина")
    eventlog.record("err", "СТОП тяги НЕ подтверждён (%s): %s — проверьте шасси" % (where, text),
                    source="тяга")


def note_lock_hardware(result, action: str) -> None:
    """Честно записать, что сделал замок кейса: физически или только программно."""
    mode = str((result or {}).get("hardware", "") or "")
    verb = "открыт" if action == "open" else "закрыт"
    env = "RUS_SLAM_LOCK_CMD" if action == "open" else "RUS_SLAM_LOCK_CMD_CLOSE"
    if mode == "hardware":
        eventlog.record("info", "кейс %s: внешняя команда замка выполнена (%s)" % (verb, env), source="замок")
    elif mode == "software":
        eventlog.record("warn", "кейс %s ТОЛЬКО программно: %s не задана, физического замка нет"
                        % (verb, env), source="замок")
    elif mode == "error":
        eventlog.record("err", "кейс: команда замка (%s) не выполнена — физическое состояние неизвестно"
                        % env, source="замок")


def note_lidar_status(status) -> None:
    """Записать в журнал смену состояния лидара — и только смену.

    Статус пересобирается на каждом запросе экрана, поэтому в журнал попадает
    переход («на связи» → «скан не идёт»), а не одна и та же строка подряд.
    """
    global _lidar_note
    if not isinstance(status, dict):
        return
    key = (
        bool(status.get("connected")),
        bool(status.get("demo")),
        str(status.get("reason", "")),
        str(status.get("source", "")),
    )
    with _lidar_note_lock:
        if key == _lidar_note:
            return
        _lidar_note = key
    connected, demo, reason, source = key
    # Причина приходит из статуса со своим «лидар:» (иногда дважды) — на экране
    # это читалось бы как «лидар: … лидар: …», поэтому префикс оставляем один.
    detail = _LIDAR_PREFIX_RE.sub("", reason).strip()
    if connected:
        points = status.get("points_count")
        where = "%s, %s точек" % (source or "источник", points) if points else (source or "")
        eventlog.record("ok", "лидар на связи (" + where + ")" if where else "лидар на связи",
                        source="лидар")
    elif demo:
        eventlog.record("warn", "лидар: " + (detail or "реального лидара нет, показан демо-скан"),
                        source="лидар")
    else:
        eventlog.record("warn", "лидар: скана нет — " + (detail or "причина неизвестна"),
                        source="лидар")


def steering_control_info(source):
    """Публичная возможность управления: мастер, адресные команды, ноль руля.

    Ноль — калибровкой по оптодатчику D18 (прошивка v2, команда ``k``), ручной
    «Задать 0°» остался запасным вариантом (например, датчик снят).
    """
    calibration = bool(getattr(source, "supports_calibration", False))
    return {
        "available": bool(getattr(source, "supports_individual_steer", False)),
        "source": getattr(source, "name", "unknown"),
        "zeroMode": "opto" if calibration else "manual",
        "optoUsed": calibration,
        "calibration": calibration,
        "manualZero": True,
        "limitDeg": STEERING_ANGLE_LIMIT_DEG,
    }


def dispatch_steering_command(source, body):
    """Проверить payload и отправить адресную команду одному рулевому модулю.

    Ответ не меняется, но отказ теперь виден в журнале экрана: иначе на борту
    «ничего не произошло» без объяснения — оператор не знает, ждать ему или
    искать причину.
    """
    result, status = _steering_command(source, body)
    if not result.get("ok"):
        note_command_rejected("руление", str(result.get("reason", "")))
    return result, status


def _steering_command(source, body):
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
        note_steering_command(module, result.get("angle", round(angle, 1)))
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


def dispatch_steering_zero(source, body):
    """Проверить payload и поставить в очередь программный ноль одного модуля."""
    result, status = _steering_zero(source, body)
    if not result.get("ok"):
        note_command_rejected("программный ноль", str(result.get("reason", "")))
    return result, status


def _steering_zero(source, body):
    if not isinstance(body, dict) or set(body) != {"module"}:
        return {"ok": False, "reason": "payload"}, 400
    module = body.get("module")
    if not isinstance(module, str) or module.upper() not in {"FL", "FR", "RL", "RR"}:
        return {"ok": False, "reason": "invalid_module"}, 400
    module = module.upper()
    if not getattr(source, "supports_individual_steer", False):
        return {"ok": False, "reason": "control_not_supported"}, 409
    command = getattr(source, "zero_module", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(module)
    if result.get("ok"):
        note_steering_zero(module)
        return {"ok": True, "queued": True, "module": module,
                "zeroMode": "manual"}, 200
    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason == "invalid_module":
        status = 400
    return {"ok": False, "reason": reason}, status


def dispatch_steering_calibration(source, body):
    """Проверить payload и запустить калибровку по оптодатчику (модуль или ALL)."""
    result, status = _steering_calibration(source, body)
    if not result.get("ok"):
        note_command_rejected("калибровка руля", str(result.get("reason", "")))
    return result, status


def _steering_calibration(source, body):
    if not isinstance(body, dict) or set(body) != {"module"}:
        return {"ok": False, "reason": "payload"}, 400
    module = body.get("module")
    if not isinstance(module, str) or module.upper() not in {"FL", "FR", "RL", "RR", "ALL"}:
        return {"ok": False, "reason": "invalid_module"}, 400
    module = module.upper()
    command = getattr(source, "calibrate_module", None)
    if not getattr(source, "supports_calibration", False) or not callable(command):
        return {"ok": False, "reason": "control_not_supported",
                "source": getattr(source, "name", "unknown")}, 409
    result = command(module)
    if result.get("ok"):
        note_steering_calibration(module)
        return {"ok": True, "queued": True, "module": module,
                "offline": list(result.get("offline") or [])}, 200
    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason == "invalid_module":
        status = 400
    return {"ok": False, "reason": reason}, status


def dispatch_drive_command(source, body):
    """Проверить ограниченную команду тяги (отрицательная величина — назад)."""
    result, status = _drive_command(source, body)
    if not result.get("ok"):
        note_command_rejected("тяга", str(result.get("reason", "")))
    return result, status


def _drive_command(source, body):
    if not isinstance(body, dict) or set(body) != {"percent"}:
        return {"ok": False, "reason": "payload"}, 400
    raw_percent = body.get("percent")
    if isinstance(raw_percent, bool):
        return {"ok": False, "reason": "invalid_drive"}, 400
    try:
        percent = float(raw_percent)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "invalid_drive"}, 400
    if not math.isfinite(percent):
        return {"ok": False, "reason": "invalid_drive"}, 400
    if abs(percent) > MAX_DRIVE_PERCENT:
        return {"ok": False, "reason": "drive_limit", "limit": MAX_DRIVE_PERCENT}, 400
    if not getattr(source, "supports_drive", False):
        return {"ok": False, "reason": "control_not_supported"}, 409
    command = getattr(source, "drive", None)
    if not callable(command):
        return {"ok": False, "reason": "control_not_supported"}, 409
    result = command(percent)
    if result.get("ok"):
        note_drive_command(result.get("percent", round(percent)))
        return {"ok": True, "queued": True, "percent": result.get("percent", round(percent)),
                "maxPercent": MAX_DRIVE_PERCENT}, 200
    reason = result.get("reason", "command_failed")
    status = 409
    if reason in {"serial_unavailable", "module_offline"}:
        status = 503
    elif reason == "command_queue_full":
        status = 429
    elif reason in {"invalid_drive", "drive_limit", "payload"}:
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
    "battery.js": "battery_js",
    "steering.css": "steering_css",
    "steering.js": "steering_js",
    "update.css": "update_css",
    "update.js": "update_js",
    # панель настроек лидара (кнопка «Лидар» на пульте и на бортовом экране)
    "lidar_settings.css": "lidar_settings_css",
    "lidar_settings.js": "lidar_settings_js",
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
    """Есть ли модуль в этом интерпретаторе.

    Для ``rclpy`` отдельная ветка: ROS 2 ставится системно в ``/opt/ros``, а
    приложение работает из ``.venv`` и без подготовки путей системных пакетов не
    видит (см. :mod:`robot_control.ros_env`). Раньше ``auto`` из-за этого
    выбирал UART-модули, хотя ROS 2 на роботе стоял.
    """
    if module_name == "rclpy":
        from . import ros_env
        return ros_env.rclpy_available()
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
    from .battery import reserved_ports
    reserved = lidar_reserved_ports() | reserved_ports()
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
            "шасси: порт(ы) %s отданы лидару/BMS (RC_LIDAR_PORT/RC_BMS_PORT) — модули их не открывают",
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
    from .battery import reserved_ports
    bms_ports = reserved_ports()
    ports = [p for p in ports if p not in bms_ports and os.path.realpath(p) not in bms_ports]
    return mod.SerialSource(
        ports,
        claim=try_lock_port,
        release=unlock_port,
        yield_to_lidar=lidar_holds_port,
        tag_module=tag_module_port,
        untag_module=untag_module_port,
    )


def peer_kiosk_url() -> str:
    """Адрес бортового экрана, откуда пульт берёт телеметрию мастера."""
    explicit = (os.environ.get("RC_SLAM_PEER_URL") or "").strip()
    if explicit:
        return explicit
    port = (os.environ.get("RC_KIOSK_PORT") or "8090").strip() or "8090"
    return f"http://127.0.0.1:{port}/api/kiosk_state"


def make_source(mode: str = "auto", ports=None, master_role: str = "owner"):
    """Источник данных робота.

    ``master``: один UART мастер-Arduino (строки "@TLM", см. firmware/motor_master).
    Порт открывает только владелец (``master_role="owner"`` — бортовой экран);
    пульт (``"peer"``) читает телеметрию с экрана и порт не трогает.
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
    if mode == "master" and master_role == "peer":
        return mod.MasterPeerSource(peer_kiosk_url())
    if mode == "master":
        # мастер-Arduino на одном UART (RC_SLAM_PORT, по умолчанию /dev/ttyACM0)
        from .battery import reserved_ports
        port = os.environ.get("RC_SLAM_PORT", "/dev/ttyACM0").strip()
        if port in reserved_ports() or os.path.realpath(port) in reserved_ports():
            log.error("Шасси: RC_SLAM_PORT совпадает с RC_BMS_PORT — мастер отключён")
            return mod.NoSource()
        # Сообщения прошивки («# FR: Калибровка ЗАВЕРШЕНА!») — в журнал экрана.
        return mod.MasterSource(port, on_message=note_master_message)

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
                max_attempts=None, lock_sec=None, master_role="owner"):
    """Собирает ``App`` из backend.py: источник данных + PIN-хранилище."""
    mod = load_backend()
    source = make_source(mode, ports, master_role=master_role)
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
    # Экранный журнал: смена состояния лидара — настоящее событие робота.
    note_lidar_status(status)
    return status


def kiosk_log_payload(limit=None, *, state=None) -> dict:
    """Настоящий журнал событий для экрана: записи, счётчики, время работы."""
    source = None
    if state is not None:
        source = getattr(getattr(state, "source", None), "name", None)
    uptime = None
    started = getattr(state, "started", None)
    if isinstance(started, (int, float)):
        uptime = max(0, int(time.time() - started))
    return eventlog.payload(limit, uptime_sec=uptime, source=source)


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
    # а руление/тяга открываются отдельной короткой PIN-сессией.
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
            "drive": snap.get("drive"),
            "masterMessages": snap.get("masterMessages") or [],
        }
        return jsonify({"ok": True, "data": public})

    @bp.get("/api/battery")
    def api_battery():
        from .battery import kiosk_snapshot
        snap = kiosk_snapshot()
        # Порт BMS держит бортовой экран: настраивать подключение можно только там.
        snap["setupAvailable"] = False
        return jsonify({"ok": True, "battery": snap}), 200, {"Cache-Control": "no-store"}

    @bp.get("/api/battery/setup")
    def api_battery_setup():
        return jsonify({"ok": True, "available": False, "ports": [],
                        "reason": "Подключение BMS настраивается на бортовом экране (порт 8090): "
                                  "страница «АКБ» → «Подключение BMS»."}), 200, {"Cache-Control": "no-store"}

    @bp.get("/api/battery/report.csv")
    def api_battery_report():
        from .battery import kiosk_snapshot, report_response
        return report_response(kiosk_snapshot())

    # PIN-оверлей киоска: телеметрия публична, команды руления и тяги доступны
    # только после проверки PIN и в короткоживущей серверной сессии.
    @bp.get("/api/kiosk_lock")
    def api_kiosk_lock():
        return jsonify({"ok": True, "lock": state.vault.state(state.lock_open),
                        "steeringAuthorized": steering_access_valid()})

    @bp.post("/api/kiosk_lock/verify")
    def api_kiosk_lock_verify():
        body = request.get_json(silent=True) or {}
        result = state.vault.verify(str(body.get("pin", "")))
        note_pin_result(result, where="бортовой экран")
        if result.get("ok"):
            grant_steering_access()
        else:
            revoke_steering_access()
        result["lock"] = state.vault.state(state.lock_open)
        result["steeringAuthorized"] = steering_access_valid() if result.get("ok") else False
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/kiosk_lock/session/end")
    def api_kiosk_steering_logout():
        if steering_access_valid():
            eventlog.record("info", "PIN-сессия руления закрыта оператором", source="руление")
        revoke_steering_access()
        return jsonify({"ok": True, "steeringAuthorized": False})

    @bp.post("/api/kiosk_lock/open")
    def api_kiosk_lock_open():
        body = request.get_json(silent=True) or {}
        result = state.open_lock(str(body.get("pin", "")))
        if result.get("ok"):
            note_lock_hardware(result, "open")
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/kiosk_lock/close")
    def api_kiosk_lock_close():
        result = state.close_lock()
        note_lock_hardware(result, "close")
        return jsonify(result)

    @bp.post("/api/kiosk_steer")
    def api_kiosk_steer():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("руление")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_command(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/kiosk_zero")
    def api_kiosk_zero():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("программный ноль")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_zero(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/kiosk_calibrate")
    def api_kiosk_calibrate():
        if request.headers.get(STEERING_COMMAND_HEADER) != STEERING_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        if not steering_access_valid():
            note_pin_required("калибровка руля")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_steering_calibration(state.source, request.get_json(silent=True))
        return jsonify(result), status

    @bp.post("/api/kiosk_drive")
    def api_kiosk_drive():
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        body = request.get_json(silent=True)
        if not steering_access_valid() and not is_stop_request(body):
            note_pin_required("тяга")
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_drive_command(state.source, body)
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

    @bp.post("/api/drive")
    @guard
    def api_drive():
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        body = request.get_json(silent=True)
        if not steering_access_valid() and not is_stop_request(body):
            return jsonify({"ok": False, "reason": "pin_required"}), 401
        result, status = dispatch_drive_command(state.source, body)
        return jsonify(result), status

    @bp.post("/api/stop")
    @guard
    def api_stop():
        """СТОП тяги с пульта: команда PWM 0 и тормоз на шасси.

        Пульт 8080 не владеет портом мастера, поэтому команда пересылается на
        бортовой экран (8090). Ответ — только реальный итог: подтверждён ли стоп.
        Это программная остановка; аварийный выключатель в силовой цепи остаётся обязательным.
        """
        if request.headers.get(STEERING_COMMAND_HEADER) != DRIVE_COMMAND_HEADER_VALUE:
            return jsonify({"ok": False, "reason": "request_header_required"}), 400
        source = state.source
        if getattr(source, "supports_drive", False):
            result, status = dispatch_drive_command(source, {"percent": 0})
            where = "пульт"
        else:
            peer = getattr(source, "peer_url", None) or peer_kiosk_url()
            result, status = relay_stop(peer)
            where = "бортовой экран"
        note_stop_result(result, where=where)
        return jsonify(result), status

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

    @bp.get("/api/kiosk_logs")
    @guard
    def api_kiosk_logs():
        """Настоящий журнал событий робота — то, что показывает панель «Логи».

        Тот же журнал, что и на бортовом экране (8090): записи логирования
        ``robot_control.*`` и явные события маршрутов. На пульте он закрыт
        входом оператора — в отличие от киоска, где панель публична, как и
        вся телеметрия борта.
        """
        limit = eventlog.clamp_limit(request.args.get("limit"))
        return jsonify({"ok": True, "data": kiosk_log_payload(limit, state=state)})

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
        if result.get("ok"):
            note_lock_hardware(result, "open")
        return jsonify(result), 200 if result.get("ok") else 403

    @bp.post("/api/lock/close")
    @guard
    def api_lock_close():
        result = state.close_lock()
        note_lock_hardware(result, "close")
        return jsonify(result)

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
