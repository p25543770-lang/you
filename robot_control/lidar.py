"""Модуль работы с 2D лидаром 360° (RPLIDAR, LDS-01, LD06/LD19, ROS 2 /scan, HTTP).

Поддерживает:
1. Прямое чтение с USB/serial лидара: RPLIDAR (A1/A2M8 — 115200,
   A2M7/A2M12/A3/S1 — 256000, C1 — 460800), LD06/LD19 (230400) и
   LDS-01/LDS-02 — лидар TurtleBot3 Burger (HLS-LFCD2, кадры 0xFA, 230400).
   Модель и скорость обычно неизвестны заранее, поэтому параметры подбираются
   перебором, а признак успеха — реальные точки скана, а не ответ устройства.
2. Подписку на топик ROS 2 /scan (sensor_msgs/LaserScan), если установлен rclpy.
3. Приём скана через HTTP API (POST /api/lidar или POST /api/scan).
4. Демо-генератор скана (стены склада с препятствиями) для отладки без железа.

Порт можно закрепить вручную: ``RC_LIDAR_PORT=/dev/ttyUSB1`` (можно список или
glob). Остальные настройки: ``RC_LIDAR_BAUD``, ``RC_LIDAR_PROTOCOL``
(auto | rplidar | ld06 | lds01), ``RC_LIDAR_MOTOR`` (auto | none | dtr-low |
dtr-high | rts-low | rts-high), ``RC_LIDAR_AUTOPROBE``, ``RC_LIDAR_EXCLUDE``,
``RC_LIDAR_PRIORITY_S``.

Почему столько осторожности: на роботе одновременно работают два процесса с этим
менеджером (пульт оператора на 8080 и бортовой экран на 8090), а USB-порт лидара
можно открыть только один раз. Поэтому порт защищён межпроцессным замком, а
второй процесс берёт готовый скан у первого по HTTP — иначе половина экранов
навсегда показывала бы «нет скана» без единой строчки в журнале.

Замка мало: он говорит только «занят / свободен». Поэтому в ``data/`` рядом с
замком лежат ещё два файла состояния (по одному на процесс) —
``lidar-state-<pid>.json`` («ищу порт» / «читаю такой-то порт») и
``chassis-ports-<pid>.json`` («на этих портах подтверждены модули шасси»). По ним
источник данных шасси уступает порты лидару на время поиска
(``RC_LIDAR_PRIORITY_S``, 20 с), а лидар не перебирает порты модулей: без этого
на роботе с двумя адаптерами оба перебора сталкивались на каждом круге и в
журнале появлялось «лидар не найден: все 2 порт(ов) заняты другой программой»,
хотя порты держало это же приложение.

Диагностика: ``python3 scripts/lidar_check.py`` печатает, что видно на портах;
причина отсутствия скана лежит в ``GET /api/lidar`` (поле ``reason``) и на
экране робота в подписи «скана лидара нет».
"""

from __future__ import annotations

import glob
import json
import logging
import math
import os
import re
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence, Tuple

from . import ros_env
from .ros_lidar import RosLidar

try:  # POSIX (робот на Ubuntu): межпроцессный замок на порт лидара.
    import fcntl
except ImportError:  # pragma: no cover - не-POSIX платформы
    fcntl = None  # type: ignore[assignment]

log = logging.getLogger("robot_control.lidar")

LIDAR_MAX_AGE_S = 1.0  # скан старше 1 с считается устаревшим
LIDAR_DEFAULT_RANGE_M = 10.0

#: Таймаут одного чтения порта, с: от него зависят отклик на stop() и переподключение.
SERIAL_READ_TIMEOUT_S = 0.2
#: Сколько ждём готовности порта к записи (команды SCAN и запуска LDS-01).
#: Без него pyserial ждёт дескриптор в ``select()`` **без ограничения времени**:
#: на вставшем передатчике (залипший USB-адаптер, забитый буфер) ``write(SCAN)``
#: останавливал поток поиска навсегда — лидар молча переставал искать порт.
SERIAL_WRITE_TIMEOUT_S = 0.5
#: Сколько ждём реальные точки, прежде чем считать набор параметров неудачным.
#: Для LDS-01 (TurtleBot3 Burger) увеличено: мотору нужно время на раскрутку
#: после команды 'b', особенно после холодного старта.
LIDAR_PROBE_TIMEOUT_S = 2.2
#: Сколько ждём точки, прежде чем считать, что поток данных оборвался.
LIDAR_DATA_TIMEOUT_S = 2.2
#: Предел одной попытки на порту, пока витка так и не появилось. Нужен потому,
#: что на порту может оказаться чужое устройство с непрерывным потоком: без
#: предела перебор висел на нём вечно и до лидара на соседнем USB не доходил.
#: Для LDS-01 увеличено до 6 с — одна попытка должна успеть пройти полный круг
#: перебора моторов у RPLIDAR и при этом дождаться раскрутки LDS-01.
LIDAR_ATTEMPT_MAX_S = 6.0
#: Виток с меньшим числом точек не публикуем: это мусор или чужой протокол.
LIDAR_MIN_POINTS = 20
#: Если полный виток не закрывается, публикуем накопленное по паузе.
LIDAR_PUBLISH_GAP_S = 0.35
#: Пауза перед новым кругом поиска порта. Уменьшено до 1 с для быстрого
#: переподключения LDS-01 после обрыва или смены USB-порта.
LIDAR_RESCAN_S = 1.0

#: Скорости RPLIDAR по моделям (у A3 на USB-адаптере есть переключатель 115200/256000).
RPLIDAR_BAUDS = (115200, 256000, 460800)
#: LD06/LD19 — односторонний поток на 230400, команд не принимает.
LD06_BAUD = 230400
#: LDS-01/LDS-02 (TurtleBot3 Burger, он же HLS-LFCD2) — тоже 230400, кадры 0xFA.
LDS01_BAUD = 230400

#: Профили поиска в порядке частоты моделей: (протокол, скорость).
#: LDS-01 (TurtleBot3 Burger, lds_01) стоит первым — робот использует именно
#: его, и он должен находиться за первый круг перебора, без ожидания остальных
#: скоростей. На 230400 живут два протокола (ld06 и lds01) с разными кадрами
#: (0x54 vs 0xFA), поэтому оба проверяются подряд, но lds01 — первым.
LIDAR_PROFILES: Tuple[Tuple[str, int], ...] = (
    ("lds01", LDS01_BAUD),    # LDS-01 / LDS-02 (TurtleBot3 Burger, HLS-LFCD2) — основной лидар робота
    ("ld06", LD06_BAUD),      # LD06 / LD19 (комплекты D500 и клоны)
    ("rplidar", 115200),      # RPLIDAR A1 / A2M8
    ("rplidar", 256000),      # RPLIDAR A2M7 / A2M12 / A3 / S1
    ("rplidar", 460800),      # RPLIDAR C1
)

#: Как в RC_LIDAR_PROTOCOL можно назвать протоколы (синонимы — в _PROTOCOL_ALIASES).
LIDAR_PROTOCOLS: Tuple[str, ...] = ("auto", "rplidar", "ld06", "lds01")

_PROTOCOL_ALIASES = {
    "lds": "lds01", "lds-01": "lds01", "lds_01": "lds01", "lds1": "lds01",
    "lds02": "lds01", "lds-02": "lds01", "lds2": "lds01",
    "hls": "lds01", "hls-lfcd2": "lds01", "hls_lfcd2": "lds01", "lfcd2": "lds01",
    "turtlebot": "lds01", "turtlebot3": "lds01",
}


#: Откуда берётся скан: ``ros`` — топик ROS 2 (так лидар отдаёт драйвер
#: производителя), ``serial`` — USB-порт читает само приложение, ``auto`` —
#: ROS 2, если он есть, иначе USB. По умолчанию ``auto``: на роботе с ROS 2
#: лидар читается из ``/scan``, а на стенде без ROS остаётся прежний путь.
LIDAR_SOURCES: Tuple[str, ...] = ("auto", "ros", "serial")
#: Синонимы RC_LIDAR_SOURCE: «ros2», «scan», «topic» → ``ros``; «uart» → ``serial``.
_SOURCE_ALIASES = {
    "ros2": "ros", "ros-2": "ros", "scan": "ros", "topic": "ros", "/scan": "ros",
    "usb": "serial", "uart": "serial", "port": "serial", "порт": "serial",
    "авто": "auto",
}


def normalize_source(value: Any) -> str:
    """Приводит RC_LIDAR_SOURCE/поле панели к ``auto`` | ``ros`` | ``serial``."""
    text = str(value or "").strip().lower().replace(" ", "")
    text = _SOURCE_ALIASES.get(text, text)
    return text if text in LIDAR_SOURCES else "auto"


#: Варианты включения мотора RPLIDAR через линии USB-переходника (MOTOCTL).
#: У адаптеров A1/A2 это DTR (низкий уровень — мотор включён), у части плат — RTS
#: и с обратной полярностью; последний вариант — линии не трогаем (у многих
#: клонов мотор питается постоянно). Перебираем, пока не пойдут точки.
RPLIDAR_MOTOR_VARIANTS: Tuple[Tuple[Optional[str], Optional[bool]], ...] = (
    ("dtr", False),
    ("rts", False),
    ("dtr", True),
    ("rts", True),
    (None, None),
)

#: Настройка RC_LIDAR_MOTOR → варианты линий (сколько вариантов — столько попыток).
MOTOR_MODES: dict[str, Tuple[Tuple[Optional[str], Optional[bool]], ...]] = {
    "auto": RPLIDAR_MOTOR_VARIANTS,
    "none": ((None, None),),
    "off": ((None, None),),
    "dtr-low": (("dtr", False),),
    "dtr-high": (("dtr", True),),
    "rts-low": (("rts", False),),
    "rts-high": (("rts", True),),
}

RPLIDAR_CMD_STOP = b"\xa5\x25"
RPLIDAR_CMD_SCAN = b"\xa5\x20"
RPLIDAR_SYNC = (0xA5, 0x5A)
RPLIDAR_ANSWER_SYNC = b"\xa5\x5a"
#: Стандартный дескриптор ответа RPLIDAR на SCAN: A5 5A 05 00 00 40 81.
RPLIDAR_ANSWER_SIZE = 0x05
RPLIDAR_ANSWER_TYPES = (0x81, 0x82)

#: Настоящий виток идёт монотонно (шаг ~1° у RPLIDAR и LDS-01, ~3° у LD06), а
#: «точки» из чужих байт прыгают по всему кругу: медиана шага угла между
#: соседними точками отделяет одно от другого с большим запасом.
SCAN_MAX_MEDIAN_STEP_RAD = math.radians(45.0)

#: Мало того, что виток идёт размеренно — он ещё и закрывает весь круг. Поток
#: чужого протокола (LDS-01 на скорости RPLIDAR, LD06 как RPLIDAR и т. п.) может
#: разбираться «в упор»: точки тогда выходят с крошечным шагом, но все в одном
#: направлении — медиана шага у такого мусора нулевая, и раньше он сходил за
#: настоящий скан. Настоящий виток оставляет разрыв не больше пары градусов,
#: так что запас в 90° огромный.
SCAN_MAX_GAP_RAD = math.radians(90.0)

LD06_SYNC = b"\x54\x2c"
LD06_PACKET_LEN = 47

#: LDS-01 / LDS-02 (TurtleBot3 Burger, HLS-LFCD2): кадр 42 байта, 60 пакетов по
#: 6 измерений — 360 точек на виток, скорость 230400.
LDS01_HEADER = 0xFA
LDS01_PACKET_LEN = 42
LDS01_INDEX_BASE = 0xA0
LDS01_PACKETS_PER_SCAN = 60
LDS01_READINGS_PER_PACKET = 6
LDS01_CMD_START = b"b"  # запуск вращения (нужен прошивкам до 2017.10)
LDS01_CMD_STOP = b"e"   # остановка вращения (мы её не шлём: пусть мотор крутится)
LDS01_INVALID_FLAG = 0x8000  # старший бит слова дальности: измерение недостоверно
LDS01_RANGE_MASK = 0x3FFF    # дальность занимает младшие 14 бит
LDS01_RANGE_MIN_M = 0.05
LDS01_RANGE_MAX_M = 16.0
#: Сколько пакетов с неверной КС ждём, прежде чем читать поток без проверки.
#: Снижено с 60 до 20 — клоны с другой формулой КС должны переключаться быстрее,
#: чтобы лидар LDS-01 находился за первый круг, а не после минуты ожидания.
LDS01_CRC_BAD_LIMIT = 20
#: Сколько пакетов подряд должно прийти строго по порядку номеров, чтобы поверить
#: потоку без контрольной суммы (у настоящего лидара номера идут по кругу).
#: Снижено с 12 до 6 — достаточно для подтверждения настоящего LDS-01, но
#: быстрее стартует на прошивках с другой КС.
LDS01_WARMUP_PACKETS = 6

DATA_DIR = Path(__file__).resolve().parent.parent / "data"

#: Сколько секунд источник данных шасси уступает порты лидару, пока тот ищет свой
#: (``RC_LIDAR_PRIORITY_S``). Увеличено с 20 до 60 с — LDS-01 (TurtleBot3) должен
#: гарантированно получить порт за первый круг поиска, даже если шасси только
#: что заняло все USB своей двухсекундной проверкой. Если лидара на роботе нет
#: вовсе, бесконечная уступка оставила бы оба экрана без телеметрии модулей,
#: поэтому окно всё равно конечно.
LIDAR_PRIORITY_S = 60.0
#: Подкаталог ``data/`` с журналом портов (отдельно от замков ``data/lidar-*.lock``).
#: Отдельный каталог нужен не для красоты: статус лидара читают оба экрана по
#: нескольку раз в секунду, а в ``data/`` за месяцы работы накапливаются сотни
#: файлов замков — сканировать такой каталог на каждый запрос нельзя.
PORTS_STATE_SUBDIR = "ports"
#: Сколько секунд держится кэш журнала портов. Файлы читают и запросы к
#: ``/api/lidar``, и потоки чтения портов (модули спрашивают перед каждой
#: проверкой), а ``data/`` на роботе лежит на SD-карте.
PORTS_STATE_CACHE_S = 0.5
#: Через сколько секунд файл состояния лидара считается мёртвым. Менеджер
#: обновляет его каждый круг поиска (``LIDAR_RESCAN_S`` = 2 с), так что запас
#: четырёхкратный: упавший или занятый процесс порты больше не держит.
LIDAR_STATE_TTL_S = 8.0
#: Сколько живёт отметка «на этом порту подтверждены модули шасси» без обновления.
MODULE_TAG_TTL_S = 60.0


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on", "да"}


def _env_int(name: str) -> Optional[int]:
    raw = (os.environ.get(name) or "").strip()
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError:
        log.warning("Лидар: %s=%r — не число, игнорирую", name, raw)
        return None


def _env_float(name: str, default: float) -> float:
    raw = (os.environ.get(name) or "").strip().replace(",", ".")
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        log.warning("Лидар: %s=%r — не число, беру %s", name, raw, default)
        return default


def pyserial_version() -> Optional[str]:
    """Версия pyserial в ЭТОМ интерпретаторе или None, если библиотеки нет.

    Веб-приложение и ``scripts/lidar_check.py`` могут запускаться разными
    интерпретаторами (``/usr/bin/python3`` против ``.venv/bin/python``), и тогда
    проверка лидара работает, а пульт — нет. Поэтому наличие pyserial видно и в
    ``GET /api/lidar`` (``usb_diagnostics.pyserial``), и в журнале запуска.
    """
    try:
        import serial  # pyserial
    except Exception:  # noqa: BLE001 — нет библиотеки / подменён None в тестах
        return None
    version = getattr(serial, "VERSION", None) or getattr(serial, "__version__", None)
    return str(version) if version else "unknown"


def normalize_protocol(value: str) -> str:
    """Приводит RC_LIDAR_PROTOCOL к одному из ``LIDAR_PROTOCOLS``.

    Понимает синонимы: ``lds-01``, ``hls-lfcd2``, ``turtlebot3`` → ``lds01`` —
    чтобы лидар TurtleBot3 можно было назвать так, как он написан на корпусе.
    """
    proto = (value or "").strip().lower().replace(" ", "")
    proto = _PROTOCOL_ALIASES.get(proto, proto)
    return proto if proto in LIDAR_PROTOCOLS else "auto"


#: Скорости, которые имеет смысл предлагать оператору в настройках лидара.
LIDAR_BAUD_CHOICES: Tuple[int, ...] = (115200, 230400, 256000, 460800)
#: Ключи .env, которыми задаётся лидар (их правит веб-панель настроек).
LIDAR_ENV_KEYS: Tuple[str, ...] = (
    "RC_LIDAR_PORT", "RC_LIDAR_PROTOCOL", "RC_LIDAR_BAUD",
    "RC_LIDAR_MOTOR", "RC_LIDAR_AUTOPROBE", "RC_LIDAR_SOURCE",
)


def env_file_path() -> Path:
    """Файл ``.env`` проекта: источник правды для настроек обоих процессов."""
    return Path(__file__).resolve().parent.parent / ".env"


def read_env_file(path: Optional[Path] = None) -> dict[str, str]:
    """Значения из ``.env`` (только строки KEY=VALUE, без изменения окружения)."""
    target = Path(path) if path is not None else env_file_path()
    values: dict[str, str] = {}
    try:
        lines = target.read_text(encoding="utf-8").splitlines()
    except OSError:
        return values
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


# ------------------- последний подтверждённый лидар -------------------- #

#: Сколько кругов перебора подсказка «лидар был тут» ставит свой порт первым.
#: Дальше работает обычный круговой перебор: устаревшая подсказка (лидар
#: переехал на другой USB) не заставляет приложение вечно начинать с мёртвого
#: порта.
LIDAR_HINT_ROUNDS = 2


def found_lidar_path() -> Path:
    """Файл с последним подтверждённым лидаром (живёт между перезапусками).

    ``RC_LIDAR_FOUND_FILE`` переносит файл (так тесты кладут его во временную
    папку). По умолчанию — ``data/lidar-found.json`` рядом с ``.env``.
    """
    custom = os.environ.get("RC_LIDAR_FOUND_FILE")
    if custom:
        return Path(custom)
    return DATA_DIR / "lidar-found.json"


def load_found_lidar() -> Optional[dict[str, Any]]:
    """Последний подтверждённый лидар из :func:`found_lidar_path` (или ``None``).

    Это **подсказка для порядка перебора**, а не истина: её порт и профиль
    проверяются первыми, но все остальные порты и скорости проверяются как
    обычно — устаревшая подсказка не может «сломать» поиск.
    """
    try:
        payload = json.loads(found_lidar_path().read_text("utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(payload, dict):
        return None
    port = payload.get("port")
    if not isinstance(port, str) or not port.strip():
        return None
    try:
        baud = int(payload.get("baud")) if payload.get("baud") else None
    except (TypeError, ValueError):
        baud = None
    protocol = payload.get("protocol")
    motor = payload.get("motor")
    return {
        "port": port.strip(),
        "protocol": protocol if isinstance(protocol, str) else "",
        "baud": baud,
        "motor": motor if isinstance(motor, str) else "",
    }


def save_found_lidar(probe: Any) -> bool:
    """Атомарно запоминает рабочие параметры подтверждённого лидара.

    Вызывается при находке (:meth:`LidarManager._mark_found`): после
    перезапуска приложение начинает с проверенного порта и профиля, а не с
    начала полного перебора. Ошибка записи не мешает скану — подсказка лишь
    ускоряет следующий поиск.
    """
    try:
        port = str(probe.get("port") or "").strip()
    except AttributeError:
        return False
    if not port:
        return False
    try:
        baud = int(probe.get("baud")) if probe.get("baud") else None
    except (TypeError, ValueError):
        baud = None
    payload = {
        "port": port,
        "protocol": str(probe.get("protocol") or ""),
        "baud": baud,
        "motor": str(probe.get("motor") or ""),
        "at": round(time.time(), 1),
    }
    path = found_lidar_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + ".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        os.replace(tmp, path)
    except OSError as exc:
        log.debug("Лидар: не удалось записать %s: %s", path, exc)
        return False
    return True


def normalize_lidar_settings(raw: Any) -> dict[str, Any]:
    """Проверяет настройки лидара из веб-формы и приводит их к одному виду.

    Возвращает ``{"source": str, "port": str, "protocol": str, "baud": str,
    "motor": str, "autoprobe": bool}`` — ровно то, что пишется в ``.env``. Плохое
    значение — ``ValueError`` с понятной оператору причиной (её показывает панель).
    """
    if not isinstance(raw, dict):
        raise ValueError("Настройки лидара должны быть объектом.")
    source_raw = str(raw.get("source") or "auto").strip().lower().replace(" ", "")
    if source_raw not in LIDAR_SOURCES and source_raw not in _SOURCE_ALIASES:
        raise ValueError(
            "Источник данных лидара: ожидается auto (ROS 2, иначе USB), ros (топик "
            "ROS 2 /scan) или serial (USB-порт читает само приложение).")
    source = normalize_source(source_raw)
    port = str(raw.get("port") or "").strip()
    if len(port) > 500 or any(ord(ch) < 32 for ch in port):
        raise ValueError("Порт лидара: слишком длинное или недопустимое значение.")
    for chunk in port.replace(";", ",").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if not chunk.startswith("/") or any(ch in chunk for ch in " \t\r\n;&|$`\\\"'"):
            raise ValueError(
                "Порт лидара: ожидается абсолютный путь вида /dev/ttyUSB0 "
                "или /dev/serial/by-id/… (можно список через запятую и glob).")
        if ".." in chunk.split("/"):
            raise ValueError("Порт лидара: путь не должен содержать «..».")
        port = port.replace(";", ",")
    protocol_raw = str(raw.get("protocol") or "auto").strip().lower()
    protocol = normalize_protocol(protocol_raw)
    if protocol_raw.replace(" ", "") not in LIDAR_PROTOCOLS and \
            protocol_raw.replace(" ", "") not in _PROTOCOL_ALIASES:
        raise ValueError(
            "Протокол лидара: ожидается auto, rplidar, ld06 или lds01 "
            "(LDS-01/LDS-02 от TurtleBot3).")
    baud_raw = str(raw.get("baud") or "").strip().lower()
    if baud_raw in {"", "auto", "0", "авто", "автоподбор"}:
        baud = ""
    else:
        try:
            baud_value = int(baud_raw)
        except ValueError:
            raise ValueError("Скорость лидара: ожидается число (например 256000) или «авто».") from None
        if not 1200 <= baud_value <= 4_000_000:
            raise ValueError("Скорость лидара: значение вне диапазона 1200…4000000 бод.")
        baud = str(baud_value)
    motor = str(raw.get("motor") or "auto").strip().lower()
    if motor not in MOTOR_MODES:
        raise ValueError(
            "Мотор RPLIDAR: ожидается auto, none, dtr-low, dtr-high, rts-low или rts-high.")
    autoprobe_raw = raw.get("autoprobe", True)
    if isinstance(autoprobe_raw, bool):
        autoprobe = autoprobe_raw
    elif isinstance(autoprobe_raw, (int, float)):
        autoprobe = bool(autoprobe_raw)
    else:
        autoprobe = str(autoprobe_raw).strip().lower() in {"1", "true", "yes", "on", "да"}
    return {"source": source, "port": port, "protocol": protocol, "baud": baud,
            "motor": motor, "autoprobe": autoprobe}


def lidar_settings_from_env(env: Optional[dict[str, str]] = None) -> dict[str, Any]:
    """Действующие настройки лидара из окружения (или переданного словаря)."""
    values = env if env is not None else os.environ
    return normalize_lidar_settings({
        "source": values.get("RC_LIDAR_SOURCE") or "auto",
        "port": values.get("RC_LIDAR_PORT") or "",
        "protocol": values.get("RC_LIDAR_PROTOCOL") or "auto",
        "baud": values.get("RC_LIDAR_BAUD") or "",
        "motor": values.get("RC_LIDAR_MOTOR") or "auto",
        "autoprobe": values.get("RC_LIDAR_AUTOPROBE") or "1",
    })


def normalize_angle(rad: float) -> float:
    """Нормализует угол в диапазон [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def scan_from_ranges(
    ranges: list[float],
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float = 0.05,
    range_max: float = 12.0,
) -> list[dict[str, float]]:
    """Преобразует массив дальностей в список точек [{a, r}].

    a — угол в радианах [-pi, pi], 0 — вперёд.
    r — дальность в метрах.
    """
    points: list[dict[str, float]] = []
    r_min = float(range_min)
    r_max = float(range_max)
    inc = float(angle_increment)
    a_start = float(angle_min)

    for i, r in enumerate(ranges):
        try:
            dist = float(r)
        except (ValueError, TypeError):
            continue
        if not math.isfinite(dist) or dist < r_min or dist > r_max:
            continue
        a = a_start + i * inc
        points.append({"a": round(normalize_angle(a), 4), "r": round(dist, 3)})
    return points


def parse_rplidar_point(b: bytes) -> Optional[dict[str, float]]:
    """Разбирает 5-байтовый пакет RPLIDAR (Standard Scan).

    b[0]: sync, quality
    b[1]: check bit, angle_q6[0:6]
    b[2]: angle_q6[7:14]
    b[3]: distance_q2[0:7]
    b[4]: distance_q2[8:15]
    """
    if len(b) < 5:
        return None
    # проверка sync/inv_sync и check_bit
    s = b[0] & 0x01
    inv_s = (b[0] >> 1) & 0x01
    if s == inv_s:  # должны быть инверсны
        return None
    if not (b[1] & 0x01):  # check bit должен быть 1
        return None

    angle_q6 = (b[1] >> 1) | (b[2] << 7)
    angle_deg = angle_q6 / 64.0
    dist_q2 = b[3] | (b[4] << 8)
    dist_m = (dist_q2 / 4.0) / 1000.0

    if dist_m < 0.05 or dist_m > 16.0:
        return None

    # В RPLIDAR угол по часовой стрелке, в RUS SLAM против часовой от оси X
    rad = normalize_angle(-math.radians(angle_deg))
    return {"a": round(rad, 4), "r": round(dist_m, 3)}


def lds01_checksum(data: bytes) -> int:
    """Контрольная сумма LDS-01 (HLS-LFCD2) по первым 40 байтам пакета.

    Алгоритм из драйвера ROBOTIS/HLDS: идём по данным 16-битными словами
    (младший байт первым), каждый раз сдвигая накопленное влево на бит, затем
    складываем младшие 15 бит со всем остальным — получается 15-битное число.
    """
    chk32 = 0
    limit = len(data) - (len(data) % 2)  # неполный хвост в расчёт не идёт
    for i in range(0, limit, 2):
        chk32 = (chk32 << 1) + (data[i] | (data[i + 1] << 8))
    return ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF


def lds01_packet_crc_ok(pkt: bytes) -> bool:
    """Совпадает ли контрольная сумма пакета LDS-01 с записанной в нём."""
    if len(pkt) < LDS01_PACKET_LEN:
        return False
    expected = pkt[40] | (pkt[41] << 8)
    return lds01_checksum(pkt[: LDS01_PACKET_LEN - 2]) == expected


def parse_lds01_packet(pkt: bytes) -> list[dict[str, float]]:
    """Разбирает 42-байтовый пакет LDS-01 / LDS-02 (TurtleBot3 Burger).

    Раскладка пакета:

    ============ ==========================================================
    байт 0       0xFA — заголовок
    байт 1       0xA0 + номер пакета (0…59), метр: 6° на пакет
    байты 2..3   скорость вращения, младший байт первым
    байты 4..39  шесть измерений по 6 байт: интенсивность (u16),
                 дальность в мм (u16), резерв (u16)
    байты 40..41 контрольная сумма (см. :func:`lds01_checksum`)
    ============ ==========================================================

    Угол точки считается как у драйвера ROBOTIS (``hls_lfcd_lds_driver``):
    359 − (номер пакета·6 + i) градусов против часовой стрелки от направления
    вперёд. Нумерацию и направление отсчёта не меняем — иначе карта повернулась
    бы относительно ROS-режима, где этот лидар работает штатно.

    В слове дальности старший бит означает недостоверное измерение (такие точки
    выбрасываются), а сама дальность занимает младшие 14 бит. Контрольная сумма
    здесь не проверяется (см. :func:`lds01_packet_crc_ok` и ``Lds01Stream``).
    """
    if len(pkt) < LDS01_PACKET_LEN or pkt[0] != LDS01_HEADER:
        return []
    index = pkt[1] - LDS01_INDEX_BASE
    if not 0 <= index < LDS01_PACKETS_PER_SCAN:
        return []

    pts: list[dict[str, float]] = []
    for i in range(LDS01_READINGS_PER_PACKET):
        offset = 4 + i * 6
        raw = pkt[offset + 2] | (pkt[offset + 3] << 8)
        if raw & LDS01_INVALID_FLAG:
            continue
        dist_m = (raw & LDS01_RANGE_MASK) / 1000.0
        if dist_m < LDS01_RANGE_MIN_M or dist_m > LDS01_RANGE_MAX_M:
            continue
        angle_deg = 359.0 - float(index * LDS01_READINGS_PER_PACKET + i)
        rad = normalize_angle(math.radians(angle_deg))
        pts.append({"a": round(rad, 4), "r": round(dist_m, 3)})
    return pts


def ld06_crc_ok(pkt: bytes) -> bool:
    """Контрольная сумма пакета LD06/LD19: XOR всех байт, кроме последнего."""
    if len(pkt) < LD06_PACKET_LEN:
        return False
    crc = 0
    for byte in pkt[: LD06_PACKET_LEN - 1]:
        crc ^= byte
    return crc == pkt[LD06_PACKET_LEN - 1]


def parse_ld06_packet(pkt: bytes) -> list[dict[str, float]]:
    """Разбирает 47-байтовый пакет LD06 / LD19.

    Заголовок: 0x54, 0x2C.
    12 точек измерений на пакет.
    Контрольная сумма здесь не проверяется (см. ``ld06_crc_ok``): у части
    клонов она не считается, а поток всё равно несёт корректные данные.
    """
    if len(pkt) < 47 or pkt[0] != 0x54 or pkt[1] != 0x2C:
        return []
    start_angle = (pkt[4] | (pkt[5] << 8)) * 0.01
    end_angle = (pkt[42] | (pkt[43] << 8)) * 0.01
    if end_angle < start_angle:
        end_angle += 360.0

    step = (end_angle - start_angle) / 11.0 if end_angle != start_angle else 0.0
    pts: list[dict[str, float]] = []

    for i in range(12):
        idx = 6 + i * 3
        dist_mm = pkt[idx] | (pkt[idx + 1] << 8)
        conf = pkt[idx + 2]
        if conf < 10 or dist_mm < 50 or dist_mm > 15000:
            continue
        angle_deg = (start_angle + i * step) % 360.0
        rad = normalize_angle(-math.radians(angle_deg))
        pts.append({"a": round(rad, 4), "r": round(dist_mm / 1000.0, 3)})
    return pts


def generate_demo_scan(t: Optional[float] = None, speed: float = 0.5) -> list[dict[str, float]]:
    """Генерирует скан помещения склада со стенами и столбиками."""
    now = time.time() if t is None else t
    pts: list[dict[str, float]] = []
    num_rays = 180

    for i in range(num_rays):
        a = (i / num_rays) * math.pi * 2.0 - math.pi
        # Базовый прямоугольный контур помещения ~5×4 м
        cos_a = math.cos(a)
        sin_a = math.sin(a)
        r_x = abs(3.5 / cos_a) if abs(cos_a) > 1e-4 else 12.0
        r_y = abs(2.5 / sin_a) if abs(sin_a) > 1e-4 else 12.0
        r = min(r_x, r_y)

        # Небольшая динамическая волна и неровность стен
        r += 0.08 * math.sin(a * 8.0 + now * 0.5) + 0.04 * math.cos(a * 15.0)

        # Столбик 1 (угол ~35°)
        if abs(a - 0.6) < 0.08:
            r = min(r, 1.8 + 0.05 * math.sin(now))
        # Столбик 2 (угол ~-50°)
        if abs(a + 0.9) < 0.07:
            r = min(r, 2.4 + 0.04 * math.cos(now * 1.2))
        # Стеллаж сбоку (угол ~110°)
        if 1.7 < a < 2.2:
            r = min(r, 2.1)

        # Движение робота влияет на скан
        r += 0.06 * math.sin(now * speed * 2.0 + a * 3.0)
        r = max(0.2, min(9.8, r))

        pts.append({"a": round(a, 4), "r": round(r, 3)})

    return pts


# ============================================================================
# Разбор потока данных лидара
# ============================================================================
class ScanAccumulator:
    """Собирает точки в виток и публикует его.

    Лидару нужно время на оборот (RPLIDAR A1 — 5…10 Гц, LD06/LD19 — 10…15 Гц),
    поэтому публикуем не каждую точку, а готовый виток; если виток не
    закрывается (клоны без достоверного бита начала), отдаём накопленное по
    паузе, чтобы на экране не было пустой карты.
    """

    def __init__(
        self,
        publish: Callable[[list[dict[str, float]]], None],
        *,
        min_points: int = LIDAR_MIN_POINTS,
        gap_s: float = LIDAR_PUBLISH_GAP_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._publish = publish
        self._min_points = max(1, int(min_points))
        self._gap_s = float(gap_s)
        self._clock = clock
        self._points: list[dict[str, float]] = []
        self._published_at = clock()
        self.published = 0

    @property
    def pending(self) -> int:
        return len(self._points)

    def add(self, points: Sequence[dict[str, float]], revolution_done: bool = False) -> bool:
        """Добавляет точки. True — опубликовали виток (или накопленное по паузе)."""
        published = False
        if revolution_done:
            published = self.flush()
        if points:
            self._points.extend(points)
        if len(self._points) >= self._min_points and self._clock() - self._published_at >= self._gap_s:
            published = self.flush() or published
        return published

    def flush(self) -> bool:
        """Публикует накопленное, если точек хватает. True — опубликовали."""
        points = self._points
        if len(points) < self._min_points:
            return False
        self._points = []
        self._published_at = self._clock()
        self.published += 1
        self._publish(points)
        return True


class RplidarStream:
    """Разбор стандартного протокола RPLIDAR: точки по 5 байт, бит начала витка."""

    def __init__(self, *, max_buffer: int = 4096) -> None:
        self.buffer = bytearray()
        self.max_buffer = max_buffer
        self.points = 0

    def feed(self, chunk: bytes) -> list[Tuple[dict[str, float], bool]]:
        """Добавляет байты потока → [(точка, начало_витка), ...]."""
        out: list[Tuple[dict[str, float], bool]] = []
        if chunk:
            self.buffer.extend(chunk)
            if len(self.buffer) > self.max_buffer:
                del self.buffer[: len(self.buffer) - 5]
        while len(self.buffer) >= 5:
            point = parse_rplidar_point(bytes(self.buffer[:5]))
            if point is None:
                del self.buffer[:1]  # синхронизация потеряна — ищем заново
                continue
            is_start = bool(self.buffer[0] & 0x01)
            del self.buffer[:5]
            self.points += 1
            out.append((point, is_start))
        return out


class Ld06Stream:
    """Разбор потока LD06/LD19: пакеты 0x54 0x2C по 12 точек."""

    def __init__(
        self,
        *,
        check_crc: bool = True,
        crc_bad_limit: int = 40,
        max_buffer: int = 8192,
    ) -> None:
        self.buffer = bytearray()
        self.check_crc = check_crc
        self.crc_bad_limit = crc_bad_limit
        self.crc_ok = 0
        self.crc_bad = 0
        self.packets = 0
        self.max_buffer = max_buffer
        self._prev_start: Optional[float] = None

    def feed(self, chunk: bytes) -> list[Tuple[list[dict[str, float]], bool]]:
        """Добавляет байты потока → [(точки пакета, виток_завершён), ...]."""
        out: list[Tuple[list[dict[str, float]], bool]] = []
        if chunk:
            self.buffer.extend(chunk)
            if len(self.buffer) > self.max_buffer:
                del self.buffer[: len(self.buffer) - LD06_PACKET_LEN]
        while True:
            packet = self._next_packet()
            if packet is None:
                return out
            pts = parse_ld06_packet(packet)
            if not pts:
                continue
            self.packets += 1
            start_angle = (packet[4] | (packet[5] << 8)) * 0.01
            revolution = self._prev_start is not None and start_angle + 1e-9 < self._prev_start
            self._prev_start = start_angle
            out.append((pts, revolution))

    def _next_packet(self) -> Optional[bytes]:
        while True:
            start = self.buffer.find(LD06_SYNC)
            if start < 0:
                # заголовок мог разрезаться между чтениями — держим последний байт
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return None
            if start:
                del self.buffer[:start]
            if len(self.buffer) < LD06_PACKET_LEN:
                return None
            packet = bytes(self.buffer[:LD06_PACKET_LEN])
            if self.check_crc and not ld06_crc_ok(packet):
                self.crc_bad += 1
                if self.crc_bad >= self.crc_bad_limit and self.crc_ok == 0:
                    # у части клонов CRC не считается — читаем поток без проверки
                    self.check_crc = False
                    log.info("Лидар: у LD06/LD19 не сходится CRC — читаю поток без проверки")
                del self.buffer[:1]
                continue
            del self.buffer[:LD06_PACKET_LEN]
            self.crc_ok += 1
            return packet


class Lds01Stream:
    """Разбор потока LDS-01 / LDS-02 (TurtleBot3 Burger): кадры 0xFA по 42 байта.

    Виток — 60 пакетов (360 точек); границей витка служит пакет с индексом 0xA0,
    как и в драйвере ROBOTIS. Контрольная сумма проверяется, но если у устройства
    она не сходится ни на одном пакете (встречаются прошивки с другой формулой),
    поток читается без неё — тогда единственной защитой остаётся строгий порядок
    номеров пакетов: у настоящего лидара он идёт по кругу 0xA0…0xDB без пропусков,
    и точкам верим лишь после серии из ``LDS01_WARMUP_PACKETS`` пакетов подряд.
    """

    def __init__(
        self,
        *,
        check_crc: bool = True,
        crc_bad_limit: int = LDS01_CRC_BAD_LIMIT,
        max_buffer: int = 8192,
    ) -> None:
        self.buffer = bytearray()
        self.check_crc = check_crc
        self.crc_bad_limit = crc_bad_limit
        self.crc_ok = 0            # пакетов принято с верной контрольной суммой
        self.crc_bad = 0           # пакетов забраковано по контрольной сумме
        self.packets = 0           # пакетов отдано
        self.max_buffer = max_buffer
        self._expected_index: Optional[int] = None
        self._run = 0             # сколько пакетов подряд идут по порядку номеров
        self._confirmed = False   # потоку без контрольной суммы уже можно верить

    def feed(self, chunk: bytes) -> list[Tuple[list[dict[str, float]], bool]]:
        """Добавляет байты потока → [(точки пакета, виток_завершён), ...]."""
        out: list[Tuple[list[dict[str, float]], bool]] = []
        if chunk:
            self.buffer.extend(chunk)
            if len(self.buffer) > self.max_buffer:
                del self.buffer[: len(self.buffer) - LDS01_PACKET_LEN]
        while True:
            packet = self._next_packet()
            if packet is None:
                return out
            index = packet[1] - LDS01_INDEX_BASE
            self.packets += 1
            # Пакет с индексом 0xA0 начинает новый виток: к этому моменту в
            # накопителе лежат точки предыдущего (359…0°), и он закрывается.
            out.append((parse_lds01_packet(packet), index == 0))

    def _next_packet(self) -> Optional[bytes]:
        """Ищет в буфере очередной пакет; None — данных пока не хватает."""
        while True:
            start = self._find_start()
            if start < 0:
                # заголовок мог разрезаться между чтениями — держим последний байт
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return None
            if start:
                del self.buffer[:start]
            if len(self.buffer) < LDS01_PACKET_LEN:
                return None
            packet = bytes(self.buffer[:LDS01_PACKET_LEN])

            if self.check_crc and not lds01_packet_crc_ok(packet):
                self.crc_bad += 1
                if self.crc_bad >= self.crc_bad_limit and self.crc_ok == 0:
                    self.check_crc = False
                    log.info(
                        "Лидар: у LDS-01/LDS-02 не сходится контрольная сумма — "
                        "читаю поток без проверки"
                    )
                del self.buffer[:1]  # контрольная сумма не та — ищем заголовок дальше
                continue

            index = packet[1] - LDS01_INDEX_BASE
            if not self.check_crc and not self._confirmed:
                # Без контрольной суммы верить пакету сразу нельзя: «пакеты» из
                # чужих байт тоже начинаются с 0xFA и допустимого номера. Зато
                # у настоящего лидара номера идут строго по кругу, поэтому поток
                # признаём своим только после серии пакетов подряд по порядку.
                self._run = self._run + 1 if index == self._expected_index else 1
                self._expected_index = (index + 1) % LDS01_PACKETS_PER_SCAN
                del self.buffer[:LDS01_PACKET_LEN]
                if self._run < LDS01_WARMUP_PACKETS:
                    continue
                self._confirmed = True
                log.info(
                    "Лидар: LDS-01/LDS-02 читается без контрольной суммы — "
                    "поток подтверждён по порядку номеров пакетов"
                )
                return packet

            del self.buffer[:LDS01_PACKET_LEN]
            self._expected_index = (index + 1) % LDS01_PACKETS_PER_SCAN
            if self.check_crc:
                self.crc_ok += 1
            return packet

    def _find_start(self) -> int:
        """Индекс первого байта кадра (0xFA + допустимый номер пакета) или −1."""
        last = LDS01_INDEX_BASE + LDS01_PACKETS_PER_SCAN - 1
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == LDS01_HEADER and LDS01_INDEX_BASE <= self.buffer[i + 1] <= last:
                return i
        return -1


def read_rplidar_stream(
    ser,
    publish: Callable[[list[dict[str, float]]], None],
    stop: Callable[[], bool],
    *,
    timeout: float = LIDAR_DATA_TIMEOUT_S,
    min_points: int = LIDAR_MIN_POINTS,
) -> int:
    """Читает точки RPLIDAR до тишины и публикует витки. Возвращает число точек."""
    stream = RplidarStream()
    acc = ScanAccumulator(publish, min_points=min_points)
    last_point = time.monotonic()
    while not stop():
        chunk = ser.read(64)
        for point, is_start in stream.feed(chunk):
            last_point = time.monotonic()
            acc.add([point], revolution_done=is_start)
        if not chunk and time.monotonic() - last_point > timeout:
            break
    acc.flush()  # хвост витка — чтобы последний скан не пропал
    return stream.points


def read_ld06_stream(
    ser,
    publish: Callable[[list[dict[str, float]]], None],
    stop: Callable[[], bool],
    *,
    timeout: float = LIDAR_DATA_TIMEOUT_S,
    min_points: int = LIDAR_MIN_POINTS,
    check_crc: bool = True,
) -> int:
    """Читает поток LD06/LD19 до тишины и публикует витки. Возвращает число точек."""
    stream = Ld06Stream(check_crc=check_crc)
    acc = ScanAccumulator(publish, min_points=min_points)
    last_point = time.monotonic()
    total = 0
    while not stop():
        chunk = ser.read(256)
        for points, revolution in stream.feed(chunk):
            last_point = time.monotonic()
            total += len(points)
            acc.add(points, revolution_done=revolution)
        if not chunk and time.monotonic() - last_point > timeout:
            break
    acc.flush()
    return total


def lds01_start_scan(ser) -> None:
    """Просит LDS-01 начать вращение (``b``) — надёжно, с повторами.

    Нужно прошивкам до 2017.10, новым не мешает. Команду остановки (``e``) не
    отправляем: мотор должен крутиться и после закрытия порта — иначе при каждом
    переподключении лидар тратил бы секунды на раскрутку и перебор его не находил.

    Для 100% старта отправляем 'b' несколько раз с паузой: на холодном старте
    первая команда может потеряться, пока USB-адаптер ещё инициализируется, а
    вторая уже доходит. Перед отправкой чистим входной буфер, чтобы не разбирать
    мусор от предыдущей сессии.
    """
    try:
        # Чистим вход — чтобы не ловить хвост от чужого протокола
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        ser.write(LDS01_CMD_START)
        # Некоторые адаптеры CP2102 глотают первый байт — шлём ещё раз
        time.sleep(0.02)
        ser.write(LDS01_CMD_START)
    except Exception as exc:  # noqa: BLE001 — порт мог пропасть
        log.debug("Лидар: не удалось отправить команду запуска LDS-01: %s", exc)


def read_lds01_stream(
    ser,
    publish: Callable[[list[dict[str, float]]], None],
    stop: Callable[[], bool],
    *,
    timeout: float = LIDAR_DATA_TIMEOUT_S,
    min_points: int = LIDAR_MIN_POINTS,
    check_crc: bool = True,
) -> int:
    """Читает поток LDS-01/LDS-02 до тишины и публикует витки. Возвращает число точек.

    Для LDS-01 (TurtleBot3 Burger) периодически переотправляет команду 'b',
    если точки долго не идут: прошивки до 2017.10 без неё не крутятся, а после
    чужой команды 'e' мотор стоит. Повторная отправка не мешает новым прошивкам.
    """
    stream = Lds01Stream(check_crc=check_crc)
    acc = ScanAccumulator(publish, min_points=min_points)
    last_point = time.monotonic()
    last_start = time.monotonic()
    total = 0
    # Сразу просим вращение — нужно старым прошивкам
    lds01_start_scan(ser)
    while not stop():
        chunk = ser.read(256)
        for points, revolution in stream.feed(chunk):
            last_point = time.monotonic()
            total += len(points)
            acc.add(points, revolution_done=revolution)
        if not chunk and time.monotonic() - last_point > timeout:
            break
        # Если точек нет дольше 0.6 с — возможно мотор не запущен, шлём 'b' ещё раз
        if total == 0 and time.monotonic() - last_start > 0.6:
            lds01_start_scan(ser)
            last_start = time.monotonic()
    acc.flush()
    return total


def scan_looks_like_rotation(
    points: Sequence[dict[str, float]],
    max_median_step: float = SCAN_MAX_MEDIAN_STEP_RAD,
    max_gap: float = SCAN_MAX_GAP_RAD,
) -> bool:
    """Похож ли набор точек на настоящий виток лидара (луч идёт размеренно)?

    Настоящий виток RPLIDAR/LDS-01/LD06 идёт монотонно — шаг между соседними
    точками около градуса, и точки закрывают весь круг. Если скорость порта
    угадана неверно (например, поток LDS-01 читают как RPLIDAR на 115200) или
    выбран чужой протокол, «точки» получаются из случайных байт: они либо
    прыгают по всему кругу, либо, наоборот, разбираются «в упор» — встают в одно
    направление с нулевым шагом. Поэтому проверяем оба признака: медиана шага
    (мусор с прыжками) и наибольший разрыв между соседними углами (мусор,
    слипшийся в одну сторону). На переборе такой виток не считается удачей:
    иначе лидар «находился» бы, а карта на экране была из мусора.
    """
    if len(points) < 3:
        return False
    steps = sorted(
        abs(normalize_angle(cur["a"] - prev["a"]))
        for prev, cur in zip(points, points[1:])
    )
    if steps[len(steps) // 2] >= max_median_step:
        return False
    angles = sorted(normalize_angle(p["a"]) % math.tau for p in points)
    gaps = [cur - prev for prev, cur in zip(angles, angles[1:])]
    gaps.append(angles[0] + math.tau - angles[-1])
    return max(gaps) <= max_gap


# ============================================================================
# Подбор порта и параметров лидара
# ============================================================================
def _natural_key(path: str) -> Tuple[str, int]:
    """Сортировка /dev/ttyUSB2 < /dev/ttyUSB10 (обычный sorted даёт наоборот)."""
    digits = re.sub(r"\D", "", path)
    return (path.rstrip("0123456789"), int(digits) if digits else 0)


def excluded_ports() -> set[str]:
    """Порты, которые лидару трогать нельзя: мастер и модули шасси."""
    ports: set[str] = set()
    raw = os.environ.get("RC_LIDAR_EXCLUDE") or ""
    for name in ("RC_SLAM_PORT", "RC_SLAM_MASTER_PORT"):
        value = os.environ.get(name)
        if value:
            raw = raw + "," + value
    raw = raw + "," + (os.environ.get("RC_SLAM_PORTS") or "")
    for chunk in raw.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        ports.add(chunk)
        ports.add(os.path.realpath(chunk))
    return ports


def port_lock_file(port: str) -> Path:
    """Файл межпроцессного замка порта лидара.

    Замок один на **устройство**, а не на имя пути: ``/dev/ttyUSB1`` и
    ``/dev/serial/by-id/usb-Silicon_Labs_CP2102…`` — это один и тот же порт, и
    пульт с бортовым экраном могут назвать его по-разному (один берёт порт из
    ``RC_LIDAR_PORT``, другой нашёл его перебором). Без приведения к realpath
    второй процесс считал бы порт свободным, открывал его и перехватывал байты
    у первого — скан не собирался ни у кого.
    """
    real = port
    try:
        resolved = os.path.realpath(port)
    except OSError:
        resolved = ""
    if resolved:
        real = resolved
    name = re.sub(r"[^A-Za-z0-9_.-]", "_", real.strip("/").replace("/", "-")) or "port"
    return DATA_DIR / ("lidar-%s.lock" % name)


def try_lock_port(port: str) -> Optional[int]:
    """Занять порт монопольно. Возвращает fd замка или None, если порт занят.

    Тот же замок используют модули шасси (``slam_gui/backend.py``): источник
    данных шасси держит его на время проверки порта и снимает, если телеметрия
    не пошла. Порт, который уже читает лидар, они не открывают, и наоборот —
    поэтому один USB-порт не может читаться двумя процессами сразу (пульт 8080
    и бортовой экран 8090 поднимают оба источника одновременно).
    """
    if fcntl is None or not port:  # не-POSIX: замков нет, читаем как читали
        return -1
    try:
        path = port_lock_file(port)
        path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(str(path), os.O_RDWR | os.O_CREAT, 0o644)
    except OSError as exc:  # нет каталога data/ и т. п. — не мешаем чтению
        log.debug("Лидар: замок порта %s недоступен: %s", port, exc)
        return -1
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        return None
    return fd


def unlock_port(fd: Optional[int]) -> None:
    """Освободить замок порта (fd из :func:`try_lock_port`)."""
    if fd is None or fd == -1:
        return
    try:
        if fcntl is not None:
            fcntl.flock(fd, fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        os.close(fd)
    except OSError:
        pass


def port_locked_by_other(port: str) -> bool:
    """Держит ли порт прямо сейчас другой процесс (лидар или модуль шасси)."""
    fd = try_lock_port(port)
    if fd is None:
        return True
    unlock_port(fd)
    return False


# ============================================================================
# Общий журнал портов: кто их ищет и где подтвердились модули шасси
# ============================================================================
#
# Замка порта (выше) мало: он говорит «занят / свободен» и не различает, кто
# именно держит порт. Из-за этого на роботе с двумя адаптерами (модули шасси и
# лидар) складывалась тупиковая картина: источник данных шасси брал порты на
# свою двухсекундную проверку ровно в тот момент, когда лидар начинал перебор,
# и лидар писал «лидар не найден: все 2 порт(ов) заняты другой программой», а
# дальше в журнале было тихо (`_set_status` пишет причину только при её смене).
# Читать это как «лидар сломан» нельзя, но и понять, что происходит, по журналу
# тоже нельзя. Поэтому два процесса робота обмениваются ещё и состоянием:
#
#   * ``data/lidar-state-<pid>.json`` — что делает менеджер лидара: ищет порт
#     (`searching`, пока действует приоритет ``RC_LIDAR_PRIORITY_S``), уже читает
#     его (`found`, с именем порта) или живёт на скане второго процесса (`peer`);
#   * ``data/chassis-ports-<pid>.json`` — какие порты источник данных шасси
#     подтвердил кадрами телеметрии: лидар такие порты не перебирает, а модули,
#     в свою очередь, не уступают их лидару.
#
# Файлов ровно по одному на процесс: чужие записи никто не переписывает, а
# живость проверяется по pid (упавший процесс порты не держит) и по времени.


def _same_port(first: str, second: str) -> bool:
    """Один и тот же порт по устройству: ``/dev/ttyUSB1`` и symlink из by-id."""
    if first == second:
        return True
    try:
        return os.path.realpath(first) == os.path.realpath(second)
    except OSError:  # pragma: no cover - realpath практически не падает
        return False


def _port_key(port: str) -> str:
    """Ключ порта для журналов: имя устройства, а не путь к symlink'у."""
    try:
        return os.path.basename(os.path.realpath(port) or port)
    except OSError:  # pragma: no cover
        return os.path.basename(port)


def _pid_alive(pid: int) -> bool:
    """Жив ли процесс, записавший состояние (после падения файл — мусор)."""
    if pid <= 0:
        return True                     # pid неизвестен — решает только TTL
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except OSError:                     # нет прав спросить — считаем живым
        return True
    return True


def _write_state_file(path: Path, payload: dict[str, Any]) -> None:
    """Пишет JSON состояния атомарно: читатель не увидит половину файла."""
    tmp = path.with_name(path.name + ".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps(payload, ensure_ascii=False), "utf-8")
        os.replace(tmp, path)
    except OSError as exc:
        log.debug("Лидар: не записать %s: %s", path.name, exc)
        try:
            tmp.unlink()
        except OSError:
            pass


def _read_state_file(path: Path, ttl: float) -> Optional[dict[str, Any]]:
    """Читает файл состояния: None — файла нет, он битый, старый или мёртвый."""
    try:
        data = json.loads(path.read_text("utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    try:
        at = float(data.get("at") or 0.0)
        pid = int(data.get("pid") or 0)
    except (TypeError, ValueError):
        return None
    if ttl > 0 and time.time() - at > ttl:
        return None
    if not _pid_alive(pid):
        return None
    return data


def ports_state_dir() -> Path:
    """Каталог журнала портов: ``data/ports`` (см. ``PORTS_STATE_SUBDIR``)."""
    return DATA_DIR / PORTS_STATE_SUBDIR


def lidar_state_file(pid: Optional[int] = None) -> Path:
    """Файл состояния лидара этого процесса (что он делает с портами).

    Записей в файле может быть несколько — по одной на сеанс поиска
    (см. :func:`_new_searcher`): менеджеров в одном процессе бывает больше одного.
    """
    return ports_state_dir() / ("lidar-state-%d.json" % (os.getpid() if pid is None else pid))


#: Ключ записи, когда состояние пишут напрямую (:func:`publish_lidar_state` без
#: своего сеанса): так писали прежние версии робота, так пишут диагностика и тесты.
_SEARCHER_DEFAULT = "default"

#: Имена сеансов поиска: у каждого менеджера своя запись в файле состояния
#: процесса. Пульт и бортовой экран могут жить в одном процессе (в тестах
#: менеджеров и вовсе несколько), и одно поле ``since`` на процесс означало бы,
#: что запись соседа, начавшего искать раньше, гасит окно приоритета живого
#: поиска: порты достались бы модулям шасси прямо посреди перебора лидара.
_searcher_lock = threading.Lock()
_searcher_seq = 0
#: Файл состояния принадлежит процессу, но пишут в него его же потоки: свой замок
#: нужен, чтобы «прочитать — поправить свою запись — записать» не потеряло чужую.
_state_file_lock = threading.Lock()


def _new_searcher() -> str:
    """Имя нового сеанса поиска (ключ записи в файле состояния процесса)."""
    global _searcher_seq
    with _searcher_lock:
        _searcher_seq += 1
        return "s%d" % _searcher_seq


def _state_records(payload: Any) -> dict[str, dict[str, Any]]:
    """Записи о лидаре в файле состояния: сеанс поиска → запись.

    ``{'pid', 'state', 'port', 'at', 'since'}`` (одна запись) — формат прежних
    версий и ручных записей; читается как запись сеанса ``default``.
    """
    if not isinstance(payload, dict):
        return {}
    records = payload.get("records")
    if isinstance(records, dict):
        return {str(name): item for name, item in records.items() if isinstance(item, dict)}
    if isinstance(payload.get("state"), str):
        return {_SEARCHER_DEFAULT: {key: value for key, value in payload.items()
                                    if key != "records"}}
    return {}


def _state_records_from_file(path: Path) -> dict[str, dict[str, Any]]:
    """Записи из файла состояния как есть (без отсева по TTL): для правки своих."""
    try:
        payload = json.loads(path.read_text("utf-8"))
    except (OSError, ValueError):
        return {}
    return _state_records(payload)


def _prune_state_records(records: dict[str, dict[str, Any]], now: float) -> None:
    """Убирает записи, которые уже никто не прочитает: они старше ``LIDAR_STATE_TTL_S``."""
    for name, record in list(records.items()):
        try:
            at = float(record.get("at") or 0.0)
        except (TypeError, ValueError):
            at = 0.0
        if now - at > LIDAR_STATE_TTL_S:
            del records[name]


def _read_lidar_records(path: Path, ttl: float) -> list[dict[str, Any]]:
    """Живые записи о лидаре из файла состояния: по одной на каждый сеанс поиска.

    Мёртвый процесс, битый файл и записи старше ``ttl`` не отдаются: иначе файл
    давно выключенного робота держал бы порты (см. :func:`lidar_holds_port`).
    """
    try:
        payload = json.loads(path.read_text("utf-8"))
    except (OSError, ValueError):
        return []
    if not isinstance(payload, dict):
        return []
    try:
        pid = int(payload.get("pid") or 0)
    except (TypeError, ValueError):
        return []
    if not _pid_alive(pid):
        return []
    now = time.time()
    #: Окно, в котором лидар вообще вправе просить порты (см. ``_publish_state``).
    window = _env_float("RC_LIDAR_PRIORITY_S", LIDAR_PRIORITY_S)
    alive: list[dict[str, Any]] = []
    for searcher, record in _state_records(payload).items():
        try:
            at = float(record.get("at") or 0.0)
        except (TypeError, ValueError):
            continue
        if record.get("state") == "searching":
            # «Ищу порт» ограничено не свежестью записи, а окном приоритета,
            # которое считается от ``since``: круг поиска по чужому (шумному)
            # устройству идёт десятками секунд, и источник данных шасси не должен
            # решать, что лидар сдался, посреди круга. Запись без ``since`` или
            # с истёкшим окном ничего не держит.
            try:
                since = float(record.get("since") or 0.0)
            except (TypeError, ValueError):
                continue
            if not since or window <= 0.0 or now - since > window:
                continue
        elif ttl > 0 and now - at > ttl:
            continue
        item = dict(record)
        item["pid"] = pid
        item["searcher"] = searcher
        alive.append(item)
    return alive


def publish_lidar_state(state: str, port: Optional[str] = None, *,
                        since: Optional[float] = None,
                        searcher: str = _SEARCHER_DEFAULT) -> None:
    """Записывает состояние лидара для источника данных шасси.

    ``searching`` — порт ещё не найден: пока состояние свежо и с начала поиска
    прошло меньше ``RC_LIDAR_PRIORITY_S``, модули шасси не занимают порты под
    свою проверку. Без этого перебор лидара упирался в «занят» на порту, который
    держало своё же приложение. ``found`` — порт найден и читается (``port``):
    этот порт модули не трогают. ``peer`` — скан берётся у второго процесса:
    собственного перебора нет, приоритет лидару не нужен.

    ``since`` — начало текущей полосы поиска: окно приоритета считается от него,
    поэтому обновление файла каждый круг поиска не продлевает окно до бесконечности.

    ``searcher`` — сеанс поиска внутри процесса (:func:`_new_searcher`): у каждого
    своя запись, поэтому второй менеджер не затирает состояние первого.
    """
    now = time.time()
    if since is None:
        since = now if state == "searching" else 0.0
    path = lidar_state_file()
    with _state_file_lock:
        records = _state_records_from_file(path)
        records[searcher] = {"state": state, "port": port or None,
                             "at": now, "since": float(since)}
        _prune_state_records(records, now)
        _write_state_file(path, {"pid": os.getpid(), "at": now, "records": records})
    _journal_changed()


def clear_lidar_state(searcher: str = "") -> None:
    """Убирает состояние лидара: процесс останавливается и порты больше не держит.

    ``searcher`` — убрать запись только этого сеанса поиска (``stop()`` менеджера),
    не трогая остальных: менеджеров в одном процессе бывает несколько.
    """
    path = lidar_state_file()
    if not searcher:
        try:
            path.unlink()
        except OSError:
            pass
        _journal_changed()
        return
    with _state_file_lock:
        records = _state_records_from_file(path)
        if searcher in records:
            del records[searcher]
            if records:
                _write_state_file(path, {"pid": os.getpid(), "at": time.time(),
                                         "records": records})
            else:
                try:
                    path.unlink()
                except OSError:
                    pass
    _journal_changed()


_journal_lock = threading.Lock()
#: Снимок журнала: {'at': время, 'states': [...], 'modules': {...}}. Журнал читают
#: и запросы к /api/lidar, и потоки чтения портов, поэтому читаем его не чаще
#: ``PORTS_STATE_CACHE_S``; свои записи сбрасывают кэш сразу (см. ``_journal_changed``).
_journal: dict[str, Any] = {"at": 0.0, "states": [], "modules": set()}


def _journal_changed() -> None:
    """Своя запись: журнал перечитываем сразу, а не через кэш."""
    with _journal_lock:
        _journal["at"] = 0.0


def _journal_snapshot() -> dict[str, Any]:
    """Снимок журнала портов: состояния лидара и отметки портов модулей.

    Возвращается общий кэш: наружу из него отдаются только копии (см.
    :func:`_lidar_states` и :func:`chassis_module_ports`).
    """
    now = time.time()
    with _journal_lock:
        if _journal["at"] and now - _journal["at"] < PORTS_STATE_CACHE_S:
            return _journal
        states: list[dict[str, Any]] = []
        for path in sorted(ports_state_dir().glob("lidar-state-*.json")):
            states.extend(_read_lidar_records(path, LIDAR_STATE_TTL_S))
        modules: set[str] = set()
        for path in sorted(ports_state_dir().glob("chassis-ports-*.json")):
            data = _read_state_file(path, MODULE_TAG_TTL_S)
            if data is None:
                continue
            entries = data.get("ports")
            if not isinstance(entries, dict):
                continue
            for entry in entries.values():
                if not isinstance(entry, dict):
                    continue
                try:
                    at = float(entry.get("at") or 0.0)
                except (TypeError, ValueError):
                    continue
                if at and now - at > MODULE_TAG_TTL_S:
                    continue
                for key in ("port", "device"):
                    value = entry.get(key)
                    if isinstance(value, str) and value:
                        modules.add(value)
        _journal["at"] = now
        _journal["states"] = states
        _journal["modules"] = modules
        return _journal


def _lidar_states() -> list[dict[str, Any]]:
    """Состояние всех живых процессов робота с лидаром (пульт 8080 и борт 8090)."""
    return list(_journal_snapshot()["states"])


def lidar_search_priority() -> float:
    """Сколько секунд ещё действует приоритет лидара на порты (0 — не действует).

    Приоритет — не «навсегда»: окно ``RC_LIDAR_PRIORITY_S`` (по умолчанию
    ``LIDAR_PRIORITY_S``) от начала текущего поиска. Лидара на роботе может не
    быть вовсе, и тогда бесконечная уступка портов оставила бы экраны без
    телеметрии модулей. После окна модули проверяют порты как раньше, а лидар
    получает окна между их проверками.
    """
    window = _env_float("RC_LIDAR_PRIORITY_S", LIDAR_PRIORITY_S)
    if window <= 0.0:
        return 0.0
    now = time.time()
    left = 0.0
    for state in _lidar_states():
        if state.get("state") != "searching":
            continue
        try:
            since = float(state.get("since") or 0.0)
        except (TypeError, ValueError):
            continue
        left = max(left, window - (now - since))
    return left if left > 0.0 else 0.0


def lidar_active_port() -> Optional[str]:
    """Порт, который лидар читает прямо сейчас (по состоянию всех процессов)."""
    for state in _lidar_states():
        if state.get("state") != "found":
            continue
        port = state.get("port")
        if isinstance(port, str) and port and os.path.exists(port):
            return port
    return None


def lidar_holds_port(port: str) -> bool:
    """Не трогать этот порт: лидар его ищет (приоритет) или уже читает.

    Источник данных шасси спрашивает это перед своей проверкой порта (см.
    ``SerialSource(yield_to_lidar=…)``). Порт, который уже доказал, что на нём
    модули шасси, не уступается никогда: иначе телеметрия пропадала бы с обоих
    экранов на каждом круге поиска лидара.
    """
    if not port:
        return False
    if module_port_confirmed(port):
        return False
    active = lidar_active_port()
    if active is not None and _same_port(active, port):
        return True
    return lidar_search_priority() > 0.0


_module_ports_lock = threading.Lock()


def module_ports_file(pid: Optional[int] = None) -> Path:
    """Файл этого процесса со списком портов, на которых подтвердились модули."""
    return ports_state_dir() / ("chassis-ports-%d.json" % (os.getpid() if pid is None else pid))


def _own_module_ports() -> dict[str, Any]:
    """Своя запись (вызывать под ``_module_ports_lock``): имя порта → отметка."""
    data = _read_state_file(module_ports_file(), ttl=0.0)
    ports = data.get("ports") if isinstance(data, dict) else None
    if not isinstance(ports, dict):
        return {}
    return {str(key): value for key, value in ports.items() if isinstance(value, dict)}


def tag_module_port(port: str) -> None:
    """Отмечает порт как подтверждённый порт модулей шасси (его видит лидар).

    Без этой отметки перебор лидара каждый круг упирался в порт, который держит
    источник данных шасси, и на экране появлялось «занят другой программой» — хотя
    порт занимало это же приложение. Отметка живёт, пока идут кадры телеметрии
    (:func:`untag_module_port` снимает её, если порт замолчал).
    """
    if not port:
        return
    with _module_ports_lock:
        entries = _own_module_ports()
        entries[_port_key(port)] = {
            "port": port,
            "device": os.path.realpath(port) or port,
            "at": time.time(),
        }
        _write_state_file(
            module_ports_file(),
            {"pid": os.getpid(), "at": time.time(), "ports": entries},
        )
    _journal_changed()


def untag_module_port(port: str) -> None:
    """Снимает отметку: окно проверки кончилось, а кадров телеметрии не было.

    Устройство могли выдернуть или заменить лидаром на том же USB — держать за
    портом чужую отметку нельзя, иначе лидар никогда его не откроет.
    """
    if not port:
        return
    removed = False
    with _module_ports_lock:
        entries = _own_module_ports()
        if entries.pop(_port_key(port), None) is not None:
            removed = True
            if entries:
                _write_state_file(
                    module_ports_file(),
                    {"pid": os.getpid(), "at": time.time(), "ports": entries},
                )
            else:
                try:
                    module_ports_file().unlink()
                except OSError:
                    pass
    if removed:
        _journal_changed()


def chassis_module_ports() -> set[str]:
    """Порты, на которых прямо сейчас подтверждены модули шасси (все процессы).

    Возвращаются и имя, и устройство порта: лидар сравнивает по realpath, потому
    что одно устройство может быть названо и ``/dev/ttyUSB0``, и symlink'ом из
    ``/dev/serial/by-id``.
    """
    return set(_journal_snapshot()["modules"])


def module_port_confirmed(port: str) -> bool:
    """Отмечен ли порт как порт модулей шасси (сравнение по устройству)."""
    if not port:
        return False
    marked = chassis_module_ports()
    if not marked:
        return False
    if port in marked:
        return True
    return any(_same_port(known, port) for known in marked)


def dedup_ports(ports: Iterable[str]) -> list[str]:
    """Список портов без повторов одного устройства (для статуса и журналов)."""
    seen: set[str] = set()
    result: list[str] = []
    for port in sorted({p for p in ports if p}, key=_natural_key):
        real = os.path.realpath(port)
        if real in seen:
            continue
        seen.add(real)
        result.append(port)
    return result


def lidar_reserved_ports() -> set[str]:
    """Порты, которые отданы лидару настройками (их не должны занимать модули шасси).

    Сюда входит порт из ``RC_LIDAR_PORT`` (вместе с realpath — на роботе это
    обычно ``/dev/serial/by-id/…`` → ``/dev/ttyUSB1``) и именованные устройства
    лидара (``/dev/rplidar*``, ``/dev/serial/by-id/*lidar*`` и т. п.). Полный
    перебор USB сюда не входит: иначе ``RC_SLAM_SOURCE=auto`` оставил бы шасси
    вовсе без портов.

    Именно отсутствие этого списка и ломало лидар в веб-интерфейсе: источник
    данных шасси в режиме ``auto`` сам находил все ``/dev/ttyUSB*`` и открывал
    порт лидара, вычитывая из него байты. Скрипт ``scripts/lidar_check.py`` при
    этом работал — он запускается, когда веб-приложения остановлены.
    """
    reserved: set[str] = set()
    for path in discover_ports(
        configured=os.environ.get("RC_LIDAR_PORT"),
        autoprobe=False,
        excluded=excluded_ports(),
    ):
        reserved.add(path)
        try:
            reserved.add(os.path.realpath(path))
        except OSError:
            continue
    return {p for p in reserved if p}


def busy_ports_note(notes: Iterable[str]) -> str:
    """Подсказка про занятые порты перебора (или пустая строка, если таких нет).

    Порт может быть занят своим же приложением: источник данных шасси берёт
    порт на короткую проверку (2 с) и отпускает его, поэтому «занят» — не
    повод искать кабель и не приговор: перебор повторится сам. Постоянно занятый
    порт — уже чужая программа (``rplidar_ros``, Serial Monitor, второй пульт).
    """
    busy = sum(1 for note in notes if note == "занят")
    if not busy:
        return ""
    return ("%d порт(ов) из них заняты другой программой: обычно это источник данных "
            "шасси (RC_SLAM_SOURCE/RC_SLAM_PORTS) — он держит порт на время проверки "
            "и отпускает его через пару секунд, перебор повторится сам" % busy)


def config_conflict_note() -> str:
    """Предупреждение, если порт лидара отдан настройками мастеру/шасси."""
    configured = (os.environ.get("RC_LIDAR_PORT") or "").strip()
    if not configured:
        return ""
    blocked = excluded_ports()
    blocked_real = {os.path.realpath(p) for p in blocked if p}
    conflict = []
    for chunk in configured.replace(";", ",").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        for path in glob.glob(chunk) or [chunk]:
            try:
                real = os.path.realpath(path)
            except OSError:
                real = path
            if path in blocked or real in blocked_real:
                conflict.append(path)
    if not conflict:
        return ""
    return ("порт лидара %s исключён настройками мастера/шасси (RC_SLAM_PORT, "
            "RC_SLAM_PORTS или RC_LIDAR_EXCLUDE) — лидар его не откроет; убери "
            "его оттуда или задай лидару другой порт" % ", ".join(sorted(set(conflict))))


def discover_ports(
    *,
    configured: Optional[str] = None,
    autoprobe: bool = True,
    excluded: Iterable[str] = (),
) -> list[str]:
    """Порты-кандидаты для лидара: сначала заданные вручную, потом symlink'и, потом tty.

    ``configured`` может быть списком через запятую и поддерживает glob
    (``RC_LIDAR_PORT=/dev/ttyUSB1`` или ``/dev/serial/by-id/*lidar*``).

    Порт из настройки — только **первый в очереди**, а не единственный:
    лидар переподключают, и после перезагрузки он оказывается на другом
    ``/dev/ttyUSBx``. Поэтому ``autoprobe=True`` (по умолчанию) добавляет в
    список все USB-порты машины, и один и тот же порт, попавший и по
    symlink'у, и по имени tty, добавляется один раз (сравнение по realpath) —
    иначе перебор тратил бы на него двойное время.
    """
    from .battery import reserved_ports
    blocked = {p for p in excluded if p} | reserved_ports()
    blocked |= {os.path.realpath(p) for p in blocked}
    found: list[str] = []
    seen_real: set[str] = set()

    def add(path: str) -> None:
        if not path or not os.path.exists(path):
            return
        real = os.path.realpath(path)
        if path in blocked or real in blocked:
            return
        if path in found or real in seen_real:
            return
        found.append(path)
        seen_real.add(real)

    patterns: list[str] = []
    for chunk in (configured or "").replace(";", ",").split(","):
        chunk = chunk.strip()
        if chunk:
            patterns.append(chunk)
    for pattern in patterns:
        for path in sorted(glob.glob(pattern), key=_natural_key):
            add(path)

    # Именованные устройства: правило udev (см. README) или постоянные symlink'и.
    for pattern in (
        "/dev/rplidar*",
        "/dev/lidar*",
        "/dev/ldlidar*",
        "/dev/serial/by-id/*lidar*",
        "/dev/serial/by-id/*rplidar*",
        "/dev/serial/by-id/*lds*",
        "/dev/serial/by-id/*ld19*",
        "/dev/serial/by-id/*ld06*",
    ):
        for path in sorted(glob.glob(pattern), key=_natural_key):
            add(path)

    # Остальные USB-порты — по умолчанию (``autoprobe``): перебор обязан
    # пройти их все, даже если RC_LIDAR_PORT задан одним портом. Иначе после
    # переподключения лидара на соседний порт приложение навсегда оставалось
    # без скана («обращается только к одному USB»).
    if autoprobe:
        for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
            for path in sorted(glob.glob(pattern), key=_natural_key):
                add(path)

    return found


def stable_port_path(port: str) -> str:
    """Постоянное имя порта: ``/dev/serial/by-id/…`` вместо ``/dev/ttyUSB1``.

    Номер ``ttyUSBx`` зависит от порядка включения USB-устройств: после
    перезагрузки или переподключения лидар может оказаться на другом номере, и
    закреплённый в ``.env`` порт перестанет работать. Имя из ``/dev/serial/by-id``
    привязано к самому переходнику, поэтому в настройках сохранять нужно его.
    Если такого имени нет, возвращается исходный путь.
    """
    if not port:
        return port
    try:
        real = os.path.realpath(port)
    except OSError:
        return port
    for pattern in ("/dev/serial/by-id/*", "/dev/serial/by-path/*"):
        for link in sorted(glob.glob(pattern), key=_natural_key):
            try:
                if os.path.realpath(link) == real:
                    return link
            except OSError:
                continue
    return port


def usb_diagnostics() -> dict[str, Any]:
    """Read-only USB/serial inventory; never open ports owned by the robot.

    Include excluded ports so a chassis-port configuration mistake is visible.
    Access is checked as the running service user, not as an administrator.
    Report the interpreter and pyserial too: the check script and the web app
    must run in the same virtualenv, otherwise the lidar is "found" by the
    script and never read by the console.
    """
    blocked = excluded_ports()
    blocked_real = {os.path.realpath(p) for p in blocked}
    modules = chassis_module_ports()
    modules_real = {os.path.realpath(p) for p in modules}
    ports = []
    for path in discover_ports(configured=os.environ.get("RC_LIDAR_PORT")):
        ports.append({
            "port": path,
            "device": os.path.realpath(path),
            "accessible": os.access(path, os.R_OK | os.W_OK),
            "excluded": os.path.realpath(path) in blocked_real,
            # Порт, который источник данных шасси подтвердил телеметрией: лидар
            # его не перебирает, и «занят» на нём — не поломка лидара.
            "chassis_module": os.path.realpath(path) in modules_real,
        })
    usable = [p for p in ports if not p["excluded"] and not p["chassis_module"]]
    pyserial = pyserial_version()
    if pyserial is None:
        summary = ("pyserial не установлен в интерпретаторе %s — USB-лидар не "
                   "читается: запусти ./scripts/ensure_deps.sh и перезапусти "
                   "веб-приложения из .venv" % sys.executable)
    elif not ports:
        summary = ("USB/serial-порты не обнаружены: проверь питание, USB-кабель "
                   "с передачей данных и драйвер USB-переходника")
    elif not usable:
        if modules and not blocked:
            summary = ("Все обнаруженные порты подтверждены как модули шасси "
                       "(их читает источник данных шасси): лидар не трогает эти порты.")
        else:
            summary = ("Все обнаруженные порты исключены настройками мастера/шасси: "
                       "проверь RC_SLAM_PORT, RC_SLAM_PORTS и RC_LIDAR_EXCLUDE")
    elif not any(p["accessible"] for p in usable):
        summary = ("Нет прав чтения/записи портов: добавь пользователя службы "
                   "в группу dialout и перезапусти его сеанс/службу")
    else:
        summary = ("USB/serial-порты доступны; это ещё не подтверждает наличие лидара. "
                   "Программа автоматически подбирает протокол, скорость и управление мотором")
    conflict = config_conflict_note()
    if conflict:
        summary = conflict + "; " + summary
    return {
        "ports": ports,
        "summary": summary,
        "pyserial": pyserial,
        "python": sys.executable,
        "config_conflict": conflict or None,
        # Порты, подтверждённые модулями шасси (data/chassis-ports-<pid>.json):
        # ровно то, что лидар пропускает в своём переборе.
        "chassis_ports": dedup_ports(modules),
    }


def startup_summary() -> list[str]:
    """Строки журнала запуска: с чем лидар собирается читать порт.

    На роботе это ответ на вопрос «сохранились ли настройки и перезапущено ли
    приложение»: в журнале пульта (8080) и бортового экрана (8090) видно, каким
    интерпретатором запущен процесс, есть ли в нём pyserial, какие ``RC_LIDAR_*``
    реально прочитаны из ``.env`` и какие порты будут перебираться.
    """
    version = pyserial_version()
    source_mode = normalize_source(os.environ.get("RC_LIDAR_SOURCE") or "auto")
    lines = [
        "интерпретатор: %s · pyserial: %s" % (sys.executable, version or "НЕТ (лидар не читается)"),
        "источник скана: RC_LIDAR_SOURCE=%s (%s)"
        % (source_mode, {"auto": "ROS 2, иначе USB", "ros": "только ROS 2",
                         "serial": "только USB-порт"}[source_mode]),
        ros_env.summary_line(),
        "настройки: RC_LIDAR_PORT=%s · RC_LIDAR_PROTOCOL=%s · RC_LIDAR_BAUD=%s · "
        "RC_LIDAR_MOTOR=%s · RC_LIDAR_AUTOPROBE=%s"
        % (
            os.environ.get("RC_LIDAR_PORT") or "(автопоиск)",
            os.environ.get("RC_LIDAR_PROTOCOL") or "auto",
            os.environ.get("RC_LIDAR_BAUD") or "(автоподбор)",
            os.environ.get("RC_LIDAR_MOTOR") or "auto",
            os.environ.get("RC_LIDAR_AUTOPROBE") or "1",
        ),
    ]
    blocked = sorted(p for p in excluded_ports() if p)
    lines.append("исключены (мастер/шасси): %s" % (", ".join(blocked) if blocked else "нет"))
    ports = discover_ports(configured=os.environ.get("RC_LIDAR_PORT"))
    lines.append("порты-кандидаты: %s" % (", ".join(ports) if ports else "не найдены"))
    conflict = config_conflict_note()
    if conflict:
        lines.append("ВНИМАНИЕ: " + conflict)
    if version is None and source_mode != "ros":
        lines.append(
            "ВНИМАНИЕ: pyserial не установлен в этом интерпретаторе — запусти "
            "./scripts/ensure_deps.sh и поднимай веб-приложения из .venv"
        )
    return lines


def lidar_profiles(protocol: str = "auto", baud: Optional[int] = None,
                   preferred: Optional[Tuple[str, int]] = None) -> list[Tuple[str, int]]:
    """Список (протокол, скорость) для подбора: от частых моделей к редким.

    На 230400 работают сразу два протокола — LD06/LD19 (кадры 0x54) и
    LDS-01/LDS-02 от TurtleBot3 (кадры 0xFA), поэтому при ``auto`` проверяются
    оба: сначала тот, у кого пакет защищён контрольной суммой.

    ``preferred`` — профиль, который проверяется **первым** (последний
    подтверждённый лидар из ``data/lidar-found.json``): остальные профили
    проверяются как обычно, порядок лишь меняется.
    """
    proto = normalize_protocol(protocol)
    profiles = list(LIDAR_PROFILES)
    if proto != "auto":
        only_baud = {"ld06": LD06_BAUD, "lds01": LDS01_BAUD}.get(proto, RPLIDAR_BAUDS[0])
        profiles = [p for p in profiles if p[0] == proto] or [(proto, only_baud)]
    if baud:
        rate = int(baud)
        preferred_rates = [p for p in profiles if p[1] == rate]
        rest = [p for p in profiles if p[1] != rate]
        names = (proto,) if proto != "auto" else ("rplidar", "ld06", "lds01")
        extra = [(name, rate) for name in names]
        profiles = preferred_rates + [p for p in extra if p[1] == rate] + rest

    seen: set[Tuple[str, int]] = set()
    out: list[Tuple[str, int]] = []
    for item in profiles:
        if item not in seen:
            seen.add(item)
            out.append(item)
    if preferred:
        pair = (str(preferred[0]), int(preferred[1]))
        if pair in out:
            out = [pair] + [p for p in out if p != pair]
    return out


def motor_variants(motor: str = "auto") -> Tuple[Tuple[Optional[str], Optional[bool]], ...]:
    """Варианты управления мотором RPLIDAR из настройки RC_LIDAR_MOTOR."""
    mode = (motor or "auto").strip().lower()
    if mode in MOTOR_MODES:
        return MOTOR_MODES[mode]
    log.warning("Лидар: неизвестное RC_LIDAR_MOTOR=%r — беру auto", motor)
    return RPLIDAR_MOTOR_VARIANTS


def motor_variant_name(variant: Tuple[Optional[str], Optional[bool]]) -> str:
    line, level = variant
    if line is None:
        return "none"
    return "%s-%s" % (line, "low" if level is False else "high")


def apply_motor_variant(ser, variant: Tuple[Optional[str], Optional[bool]]) -> None:
    """Выставляет линии USB-переходника так, чтобы мотор RPLIDAR крутился."""
    line, level = variant
    if line is None or level is None:
        return
    try:
        # DTR у части драйверов дёргает устройство при открытии порта — ставим явно.
        if line == "dtr":
            ser.dtr = bool(level)
        else:
            ser.rts = bool(level)
    except Exception as exc:  # noqa: BLE001 — драйвер может не уметь линии
        log.debug("Лидар: не удалось выставить %s=%s: %s", line, level, exc)


def _flush_input(ser) -> None:
    try:
        ser.reset_input_buffer()
    except Exception:  # noqa: BLE001 — у самодельных «портов» метода может не быть
        pass


def rplidar_handshake(ser, variant: Tuple[Optional[str], Optional[bool]], *,
                      timeout: float = 0.6, stop: Optional[Callable[[], bool]] = None) -> bool:
    """Включает мотор, посылает SCAN и ждёт стандартный дескриптор ответа.

    Ответ RPLIDAR на SCAN — ``A5 5A 05 00 00 40 81`` (длина 5, тип «измерение»).
    Ждём именно его: раньше за ответ принимались любые семь байт после
    синхрослова, и чужой поток на этой скорости (например, LDS-01, читаемый не
    на своей скорости) иногда «отвечал» сам — лидар находился там, где его нет,
    а на экран шёл мусор. Нестандартный дескриптор тоже принимаем, но только как
    запасной вариант, если стандартного за отведённое время не было: у клонов он
    бывает свой, а от мусорного скана дальше защищает проверка витка
    (:func:`scan_looks_like_rotation`).
    """
    stop = stop or (lambda: False)
    apply_motor_variant(ser, variant)
    time.sleep(0.05)  # мотору нужно тронуться, а лидару — сбросить буфер
    _flush_input(ser)
    try:
        ser.write(RPLIDAR_CMD_STOP)
        time.sleep(0.02)
        ser.write(RPLIDAR_CMD_SCAN)
    except Exception as exc:  # noqa: BLE001 — порт мог пропасть
        log.debug("Лидар: запись в порт не удалась: %s", exc)
        return False

    deadline = time.monotonic() + max(0.1, float(timeout))
    buf = bytearray()
    loose = False
    while time.monotonic() < deadline and not stop():
        chunk = ser.read(7)
        if not chunk:
            continue
        buf.extend(chunk)
        idx = buf.find(RPLIDAR_ANSWER_SYNC)
        if idx >= 0 and len(buf) - idx >= 7:
            header = bytes(buf[idx: idx + 7])
            if header[2] == RPLIDAR_ANSWER_SIZE and header[6] in RPLIDAR_ANSWER_TYPES:
                return True
            if not loose:
                loose = True
                log.debug("Лидар: нестандартный дескриптор ответа на SCAN: %s", header.hex(" "))
            del buf[idx]  # ищем стандартный ответ дальше, синхрослово отброшено
            continue
        if len(buf) > 64:
            del buf[:-1]
    if loose:
        log.warning("Лидар: ответ на SCAN нестандартный — сужу о лидаре по точкам, а не по дескриптору")
    return loose


def probe_port(
    ser,
    port: str,
    *,
    protocol: str = "auto",
    baud: Optional[int] = None,
    motor: str = "auto",
    timeout: float = LIDAR_PROBE_TIMEOUT_S,
    stop: Optional[Callable[[], bool]] = None,
    publish: Optional[Callable[[list[dict[str, float]]], None]] = None,
    on_found: Optional[Callable[[dict[str, Any]], None]] = None,
    min_points: int = LIDAR_MIN_POINTS,
    attempt_max: Optional[float] = None,
    preferred: Optional[Tuple[str, int]] = None,
    preferred_motor: Optional[str] = None,
) -> Optional[dict[str, Any]]:
    """Подбирает параметры лидара на уже открытом порту.

    Возвращает {\"port\", \"protocol\", \"baud\", \"motor\", \"points\"} или None.
    ``publish(точки)`` вызывается на каждом готовом витке; первый вызов означает,
    что данные реально пошли (``on_found`` срабатывает на нём же). Пока данные не
    пошли, витков не публикуем — иначе «скан» из случайных байт чужого протокола
    попадал бы на экран.

    ``preferred`` (протокол, скорость) и ``preferred_motor`` (линия мотора) —
    параметры прошлого подтверждённого лидара: проверяются первыми, но не
    исключают остальные варианты.
    """
    stop = stop or (lambda: False)
    budget = LIDAR_ATTEMPT_MAX_S if attempt_max is None else float(attempt_max)
    state: dict[str, Any] = {"found": False, "probe": {"port": port}, "noise": 0}

    def on_scan(points: list[dict[str, float]]) -> None:
        if not state["found"]:
            # Первый виток — доказательство, что на порту настоящий лидар. Если
            # скорость или протокол угаданы неверно, точки получаются из чужих
            # байт и разбросаны по всему кругу: такой виток не считаем находкой
            # и не отдаём на экран (иначе «лидар» находился бы на любом мусоре).
            if not scan_looks_like_rotation(points):
                if not state["noise"]:
                    log.debug(
                        "Лидар: %s — виток не похож на настоящий скан (%d точек), "
                        "продолжаю перебор", port, len(points),
                    )
                state["noise"] += 1
                return
            state["found"] = True
            # Сколько точек в одном витке (360 у LDS-01, ~400 у RPLIDAR): это
            # честнее, чем «всего принято за время подбора».
            state["probe"]["points"] = len(points)
            if on_found is not None:
                on_found(dict(state["probe"]))
        if publish is not None:
            publish(points)

    for proto, rate in lidar_profiles(protocol, baud, preferred=preferred):
        if stop() or state["found"]:
            break
        variants = motor_variants(motor) if proto == "rplidar" else ((None, None),)
        if proto == "rplidar" and preferred_motor and len(variants) > 1:
            # Линия мотора с прошлого запуска — первой, остальные как обычно.
            head = [v for v in variants if motor_variant_name(v) == preferred_motor]
            if head:
                variants = tuple(head) + tuple(
                    v for v in variants if motor_variant_name(v) != preferred_motor
                )
        for variant in variants:
            if stop() or state["found"]:
                break
            try:
                ser.baudrate = rate
            except Exception as exc:  # noqa: BLE001
                log.debug("Лидар: %s: не переключить скорость %d: %s", port, rate, exc)
                return None
            _flush_input(ser)
            state["probe"].update(
                {
                    "protocol": proto,
                    "baud": rate,
                    "motor": motor_variant_name(variant),
                }
            )

            # Одна попытка не может длиться бесконечно: чужое устройство на порту
            # (телеметрия модуля, GPS, лидар другой модели) может сыпать данными
            # непрерывно, и перебор застревал бы на нём навсегда. Как только виток
            # найден, читаем поток дальше без ограничения — иначе на каждом круге
            # скана появлялась бы пауза на переоткрытие порта.
            attempt_deadline = time.monotonic() + budget

            def attempt_stop() -> bool:
                if stop():
                    return True
                return not state["found"] and time.monotonic() >= attempt_deadline

            if proto == "rplidar":
                if not rplidar_handshake(
                    ser, variant, timeout=min(0.6, max(0.2, timeout / 2)), stop=stop
                ):
                    break  # нет ответа на SCAN — на этой скорости устройства нет
                got = read_rplidar_stream(
                    ser, on_scan, attempt_stop, timeout=timeout, min_points=min_points
                )
            elif proto == "lds01":
                # LDS-01 сыплет кадры сам, но прошивки до 2017.10 не крутят мотор
                # без команды 'b' — она же полезна после чужой команды 'e'.
                lds01_start_scan(ser)
                got = read_lds01_stream(
                    ser, on_scan, attempt_stop, timeout=timeout, min_points=min_points
                )
            else:
                got = read_ld06_stream(
                    ser, on_scan, attempt_stop, timeout=timeout, min_points=min_points
                )

            if state["found"]:
                state["probe"].setdefault("points", got)
                return state["probe"]
            if proto == "rplidar" and got:
                log.info(
                    "Лидар: %s ответил на SCAN (%d бод), но точек нет "
                    "(мотор: %s) — пробую другую линию мотора",
                    port, rate, motor_variant_name(variant),
                )
    return None


def open_serial(serial_module, port: str, baud: int):
    """Открывает порт лидара с нашими таймаутами (и эксклюзивно, если умеет pyserial).

    ``write_timeout`` ставим после открытия: без него запись команды в порт
    (``SCAN`` у RPLIDAR, ``b`` у LDS-01) ждёт готовности дескриптора бесконечно,
    и вставший передатчик навсегда останавливает поток поиска лидара.

    Для LDS-01 (TurtleBot3 Burger) сразу выставляем DTR/RTS в False: у части
    адаптеров CP2102 высокий уровень на этих линиях держит лидар в сбросе или
    останавливает мотор, и поток не начинается. RPLIDAR позже переберёт свои
    варианты линий сам (см. ``apply_motor_variant``), а для LDS-01/LD06
    безопасный уровень — низкий.
    """
    try:
        ser = serial_module.Serial(port, baud, timeout=SERIAL_READ_TIMEOUT_S, exclusive=True)
    except TypeError:  # pyserial < 3.3 — параметра exclusive ещё нет
        ser = serial_module.Serial(port, baud, timeout=SERIAL_READ_TIMEOUT_S)
    try:
        ser.write_timeout = SERIAL_WRITE_TIMEOUT_S
    except Exception:  # noqa: BLE001 — не pyserial (тесты, обёртки): там блокировок нет
        pass
    # LDS-01/LD06 не любят, когда адаптер держит DTR/RTS в True — лидар может
    # оказаться в сбросе. Ставим безопасные уровни сразу после открытия.
    try:
        ser.dtr = False
    except Exception:
        pass
    try:
        ser.rts = False
    except Exception:
        pass
    return ser


# ============================================================================
# Менеджер лидара
# ============================================================================
class LidarManager:
    """Менеджер лидара: собирает данные из ROS 2, USB/serial или HTTP API."""

    _instance: Optional["LidarManager"] = None

    def __init__(self, port: Optional[str] = None):
        #: Откуда берём скан: auto | ros | serial (RC_LIDAR_SOURCE).
        self.source_mode = normalize_source(os.environ.get("RC_LIDAR_SOURCE") or "auto")
        self.port = port or os.environ.get("RC_LIDAR_PORT") or None
        self.baud = _env_int("RC_LIDAR_BAUD")
        raw_protocol = (os.environ.get("RC_LIDAR_PROTOCOL") or "auto").strip().lower()
        self.protocol = normalize_protocol(raw_protocol)
        if raw_protocol and self.protocol == "auto" and raw_protocol != "auto":
            log.warning(
                "Лидар: неизвестный RC_LIDAR_PROTOCOL=%r — беру auto (%s)",
                raw_protocol, " | ".join(LIDAR_PROTOCOLS),
            )
        raw_motor = (os.environ.get("RC_LIDAR_MOTOR") or "auto").strip().lower()
        self.motor = raw_motor if raw_motor in MOTOR_MODES else "auto"
        self.autoprobe = _env_bool("RC_LIDAR_AUTOPROBE", True)
        self.excluded = excluded_ports()
        self.peer_urls = peer_urls()

        self.lock = threading.Lock()
        self.scan: list[dict[str, float]] = []
        self.scan_at: float = 0.0
        self.source_type: str = "none"
        self.reason: str = "лидар: идёт поиск порта"
        self.hint: str = ""
        self.detected: Optional[dict[str, Any]] = None
        self.last_error: str = ""
        self.candidates: list[str] = []

        self._stop = False
        self._thread: Optional[threading.Thread] = None
        #: Подписка на ROS 2: топик скана (/scan) и состояние супервизора
        #: драйвера (/zapret/lidar/status). Живёт в фоне (см. ros_lidar.py).
        self.ros = RosLidar(self._on_ros_scan)
        #: Обновляется ли скан из ROS 2 (для «геолокации» кнопки «Пересканировать»).
        self._ros_started = False
        self._lock_fd: Optional[int] = None
        self._peer_misses = 0
        self._peer_reason = ""
        self._peer_retry_at = 0.0  # запасная peer-проверка не чаще раза в 2 с
        # С какого порта начинать очередной круг перебора и что нашли в
        # прошлом круге (видно в GET /api/lidar → tried).
        self._rotate = 0
        self._fallback_all = False
        #: Итог последней попытки на порту (для ``tried`` в статусе).
        self._port_note = "нет данных"
        #: Порт, на котором лидар реально отвечал: следующий круг начинаем с
        #: него, а не «со следующего» — иначе после короткого обрыва связи
        #: перебор заново тратил по несколько секунд на каждый мёртвый порт.
        self._sticky: Optional[str] = None
        #: Последний подтверждённый лидар с прошлого запуска
        #: (``data/lidar-found.json``): его порт и профиль проверяются первыми,
        #: пока поиск не нашёл «своё» (см. ``load_found_lidar``).
        self._found_hint = load_found_lidar()
        self._hint_budget = LIDAR_HINT_ROUNDS if self._found_hint else 0
        self.tried: list[dict[str, Any]] = []
        self._logged_ports: tuple[str, ...] = ()
        self._logged_modules: tuple[str, ...] = ()
        self._logged_reason = ""
        self._logged_hint = ""
        #: Начало текущей полосы поиска порта и время последней записи состояния
        #: (``data/lidar-state-<pid>.json``): по нему источник данных шасси
        #: понимает, что порты пока нужны лидару (см. :func:`lidar_holds_port`).
        self._search_since = 0.0
        self._state_written = 0.0
        self._state_written_kind = ""
        #: Порт, с которого сейчас идут витки (для обновления состояния).
        self._active_port: Optional[str] = None
        #: Версия параметров: активный serial-поток прекращает старый перебор,
        #: если соседний процесс сохранил новые настройки в .env.
        self._settings_revision = 0
        #: Сеанс поиска этого менеджера в файле состояния процесса: записи
        #: нескольких менеджеров в одном процессе не затирают друг друга.
        self._searcher = _new_searcher()
        #: Файл .env и его mtime на момент старта: второй процесс робота
        #: (пульт ↔ борт) подхватывает сохранённые из веб-панели настройки по
        #: изменению файла, без перезапуска обоих приложений руками.
        self._env_file = env_file_path()
        self._env_checked = 0.0
        try:
            self._env_mtime = self._env_file.stat().st_mtime
        except OSError:
            self._env_mtime = 0.0

    # --------------------------- жизненный цикл --------------------------- #

    @classmethod
    def get_instance(cls, port: Optional[str] = None) -> "LidarManager":
        if cls._instance is None:
            cls._instance = LidarManager(port)
            cls._instance.start()
        return cls._instance

    def start(self) -> None:
        """Запускает фоновый опрос serial-лидара и ROS 2 (если доступно).

        Состояние «ищу порт» пишется сразу: источник данных шасси поднимается тем
        же приложением и должен успеть уступить порты, а не занять их своей
        проверкой (см. :func:`lidar_holds_port`).
        """
        if self._thread and self._thread.is_alive():
            return
        self._stop = False
        self._publish_state("searching")
        # ROS 2 поднимаем первым: если он есть, драйвер лидара уже держит порт, и
        # USB-перебор не нужен вовсе (см. _ros_owns_lidar).
        self._start_ros()
        self._thread = threading.Thread(target=self._serial_worker, daemon=True)
        self._thread.start()

    def _start_ros(self, force: bool = False) -> bool:
        """Поднимает подписку на ROS 2, если источник это допускает.

        ``RC_LIDAR_SOURCE=serial`` отключает ROS 2 полностью (на стенде без ROS
        и для отладки USB-пути). В ``auto`` (по умолчанию) ROS 2 используется,
        когда он вообще есть в системе: топик ``/scan`` отдаёт драйвер
        производителя, а USB-перебор в это время порт не трогает.
        """
        if self.source_mode == "serial" and not force:
            self._ros_started = False
            return False
        started = self.ros.start()
        self._ros_started = bool(started)
        return self._ros_started

    def _on_ros_scan(self, **fields: Any) -> int:
        """Сообщение ``sensor_msgs/LaserScan`` из ROS 2 → общий скан менеджера."""
        return self.feed_scan_msg(**fields)

    def stop(self) -> None:
        self._stop = True
        self._release_port_lock()
        # Состояние в общем журнале: без этого мёртвый процесс держал бы порты
        # «в приоритете» лидара, пока не истечёт TTL файла.
        self._clear_state()

    # ------------------------- настройки (веб-панель) ---------------------- #

    def current_settings(self) -> dict[str, Any]:
        """Настройки, с которыми этот менеджер ищет лидар прямо сейчас."""
        return {
            "source": self.source_mode,
            "port": self.port or "",
            "protocol": self.protocol,
            "baud": str(self.baud) if self.baud else "",
            "motor": self.motor,
            "autoprobe": bool(self.autoprobe),
        }

    def _apply_settings(self, settings: dict[str, Any]) -> None:
        """Применяет проверенные настройки и сбрасывает поиск на новый круг."""
        self.source_mode = normalize_source(settings.get("source") or "auto")
        self.port = settings["port"] or None
        self.protocol = settings["protocol"]
        self.baud = int(settings["baud"]) if settings["baud"] else None
        self.motor = settings["motor"]
        self.autoprobe = bool(settings["autoprobe"])
        self.excluded = excluded_ports()
        self.peer_urls = peer_urls()
        # Окружение тоже обновляем: перебор портов, диагностика и соседние
        # модули (slam.py) читают RC_LIDAR_* из него, а не из менеджера.
        os.environ["RC_LIDAR_PORT"] = settings["port"]
        os.environ["RC_LIDAR_PROTOCOL"] = settings["protocol"]
        os.environ["RC_LIDAR_BAUD"] = settings["baud"]
        os.environ["RC_LIDAR_MOTOR"] = settings["motor"]
        os.environ["RC_LIDAR_AUTOPROBE"] = "1" if settings["autoprobe"] else "0"
        os.environ["RC_LIDAR_SOURCE"] = self.source_mode
        with self.lock:
            self.scan = []
            self.scan_at = 0.0
            self.source_type = "none"
        self.detected = None
        self.last_error = ""
        self.tried = []
        self.candidates = []
        self._sticky = None
        self._rotate = 0
        self._fallback_all = False
        self._active_port = None
        self._peer_misses = 0
        self._settings_revision += 1
        self._set_status("лидар: идёт поиск порта (настройки применены)")

    def _stop_worker(self, timeout: float = 5.0) -> None:
        """Останавливает поток перебора и ждёт, пока он отпустит порт."""
        thread = self._thread
        self._stop = True
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=timeout)
        self._release_port_lock()
        self._clear_state()

    def reconfigure(self, settings: dict[str, Any]) -> dict[str, Any]:
        """Новые настройки без перезапуска приложения: поиск начинается заново.

        Так веб-панель применяет порт/протокол/скорость/линию мотора сразу:
        старый поток останавливается (он отпускает USB-порт), параметры
        меняются, и перебор стартует уже с ними.
        """
        normalized = normalize_lidar_settings(settings)
        was_running = bool(self._thread and self._thread.is_alive())
        if was_running:
            self._stop_worker()
        self._apply_settings(normalized)
        if was_running:
            self.start()
        log.info(
            "Лидар: настройки применены: порт=%s протокол=%s скорость=%s мотор=%s автопоиск=%s",
            normalized["port"] or "(автопоиск)", normalized["protocol"],
            normalized["baud"] or "(автоподбор)", normalized["motor"],
            "1" if normalized["autoprobe"] else "0",
        )
        return self.current_settings()

    def restart_search(self) -> dict[str, Any]:
        """Пересканировать лидар с текущими настройками (кнопка в панели).

        При источнике ``ros`` перезапускать нужно драйвер, а не USB-перебор
        внутри приложения: запрос уходит супервизору (``~/rescan``). Дальше всё
        равно применяем настройки — это заодно перезапускает и USB-перебор для
        ``serial``/``auto`` на стенде без ROS.
        """
        ros_note = ""
        if self._ros_owns_lidar():
            ros_note = self.request_ros_rescan()
        result = self.reconfigure(self.current_settings())
        if ros_note:
            result = dict(result)
            result["ros_rescan"] = ros_note
        return result

    def _sync_settings_from_env_file(self) -> bool:
        """Подхватывает изменения ``.env`` даже во время чтения активного скана.

        Панель настроек может работать во втором процессе. Обычно этот метод
        вызывается на границе круга поиска; активный поток чтения вызывает его
        также из stop-callback раз в секунду. ``True`` означает, что настройки
        изменились и текущую попытку на старом порту нужно прервать.
        """
        now = time.time()
        if now - self._env_checked < 1.0:
            return False
        self._env_checked = now
        try:
            mtime = self._env_file.stat().st_mtime
        except OSError:
            return False
        if mtime == self._env_mtime:
            return False
        self._env_mtime = mtime
        values = read_env_file(self._env_file)
        if not any(key in values for key in LIDAR_ENV_KEYS):
            return False
        try:
            settings = normalize_lidar_settings({
                "source": values.get("RC_LIDAR_SOURCE", "auto"),
                "port": values.get("RC_LIDAR_PORT", ""),
                "protocol": values.get("RC_LIDAR_PROTOCOL", "auto"),
                "baud": values.get("RC_LIDAR_BAUD", ""),
                "motor": values.get("RC_LIDAR_MOTOR", "auto"),
                "autoprobe": values.get("RC_LIDAR_AUTOPROBE", "1"),
            })
        except ValueError as exc:
            log.warning("Лидар: настройки в .env не применены: %s", exc)
            return False
        if settings != self.current_settings():
            log.info("Лидар: настройки изменились в .env — применяю без перезапуска")
            self._apply_settings(settings)
            return True
        return False

    # ----------------------------- приём данных ---------------------------- #

    def feed_points(self, points: list[dict[str, float]], source_label: str = "api") -> int:
        """Принимает готовый список точек [{'a': ..., 'r': ...}]."""
        valid: list[dict[str, float]] = []
        for p in points:
            if not isinstance(p, dict):
                continue
            a = p.get("a")
            r = p.get("r")
            if a is None or r is None:
                continue
            try:
                fa = float(a)
                fr = float(r)
                if math.isfinite(fa) and math.isfinite(fr) and fr > 0.0:
                    valid.append({"a": round(normalize_angle(fa), 4), "r": round(fr, 3)})
            except (ValueError, TypeError):
                continue

        with self.lock:
            self.scan = valid
            self.scan_at = time.time()
            self.source_type = source_label
        return len(valid)

    def feed_scan_msg(
        self,
        ranges: list[float],
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float = 0.05,
        range_max: float = 12.0,
        source_label: str = "ros",
    ) -> int:
        """Принимает параметры LaserScan и сохраняет точки."""
        pts = scan_from_ranges(ranges, angle_min, angle_max, angle_increment, range_min, range_max)
        with self.lock:
            self.scan = pts
            self.scan_at = time.time()
            self.source_type = source_label
        return len(pts)

    def get_scan(
        self, max_age: float = LIDAR_MAX_AGE_S, allow_demo: bool = False
    ) -> Optional[list[dict[str, float]]]:
        """Возвращает свежий скан или None (если устарел)."""
        now = time.time()
        with self.lock:
            if self.scan and (now - self.scan_at) < max_age:
                return list(self.scan)
        if allow_demo:
            return generate_demo_scan()
        return None

    def get_status(self) -> dict[str, Any]:
        """Статус работы лидара для API /api/lidar и подписи на экране."""
        now = time.time()
        with self.lock:
            scan = list(self.scan)
            scan_at = self.scan_at
            src = self.source_type

        is_fresh = bool(scan and (now - scan_at) < LIDAR_MAX_AGE_S)
        near = min((p["r"] for p in scan), default=None) if scan else None

        if is_fresh:
            reason = "скан идёт: %s" % src
        elif scan:
            reason = "скан устарел (%.1f с без точек)" % (now - scan_at)
        else:
            reason = self.reason

        return {
            "ok": True,
            "connected": is_fresh,
            "source": src if is_fresh else "no_signal",
            # Откуда настройки велят брать скан (auto | ros | serial) и что
            # реально происходит в ROS 2: топик, публикаторы, супервизор
            # драйвера (пакет zapret_lidar), установленные пакеты драйверов.
            "source_mode": self.source_mode,
            "ros": self._ros_status_block(),
            # Демо-скан (стенд без железа) никогда не выдаётся за реальный:
            # флаг ставит слой API, а соседний процесс по нему отказывается
            # принимать выдуманные точки за скан с USB-лидара.
            "demo": False,
            "points_count": len(scan) if is_fresh else 0,
            "scan_at": scan_at,
            "age_s": round(now - scan_at, 2) if scan_at > 0 else None,
            "nearest_m": near if is_fresh else None,
            "configured_port": self.port,
            "settings": self.current_settings(),
            # В каком интерпретаторе работает веб-приложение и есть ли в нём
            # pyserial: проверка лидара и пульт обязаны запускаться из одной .venv.
            "python": sys.executable,
            "pyserial": pyserial_version(),
            "usb_diagnostics": usb_diagnostics() if not is_fresh else None,
            "reason": reason,
            "hint": self.hint,
            "detected": dict(self.detected) if self.detected else None,
            # Лидар, подтверждённый на прошлом запуске (data/lidar-found.json):
            # его порт и профиль проверяются первыми.
            "remembered": dict(self._found_hint) if self._found_hint else None,
            "candidates": list(self.candidates),
            # Порты, подтверждённые источником данных шасси: лидар их не трогает.
            # Видно прямо в статусе — иначе «занят» на порту модулей выглядит как
            # сломанный лидар, хотя порт держит это же приложение.
            "chassis_ports": dedup_ports(chassis_module_ports()),
            # Сколько секунд ещё модули шасси уступают порты лидару (0 — не уступают).
            "priority_s": round(lidar_search_priority(), 1),
            # Результат последнего круга перебора: видно, что проверены все
            # порты, а не один (``ok`` — на нём пошли данные).
            "tried": [dict(item) for item in self.tried],
            "last_error": self.last_error or None,
        }

    # --------------------------- поиск и чтение --------------------------- #

    def candidate_ports(self, *, wide: bool = True,
                        module_ports: Iterable[str] = ()) -> list[str]:
        """Порты для очередного круга перебора, начиная со следующего за прошлым.

        Перебор идёт по кругу: если лидар стоял на одном порту, а потом его
        переподключили (или порт заняла другая программа), круг начинается не с
        него, а со следующего кандидата — «залипания» на одном USB нет.
        ``wide=False`` (``RC_LIDAR_AUTOPROBE=0``) — узкий список: только порт из
        ``RC_LIDAR_PORT`` и именованные symlink'и; полный перебор в этом случае
        делает :meth:`_serial_worker` на следующем круге.

        ``module_ports`` — порты, на которых источник данных шасси подтвердил
        модули телеметрией (см. :func:`chassis_module_ports`). Он держит их
        открытыми, поэтому перебирать их бессмысленно: без этого перебор каждый
        круг упирался в «занят» на порту модулей и тянул на него секунды.

        Порт, на котором лидар уже отвечал (``self._sticky``), ставится первым:
        после короткого обрыва связи скан возвращается сразу, а не после обхода
        всех мёртвых портов.
        """
        excluded = set(self.excluded) | {p for p in module_ports if p}
        ports = discover_ports(
            configured=self.port, autoprobe=wide, excluded=excluded
        )
        if ports:
            known = self._sticky_port(ports)
            if known is not None:
                ports = [known] + [p for p in ports if p != known]
                self._rotate = 0
                if not self._sticky and self._hint_budget > 0:
                    # Поставлен порт из подсказки «лидар был тут»: кругов с ним
                    # всего LIDAR_HINT_ROUNDS — дальше работает перебор по кругу.
                    self._hint_budget -= 1
            else:
                step = self._rotate % len(ports)
                if step:
                    ports = ports[step:] + ports[:step]
                self._rotate = step + 1
        self.candidates = ports
        return ports

    def _sticky_port(self, ports: Sequence[str]) -> Optional[str]:
        """Порт из списка, совпадающий с уже найденным лидаром (по устройству)."""
        wanted_path = self._sticky
        if not wanted_path and self._hint_budget > 0:
            wanted_path = (self._found_hint or {}).get("port") or None
        if not wanted_path:
            return None
        try:
            wanted = os.path.realpath(wanted_path)
        except OSError:
            wanted = wanted_path
        for path in ports:
            if path == wanted_path:
                return path
            try:
                if os.path.realpath(path) == wanted:
                    return path
            except OSError:
                continue
        return None

    def _preferred_profile(self) -> Optional[Tuple[str, int]]:
        """Профиль (протокол, скорость) прошлого подтверждённого лидара.

        Используется только при автоподборе: закреплённые оператором
        ``RC_LIDAR_PROTOCOL``/``RC_LIDAR_BAUD`` важнее любой подсказки.
        """
        if self.protocol != "auto":
            return None
        hint = self._found_hint or {}
        proto = str(hint.get("protocol") or "")
        try:
            baud = int(hint.get("baud")) if hint.get("baud") else 0
        except (TypeError, ValueError):
            baud = 0
        if proto and baud:
            return (proto, baud)
        return None

    def _preferred_motor(self) -> Optional[str]:
        """Линия мотора прошлого подтверждённого лидара (только при ``auto``)."""
        if self.motor != "auto":
            return None
        return str((self._found_hint or {}).get("motor") or "") or None

    def _excluded_note(self) -> str:
        """Подсказка про порты, которые лидару запрещено трогать (мастер/шасси)."""
        if not self.excluded:
            return ""
        ports = sorted(p for p in self.excluded if p)
        return ("исключены порты мастера/модулей шасси: %s — если лидар подключён "
                "туда, убери порт из RC_SLAM_PORT/RC_SLAM_PORTS или из RC_LIDAR_EXCLUDE"
                % ", ".join(ports))

    def _set_status(self, reason: str, hint: str = "", *, warn: bool = False,
                    quiet: bool = False) -> None:
        """Меняет причину «нет скана»; в журнал пишет только при изменении.

        ``quiet`` — для частых сообщений о ходе перебора портов: на экране они
        видны (это и есть ответ «что происходит»), а журнал не засоряется.
        Итог круга перебора в журнал повторно не пишется, даже если между
        кругами статус успел пробежать по портам (``quiet``-сообщения).
        """
        if reason == self.reason and hint == self.hint:
            return
        self.reason = reason
        self.hint = hint
        if quiet:
            return
        if reason == self._logged_reason and hint == self._logged_hint:
            return
        self._logged_reason = reason
        self._logged_hint = hint
        text = reason + ((" (" + hint + ")") if hint else "")
        if warn:
            log.warning("Лидар: %s", text)
        else:
            log.info("Лидар: %s", text)

    def _store_scan(self, points: list[dict[str, float]], source_label: str) -> None:
        with self.lock:
            self.scan = list(points)
            self.scan_at = time.time()
            self.source_type = source_label
        if self._active_port:
            # Порт читается, витки идут — обновляем состояние (не чаще throttle в
            # ``_publish_state``), чтобы источник данных шасси не трогал этот порт
            # и видел, что лидар на связи: одна запись при находке протухла бы,
            # если лидар читает поток непрерывно и новых находок не сообщает.
            self._publish_state("found", self._active_port)

    def _clear_state(self) -> None:
        """Убирает состояние из общего журнала: порты лидару больше не нужны."""
        clear_lidar_state(self._searcher)
        self._state_written = 0.0
        self._state_written_kind = ""
        # Полоса поиска прервана: следующая начнёт окно приоритета заново.
        self._search_since = 0.0

    def _publish_state(self, state: str, port: Optional[str] = None) -> None:
        """Пишет в общий журнал, что менеджер делает с портами.

        Состояние читает источник данных шасси (:func:`lidar_holds_port`): пока
        идёт поиск (``searching``), порты уступаются лидару — окно
        ``RC_LIDAR_PRIORITY_S`` (20 с) от начала поиска. ``found`` с именем порта
        — этот порт модули не трогают вовсе. ``peer`` — скан берётся у второго
        процесса робота: собственного перебора нет, уступать нечего.

        Файл обновляется не чаще, чем раз в половину TTL: он читается и в каждом
        запросе к ``/api/lidar``, а круги поиска идут каждые 2 с.
        """
        now = time.time()
        throttle = max(1.0, LIDAR_STATE_TTL_S / 2.0)
        changed = state != self._state_written_kind
        if state == "searching":
            if not self._search_since:
                # Полоса поиска началась: окно приоритета считается от неё, а не
                # от каждой записи файла — иначе оно продлевалось бы бесконечно.
                self._search_since = now
            if not changed and (now - self._state_written) < throttle:
                return
            publish_lidar_state(state, since=self._search_since, searcher=self._searcher)
        else:
            self._search_since = 0.0
            if not changed and (now - self._state_written) < throttle:
                return
            publish_lidar_state(state, port, searcher=self._searcher)
        self._state_written = now
        self._state_written_kind = state

    def _ros_owns_lidar(self) -> bool:
        """Лидар отдаёт ROS 2 — USB-порт трогать нельзя.

        Драйвер лидара (пакет ROS 2) уже держит порт: свой перебор в этом случае
        отбирал бы у него устройство и получал «порт занят другой программой» —
        именно так скана не было ни на пульте, ни на борту, хотя ``ros2 topic
        echo /scan`` в терминале работал.

        ``RC_LIDAR_SOURCE=serial`` — единственный случай, когда ROS 2 не мешает:
        тогда скан читает само приложение (стенд без ROS, отладка USB-пути).
        """
        if self.source_mode == "serial":
            return False
        if self.source_mode == "ros":
            # Источник задан явно: работаем с ROS 2 и говорим, чего не хватает,
            # а не уходим молча на USB (иначе оператор не поймёт, почему скан
            # идёт не так, как он выбрал).
            return True
        return bool(self._ros_started or self.ros.available)  # auto

    def _serial_worker(self) -> None:
        """Поток поиска порта лидара и чтения скана."""
        try:
            import serial  # pyserial
        except ImportError:
            serial = None  # type: ignore[assignment]
            if not self._ros_owns_lidar():
                # Порт искать нечем: состояние «ищу» снимаем, иначе модули шасси
                # зря уступали бы порты лидару, которого в этом интерпретаторе нет.
                self._clear_state()
                self._set_status(
                    "нет библиотеки pyserial — USB-лидар не читается",
                    hint="поставь пакет: .venv/bin/pip install pyserial (или ./scripts/ensure_deps.sh)",
                    warn=True,
                )
                return

        while not self._stop:
            # Настройки могли сохранить из веб-панели в соседнем процессе:
            # применяем их с нового круга или прекращаем текущую попытку чтения.
            self._sync_settings_from_env_file()
            settings_revision = self._settings_revision
            if self._ros_owns_lidar():
                # Скан даёт ROS 2: своих портов лидар не занимает, уступать нечего.
                # Но состояние ROS 2 показываем то же, что и на экране: «ROS 2
                # есть, но драйвер не запущен» — это не «лидар не найден», и
                # подсказка должна быть про драйвер, а не про кабель.
                self._clear_state()
                if self.ros.fresh():
                    self._set_status(self.ros.summary())
                else:
                    self._set_status(self.ros.summary(), hint=self.ros.hint(), warn=True)
                time.sleep(LIDAR_RESCAN_S)
                continue

            if serial is None and not self._ros_owns_lidar():
                # Источник переключили на USB, а pyserial в этом интерпретаторе
                # нет: говорим прямо, вместо того чтобы искать порт «вслепую».
                self._clear_state()
                self._set_status(
                    "USB-лидар не читается: нет библиотеки pyserial",
                    hint="поставь пакет: .venv/bin/pip install pyserial "
                         "(или ./scripts/ensure_deps.sh) либо верни RC_LIDAR_SOURCE=ros",
                    warn=True,
                )
                time.sleep(LIDAR_RESCAN_S)
                continue

            # Пока порт не найден — сообщаем об этом источник данных шасси: он
            # уступает порты под нашу проверку (окно RC_LIDAR_PRIORITY_S). Здесь
            # же видно, какие порты он подтвердил как модули: их не перебираем.
            module_ports = dedup_ports(chassis_module_ports())
            wide = self.autoprobe or self._fallback_all
            ports = self.candidate_ports(wide=wide, module_ports=module_ports)
            if tuple(sorted(ports)) != self._logged_ports and self._logged_ports:
                # Набор портов изменился: адаптер вынули или воткнули. Лидару
                # снова даётся окно приоритета — иначе горячее подключение
                # лидара ждало бы перезапуска: окно уже истекло, а модули
                # занимают порты своими проверками.
                self._search_since = 0.0
            self._publish_state("searching")
            if not ports:
                # Своих кандидатов нет, но скан может уже идти у второго
                # процесса (веб-пульт держит порт, ROS или HTTP-фид): без этой
                # проверки борт показывал «порт не найден» при живом скане
                # по соседству только потому, что его список портов пуст.
                if self._peer_cycle():
                    self._publish_state("peer")
                    continue
                hint = ("подключи лидар или задай RC_LIDAR_PORT (список портов: "
                        "ls /dev/ttyUSB* /dev/serial/by-id/; проверка: scripts/lidar_check.py)")
                if module_ports:
                    hint = (hint + "; все видимые порты заняты модулями шасси (%s) — "
                            "лидар, похоже, не подключён" % ", ".join(module_ports))
                note = self._excluded_note()
                if note:
                    hint = hint + "; " + note
                self._set_status("порт лидара не найден", hint=hint)
                time.sleep(LIDAR_RESCAN_S)
                continue

            # Перебор всех портов: у лидара нет постоянного имени, поэтому
            # каждая попытка явно попадает и в статус, и в журнал — иначе
            # кажется, что приложение «обращается только к одному USB».
            # Список портов печатаем, только когда он изменился: ротация
            # каждые 2 с одного и того же набора портов — это не новость.
            if (tuple(sorted(ports)) != self._logged_ports
                    or tuple(module_ports) != self._logged_modules):
                self._logged_ports = tuple(sorted(ports))
                self._logged_modules = tuple(module_ports)
                suffix = ""
                if module_ports:
                    suffix = (". Пропускаю порты модулей шасси: %s — их подтвердил "
                              "телеметрией источник данных шасси"
                              % ", ".join(module_ports))
                log.info(
                    "Лидар: перебираю %d порт(ов)%s: %s%s",
                    len(ports),
                    "" if wide else " (узкий список: RC_LIDAR_AUTOPROBE=0)",
                    ", ".join(ports),
                    suffix,
                )
            # Пропущенные порты модулей — в ``tried``: по GET /api/lidar должно
            # быть видно, почему порт не проверялся, а не только «занят».
            self.tried = [{"port": port, "ok": False, "note": "модуль шасси"}
                          for port in module_ports]
            progressed = False
            for index, port in enumerate(ports, start=1):
                if self._stop:
                    return
                if self._settings_revision != settings_revision:
                    break
                self._set_status(
                    "перебираю порты: %d из %d — %s" % (index, len(ports), os.path.basename(port)),
                    quiet=True,
                )
                # Запись «ищу порт» обновляем на каждом порту: круг по чужому
                # устройству длится десятками секунд (у каждой модели своя
                # скорость), а источник данных шасси смотрит на её свежесть.
                self._publish_state("searching")
                if not self._acquire_port_lock(port):
                    # Порт читает второй процесс робота: пульт (8080) и бортовой
                    # экран (8090) поднимают этот менеджер одновременно, а порт
                    # открывается только один раз.
                    if self._peer_cycle():
                        progressed = True
                        self._fallback_all = False
                        # Порт соседа запоминаем как «свой найденный»: без
                        # этого следующий круг начинался ротацией со СЛЕДУЮЩЕГО
                        # порта, менеджер уходил подбирать параметры чужих
                        # USB-устройств на секунды…минуты (у каждой модели
                        # своя скорость и линии мотора), а скан от соседа за
                        # это время протухал (свежесть — 1 с). Итог на борту:
                        # веб показывал лидар, бортовой экран — «нет скана»,
                        # хотя оба процесса исправны. Теперь круг всегда
                        # начинается с порта соседа и сразу обновляет peer-скан.
                        self._sticky = port
                        # Скан идёт от соседа: свой перебор стоит, приоритет на
                        # порты больше не нужен — модули могут читать свои.
                        self._publish_state("peer")
                        self.tried.append({"port": port, "ok": True,
                                           "note": "скан берётся у второго процесса робота"})
                        break
                    peer_reason = ("; сосед сообщает: " + self._peer_reason) if self._peer_reason else ""
                    if module_port_confirmed(port):
                        # Порт модулей мог появиться уже после сборки списка:
                        # не выдаём «занят чужой программой» за сломанный лидар.
                        self.tried.append({"port": port, "ok": False, "note": "модуль шасси"})
                        continue
                    self._set_status(
                        "порт %s занят другой программой" % port,
                        hint="обычно это источник данных шасси (RC_SLAM_SOURCE/RC_SLAM_PORTS): "
                             "он берёт порт на время проверки и отпускает его через пару "
                             "секунд — перебор сам повторит попытку; если порт занят "
                             "постоянно, это rplidar_ros, Serial Monitor или второй процесс "
                             "пульта (у него скан берётся по HTTP)" + peer_reason,
                        quiet=True,
                    )
                    self.tried.append({"port": port, "ok": False, "note": "занят"})
                    continue
                try:
                    ok = self._read_port(serial, port)
                    note = self._port_note
                finally:
                    self._release_port_lock()
                if self._settings_revision != settings_revision:
                    break
                self.tried.append({"port": port, "ok": ok, "note": note})
                if ok:
                    progressed = True
                    self._fallback_all = False
                    # Порт, на котором реально пошли точки, — первый в следующем
                    # круге: после обрыва связи лидар находится сразу.
                    self._sticky = port
                    # И порт, который модули шасси больше не трогают (см.
                    # lidar_holds_port): они знают, что тут читает лидар.
                    self._publish_state("found", port)
                    break

            # Изменение .env могло прийти, пока probe_port читал непрерывный
            # поток. Старый круг больше не нужен — сразу ищем по новым значениям.
            if self._settings_revision != settings_revision:
                continue

            if not progressed:
                # Сосед мог отдать скан и без занятого порта (веб-пульт
                # получает точки по ROS/HTTP): пробуем peer ещё раз, прежде
                # чем объявлять «скана нет» и уходить в паузу перебора. Не
                # чаще раза в 2 с: peer-проверка ходит по HTTP и не должна
                # тормозить сам перебор портов.
                if time.time() >= self._peer_retry_at:
                    self._peer_retry_at = time.time() + 2.0
                    if self._peer_cycle():
                        self._publish_state("peer")
                        continue
                # Ни один порт не ответил — говорим об этом прямо:
                # видно, что перебор был по всем портам, а не по одному.
                self._active_port = None          # витков нет: состояние не обновляем
                notes = [item["note"] for item in self.tried]
                busy = notes.count("занят")
                modules = notes.count("модуль шасси")
                if notes and all(note in ("занят", "модуль шасси") for note in notes):
                    # Все кандидаты кем-то заняты: либо это порты модулей шасси
                    # (их держит источник данных шасси), либо чужая программа.
                    # Советы про кабель и скорость здесь бесполезны.
                    total = len(notes)
                    if busy and modules:
                        reason = ("лидар не найден: из %d порт(ов) %d заняты другой программой, "
                                  "%d — модули шасси" % (total, busy, modules))
                    elif busy:
                        reason = "лидар не найден: все %d порт(ов) заняты другой программой" % total
                    else:
                        reason = "лидар не найден: все %d порт(ов) — модули шасси" % total
                    hint = ("обычно их держит источник данных шасси "
                            "(RC_SLAM_SOURCE/RC_SLAM_PORTS): он берёт порт на время "
                            "проверки и отпускает его через пару секунд — перебор "
                            "повторится сам; если порты заняты постоянно, это "
                            "rplidar_ros, Serial Monitor или второй процесс пульта"
                            ) if busy else ""
                else:
                    # Порты в сообщении — по имени, а не в порядке обхода:
                    # иначе из-за ротации причина менялась бы каждый круг и
                    # один и тот же итог сыпался бы в журнал каждые 2 с.
                    items = sorted(self.tried, key=lambda item: os.path.basename(item["port"]))
                    checked = ", ".join(
                        "%s — %s" % (os.path.basename(item["port"]), item["note"])
                        for item in items[:5]
                    )
                    if len(items) > 5:
                        checked += " …"
                    reason = "лидар не ответил ни на одном из %d портов (%s)" % (
                        len(items), checked)
                    # Часть портов могла быть занята своим же источником данных
                    # шасси: тогда «проверь кабель» уводит в сторону, и это видно
                    # в подсказке, а не только в списке «занят» в причине.
                    busy_note = busy_ports_note(notes)
                    hint = (busy_note + "; " if busy_note else "") + (
                        "проверь питание и кабель лидара; скорость зависит от модели: "
                        "115200 (A1/A2M8), 256000 (A2M7/A2M12/A3/S1), 460800 (C1), "
                        "230400 (LD06/LD19 и LDS-01/LDS-02 от TurtleBot3); "
                        "полный разбор — .venv/bin/python scripts/lidar_check.py")
                if modules:
                    # Часть портов — модули шасси: их держит своё же приложение,
                    # поэтому «проверь кабель» уводит в сторону. Список таких
                    # портов лежит в статусе (``chassis_ports``).
                    module_note = ("порты модулей шасси (GET /api/lidar → chassis_ports) "
                                   "лидар не трогает: если лидар подключён именно туда, "
                                   "закрепи его порт (RC_LIDAR_PORT) или убери порт из "
                                   "RC_SLAM_PORTS/RC_LIDAR_EXCLUDE")
                    hint = hint + "; " + module_note
                note = self._excluded_note()
                if note:
                    hint = hint + "; " + note
                self._set_status(reason, hint=hint, warn=True)
                if not self.autoprobe and not self._fallback_all:
                    # RC_LIDAR_AUTOPROBE=0 — узкий список. Он только первым, а
                    # не единственным: со следующего круга перебираем все USB.
                    self._fallback_all = True
                    log.info("Лидар: на закреплённых портах данных нет — "
                             "следующий круг перебирает все USB-порты")
                time.sleep(LIDAR_RESCAN_S)

    def _read_port(self, serial, port: str) -> bool:
        """Подбирает параметры и читает скан. True — данные шли (до обрыва).

        Короткий итог попытки пишется в ``self._port_note`` и попадает в
        ``GET /api/lidar`` → ``tried``: на экране различимы «нет данных»,
        «занят» и «нет доступа», а не одно «нет данных» на все случаи.
        """
        self._port_note = "нет данных"
        self._set_status("подбираю параметры порта %s…" % port, quiet=True)
        try:
            ser = open_serial(serial, port, RPLIDAR_BAUDS[0])
        except Exception as exc:  # noqa: BLE001 — порт занят/пропал/нет прав
            self.last_error = "%s: %s" % (port, exc)
            text = str(exc).lower()
            hint = ""
            if isinstance(exc, PermissionError) or "permission" in text:
                hint = "добавь пользователя в группу dialout: sudo usermod -aG dialout $USER (и перезайди)"
                self._port_note = "нет доступа"
            elif "busy" in text or "resource" in text or "in use" in text:
                # Порт уже открыт кем-то другим: источник данных шасси
                # (RC_SLAM_SOURCE=auto/serial), второй процесс пульта,
                # rplidar_ros или Serial Monitor. Это не «лидар не отвечает» —
                # причина другая, и на экране она должна быть другой. Источник
                # шасси держит порт только на время проверки (2 с), так что
                # следующий круг перебора обычно застаёт порт свободным.
                hint = ("порт открыт другой программой: чаще всего это источник данных "
                        "шасси (RC_SLAM_SOURCE/RC_SLAM_PORTS) — он отпускает порт через "
                        "пару секунд, и перебор повторяется сам; если порт занят "
                        "постоянно — второй процесс пульта, rplidar_ros или Serial Monitor")
                self._port_note = "занят"
            elif getattr(serial, "SerialException", None) and isinstance(exc, serial.SerialException):
                hint = ("порт занят другой программой: источник данных шасси "
                        "(RC_SLAM_SOURCE/RC_SLAM_PORTS) или второй процесс робота — "
                        "перебор повторит попытку сам")
                self._port_note = "не открылся"
            else:
                self._port_note = "не открылся"
            self._set_status("порт %s не открылся" % port, hint=hint, quiet=True)
            return False

        settings_revision = self._settings_revision

        def stop_probe() -> bool:
            if self._stop:
                return True
            self._sync_settings_from_env_file()
            return self._settings_revision != settings_revision

        def publish_if_current(points: list[dict[str, float]]) -> None:
            if self._settings_revision == settings_revision:
                self._store_scan(points, "serial:%s" % os.path.basename(port))

        def mark_found_if_current(probe: dict[str, Any]) -> None:
            if self._settings_revision == settings_revision:
                self._mark_found(probe)

        try:
            found = probe_port(
                ser,
                port,
                protocol=self.protocol,
                baud=self.baud,
                motor=self.motor,
                timeout=LIDAR_PROBE_TIMEOUT_S,
                stop=stop_probe,
                publish=publish_if_current,
                on_found=mark_found_if_current,
                preferred=self._preferred_profile(),
                preferred_motor=self._preferred_motor(),
            )
            if self._settings_revision != settings_revision:
                # Поток мог успеть подтвердить старый лидар до выхода по stop.
                # Не возвращаем статус «подключён» для уже устаревших настроек.
                self._port_note = "настройки изменились"
                return False
        except Exception as exc:  # noqa: BLE001 — кабель выдернули на ходу
            self.last_error = "%s: %s" % (port, exc)
            self._port_note = "обрыв"
            self._set_status("обрыв связи с портом %s" % port, hint=str(exc), quiet=True)
            return False
        finally:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

        if not found:
            self.detected = None
            self._set_status(
                "лидар на %s не отвечает" % port,
                hint="проверь скорость (A3/S1 — 256000, C1 — 460800, LD06/LD19 и "
                     "LDS-01/LDS-02 — 230400) и переключатель на USB-адаптере; "
                     "полный прогон: scripts/lidar_check.py",
                quiet=True,
            )
            return False

        self.last_error = ""
        if self.detected is None:  # данные шли, но поток уже оборвался
            self._mark_found(found)
        return True

    def _mark_found(self, probe: dict[str, Any]) -> None:
        """Запоминает рабочие параметры порта (и пишет об этом в журнал один раз)."""
        port = str(probe.get("port") or "")
        self.detected = {
            "port": port,
            "protocol": probe.get("protocol"),
            "baud": probe.get("baud"),
            "motor": probe.get("motor"),
        }
        self.last_error = ""
        self._active_port = port or None
        # Подтверждённый лидар переживает перезапуск: data/lidar-found.json,
        # порт и профиль проверяются первыми в следующем поиске (см.
        # load_found_lidar). Устаревшая запись поиску не мешает.
        if save_found_lidar(self.detected):
            self._found_hint = load_found_lidar()
            self._hint_budget = 0  # лидар найден сейчас — подсказка больше не нужна
        self._set_status(
            "лидар на связи: %s, %s бод, мотор %s"
            % (os.path.basename(port), probe.get("baud"), probe.get("motor"))
        )
        # Порт найден: сообщаем это источнику данных шасси, чтобы он не занимал
        # его своей проверкой (см. lidar_holds_port) — иначе лидар терял бы порт
        # на пару секунд в каждом круге перебора модулей.
        self._publish_state("found", port)
        log.info(
            "Лидар: найден на %s (%s %s бод, мотор: %s)",
            port, probe.get("protocol"), probe.get("baud"), probe.get("motor"),
        )

    # ------------------ межпроцессный замок и обмен сканом ------------------ #

    def _lock_path(self, port: str) -> Path:
        """Файл замка порта (общий с модулями шасси — см. :func:`port_lock_file`)."""
        return port_lock_file(port)

    def _acquire_port_lock(self, port: str) -> bool:
        """Пытается занять порт. False — его уже читает другой процесс робота."""
        fd = try_lock_port(port)
        if fd is None:
            return False
        self._release_port_lock()
        self._lock_fd = fd
        return True

    def _release_port_lock(self) -> None:
        fd, self._lock_fd = self._lock_fd, None
        unlock_port(fd)

    def _peer_cycle(self) -> bool:
        """Берёт готовый скан у второго процесса робота (у него занят порт).

        Демо-скан соседа за реальный не принимается: иначе стенд без лидара
        (``RC_SLAM_SOURCE=sim``) «заражал» второй процесс выдуманными точками,
        и оба экрана показывали бы демо под видом работающего лидара.
        """
        for url in self._peer_targets():
            data = _fetch_json(url)
            if not isinstance(data, dict):
                continue
            scan = data.get("scan")
            if data.get("demo"):
                self._peer_reason = "у соседа демо-скан (реального лидара там нет)"
                continue
            if isinstance(scan, list) and scan:
                count = self.feed_points(scan, source_label="peer")
                detected = data.get("detected")
                if isinstance(detected, dict):
                    self.detected = {
                        "port": detected.get("port"),
                        "protocol": detected.get("protocol"),
                        "baud": detected.get("baud"),
                        "motor": detected.get("motor"),
                    }
                self._peer_reason = ""
                self._set_status(
                    "скан берётся у второго процесса робота (%d точек)" % count,
                    hint="этот процесс сам порт лидара не открывает",
                )
                time.sleep(0.25)  # сосед обновляет скан со скоростью лидара (10 Гц)
                return True
            reason = str(data.get("reason") or "")
            if reason:
                self._peer_reason = reason
        self._peer_misses += 1
        time.sleep(0.4)
        return False

    def _peer_targets(self) -> list[str]:
        """Адреса соседей без собственного: у себя скан брать бессмысленно."""
        return [url for url in (self.peer_urls or peer_urls()) if not is_self_url(url)]

    # ----------------------- ROS 2 интеграция ----------------------- #

    def _ros_status_block(self) -> dict[str, Any]:
        """Блок ``ros`` для ``GET /api/lidar``: окружение ROS 2 + живой мост.

        Сливаем две картины: что вообще есть в системе (дистрибутив, ``rclpy``,
        поставленные пакеты драйверов — :mod:`robot_control.ros_env`) и что
        происходит прямо сейчас (топик скана, публикаторы, супервизор драйвера —
        :class:`robot_control.ros_lidar.RosLidar`).
        """
        block = ros_env.describe(self.ros.topic)
        block.update(self.ros.status())
        block["source_mode"] = self.source_mode
        block["owns_lidar"] = self._ros_owns_lidar()
        # Подсказка ROS 2 не должна затирать причину уровня менеджера: если скан
        # идёт из ROS 2, пользователю показывать нечего.
        if self.ros.fresh():
            block["hint"] = ""
        return block

    def request_ros_rescan(self) -> str:
        """Просит супервизор ROS 2 заново подобрать драйвер лидара.

        Кнопка «Пересканировать» раньше только перезапускала USB-перебор внутри
        приложения; при источнике ``ros`` перезапускать нужно драйвер, поэтому
        вызов уходит в сервис ``~/rescan`` супервизора (пакет ``zapret_lidar``).
        """
        if not self.ros.available:
            return "ROS 2 не подключён к этому процессу"
        ok = self.ros.request_rescan()
        return self.ros.rescan_result or ("пересканирование запущено" if ok else "не удалось")

#: HTTP-порты, которые слушает ЭТОТ процесс (пульт 8080 или бортовой экран 8090).
#: У себя самого скан брать не нужно: это лишний запрос и «сосед сообщает: …»
#: из собственного статуса в подсказке.
_SELF_PORTS: set[int] = set()


def set_self_port(port: Any) -> None:
    """Запоминает порт HTTP этого процесса, чтобы не ходить к себе за сканом."""
    try:
        number = int(port)
    except (TypeError, ValueError):
        return
    if number > 0:
        _SELF_PORTS.add(number)


def url_port(url: str) -> Optional[int]:
    """Порт из адреса вида ``http://127.0.0.1:8090/api/lidar`` (None, если нет)."""
    match = re.search(r"://[^/]*?:(\d+)", url or "")
    return int(match.group(1)) if match else None


def is_self_url(url: str) -> bool:
    """Ведёт ли адрес на этот же процесс."""
    port = url_port(url)
    return port is not None and port in _SELF_PORTS


def peer_urls() -> list[str]:
    """Адреса второго процесса робота, у которого можно взять готовый скан."""
    raw = os.environ.get("RC_LIDAR_PEER_URLS")
    if raw is not None:
        return [u.strip() for u in raw.split(",") if u.strip()]
    return ["http://127.0.0.1:8090/api/lidar", "http://127.0.0.1:8080/api/lidar"]


def _fetch_json(url: str, timeout: float = 1.0) -> Optional[dict]:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:  # noqa: S310
            if response.status != 200:
                return None
            return json.loads(response.read().decode("utf-8", "replace"))
    except (urllib.error.URLError, OSError, ValueError, json.JSONDecodeError):
        return None
    except Exception:  # noqa: BLE001 — соседний процесс может ответить чем угодно
        return None
