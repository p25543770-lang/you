"""Подключение JBD BMS с экрана робота: список портов, проверка, автопоиск, сохранение.

Раньше порт BMS задавался только руками в ``.env`` (``RC_BMS_PORT``), а пока его
нет, страница «АКБ» показывала пустоту. Теперь на самой странице можно:

* увидеть все USB/serial-порты робота и чем каждый занят (мастер руля, лидар,
  модули шасси, другой процесс);
* проверить выбранный порт: одно чтение регистра 0x03 на 9600 8N1 — тот же
  запрос, что шлёт телеметрия, никакой записи в BMS;
* найти BMS автоматически среди свободных портов;
* сохранить порт в ``.env`` (атомарно, остальные строки не трогаются) и сразу
  подключиться — без перезапуска сервера.

Порт мастера руля, лидара и модулей шасси не открывается ни при проверке, ни
при автопоиске. Открытие любого другого порта может перезагрузить Arduino на
нём (линия DTR), поэтому порты, похожие на Arduino, автопоиск пропускает, а
проверка и сохранение доступны без PIN на бортовом экране.
"""

from __future__ import annotations

import glob
import os
import secrets
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Iterable

from .battery import ProtocolError, extract_frames, parse_snapshot, read_request, reserved_ports

#: USB VID плат Arduino (официальные и частые клоны с собственным USB).
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1B4F, 0x239A, 0x2886}
ENV_KEY = "RC_BMS_PORT"
PROBE_TIMEOUT_S = 1.2


def _real(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def _comports() -> dict[str, Any]:
    """Описание портов от pyserial (производитель, VID/PID); без pyserial — пусто."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return {}
    info = {}
    try:
        for item in list_ports.comports():
            info[_real(item.device)] = item
    except Exception:  # noqa: BLE001 — перечисление портов не должно ронять страницу
        return {}
    return info


def master_ports() -> set[str]:
    """Порты мастера руля и модулей шасси из настроек: их BMS-поиск не открывает.

    RC_SLAM_PORT считается занятым всегда, когда задан (и /dev/ttyACM0 по
    умолчанию в режиме master): открыть порт Arduino — значит перезагрузить её.
    """
    source = (os.environ.get("RC_SLAM_SOURCE") or "").strip().lower()
    ports: set[str] = set()
    master = (os.environ.get("RC_SLAM_PORT") or "").strip()
    if not master and source == "master":
        master = "/dev/ttyACM0"
    for value in (master, (os.environ.get("RC_SLAM_MASTER_PORT") or "").strip()):
        if value:
            ports |= {value, _real(value)}
    for chunk in (os.environ.get("RC_SLAM_PORTS") or "").split(","):
        chunk = chunk.strip()
        if chunk:
            ports |= {chunk, _real(chunk)}
    return ports


def _lidar_ports() -> set[str]:
    from .lidar import lidar_active_port, lidar_reserved_ports
    ports = set(lidar_reserved_ports())
    active = lidar_active_port()
    if active:
        ports |= {active, _real(active)}
    return ports


def _chassis_ports() -> set[str]:
    from .lidar import chassis_module_ports
    return {_real(port) for port in chassis_module_ports()} | set(chassis_module_ports())


def candidate_paths() -> list[str]:
    """Все serial-порты машины, по одному имени на устройство (by-id предпочтительнее)."""
    from .lidar import stable_port_path
    seen: dict[str, str] = {}
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/ttyS[0-3]"):
        for path in sorted(glob.glob(pattern)):
            real = _real(path)
            if real not in seen:
                seen[real] = stable_port_path(path)
    configured = (os.environ.get(ENV_KEY) or "").strip()
    if configured and os.path.exists(configured) and _real(configured) not in seen:
        seen[_real(configured)] = configured
    return sorted(seen.values())


def describe_ports(paths: Iterable[str] | None = None,
                   locked: Callable[[str], bool] | None = None) -> list[dict[str, Any]]:
    """Порты с ролями: bms / master / lidar / chassis / busy / arduino / free."""
    from .lidar import port_locked_by_other
    locked = locked or port_locked_by_other
    configured = (os.environ.get(ENV_KEY) or "").strip()
    configured_real = _real(configured) if configured else ""
    master, lidar, chassis = master_ports(), _lidar_ports(), _chassis_ports()
    details = _comports()
    result = []
    for path in (candidate_paths() if paths is None else list(paths)):
        real = _real(path)
        item = details.get(real)
        vid = getattr(item, "vid", None)
        description = " ".join(part for part in (
            getattr(item, "manufacturer", None), getattr(item, "product", None) or getattr(item, "description", None),
        ) if part and part != "n/a").strip()
        if description in (os.path.basename(real), os.path.basename(path)):
            description = ""          # pyserial повторяет имя порта — это не описание
        if configured and real == configured_real:
            role, note = "bms", "порт BMS из настроек"
        elif path in master or real in master:
            role, note = "master", "мастер руля (RC_SLAM_PORT) — не трогаем"
        elif path in lidar or real in lidar:
            role, note = "lidar", "лидар — не трогаем"
        elif path in chassis or real in chassis:
            role, note = "chassis", "модуль шасси — не трогаем"
        elif locked(path):
            role, note = "busy", "занят другим процессом"
        elif vid in ARDUINO_VIDS:
            role, note = "arduino", "похоже на Arduino: проверка может её перезагрузить"
        else:
            role, note = "free", "свободен"
        result.append({
            "port": path, "device": real, "role": role, "note": note,
            "description": description or None,
            "vid": f"{vid:04X}" if isinstance(vid, int) else None,
            "pid": f"{item.pid:04X}" if isinstance(getattr(item, "pid", None), int) else None,
            "serial": getattr(item, "serial_number", None),
        })
    return result


def _open_serial(port: str):
    import serial
    return serial.Serial(port, baudrate=9600, bytesize=8, parity="N", stopbits=1,
                         timeout=0.05, write_timeout=0.8, xonxoff=False, rtscts=False, dsrdtr=False)


def _exchange(transport, register: int, timeout: float) -> bytes:
    transport.reset_input_buffer()
    transport.write(read_request(register))
    buffer = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        buffer += transport.read(256)
        frames, buffer = extract_frames(buffer)
        for reg, status, data in frames:
            if reg == register:
                if status:
                    raise ProtocolError(f"BMS: статус 0x{status:02X}, регистр 0x{reg:02X}")
                return data
    raise ProtocolError("нет ответа JBD на 9600 8N1")


def probe_port(port: str, *, opener: Callable[[str], Any] | None = None,
               timeout: float = PROBE_TIMEOUT_S) -> dict[str, Any]:
    """Одно чтение 0x03 и 0x04: это JBD BMS или нет. Ничего не пишет в плату."""
    from .lidar import try_lock_port, unlock_port
    port = (port or "").strip()
    if not port or not os.path.exists(port):
        return {"ok": False, "port": port, "found": False, "error": "порт не существует"}
    real = _real(port)
    if port in master_ports() or real in master_ports():
        return {"ok": False, "port": port, "found": False, "error": "это порт мастера руля — не трогаем"}
    if port in _lidar_ports() or real in _lidar_ports():
        return {"ok": False, "port": port, "found": False, "error": "это порт лидара — не трогаем"}
    if port in _chassis_ports() or real in _chassis_ports():
        return {"ok": False, "port": port, "found": False, "error": "это порт модуля шасси — не трогаем"}
    fd = try_lock_port(port)
    if fd is None:
        if port in reserved_ports() or real in reserved_ports():
            return {"ok": True, "port": port, "found": True, "current": True,
                    "error": "", "summary": "этот порт уже читает BMS (текущее подключение)"}
        return {"ok": False, "port": port, "found": False, "error": "порт занят другим процессом"}
    transport = None
    try:
        transport = (opener or _open_serial)(port)
        time.sleep(0.05)
        basic = _exchange(transport, 0x03, timeout)
        cells = _exchange(transport, 0x04, timeout)
        sample = parse_snapshot(basic, cells)
        return {"ok": True, "port": port, "found": True, "error": "",
                "volts": sample["volts"], "soc": sample["soc"], "cells": sample["receivedCount"],
                "summary": "JBD BMS: %.2f В, %d %%, банок %d" % (
                    sample["volts"], sample["soc"], sample["receivedCount"])}
    except (ProtocolError, OSError, ValueError) as exc:
        return {"ok": True, "port": port, "found": False, "error": str(exc) or type(exc).__name__}
    except Exception as exc:  # noqa: BLE001 — pyserial SerialException и т. п.
        return {"ok": True, "port": port, "found": False, "error": str(exc) or type(exc).__name__}
    finally:
        try:
            if transport is not None:
                transport.close()
        finally:
            unlock_port(fd)


def scan_ports(*, probe: Callable[[str], dict[str, Any]] | None = None,
               ports: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    """Автопоиск: проверить свободные порты (не мастер, не лидар, не Arduino)."""
    probe = probe or probe_port
    described = describe_ports() if ports is None else ports
    results, skipped = [], []
    for item in described:
        if item["role"] in ("free", "bms"):
            results.append(probe(item["port"]))
        else:
            skipped.append({"port": item["port"], "role": item["role"], "note": item["note"]})
    found = [result["port"] for result in results if result.get("found")]
    return {"ok": True, "found": found, "results": results, "skipped": skipped}


def env_file_path() -> Path:
    from .lidar import env_file_path as lidar_env_path
    return lidar_env_path()


def save_port(port: str, env_path: Path | None = None) -> str:
    """Записать RC_BMS_PORT в .env атомарно; остальные строки файла не меняются."""
    port = (port or "").strip()
    if any(ch in port for ch in "\r\n\"'` $;"):
        raise ValueError("недопустимые символы в имени порта")
    if port and not port.startswith("/"):
        raise ValueError("нужен полный путь к порту: /dev/ttyUSB0 или /dev/serial/by-id/…")
    target = Path(env_path) if env_path is not None else env_file_path()
    lines = target.read_text(encoding="utf-8").splitlines() if target.is_file() else []
    out, seen = [], False
    for raw in lines:
        key = raw.strip().split("=", 1)[0].strip() if "=" in raw else ""
        if key == ENV_KEY:
            if not seen:
                out.append(f"{ENV_KEY}={port}")
            seen = True
        else:
            out.append(raw)
    if not seen:
        out.append(f"{ENV_KEY}={port}")
    if not target.is_file() and not any(line.strip().startswith("RC_SECRET_KEY=") for line in out):
        out.insert(0, f"RC_SECRET_KEY={secrets.token_urlsafe(48)}")
    target.parent.mkdir(parents=True, exist_ok=True)
    mode = 0o600
    try:
        mode = target.stat().st_mode & 0o777 or 0o600
    except OSError:
        pass
    fd, tmp_name = tempfile.mkstemp(prefix=".env-", suffix=".tmp", dir=target.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write("\n".join(out).rstrip() + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(tmp_name, mode)
        os.replace(tmp_name, target)
    finally:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass
    return port


def apply_port(manager, port: str, env_path: Path | None = None) -> dict[str, Any]:
    """Проверить, сохранить в .env и сразу переподключить BMS (пустой порт — отключить)."""
    port = (port or "").strip()
    if port:
        if not os.path.exists(port):
            raise ValueError("порт %s не найден" % port)
        real = _real(port)
        if port in master_ports() or real in master_ports():
            raise ValueError("это порт мастера руля (RC_SLAM_PORT)")
        if port in _lidar_ports() or real in _lidar_ports():
            raise ValueError("это порт лидара")
    save_port(port, env_path)
    if port:
        os.environ[ENV_KEY] = port
    else:
        os.environ.pop(ENV_KEY, None)
    manager.reconfigure(port)
    return {"ok": True, "port": port or None, "saved": True,
            "battery": manager.snapshot()}


def setup_payload(manager) -> dict[str, Any]:
    snap = manager.snapshot()
    return {
        "ok": True, "available": True, "envKey": ENV_KEY,
        "port": manager.port or None, "configured": bool(manager.port),
        "connected": snap["connected"], "reason": snap["reason"],
        "ports": describe_ports(),
    }
