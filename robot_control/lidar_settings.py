"""Настройки лидара для веб-панели: чтение, проверка и сохранение в ``.env``.

Раньше порт/протокол/скорость/линию мотора можно было задать только руками в
``.env`` на роботе, а увидеть ошибку — только в журнале. Теперь оператор на
странице пульта открывает панель «Лидар», выбирает значения, и приложение:

  * проверяет их (:func:`robot_control.lidar.normalize_lidar_settings`);
  * пишет в ``.env`` атомарно, не трогая остальные строки (секрет, порты шасси,
    PIN и т. д.) — настройки переживают перезапуск обоих приложений;
  * применяет их к менеджеру лидара сразу, без перезапуска сервера;
  * второй процесс (бортовой экран 8090) подхватывает тот же ``.env`` по mtime —
    см. ``LidarManager._sync_settings_from_env_file``.

Секретов в настройках лидара нет. GET и запись доступны без PIN на бортовом
экране; операторский пульт по-прежнему требует вход и CSRF-токен. Команды
руления и тяги имеют отдельную защиту и этими настройками не затрагиваются.
"""

from __future__ import annotations

import os
import secrets
import tempfile
from pathlib import Path
from typing import Any

from .config import PROJECT_ROOT
from .lidar import (
    LIDAR_BAUD_CHOICES,
    LIDAR_PROTOCOLS,
    LIDAR_SOURCES,
    MOTOR_MODES,
    LidarManager,
    discover_ports,
    env_file_path,
    lidar_settings_from_env,
    normalize_lidar_settings,
    usb_diagnostics,
)

#: Какие ключи .env принадлежат лидару (остальные строки файла не трогаем).
ENV_KEYS = (
    "RC_LIDAR_SOURCE", "RC_LIDAR_PORT", "RC_LIDAR_PROTOCOL", "RC_LIDAR_BAUD",
    "RC_LIDAR_MOTOR", "RC_LIDAR_AUTOPROBE",
)


def settings_payload(manager: LidarManager | None = None) -> dict[str, Any]:
    """Всё, что нужно панели: текущие настройки, варианты и живой статус."""
    manager = manager or LidarManager.get_instance()
    discovered = discover_ports(configured=manager.port, autoprobe=True,
                                excluded=manager.excluded)
    return {
        "ok": True,
        "settings": manager.current_settings(),
        "options": {
            "sources": list(LIDAR_SOURCES),
            "protocols": list(LIDAR_PROTOCOLS),
            "bauds": ["", *[str(rate) for rate in LIDAR_BAUD_CHOICES]],
            "motors": [mode for mode in MOTOR_MODES if mode != "off"],
            "ports": discovered,
        },
        "status": manager.get_status(),
        "usb": usb_diagnostics(),
    }


def save_lidar_settings(settings: dict[str, Any],
                        env_path: Path | None = None) -> dict[str, Any]:
    """Проверяет настройки и пишет их в ``.env``. Возвращает нормализованные.

    Файл переписывается атомарно (временный файл + ``os.replace``), права
    сохраняются, а при первом создании добавляется и ``RC_SECRET_KEY`` — иначе
    сессии пульта слетали бы при каждом перезапуске.
    """
    normalized = normalize_lidar_settings(settings)
    target = Path(env_path) if env_path is not None else env_file_path()
    lines: list[str] = []
    if target.is_file():
        lines = target.read_text(encoding="utf-8").splitlines()
    values = {
        "RC_LIDAR_SOURCE": normalized["source"],
        "RC_LIDAR_PORT": normalized["port"],
        "RC_LIDAR_PROTOCOL": normalized["protocol"],
        "RC_LIDAR_BAUD": normalized["baud"],
        "RC_LIDAR_MOTOR": normalized["motor"],
        "RC_LIDAR_AUTOPROBE": "1" if normalized["autoprobe"] else "0",
    }
    seen: set[str] = set()
    out: list[str] = []
    for raw in lines:
        stripped = raw.strip()
        key = stripped.split("=", 1)[0].strip() if "=" in stripped else ""
        if key in values:
            if key in seen:
                continue  # дубль ключа в старом файле — оставляем один
            seen.add(key)
            out.append(f"{key}={values[key]}")
        else:
            out.append(raw)
    for key in ENV_KEYS:
        if key not in seen:
            out.append(f"{key}={values[key]}")
    if target.is_file() is False and not any(
            line.strip().startswith("RC_SECRET_KEY=") for line in out):
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
    return normalized


def apply_lidar_settings(payload: Any,
                         manager: LidarManager | None = None,
                         env_path: Path | None = None) -> dict[str, Any]:
    """Сохраняет настройки в ``.env`` и сразу применяет к менеджеру лидара."""
    normalized = save_lidar_settings(payload, env_path=env_path)
    manager = manager or LidarManager.get_instance()
    applied = manager.reconfigure(normalized)
    return {"ok": True, "settings": applied, "status": manager.get_status()}


def lidar_env_path() -> Path:
    """Путь ``.env`` проекта (тот же, что читает ``Config.from_env``)."""
    return Path(PROJECT_ROOT) / ".env"


def settings_from_disk(env_path: Path | None = None) -> dict[str, Any]:
    """Настройки из ``.env`` на диске (для диагностики, не для менеджера)."""
    from .lidar import read_env_file
    return lidar_settings_from_env(read_env_file(env_path or lidar_env_path()))
