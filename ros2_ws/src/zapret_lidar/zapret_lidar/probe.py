# Copyright 2026 Zapret robot control
#
# Быстрая проверка порта перед запуском драйвера: читает байты и говорит, какой
# это лидар. Нужна для ``model:=auto`` — иначе супервизор тратил бы по таймауту
# на каждый драйвер, а оператор ждал бы скан десятки секунд.
"""Проверка последовательного порта: какой лидар на нём виден."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Optional

from .frames import PROTOCOL_BAUDS, RPLIDAR_CMD_SCAN, SCORE_WEAK, score_bytes

log = logging.getLogger("zapret_lidar.probe")

#: Сколько читаем на одной скорости, с. Меньше — не увидим кадры LDS-01 (они
#: идут ~5 Гц), больше — перебор портов станет заметным на глаз.
READ_WINDOW_S = 0.45


def _open(port: str, baud: int, timeout: float = 0.15):
    """Открывает порт через pyserial; None, если библиотеки нет или порт занят."""
    try:
        import serial  # pyserial
    except ImportError:  # pragma: no cover — на роботе pyserial стоит в .venv
        log.debug("нет pyserial: проверка порта пропускается")
        return None
    try:
        return serial.Serial(port, baud, timeout=timeout)
    except Exception as exc:  # noqa: BLE001 — порт занят/нет прав/нет устройства
        log.debug("порт %s: не открылся на %s бод: %s", port, baud, exc)
        return None


def read_bytes(port: str, baud: int, window: float = READ_WINDOW_S,
               *, command: Optional[bytes] = None) -> bytes:
    """Читает байты порта за окно времени (без исключений: пустая строка — молчит)."""
    handle = _open(port, baud)
    if handle is None:
        return b""
    try:
        if command:
            try:
                handle.dtr = False
                handle.rts = False
            except Exception:  # noqa: BLE001 — у части адаптеров линий нет
                pass
            try:
                handle.write(command)
                handle.flush()
            except Exception as exc:  # noqa: BLE001 — передатчик залип
                log.debug("порт %s: команда не ушла: %s", port, exc)
                return b""
        chunks: list[bytes] = []
        while True:
            try:
                data = handle.read(4096)
            except Exception as exc:  # noqa: BLE001 — обрыв кабеля на ходу
                log.debug("порт %s: чтение оборвалось: %s", port, exc)
                break
            if not data:
                break
            chunks.append(data)
            if sum(len(chunk) for chunk in chunks) >= 64 * 1024:
                break
        return b"".join(chunks)
    finally:
        try:
            handle.close()
        except Exception:  # noqa: BLE001
            pass


def probe_port(port: str, *, bauds: Optional[dict[str, tuple[int, ...]]] = None) -> dict[str, Any]:
    """Что видно на порту: протокол, оценка и скорости, на которых проверяли.

    Возвращает ``{"port":…, "protocol": "", "score": 0.0, "bauds": [...],
    "scores": {"lds01": 1.0, …}}``. Пустой ``protocol`` — порт молчит или на нём
    не лидар; это не ошибка, а результат проверки.
    """
    rates = bauds or PROTOCOL_BAUDS
    scores: dict[str, float] = {}
    for protocol, candidates in rates.items():
        for baud in candidates:
            # RPLIDAR сам молчит: его надо попросить о скане командой SCAN.
            command = RPLIDAR_CMD_SCAN if protocol == "rplidar" else None
            data = read_bytes(port, baud, command=command)
            if not data:
                continue
            score = score_bytes(data, protocol)
            if score > scores.get(protocol, 0.0):
                scores[protocol] = score
            if score >= 1.0 and protocol in ("lds01", "ld06"):
                break  # поток односторонний: убедились и хватит
    protocol, score = best_protocol_from(scores)
    return {"port": port, "protocol": protocol, "score": round(score, 3),
            "bauds": {name: list(values) for name, values in rates.items()},
            "scores": {name: round(value, 3) for name, value in scores.items()}}


def best_protocol_from(scores: dict[str, float]) -> tuple[str, float]:
    """Протокол с наибольшей оценкой (пусто, если все оценки ниже порога)."""
    if not scores:
        return "", 0.0
    name = max(scores, key=lambda key: scores[key])
    score = float(scores[name])
    if score < SCORE_WEAK:
        return "", 0.0
    return name, score


def probe_ports(ports: list[str], *, limit: int = 4) -> dict[str, dict[str, Any]]:
    """Проверяет порты по очереди и возвращает отчёт по каждому (для статуса)."""
    reports: dict[str, dict[str, Any]] = {}
    for port in ports[:limit]:
        if not Path(port).exists():
            continue
        reports[port] = probe_port(port)
    return reports


def scores_for_plan(reports: dict[str, dict[str, Any]]) -> dict[str, float]:
    """Оценки в виде, который понимает :func:`zapret_lidar.drivers.build_plan`."""
    out: dict[str, float] = {}
    for port, report in reports.items():
        for protocol, score in (report.get("scores") or {}).items():
            key = "%s@%s" % (protocol, port)
            out[key] = max(out.get(key, 0.0), float(score))
    return out
