#!/usr/bin/env python3
"""Проверка лидара на роботе: что подключено, на какой скорости и почему нет скана.

Скрипт ничего не меняет в системе — только открывает порты на чтение. Запускать
на роботе (можно без остановки пульта; если порт занят другим процессом, скрипт
честно об этом скажет):

    .venv/bin/python scripts/lidar_check.py              # найти лидар сам
    .venv/bin/python scripts/lidar_check.py --list       # только список портов
    .venv/bin/python scripts/lidar_check.py --port /dev/ttyUSB0
    .venv/bin/python scripts/lidar_check.py --port /dev/ttyUSB0 --baud 256000
    .venv/bin/python scripts/lidar_check.py --json       # для скриптов
    .venv/bin/python scripts/lidar_check.py --save-env   # записать найденное в .env

``--save-env`` сохраняет найденные порт/протокол/скорость/мотор в ``.env``
(RC_LIDAR_PORT, RC_LIDAR_PROTOCOL, RC_LIDAR_BAUD, RC_LIDAR_MOTOR), чтобы
веб-приложения не тратили время на перебор. Порт пишется постоянным именем
``/dev/serial/by-id/…``, если оно есть: номер ``/dev/ttyUSBx`` после
перезагрузки может измениться. После записи веб-приложения нужно перезапустить
(``./scripts/stop.sh && ./scripts/start.sh``), а проверить, что скан дошёл до
обоих экранов, — ``scripts/lidar_web_check.py``.

Коды возврата: 0 — лидар найден и отдаёт точки, 1 — не найден (в выводе причина).

Что означает вывод:
  * «нет доступа (Permission denied)» — пользователь не в группе dialout;
  * «порт занят» — его уже открыл пульт/киоск/rplidar_ros/Serial Monitor;
  * «ответил на SCAN, но точек нет» — скорость угадана, а мотор не крутится:
    нужен другой уровень линии DTR/RTS (RC_LIDAR_MOTOR) или питание лидара;
  * «нет ответа» — скорость не та (A1/A2M8 — 115200, A2M7/A2M12/A3/S1 — 256000,
    C1 — 460800, LD06/LD19 и LDS-01/LDS-02 — 230400) или лидар подключён не к тому
    порту. Лидар TurtleBot3 Burger (LDS-01, он же HLS-LFCD2) идёт кадрами 0xFA по
    42 байта; в списке настроек он назван протоколом ``lds01``.

Проверяются ВСЕ USB-порты (и порт из RC_LIDAR_PORT первым): лидар легко
переезжает на соседний /dev/ttyUSBx после переподключения.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_control.lidar import (  # noqa: E402
    LIDAR_MIN_POINTS,
    LIDAR_PROBE_TIMEOUT_S,
    LIDAR_PROTOCOLS,
    discover_ports,
    excluded_ports,
    lidar_profiles,
    motor_variant_name,
    motor_variants,
    open_serial,
    probe_port,
    stable_port_path,
)

PROTOCOL_TITLES = {
    "rplidar": "RPLIDAR (A1/A2/A3/S1/C1)",
    "ld06": "LD06/LD19 (D500 и клоны)",
    "lds01": "LDS-01/LDS-02 — лидар TurtleBot3 Burger (HLS-LFCD2)",
}

ENV_FILE = Path(__file__).resolve().parent.parent / ".env"
#: Какие ключи .env правит --save-env (остальные строки файла не трогаем).
LIDAR_ENV_KEYS = ("RC_LIDAR_PORT", "RC_LIDAR_PROTOCOL", "RC_LIDAR_BAUD", "RC_LIDAR_MOTOR")


def log(text: str = "") -> None:
    print(text, flush=True)


def collect_ports(port: str = "") -> list[str]:
    """Порты-кандидаты: явно заданный, потом symlink'и, потом все остальные tty.

    Проверка всегда перебирает все USB-порты: порт из ``RC_LIDAR_PORT`` — только
    первый в очереди. Иначе скрипт повторял бы ошибку «лидар виден только на одном
    USB» и не находил лидар, переехавший на соседний /dev/ttyUSBx.
    """
    return discover_ports(
        configured=port or os.environ.get("RC_LIDAR_PORT"),
        autoprobe=True,
        excluded=excluded_ports(),
    )


def describe_ports(port: str = "") -> None:
    ports = collect_ports(port)
    blocked = sorted(p for p in excluded_ports() if p)
    log("Порты, которые видит система:")
    if not ports:
        log("  — подходящих портов нет")
    for path in ports:
        try:
            target = os.path.realpath(path)
            note = "" if target == path else " → %s" % target
        except OSError:
            note = ""
        log("  • %s%s" % (path, note))
    if blocked:
        log("Исключены (мастер/модули шасси или RC_LIDAR_EXCLUDE): %s" % ", ".join(blocked))
    if (os.environ.get("RC_LIDAR_AUTOPROBE") or "").strip().lower() in {"0", "false", "no", "off"}:
        log("Примечание: RC_LIDAR_AUTOPROBE=0 в приложении — «сначала закреплённый порт», "
            "а не «только он»; проверка всё равно проходит все порты выше.")
    log()


def probe(port: str, *, baud=None, motor="auto", protocol="auto", timeout=LIDAR_PROBE_TIMEOUT_S,
          seconds: float = 2.0, min_points: int = LIDAR_MIN_POINTS) -> dict:
    """Открывает порт и подбирает параметры. Возвращает отчёт для печати."""
    report: dict = {"port": port, "ok": False, "protocol": None, "baud": None,
                    "motor": None, "points": 0, "samples": [], "skip": False, "error": None}

    try:
        import serial  # pyserial
    except ImportError:
        report["error"] = ("нет библиотеки pyserial — поставь: .venv/bin/pip install pyserial "
                           "(или ./scripts/ensure_deps.sh)")
        return report

    deadline = time.monotonic() + max(0.5, seconds)
    stop = {"flag": False}
    scans: list[list[dict]] = []

    def publish(points):
        scans.append(points)
        if time.monotonic() >= deadline:
            stop["flag"] = True

    try:
        ser = open_serial(serial, port, 115200)
    except Exception as exc:  # noqa: BLE001 — порт занят/нет прав/нет устройства
        text = str(exc)
        if isinstance(exc, PermissionError) or "permission" in text.lower():
            report["error"] = ("нет доступа к порту — добавь пользователя в группу dialout: "
                               "sudo usermod -aG dialout $USER (и перезайди)")
        elif "busy" in text.lower() or "resource" in text.lower():
            report["skip"] = True
            report["error"] = ("порт занят другой программой (пульт/киоск, rplidar_ros, "
                              "Serial Monitor) — останови её или смотри скан на её экране")
        else:
            report["error"] = text
        return report

    try:
        found = probe_port(
            ser, port,
            protocol=protocol, baud=baud, motor=motor,
            timeout=timeout,
            stop=lambda: stop["flag"],
            publish=publish,
            min_points=min_points,
        )
    except Exception as exc:  # noqa: BLE001
        report["error"] = str(exc)
        return report
    finally:
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass

    if found:
        report.update({"ok": True, "protocol": found.get("protocol"), "baud": found.get("baud"),
                       "motor": found.get("motor"), "points": found.get("points")})
        if scans:
            report["samples"] = scans[0][:5]
        return report

    # ничего не нашли: объясняем, что именно проверили
    report["tried"] = [
        {"protocol": proto, "baud": rate,
         "motor": [motor_variant_name(v) for v in (motor_variants(motor) if proto == "rplidar" else ((None, None),))]}
        for proto, rate in lidar_profiles(protocol, baud)
    ]
    return report


def env_settings(report: dict) -> dict:
    """Настройки .env по найденному лидару (порт — постоянным именем)."""
    return {
        "RC_LIDAR_PORT": stable_port_path(report.get("port") or ""),
        "RC_LIDAR_PROTOCOL": report.get("protocol") or "auto",
        "RC_LIDAR_BAUD": str(report.get("baud") or ""),
        "RC_LIDAR_MOTOR": report.get("motor") or "none",
    }


def save_env(report: dict, path: Path = ENV_FILE) -> dict:
    """Пишет найденные настройки лидара в ``.env``, не трогая остальные строки.

    Возвращает {ключ: (было, стало)} — видно, что именно изменилось. Файл
    перезаписывается атомарно, права 600 сохраняются (в .env лежит
    RC_SECRET_KEY).
    """
    values = env_settings(report)
    lines: list[str] = []
    if path.is_file():
        lines = path.read_text(encoding="utf-8").splitlines()

    changes: dict = {}
    seen: set = set()
    out: list[str] = []
    for raw in lines:
        key = raw.split("=", 1)[0].strip() if "=" in raw else ""
        if key in values and not raw.lstrip().startswith("#"):
            old = raw.split("=", 1)[1].strip()
            changes[key] = (old, values[key])
            seen.add(key)
            out.append("%s=%s" % (key, values[key]))
        else:
            out.append(raw)
    for key in LIDAR_ENV_KEYS:
        if key in values and key not in seen and values[key]:
            changes[key] = (None, values[key])
            out.append("%s=%s" % (key, values[key]))

    if not any(line.startswith("RC_SECRET_KEY=") for line in out):
        # Иначе run.py увидит существующий .env и не создаст ключ: сессии и
        # CSRF-токены сбрасывались бы при каждом перезапуске пульта.
        import secrets
        out.insert(0, "RC_SECRET_KEY=%s" % secrets.token_urlsafe(48))
        changes["RC_SECRET_KEY"] = (None, "(создан)")

    text = "\n".join(out).rstrip("\n") + "\n"
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    try:
        os.chmod(tmp, 0o600)
    except OSError:
        pass
    os.replace(tmp, path)
    return changes


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Проверка лидара: порт, скорость, протокол, точки (робот не трогаем)",
    )
    parser.add_argument("--port", default="", help="порт или glob (по умолчанию — поиск сам)")
    parser.add_argument("--baud", type=int, default=None, help="перебрать только эту скорость")
    parser.add_argument("--protocol", choices=list(LIDAR_PROTOCOLS), default="auto",
                        help="auto | rplidar | ld06 | lds01 (LDS-01/LDS-02 — TurtleBot3); "
                             "понимает и синонимы: lds, hls-lfcd2, turtlebot3")
    parser.add_argument("--motor", default="auto",
                        help="управление мотором RPLIDAR: auto | none | dtr-low | dtr-high | rts-low | rts-high")
    parser.add_argument("--list", action="store_true", help="только показать порты и выйти")
    parser.add_argument("--seconds", type=float, default=2.0, help="сколько секунд читать порт")
    parser.add_argument("--points", type=int, default=LIDAR_MIN_POINTS,
                        help="сколько точек считать признаком живого лидара (по умолчанию %d)" % LIDAR_MIN_POINTS)
    parser.add_argument("--json", action="store_true", help="вывод в JSON (для скриптов)")
    parser.add_argument("--save-env", action="store_true",
                        help="записать найденные порт/протокол/скорость/мотор в .env "
                             "(после этого веб-приложения нужно перезапустить)")
    parser.add_argument("--env-file", default=str(ENV_FILE),
                        help="файл .env для --save-env (по умолчанию .env проекта)")
    args = parser.parse_args(argv)

    if not args.json:
        log("Проверка лидара: пульт/киоск можно не останавливать —\n"
            "занятый порт будет виден в отчёте.\n")

    ports = collect_ports(args.port)
    if args.list:
        describe_ports(args.port)
        return 0 if ports else 1

    if not args.json:
        describe_ports(args.port)

    if not ports:
        if args.json:
            log(json.dumps({"ok": False, "error": "портов-кандидатов нет",
                            "hint": "подключи лидар или задай RC_LIDAR_PORT"}, ensure_ascii=False))
        else:
            log("Лидар не найден: подходящих портов нет.")
            log("• подключи лидар (проверь `dmesg | tail` после подключения) и повтори;")
            log("• если лидар на нестандартном порту — задай RC_LIDAR_PORT (например /dev/ttyAMA0).")
        return 1

    reports = [probe(p, baud=args.baud, motor=args.motor, protocol=args.protocol,
                     seconds=args.seconds, min_points=max(1, args.points))
               for p in ports]
    found = next((r for r in reports if r["ok"]), None)

    if args.json:
        payload = {"ok": bool(found), "reports": reports}
        if found:
            payload["settings"] = env_settings(found)
            if args.save_env:
                try:
                    payload["env_saved"] = {
                        key: {"was": old, "now": new}
                        for key, (old, new) in save_env(found, Path(args.env_file)).items()
                    }
                except OSError as exc:
                    payload["env_saved"] = {"error": str(exc)}
        log(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if found else 1

    for report in reports:
        title = PROTOCOL_TITLES.get(report.get("protocol") or "", report.get("protocol") or "")
        if report["ok"]:
            log("✔ %s — лидар отвечает: %s, %s бод, мотор: %s, точек за виток: %s"
                % (report["port"], title, report["baud"], report["motor"], report["points"]))
            for point in report["samples"]:
                log("    точка: угол %.3f рад, %.2f м" % (point["a"], point["r"]))
        elif report.get("error"):
            log("✘ %s — %s" % (report["port"], report["error"]))
        else:
            tried = ", ".join("%s@%s" % (t["protocol"], t["baud"]) for t in report.get("tried", []))
            log("✘ %s — нет ответа ни на одной настройке (%s)" % (report["port"], tried))

    log()
    if found:
        log("Итог: лидар на %s (%s, %s бод, мотор %s)." %
            (found["port"], found["protocol"], found["baud"], found["motor"]))
        settings = env_settings(found)
        if args.save_env:
            try:
                changes = save_env(found, Path(args.env_file))
            except OSError as exc:
                log("✘ не удалось записать %s: %s" % (args.env_file, exc))
                return 1
            log("Записано в %s:" % args.env_file)
            for key in LIDAR_ENV_KEYS:
                old, new = changes.get(key, (None, settings.get(key)))
                if new is None:
                    continue
                log("  %s=%s%s" % (key, new, "" if old in (None, new) else "  (было: %s)" % (old or "—")))
            log("Теперь перезапусти веб-приложения, иначе они останутся со старыми настройками:")
            log("  ./scripts/stop.sh && ./scripts/start.sh &   # пульт (8080)")
            log("  ./scripts/kiosk.sh                        # бортовой экран (8090)")
            log("Проверка, что скан дошёл до обоих экранов:")
            log("  .venv/bin/python scripts/lidar_web_check.py")
            return 0
        log("Чтобы не тратить время на перебор, можно закрепить в .env "
            "(или сразу: --save-env):")
        log("  RC_LIDAR_PORT=%s" % settings["RC_LIDAR_PORT"])
        if settings["RC_LIDAR_PORT"] != found["port"]:
            log("  (это постоянное имя того же порта %s: номер ttyUSBx может "
                "измениться после перезагрузки)" % found["port"])
        log("  RC_LIDAR_PROTOCOL=%s" % settings["RC_LIDAR_PROTOCOL"])
        log("  RC_LIDAR_BAUD=%s" % settings["RC_LIDAR_BAUD"])
        if found["motor"] and found["motor"] != "none":
            log("  RC_LIDAR_MOTOR=%s" % found["motor"])
        return 0

    log("Итог: лидар не найден.")
    log("• скорость зависит от модели: A1/A2M8 — 115200, A2M7/A2M12/A3/S1 — 256000,")
    log("  C1 — 460800, LD06/LD19 и LDS-01/LDS-02 (TurtleBot3) — 230400")
    log("  (у RPLIDAR A3 на адаптере есть переключатель 115200/256000);")
    log("• проверь питание и кабель лидара, а также что порт не занят другой программой;")
    log("• если лидар подключён к мастер-плате (а не к USB) — сканы должен отдавать её протокол.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
