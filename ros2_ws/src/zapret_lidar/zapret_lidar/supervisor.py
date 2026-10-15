# Copyright 2026 Zapret robot control
#
# Точка входа пакета ``zapret_lidar``.
#
#   * ``--print-plan``  — показать, какие драйверы и на каких портах будут
#     пробоваться (работает без ROS 2: удобно для проверки на стенде и в тестах);
#   * ``--list-drivers`` — таблица поддерживаемых лидаров и их пакетов;
#   * ``--probe`` — быстро проверить порты и напечатать, какой лидар виден;
#   * без аргументов — запустить узел ``lidar_supervisor`` (нужен rclpy).
"""CLI пакета: план запуска драйверов и сам супервизор."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Optional

if __package__ in (None, ""):  # pragma: no cover — запуск файлом, а не -m zapret_lidar
    # Разрешаем `python3 zapret_lidar/supervisor.py --print-plan`: без этого
    # относительный импорт ниже падал бы, а диагностика должна работать даже до
    # сборки рабочего пространства (см. ros2_ws/README.md).
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    __package__ = "zapret_lidar"

from . import drivers as drv  # noqa: E402


def default_prefixes() -> list[Path]:
    """Префиксы ROS 2 для проверки установленных пакетов (как в узле)."""
    out: list[Path] = []
    for value in (os.environ.get("RC_ROS_PREFIX"), os.environ.get("AMENT_PREFIX_PATH")):
        for item in (value or "").split(os.pathsep):
            if item.strip():
                out.append(Path(item.strip()))
    for distro in (os.environ.get("ROS_DISTRO"), "humble", "jazzy"):
        if distro:
            out.append(Path("/opt/ros") / distro)
    try:
        out.extend(sorted(path for path in Path("/opt/ros").iterdir() if path.is_dir()))
    except OSError:
        pass
    return [path for index, path in enumerate(out) if path.is_dir() and path not in out[:index]]


def print_plan(model: str, port: str, baud: int, prefix: Optional[str],
               installed: str, as_json: bool) -> int:
    """Печатает план попыток: драйвер, порт, скорость, установлен ли пакет."""
    if prefix:
        prefixes = [Path(prefix)]
    elif installed:
        prefixes = []
    else:
        prefixes = default_prefixes()
    if installed:
        names = {item.strip() for item in installed.split(",") if item.strip()}
    else:
        names = drv.installed_packages(prefixes) if prefixes else None
    plan = drv.build_plan(model=model, ports=port or None, installed=names)
    if as_json:
        print(json.dumps({
            "model": model,
            "port": port,
            "installed": sorted(names) if names is not None else None,
            "plan": [item.as_dict() for item in plan],
        }, ensure_ascii=False, indent=2))
        return 0
    if not plan:
        print("План пуст: для модели %r не нашлось драйверов." % model)
        return 1
    print("План запуска драйверов лидара (model=%s):" % model)
    for index, item in enumerate(plan, 1):
        state = "пакет есть" if item.installed else "НЕТ пакета %s" % item.driver.package
        print("  %d. %-8s %-10s %-8s %-22s %s"
              % (index, item.driver.name, item.port or "—", item.baud or "авто",
                 item.driver.title, state))
        if not item.installed:
            for line in drv.install_lines(item.driver, os.environ.get("ROS_DISTRO") or "humble"):
                print("       поставить: %s" % line)
    return 0


def list_drivers(as_json: bool) -> int:
    """Таблица драйверов: модель, пакет, узел, apt/исходники."""
    rows = []
    for name in drv.driver_names():
        driver = drv.DRIVERS[name]
        rows.append({
            "driver": name,
            "title": driver.title,
            "package": driver.package,
            "executable": driver.executable,
            "protocols": list(driver.protocols),
            "baud": driver.baud,
            "apt": driver.apt.format(d=os.environ.get("ROS_DISTRO") or "humble"),
            "source": driver.source,
            "note": driver.note,
        })
    if as_json:
        print(json.dumps(rows, ensure_ascii=False, indent=2))
        return 0
    print("Драйверы лидара, которые умеет поднимать zapret_lidar:")
    for row in rows:
        where = row["apt"] or ("из исходников: " + row["source"])
        print("  %-8s %-46s %s" % (row["driver"], row["title"], where))
        print("           узел: %s/%s · бод: %s" % (row["package"], row["executable"],
                                                    row["baud"] or "авто"))
        if row["note"]:
            print("           %s" % row["note"])
    return 0


def probe(ports: str, as_json: bool) -> int:
    """Проверяет порты и печатает, какой лидар на них виден."""
    from .probe import probe_port  # pyserial нужен только здесь

    found = drv.candidate_ports(ports or None)
    if not found:
        print("Портов-кандидатов нет (проверь RC_LIDAR_PORT и подключение).")
        return 1
    reports = [probe_port(port) for port in found]
    if as_json:
        print(json.dumps({"ports": reports}, ensure_ascii=False, indent=2))
    else:
        for report in reports:
            protocol = report["protocol"] or "не опознан"
            print("%-24s %-12s %s" % (report["port"], protocol, report["scores"]))
    return 0 if any(report["protocol"] for report in reports) else 1


def main(argv: Optional[list[str]] = None) -> int:
    """Точка входа: план/проверка портов без ROS 2, запуск узла — с ROS 2."""
    parser = argparse.ArgumentParser(
        description="Супервизор драйвера лидара ROS 2 (пакет zapret_lidar)",
    )
    parser.add_argument("--print-plan", action="store_true",
                        help="показать порядок запуска драйверов и выйти")
    parser.add_argument("--list-drivers", action="store_true",
                        help="показать поддерживаемые драйверы и выйти")
    parser.add_argument("--probe", action="store_true",
                        help="проверить порты и выйти")
    parser.add_argument("--json", action="store_true", help="машинный вывод")
    parser.add_argument("--model", default=os.environ.get("RC_LIDAR_PROTOCOL") or "auto",
                        help="модель лидара: auto | lds01 | rplidar | ld06 | ld19 | stl27l | opencr")
    parser.add_argument("--port", default=os.environ.get("RC_LIDAR_PORT") or "",
                        help="порт(ы) лидара через запятую; можно glob")
    parser.add_argument("--baud", type=int, default=int(os.environ.get("RC_LIDAR_BAUD") or 0),
                        help="скорость порта; 0 — по умолчанию для драйвера")
    parser.add_argument("--prefix", default=os.environ.get("RC_ROS_PREFIX") or "",
                        help="префикс ROS 2 для проверки пакетов")
    parser.add_argument("--installed", default="",
                        help="список установленных пакетов через запятую (для тестов)")
    args, rest = parser.parse_known_args(argv)

    if args.list_drivers:
        return list_drivers(args.json)
    if args.print_plan:
        return print_plan(args.model, args.port, args.baud, args.prefix,
                          args.installed, args.json)
    if args.probe:
        return probe(args.port, args.json)

    # Обычный запуск: дальше нужен rclpy, поэтому импорт узла — только здесь.
    try:
        from .node import main as node_main
    except ImportError as exc:  # pragma: no cover — зависит от окружения робота
        print("ROS 2 (rclpy) не найден: %s" % exc, file=sys.stderr)
        print("Поставь ROS 2 и пакеты драйверов: ./scripts/ros2_setup.sh", file=sys.stderr)
        print("Проверить план без ROS 2: --print-plan", file=sys.stderr)
        return 2
    extra = list(rest)
    if extra:
        # Аргументы командной строки → параметры ROS 2 (--ros-args -p key:=value).
        if "--ros-args" not in extra:
            extra = ["--ros-args"] + extra
    return node_main(extra or None)


if __name__ == "__main__":
    sys.exit(main())
