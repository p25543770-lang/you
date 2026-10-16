"""Отрисовка карты лидара на экранах: node-прогон slam_gui/main.js и robot.js.

Экранные скрипты написаны под браузер и ничего не экспортируют, поэтому их
поведение проверяется через ``tests/js/lidar_screen.js``: поднимается минимальный
DOM, скрипт выполняется как в странице, ``fetch`` отдаёт заготовленный ответ API,
а результат читается из тех же элементов, которые видит оператор (подпись под
картой, счётчик точек, плашка «скана нет», число нарисованных точек на канве).

Здесь закреплено то, что ломалось по-настоящему:

* реальный скан обязан доехать до **обоих** экранов и нарисоваться;
* демо-скан обязан быть подписан как демо, а не как работающий лидар;
* при отсутствии скана показывается причина и USB-диагностика;
* отрисовка не должна падать (``renderErrors``) — одна ошибка в чужом блоке
  раньше обрывала весь ``render()`` и карта лидара просто переставала
  обновляться, хотя данные приходили исправно.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
HARNESS = Path(__file__).resolve().parent / "js" / "lidar_screen.js"
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(
    NODE is None, reason="нет node: экранные скрипты проверяются только с ним")

SCREENS = ("main", "robot")


# ---------------------------------------------------------------------------
# Заготовки ответов API (форма /api/state и /api/kiosk_state)
# ---------------------------------------------------------------------------


def scan_points(count: int, radius: float = 1.5) -> list[dict]:
    step = round(2 * 3.141592653589793 / max(1, count), 4)
    return [{"a": round(i * step, 4), "r": radius} for i in range(count)]


def lidar_status(**overrides) -> dict:
    """Состояние лидара из robot_control/lidar.py: Status.as_dict()."""
    status = {
        "ok": True,
        "connected": True,
        "source": "serial:ttyUSB1",
        "demo": False,
        "points_count": 360,
        "age_s": 0.1,
        "scan_at": 1_800_000_000.0,
        "nearest_m": 1.5,
        "configured_port": "/dev/ttyUSB1",
        "python": "/home/robot/you/.venv/bin/python",
        "pyserial": "3.5",
        "detected": {"port": "/dev/ttyUSB1", "protocol": "lds01",
                     "baud": 230400, "motor": "none"},
        "reason": "скан идёт: serial:ttyUSB1",
        "hint": "",
        "usb_diagnostics": None,
        "last_error": "",
        "candidates": ["/dev/ttyUSB1"],
        "tried": ["/dev/ttyUSB1"],
    }
    status.update(overrides)
    return status


def state_payload(points: list[dict], status: dict) -> dict:
    """Ответ экрана: телеметрия + скан + состояние лидара."""
    return {
        "ok": True,
        "data": {
            "ts": 0,
            "linkOk": True,
            "source": "sim",
            "mode": None,
            "speedMps": 0.0,
            "powerKw": 0.08,
            "steeringControl": {"available": False, "source": "none"},
            "battery": {"soc": 80, "volts": 40.1, "amps": 2.0, "watts": 80,
                        "remainingWh": 500, "rangeKm": 22, "state": "разряд",
                        "level": "НОРМА", "tempC": 26, "cellMin": 3.3, "cellMax": 3.4,
                        "thresholds": {"lowV": 35.5, "criticalV": 33.5}},
            "motors": [{"id": name, "title": name, "angle": 0.0, "rpm": 0.0,
                        "temp": 30.0, "homed": True, "online": True}
                       for name in ("FL", "FR", "RL", "RR")],
            "lidar": points,
            "lidarDemo": bool(status.get("demo")),
            "lidarStatus": status,
        },
    }


def real_payload(points_count: int = 360, **status_over) -> dict:
    return state_payload(scan_points(points_count),
                         lidar_status(points_count=points_count, **status_over))


def demo_payload(points_count: int = 180) -> dict:
    """Демо-скан источника данных: точки есть, лидара нет."""
    status = lidar_status(
        connected=False, demo=True, source="no_signal", points_count=0,
        detected=None, configured_port="/dev/ttyUSB1", pyserial="3.5",
        reason="демо-скан источника данных (реального лидара нет): "
               "лидар не ответил ни на одном из 2 портов",
        hint="проверь питание и кабель лидара",
    )
    return state_payload(scan_points(points_count, radius=3.0), status)


def ros_payload(points_count: int = 360, **ros_over) -> dict:
    """Скан идёт из ROS 2: драйвер публикует /scan, USB-порт приложения нет."""
    ros = {
        "available": True,
        "connected": True,
        "topic": "/scan",
        "points": points_count,
        "hz": 9.4,
        "age_s": 0.05,
        "publishers": 1,
        "publisher_names": ["lidar_supervisor"],
        "supervisor": {"backend": "zapret_lidar", "state": "running",
                       "driver": "lds01", "driver_package": "hls_lfcd_lds_driver",
                       "driver_title": "LDS-01/LDS-02 (TurtleBot3 Burger, HLS-LFCD2)",
                       "port": "/dev/ttyUSB0", "baud": 230400, "points": points_count},
        "reason": "скан идёт из ROS 2 /scan: 360 точек, 9.4 Гц",
        "hint": "",
    }
    ros.update(ros_over)
    status = lidar_status(
        source="ros", points_count=points_count,
        detected=None, python="/home/robot/you/.venv/bin/python",
        reason=ros["reason"], hint=ros["hint"],
    )
    status["ros"] = ros
    status["source_mode"] = "ros"
    return state_payload(scan_points(points_count), status)


def empty_payload() -> dict:
    """Скана нет совсем: ни реального, ни демо."""
    status = lidar_status(
        connected=False, demo=False, source="no_signal", points_count=0,
        age_s=None, scan_at=None, nearest_m=None, detected=None,
        reason="лидар не ответил ни на одном из 2 портов",
        hint="проверь кабель и питание лидара",
        usb_diagnostics={
            "summary": "USB/serial-порты доступны",
            "ports": [{"port": "/dev/serial/by-id/usb-CP2102", "device": "/dev/ttyUSB1",
                       "accessible": True, "excluded": False},
                      {"port": "/dev/ttyUSB0", "device": "/dev/ttyUSB0",
                       "accessible": False, "excluded": True}],
        },
    )
    return state_payload([], status)


# ---------------------------------------------------------------------------
# Прогон экранного скрипта
# ---------------------------------------------------------------------------


def ports_overview_payload() -> dict:
    """Блок ``lidarStatus.ports`` из robot_control/lidar.py: ports_overview()."""
    return {
        "ports": [
            {"port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102-if00-port0", "device": "/dev/ttyUSB1",
             "by_id": ["/dev/serial/by-id/usb-Silicon_Labs_CP2102-if00-port0"],
             "by_path": ["/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0"],
             "accessible": True, "excluded": False, "excluded_by": None, "chassis_module": False,
             "candidate": True, "note": "нет данных", "role": "no_response",
             "role_label": "не отвечает", "why": "порт открылся, но лидар не ответил ни по одному протоколу"},
            {"port": "/dev/ttyUSB0", "device": "/dev/ttyUSB0", "by_id": [],
             "by_path": ["/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0"],
             "accessible": True, "excluded": True, "excluded_by": "RC_SLAM_MASTER_PORT",
             "chassis_module": False, "candidate": False, "note": "", "role": "master",
             "role_label": "мастер руля", "why": "исключён настройкой RC_SLAM_MASTER_PORT — лидар этот порт не открывает"},
            {"port": "/dev/ttyACM0", "device": "/dev/ttyACM0", "by_id": [], "by_path": [],
             "accessible": True, "excluded": False, "excluded_by": None, "chassis_module": True,
             "candidate": False, "note": "модуль шасси", "role": "chassis_module",
             "role_label": "модуль шасси", "why": "источник данных шасси подтвердил здесь телеметрию модулей — лидар этот порт не перебирает"},
            {"port": "/dev/ttyUSB2", "device": "/dev/ttyUSB2", "by_id": [], "by_path": [],
             "accessible": True, "excluded": False, "excluded_by": None, "chassis_module": False,
             "candidate": True, "note": "занят", "role": "busy",
             "role_label": "занят другим процессом", "why": "порт открыт другой программой"},
        ],
        "usb_devices": [],
        "find": {"active": False, "started_at": None, "elapsed_s": None, "expires_in_s": 0.0,
                 "stopped_reason": "", "baseline": [], "events": [], "highlight": None,
                 "highlight_kind": None},
        "place_note": "",
    }


def ports_payload(**overview_overrides) -> dict:
    """Скана нет, но сервер прислал блок «Порты на роботе»."""
    payload = empty_payload()
    overview = ports_overview_payload()
    overview.update(overview_overrides)
    payload["data"]["lidarStatus"]["ports"] = overview
    return payload


def render(screen: str, payload: dict, tmp_path: Path, *, demo: bool = False,
           click_find: bool = False) -> dict:
    """Прогоняет ``slam_gui/<screen>.js`` на заготовленном ответе API.

    ``demo=True`` — тот же экран с явно включённым демо-режимом (``?demo=1``):
    только в нём панель логов имеет право рисовать выдуманные строки.
    """
    assert screen in SCREENS
    payload_file = tmp_path / ("payload_%s.json" % screen)
    payload_file.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    env = dict(os.environ)
    if demo:
        env["HARNESS_DEMO"] = "1"
    if click_find:
        env["HARNESS_CLICK_FIND"] = "1"
    result = subprocess.run(
        [NODE, str(HARNESS), screen, str(payload_file)],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120, env=env,
    )
    assert result.returncode == 0, \
        "экран %s упал:\n%s\n%s" % (screen, result.stdout, result.stderr)
    report = json.loads(result.stdout)
    report["_stderr"] = result.stderr
    return report


# ---------------------------------------------------------------------------
# Реальный скан
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("screen", SCREENS)
def test_real_scan_is_drawn(screen, tmp_path):
    """Реальные точки доехали до экрана и нарисованы на карте."""
    report = render(screen, real_payload(), tmp_path)
    assert report["dots"] > 0, report
    assert report["count"] == "360"
    assert report["emptyHidden"] is True, "плашка «нет скана» висит над живыми данными"
    assert report["renderErrors"] == [], report["renderErrors"]


def test_kiosk_map_does_not_clamp_far_points_to_outer_ring(tmp_path):
    """Точки за 10 м не должны собираться в искусственную окружность."""
    points = [{"a": 0.0, "r": 12.0}, {"a": 1.57, "r": 2.0}]
    payload = state_payload(points, lidar_status(points_count=2, nearest_m=2.0))
    report = render("robot", payload, tmp_path)
    assert report["count"] == "2"  # полный скан остаётся виден в счётчике
    assert report["dots"] == 1     # на карту попала только точка в пределах 10 м
    assert "на карте: 1/2" in report["meta"]


@pytest.mark.parametrize("screen", SCREENS)
def test_scan_from_ros_is_drawn_and_signed_as_ros(screen, tmp_path):
    """Скан из ROS 2 рисуется, а подпись говорит «ROS 2», топик и драйвер.

    Раньше в подписи был только USB-порт, поэтому на роботе (где скан отдаёт
    драйвер ROS 2) выглядело так, будто лидар читает само приложение, и в
    диагностике искали не то.
    """
    report = render(screen, ros_payload(360), tmp_path)
    assert report["dots"] > 0, report
    assert report["count"] == "360"
    assert report["emptyHidden"] is True
    assert report["renderErrors"] == [], report["renderErrors"]
    meta = report["meta"]
    assert "ROS 2" in meta, meta
    assert "/scan" in meta
    assert "hls_lfcd_lds_driver" in meta
    assert "9.4" in meta


def test_ros_without_scan_shows_the_driver_reason_on_the_console(tmp_path):
    """ROS 2 есть, драйвер не отдал скан — на пульте причина и команда запуска.

    Бортовой экран намеренно держит статичную плашку (диагностика живёт в панели
    «Лидар» и на пульте), поэтому проверяем пульт: там оператор и должен увидеть,
    что дело не в кабеле, а в незапущенном драйвере ROS 2.
    """
    ros = {"available": True, "connected": False, "points": 0, "hz": 0.0, "age_s": None,
           "publishers": 0, "supervisor": {"backend": "zapret_lidar", "state": "failed",
                                           "reason": "ни один драйвер не отдал /scan",
                                           "driver": "lds01", "port": "/dev/ttyUSB0"},
           "reason": "ROS 2 есть, но /scan никто не публикует — драйвер лидара не запущен",
           "hint": "запусти драйвер лидара: ./scripts/ros2_lidar.sh"}
    status = lidar_status(connected=False, source="no_signal", points_count=0, age_s=None,
                          detected=None, reason=ros["reason"], hint=ros["hint"])
    status["ros"] = ros
    status["source_mode"] = "ros"
    report = render("main", state_payload([], status), tmp_path)
    assert report["dots"] == 0
    assert report["emptyHidden"] is False
    text = (report.get("emptyText") or "") + " " + (report.get("title") or "")
    assert "ros2_lidar.sh" in text, report
    assert "ROS 2" in text, report


@pytest.mark.parametrize("screen", SCREENS)
def test_real_scan_signature_shows_port_protocol_and_baud(screen, tmp_path):
    """Оператор видит, что это именно лидар: порт, протокол, скорость."""
    report = render(screen, real_payload(), tmp_path)
    meta = report["meta"]
    assert "в реальном времени" in meta
    assert "360 точек" in meta
    assert "ttyUSB1" in meta
    assert "lds01" in meta
    assert "230400" in meta
    assert "демо" not in meta and "не лидар" not in meta


@pytest.mark.parametrize("screen,expected", [
    ("main", "скан от второго процесса"),
    ("robot", "скан от пульта"),
])
def test_peer_scan_is_marked(screen, expected, tmp_path):
    """Скан взят у второго процесса робота — это штатно, и так и подписано."""
    payload = real_payload(source="peer", reason="скан берётся у второго процесса робота")
    report = render(screen, payload, tmp_path)
    assert report["dots"] > 0
    assert report["emptyHidden"] is True
    assert expected in report["meta"]
    assert report["renderErrors"] == []


def test_stale_scan_is_not_shown_as_live(tmp_path):
    """Скан протух (лидар отвалился) — экран не врёт про «реальное время»."""
    payload = real_payload(connected=False, source="no_signal", demo=False,
                           points_count=0, age_s=12.0, detected=None,
                           reason="скан устарел: лидар отвалился")
    # Бортовой (kiosk 8090) — только статичная плашка без причины (диагностика — на вебке 8080)
    report = render("robot", payload, tmp_path)
    assert "в реальном времени" not in report["meta"]
    assert report["emptyHidden"] is False
    assert report["emptyText"] == "скана лидара нет"
    # Веб (8080) — показывает причину
    report_main = render("main", payload, tmp_path)
    assert "устарел" in report_main["emptyText"]


# ---------------------------------------------------------------------------
# Демо-скан не выдаётся за лидар
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("screen", SCREENS)
def test_demo_scan_is_labelled_as_demo(screen, tmp_path):
    report = render(screen, demo_payload(), tmp_path)
    meta = report["meta"]
    assert "демо" in meta
    assert "не лидар" in meta
    assert "в реальном времени" not in meta
    # точки демо рисуются, но причина отсутствия лидара остаётся на экране
    assert report["emptyHidden"] is False
    if screen == "robot":
        # Бортовой — статичная плашка без текста ошибки (диагностика только на вебке)
        assert report["emptyText"] == "скана лидара нет"
    else:
        assert "демо-скан" in report["emptyText"]
    assert report["renderErrors"] == []


@pytest.mark.parametrize("screen", SCREENS)
def test_demo_flag_in_status_overrides_points(screen, tmp_path):
    """Даже если точки пришли, ``demo`` в состоянии обязывает подписать демо."""
    payload = state_payload(scan_points(120), lidar_status(
        demo=True, connected=False, source="sim", points_count=120, detected=None,
        reason="демо-скан источника данных (реального лидара нет)"))
    report = render(screen, payload, tmp_path)
    assert "демо" in report["meta"] and "не лидар" in report["meta"]
    assert report["emptyHidden"] is False


# ---------------------------------------------------------------------------
# Скана нет
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("screen", SCREENS)
def test_missing_scan_shows_reason_and_usb_diagnostics(screen, tmp_path):
    report = render(screen, empty_payload(), tmp_path)
    assert report["dots"] == 0
    assert report["count"] == "—"
    assert report["emptyHidden"] is False
    text = report["emptyText"]
    if screen == "robot":
        # Бортовой (kiosk 8090) — только статичная плашка, без причины и USB-диагностики
        assert text == "скана лидара нет"
    else:
        assert "лидар не ответил" in text
        # USB-диагностика из API видна оператору: порты, права, исключения
        assert "/dev/serial/by-id/usb-CP2102" in text and "(доступен)" in text
        assert "нет доступа — нужна группа dialout" in text      # /dev/ttyUSB0
        assert "исключён настройками" in text                    # и то и другое сразу
    assert report["renderErrors"] == []


def test_kiosk_hides_telemetry_until_real_lidar_scan(tmp_path):
    working = render("robot", real_payload(), tmp_path)
    assert working["lidarBlocked"] is False

    missing = render("robot", empty_payload(), tmp_path)
    assert missing["lidarBlocked"] is True
    assert "лидар не ответил ни на одном из 2 портов" in missing["lidarBlockReason"]
    assert "проверь кабель и питание лидара" in missing["lidarBlockHint"]
    assert "USB/serial-порты доступны" in missing["lidarBlockDetails"]


def test_kiosk_does_not_treat_demo_scan_as_working_lidar(tmp_path):
    report = render("robot", demo_payload(), tmp_path)
    assert report["lidarBlocked"] is True
    assert "реального лидара нет" in report["lidarBlockReason"]


@pytest.mark.parametrize("screen", SCREENS)
def test_missing_scan_meta_text(screen, tmp_path):
    report = render(screen, empty_payload(), tmp_path)
    assert report["meta"] == ("лидар: нет данных" if screen == "main" else "нет скана")


# ---------------------------------------------------------------------------
# Панель «Логи»: настоящий журнал робота, а не выдуманный поток
# ---------------------------------------------------------------------------


def journal_payload(entries: list[dict]) -> dict:
    """Ответ /api/kiosk_logs: записи настоящего журнала процесса."""
    levels = {"info": 0, "ok": 0, "warn": 0, "err": 0, "debug": 0}
    for entry in entries:
        levels[entry["level"]] = levels.get(entry["level"], 0) + 1
    return {
        "ok": True,
        "data": {
            "entries": entries,
            "count": len(entries),
            "shown": len(entries),
            "dropped": 3,
            "limit": 400,
            "levels": levels,
            "uptimeSec": 128,
            "source": "master",
            "demo": False,
            "journal": "robot",
        },
    }


def journal_entry(seq: int, level: str, msg: str, *, source: str = "",
                  ts: float = 1_800_000_000.0, repeat: int = 1) -> dict:
    return {"seq": seq, "ts": ts + seq, "time": "12:00:%02d" % seq, "level": level,
            "source": source, "msg": msg, "repeat": repeat}


def test_log_panel_shows_real_journal_lines(tmp_path):
    """Оператор видит настоящие события робота: лидар, отказ команды, PIN."""
    payload = real_payload()
    payload["logs"] = journal_payload([
        journal_entry(1, "ok", "лидар на связи (serial:ttyUSB1, 360 точек)", source="лидар"),
        journal_entry(2, "warn", "руление отклонено: угол вне допуска ±85°", source="управление"),
        journal_entry(3, "warn", "PIN не принят (бортовой экран): неверный PIN",
                      source="pin", repeat=4),
    ])
    report = render("robot", payload, tmp_path)
    lines = report["logLines"]
    assert report["renderErrors"] == [], report["renderErrors"]
    assert [line["msg"].split("  ×")[0] for line in lines] == [
        "лидар на связи (serial:ttyUSB1, 360 точек)",
        "руление отклонено: угол вне допуска ±85°",
        "PIN не принят (бортовой экран): неверный PIN",
    ]
    assert [line["level"] for line in lines] == ["ok", "warn", "warn"]
    assert lines[0]["time"] == "12:00:01"
    # повторяющееся событие подписано счётчиком, а не размножено строками
    assert lines[2]["msg"].endswith("×4")
    # подпись под журналом — про настоящий журнал, с числом записей и ошибок
    status = report["logStatus"]
    assert "журнал робота" in status and "3 записи" in status
    assert "вытеснено 3" in status and "источник: master" in status


def test_log_panel_does_not_invent_lines(tmp_path):
    """Журнал пуст — панель молчит о событиях, а не рисует демо-поток."""
    report = render("robot", real_payload(), tmp_path)
    assert report["logLines"] == [], report["logLines"]
    assert "журнал робота" in report["logStatus"]
    assert "демо" not in report["logStatus"]
    assert report["renderErrors"] == []


def test_demo_logs_only_when_explicitly_asked(tmp_path):
    """Демо-строки появляются только по явному «?demo=1» и подписаны как демо."""
    report = render("robot", real_payload(), tmp_path, demo=True)
    assert report["logLines"], "демо-режим не показал ни одной строки"
    assert "демо-строки включены" in report["logStatus"]
    assert all("демо" in line["msg"] for line in report["logLines"]), report["logLines"]
    assert report["renderErrors"] == []


def test_screen_survives_payload_without_lidar(tmp_path):
    """В ответе вообще нет ключа lidar: экран жив, карта пустая, цикл не встал."""
    payload = real_payload()
    del payload["data"]["lidar"]
    del payload["data"]["lidarStatus"]
    report = render("robot", payload, tmp_path)
    assert report["dots"] == 0
    assert report["emptyHidden"] is False
    assert report["meta"] == "нет скана"
    assert report["_stderr"] == ""


# ---------------------------------------------------------------------------
# Блок «Порты на роботе»: где физически искать лидар
# ---------------------------------------------------------------------------


def test_kiosk_lists_ports_with_roles_and_physical_paths(tmp_path):
    """Каждый порт — с ролью, причиной исключения и путём by-path (разъём/хаб)."""
    report = render("robot", ports_payload(), tmp_path)
    assert report["lidarBlocked"] is True
    rows = {row["text"].split(" ")[0]: row for row in report["lidarPorts"]}
    assert set(rows) == {"/dev/ttyUSB1", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB2"}
    assert "не отвечает" in rows["/dev/ttyUSB1"]["text"]
    assert "usb-0:1.3:1.0-port0" in rows["/dev/ttyUSB1"]["text"]          # by-path
    assert "usb-Silicon_Labs_CP2102" in rows["/dev/ttyUSB1"]["text"]      # by-id
    assert "мастер руля" in rows["/dev/ttyUSB0"]["text"]
    assert "RC_SLAM_MASTER_PORT" in rows["/dev/ttyUSB0"]["text"]          # чем исключён
    assert "модуль шасси" in rows["/dev/ttyACM0"]["text"]
    assert "by-path: нет" in rows["/dev/ttyACM0"]["text"]                 # нет данных — так и написано
    assert "занят другим процессом" in rows["/dev/ttyUSB2"]["text"]
    assert "role-master" in rows["/dev/ttyUSB0"]["className"]
    assert report["lidarPortsEmpty"] == ""
    assert "Найти лидар" in report["lidarFindButton"]
    assert report["renderErrors"] == []


def test_kiosk_highlights_the_port_that_changed_in_find_mode(tmp_path):
    """Режим «найти лидар»: порт, который появился при подключении кабеля, подсвечен."""
    find = {"active": True, "started_at": 1_800_000_000.0, "elapsed_s": 12.0, "expires_in_s": 168.0,
            "stopped_reason": "", "baseline": ["/dev/ttyUSB0", "/dev/ttyUSB1"],
            "events": [{"kind": "disappeared", "device": "/dev/ttyUSB1", "at": 1_800_000_005.0,
                        "by_id": [], "by_path": ["/dev/serial/by-path/pci-usb-0:1.3:1.0-port0"]},
                       {"kind": "appeared", "device": "/dev/ttyUSB1", "at": 1_800_000_010.0,
                        "by_id": [], "by_path": ["/dev/serial/by-path/pci-usb-0:1.3:1.0-port0"]}],
            "highlight": "/dev/ttyUSB1", "highlight_kind": "appeared"}
    report = render("robot", ports_payload(find=find), tmp_path)
    lit = [row for row in report["lidarPorts"] if "is-highlight" in row["className"]]
    assert [row["text"].split(" ")[0] for row in lit] == ["/dev/ttyUSB1"]
    assert "появился" in lit[0]["text"]
    assert "Режим «найти лидар» включён" in report["lidarFindStatus"]
    assert "переподключ" in report["lidarFindStatus"] or "подключи кабель" in report["lidarFindStatus"]
    assert "Остановить поиск" in report["lidarFindButton"]
    assert report["renderErrors"] == []


def test_kiosk_shows_disappeared_port_even_when_it_is_gone(tmp_path):
    """Кабель отключили: порта в списке нет, но он показан как исчезнувший с его by-path."""
    overview = ports_overview_payload()
    overview["ports"] = [p for p in overview["ports"] if p["device"] != "/dev/ttyUSB1"]
    overview["find"] = {"active": True, "started_at": 1.0, "elapsed_s": 3.0, "expires_in_s": 177.0,
                        "stopped_reason": "", "baseline": [],
                        "events": [{"kind": "disappeared", "device": "/dev/ttyUSB1", "at": 2.0,
                                    "by_id": [], "by_path": ["/dev/serial/by-path/pci-usb-0:1.3:1.0-port0"]}],
                        "highlight": "/dev/ttyUSB1", "highlight_kind": "disappeared"}
    report = render("robot", ports_payload(**overview), tmp_path)
    gone = [row for row in report["lidarPorts"] if row["text"].startswith("/dev/ttyUSB1")]
    assert len(gone) == 1
    assert "исчез" in gone[0]["text"] and "usb-0:1.3:1.0-port0" in gone[0]["text"]
    assert "is-highlight" in gone[0]["className"]


def test_kiosk_find_button_asks_the_server_to_watch_ports(tmp_path):
    report = render("robot", ports_payload(), tmp_path, click_find=True)
    posts = [c for c in report["fetchCalls"] if c["method"] == "POST" and "lidar/find" in c["url"]]
    assert len(posts) == 1
    assert json.loads(posts[0]["body"]) == {"action": "start"}


def test_kiosk_without_ports_tells_what_to_check(tmp_path):
    """Портов нет совсем — чек-лист (кабель, питание, переходник, lsusb), без выдумок."""
    report = render("robot", ports_payload(ports=[], usb_devices=[]), tmp_path)
    assert report["lidarPorts"] == []
    text = report["lidarPortsEmpty"]
    for word in ("кабель", "итание", "переходник", "lsusb"):
        assert word in text, word
    assert "ядро не видит" in text

    seen = render("robot", ports_payload(ports=[], usb_devices=[
        {"bus_path": "1-1.3", "id": "10c4:ea60", "manufacturer": "Silicon Labs", "product": "CP2102 USB to UART"}]), tmp_path)
    assert "10c4:ea60" in seen["lidarPortsEmpty"] and "драйвер" in seen["lidarPortsEmpty"]


def test_kiosk_says_so_when_server_has_no_ports_block(tmp_path):
    """Старый сервер без поля ports: экран не рисует демо-список, а пишет, что данных нет."""
    report = render("robot", empty_payload(), tmp_path)
    assert report["lidarPorts"] == []
    assert "не передал список портов" in report["lidarPortsEmpty"]
    assert report["renderErrors"] == []


def test_kiosk_shows_human_place_note_only_when_given(tmp_path):
    assert render("robot", ports_payload(), tmp_path)["lidarPlaceNote"] == ""
    report = render("robot", ports_payload(place_note="разъём 1.3 — хаб под крышкой, лидар сверху"), tmp_path)
    assert "хаб под крышкой" in report["lidarPlaceNote"]


def test_console_names_port_roles_and_paths(tmp_path):
    """Пульт (main.js): роли и by-path тех же портов — в причине под картой."""
    report = render("main", ports_payload(), tmp_path)
    text = report["emptyText"]
    assert "порты на роботе" in text
    assert "/dev/ttyUSB0 [мастер руля, RC_SLAM_MASTER_PORT" in text
    assert "by-path platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0" in text
    assert "модуль шасси" in text and "занят другим процессом" in text
    assert report["renderErrors"] == []
