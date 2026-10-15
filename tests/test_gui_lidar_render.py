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


def render(screen: str, payload: dict, tmp_path: Path) -> dict:
    """Прогоняет ``slam_gui/<screen>.js`` на заготовленном ответе API."""
    assert screen in SCREENS
    payload_file = tmp_path / ("payload_%s.json" % screen)
    payload_file.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    result = subprocess.run(
        [NODE, str(HARNESS), screen, str(payload_file)],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120,
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
    report = render("robot", payload, tmp_path)
    assert "в реальном времени" not in report["meta"]
    assert report["emptyHidden"] is False
    assert "устарел" in report["emptyText"]


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
    assert "лидар не ответил" in text
    # USB-диагностика из API видна оператору: порты, права, исключения
    assert "/dev/serial/by-id/usb-CP2102" in text and "(доступен)" in text
    assert "нет доступа — нужна группа dialout" in text      # /dev/ttyUSB0
    assert "исключён настройками" in text                    # и то и другое сразу
    assert report["renderErrors"] == []


@pytest.mark.parametrize("screen", SCREENS)
def test_missing_scan_meta_text(screen, tmp_path):
    report = render(screen, empty_payload(), tmp_path)
    assert report["meta"] == ("лидар: нет данных" if screen == "main" else "нет скана")


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
