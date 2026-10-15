"""Лидар помнит себя между перезапусками: data/lidar-found.json.

На роботе лидар находился перебором всех USB-портов и всех скоростей, а после
перезапуска приложения (или ребута машины) поиск начинался заново: десятки
секунд «скана нет», пока круг перебора снова дойдёт до нужного порта. Теперь
подтверждённый лидар запоминается (порт, протокол, скорость, мотор) и в новом
поиске проверяется **первым** — это подсказка порядка, а не жёсткая настройка:
если лидар переехал на другой USB, перебор всё равно его найдёт.

Проверки идут без железа: порядок портов — на фиктивных списках, а сама
«находка с запоминанием» — на настоящем псевдотерминале с кадрами LDS-01.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import pytest

from robot_control import lidar
from robot_control.lidar import (
    LIDAR_HINT_ROUNDS,
    LidarManager,
    found_lidar_path,
    lidar_profiles,
    load_found_lidar,
    save_found_lidar,
)

from test_lidar_serial import encode_lds01_packet, wait_for


@pytest.fixture
def found_file(tmp_path, monkeypatch):
    """Файл подсказки во временной папке: тесты не трогают data/ робота."""
    path = tmp_path / "lidar-found.json"
    monkeypatch.setenv("RC_LIDAR_FOUND_FILE", str(path))
    return path


@pytest.fixture
def manager(found_file):
    """Менеджер без фонового потока — состояние известно с самого начала."""
    previous = LidarManager._instance
    instance = LidarManager()
    instance.source_mode = "serial"
    instance.peer_urls = []
    LidarManager._instance = instance
    try:
        yield instance
    finally:
        instance.stop()
        LidarManager._instance = previous


# --------------------------------------------------------------------------- #
# Запись и чтение подсказки
# --------------------------------------------------------------------------- #


def test_save_and_load_found_lidar_roundtrip(found_file):
    assert load_found_lidar() is None  # файла ещё нет
    assert save_found_lidar({"port": "/dev/ttyUSB1", "protocol": "lds01",
                             "baud": 230400, "motor": "none", "points": 360})
    assert found_file.exists()
    hint = load_found_lidar()
    assert hint == {
        "port": "/dev/ttyUSB1",
        "protocol": "lds01",
        "baud": 230400,
        "motor": "none",
    }


def test_save_found_lidar_creates_parent_directory(tmp_path, monkeypatch):
    path = tmp_path / "deep" / "dir" / "found.json"
    monkeypatch.setenv("RC_LIDAR_FOUND_FILE", str(path))
    assert save_found_lidar({"port": "/dev/serial/by-id/usb-lidar", "protocol": "ld06",
                             "baud": 230400, "motor": "none"})
    assert json.loads(path.read_text("utf-8"))["port"] == "/dev/serial/by-id/usb-lidar"
    assert found_lidar_path() == path


@pytest.mark.parametrize("payload", [
    "",
    "[]",
    '{"baud": 230400}',
    '{"port": ""}',
    '{"port": 42}',
])
def test_load_found_lidar_ignores_broken_files(found_file, payload):
    found_file.write_text(payload, encoding="utf-8")
    assert load_found_lidar() is None


def test_load_found_lidar_keeps_port_when_baud_is_garbage(found_file):
    """Порт важнее профиля: битая скорость не отменяет подсказку «лидар был тут»."""
    found_file.write_text('{"port": "/dev/ttyUSB0", "baud": "не число"}', encoding="utf-8")
    hint = load_found_lidar()
    assert hint is not None and hint["port"] == "/dev/ttyUSB0"
    assert hint["baud"] is None


def test_load_found_lidar_never_raises_on_missing_directory(tmp_path, monkeypatch):
    monkeypatch.setenv("RC_LIDAR_FOUND_FILE", str(tmp_path / "no" / "such" / "f.json"))
    assert load_found_lidar() is None
    assert save_found_lidar({"port": "/dev/ttyUSB0"}) is True  # каталог создаётся


# --------------------------------------------------------------------------- #
# Порядок перебора: подсказка первые LIDAR_HINT_ROUNDS кругов
# --------------------------------------------------------------------------- #


def test_remembered_port_is_probed_first_then_rotation_takes_over(manager, monkeypatch):
    ports = ["/tmp/fake/ttyUSB0", "/tmp/fake/ttyUSB1", "/tmp/fake/ttyUSB2"]
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: list(ports))
    save_found_lidar({"port": "/tmp/fake/ttyUSB1", "protocol": "lds01",
                      "baud": 230400, "motor": "none"})
    manager._found_hint = load_found_lidar()
    manager._hint_budget = LIDAR_HINT_ROUNDS

    for _ in range(LIDAR_HINT_ROUNDS):
        assert manager.candidate_ports()[0] == "/tmp/fake/ttyUSB1"
    # Подсказка исчерпана: дальше работает круговой перебор, а не «вечно первый».
    assert manager._hint_budget == 0
    first_round = manager.candidate_ports()
    assert first_round[0] == ports[0]
    rotated = manager.candidate_ports()
    assert rotated != first_round  # сдвиг по кругу остался прежним


def test_sticky_port_beats_the_hint(manager, monkeypatch):
    ports = ["/tmp/fake/ttyUSB0", "/tmp/fake/ttyUSB1"]
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: list(ports))
    save_found_lidar({"port": "/tmp/fake/ttyUSB0", "protocol": "lds01",
                      "baud": 230400, "motor": "none"})
    manager._found_hint = load_found_lidar()
    manager._hint_budget = LIDAR_HINT_ROUNDS
    manager._sticky = "/tmp/fake/ttyUSB1"  # лидар уже найден в этом запуске

    assert manager.candidate_ports()[0] == "/tmp/fake/ttyUSB1"
    assert manager._hint_budget == LIDAR_HINT_ROUNDS  # бюджет не тратится


def test_hint_matching_is_by_device_realpath(manager, monkeypatch, tmp_path):
    """/dev/ttyUSB1 и /dev/serial/by-id/… — один порт, подсказка должна сработать."""
    device = tmp_path / "ttyUSB1"
    device.write_text("")
    alias = tmp_path / "by-id" / "usb-Silicon_Labs"
    alias.parent.mkdir()
    alias.symlink_to(device)
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: [str(alias), str(device)])
    save_found_lidar({"port": str(device), "protocol": "lds01", "baud": 230400})
    manager._found_hint = load_found_lidar()
    manager._hint_budget = LIDAR_HINT_ROUNDS

    assert manager._sticky_port([str(alias), str(device)]) == str(alias)


# --------------------------------------------------------------------------- #
# Профиль и мотор: подсказка первой строкой, но без фильтрации
# --------------------------------------------------------------------------- #


def test_preferred_profile_moves_pair_to_front_without_filtering():
    order = lidar_profiles("auto", preferred=("rplidar", 460800))
    assert order[0] == ("rplidar", 460800)
    assert sorted(order) == sorted(lidar_profiles("auto"))  # состав не изменился


def test_preferred_profile_is_ignored_when_operator_pinned_protocol(manager):
    save_found_lidar({"port": "/dev/ttyUSB0", "protocol": "lds01", "baud": 230400})
    manager._found_hint = load_found_lidar()
    assert manager._preferred_profile() == ("lds01", 230400)
    manager.protocol = "ld06"  # оператор закрепил протокол в .env
    assert manager._preferred_profile() is None


def test_preferred_motor_only_applies_in_auto(manager):
    save_found_lidar({"port": "/dev/ttyUSB0", "protocol": "rplidar", "baud": 115200,
                      "motor": "rts-low"})
    manager._found_hint = load_found_lidar()
    assert manager._preferred_motor() == "rts-low"
    manager.motor = "dtr-low"
    assert manager._preferred_motor() is None


def test_mark_found_persists_and_clears_hint_budget(manager, found_file):
    manager._hint_budget = LIDAR_HINT_ROUNDS
    manager._mark_found({"port": "/dev/ttyUSB3", "protocol": "lds01",
                         "baud": 230400, "motor": "none"})
    assert json.loads(found_file.read_text("utf-8"))["port"] == "/dev/ttyUSB3"
    assert manager._found_hint["port"] == "/dev/ttyUSB3"
    assert manager._hint_budget == 0
    status = manager.get_status()
    assert status["remembered"]["port"] == "/dev/ttyUSB3"


# --------------------------------------------------------------------------- #
# Стенд: настоящий pyserial + псевдотерминал с кадрами LDS-01
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="нужен POSIX-псевдотерминал")
def test_detection_on_pty_is_remembered_for_the_next_launch(monkeypatch, tmp_path):
    """Полный цикл: лидар найден → запомнен → новый менеджер идёт сразу к нему."""
    serial = pytest.importorskip("serial")
    pty = pytest.importorskip("pty")

    found_file = tmp_path / "lidar-found.json"
    monkeypatch.setenv("RC_LIDAR_FOUND_FILE", str(found_file))
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")
    monkeypatch.setenv("RC_LIDAR_PROTOCOL", "auto")
    monkeypatch.setenv("RC_LIDAR_BAUD", "")
    monkeypatch.setenv("RC_LIDAR_MOTOR", "auto")

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    stop = threading.Event()
    written = {"count": 0}

    def device():
        started = time.monotonic()
        while not stop.is_set():
            allowed = int((time.monotonic() - started) * 23040.0)  # 230400 бод
            while written["count"] < allowed:
                os.write(master, encode_lds01_packet(written["count"] % 60, 1.5))
                written["count"] += 1
            time.sleep(0.003)

    thread = threading.Thread(target=device, daemon=True)
    thread.start()
    try:
        monkeypatch.setenv("RC_LIDAR_PORT", port)
        monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")

        first = LidarManager()
        first.source_mode = "serial"
        worker = threading.Thread(target=first._serial_worker, daemon=True)
        worker.start()
        try:
            assert wait_for(lambda: first.get_scan() is not None, timeout=8.0)
            assert wait_for(lambda: found_file.exists(), timeout=2.0)
        finally:
            first.stop()
            worker.join(timeout=3.0)

        hint = load_found_lidar()
        assert hint is not None
        # Путь в подсказке — тот же tty (сравнение по realpath сделает менеджер).
        assert os.path.realpath(hint["port"]) == os.path.realpath(port)
        assert hint["protocol"] == "lds01"
        assert hint["baud"] == 230400

        # «Перезапуск»: новый менеджер без RC_LIDAR_PORT, порты — фиктивный
        # список, где живой tty далеко не первый. Подсказка ставит его первым.
        other = tmp_path / "ttyUSB9"
        other.write_text("")
        monkeypatch.delenv("RC_LIDAR_PORT", raising=False)
        monkeypatch.setattr(lidar, "discover_ports",
                            lambda **kw: [str(other), port])

        second = LidarManager()
        try:
            second._found_hint = load_found_lidar()
            second._hint_budget = LIDAR_HINT_ROUNDS
            assert second.candidate_ports()[0] == port
        finally:
            second.stop()
    finally:
        stop.set()
        thread.join(timeout=2.0)
        os.close(slave)
        os.close(master)
