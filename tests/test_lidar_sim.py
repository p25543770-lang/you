"""Симулятор USB-лидара (scripts/lidar_sim.py): кадры и стенд без железа.

Скрипт нужен, чтобы проверять весь путь «кабель → скан → экран» без настоящего
лидара: он сыплет в pty настоящие кадры LDS-01/LD06/RPLIDAR — те же, которые
понимает приложение. Здесь проверяется, что кодировщики симулятора обратны
парсерам лидара (кадры не «мусор») и что менеджер лидара находит симулятор
ровно так же, как настоящее устройство.
"""

from __future__ import annotations

import os
import threading
import time

import pytest

from robot_control.lidar import (
    LidarManager,
    parse_ld06_packet,
    parse_lds01_packet,
    parse_rplidar_point,
    ld06_crc_ok,
    lds01_packet_crc_ok,
)
from robot_control import lidar as lidar_mod

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import lidar_sim  # noqa: E402  (scripts/lidar_sim.py)

from test_lidar_serial import wait_for  # noqa: E402


# --------------------------------------------------------------------------- #
# Кодировщики симулятора обратны парсерам приложения
# --------------------------------------------------------------------------- #


def test_lds01_frames_roundtrip_through_the_parser():
    pkt = lidar_sim.encode_lds01_packet(17, 1.5)
    assert pkt[0] == 0xFA and pkt[1] == 0xA0 + 17
    assert lds01_packet_crc_ok(pkt)
    points = parse_lds01_packet(pkt)
    assert len(points) == 6
    # Углы пакета 17: 359 − (17·6 + i); дальности из «комнаты» рядом с базовой.
    angles_deg = sorted(round(__import__("math").degrees(p["a"]), 1) for p in points)
    assert all(0.25 <= p["r"] <= 8.0 for p in points)
    assert any(abs(a) > 50 for a in angles_deg)


def test_ld06_frames_roundtrip_through_the_parser():
    pkt = lidar_sim.encode_ld06_packet(36.0, 1.5)
    assert pkt[0] == 0x54 and pkt[1] == 0x2C
    assert ld06_crc_ok(pkt)
    points = parse_ld06_packet(pkt)
    assert len(points) == 12
    assert all(0.25 <= p["r"] <= 8.0 for p in points)


def test_rplidar_nodes_roundtrip_through_the_parser():
    node = lidar_sim.encode_rplidar_node(90.0, 1.25, start=True)
    point = parse_rplidar_point(node)
    assert point is not None
    assert point["r"] == pytest.approx(1.25, abs=0.01)
    assert abs(__import__("math").degrees(point["a"])) == pytest.approx(90.0, abs=0.1)


# --------------------------------------------------------------------------- #
# Стенд: симулятор на pty + настоящий менеджер лидара
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="нужен POSIX-псевдотерминал")
def test_manager_reads_the_simulated_lds01(monkeypatch):
    """Как на роботе: менеджер открывает порт симулятора и собирает скан."""
    pytest.importorskip("serial")

    master, slave = os.openpty()
    os.set_blocking(master, False)
    port = os.ttyname(slave)
    sim = lidar_sim.Simulator(master, "lds01", 230400, 1.5)
    stop = threading.Event()

    def device():
        while not stop.is_set():
            sim.step()
            time.sleep(0.004)

    thread = threading.Thread(target=device, daemon=True)
    thread.start()
    try:
        monkeypatch.setenv("RC_LIDAR_PORT", port)
        monkeypatch.setenv("RC_LIDAR_PROTOCOL", "auto")
        monkeypatch.setenv("RC_LIDAR_BAUD", "")
        monkeypatch.setenv("RC_LIDAR_MOTOR", "auto")
        monkeypatch.setenv("RC_LIDAR_SOURCE", "serial")
        monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")

        manager = LidarManager()
        manager.source_mode = "serial"
        worker = threading.Thread(target=manager._serial_worker, daemon=True)
        worker.start()
        try:
            assert wait_for(lambda: manager.get_scan() is not None, timeout=8.0)
            scan = manager.get_scan() or []
            assert len(scan) >= 100  # виток симулятора: ~360 точек
            status = manager.get_status()
            assert status["connected"] is True
            assert status["detected"]["protocol"] == "lds01"
            assert status["detected"]["baud"] == 230400
        finally:
            manager.stop()
            worker.join(timeout=3.0)
    finally:
        stop.set()
        thread.join(timeout=2.0)
        os.close(slave)
        os.close(master)


def test_simulator_respects_commands_of_the_app():
    """RPLIDAR отвечает на SCAN и останавливается по STOP; LDS-01 — по b/e."""
    master, slave = os.openpty()
    os.set_blocking(master, False)
    try:
        sim = lidar_sim.Simulator(master, "rplidar", 115200, 1.5)
        assert sim.scanning is False
        sim.feed_app_write(b"\xa5\x20")            # SCAN
        assert sim.scanning is True
        assert bytes(sim._pending).startswith(lidar_sim.RPLIDAR_ANSWER_DESCRIPTOR)
        sim.feed_app_write(b"\xa5\x25")            # STOP
        assert sim.scanning is False

        sim = lidar_sim.Simulator(master, "lds01", 230400, 1.5)
        assert sim.scanning is True               # поток идёт сразу
        sim.feed_app_write(b"e")
        assert sim.scanning is False
        sim.feed_app_write(b"b")
        assert sim.scanning is True
    finally:
        os.close(slave)
        os.close(master)
