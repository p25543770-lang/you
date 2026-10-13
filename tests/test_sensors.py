"""Тесты чтения данных лидара и работы с веб-камерами."""

from __future__ import annotations

import io
import json
import math
import time

import pytest

from robot_control.camera import CameraManager
from robot_control.lidar import (
    LIDAR_MAX_AGE_S,
    LidarManager,
    generate_demo_scan,
    normalize_angle,
    parse_ld06_packet,
    parse_rplidar_point,
    scan_from_ranges,
)


def test_normalize_angle():
    assert abs(normalize_angle(0.0)) < 1e-6
    assert abs(normalize_angle(math.pi * 2)) < 1e-6
    assert abs(normalize_angle(math.pi)) <= math.pi
    assert -math.pi <= normalize_angle(-math.pi * 3) <= math.pi


def test_scan_from_ranges_filtering():
    ranges = [1.0, float("inf"), float("nan"), 0.01, 5.0, 15.0]
    pts = scan_from_ranges(ranges, angle_min=0.0, angle_max=math.pi, angle_increment=0.5, range_min=0.1, range_max=10.0)
    assert len(pts) == 2
    assert pts[0]["r"] == 1.0
    assert pts[1]["r"] == 5.0


def test_parse_rplidar_point():
    # Валидный 5-байтовый пакет:
    # b0: sync=1, inv_sync=0, quality=10 -> (10 << 2) | 1 = 41
    # b1: angle_q6[0:6] << 1 | check_bit(1)
    # b2: angle_q6[7:14]
    # b3: distance_q2_lo
    # b4: distance_q2_hi
    # Угол 90° = 90 * 64 = 5760 = 0x1680. b1: (0x80 << 1) & 0xFE | 1 = 1, b2: 0x1680 >> 7 = 0x2D = 45
    # Дистанция 2000 мм = 2000 * 4 = 8000 = 0x1F40. b3 = 0x40, b4 = 0x1F
    b = bytes([41, 1, 45, 0x40, 0x1F])
    pt = parse_rplidar_point(b)
    assert pt is not None
    assert pt["r"] == 2.0
    # Проверка угла: нормализован
    assert -math.pi <= pt["a"] <= math.pi


def test_parse_ld06_packet():
    # 47 байт: 0x54, 0x2C, speed, start_angle, 12 points, end_angle, ts, crc
    pkt = bytearray(47)
    pkt[0] = 0x54
    pkt[1] = 0x2C
    pkt[4] = 0x00  # start_angle = 0
    pkt[5] = 0x00
    pkt[42] = 0x60  # end_angle = 3000 (30.00 deg)
    pkt[43] = 0x0B
    for i in range(12):
        idx = 6 + i * 3
        pkt[idx] = 0xE8  # dist = 1000 мм
        pkt[idx + 1] = 0x03
        pkt[idx + 2] = 200  # confidence
    pts = parse_ld06_packet(bytes(pkt))
    assert len(pts) == 12
    assert pts[0]["r"] == 1.0


@pytest.fixture
def kiosk_client(config):
    from robot_control.kiosk_app import create_kiosk_app

    application = create_kiosk_app(config)
    application.config.update(TESTING=True)
    return application.test_client()


def test_generate_demo_scan():
    pts = generate_demo_scan()
    assert len(pts) == 180
    assert all("a" in p and "r" in p for p in pts)
    assert all(-math.pi - 1e-3 <= p["a"] <= math.pi + 1e-3 for p in pts)
    assert all(0.1 <= p["r"] <= 12.0 for p in pts)


def test_lidar_manager_feed_and_expiration():
    mgr = LidarManager.get_instance()
    # Загружаем точки
    test_points = [{"a": 0.0, "r": 1.5}, {"a": 1.57, "r": 2.5}]
    count = mgr.feed_points(test_points, source_label="unit_test")
    assert count == 2
    scan = mgr.get_scan(max_age=1.0)
    assert scan == [{"a": 0.0, "r": 1.5}, {"a": 1.57, "r": 2.5}]

    status = mgr.get_status()
    assert status["connected"] is True
    assert status["points_count"] == 2
    assert status["nearest_m"] == 1.5

    # Истечение по таймауту
    mgr.scan_at = time.time() - 2.0
    assert mgr.get_scan(max_age=1.0) is None
    status_exp = mgr.get_status()
    assert status_exp["connected"] is False


def test_camera_manager_feed_and_get():
    cam_mgr = CameraManager.get_instance()
    cameras = cam_mgr.list_cameras()
    assert len(cameras) >= 2
    assert cameras[0]["id"] == 0
    assert cameras[1]["id"] == 1

    # Загружаем тестовый JPEG кадр
    fake_jpeg = b"\xFF\xD8\xFF\xE0\x00\x10JFIF\x00\x01\x01\x01\x00`\x00`\x00\x00\xFF\xDB"
    cam_mgr.feed_frame(0, fake_jpeg)
    frame = cam_mgr.get_frame(0, max_age_s=2.0)
    assert frame == fake_jpeg
    assert cam_mgr.get_camera(0).is_active is True


def test_slam_lidar_and_camera_api(client):
    from conftest import login

    login(client)

    # 1. POST /api/lidar с точками
    res = client.post(
        "/api/lidar",
        data=json.dumps({"points": [{"a": 0.5, "r": 3.2}]}),
        content_type="application/json",
    )
    assert res.status_code == 200
    assert res.get_json()["count"] == 1

    # 2. GET /api/lidar
    get_res = client.get("/api/lidar")
    assert get_res.status_code == 200
    data = get_res.get_json()
    assert data["ok"] is True
    assert data["count"] == 1

    # 3. GET /api/cameras
    cams_res = client.get("/api/cameras")
    assert cams_res.status_code == 200
    assert len(cams_res.get_json()["cameras"]) >= 2

    # 4. POST /api/camera/0/frame и GET /api/camera/0/frame
    fake_frame = b"\xFF\xD8\xFF\xE0dummy_frame_data"
    p_res = client.post("/api/camera/0/frame", data=fake_frame, content_type="image/jpeg")
    assert p_res.status_code == 200

    g_res = client.get("/api/camera/0/frame")
    assert g_res.status_code == 200
    assert g_res.data == fake_frame
    assert g_res.headers["Content-Type"] == "image/jpeg"


def test_kiosk_lidar_and_camera_api(kiosk_client):
    # Kiosk сервер на 8090 доступен без логина
    # Проверка /api/lidar
    res = kiosk_client.post(
        "/api/lidar",
        data=json.dumps({"points": [{"a": -0.2, "r": 1.8}]}),
        content_type="application/json",
    )
    assert res.status_code == 200
    assert res.get_json()["count"] == 1

    # В телеметрии /api/kiosk_state лидар подхватывается
    state_res = kiosk_client.get("/api/kiosk_state")
    assert state_res.status_code == 200
    body = state_res.get_json()
    assert body["data"]["lidar"] == [{"a": -0.2, "r": 1.8}]

    # Проверка камер на киоске
    cams_res = kiosk_client.get("/api/cameras")
    assert cams_res.status_code == 200
    assert len(cams_res.get_json()["cameras"]) >= 2
