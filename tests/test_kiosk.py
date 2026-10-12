"""Бортовой экран робота (киоск): страница на 8090, открытие экрана, флаги.

Проверяем две вещи:
  * приложение киоска отдаёт экран без авторизации (и не отдаёт ничего лишнего);
  * запуск экрана в браузере работает без графики — «браузером» в тестах
    служит скрипт-заглушка, который только отмечает свой запуск в файле.
"""

from __future__ import annotations

import os
import shutil
import socket
import stat
import sys
import threading
import time
from pathlib import Path

import pytest
from werkzeug.serving import make_server

from robot_control.config import Config
from robot_control.kiosk_app import (
    DEFAULT_PORT,
    browser_command,
    create_kiosk_app,
    display_available,
    find_browser,
    kiosk_port,
    kiosk_url,
    open_kiosk_screen,
    parse_args,
    port_in_use,
    wait_for_server,
)


@pytest.fixture
def kiosk_app(config: Config):
    application = create_kiosk_app(config)
    application.config.update(TESTING=True)
    return application


@pytest.fixture
def kiosk_client(kiosk_app):
    return kiosk_app.test_client()


# ------------------------------- страница и API ------------------------------ #
def test_root_opens_the_screen_itself(kiosk_client):
    """Корень без запроса — сразу на экран: браузер робота открывает /."""
    response = kiosk_client.get("/")
    assert response.status_code == 302
    assert response.headers["Location"] == "/kiosk"


@pytest.mark.parametrize("path", ["/kiosk", "/robot.html", "/main.html"])
def test_screen_is_served_without_login(kiosk_client, path):
    response = kiosk_client.get(path)
    assert response.status_code == 200
    text = response.get_data(as_text=True)
    assert "Бортовой экран" in text      # заголовок киоска
    assert "k-soc" in text               # крупный заряд АКБ
    assert "robot.js" in text


def test_screen_assets_are_served(kiosk_client):
    for name in ("robot.css", "robot.js"):
        response = kiosk_client.get(f"/{name}")
        assert response.status_code == 200, name


def test_telemetry_api_returns_only_public_values(kiosk_client):
    for path in ("/api/kiosk_state", "/api/state"):
        response = kiosk_client.get(path)
        assert response.status_code == 200, path
        body = response.get_json()
        assert body["ok"] is True
        assert set(body["data"]) == {
            "battery", "motors", "lidar", "speedMps", "powerKw", "mode", "linkOk", "ts",
        }


def test_telemetry_has_no_cargo_or_audit_data(kiosk_client):
    """В киоске нет ни замка отсека, ни журнала — только АКБ, шасси, лидар."""
    body = kiosk_client.get("/api/kiosk_state").get_json()
    assert "lock" not in body["data"]
    assert "audit" not in body["data"]
    assert "cargo" not in str(body["data"]).lower()


def test_healthz_reports_kiosk_mode(kiosk_client):
    body = kiosk_client.get("/healthz").get_json()
    assert body["ok"] is True
    assert body["mode"] == "kiosk"


def test_screen_can_be_embedded_in_preview(kiosk_client):
    """Экран показывают во встроенном предпросмотре — запрет iframe не ставим."""
    response = kiosk_client.get("/kiosk")
    assert "X-Frame-Options" not in response.headers
    assert response.headers["Cache-Control"] == "no-store"


# ------------------------------ адрес и браузер ------------------------------ #
def test_screen_lives_on_port_8090(monkeypatch):
    monkeypatch.delenv("RC_KIOSK_PORT", raising=False)
    assert DEFAULT_PORT == 8090
    assert kiosk_port() == 8090
    assert kiosk_url() == "http://127.0.0.1:8090/kiosk"


def test_port_comes_from_environment(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_PORT", "8091")
    assert kiosk_port() == 8091
    assert kiosk_url(port=8091) == "http://127.0.0.1:8091/kiosk"


def test_broken_port_value_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_PORT", "не число")
    assert kiosk_port() == DEFAULT_PORT


@pytest.mark.parametrize(
    "env, expected",
    [
        ({}, False),
        ({"DISPLAY": ":0"}, True),
        ({"WAYLAND_DISPLAY": "wayland-0"}, True),
    ],
)
def test_display_detection(env, expected):
    assert display_available(env) is expected


def test_chromium_opens_without_frames():
    args = browser_command("/usr/bin/chromium-browser", "http://127.0.0.1:8090/kiosk")
    assert args[0] == "/usr/bin/chromium-browser"
    assert "--kiosk" in args
    assert "--app=http://127.0.0.1:8090/kiosk" in args


def test_firefox_opens_in_kiosk_mode():
    args = browser_command("/usr/bin/firefox", "http://127.0.0.1:8090/kiosk")
    assert args == ["/usr/bin/firefox", "--kiosk", "http://127.0.0.1:8090/kiosk"]


def test_browser_can_be_set_explicitly(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_BROWSER", sys.executable)
    assert find_browser() == sys.executable


def test_missing_browser_is_not_an_error(monkeypatch):
    """Нет браузера — не падаем: сервер живёт, подсказка уходит в лог."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", "/nonexistent/browser")
    monkeypatch.setattr(shutil, "which", lambda _name: None)
    assert find_browser() is None


# --------------------------------- порт и сервер ----------------------------- #
def test_port_in_use_detects_listener():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        sock.listen(1)
        busy = sock.getsockname()[1]
        assert port_in_use(busy) is True
    assert port_in_use(busy) is False


def test_wait_for_server_times_out_on_dead_port():
    with socket.socket() as sock:          # заведомо свободный порт
        sock.bind(("127.0.0.1", 0))
        free = sock.getsockname()[1]
    assert wait_for_server(f"http://127.0.0.1:{free}", timeout=0.3) is False


@pytest.fixture
def fake_browser(tmp_path: Path) -> Path:
    """«Браузер»: пишет строку в файл и сразу выходит (графики в тестах нет)."""
    marker = tmp_path / "opened.txt"
    script = tmp_path / "fake-browser.sh"
    script.write_text(
        "#!/bin/sh\n"
        f'echo "$@" >> "{marker}"\n',
        encoding="utf-8",
    )
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return script


def _wait_for(marker: Path, lines: int, timeout: float = 20.0) -> list[str]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if marker.exists():
            got = [line for line in marker.read_text(encoding="utf-8").splitlines() if line]
            if len(got) >= lines:
                return got
        time.sleep(0.2)
    return []


def test_screen_opens_itself_when_server_starts(config, fake_browser, monkeypatch):
    """Главное требование: экран на 8090 открывается сам, без команд в браузере."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(fake_browser))
    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    try:
        open_kiosk_screen(port, watch=False, stop_event=stop)
        opened = _wait_for(fake_browser.parent / "opened.txt", 1)
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
    assert opened, "экран не открылся в браузере"
    assert "/kiosk" in opened[0]


def test_screen_reopens_if_browser_was_closed(config, fake_browser, monkeypatch):
    """Киоск-режим: закрыли браузер — экран возвращается на дисплей сам."""
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(fake_browser))
    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    try:
        open_kiosk_screen(port, watch=True, stop_event=stop)
        opened = _wait_for(fake_browser.parent / "opened.txt", 2)
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
    assert len(opened) >= 2, f"экран не открылся заново: {opened}"


# ----------------------------------- запуск ---------------------------------- #
def test_default_flags_mean_open_the_screen():
    args = parse_args([])
    assert args.open_screen is None       # «как настроено/есть ли дисплей»
    assert args.port is None
    assert args.source is None


def test_flags_disable_and_force_opening():
    assert parse_args(["--no-open"]).open_screen is False
    assert parse_args(["--open"]).open_screen is True
    args = parse_args(["--open", "--port", "8091", "--source", "sim", "--no-watch"])
    assert (args.port, args.source, args.watch) == (8091, "sim", False)


@pytest.mark.parametrize("name", ["kiosk.sh", "robot-kiosk.service"])
def test_kiosk_launch_files_are_in_place(name):
    path = Path(__file__).resolve().parent.parent / "scripts" / name
    assert path.is_file(), name


def test_kiosk_script_is_executable():
    path = Path(__file__).resolve().parent.parent / "scripts" / "kiosk.sh"
    assert os.access(path, os.X_OK), "scripts/kiosk.sh должен быть исполняемым"
