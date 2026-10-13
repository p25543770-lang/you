"""Бортовой экран робота (киоск): страница на 8090, открытие экрана, флаги.

Проверяем три вещи:
  * публичная телеметрия и экран киоска работают без логина;
  * поворот моторов требует короткую PIN-сессию и точный адресный payload;
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
    PROFILE_DIR_NAME,
    _reopen_delay,
    browser_command,
    create_kiosk_app,
    display_available,
    display_claimed,
    find_browser,
    kiosk_port,
    kiosk_profile_dir,
    kiosk_url,
    open_kiosk_screen,
    parse_args,
    port_in_use,
    prepare_browser_profile,
    snap_name,
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


def test_screen_contains_individual_steering_controls(kiosk_client):
    text = kiosk_client.get("/kiosk").get_data(as_text=True)
    js = kiosk_client.get("/robot.js").get_data(as_text=True)
    assert 'id="swipe-motors-view"' in text
    assert 'class="swipe-tabs"' not in text
    assert 'id="swipe-tab-cameras"' not in text
    assert 'id="swipe-tab-motors"' not in text
    assert 'class="swipe-handle' not in text
    for module in ("FL", "FR", "RL", "RR"):
        assert f'id="steer-slider-{module}"' in text
        assert f'data-steer-send="{module}"' in text
    assert "api/kiosk_steer" in js
    assert "X-Robot-Control" in js
    assert "selectSwipeView(view || 'motors')" in js
    assert "openSwipe('motors')" in js  # переход из PIN открывает именно управление
    assert "openSwipe('cameras')" in js
    assert "if (dx > 65) openSwipe('motors');" in js
    assert "else if (dx < -65) openSwipe('cameras');" in js
    assert 'id="swipe-open-cameras"' not in text
    assert 'поворот модулей →' in text and '← камеры · логи' in text


def test_telemetry_api_returns_only_public_values(kiosk_client):
    for path in ("/api/kiosk_state", "/api/state"):
        response = kiosk_client.get(path)
        assert response.status_code == 200, path
        body = response.get_json()
        assert body["ok"] is True
        assert set(body["data"]) == {
            "battery", "motors", "lidar", "speedMps", "powerKw", "mode", "linkOk", "ts",
            "source", "steeringControl", "lidarStatus",
        }
        # причина «нет скана» видна на экране, а не только в журнале
        status = body["data"]["lidarStatus"]
        assert status["ok"] is True
        assert isinstance(status["reason"], str) and status["reason"]


def test_steering_endpoint_requires_pin_and_command_header(kiosk_client):
    body = {"module": "FL", "angle": 10}
    headers = {"X-Robot-Control": "steering"}
    assert kiosk_client.post("/api/kiosk_steer", json=body, headers=headers).status_code == 401

    verified = kiosk_client.post("/api/kiosk_lock/verify", json={"pin": "7232"})
    assert verified.status_code == 200
    assert verified.get_json()["steeringAuthorized"] is True
    assert kiosk_client.post("/api/kiosk_steer", json=body).status_code == 400
    # Эта фикстура работает на симуляторе, поэтому он заведомо не может двигать моторы.
    response = kiosk_client.post("/api/kiosk_steer", json=body, headers=headers)
    assert response.status_code == 409
    assert response.get_json()["reason"] == "control_not_supported"


def test_steering_endpoint_dispatches_exactly_one_module_and_angle(kiosk_app, kiosk_client):
    class FakeSteeringSource:
        name = "master"
        supports_individual_steer = True

        def __init__(self):
            self.commands = []

        def steer_module(self, module, angle):
            self.commands.append((module, angle))
            return {"ok": True, "angle": round(angle, 1)}

    source = FakeSteeringSource()
    kiosk_app.extensions["rc_slam"].source = source
    verified = kiosk_client.post("/api/kiosk_lock/verify", json={"pin": "7232"})
    assert verified.status_code == 200
    headers = {"X-Robot-Control": "steering"}

    response = kiosk_client.post("/api/kiosk_steer", json={"module": "fr", "angle": 12.5}, headers=headers)
    assert response.status_code == 200
    assert response.get_json()["module"] == "FR"
    assert source.commands == [("FR", 12.5)]

    for bad_body in (
        {"module": "FR", "angle": 12, "pwm": 100},
        {"module": "drive", "angle": 12},
        {"module": "FL", "angle": 33},
    ):
        response = kiosk_client.post("/api/kiosk_steer", json=bad_body, headers=headers)
        assert response.status_code == 400
    assert source.commands == [("FR", 12.5)]

    ended = kiosk_client.post("/api/kiosk_lock/session/end", json={})
    assert ended.status_code == 200
    assert kiosk_client.post("/api/kiosk_steer", json={"module": "FR", "angle": 10}, headers=headers).status_code == 401


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


def test_chromium_uses_its_own_profile():
    """Свой профиль: повторный запуск не «передаёт адрес» чужой копии браузера.

    Без --user-data-dir процесс Chromium сразу выходит (окно открыла уже
    работающая копия), сторожевой таймер считает это закрытием окна и
    открывает экран снова и снова — окна плодятся.
    """
    args = browser_command(
        "/usr/bin/chromium-browser", "http://127.0.0.1:8090/kiosk",
        profile_dir="/tmp/robot-kiosk-profile",
    )
    assert "--user-data-dir=/tmp/robot-kiosk-profile" in args


def test_firefox_opens_in_kiosk_mode():
    args = browser_command(
        "/usr/bin/firefox", "http://127.0.0.1:8090/kiosk",
        profile_dir="/tmp/robot-kiosk-profile",
    )
    assert args == [
        "/usr/bin/firefox", "--kiosk", "--no-remote",
        "--profile", "/tmp/robot-kiosk-profile",
        "http://127.0.0.1:8090/kiosk",
    ]


def test_snap_name_from_path_and_wrapper(tmp_path: Path):
    """Ubuntu кладёт в PATH обёртку, а не сам бинарник snap."""
    assert snap_name("/snap/bin/firefox") == "firefox"
    assert snap_name("/snap/bin/firefox.geckodriver") == "firefox"
    assert snap_name("/snap/firefox/current/usr/lib/firefox/firefox") == "firefox"
    assert snap_name(str(tmp_path / "no-such-firefox")) is None

    wrapper = tmp_path / "firefox"
    wrapper.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")
    assert snap_name(str(wrapper)) == "firefox"

    via_run = tmp_path / "chromium-browser"
    via_run.write_text("#!/bin/sh\nexec snap run chromium \"$@\"\n", encoding="utf-8")
    assert snap_name(str(via_run)) == "chromium"

    native = tmp_path / "firefox-bin"
    native.write_bytes(b"\x7fELF")
    assert snap_name(str(native)) is None


def test_prepare_creates_missing_firefox_profile(tmp_path, monkeypatch):
    """Каталога нет — Firefox сам его не создаст и покажет «profile cannot be loaded»."""
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    monkeypatch.setenv("XDG_CACHE_HOME", str(home / "cache"))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == (home / "cache" / PROFILE_DIR_NAME).resolve()
    assert profile.is_dir()
    assert oct(profile.stat().st_mode & 0o777) == "0o700"
    text = (profile / "user.js").read_text(encoding="utf-8")
    assert "browser.shell.checkDefaultBrowser" in text
    assert "browser.sessionstore.resume_from_crash" in text


def test_prepare_does_not_clobber_existing_user_js(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("XDG_CACHE_HOME", str(home / "cache"))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    profile = prepare_browser_profile(str(exe))
    (profile / "user.js").write_text("user_pref(\"custom\", true);\n", encoding="utf-8")

    again = prepare_browser_profile(str(exe))

    assert again == profile
    assert (profile / "user.js").read_text(encoding="utf-8") == "user_pref(\"custom\", true);\n"


def test_snap_firefox_profile_is_not_in_a_hidden_dir(tmp_path, monkeypatch):
    """Snap не читает ~/.cache — оттуда и диалог «profile cannot be loaded»."""
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    monkeypatch.delenv("XDG_CACHE_HOME", raising=False)
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    expected = home / "snap" / "firefox" / "common" / PROFILE_DIR_NAME
    assert profile == expected.resolve()
    assert profile.is_dir()
    assert (profile / "user.js").is_file()
    assert not any(part.startswith(".") for part in profile.relative_to(home.resolve()).parts)


def test_snap_keeps_explicit_visible_profile(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    visible = home / "kiosk-profile"
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("RC_KIOSK_PROFILE_DIR", str(visible))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec snap run firefox \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == visible.resolve()
    assert profile.is_dir()


def test_snap_relocates_tmp_and_hidden_profiles(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")
    expected = (home / "snap" / "firefox" / "common" / PROFILE_DIR_NAME).resolve()

    from_tmp = prepare_browser_profile(str(exe), profile_dir=tmp_path / "outside")
    from_hidden = prepare_browser_profile(
        str(exe), profile_dir=home / ".cache" / PROFILE_DIR_NAME,
    )

    assert from_tmp == expected
    assert from_hidden == expected


def test_snap_falls_back_to_visible_home_dir_when_snap_tree_is_blocked(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    (home / "snap").write_text("not a directory", encoding="utf-8")
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexec /snap/bin/firefox \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == (home / PROFILE_DIR_NAME).resolve()
    assert profile.is_dir()
    assert (profile / "user.js").is_file()


def test_chromium_snap_gets_a_visible_profile_without_firefox_prefs(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    exe = tmp_path / "chromium-browser"
    exe.write_text("#!/bin/sh\nexec /snap/bin/chromium \"$@\"\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))

    assert profile == (home / "snap" / "chromium" / "common" / PROFILE_DIR_NAME).resolve()
    assert profile.is_dir()
    assert not (profile / "user.js").exists()


def test_stale_firefox_lock_is_cleared_and_live_lock_is_kept(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("XDG_CACHE_HOME", str(home / "cache"))
    exe = tmp_path / "firefox"
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    profile = prepare_browser_profile(str(exe))
    (profile / "lock").symlink_to("127.0.0.1:+999999999")
    (profile / ".parentlock").write_text("", encoding="utf-8")
    prepare_browser_profile(str(exe))
    assert not (profile / "lock").exists()
    assert not (profile / ".parentlock").exists()

    (profile / "lock").symlink_to(f"127.0.0.1:+{os.getpid()}")
    prepare_browser_profile(str(exe))
    assert (profile / "lock").is_symlink()


def test_screen_launches_snap_firefox_with_a_readable_profile(config, tmp_path, monkeypatch):
    """Окно ошибки больше не из чего рисовать: каталог есть и snap его видит."""
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR", raising=False)
    monkeypatch.delenv("XDG_CACHE_HOME", raising=False)
    marker = tmp_path / "opened.txt"
    script = tmp_path / "firefox"
    script.write_text(
        "#!/bin/sh\n"
        "# Ubuntu wrapper: exec /snap/bin/firefox \"$@\"\n"
        f'echo "$@" >> "{marker}"\n',
        encoding="utf-8",
    )
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(script))

    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    try:
        open_kiosk_screen(port, watch=False, stop_event=stop)
        opened = _wait_for(marker, 1)
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    expected = (home / "snap" / "firefox" / "common" / PROFILE_DIR_NAME).resolve()
    assert opened, "экран не открылся"
    assert "--profile" in opened[0]
    assert str(expected) in opened[0]
    assert expected.is_dir()
    assert (expected / "user.js").is_file()
    assert "--kiosk" in opened[0]


def test_kiosk_profile_dir_from_environment(monkeypatch):
    monkeypatch.setenv("RC_KIOSK_PROFILE_DIR", "/tmp/custom-kiosk")
    assert kiosk_profile_dir() == Path("/tmp/custom-kiosk")
    monkeypatch.delenv("RC_KIOSK_PROFILE_DIR")
    monkeypatch.setenv("XDG_CACHE_HOME", "/tmp/cache")
    assert kiosk_profile_dir() == Path("/tmp/cache/robot-kiosk-browser")


def test_reopen_delay_grows_for_instantly_exiting_browser():
    """Браузер выходит сразу — не долбим его каждые 2 секунды очередями окон."""
    assert _reopen_delay(0) == 2.0
    assert _reopen_delay(1) == 2.0
    assert _reopen_delay(2) == 4.0
    assert _reopen_delay(3) == 8.0
    assert _reopen_delay(100) == 30.0  # потолок


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


@pytest.fixture
def long_lived_browser(tmp_path: Path) -> Path:
    """«Браузер», который не выходит сразу: окно как будто открыто."""
    marker = tmp_path / "opened.txt"
    script = tmp_path / "long-browser.sh"
    script.write_text(
        "#!/bin/sh\n"
        f'echo "$@" >> "{marker}"\n'
        "sleep 60\n",
        encoding="utf-8",
    )
    script.chmod(script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return script


def test_screen_is_not_opened_twice(config, long_lived_browser, monkeypatch):
    """Главное: повторный запуск не плодит окна — экран строго в одном экземпляре.

    Автозапуск рабочего стола, ярлык и ручной kiosk.sh приводят сюда же:
    кто первый занял показ, тот окно и держит, остальные получают отказ.
    """
    monkeypatch.setenv("RC_KIOSK_BROWSER", str(long_lived_browser))
    server = make_server("127.0.0.1", 0, create_kiosk_app(config), threaded=True)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop = threading.Event()
    stop2 = threading.Event()
    marker = long_lived_browser.parent / "opened.txt"
    try:
        first = open_kiosk_screen(port, watch=True, stop_event=stop)
        assert first is not None
        assert _wait_for(marker, 1), "первый экран не открылся"
        assert display_claimed(port) is True

        # Второй и третий показы того же порта — отказ, а не новое окно.
        assert open_kiosk_screen(port, watch=True, stop_event=threading.Event()) is None
        assert open_kiosk_screen(port, watch=False, stop_event=threading.Event()) is None
        assert len(_wait_for(marker, 1)) == 1, "экран открылся повторно"

        # Показ остановлен — замок отпущен, экран можно открыть снова.
        stop.set()
        first.join(timeout=10)
        assert display_claimed(port) is False
        assert open_kiosk_screen(port, watch=True, stop_event=stop2) is not None
        assert len(_wait_for(marker, 2)) == 2
    finally:
        stop.set()
        stop2.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


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
