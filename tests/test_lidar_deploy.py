"""Установка и запуск на роботе: зависимости, службы, сохранение настроек, проверка экранов.

Это «штатный способ» из задания: pyserial должен ставиться сам (и чиниться, если
его нет), службы systemd — стартовать из ``.venv`` проекта и иметь доступ к
``/dev/ttyUSB*``, найденные настройки лидара — сохраняться в ``.env`` одной
командой, а путь «порт → API → экран» — проверяться без браузера.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

# фикстуры фальшивого порта — там же, где и в тестах протоколов лидара
from test_lidar_serial import (FakeLds01, FakeSerial, fake_port, fake_serial,  # noqa: F401
                               load_lidar_check)

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"

lidar_web_check = None


def load_web_check():
    """Загружает scripts/lidar_web_check.py как модуль."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "lidar_web_check_cli", SCRIPTS / "lidar_web_check.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------
# Службы systemd: тот же интерпретатор, что и проверка лидара
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("unit", ["robot-control.service", "robot-kiosk.service"])
def test_service_unit_runs_from_project_venv(unit):
    """Служба стартует python'ом из .venv: в системном нет ни Flask, ни pyserial."""
    text = (SCRIPTS / unit).read_text(encoding="utf-8")
    exec_start = [line for line in text.splitlines() if line.startswith("ExecStart=")]
    assert exec_start, "в юните нет ExecStart"
    assert "/.venv/bin/python" in exec_start[0], exec_start
    assert "/usr/bin/python3" not in exec_start[0], \
        "системный python3: лидар не читается, хотя scripts/lidar_check.py работает"


@pytest.mark.parametrize("unit", ["robot-control.service", "robot-kiosk.service"])
def test_service_unit_has_serial_port_access(unit):
    """Без группы dialout служба получает Permission denied на /dev/ttyUSB*."""
    text = (SCRIPTS / unit).read_text(encoding="utf-8")
    assert "SupplementaryGroups=dialout" in text
    # каталог data/ нужен на запись: там замок порта лидара
    assert "ReadWritePaths=" in text and "data" in text


def test_install_ubuntu_points_service_at_venv_python():
    text = (SCRIPTS / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert ".venv/bin/python" in text
    assert "ExecStart=" in text          # юнит правится под фактический путь


# ---------------------------------------------------------------------------
# Зависимости: pyserial ставится сам и чинится, если пропал
# ---------------------------------------------------------------------------


@pytest.fixture
def offline_install(tmp_path):
    """Мини-копия проекта для запуска ensure_deps.sh без сети."""
    if shutil.which("python3") is None:
        pytest.skip("нет python3")
    project = tmp_path / "project"
    (project / "scripts").mkdir(parents=True)
    (project / "vendor").mkdir()
    shutil.copy(SCRIPTS / "ensure_deps.sh", project / "scripts" / "ensure_deps.sh")
    shutil.copy(ROOT / "requirements.txt", project / "requirements.txt")
    for wheel in (ROOT / "vendor").glob("*.whl"):
        shutil.copy(wheel, project / "vendor" / wheel.name)
    env = dict(os.environ)
    env.update({
        # установка идёт строго из vendor/: тест не зависит от наличия сети
        "PIP_NO_INDEX": "1",
        "PIP_FIND_LINKS": str(project / "vendor"),
        "PIP_DISABLE_PIP_VERSION_CHECK": "1",
        "PIP_NO_INPUT": "1",
    })
    return project, env


def run_deps(project: Path, env: dict) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", str(project / "scripts" / "ensure_deps.sh")],
        cwd=str(project), env=env, capture_output=True, text=True, timeout=600,
    )


def test_ensure_deps_installs_pyserial_offline(offline_install):
    project, env = offline_install
    result = run_deps(project, env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "pyserial" in result.stdout
    check = subprocess.run(
        [str(project / ".venv" / "bin" / "python"), "-c", "import serial; print(serial.VERSION)"],
        capture_output=True, text=True, timeout=60,
    )
    assert check.returncode == 0 and check.stdout.strip() == "3.5", check.stderr


def test_ensure_deps_repairs_missing_pyserial(offline_install):
    """Хэш requirements.txt не изменился, а pyserial пропал — ставится заново.

    Ровно этот случай и заставлял ставить pyserial вручную на роботе: скрипт
    видел совпадающий хэш и рапортовал «зависимости уже установлены».
    """
    project, env = offline_install
    assert run_deps(project, env).returncode == 0
    pip = project / ".venv" / "bin" / "pip"
    python = project / ".venv" / "bin" / "python"
    assert subprocess.run([str(pip), "uninstall", "-y", "pyserial"],
                          capture_output=True, text=True, timeout=300).returncode == 0
    assert subprocess.run([str(python), "-c", "import serial"],
                          capture_output=True, text=True).returncode != 0

    repaired = run_deps(project, env)
    assert repaired.returncode == 0, repaired.stdout + repaired.stderr
    assert "не хватает" in repaired.stdout
    assert subprocess.run([str(python), "-c", "import serial"],
                          capture_output=True, text=True, timeout=60).returncode == 0


def test_ensure_deps_fails_loudly_when_install_is_impossible(offline_install):
    """Ставить не откуда — скрипт говорит об этом прямо, а не запускает пульт без лидара."""
    project, env = offline_install
    for wheel in (project / "vendor").glob("pyserial*.whl"):
        wheel.unlink()
    env["PIP_FIND_LINKS"] = str(project / "vendor" / "empty")
    result = run_deps(project, env)
    assert result.returncode != 0
    assert "pyserial" in (result.stderr + result.stdout) or "serial" in result.stderr


# ---------------------------------------------------------------------------
# Сохранение найденных настроек в .env
# ---------------------------------------------------------------------------


# fake_serial/fake_port — фикстуры из test_lidar_serial (импортированы выше)
def test_lidar_check_saves_settings_to_env(
        fake_serial, fake_port, tmp_path, capsys):  # noqa: F811
    FakeSerial.devices[fake_port] = FakeLds01()
    env_file = tmp_path / ".env"
    env_file.write_text(
        "RC_SECRET_KEY=secret\nRC_PORT=8080\n# комментарий\nRC_LIDAR_PORT=\n",
        encoding="utf-8",
    )
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--seconds", "0.2",
                        "--save-env", "--env-file", str(env_file)]) == 0

    lines = env_file.read_text(encoding="utf-8").splitlines()
    values = dict(
        (line.split("=", 1)[0], line.split("=", 1)[1])
        for line in lines if "=" in line and not line.startswith("#")
    )
    assert values["RC_LIDAR_PORT"] == fake_port
    assert values["RC_LIDAR_PROTOCOL"] == "lds01"
    assert values["RC_LIDAR_BAUD"] == "230400"
    assert values["RC_LIDAR_MOTOR"] == "none"
    # остальной файл не пострадал, ключ сессии на месте, права — 600
    assert values["RC_SECRET_KEY"] == "secret"
    assert values["RC_PORT"] == "8080"
    assert "# комментарий" in lines
    assert oct(env_file.stat().st_mode & 0o777) == "0o600"
    out = capsys.readouterr().out
    assert "перезапусти веб-приложения" in out
    assert "lidar_web_check.py" in out


def test_lidar_check_creates_env_with_secret_key(
        fake_serial, fake_port, tmp_path):  # noqa: F811
    """--save-env без .env: файл создаётся, и RC_SECRET_KEY в нём есть."""
    FakeSerial.devices[fake_port] = FakeLds01()
    env_file = tmp_path / ".env"
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--seconds", "0.2",
                        "--save-env", "--env-file", str(env_file)]) == 0
    text = env_file.read_text(encoding="utf-8")
    assert "RC_SECRET_KEY=" in text and len(text.split("RC_SECRET_KEY=")[1].splitlines()[0]) > 20
    assert "RC_LIDAR_PROTOCOL=lds01" in text


def test_lidar_check_suggests_stable_port_name(
        fake_serial, tmp_path, monkeypatch, capsys):  # noqa: F811
    """В .env предлагается /dev/serial/by-id/…, а не меняющийся /dev/ttyUSBx."""
    device = tmp_path / "ttyUSB1"
    device.write_text("")
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    link = by_id / "usb-CP2102-lidar"
    link.symlink_to(device)

    def fake_glob(pattern, **_):
        if "by-id" in pattern:
            return [str(link)]
        return []

    monkeypatch.setattr("robot_control.lidar.glob.glob", fake_glob)
    from robot_control.lidar import stable_port_path

    assert stable_port_path(str(device)) == str(link)
    assert stable_port_path(str(link)) == str(link)

    FakeSerial.devices[str(device)] = FakeLds01()
    FakeSerial.devices[str(link)] = FakeSerial.devices[str(device)]   # тот же лидар
    module = load_lidar_check()
    assert module.main(["--port", str(device), "--seconds", "0.2"]) == 0
    out = capsys.readouterr().out
    assert "RC_LIDAR_PORT=%s" % link in out


# ---------------------------------------------------------------------------
# Проверка веб-экранов без браузера
# ---------------------------------------------------------------------------


def real_payload(points: int = 360) -> dict:
    return {
        "ok": True, "connected": True, "demo": False, "source": "serial:ttyUSB1",
        "points_count": points, "age_s": 0.1, "scan": [{"a": 0.0, "r": 1.5}] * points,
        "detected": {"port": "/dev/ttyUSB1", "protocol": "lds01", "baud": 230400,
                     "motor": "none"},
        "python": "/home/robot/you/.venv/bin/python", "pyserial": "3.5",
        "reason": "скан идёт: serial:ttyUSB1", "hint": "", "usb_diagnostics": None,
    }


@pytest.fixture
def api_server():
    """Сервер, отдающий на /api/lidar то, что положат в ``state['payload']``."""
    state = {"payload": real_payload()}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):                                     # noqa: N802
            body = json.dumps(state["payload"]).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield "http://127.0.0.1:%d/api/lidar" % server.server_port, state
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_web_check_accepts_real_scan_on_both_screens(api_server, capsys):
    url, _state = api_server
    module = load_web_check()
    assert module.main(["--console-url", url, "--kiosk-url", url, "--wait", "0"]) == 0
    out = capsys.readouterr().out
    assert "реальный скан: 360 точек" in out
    assert "lds01" in out and "230400" in out
    assert "pyserial: 3.5" in out


def test_web_check_rejects_demo_scan(api_server, capsys):
    url, state = api_server
    state["payload"] = dict(real_payload(), connected=False, demo=True, source="no_signal",
                            points_count=0, scan=[{"a": 0.0, "r": 3.0}], detected=None,
                            reason="демо-скан источника данных (реального лидара нет)")
    module = load_web_check()
    assert module.main(["--console-url", url, "--kiosk-url", url, "--wait", "0"]) == 1
    out = capsys.readouterr().out
    assert "ДЕМО-скан вместо реального лидара" in out
    assert "lidar_check.py --save-env" in out


def test_web_check_reports_missing_pyserial(api_server, capsys):
    """Пульт запущен не из .venv — проверка говорит об этом прямо."""
    url, state = api_server
    state["payload"] = dict(real_payload(), connected=False, demo=False, points_count=0,
                            scan=None, pyserial=None, source="no_signal",
                            python="/usr/bin/python3",
                            reason="нет библиотеки pyserial — USB-лидар не читается",
                            hint="поставь пакет: .venv/bin/pip install pyserial",
                            usb_diagnostics={"summary": "pyserial не установлен", "ports": []})
    module = load_web_check()
    assert module.main(["--console-url", url, "--kiosk-url", url, "--wait", "0"]) == 1
    out = capsys.readouterr().out
    assert "pyserial: НЕТ" in out
    assert "не из .venv" in out


def test_web_check_reports_unreachable_app(capsys):
    module = load_web_check()
    dead = "http://127.0.0.1:1/api/lidar"
    assert module.main(["--console-url", dead, "--kiosk-url", dead, "--wait", "0"]) == 2
    out = capsys.readouterr().out
    assert "не отвечает" in out
    assert "./scripts/start.sh" in out and "./scripts/kiosk.sh" in out


def test_web_check_waits_for_a_fresh_scan(api_server):
    """Сразу после перезапуска скана ещё нет — проверка ждёт, а не сдаётся."""
    url, state = api_server
    state["payload"] = dict(real_payload(), connected=False, points_count=0, scan=None,
                            demo=False, source="no_signal", reason="идёт поиск порта")
    module = load_web_check()

    def appear_later():
        time.sleep(0.6)
        state["payload"] = real_payload()

    threading.Thread(target=appear_later, daemon=True).start()
    assert module.main(["--console-url", url, "--kiosk-url", url, "--wait", "5"]) == 0


def test_web_check_json_output(api_server):
    url, _state = api_server
    result = subprocess.run(
        [sys.executable, str(SCRIPTS / "lidar_web_check.py"),
         "--console-url", url, "--kiosk-url", url, "--wait", "0", "--json"],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    assert report["ok"] is True
    assert len(report["reports"]) == 2
    assert all(item["real_scan"] for item in report["reports"])
