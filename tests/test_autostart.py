"""Автозапуск приложения: по умолчанию его нет, а включённый — выключается скриптом.

Проверяем поведение, из-за которого робот «сам поднимал» пульт и экран:
`install_ubuntu.sh` больше не включает автозапуск без флага, `make_shortcut.sh`
не ставит ярлык экрана в автозапуск сеанса, а `scripts/autostart.sh` умеет
показать, выключить и вернуть автозапуск (службы systemd, ярлык сеанса,
автоподъём точки доступа Wi-Fi). `autostart.sh off` = «выключить всё»: он ещё
и останавливает запущенное (через `scripts/stop.sh`), а `--keep-running`
выключает только автозапуск. `scripts/stop.sh` останавливает и серверы по
pid-файлам, и службы, и окно киоск-браузера.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"

FAKE_SYSTEMCTL = """#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$FAKE_LOG"
case "$1" in
  list-unit-files) exit 0 ;;
  cat)             exit 0 ;;
  is-enabled)      echo "${FAKE_UNIT_ENABLED:-disabled}"; exit 0 ;;
  is-active)       echo "${FAKE_UNIT_ACTIVE:-inactive}"; exit 0 ;;
  disable|enable)  exit 0 ;;
esac
exit 0
"""

FAKE_NMCLI = """#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$FAKE_LOG"
if [[ "$*" == *"-g"* ]]; then
  echo "${FAKE_AP_AUTOCONNECT:-no}"; exit 0
fi
if [[ "$*" == *"connection show"* ]]; then
  echo "robot-ap"; exit 0
fi
exit 0
"""

FAKE_SUDO = """#!/usr/bin/env bash
exec "$@"
"""


@pytest.fixture
def fake_host(tmp_path, monkeypatch):
    """Фальшивый «робот»: HOME во временной папке и подменённые systemctl/nmcli."""
    home = tmp_path / "home"
    home.mkdir()
    shims = tmp_path / "bin"
    shims.mkdir()
    for name, body in (("systemctl", FAKE_SYSTEMCTL), ("nmcli", FAKE_NMCLI), ("sudo", FAKE_SUDO)):
        path = shims / name
        path.write_text(body, encoding="utf-8")
        path.chmod(0o755)
    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")

    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("PATH", "%s%s%s" % (shims, os.pathsep, os.environ.get("PATH", "")))
    monkeypatch.setenv("FAKE_LOG", str(log))
    monkeypatch.setenv("RC_VAR_DIR", str(tmp_path / "var"))
    monkeypatch.delenv("DISPLAY", raising=False)

    class Host:
        root = home
        calls = log

    return Host


def run(name: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", str(SCRIPTS / name), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def autostart_file(home: Path) -> Path:
    return home / ".config" / "autostart" / "robot-kiosk.desktop"


# --------------------------------------------------------------- файлы скриптов --- #


@pytest.mark.parametrize("name", ["autostart.sh", "make_shortcut.sh", "install_ubuntu.sh"])
def test_script_is_in_place_and_runs(name):
    path = SCRIPTS / name
    assert path.is_file(), name
    assert os.access(path, os.X_OK), "%s должен быть исполняемым" % name
    syntax = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True, check=False)
    assert syntax.returncode == 0, syntax.stderr


# ----------------------------------------------------------------- автозапуск off --- #


def test_autostart_off_disables_services_session_entry_and_access_point(fake_host):
    """`autostart.sh off` убирает все места, из которых приложение стартовало само."""
    entry = autostart_file(fake_host.root)
    entry.parent.mkdir(parents=True)
    entry.write_text("[Desktop Entry]\nName=Экран робота (киоск)\n", encoding="utf-8")

    result = run("autostart.sh", "off")
    assert result.returncode == 0, result.stderr

    calls = fake_host.calls.read_text(encoding="utf-8")
    assert "disable --now robot-control.service" in calls
    assert "disable --now robot-kiosk.service" in calls
    assert "connection.autoconnect no" in calls
    assert not entry.exists(), "ярлык автозапуска сеанса должен быть удалён"
    assert "вручную" in result.stdout


def test_autostart_status_shows_what_starts_itself(fake_host, monkeypatch):
    """`status` отвечает на вопрос «что поднимается само» без запуска служб."""
    monkeypatch.setenv("FAKE_UNIT_ENABLED", "enabled")
    monkeypatch.setenv("FAKE_UNIT_ACTIVE", "active")
    monkeypatch.setenv("FAKE_AP_AUTOCONNECT", "yes")
    entry = autostart_file(fake_host.root)
    entry.parent.mkdir(parents=True)
    entry.write_text("[Desktop Entry]\nName=Экран робота (киоск)\n", encoding="utf-8")

    result = run("autostart.sh", "status")
    assert result.returncode == 0, result.stderr
    out = result.stdout
    assert out.count("автозапуск ВКЛЮЧЁН") == 2      # пульт и экран
    assert "ВКЛЮЧЁН" in out and str(entry) in out
    assert "автоподъём ВКЛЮЧЁН" in out
    assert "autostart.sh off" in out


def test_autostart_status_is_quiet_when_nothing_is_enabled(fake_host):
    result = run("autostart.sh", "status")
    assert result.returncode == 0, result.stderr
    assert "само ничего не запускается" in result.stdout
    assert "./scripts/start.sh" in result.stdout
    assert "./scripts/kiosk.sh" in result.stdout


def test_autostart_rejects_unknown_action(fake_host):
    result = run("autostart.sh", "ол-инклюд")
    assert result.returncode != 0
    assert "неизвестное действие" in result.stderr


def test_autostart_on_enables_services_and_session_entry(fake_host):
    result = run("autostart.sh", "on")
    assert result.returncode == 0, result.stderr
    calls = fake_host.calls.read_text(encoding="utf-8")
    assert "enable --now robot-control.service" in calls
    assert "enable --now robot-kiosk.service" in calls
    assert autostart_file(fake_host.root).is_file(), "ярлык автозапуска сеанса должен появиться"


def spawn_fake_server(var_dir: Path, *, name: str = "run.pid") -> subprocess.Popen:
    """Запускает «сервер» (долгоживущий процесс) и кладёт его pid в var/, как start.sh."""
    process = subprocess.Popen(["sleep", "60"], start_new_session=True)
    var_dir.mkdir(parents=True, exist_ok=True)
    (var_dir / name).write_text("%d\n" % process.pid, encoding="utf-8")
    return process


def alive(process: subprocess.Popen) -> bool:
    return process.poll() is None


def wait_dead(process: subprocess.Popen, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return True
        time.sleep(0.05)
    return False


def test_autostart_off_stops_what_is_running(fake_host, tmp_path):
    """`off` без флагов = выключить всё: и автозапуск, и то, что запущено сейчас."""
    var_dir = tmp_path / "var"
    server = spawn_fake_server(var_dir)
    try:
        result = run("autostart.sh", "off")
        assert result.returncode == 0, result.stderr
        assert wait_dead(server), "запущенный сервер должен быть остановлен"
        assert not (var_dir / "run.pid").exists(), "pid-файл должен быть убран"
        assert "останавливаю то, что запущено сейчас" in result.stdout
    finally:
        if alive(server):
            server.kill()
            server.wait(timeout=5)


def test_autostart_off_keep_running_only_disables(fake_host, tmp_path):
    """`off --keep-running` выключает автозапуск, но не глушит запущенное."""
    var_dir = tmp_path / "var"
    server = spawn_fake_server(var_dir)
    try:
        result = run("autostart.sh", "off", "--keep-running")
        assert result.returncode == 0, result.stderr
        assert alive(server), "с --keep-running работающий сервер остаётся"
        assert (var_dir / "run.pid").exists()
        assert "запущенное не трогаю" in result.stdout
        calls = fake_host.calls.read_text(encoding="utf-8")
        assert "disable robot-control.service" in calls
        assert "disable --now robot-control.service" not in calls
    finally:
        server.kill()
        server.wait(timeout=5)


def test_autostart_off_rejects_unknown_flag(fake_host):
    result = run("autostart.sh", "off", "--не-флаг")
    assert result.returncode != 0
    assert "неизвестный аргумент" in result.stderr


def test_stop_sh_stops_servers_from_pid_files(fake_host, tmp_path):
    """stop.sh гасит и пульт, и экран по pid-файлам, и убирает сами файлы."""
    var_dir = tmp_path / "var"
    control = spawn_fake_server(var_dir, name="run.pid")
    kiosk = spawn_fake_server(var_dir, name="kiosk.pid")
    try:
        result = run("stop.sh", "--servers")
        assert result.returncode == 0, result.stderr
        assert wait_dead(control) and wait_dead(kiosk)
        assert not any(var_dir.iterdir()), "pid-файлы должны быть удалены"
        assert "Всё остановлено" in result.stdout
        # --servers не трогает службы: ими занимается autostart.sh off
        assert "systemctl" not in fake_host.calls.read_text(encoding="utf-8")
    finally:
        for process in (control, kiosk):
            if alive(process):
                process.kill()
                process.wait(timeout=5)


def test_stop_sh_reports_nothing_running(fake_host):
    result = run("stop.sh")
    assert result.returncode == 0, result.stderr
    assert "Запущенного приложения не найдено" in result.stdout
    assert "autostart.sh" in result.stdout


def test_stop_sh_help_explains_autostart_separation(fake_host):
    result = run("stop.sh", "--help")
    assert result.returncode == 0, result.stderr
    assert "autostart.sh off" in result.stdout


# ------------------------------------------------------------- ярлыки на рабочем столе --- #


def test_make_shortcut_does_not_autostart_by_default(fake_host):
    """Ярлыки ставятся, а в автозапуск сеанса экран не попадает."""
    result = run("make_shortcut.sh")
    assert result.returncode == 0, result.stderr
    assert (fake_host.root / ".local" / "share" / "applications" / "robot-console.desktop").is_file()
    assert not autostart_file(fake_host.root).exists()
    assert "Автозапуск не включаю" in result.stdout


def test_make_shortcut_removes_old_autostart_entry(fake_host):
    """Обновление кода не должно оставлять на роботе старый автозапуск."""
    entry = autostart_file(fake_host.root)
    entry.parent.mkdir(parents=True)
    entry.write_text("[Desktop Entry]\nName=Экран робота (киоск)\n", encoding="utf-8")

    result = run("make_shortcut.sh")
    assert result.returncode == 0, result.stderr
    assert not entry.exists()
    assert "Автозапуск выключен" in result.stdout


def test_make_shortcut_autostart_only_on_request(fake_host):
    result = run("make_shortcut.sh", "--autostart")
    assert result.returncode == 0, result.stderr
    content = autostart_file(fake_host.root).read_text(encoding="utf-8")
    assert "X-GNOME-Autostart-enabled=true" in content
    assert "Terminal=false" in content

    result = run("make_shortcut.sh", "--no-autostart")
    assert result.returncode == 0, result.stderr
    assert not autostart_file(fake_host.root).exists()


# ---------------------------------------------------------------- установка --- #


def test_install_does_not_enable_autostart_without_flag():
    """Автозапуск ставится только с флагом --autostart, а не «за компанию»."""
    text = (SCRIPTS / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert "--autostart" in text
    assert "if [[ $AUTOSTART -eq 1 ]]; then" in text
    # без флага сервер поднимается только в текущем окне терминала
    assert "if [[ $RUN -eq 1 && $AUTOSTART -eq 0 ]]; then" in text
    # точка доступа поднимается без автоподъёма при загрузке
    assert "./scripts/setup_ap.sh --autostart" in text


def test_install_help_explains_manual_start():
    result = run("install_ubuntu.sh", "--help")
    assert result.returncode == 0, result.stderr
    assert "Автозапуск при загрузке по умолчанию НЕ ставится" in result.stdout
    assert "./scripts/autostart.sh on" in result.stdout


def test_install_rejects_unknown_flag():
    result = run("install_ubuntu.sh", "--автозапуск")
    assert result.returncode != 0
    assert "неизвестный аргумент" in result.stderr


if __name__ == "__main__":  # pragma: no cover — ручной запуск
    sys.exit(pytest.main([__file__, "-q"]))
