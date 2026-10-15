"""Fast-forward Git update safety and the authenticated updater API."""
from __future__ import annotations

import subprocess
import time
from pathlib import Path

import pytest

from robot_control.updater import UpdateManager
from conftest import login


def git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout.strip()


def make_remote(tmp_path: Path) -> tuple[Path, Path, Path]:
    remote = tmp_path / "origin.git"
    seed = tmp_path / "seed"
    clone = tmp_path / "clone"
    seed.mkdir()
    subprocess.run(["git", "init", "--bare", "--initial-branch=main", str(remote)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["git", "init", "--initial-branch=main", str(seed)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    git(seed, "config", "user.name", "Update test")
    git(seed, "config", "user.email", "update-test@example.invalid")
    (seed / ".gitignore").write_text("var/\n", encoding="utf-8")
    (seed / "README.md").write_text("version one\n", encoding="utf-8")
    (seed / "requirements.txt").write_text("flask==3.0\n", encoding="utf-8")
    git(seed, "add", ".")
    git(seed, "commit", "-m", "initial")
    git(seed, "remote", "add", "origin", str(remote))
    git(seed, "push", "-u", "origin", "main")
    subprocess.run(["git", "clone", str(remote), str(clone)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    git(clone, "config", "user.name", "Update test")
    git(clone, "config", "user.email", "update-test@example.invalid")
    return remote, seed, clone


def wait_for_terminal(manager: UpdateManager, timeout: float = 15) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = manager.get_status()
        if state["status"] in {"done", "error", "cancelled"}:
            return state
        time.sleep(0.05)
    pytest.fail("update thread did not finish")


def test_fast_forward_update_installs_code_and_dependencies(tmp_path):
    _remote, seed, clone = make_remote(tmp_path)
    (seed / "README.md").write_text("version two\n", encoding="utf-8")
    (seed / "requirements.txt").write_text("flask==3.1\n", encoding="utf-8")
    git(seed, "add", "README.md", "requirements.txt")
    git(seed, "commit", "-m", "update app and requirements")
    git(seed, "push", "origin", "main")

    installed: list[Path] = []
    restarted: list[bool] = []
    manager = UpdateManager(
        clone,
        branch="main",
        dependency_installer=lambda root: (installed.append(root) or 0, "fake installer OK"),
        on_restart=lambda: restarted.append(True),
    )
    check = manager.check()
    assert check["ok"] is True
    assert check["updateAvailable"] is True
    assert check["canUpdate"] is True
    assert check["dirty"] is False

    assert manager.start() is True
    state = wait_for_terminal(manager)
    assert state["status"] == "done"
    assert state["changes"] == 1
    assert state["restart"] is True
    assert git(clone, "rev-parse", "HEAD") == git(seed, "rev-parse", "HEAD")
    assert (clone / "README.md").read_text(encoding="utf-8") == "version two\n"
    assert installed == [clone]
    assert restarted == [True]


def test_dirty_tree_is_never_overwritten(tmp_path):
    _remote, seed, clone = make_remote(tmp_path)
    (seed / "README.md").write_text("remote version\n", encoding="utf-8")
    git(seed, "add", "README.md")
    git(seed, "commit", "-m", "remote update")
    git(seed, "push", "origin", "main")

    local_sha = git(clone, "rev-parse", "HEAD")
    (clone / "README.md").write_text("operator's local edit\n", encoding="utf-8")
    manager = UpdateManager(clone, branch="main")
    check = manager.check()
    assert check["dirty"] is True
    assert check["canUpdate"] is False

    assert manager.start() is True
    state = wait_for_terminal(manager)
    assert state["status"] == "error"
    assert "рабочей копии есть изменения" in state["error"]
    assert git(clone, "rev-parse", "HEAD") == local_sha
    assert (clone / "README.md").read_text(encoding="utf-8") == "operator's local edit\n"


def test_diverged_branch_is_refused_without_reset(tmp_path):
    _remote, seed, clone = make_remote(tmp_path)
    (clone / "local.txt").write_text("local commit\n", encoding="utf-8")
    git(clone, "add", "local.txt")
    git(clone, "commit", "-m", "local only")
    local_sha = git(clone, "rev-parse", "HEAD")

    (seed / "remote.txt").write_text("remote commit\n", encoding="utf-8")
    git(seed, "add", "remote.txt")
    git(seed, "commit", "-m", "remote only")
    git(seed, "push", "origin", "main")

    manager = UpdateManager(clone, branch="main")
    assert manager.start() is True
    state = wait_for_terminal(manager)
    assert state["status"] == "error"
    assert "ветки разошлись" in state["error"]
    assert git(clone, "rev-parse", "HEAD") == local_sha
    assert (clone / "local.txt").is_file()


def test_updater_api_requires_login_and_csrf(client, app, operator, monkeypatch):
    updater = app.extensions["rc_updater"]
    monkeypatch.setattr(updater, "check", lambda: {
        "ok": True, "repo": "example/project", "branch": "main",
        "local": "1234567", "localSha": "1" * 40,
        "latest": "abcdef0", "latestSha": "a" * 40,
        "updateAvailable": True, "dirty": False, "canUpdate": True,
        "busy": False, "tooling": {"git": True}, "mode": "clone",
    })
    monkeypatch.setattr(updater, "start", lambda: True)

    assert client.get("/api/update/check").status_code == 302
    assert login(client).status_code == 302

    check = client.get("/api/update/check")
    assert check.status_code == 200
    payload = check.get_json()
    assert payload["csrfToken"]
    assert payload["branch"] == "main"

    assert client.post("/api/update/run", json={}).status_code == 400
    run = client.post("/api/update/run", json={}, headers={"X-CSRF-Token": payload["csrfToken"]})
    assert run.status_code == 202
    assert run.get_json()["status"] == "busy"


def test_update_reinstalls_and_builds_even_without_requirements_change(tmp_path):
    """Кнопка «Обновить с GitHub»: полный цикл, а не только git merge.

    Раньше зависимости ставились лишь при смене requirements.txt, а сборки не
    было вовсе: оператор жал обновление, а окружение и битый код могли
    остаться прежними. Теперь каждый апдейт: fetch → ff → ensure_deps →
    compile/import check → restart, а падение сборки откатывает код.
    """
    _remote, seed, clone = make_remote(tmp_path)
    (seed / "README.md").write_text("version two, code only\n", encoding="utf-8")
    git(seed, "add", "README.md")
    git(seed, "commit", "-m", "code only update")
    git(seed, "push", "origin", "main")

    calls: list[str] = []
    manager = UpdateManager(
        clone, branch="main",
        dependency_installer=lambda root: (calls.append("deps") or 0, "deps ok"),
        build_runner=lambda root: (calls.append("build") or 0, "build ok"),
        on_restart=lambda: calls.append("restart"),
    )
    assert manager.start() is True
    state = wait_for_terminal(manager)
    assert state["status"] == "done"
    assert calls == ["deps", "build", "restart"]


def test_failed_build_rolls_back_to_previous_version(tmp_path):
    _remote, seed, clone = make_remote(tmp_path)
    old_sha = git(clone, "rev-parse", "HEAD")
    (seed / "README.md").write_text("broken version\n", encoding="utf-8")
    git(seed, "add", "README.md")
    git(seed, "commit", "-m", "broken update")
    git(seed, "push", "origin", "main")

    manager = UpdateManager(
        clone, branch="main",
        dependency_installer=lambda root: (0, "deps ok"),
        build_runner=lambda root: (1, "SyntaxError: broken"),
    )
    assert manager.start() is True
    state = wait_for_terminal(manager)
    assert state["status"] == "error"
    assert "не собралась" in state["error"]
    assert git(clone, "rev-parse", "HEAD") == old_sha
