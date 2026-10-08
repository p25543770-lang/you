"""Проверки CLI-скрипта создания учёткок (scripts/create_operator.py)."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "create_operator.py"


def load_cli():
    spec = importlib.util.spec_from_file_location("create_operator_cli", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def cli():
    return load_cli()


def feed(monkeypatch, cli, *answers):
    queue = iter(answers)
    monkeypatch.setattr(cli.getpass, "getpass", lambda prompt="": next(queue))


def test_read_password_accepts_long_password(monkeypatch, cli):
    feed(monkeypatch, cli, "пароль-123", "пароль-123")
    assert cli.read_password() == "пароль-123"


def test_read_password_rejects_short_by_default(monkeypatch, cli):
    feed(monkeypatch, cli, "admin", "admin")
    with pytest.raises(SystemExit) as error:
        cli.read_password()
    assert "короче 8" in str(error.value)


def test_read_password_allows_short_when_explicitly_asked(monkeypatch, cli, capsys):
    feed(monkeypatch, cli, "admin", "admin")
    assert cli.read_password(min_length=5) == "admin"
    assert "ВНИМАНИЕ" in capsys.readouterr().out


def test_read_password_rejects_mismatch(monkeypatch, cli):
    feed(monkeypatch, cli, "admin", "не-тот")
    with pytest.raises(SystemExit) as error:
        cli.read_password(min_length=5)
    assert "не совпадают" in str(error.value)


def test_add_or_update_stores_hash_not_password(tmp_path, cli):
    from robot_control.auth import verify_password

    store = cli.OperatorStore(tmp_path / "operators.json")
    store.add_or_update("admin", "admin", iterations=1_000)
    operator = store.get("admin")
    assert operator.password_hash.startswith("pbkdf2_sha256$")
    assert "admin" not in operator.password_hash
    assert verify_password("admin", operator.password_hash)
