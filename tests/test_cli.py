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


def test_read_password_from_stdin(monkeypatch, cli):
    """Неинтерактивный ввод для скриптов: две строки из stdin."""
    import io
    import sys

    monkeypatch.setattr(sys, "stdin", io.StringIO("секрет-1\nсекрет-1\n"))
    assert cli.read_password(from_stdin=True) == "секрет-1"


def test_read_password_from_stdin_rejects_mismatch(monkeypatch, cli):
    import io
    import sys

    monkeypatch.setattr(sys, "stdin", io.StringIO("один\nдва\n"))
    with pytest.raises(SystemExit) as error:
        cli.read_password(min_length=1, from_stdin=True)
    assert "не совпадают" in str(error.value)


def test_read_password_from_stdin_rejects_short(monkeypatch, cli):
    import io
    import sys

    monkeypatch.setattr(sys, "stdin", io.StringIO("1234\n1234\n"))
    with pytest.raises(SystemExit) as error:
        cli.read_password(from_stdin=True)
    assert "короче 8" in str(error.value)


def test_read_password_from_stdin_rejects_empty(monkeypatch, cli):
    import io
    import sys

    monkeypatch.setattr(sys, "stdin", io.StringIO(""))
    with pytest.raises(SystemExit) as error:
        cli.read_password(from_stdin=True)
    assert "stdin пуст" in str(error.value)


def test_main_with_password_stdin(tmp_path, monkeypatch, cli):
    """Сквозная проверка: скрипт создаёт учётку из pipe без терминала."""
    import io
    import sys

    target = tmp_path / "operators.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "create_operator.py",
            "--username",
            "pipeop",
            "--file",
            str(target),
            "--min-length",
            "4",
            "--password-stdin",
        ],
    )
    monkeypatch.setattr(sys, "stdin", io.StringIO("1234\n1234\n"))
    assert cli.main() == 0

    from robot_control.auth import verify_password

    operator = cli.OperatorStore(target).get("pipeop")
    assert operator is not None
    assert verify_password("1234", operator.password_hash)


def test_add_or_update_stores_hash_not_password(tmp_path, cli):
    from robot_control.auth import verify_password

    store = cli.OperatorStore(tmp_path / "operators.json")
    store.add_or_update("admin", "admin", iterations=1_000)
    operator = store.get("admin")
    assert operator.password_hash.startswith("pbkdf2_sha256$")
    assert "admin" not in operator.password_hash
    assert verify_password("admin", operator.password_hash)


def _check_argv(target, *extra):
    return [
        "create_operator.py",
        "--username",
        "checked",
        "--file",
        str(target),
        "--check",
        *extra,
    ]


def test_check_accepts_correct_password(tmp_path, monkeypatch, cli, capsys):
    """--check: верный пароль подтверждается, учётка не меняется."""
    import io
    import sys

    target = tmp_path / "operators.json"
    before = cli.OperatorStore(target).add_or_update("checked", "верный-пароль", iterations=1_000)
    monkeypatch.setattr(sys, "argv", _check_argv(target, "--password-stdin"))
    monkeypatch.setattr(sys, "stdin", io.StringIO("верный-пароль\n"))
    assert cli.main() == 0
    assert "верный" in capsys.readouterr().out
    after = cli.OperatorStore(target).get("checked")
    assert after.password_hash == before.password_hash  # ничего не перезаписано


def test_check_rejects_wrong_password(tmp_path, monkeypatch, cli, capsys):
    import io
    import sys

    target = tmp_path / "operators.json"
    cli.OperatorStore(target).add_or_update("checked", "верный-пароль", iterations=1_000)
    monkeypatch.setattr(sys, "argv", _check_argv(target, "--password-stdin"))
    monkeypatch.setattr(sys, "stdin", io.StringIO("не-тот\n"))
    assert cli.main() == 1
    assert "НЕ совпадает" in capsys.readouterr().out


def test_check_unknown_login(tmp_path, monkeypatch, cli, capsys):
    import io
    import sys

    target = tmp_path / "operators.json"
    cli.OperatorStore(target).add_or_update("someone", "пароль-123", iterations=1_000)
    monkeypatch.setattr(sys, "argv", _check_argv(target, "--password-stdin"))
    monkeypatch.setattr(sys, "stdin", io.StringIO("пароль-123\n"))
    assert cli.main() == 1
    assert "нет в" in capsys.readouterr().out


def test_check_disabled_account(tmp_path, monkeypatch, cli, capsys):
    import io
    import sys

    target = tmp_path / "operators.json"
    store = cli.OperatorStore(target)
    operator = store.add_or_update("checked", "верный-пароль", iterations=1_000)
    store.save([type(operator)(operator.username, operator.role, operator.password_hash, False)])
    monkeypatch.setattr(sys, "argv", _check_argv(target, "--password-stdin"))
    monkeypatch.setattr(sys, "stdin", io.StringIO("верный-пароль\n"))
    assert cli.main() == 1
    assert "ОТКЛЮЧЕНА" in capsys.readouterr().out


def test_log_operators_lists_accounts(tmp_path, capsys):
    """Стартовый листинг: сервер печатает учётки, которые реально увидит."""
    import run as entry
    from robot_control.auth import OperatorStore
    from robot_control.config import Config

    config = Config(operators_file=tmp_path / "operators.json")
    entry.log_operators(config)  # пусто — молчим
    assert capsys.readouterr().out == ""

    OperatorStore(config.operators_file).add_or_update("admin", "1234", iterations=1_000)
    entry.log_operators(config)
    out = capsys.readouterr().out
    assert "admin" in out and "активна" in out


def test_log_operators_flags_disabled(tmp_path, capsys):
    import run as entry
    from robot_control.auth import OperatorStore
    from robot_control.config import Config

    config = Config(operators_file=tmp_path / "operators.json")
    store = OperatorStore(config.operators_file)
    operator = store.add_or_update("admin", "1234", iterations=1_000)
    store.save([type(operator)(operator.username, operator.role, operator.password_hash, False)])
    entry.log_operators(config)
    assert "ОТКЛЮЧЕНА" in capsys.readouterr().out
