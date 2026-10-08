"""Общие фикстуры: приложение и хранилище операторов во временной папке."""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_control.app import create_app  # noqa: E402
from robot_control.auth import OperatorStore  # noqa: E402
from robot_control.config import Config  # noqa: E402

OPERATOR = "operator"
PASSWORD = "correct-horse-battery"
FAST_ITERATIONS = 1_000  # тесты не должны тормозить на PBKDF2


@pytest.fixture
def config(tmp_path: Path) -> Config:
    return Config(
        secret_key="test-secret-key",
        operators_file=tmp_path / "operators.json",
        bind_host="127.0.0.1",
        port=8080,
        max_failed_attempts=3,
        lockout_seconds=300,
        pbkdf2_iterations=FAST_ITERATIONS,
        testing=True,
    )


@pytest.fixture
def store(config: Config) -> OperatorStore:
    return OperatorStore(config.operators_file)


@pytest.fixture
def operator(store: OperatorStore, config: Config):
    return store.add_or_update(OPERATOR, PASSWORD, iterations=config.pbkdf2_iterations)


@pytest.fixture
def app(config: Config):
    application = create_app(config)
    application.config.update(TESTING=True)
    return application


@pytest.fixture
def client(app):
    return app.test_client()


def csrf_token(response) -> str:
    """Достаём CSRF-токен из отрендеренной формы."""
    match = re.search(r'name="csrf_token" value="([^"]+)"', response.get_data(as_text=True))
    assert match, "в ответе нет CSRF-токена"
    return match.group(1)


def login(client, username=OPERATOR, password=PASSWORD):
    page = client.get("/login")
    return client.post(
        "/login",
        data={
            "username": username,
            "password": password,
            "csrf_token": csrf_token(page),
        },
        follow_redirects=False,
    )
