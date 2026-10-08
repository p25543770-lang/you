#!/usr/bin/env python3
"""Точка входа веб-интерфейса.

По умолчанию поднимает production WSGI-сервер gunicorn (без предупреждения
Flask про development server):

    python3 run.py

Для отладки с релоадом — режим разработчика:

    python3 run.py --dev

Для боевой установки используйте systemd-юнит scripts/robot-control.service
(он дергает этот же run.py).
"""

from __future__ import annotations

import importlib.util
import logging
import os
import secrets
import sys
from pathlib import Path

from robot_control.config import Config

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s"
)

PROJECT_ROOT = Path(__file__).resolve().parent

DEFAULT_OPERATOR = "admin"
DEFAULT_PASSWORD = "1234"


def ensure_env_file() -> None:
    """Создаёт .env с постоянным RC_SECRET_KEY, если его нет.

    Без файла ключ генерируется заново при каждом запуске: сессии слетают, а
    открытые ранее страницы входа получают недействительный CSRF-токен.
    """
    env_path = PROJECT_ROOT / ".env"
    if env_path.is_file():
        return
    env_path.write_text(
        "RC_SECRET_KEY={}\nRC_BIND_HOST=0.0.0.0\nRC_PORT=8080\n".format(
            secrets.token_urlsafe(48)
        ),
        encoding="utf-8",
    )
    os.chmod(env_path, 0o600)
    print("создан .env с новым RC_SECRET_KEY (права 600)")


def ensure_operator(config: Config) -> None:
    """Заводит учётку оператора, если ни одной нет.

    Иначе после потери data/operators.json вход отвергает любой логин-пароль,
    а причина («учёток нет вовсе») пользователю не видна.
    """
    from robot_control.auth import OperatorStore

    store = OperatorStore(config.operators_file)
    if store.exists():
        try:
            if store.load():
                return
        except (OSError, ValueError) as exc:
            print(f"файл учёткок не читается ({exc}) — создаю заново", file=sys.stderr)

    username = (os.environ.get("RC_OPERATOR") or DEFAULT_OPERATOR).strip()
    password = os.environ.get("RC_OPERATOR_PASSWORD") or DEFAULT_PASSWORD

    config.operators_file.parent.mkdir(parents=True, exist_ok=True)
    store.add_or_update(username, password, iterations=config.pbkdf2_iterations)

    credentials = config.operators_file.parent / "credentials.txt"
    credentials.write_text(
        f"login: {username}\npassword: {password}\n", encoding="utf-8"
    )
    os.chmod(credentials, 0o600)

    print(f"учёток не было — создана: {username} / {password}")
    if len(password) < 8:
        print(
            "  ⚠ пароль короткий: для точки доступа лучше длиннее "
            "(RC_OPERATOR_PASSWORD='...' в .env)"
        )



def run_gunicorn(config: Config) -> int:
    """Запустить gunicorn в текущем процессе (os.execv заменяет интерпретатор)."""
    argv = [
        sys.executable,
        "-m",
        "gunicorn",
        "--bind",
        f"{config.bind_host}:{config.port}",
        # Один процесс с пулом потоков: состояние (троттлинг входа, отзыв
        # URL-токенов) живёт в памяти, поэтому несколько процессов не плодим.
        "--workers",
        os.environ.get("RC_WORKERS", "1"),
        "--threads",
        os.environ.get("RC_THREADS", "4"),
        "--access-logfile",
        "-",
        "--error-logfile",
        "-",
        "robot_control.app:create_app()",
    ]
    print(
        f"Запуск gunicorn: http://{config.bind_host}:{config.port}/  "
        f"(файл операторов: {config.operators_file})"
    )
    # os.execv заменяет процесс вместе с буферами: без flush сообщения выше
    # (в том числе про созданную учётку) просто теряются.
    sys.stdout.flush()
    sys.stderr.flush()
    os.execv(sys.executable, argv)
    return 0  # недостижимо


def run_dev(config: Config) -> int:
    from robot_control.app import create_app

    app = create_app(config)
    print(
        f"[dev] Веб-интерфейс: http://{config.bind_host}:{config.port}/  "
        f"(файл операторов: {config.operators_file})"
    )
    # debug=False: Werkzeug-отладчик даёт RCE и не должен попадать на робота.
    app.run(host=config.bind_host, port=config.port, debug=False, use_reloader=False)
    return 0


def main() -> int:
    ensure_env_file()
    config = Config.from_env()
    ensure_operator(config)
    if "--dev" in sys.argv:
        return run_dev(config)
    if importlib.util.find_spec("gunicorn") is None:
        print(
            "gunicorn не установлен — запускаю встроенный dev-сервер Flask "
            "(только для отладки!). Поставьте зависимости: "
            "pip install -r requirements.txt",
            file=sys.stderr,
        )
        return run_dev(config)
    return run_gunicorn(config)


if __name__ == "__main__":
    sys.exit(main())
