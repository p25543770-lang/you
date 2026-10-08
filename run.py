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
import sys

from robot_control.config import Config

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s"
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
    config = Config.from_env()
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
