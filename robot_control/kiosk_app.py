"""Отдельное приложение для дисплея на борту робота (без авторизации).

Корень / и /main.html сразу отдают экран киоска с АКБ, шасси и лидаром.
Никакого логина, никакого пульта оператора, никаких эндпоинтов управления.
"""
from __future__ import annotations

import logging
import os
import secrets
from pathlib import Path

from flask import Flask, jsonify, redirect

from robot_control.config import Config
from robot_control.slam import (
    ASSETS,
    CONSOLE_PAGE,
    GUI_DIR,
    KIOSK_PAGE,
    MAIN_PAGE,
    build_state,
)

log = logging.getLogger("robot_control.kiosk")


def create_kiosk_app(config: Config | None = None) -> Flask:
    config = config or Config.from_env()
    app = Flask(__name__)
    app.config["SECRET_KEY"] = config.secret_key or secrets.token_urlsafe(48)

    # Только источник данных — никакой авторизации, никаких операторов.
    try:
        state = build_state(
            mode=config.slam_source,
            ports=config.slam_ports,
            lock_file=config.slam_lock_file,
            pin=config.slam_pin,
            max_attempts=config.slam_max_attempts,
            lock_sec=config.slam_lock_seconds,
        )
        log.info("киоск: интерфейс подключён (источник: %s)",
                 getattr(state.source, "name", "?"))
    except Exception as exc:  # noqa: BLE001
        log.error("киоск: не удалось подключить источник: %s", exc)
        state = None

    def _send(name: str):
        from flask import send_from_directory
        response = send_from_directory(GUI_DIR, name)
        response.headers["Cache-Control"] = "no-store"
        return response

    # ---------- маршруты ----------
    @app.get("/")
    def root():
        return redirect("/kiosk")

    @app.get("/kiosk")
    def kiosk():
        return _send(KIOSK_PAGE)

    @app.get("/robot.html")
    def kiosk_alt():
        return _send(KIOSK_PAGE)

    @app.get("/api/state")
    @app.get("/api/kiosk_state")
    def api_state():
        if state is None:
            return jsonify({"ok": False, "error": "no source"}), 503
        snap = state.snapshot()
        # Только публичные показатели — АКБ, шасси, лидар, скорость/мощность/связь.
        return jsonify({"ok": True, "data": {
            "battery": snap.get("battery"),
            "motors": snap.get("motors"),
            "lidar": snap.get("lidar"),
            "speedMps": snap.get("speedMps"),
            "powerKw": snap.get("powerKw"),
            "mode": snap.get("mode"),
            "linkOk": snap.get("linkOk"),
            "ts": snap.get("ts"),
        }})

    @app.get("/healthz")
    def healthz():
        return jsonify({
            "ok": True,
            "mode": "kiosk",
            "source": getattr(getattr(state, "source", None), "name", None) if state else None,
        })

    # статика (robot.css, robot.js)
    for filename in ("robot.css", "robot.js"):
        # нужен closure, чтобы цикл не переиспользовал последнее значение
        def _view(name=filename):
            return _send(name)
        endpoint = f"asset_{filename.replace('.', '_')}"
        app.add_url_rule(f"/{filename}", endpoint=endpoint, view_func=_view)

    # заголовки безопасности (разрешены iframe для превью)
    @app.after_request
    def _headers(response):
        response.headers.setdefault("X-Content-Type-Options", "nosniff")
        response.headers.setdefault("Referrer-Policy", "no-referrer")
        response.headers.setdefault("X-Robots-Tag", "noindex, nofollow")
        response.headers.setdefault("Cache-Control", "no-store")
        return response

    return app


def main():
    # По умолчанию — реальные данные (auto: ROS → UART → мастер).
    # Симуляция включается явно: RC_SLAM_SOURCE=sim ./run_kiosk.py
    os.environ.setdefault("RC_SLAM_SOURCE", "auto")
    cfg = Config.from_env()
    host = os.environ.get("RC_BIND_HOST", "0.0.0.0")
    port = int(os.environ.get("RC_KIOSK_PORT", "8090"))
    app = create_kiosk_app(cfg)
    print(f"Экран робота (киоск) слушает на http://{host}:{port}/ (источник: auto)")
    app.run(host=host, port=port, debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
