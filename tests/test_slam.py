"""Проверки веб-интерфейса RUS SLAM: страницы, API, замок отсека, статика."""

from __future__ import annotations

import re

from conftest import login

DEFAULT_PIN = "2580"  # заводской PIN грузового отсека из backend.py


# ------------------------------- доступ ---------------------------------- #
def test_main_screen_requires_login(client):
    response = client.get("/")
    assert response.status_code == 302
    assert response.headers["Location"].endswith("/login?next=/")


def test_console_requires_login(client):
    response = client.get("/console")
    assert response.status_code == 302
    assert "/login" in response.headers["Location"]


def test_api_requires_login(client):
    assert client.get("/api/state").status_code == 302
    assert client.get("/api/health").status_code == 302


# ------------------------------- страницы -------------------------------- #
def test_main_screen_after_login(client, operator):
    login(client)
    response = client.get("/")
    assert response.status_code == 200
    text = response.get_data(as_text=True)
    # это основной экран робота из rus_slam, а не наша заглушка
    assert 'href="main.css"' in text
    assert 'src="main.js"' in text
    assert "Управление роботом ещё не подключено" not in text


def test_main_screen_also_at_main_html(client, operator):
    login(client)
    assert client.get("/main.html").status_code == 200


def test_console_after_login(client, operator):
    login(client)
    response = client.get("/console")
    assert response.status_code == 200
    text = response.get_data(as_text=True)
    for asset in ("console-core.js", "vision.js", "app.js", "console.js", "styles.css"):
        assert asset in text


def test_pages_not_cached(client, operator):
    login(client)
    assert client.get("/").headers["Cache-Control"] == "no-store"


# --------------------------------- API ----------------------------------- #
def test_api_state_shape(client, operator):
    login(client)
    body = client.get("/api/state").get_json()
    assert body["ok"] is True
    data = body["data"]
    assert data["source"] == "sim"
    assert len(data["motors"]) == 4
    for motor in data["motors"]:
        assert {"id", "title", "angle", "rpm", "temp", "homed"} <= set(motor)
    battery = data["battery"]
    assert {"soc", "volts", "amps", "rangeKm", "thresholds"} <= set(battery)
    assert 0 <= battery["soc"] <= 100
    assert data["cargo"]["closed"] is True
    assert data["lock"]["open"] is False


def test_api_health_reports_source(client, operator):
    login(client)
    body = client.get("/api/health").get_json()
    assert body["ok"] is True
    assert body["source"] == "sim"
    assert body["uptimeSec"] >= 0


def test_healthz_reports_interface(app):
    body = app.test_client().get("/healthz").get_json()
    assert body["panel"] == "rus_slam"
    assert body["source"] == "sim"


# --------------------------- грузовой отсек ------------------------------ #
def test_lock_opens_with_default_pin(client, operator):
    login(client)
    response = client.post("/api/lock/open", json={"pin": DEFAULT_PIN})
    assert response.status_code == 200
    assert response.get_json()["ok"] is True

    state = client.get("/api/state").get_json()["data"]
    assert state["lock"]["open"] is True
    assert state["cargo"]["closed"] is False


def test_lock_rejects_wrong_pin(client, operator):
    login(client)
    response = client.post("/api/lock/open", json={"pin": "0000"})
    assert response.status_code == 403
    assert response.get_json()["ok"] is False

    state = client.get("/api/state").get_json()["data"]
    assert state["lock"]["open"] is False
    assert state["lock"]["attemptsLeft"] == 4  # попытка учтена


def test_lock_closes(client, operator):
    login(client)
    client.post("/api/lock/open", json={"pin": DEFAULT_PIN})
    response = client.post("/api/lock/close")
    assert response.status_code == 200
    assert response.get_json()["ok"] is True
    assert client.get("/api/state").get_json()["data"]["cargo"]["closed"] is True


def test_audit_records_attempts(client, operator):
    login(client)
    client.post("/api/lock/open", json={"pin": "0000"})
    body = client.get("/api/audit?limit=5").get_json()
    assert body["ok"] is True
    assert body["audit"], "журнал пуст"
    assert body["integrity"]["ok"] is True  # хеш-цепочка не повреждена


# -------------------------------- статика -------------------------------- #
def test_assets_are_served(client, operator):
    login(client)
    for name in ("main.css", "main.js", "styles.css", "app.js",
                 "console.js", "console-core.js", "vision.js"):
        response = client.get(f"/{name}")
        assert response.status_code == 200, name
        assert len(response.data) > 100, name


def test_assets_are_public_without_login(client):
    """CSS/JS отдаются без входа: в iframe-предпросмотре cookie нет, и подзапросы
    не могут передать сессию — иначе страница приезжает сломанной."""
    for name in ("main.css", "main.js", "styles.css", "app.js",
                 "console.js", "console-core.js", "vision.js", "slam_auth.js"):
        assert client.get(f"/{name}").status_code == 200, name


def test_data_endpoints_still_closed_without_login(client):
    assert client.get("/api/state").status_code == 302
    assert client.get("/").status_code == 302
    assert client.get("/console").status_code == 302


def test_auth_shim_is_loaded_before_page_scripts(client, operator):
    """Скрипт, пробрасывающий токен st в fetch, должен подключаться первым.

    Сравниваем именно теги <script src=…>: слова вроде "main.js" встречаются
    в тексте страницы и в комментариях.
    """
    def order_of(text: str) -> list[str]:
        return re.findall(r'<script src="([^"]+)"', text)

    login(client)
    main_scripts = order_of(client.get("/").get_data(as_text=True))
    assert "/slam_auth.js" in main_scripts
    assert main_scripts.index("/slam_auth.js") < main_scripts.index("main.js")

    console_scripts = order_of(client.get("/console").get_data(as_text=True))
    assert "/slam_auth.js" in console_scripts
    assert console_scripts.index("/slam_auth.js") < console_scripts.index("console-core.js")


def test_unknown_asset_is_404(client, operator):
    login(client)
    assert client.get("/backend.py").status_code == 404
    assert client.get("/UPSTREAM_COMMIT.txt").status_code == 404


# ------------------------------- заголовки ------------------------------- #
def test_csp_allows_inline_styles_for_console(client, operator):
    """В index.html есть атрибуты style="…" — CSP не должен их резать."""
    login(client)
    policy = client.get("/console").headers["Content-Security-Policy"]
    assert "style-src 'self' 'unsafe-inline'" in policy
    assert "script-src 'self'" in policy


def test_exit_link_logs_out(client, operator):
    login(client)
    assert client.get("/").status_code == 200
    assert client.get("/exit").status_code == 302
    assert client.get("/").status_code == 302  # интерфейс снова закрыт


# ------------------------ основной экран (киоск) ------------------------- #
def test_main_screen_has_no_cargo_compartment(client, operator):
    """Грузовой отсек с PIN-клавиатурой убран с главного экрана. Ячейка и /api/lock
    остались у инженерного пульта (их проверяют тесты замка выше)."""
    login(client)
    text = client.get("/").get_data(as_text=True)
    for phrase in ("Грузовой отсек", "Открыть отсек", "Введите PIN"):
        assert phrase not in text, phrase
    assert not re.search(r"\bPIN\b", text)

    js = client.get("/main.js").get_data(as_text=True)
    assert "api/lock" not in js
    assert "api/audit" not in js
    assert "cargo" not in js
    assert not re.search(r"\b(lock|pin|PIN)\b", js)


def test_main_screen_shows_four_drive_modules(client, operator):
    login(client)
    text = client.get("/").get_data(as_text=True)
    for module in ("FL", "FR", "RL", "RR"):
        assert f'id="mod-{module}"' in text, module
        assert f'id="wheel-{module}"' in text, module


def test_main_screen_has_logout_and_console_links(client, operator):
    login(client)
    text = client.get("/").get_data(as_text=True)
    assert 'href="/exit"' in text
    assert 'href="/console"' in text


def test_main_screen_side_menu_points_to_real_sections(client, operator):
    """Меню слева: два пункта — «Пульт» (наверх экрана) и «Сервис» (инженерный пульт);
    якоря, если появятся, должны вести к существующим разделам экрана."""
    login(client)
    text = client.get("/").get_data(as_text=True)
    for label in ("Пульт", "Сервис"):
        assert label in text, label
    for label in ("Ходовая", "Миссия", "Сенсоры", "Безопасность", "Журнал"):
        assert f'<span class="nav-text">{label}</span>' not in text, label
    for target in re.findall(r'href="#([^"]+)"', text):
        assert f'id="{target}"' in text, target


def test_main_screen_polls_state_with_relative_url(client, operator):
    """Относительный путь нужен, чтобы экран работал и под /main.html,
    и за прокси предпросмотра, где префикс может отличаться."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "fetch('api/state'" in js
    assert "'/api/state'" not in js


def test_main_screen_demo_only_on_request(client, operator):
    """Выдуманные данные не подменяют отказ сервера: демо только при ?demo=1."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "get('demo') === '1'" in js


def test_main_screen_keeps_session_token_for_console(client, operator):
    """Ссылка на пульт сохраняет st, иначе в iframe-предпросмотре вход потеряется."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "'/console?st='" in js


def _load_backend():
    """slam_gui/backend.py грузится по пути, как в robot_control/slam.py."""
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "slam_gui" / "backend.py"
    spec = importlib.util.spec_from_file_location("rus_slam_backend_test", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scan_points_skips_bad_beams_and_normalizes_angles():
    """LaserScan -> точки: битые лучи пропускаются, угол в [-pi, pi]."""
    import math
    from types import SimpleNamespace

    backend = _load_backend()
    msg = SimpleNamespace(
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.1,
        range_max=10.0,
        ranges=[2.0, float("inf"), float("nan"), 0.05, 4.0, 12.0],
    )
    pts = backend.scan_points(msg)
    assert [p["r"] for p in pts] == [2.0, 4.0]
    # луч с индексом 4: 4 * pi/2 = 2pi, нормализован к 0
    assert abs(pts[1]["a"]) < 1e-3
    assert all(-math.pi <= p["a"] <= math.pi for p in pts)


def test_ros_source_exposes_fresh_lidar_only():
    """В /api/state попадает свежий скан; старый или отсутствующий даёт None."""
    import time

    backend = _load_backend()
    src = object.__new__(backend.RosSource)
    src.ok = True
    src.fallback = None
    src.data = {}
    src.scan = [{"a": 0.0, "r": 2.5}]
    src.scan_at = time.time()
    assert src.read()["lidar"] == [{"a": 0.0, "r": 2.5}]

    src.scan_at = time.time() - 10
    assert src.read()["lidar"] is None


def test_main_screen_manual_drive_toggle_on_r(client, operator):
    """R включает режим WASD на главном экране: лидар обводится красным, команда не уходит на сервер."""
    login(client)
    js = client.get("/main.js").get_data(as_text=True)
    assert "'KeyR'" in js and "'KeyW'" in js and "'Space'" in js
    assert "fetch(" not in js.split("function driveKey", 1)[1].split("function boot", 1)[0]
    css = client.get("/main.css").get_data(as_text=True)
    assert ".lidar.is-control" in css


def test_master_line_parses_tlm_and_ignores_text():
    """Строка "@TLM ..." разбирается; обычный текст монитора мастера игнорируется."""
    backend = _load_backend()
    frame = backend.parse_master_line(
        "@TLM mod=fl deg=-9.25 tgt=0.00 moving=1 cal=1 cycle=0 calib=0 opto=0 t=12345\r\n")
    assert frame["mod"] == "FL"
    assert frame["deg"] == -9.25
    assert frame["moving"] is True and frame["cal"] is True
    assert frame["cycle"] is False and frame["opto"] is False
    assert backend.parse_master_line("Цель достигнута. Угол: 12.00°") is None
    assert backend.parse_master_line("@TLM mod=FL deg=abc") is None


def test_master_source_reports_each_module_and_empty_for_silent_ones():
    """Мастер передаёт FL и узлы FR/RL/RR; модуль, который молчит, — пустой, без выдумок."""
    backend = _load_backend()
    src = backend.MasterSource("/dev/null")                 # порт не открывается в тесте
    src.feed("@TLM mod=FL deg=30.00 tgt=30.00 moving=0 cal=1 cycle=0 calib=0 opto=0 t=1", now=100.0)
    src.feed("@TLM mod=FR deg=-5.50 tgt=0.00 moving=1 cal=0 cycle=0 calib=1 opto=0 t=2", now=100.0)
    out = src.read(now=100.5)
    fl = next(m for m in out["motors"] if m["id"] == "FL")
    fr = next(m for m in out["motors"] if m["id"] == "FR")
    rl = next(m for m in out["motors"] if m["id"] == "RL")
    assert fl["angle"] == 30.0 and fl["online"] is True and fl["homed"] is True
    assert fl["rpm"] is None and fl["temp"] is None
    assert fr["angle"] == -5.5 and fr["moving"] is True and fr["homed"] is False
    assert rl["online"] is False and rl["angle"] is None
    assert out["linkOk"] is True and out["mode"] == "МАСТЕР НА СВЯЗИ"
    assert out["battery"]["volts"] is None and out["speedMps"] is None
    stale = src.read(now=102.0)                             # старше STALE_S — связи нет
    assert stale["linkOk"] is False
    assert all(m["online"] is False for m in stale["motors"])
