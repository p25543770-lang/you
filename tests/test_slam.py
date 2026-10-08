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


# --------------------------- приборная оснастка --------------------------- #
def test_console_loads_instrumentation(client, operator):
    """Инженерный пульт подключает приборную оснастку и она отдаётся."""
    login(client)
    text = client.get("/console").get_data(as_text=True)
    assert 'src="instrument.js"' in text

    asset = client.get("/instrument.js")
    assert asset.status_code == 200
    body = asset.get_data(as_text=True)
    # оснастка живёт на тех же данных, что и экраны
    assert "api/state" in body
    # и не трогает чужие элементы — только свои, с префиксом ins-
    assert "ins-" in body


def test_main_screen_loads_instrumentation(client, operator):
    """Приборная линейка нужна и на основном экране робота."""
    login(client)
    text = client.get("/").get_data(as_text=True)
    assert 'src="instrument.js"' in text
    assert 'id="ins-lamp-link"' in text


def test_state_reports_bus_diagnostics(client, operator):
    """Состояние несёт счётчики обмена — по ним считается диагностика."""
    login(client)
    data = client.get("/api/state").get_json()["data"]
    bus = data["bus"]
    assert {"framesOk", "discarded", "ageMs", "online", "modules"} <= set(bus)
    # демонстрационный источник обязан это признавать: панель диагностики
    # не должна выдавать модель за реальный обмен с модулями
    assert bus["simulated"] is True
    assert bus["online"] == 4


def test_console_state_toggles_have_style_rules():
    """Классы, которые скрипты ставят на <body>, должны быть оформлены.

    Регрессия: кнопки «Тема», «Крупно» и режим «стенд» существовали, но
    правил для этих состояний в таблице стилей не было — переключатели
    работали вхолостую и выглядели сломанными.
    """
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent / "slam_gui"
    css = (root / "styles.css").read_text(encoding="utf-8")

    toggled = set()
    for name in ("app.js", "console.js", "main.js"):
        script = (root / name).read_text(encoding="utf-8")
        # только переключения на самом документе: body.classList.toggle('x', …)
        for match in re.finditer(
            r"document\.body\.classList\.(?:toggle|add|remove)\(\s*['\"]([a-z-]+)['\"]",
            script,
        ):
            toggled.add(match.group(1))

    assert toggled, "не найдено ни одного переключателя классов на <body>"
    missing = sorted(c for c in toggled if f"body.{c}" not in css)
    assert not missing, f"у состояний нет правил в styles.css: {missing}"


def test_main_screen_has_no_blind_lock_controls():
    """С киоска отсек нельзя открыть «вслепую».

    Регрессия: обработчик физической клавиатуры вызывал открытие отсека по
    Enter безусловно. Убери панель с экрана, но оставь обработчик — и замок
    открывался бы набором цифр без единого органа управления на виду.
    Поэтому: панели на киоске нет, а обработчик клавиш подключается только
    при наличии клавиатуры набора, и открытие остаётся на инженерном пульте.
    """
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent / "slam_gui"
    main_html = (root / "main.html").read_text(encoding="utf-8")

    # на основном экране нет ни клавиатуры набора, ни кнопки отсека
    assert 'id="sc-keypad"' not in main_html
    assert 'id="sc-btn-open"' not in main_html
    assert 'id="sc-pin-dots"' not in main_html

    # обработчик клавиш живёт внутри проверки наличия клавиатуры
    script = (root / "main.js").read_text(encoding="utf-8")
    guarded = re.search(
        r"getElementById\('sc-keypad'\);\s*\n\s*if \(keypad\) \{(.+?)\n    \}",
        script,
        re.S,
    )
    assert guarded, "обработчик набора PIN не привязан к наличию клавиатуры"
    assert "press(e.key)" in guarded.group(1)
    assert "toggleCargo()" in guarded.group(1)

    # сама возможность открыть отсек сохранена — на инженерном пульте
    console = (root / "index.html").read_text(encoding="utf-8")
    assert 'id="csl-keypad"' in console
    assert 'id="csl-pin-open"' in console
