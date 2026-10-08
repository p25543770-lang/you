"""Проверки веб-интерфейса RUS SLAM: страницы, API, замок отсека, статика."""

from __future__ import annotations

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
