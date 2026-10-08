"""Проверки HTTP-части: вход оператора, доступ к интерфейсу робота."""

from __future__ import annotations

from conftest import OPERATOR, PASSWORD, csrf_token, login


def test_healthz_is_public(client):
    response = client.get("/healthz")
    assert response.status_code == 200
    body = response.get_json()
    assert body["status"] == "ok"
    assert body["panel"] == "rus_slam"
    assert body["source"]  # выбран источник данных (sim, serial или ros)


def test_panel_requires_login(client):
    response = client.get("/")
    assert response.status_code == 302
    assert response.headers["Location"].endswith("/login?next=/")


def test_login_page_shows_form(client):
    response = client.get("/login")
    assert response.status_code == 200
    text = response.get_data(as_text=True)
    assert "Вход оператора" in text
    assert 'name="password"' in text


def test_login_without_operators_file(client):
    response = login(client, "operator", "любой-пароль")
    assert response.status_code == 401
    assert "create_operator.py" in response.get_data(as_text=True)


def test_login_wrong_password(app, client, operator):
    response = login(client, OPERATOR, "не-тот-пароль")
    assert response.status_code == 401
    assert "Неверный логин или пароль" in response.get_data(as_text=True)


def test_login_success_gives_session_and_redirect(app, client, operator):
    response = login(client)
    assert response.status_code == 302
    location = response.headers["Location"]
    assert "/?just_logged_in=1" in location
    assert "&st=" in location  # резервная URL-сессия для сред без cookie

    panel = client.get("/")
    assert panel.status_code == 200
    with client.session_transaction() as session:
        assert session["operator"] == OPERATOR
        assert session["role"] == "operator"

    cookie = client.get_cookie("rc_session")
    assert cookie is not None
    assert cookie.http_only is True
    assert cookie.same_site == "Lax"
    assert cookie.secure is False  # RC_COOKIE_SECURE не включён


def test_panel_after_login_is_a_stub(client, operator):
    """Служебная страница /panel — по-прежнему без органов управления."""
    login(client)
    text = client.get("/panel").get_data(as_text=True)
    assert OPERATOR in text
    assert "Управление роботом ещё не подключено" in text
    # никаких органов управления в заглушке быть не должно
    for forbidden in ('name="throttle"', 'name="steer"', 'id="joystick"', "/api/robot"):
        assert forbidden not in text


def test_sidebar_shows_only_user_at_bottom(client, operator):
    """Левая колонка: сверху бренд, в середине пусто, внизу — только пользователь."""
    login(client)
    text = client.get("/panel").get_data(as_text=True)
    assert 'class="sidebar"' in text
    assert 'class="user-card"' in text
    assert 'class="user-name" title="operator">operator' in text.replace("\n", "")
    assert "оператор" in text
    # прежней верхней панели и нижнего футера больше нет
    assert 'class="topbar"' not in text
    assert 'class="footer"' not in text
    # середина колонки пустая: между брендом и пользователем нет пунктов меню
    import re

    sidebar = re.search(r'<aside class="sidebar">(.*?)</aside>', text, re.S).group(1)
    assert "New Chat" not in sidebar
    assert 'class="menu"' not in sidebar
    assert "<nav" not in sidebar


def test_no_sidebar_before_login(client):
    """Панель-колонка появляется только после входа — до входа её нет."""
    text = client.get("/login").get_data(as_text=True)
    assert 'class="sidebar"' not in text
    assert "user-card" not in text
    assert "вход не выполнен" not in text
    # форма входа при этом на месте и центрирована
    assert 'name="password"' in text
    assert "content-full" in text


def test_sidebar_appears_after_login(client, operator):
    login(client)
    text = client.get("/panel").get_data(as_text=True)
    assert 'class="sidebar"' in text
    assert "content-full" not in text


def test_logout_clears_session(client, operator):
    login(client)
    assert client.get("/").status_code == 200

    page = client.get("/panel")
    response = client.post(
        "/logout", data={"csrf_token": csrf_token(page)}, follow_redirects=False
    )
    assert response.status_code == 302
    assert client.get("/").status_code == 302  # панель снова закрыта


def test_csrf_token_is_required(client, operator):
    client.get("/login")
    response = client.post(
        "/login", data={"username": OPERATOR, "password": PASSWORD, "csrf_token": "подделка"}
    )
    assert response.status_code == 400
    with client.session_transaction() as session:
        assert "operator" not in session


def test_stale_csrf_shows_form_again_and_retry_works(client, operator):
    """Устаревший токен (сервер перезапускали) — не тупик: форма со свежим
    токеном возвращается, и повторный ввод проходит."""
    stale = client.post(
        "/login",
        data={"username": OPERATOR, "password": PASSWORD, "csrf_token": "был.до.перезапуска"},
    )
    assert stale.status_code == 400
    text = stale.get_data(as_text=True)
    assert "Форма устарела" in text
    assert 'name="password"' in text          # форму можно заполнить заново
    assert OPERATOR in text                    # введённый логин не потерялся

    fresh_token = csrf_token(stale)
    assert fresh_token != "был.до.перезапуска"

    retry = client.post(
        "/login",
        data={
            "username": OPERATOR,
            "password": PASSWORD,
            "csrf_token": fresh_token,
        },
    )
    assert retry.status_code == 302            # вход состоялся


def test_failed_attempts_are_limited(app, client, operator):
    limit = app.extensions["rc_config"].max_failed_attempts
    for _ in range(limit):
        assert login(client, OPERATOR, "не-тот-пароль").status_code == 401
    blocked = login(client, OPERATOR, "не-тот-пароль")
    assert blocked.status_code == 429
    # даже верный пароль в период блокировки не пускаем
    assert login(client, OPERATOR, PASSWORD).status_code == 429


def test_unknown_role_in_session_is_rejected(app, client, operator):
    login(client)
    with client.session_transaction() as session:
        session["role"] = "viewer"
    response = client.get("/")
    assert response.status_code == 302
    assert "/login" in response.headers["Location"]


def test_external_next_is_ignored(client, operator):
    response = client.post(
        "/login?next=http://evil.example/steal",
        data={
            "username": OPERATOR,
            "password": PASSWORD,
            "csrf_token": csrf_token(client.get("/login")),
        },
    )
    assert response.status_code == 302
    assert response.headers["Location"].startswith("/")
    assert "evil.example" not in response.headers["Location"]


def test_security_headers_present(client):
    headers = client.get("/login").headers
    assert headers["X-Frame-Options"] == "DENY"
    assert headers["X-Content-Type-Options"] == "nosniff"
    assert headers["Content-Security-Policy"].startswith("default-src 'self'")
    assert headers["Cache-Control"] == "no-store"


def test_csrf_token_works_without_session_cookie(app, client, operator):
    """Stateless-токен: вход проходит, даже если браузер не сохранил cookie.

    Снимаем токен на одном клиенте, а POST шлём с нового — без cookie сессии.
    """
    token = csrf_token(client.get("/login"))
    fresh = app.test_client()
    response = fresh.post(
        "/login",
        data={"username": OPERATOR, "password": PASSWORD, "csrf_token": token},
        follow_redirects=False,
    )
    assert response.status_code == 302


def test_cookie_loss_shows_hint(client):
    """Вход был, а сессия не доехала — показываем понятную подсказку."""
    response = client.get("/?just_logged_in=1")
    assert response.status_code == 302
    assert "nocookie=1" in response.headers["Location"]

    page = client.get("/login?nocookie=1")
    assert page.status_code == 200
    assert "не сохранил cookie" in page.get_data(as_text=True)


def _path_of(location: str) -> str:
    from urllib.parse import urlparse

    parsed = urlparse(location)
    return parsed.path + ("?" + parsed.query if parsed.query else "")


def test_url_session_allows_cookieless_dashboard(app, client, operator):
    """iframe без cookie: токен st в URL даёт доступ к панели и выходу."""
    location = login(client).headers["Location"]
    assert "&st=" in location

    fresh = app.test_client()  # без cookie вообще
    page = fresh.get(_path_of(location))
    assert page.status_code == 200
    # по URL-сессии открылся экран робота
    assert "main.css" in page.get_data(as_text=True)

    token = _path_of(location).split("st=", 1)[1]
    panel = fresh.get(f"/panel?st={token}")
    assert panel.status_code == 200
    text = panel.get_data(as_text=True)
    assert 'class="sidebar"' in text

    # Кнопка выхода в ссылке несёт тот же токен st
    import re

    action = re.search(r'action="([^"]+)" class="inline-form"', text).group(1)
    assert "st=" in action

    # Выход по токену работает без cookie
    out = fresh.post(f"/logout?st={token}", data={"csrf_token": _csrf_from(text)})
    assert out.status_code == 302
    # После выхода панель снова закрыта
    assert fresh.get(f"/?st={token}").status_code == 302


def test_url_session_rejects_tampered_token(app, client, operator):
    location = login(client).headers["Location"]
    path = _path_of(location)
    tampered = path.replace("st=", "st=X", 1)
    fresh = app.test_client()
    response = fresh.get(tampered)
    assert response.status_code == 302  # токен невалиден — на логин
    assert "/login" in response.headers["Location"]


def _csrf_from(text: str) -> str:
    import re

    return re.search(r'name="csrf_token" value="([^"]+)"', text).group(1)


def test_samesite_none_forces_secure():
    from robot_control.app import create_app
    from robot_control.config import Config

    app = create_app(Config(secret_key="k", cookie_samesite="None", testing=True))
    assert app.config["SESSION_COOKIE_SAMESITE"] == "None"
    assert app.config["SESSION_COOKIE_SECURE"] is True


def test_default_samesite_is_lax():
    from robot_control.app import create_app
    from robot_control.config import Config

    app = create_app(Config(secret_key="k", testing=True))
    assert app.config["SESSION_COOKIE_SAMESITE"] == "Lax"
    assert app.config["SESSION_COOKIE_SECURE"] is False


def test_500_handler_shows_readable_page(app, client):
    app.config.update(TESTING=False)  # иначе Flask пробрасывает исключение в тест

    @app.route("/boom")
    def boom():
        raise RuntimeError("старый процесс + новые шаблоны")

    response = client.get("/boom")
    assert response.status_code == 500
    text = response.get_data(as_text=True)
    assert "500" in text
    assert "Перезапустите сервер" in text


def test_unknown_page_returns_404(client):
    assert client.get("/нет-такой-страницы").status_code == 404
    api = client.get("/api/robot/state")
    assert api.status_code == 404
    assert api.get_json() == {"error": "not_found"}


# --------------------------- режим без пароля ---------------------------- #
# По умолчанию (RC_REQUIRE_LOGIN=0) пульт открыт: пароля нет ни у основного
# экрана, ни у инженерного пульта, ни у служебной страницы. Парольный режим
# проверяют тесты выше — они собирают приложение с require_login=True.


def test_default_config_has_no_password():
    from robot_control.config import Config

    assert Config(secret_key="k").require_login is False


def test_open_mode_pages_do_not_ask_for_password(open_client):
    for path in ("/", "/console", "/index.html", "/main.html", "/panel"):
        response = open_client.get(path)
        assert response.status_code == 200, f"{path} требует вход"
        assert 'name="password"' not in response.get_data(as_text=True)


def test_open_mode_login_page_leads_to_the_screen(open_client):
    """/login без пароля не тупик: сразу уводит на экран робота."""
    response = open_client.get("/login")
    assert response.status_code == 302
    assert not response.headers["Location"].startswith("/login")


def test_open_mode_healthz_reports_auth_off(open_client):
    body = open_client.get("/healthz").get_json()
    assert body["auth"] == "off"


def test_open_mode_operator_name_is_visible(open_client):
    """Даже без пароля видно, от чьего имени идёт работа."""
    text = open_client.get("/panel").get_data(as_text=True)
    assert "без пароля" in text


def test_open_mode_exit_returns_to_the_screen(open_client):
    response = open_client.get("/exit")
    assert response.status_code == 302
    assert response.headers["Location"].endswith("/")


def test_password_mode_is_turned_on_by_env(monkeypatch, tmp_path):
    from robot_control.config import Config

    monkeypatch.setenv("RC_REQUIRE_LOGIN", "1")
    monkeypatch.setenv("RC_OPERATORS_FILE", str(tmp_path / "ops.json"))
    config = Config.from_env(project_root=tmp_path)
    assert config.require_login is True

    monkeypatch.setenv("RC_REQUIRE_LOGIN", "0")
    assert Config.from_env(project_root=tmp_path).require_login is False
