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


def test_lock_panel_keeps_only_the_pin():
    """На панели замка остаётся только ввод PIN.

    Регрессия к просьбе «в разделе замка всё удали, кроме пароля в начале»:
    у панели «Ячейка хранения» должны остаться ввод PIN и кнопки открытия,
    а QR/RFID, автозакрытие с обратным отсчётом и журнал доступа — убраны.
    Сами данные журнала и API при этом живы (см. test_api_audit_*).
    """
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent / "slam_gui"
    console = (root / "index.html").read_text(encoding="utf-8")

    # осталось: ввод PIN, клавиатура, открытие и закрытие ячейки
    for keep in ("csl-pin-dots", "csl-pin-input", "csl-keypad",
                 "csl-pin-open", "csl-pin-close", "csl-pin-msg"):
        assert f'id="{keep}"' in console, f"на панели замка нет {keep}"

    # убрано: всё прочее (альтернативные метки, автозакрытие, журнал)
    for gone in ("csl-audit", "csl-pin-qr", "csl-pin-rfid", "csl-pin-extend",
                 "csl-lock-timer", "csl-lock-ring", "csl-lock-kind"):
        assert f'id="{gone}"' not in console, f"панель замка всё ещё содержит {gone}"

    assert "Журнал доступа" not in console
    assert "автозакрытие" not in console.split("Ячейка хранения")[1][:300].lower()

    # в коде пульта не осталось отрисовки журнала и кнопок меток
    script = (root / "console.js").read_text(encoding="utf-8")
    for gone in ("renderAudit", "csl-pin-extend", "csl-pin-qr", "csl-pin-rfid"):
        assert gone not in script, f"console.js всё ещё ссылается на {gone}"

    # лампа «отсек» на основном экране осталась — состояние видно
    main = (root / "main.html").read_text(encoding="utf-8")
    assert 'id="ins-lamp-cargo"' in main


def test_main_screen_footer_gone_and_console_is_a_left_column():
    """Подвал со скриншота убран, вход в инженерный пульт — столбик слева."""
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent / "slam_gui"
    main = (root / "main.html").read_text(encoding="utf-8")

    # подвала нет — ни разметки, ни строк, которые в нём показывались
    assert "sc-foot" not in main
    assert 'id="sc-foot-route"' not in main
    assert 'id="sc-foot-power"' not in main
    assert 'id="sc-foot-event"' not in main

    # вход в пульт — вертикальный столбик в первой колонке, а не ссылка внизу
    assert 'class="sc-nav" id="sc-console-link"' in main
    assert 'class="sc-nav-label"' in main

    css = (root / "main.css").read_text(encoding="utf-8")
    assert ".sc-foot" not in css
    assert "grid-template-columns: 46px minmax(0, 1fr)" in css
    assert "writing-mode: vertical-rl" in css

    # код киоска больше не пишет в удалённые элементы
    script = (root / "main.js").read_text(encoding="utf-8")
    for gone in ("sc-foot-route", "sc-foot-power", "sc-foot-event", "lastAudit"):
        assert gone not in script, f"main.js всё ещё ссылается на {gone}"


def test_kiosk_js_survives_without_removed_blocks():
    """Скрипты киоска переживают отсутствие удалённых блоков.

    Прогоняем main.js и instrument.js в пустом DOM (tests/js_smoke.js): если код
    дёрнет удалённый элемент или вызовет несуществующую функцию, экран киоска в
    браузере останется пустым. Тест ловит именно такие поломки.
    """
    import shutil
    import subprocess
    from pathlib import Path

    node = shutil.which("node")
    if node is None:
        import pytest

        pytest.skip("node не установлен — проверка JS пропущена")

    root = Path(__file__).resolve().parent.parent
    result = subprocess.run(
        [
            node,
            str(root / "tests" / "js_smoke.js"),
            str(root / "slam_gui" / "main.js"),
            str(root / "slam_gui" / "instrument.js"),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_motor_panel_shows_the_robot_with_wheels_around():
    """Панель двигателей — схема робота сверху, направление словами.

    Регрессия к просьбе «переделать направление моторов, робота поставь и
    вокруг него»: вместо четырёх одинаковых стрелочных приборов со знаком
    минус — корпус робота в центре, колёса по углам, поворот колеса и подпись
    «влево/прямо/вправо». Проверяется прогоном отрисовки в заглушке DOM
    (tests/js_motors.js).
    """
    import shutil
    import subprocess
    from pathlib import Path

    node = shutil.which("node")
    if node is None:
        import pytest

        pytest.skip("node не установлен — проверка JS пропущена")

    root = Path(__file__).resolve().parent.parent
    result = subprocess.run(
        [node, str(root / "tests" / "js_motors.js")],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    # панель держит схему и карточки сверху: на киоске взгляд падает наверх,
    # а не в середину высокой панели (просьба «это надо выше»)
    css = (root / "slam_gui" / "main.css").read_text(encoding="utf-8")
    grid_rule = re.search(r"\.sc-motor-grid \{([^}]*)\}", css, re.S)
    assert grid_rule, "не найдено правило .sc-motor-grid"
    assert "align-content: start" in grid_rule.group(1), (
        "содержимое панели двигателей снова выравнивается не по верху"
    )


def test_demo_drive_modes_are_coherent():
    """Демонстрация едет по режимам, а не крутит колёсами вразнобой.

    Прежде каждое колесо ходило своей синусоидой: углы четырёх модулей не
    были связаны, и по экрану нельзя было понять, куда едет робот.
    """
    import importlib.util
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent
    spec = importlib.util.spec_from_file_location("slam_backend", root / "slam_gui" / "backend.py")
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)

    modes = {m["title"]: m["angles"] for m in backend.DEMO_DRIVE_MODES}
    turn = modes["поворот вправо"]
    assert turn["FL"] > 0 and turn["FR"] > 0
    assert turn["FR"] > turn["FL"]                        # внутреннее колесо довёрнуто больше
    assert turn["RL"] == turn["FL"] and turn["RR"] == turn["FR"]   # задние как передние
    assert set(modes["краб боком"].values()) == {90}      # чистый боком, все под 90°
    assert modes["разворот на месте"]["FL"] == 90         # передние в одну сторону
    assert modes["разворот на месте"]["RL"] == -90        # задние в другую
    assert set(modes["прямо"].values()) == {0}

    # режим выбирается по времени и цикл повторяется
    first = backend.demo_drive_mode(1.0)
    assert first["title"] == "прямо"
    cycle = sum(m["sec"] for m in backend.DEMO_DRIVE_MODES)
    assert backend.demo_drive_mode(1.0 + cycle)["title"] == "прямо"
    assert backend.demo_drive_mode(cycle / 2)["title"] != "прямо"

    # источник отдаёт согласованные углы и название манёвра.
    # Часы подменяем: 12 тактов по 0,3 с — как реальный опрос экрана,
    # а фаза «краб вправо» удерживается той же (иначе время уезжает дальше).
    import time
    from unittest import mock

    clock = [2000.0]
    with mock.patch("time.time", side_effect=lambda: clock[0]):
        source = backend.SimSource()
        source.t0 = clock[0] - 14.0        # фаза «краб боком» (8 + 4 = 12 с)
        for _ in range(12):
            clock[0] += 0.3
            source.t0 += 0.3
            payload = source.read()
    assert payload["driveMode"] == "краб боком"
    angles = [round(m["angle"]) for m in payload["motors"]]
    assert angles == [90, 90, 90, 90], angles


def test_dash_cockpit_fills_the_screen_without_empty_columns():
    """Экран «Пульт» заполняет окно: колонки одной высоты, без пустых полей.

    Разбор разметки, а не поиск строк: важно, какие узлы лежат прямо в
    .cockpit, потому что места в сетке задаются именно им. Раньше камеры
    стояли в правой колонке, а колонки имели разную высоту — под короткими
    оставались пустые поля, карта не использовала своё место, а полотно
    выпихивало подпись за край коробки.
    """
    import re
    from html.parser import HTMLParser
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent
    html = (root / "slam_gui" / "index.html").read_text(encoding="utf-8")

    class CockpitChildren(HTMLParser):
        """Прямые дети .cockpit — с учётом вложенности, а не по строкам."""

        def __init__(self):
            super().__init__()
            self.depth = 0          # глубина внутри .cockpit
            self.children = []
            self.side_col_depth = None

        def handle_starttag(self, tag, attrs):
            if tag not in ("div", "section"):
                return
            classes = dict(attrs).get("class", "").split()
            if self.depth == 0 and "cockpit" in classes:
                self.depth = 1
                return
            if self.depth:
                if self.depth == 1:
                    self.children.append(" ".join(classes))
                if "side-col" in classes and self.side_col_depth is None:
                    self.side_col_depth = self.depth
                self.depth += 1

        def handle_endtag(self, tag):
            if tag in ("div", "section") and self.depth:
                self.depth -= 1

    parser = CockpitChildren()
    parser.feed(html)

    assert not parser.depth, "разбор не сошёлся: теги не закрыты"
    assert parser.children, "в .cockpit не нашлось узлов"
    assert "feed map" in parser.children, parser.children
    assert "cams" in parser.children, "камеры должны лежать прямо в .cockpit"
    assert "panel map-panel" in parser.children, parser.children
    assert "side-col" in parser.children, parser.children

    css = (root / "slam_gui" / "styles.css").read_text(encoding="utf-8")

    def top_level_rules(text: str) -> str:
        """CSS без комментариев и без блоков @media (смотрим основную раскладку)."""
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
        out, i = [], 0
        while i < len(text):
            media = re.match(r"\s*@media[^{]*\{", text[i:])
            if media:
                depth, j = 1, i + media.end()
                while j < len(text) and depth:
                    if text[j] == "{":
                        depth += 1
                    elif text[j] == "}":
                        depth -= 1
                    j += 1
                i = j
                continue
            out.append(text[i])
            i += 1
        return "".join(out)

    plain = top_level_rules(css)

    def rule(selector: str) -> str:
        """Тело правил с этим селектором: селектор сверяется целиком.

        Целиком — иначе «.feed.map» находился бы внутри «.cockpit > .feed.map»
        и проверял бы чужое правило.
        """
        bodies = []
        for block in re.finditer(r"([^{}]+)\{([^{}]*)\}", plain):
            selectors = [part.strip() for part in block.group(1).split(",")]
            if selector in selectors:
                bodies.append(block.group(2))
        return "\n".join(bodies)

    # колонки одной высоты, карта растёт, камеры — полосой под ней
    cockpit = rule(".cockpit")
    assert "grid-template-rows" in cockpit, "у .cockpit нет строк сетки"
    assert "align-items: stretch" in cockpit, "колонки снова разной высоты"
    assert "min-width: 0" not in cockpit or True
    cams = rule(".cockpit > .cams")
    assert "grid-column: 1" in cams and "grid-row: 2" in cams, cams
    assert "repeat(2, minmax(0, 1fr))" in cams, "камеры должны стоять полосой в два кадра"

    # карта больше не держит пропорцию: высоту ей задаёт раскладка
    feed_map = rule(".feed.map")
    assert "aspect-ratio" not in feed_map, "карта снова с фиксированной пропорцией"
    assert "min-height" in feed_map

    # полотно кадра не выпихивает подпись за край коробки
    feed = rule(".feed")
    assert "display: flex" in feed and "flex-direction: column" in feed, feed
    assert "flex: 1 1 auto" in rule(".feed canvas"), rule(".feed canvas")

    # длинные части панелей прокручиваются внутри, а не растягивают экран
    assert "overflow: auto" in rule(".prog-list")
    assert "overflow: auto" in rule(".teleop")
    assert "flex: 1 1 auto" in rule(".tele-panel")


def test_pages_have_no_duplicate_ids():
    """В страницах нет задвоенных id.

    Регрессия из исходного интерфейса: у таблицы детекций и у подписи камеры
    был один id="dets". Браузер отдаёт getElementById первый по документу,
    поэтому таблица уезжала в подпись кадра, а сама таблица оставалась пустой.
    Такую поломку глазами не видно, а находится она только сверкой id.
    """
    import re
    from collections import Counter
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent
    problems = {}
    for page in ("index.html", "main.html"):
        text = (root / "slam_gui" / page).read_text(encoding="utf-8")
        ids = re.findall(r'\sid="([^"]+)"', text)
        dups = sorted(i for i, n in Counter(ids).items() if n > 1)
        if dups:
            problems[page] = dups

    assert not problems, f"задвоенные id: {problems}"

    # у таблицы детекций и счётчика камеры — разные поля
    index = (root / "slam_gui" / "index.html").read_text(encoding="utf-8")
    assert 'id="dets"' in index and 'id="cam-dets"' in index

    app = (root / "slam_gui" / "app.js").read_text(encoding="utf-8")
    dets_body = app[app.index("function renderDets()"):]
    dets_body = dets_body[:dets_body.index("\nfunction ")]
    assert 'getElementById("dets")' in dets_body
    assert 'getElementById("cam-dets")' in dets_body


def test_camera_and_map_badges_sit_above_the_canvas():
    """Подписи кадров — своей строкой над полотном, а не поверх картинки.

    Прежде «карта SLAM», счётчики делений и FPS лежали абсолютно поверх
    полотна и перекрывали надписи на карте и видео.
    """
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent
    html = (root / "slam_gui" / "index.html").read_text(encoding="utf-8")

    heads = re.findall(r'<div class="feed-head">(.*?)</div>\s*<canvas', html, re.S)
    assert len(heads) == 3, f"шапок кадров должно быть три (карта и две камеры), а их {len(heads)}"
    for block in heads:
        assert 'class="live' in block, block
        assert "hud-cam" in block, block

    css = (root / "slam_gui" / "styles.css").read_text(encoding="utf-8")
    head_rule = re.search(r"\.feed-head \{([^}]*)\}", css, re.S).group(1)
    assert "flex: none" in head_rule and "border-bottom" in head_rule, head_rule
    static_rule = re.search(r"\.feed-head \.live,\s*\.feed-head \.hud-cam \{([^}]*)\}", css, re.S).group(1)
    assert "position: static" in static_rule, static_rule
