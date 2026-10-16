"""Проверки веб-интерфейса RUS SLAM: страницы, API, замок отсека, статика."""

from __future__ import annotations

import re

from conftest import login

DEFAULT_PIN = "7232"  # заводской PIN грузового отсека из backend.py


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


def test_kiosk_steering_endpoint_uses_pin_session_without_operator_login(app, client):
    """Киоск открыт без логина оператора, но физическая команда закрыта PIN-сессией."""
    class FakeSteeringSource:
        name = "master"
        supports_individual_steer = True

        def steer_module(self, module, angle):
            return {"ok": True, "angle": angle}

    app.extensions["rc_slam"].source = FakeSteeringSource()
    command = {"module": "RL", "angle": -8}
    header = {"X-Robot-Control": "steering"}
    assert client.post("/api/kiosk_steer", json=command, headers=header).status_code == 401
    assert client.post("/api/kiosk_lock/verify", json={"pin": DEFAULT_PIN}).status_code == 200
    response = client.post("/api/kiosk_steer", json=command, headers=header)
    assert response.status_code == 200
    assert response.get_json()["module"] == "RL"


def test_operator_drive_requires_login_pin_and_drive_header(app, client, operator):
    class FakeDriveSource:
        name = "master"
        supports_drive = True

        def __init__(self):
            self.commands = []

        def drive(self, percent):
            self.commands.append(percent)
            return {"ok": True, "percent": percent}

    source = FakeDriveSource()
    app.extensions["rc_slam"].source = source
    body = {"percent": 8}
    header = {"X-Robot-Control": "drive"}

    assert client.post("/api/drive", json=body, headers=header).status_code == 302
    login(client)
    assert client.post("/api/drive", json=body, headers=header).status_code == 401
    assert client.post("/api/kiosk_lock/verify", json={"pin": DEFAULT_PIN}).status_code == 200
    assert client.post("/api/drive", json=body).status_code == 400

    response = client.post("/api/drive", json=body, headers=header)
    assert response.status_code == 200
    assert response.get_json()["percent"] == 8
    assert source.commands == [8.0]

    client.post("/api/kiosk_lock/session/end", json={})
    # Ненулевая тяга без PIN по-прежнему запрещена.
    assert client.post("/api/drive", json={"percent": 5}, headers=header).status_code == 401
    assert source.commands == [8.0]
    # СТОП (0 %) разрешён без PIN: он только гасит PWM и включает тормоз.
    stopped = client.post("/api/drive", json={"percent": 0}, headers=header)
    assert stopped.status_code == 200
    assert source.commands == [8.0, 0]


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


def test_kiosk_logs_are_real_and_need_login(client, operator):
    """Журнал событий для панели «Логи»: настоящие записи и только после входа.

    На борту (8090) панель публична вместе с телеметрией, а на пульте журнал
    процесса закрыт входом оператора — как и остальные служебные данные.
    """
    assert client.get("/api/kiosk_logs").status_code in (302, 401)

    login(client)
    body = client.get("/api/kiosk_logs?limit=20").get_json()
    assert body["ok"] is True
    data = body["data"]
    assert data["demo"] is False           # выдуманных строк в журнале нет

    # Настоящее событие робота: неверный PIN отсечён — в журнале это видно.
    client.post("/api/lock/open", json={"pin": "0000"})
    client.post("/api/kiosk_lock/verify", json={"pin": "0000"})
    entries = client.get("/api/kiosk_logs?limit=50").get_json()["data"]["entries"]
    text = "\n".join(entry["msg"] for entry in entries)
    assert "PIN" in text and "неверный PIN" in text
    assert "0000" not in text


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
    assert backend.parse_master_line("@TLM mod=DRIVE deg=0") is None
    assert backend.parse_master_line("@TLM mod=FL deg=nan") is None


def test_master_source_queues_only_safe_module_angle_commands():
    """Команда отправляется только свежему откалиброванному и остановленному модулю."""
    import queue
    import threading

    backend = _load_backend()
    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=4)
    src.serial_ready = threading.Event()
    src.serial_ready.set()
    src.feed("@TLM mod=FR deg=0 tgt=0 moving=0 cal=1 cycle=0 calib=0", now=100.0)

    result = src.steer_module("fr", 12.5, now=100.2)
    assert result == {"ok": True, "queued": True, "module": "FR", "angle": 12.5}
    assert src.commands.get_nowait() == b"w FR 12.5\n"

    assert src.steer_module("FR", 33, now=100.2)["reason"] == "angle_limit"
    assert src.steer_module("FL", 10, now=100.2)["reason"] == "module_offline"
    src.feed("@TLM mod=RL deg=0 tgt=0 moving=0 cal=0 cycle=0 calib=0", now=100.0)
    assert src.steer_module("RL", 10, now=100.2)["reason"] == "not_homed"
    src.feed("@TLM mod=RR deg=0 tgt=0 moving=1 cal=1 cycle=0 calib=0", now=100.0)
    assert src.steer_module("RR", 10, now=100.2)["reason"] == "module_busy"
    assert src.steer_module("FR", 10, now=101.2)["reason"] == "module_offline"


def test_master_telemetry_parses_bldc_rpm_and_drive_percent():
    backend = _load_backend()
    frame = backend.parse_master_line(
        "@TLM mod=FL deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0 opto=0 rpm=245.5 drive=-12 t=42")
    assert frame["rpm"] == 245.5
    assert frame["drivePercent"] == -12
    assert backend.parse_master_line(
        "@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 rpm=20 drive=21") is None


def test_master_source_drive_requires_aligned_zeroed_modules_and_prioritizes_stop():
    import queue
    import threading

    backend = _load_backend()
    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=2)
    src.serial_ready = threading.Event()
    src.serial_ready.set()

    for module in ("FL", "FR", "RL", "RR"):
        src.feed("@TLM mod=%s deg=0.0 tgt=0.0 moving=0 cal=1 rpm=0.0 drive=0" % module, now=100.0)
    status = src.drive_status(now=100.2)
    assert status["canDrive"] is True
    assert status["reason"] is None

    # Тяга — «t N»: «d» в прошивке осталась поворотом на угол (как в проверенном скетче).
    assert src.drive(12, now=100.2)["ok"] is True
    assert src.commands.get_nowait() == b"t 12\n"
    assert src.drive(-8, now=100.3)["ok"] is True
    assert src.commands.get_nowait() == b"t -8\n"
    assert src.steer_module("FL", 5, now=100.3)["reason"] == "module_busy"

    # Очередь заполнена поворотами — STOP вытесняет их, а не пропадает.
    src.commands.put_nowait(b"w FL 5\n")
    src.commands.put_nowait(b"w FR 5\n")
    assert src.stop_drive(now=100.4)["ok"] is True
    assert src.commands.get_nowait() == b"b\n"
    assert src.commands.empty()

    src.feed("@TLM mod=RR deg=4.0 tgt=4.0 moving=0 cal=1 rpm=0 drive=0", now=100.5)
    assert src.drive_status(now=100.6)["reason"] == "steering_misaligned"


def test_master_line_v2_reports_zero_source_calibration_error_and_firmware():
    """Прошивка v2: откуда ноль (датчик/вручную), код ошибки калибровки, версия."""
    backend = _load_backend()
    frame = backend.parse_master_line(
        "@TLM mod=RR deg=0.00 tgt=0.00 moving=0 cal=1 cycle=0 calib=0 opto=1 home=opto "
        "err=0 rpm=0.0 drive=0 fw=2 t=5000")
    assert frame["cal"] is True and frame["home"] == "opto" and frame["manualZero"] is False
    assert frame["opto"] is True and frame["firmware"] == 2 and frame["calibError"] == 0
    failed = backend.parse_master_line(
        "@TLM mod=FR deg=-75.00 tgt=-75.00 moving=0 cal=0 cycle=0 calib=0 opto=0 home=none "
        "err=1 rpm=0.0 drive=0 fw=2 t=9000")
    assert failed["cal"] is False and failed["home"] == "none" and failed["calibError"] == 1
    manual = backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1 home=manual fw=2")
    assert manual["manualZero"] is True and manual["home"] == "manual"
    # Старая строка без home= — ноль мог быть только ручным.
    legacy = backend.parse_master_line("@TLM mod=FL deg=0 tgt=0 moving=0 cal=1")
    assert legacy["home"] == "manual" and legacy["firmware"] is None
    assert backend.parse_master_line("@TLM mod=FL deg=0 home=elsewhere") is None
    assert backend.parse_master_line("@TLM mod=FL deg=0 err=99") is None


def _bare_master_source(backend, maxsize=4):
    import queue
    import threading

    src = object.__new__(backend.MasterSource)
    src.lock = threading.Lock()
    src.frames = {}
    src.drive_percent = 0
    src.drive_last_command = 0.0
    src.commands = queue.Queue(maxsize=maxsize)
    src.serial_ready = threading.Event()
    src.serial_ready.set()
    src.messages = []
    src.on_message = None
    return src


def test_master_source_calibration_command_is_guarded():
    """Калибровка по датчику: один модуль «w FR calib», все — «k»; не при тяге и не на ходу."""
    backend = _load_backend()
    src = _bare_master_source(backend)
    line = "@TLM mod=%s deg=0 tgt=0 moving=0 cal=0 cycle=0 calib=0 opto=0 home=none err=0 rpm=0 drive=0 fw=2"
    src.feed(line % "FR", now=100.0)
    assert src.calibrate_module("fr", now=100.2) == {"ok": True, "queued": True, "module": "FR",
                                                     "offline": []}
    assert src.commands.get_nowait() == b"w FR calib\n"
    assert src.calibrate_module("FL", now=100.2)["reason"] == "module_offline"
    assert src.calibrate_module("XX", now=100.2)["reason"] == "invalid_module"
    result = src.calibrate_module("all", now=100.2)
    assert result["ok"] is True and result["offline"] == ["FL", "RL", "RR"]
    assert src.commands.get_nowait() == b"k\n"
    src.feed(line.replace("calib=0", "calib=1") % "FR", now=100.3)
    assert src.calibrate_module("FR", now=100.4)["reason"] == "module_busy"
    src.feed(line % "FR", now=100.5)
    src.drive_percent = 8
    assert src.calibrate_module("FR", now=100.6)["reason"] == "module_busy"
    src.drive_percent = 0
    src.serial_ready.clear()
    assert src.calibrate_module("FR", now=100.6)["reason"] == "serial_unavailable"


def test_master_source_keeps_firmware_messages_for_the_journal():
    backend = _load_backend()
    src = _bare_master_source(backend)
    seen = []
    src.on_message = seen.append
    assert src.feed("# FR: Калибровка ЗАВЕРШЕНА! Точка 0 установлена по датчику.\r\n", now=5.0) is False
    assert src.feed("просто текст", now=5.1) is False
    assert seen == ["FR: Калибровка ЗАВЕРШЕНА! Точка 0 установлена по датчику."]
    assert src.recent_messages() == [{"ts": 5.0, "text": seen[0]}]
    for index in range(60):
        src.feed("# FL: строка %d" % index, now=6.0 + index)
    assert len(src.messages) == backend.MasterSource.MESSAGE_LIMIT
    assert src.recent_messages(2)[-1]["text"] == "FL: строка 59"


def test_steering_control_info_reports_opto_calibration_for_master():
    from robot_control.slam import steering_control_info

    backend = _load_backend()
    info = steering_control_info(_bare_master_source(backend))
    assert info["optoUsed"] is True and info["calibration"] is True and info["zeroMode"] == "opto"
    assert info["manualZero"] is True
    peer = steering_control_info(backend.MasterPeerSource.__new__(backend.MasterPeerSource))
    assert peer["available"] is False and peer["calibration"] is False


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


# ------------------------- СТОП тяги и замок кейса ------------------------- #
def test_stop_is_accepted_without_pin_but_motion_is_not(app, client, operator):
    """СТОП (0 %) — единственная команда тяги без PIN; проверка заголовка и входа остаётся."""
    class FakeDriveSource:
        name = "master"
        supports_drive = True

        def __init__(self):
            self.commands = []

        def drive(self, percent):
            self.commands.append(percent)
            return {"ok": True, "percent": round(percent)}

    source = FakeDriveSource()
    app.extensions["rc_slam"].source = source
    header = {"X-Robot-Control": "drive"}
    login(client)
    assert client.post("/api/drive", json={"percent": 0}).status_code == 400  # нет заголовка
    assert client.post("/api/drive", json={"percent": 0}, headers=header).status_code == 200
    assert client.post("/api/drive", json={"percent": -3}, headers=header).status_code == 401
    # Тело, которое не является чистым СТОПом, без PIN не проходит (401 раньше разбора тела).
    assert client.post("/api/drive", json={"percent": 0.0, "x": 1}, headers=header).status_code == 401
    assert client.post("/api/drive", json={"percent": False}, headers=header).status_code == 401
    assert source.commands == [0]


def test_api_stop_relays_to_kiosk_and_reports_real_result(app, client, operator):
    """Пульт 8080 не владеет портом: СТОП уходит на бортовой экран и возвращается его ответ."""
    import http.server
    import json
    import threading

    received = []

    class Kiosk(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            received.append((self.path, self.headers.get("X-Robot-Control"), body))
            payload = json.dumps({"ok": True, "queued": True, "percent": 0}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Kiosk)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        class PeerSource:
            name = "master-peer"
            supports_drive = False
            peer_url = "http://127.0.0.1:%d/api/kiosk_state" % server.server_address[1]

        app.extensions["rc_slam"].source = PeerSource()
        login(client)
        assert client.post("/api/stop", json={}).status_code == 400  # нет заголовка
        response = client.post("/api/stop", json={}, headers={"X-Robot-Control": "drive"})
        assert response.status_code == 200
        assert response.get_json()["ok"] is True
        assert received == [("/api/kiosk_drive", "drive", {"percent": 0})]
    finally:
        server.shutdown()
        server.server_close()


def test_api_stop_reports_unreachable_kiosk_honestly(app, client, operator):
    class PeerSource:
        name = "master-peer"
        supports_drive = False
        peer_url = "http://127.0.0.1:9/api/kiosk_state"  # порт, на котором никто не слушает

    app.extensions["rc_slam"].source = PeerSource()
    login(client)
    response = client.post("/api/stop", json={}, headers={"X-Robot-Control": "drive"})
    assert response.status_code == 503
    assert response.get_json() == {"ok": False, "reason": "kiosk_unreachable"}


def test_stop_url_is_derived_from_peer_telemetry_url():
    from robot_control.slam import is_stop_request, stop_url_for

    assert stop_url_for("http://127.0.0.1:8090/api/kiosk_state") == "http://127.0.0.1:8090/api/kiosk_drive"
    assert is_stop_request({"percent": 0}) and is_stop_request({"percent": 0.0})
    assert not is_stop_request({"percent": 0, "extra": 1})
    assert not is_stop_request({"percent": False})
    assert not is_stop_request(None)


def test_lock_without_external_command_is_logged_as_software_only(app, client, operator, monkeypatch):
    """Без RUS_SLAM_LOCK_CMD открытие — только программное. Журнал должен сказать это прямо."""
    from robot_control import slam as slam_module

    monkeypatch.delenv("RUS_SLAM_LOCK_CMD", raising=False)
    messages = []
    monkeypatch.setattr(slam_module.eventlog, "record",
                        lambda level, message, **kw: messages.append((level, message)))
    login(client)
    response = client.post("/api/lock/open", json={"pin": DEFAULT_PIN})
    assert response.status_code == 200
    assert response.get_json()["hardware"] == "software"
    assert any(level == "warn" and "ТОЛЬКО программно" in text for level, text in messages)


def test_frame_ancestors_no_longer_allow_any_site(client):
    """Clickjacking: пульт не встраивается чужими сайтами, только доменами превью."""
    csp = client.get("/login").headers["Content-Security-Policy"]
    frame = [part for part in csp.split(";") if "frame-ancestors" in part][0]
    assert "'self'" in frame
    tokens = frame.replace("frame-ancestors", "").split()
    assert "http://*" not in tokens and "https://*" not in tokens
    assert "https://*.arena.ai" in tokens


def test_factory_pin_warning_until_pin_is_changed(tmp_path, monkeypatch):
    from robot_control import slam as slam_module

    state = slam_module.build_state(mode="none", lock_file=tmp_path / "lock.json")
    assert slam_module.warn_if_factory_pin(slam_module.load_backend(), state.vault) is True
    # Значение PIN в тексте предупреждения не выводится (проверяем сам текст через лог).

    state.vault.set_pin("7232", "915374")
    assert slam_module.warn_if_factory_pin(slam_module.load_backend(), state.vault) is False
    assert state.vault.data.get("pinChanged") is True
