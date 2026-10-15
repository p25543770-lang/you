"""Реальный USB-лидар в веб-интерфейсе: от чтения порта до скана в API обоих экранов.

Здесь закрыты ситуации, из-за которых лидар работал в ``scripts/lidar_check.py``,
а на экранах пульта (8080) и бортового экрана (8090) оставался «скана лидара
нет»:

  * источник данных шасси в режиме ``auto`` сам находил все ``/dev/ttyUSB*`` и
    открывал порт лидара, вычитывая из него байты (теперь порт из
    ``RC_LIDAR_PORT`` шасси не занимает, а «чужой» порт освобождает сам);
  * пульт и бортовой экран называли одно устройство разными путями
    (``/dev/ttyUSB1`` против ``/dev/serial/by-id/…``) и считали порт свободным —
    замок теперь один на устройство;
  * демо-скан стенда (``RC_SLAM_SOURCE=sim``) отдавался как настоящий: соседний
    процесс принимал его за реальный лидар, а экран писал «в реальном времени»;
  * менеджер лидара создавался только по первому запросу браузера, то есть
    настройки из ``.env`` могли и не примениться к чтению порта.

Тесты с настоящим pyserial идут на псевдотерминале: это тот же путь открытия
порта, что и на роботе, только вместо железа — поток кадров LDS-01 из теста.
"""

from __future__ import annotations

import json
import os
import pty
import struct
import subprocess
import sys
import threading
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from robot_control import lidar, slam
from robot_control.lidar import LidarManager

from test_lidar_serial import encode_lds01_packet, wait_for


@pytest.fixture
def backend():
    """Vendored ``slam_gui/backend.py`` как модуль (источники данных робота)."""
    return slam.load_backend()


@pytest.fixture
def kiosk_client(config):
    """Клиент бортового экрана (8090) — тот же, что в тестах киоска."""
    from robot_control.kiosk_app import create_kiosk_app

    application = create_kiosk_app(config)
    application.config.update(TESTING=True)
    return application.test_client()


@pytest.fixture
def manager():
    """Менеджер лидара в известном состоянии, без фонового перебора портов."""
    previous = LidarManager._instance
    instance = LidarManager()
    LidarManager._instance = instance
    try:
        yield instance
    finally:
        instance.stop()
        LidarManager._instance = previous


@pytest.fixture
def self_ports():
    """Порты HTTP этого процесса (для отказа от запроса скана у себя же)."""
    saved = set(lidar._SELF_PORTS)
    try:
        yield lidar._SELF_PORTS
    finally:
        lidar._SELF_PORTS.clear()
        lidar._SELF_PORTS.update(saved)


def open_fds_to(port: str) -> int:
    """Сколько fd этого процесса указывают на устройство порта."""
    target = os.path.realpath(port)
    count = 0
    for name in os.listdir("/proc/self/fd"):
        try:
            if os.readlink("/proc/self/fd/" + name) == target:
                count += 1
        except OSError:
            continue
    return count


def encode_telemetry(backend, module_id: int = 1, steer_cdeg: int = -925,
                     status: int = 0x03) -> bytes:
    """Кадр телеметрии модуля шасси (16 Б, см. backend.parse_telemetry)."""
    body = (bytes([0xBB, 0x44, module_id & 0xFF, status])
            + struct.pack(">hhh", steer_cdeg, 12, 100)
            + struct.pack(">HH", 4000, 1500))
    return body + struct.pack(">H", backend.crc16(body))


# ---------------------------------------------------------------------------
# 1. Порт лидара не должен занимать источник данных шасси
# ---------------------------------------------------------------------------


def test_autodetected_chassis_ports_skip_the_lidar_port(tmp_path, monkeypatch, backend):
    """RC_SLAM_SOURCE=auto: порт из RC_LIDAR_PORT шасси не открывает.

    Именно это ломало лидар в веб-интерфейсе: автопоиск модулей забирал все
    /dev/ttyUSB*, включая порт лидара, и держал их открытыми постоянно.
    """
    chassis = tmp_path / "ttyUSB0"
    chassis.write_text("")
    lidar_port = tmp_path / "ttyUSB1"
    lidar_port.write_text("")
    monkeypatch.setattr(slam, "DEFAULT_PORTS", [str(chassis), str(lidar_port)])
    monkeypatch.setenv("RC_LIDAR_PORT", str(lidar_port))
    monkeypatch.setitem(sys.modules, "serial", types.ModuleType("serial"))

    source = slam.make_source("auto")
    try:
        assert isinstance(source, backend.SerialSource)
        assert source.ports == [str(chassis)]
    finally:
        source._stop = True


def test_chassis_skips_lidar_port_found_by_symlink(tmp_path, monkeypatch, backend):
    """Порт задан как /dev/serial/by-id/… — шасси не должно открыть /dev/ttyUSB1."""
    device = tmp_path / "ttyUSB1"
    device.write_text("")
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    link = by_id / "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    link.symlink_to(device)
    other = tmp_path / "ttyUSB0"
    other.write_text("")

    monkeypatch.setattr(slam, "DEFAULT_PORTS", [str(other), str(device)])
    monkeypatch.setenv("RC_LIDAR_PORT", str(link))
    monkeypatch.setitem(sys.modules, "serial", types.ModuleType("serial"))

    assert str(device) in lidar.lidar_reserved_ports()
    source = slam.make_source("auto")
    try:
        assert source.ports == [str(other)]
    finally:
        source._stop = True


def test_chassis_ports_keep_everything_when_lidar_is_not_pinned(monkeypatch):
    """Без RC_LIDAR_PORT шасси не теряет порты: отбирается только закреплённое."""
    monkeypatch.delenv("RC_LIDAR_PORT", raising=False)
    monkeypatch.setattr(lidar, "excluded_ports", lambda: set())
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: [])
    assert slam.chassis_ports(["/dev/ttyUSB0", "/dev/ttyUSB1"]) == ["/dev/ttyUSB0", "/dev/ttyUSB1"]


def test_explicit_chassis_port_wins_and_is_visible_in_diagnostics(monkeypatch):
    """Порт, явно отданный шасси, лидар не трогает — и это видно в диагностике."""
    monkeypatch.setenv("RC_LIDAR_PORT", "/dev/ttyUSB1")
    monkeypatch.setenv("RC_SLAM_PORTS", "/dev/ttyUSB1")
    assert "/dev/ttyUSB1" in lidar.excluded_ports()
    assert lidar.lidar_reserved_ports() == set()
    note = lidar.config_conflict_note()
    assert "/dev/ttyUSB1" in note and "мастера/шасси" in note
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: ["/dev/ttyUSB1"])
    diagnostics = lidar.usb_diagnostics()
    assert "исключены настройками мастера/шасси" in diagnostics["summary"]
    assert diagnostics["config_conflict"] == note


@pytest.mark.skipif(not os.path.isdir("/proc/self/fd"), reason="нужен /proc")
def test_serial_source_releases_port_that_does_not_speak_chassis_protocol(backend, monkeypatch):
    """Порт без кадров телеметрии освобождается — лидар успевает его забрать."""
    pytest.importorskip("serial")
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)                      # имя порта живо, пока открыт мастер
    monkeypatch.setattr(backend.SerialSource, "PROBE_WINDOW_S", 0.3)
    monkeypatch.setattr(backend.SerialSource, "BACKOFF_START_S", 30.0)
    source = backend.SerialSource([port])
    try:
        assert wait_for(lambda: open_fds_to(port) == 0, timeout=5.0), \
            "источник шасси держит порт, который не отвечает его протоколом"
    finally:
        source._stop = True
        time.sleep(0.4)      # дочитываем до закрытия мастера: без этого порт
        os.close(master)     # «отваливается» на середине чтения и шумит в логе


@pytest.mark.skipif(not os.path.isdir("/proc/self/fd"), reason="нужен /proc")
def test_serial_source_keeps_port_with_real_chassis_telemetry(backend, monkeypatch):
    """Настоящий модуль шасси порт не теряет: кадры пришли — чтение продолжается."""
    pytest.importorskip("serial")
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    monkeypatch.setattr(backend.SerialSource, "PROBE_WINDOW_S", 1.0)
    source = backend.SerialSource([port])
    try:
        assert wait_for(lambda: open_fds_to(port) >= 1, timeout=5.0)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            os.write(master, encode_telemetry(backend))
            time.sleep(0.05)
        data = source.read()
        online = [m for m in data["motors"] if m.get("online")]
        assert online, "модуль присылал телеметрию, но не появился в данных"
        assert online[0]["angle"] == pytest.approx(-9.25, abs=0.01)
        time.sleep(0.3)
        assert open_fds_to(port) >= 1, "порт с живой телеметрией был отпущен"
    finally:
        source._stop = True
        time.sleep(0.4)      # дочитываем до закрытия мастера: без этого порт
        os.close(master)     # «отваливается» на середине чтения и шумит в логе


def fake_serial_module(monkeypatch, opened: list):
    """Подменяет ``serial``: открытие порта только записывается, порт «занят».

    Так проверяется, что источник данных шасси вообще не берётся за порт —
    без железа и без гонки с ним.
    """

    class FakeSerial:
        SerialException = Exception

        @staticmethod
        def Serial(port, *args, **kwargs):  # noqa: N802 — интерфейс pyserial
            opened.append(port)
            raise OSError(16, "Device or resource busy")

    fake = types.ModuleType("serial")
    fake.Serial = FakeSerial.Serial
    fake.SerialException = FakeSerial.SerialException
    monkeypatch.setitem(sys.modules, "serial", fake)
    return fake


def test_chassis_does_not_touch_the_port_the_lidar_holds(tmp_path, monkeypatch, backend):
    """Пока лидар держит порт, источник шасси его не открывает вовсе.

    Пульт (8080) и бортовой экран (8090) поднимали источник данных шасси в двух
    процессах, и один USB-порт читался двумя процессами сразу — pyserial отвечал
    «device reports readiness to read but returned no data (… multiple access on
    port?)». Лидару же порт не доставался ни на секунду: его перебор всё время
    упирался в «занят», а на экране появлялся совет «проверь питание и кабель».
    Теперь порт берётся под общий замок, и лидар всегда получает его свободным.
    """
    monkeypatch.setattr(lidar, "DATA_DIR", tmp_path / "locks")
    port = tmp_path / "ttyUSB0"
    port.write_text("")
    opened: list = []
    fake_serial_module(monkeypatch, opened)

    monkeypatch.setattr(backend.SerialSource, "PROBE_WINDOW_S", 0.1)
    monkeypatch.setattr(backend.SerialSource, "BACKOFF_START_S", 0.05)

    token = lidar.try_lock_port(str(port))
    assert token is not None, "замок порта не взялся"
    source = backend.SerialSource([str(port)], claim=lidar.try_lock_port,
                                  release=lidar.unlock_port)
    try:
        time.sleep(0.4)
        assert opened == [], "источник шасси открыл порт, который держит лидар"
        lidar.unlock_port(token)
        token = None
        assert wait_for(lambda: bool(opened), timeout=5.0), \
            "после освобождения порта источник шасси не попробовал его открыть"
    finally:
        source._stop = True
        lidar.unlock_port(token)


def test_chassis_gives_the_port_back_when_there_is_no_telemetry(tmp_path, monkeypatch, backend):
    """Порт без кадров телеметрии шасси отпускает — замок не остаётся навсегда."""
    monkeypatch.setattr(lidar, "DATA_DIR", tmp_path / "locks")
    port = tmp_path / "ttyUSB0"
    port.write_text("")

    class FakeDevice:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def read(self, size):  # noqa: ARG002 — пустой поток: телеметрии нет
            time.sleep(0.01)
            return b""

    fake = types.ModuleType("serial")
    fake.Serial = lambda *args, **kwargs: FakeDevice()  # noqa: E731 — интерфейс pyserial
    fake.SerialException = Exception
    monkeypatch.setitem(sys.modules, "serial", fake)
    monkeypatch.setattr(backend.SerialSource, "PROBE_WINDOW_S", 0.1)
    monkeypatch.setattr(backend.SerialSource, "BACKOFF_START_S", 0.05)

    releases: list = []

    def release(token):
        releases.append(token)
        lidar.unlock_port(token)

    source = backend.SerialSource([str(port)], claim=lidar.try_lock_port, release=release)
    try:
        assert wait_for(lambda: bool(releases), timeout=5.0), \
            "источник шасси не отпустил порт после неудачной проверки"
    finally:
        source._stop = True
    assert wait_for(lambda: not lidar.port_locked_by_other(str(port)), timeout=3.0), \
        "источник шасси оставил порт под замком — лидар его больше не получит"


@pytest.mark.skipif(not os.path.isdir("/proc/self/fd"), reason="нужен /proc")
def test_confirmed_module_port_is_still_read_beside_the_lidar(tmp_path, monkeypatch, backend):
    """Живая телеметрия: замок снят и порт читается дальше — модули видит и второй процесс.

    Замок нужен только на время выяснения, чей это порт. Как только телеметрия
    подтвердилась, пульт и бортовой экран читают модули как раньше: порт с
    модулями терять нельзя, иначе бортовой экран останется без телеметрии.
    """
    pytest.importorskip("serial")
    monkeypatch.setattr(lidar, "DATA_DIR", tmp_path / "locks")
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    monkeypatch.setattr(backend.SerialSource, "PROBE_WINDOW_S", 3.0)
    source = backend.SerialSource([port], claim=lidar.try_lock_port,
                                  release=lidar.unlock_port)
    try:
        assert wait_for(lambda: open_fds_to(port) >= 1, timeout=5.0)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            os.write(master, encode_telemetry(backend))
            time.sleep(0.05)
        data = source.read()
        assert [m for m in data["motors"] if m.get("online")], "телеметрия не дошла"
        assert open_fds_to(port) >= 1, "порт с живой телеметрией был отпущен"
        assert wait_for(lambda: not lidar.port_locked_by_other(port), timeout=3.0), \
            "замок порта остался у шасси: второй процесс робота не прочитает модули"
    finally:
        source._stop = True
        time.sleep(0.4)      # дочитываем до закрытия мастера: без этого порт
        os.close(master)     # «отваливается» на середине чтения и шумит в логе


def test_app_source_takes_the_same_lock_as_the_lidar(tmp_path, monkeypatch, backend):
    """Источник шасси из приложения (RC_SLAM_SOURCE=auto) берёт общий замок порта."""
    monkeypatch.setattr(lidar, "DATA_DIR", tmp_path / "locks")
    port = tmp_path / "ttyUSB0"
    port.write_text("")
    monkeypatch.setattr(slam, "DEFAULT_PORTS", [str(port)])
    monkeypatch.setenv("RC_LIDAR_PORT", str(tmp_path / "нет-такого-порта"))
    opened: list = []
    fake_serial_module(monkeypatch, opened)

    token = lidar.try_lock_port(str(port))
    assert token is not None, "замок порта не взялся"
    source = slam.make_source("auto")
    try:
        assert isinstance(source, backend.SerialSource) and source.ports == [str(port)]
        time.sleep(0.3)
        assert opened == [], "источник шасси приложения не проверил замок порта"
    finally:
        source._stop = True
        lidar.unlock_port(token)


# ---------------------------------------------------------------------------
# 2. Один замок на устройство, а не на имя пути
# ---------------------------------------------------------------------------


def test_port_lock_is_one_per_device_not_per_name(tmp_path):
    """ttyUSB1 и его symlink из /dev/serial/by-id — один замок на двоих."""
    device = tmp_path / "ttyUSB1"
    device.write_text("")
    link = tmp_path / "by-id-link"
    link.symlink_to(device)

    assert lidar.port_lock_file(str(link)) == lidar.port_lock_file(str(device))
    fd = lidar.try_lock_port(str(link))
    try:
        assert fd is not None
        assert lidar.try_lock_port(str(device)) is None
        assert lidar.port_locked_by_other(str(device)) is True
    finally:
        lidar.unlock_port(fd)
    assert lidar.port_locked_by_other(str(device)) is False


def test_manager_reports_busy_port_as_busy_not_as_dead_lidar(monkeypatch, manager):
    """Порт занят чужой программой — в статусе причина «занят», а не «не отвечает»."""
    port = "/dev/ttyBUSY"

    class BusySerial:
        SerialException = Exception

        @staticmethod
        def Serial(*args, **kwargs):  # noqa: N802 — интерфейс pyserial
            raise OSError(16, "Device or resource busy")

    assert manager._read_port(BusySerial, port) is False
    assert manager._port_note == "занят"
    assert "другой программой" in manager.hint
    assert "не отвечает" not in manager.reason


def test_found_port_is_tried_first_after_a_drop(fake_usb_like, manager):
    """После обрыва связи круг перебора начинается с уже найденного порта."""
    ports = fake_usb_like
    manager._sticky = ports[2]
    assert manager.candidate_ports()[0] == ports[2]
    # ротация для остальных портов сохраняется, если найденный пропал из списка
    manager._sticky = "/dev/gone"
    first = manager.candidate_ports()[0]
    assert first in ports


@pytest.fixture
def fake_usb_like(tmp_path, monkeypatch):
    """Три «USB-порта» во временной папке (подмена glob, как в других тестах)."""
    paths = []
    for name in ("ttyUSB0", "ttyUSB1", "ttyUSB2"):
        path = tmp_path / name
        path.write_text("")
        paths.append(str(path))

    def fake_glob(pattern, **_):
        if "ttyUSB" in pattern:
            return sorted(p for p in paths if "ttyUSB" in p)
        return []

    monkeypatch.setattr(lidar.glob, "glob", fake_glob)
    monkeypatch.delenv("RC_LIDAR_PORT", raising=False)
    return paths


# ---------------------------------------------------------------------------
# 3. Соседний процесс: скан берём по HTTP, демо — не берём
# ---------------------------------------------------------------------------


def peer_server(payload: dict):
    """HTTP-сервер, отдающий ``payload`` на любой GET (имитация соседа)."""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):                                     # noqa: N802
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_peer_demo_scan_is_not_taken_for_a_real_lidar(monkeypatch, manager):
    """Сосед показывает демо — свои точки не подменяются выдуманными."""
    payload = {"scan": [{"a": 0.5, "r": 1.5}], "demo": True, "connected": False,
               "reason": "демо-скан источника данных (реального лидара нет)"}
    server, thread = peer_server(payload)
    try:
        monkeypatch.setenv(
            "RC_LIDAR_PEER_URLS", "http://127.0.0.1:%d/api/lidar" % server.server_port)
        manager.peer_urls = lidar.peer_urls()
        assert manager._peer_cycle() is False
        assert manager.get_scan() is None
        assert manager.get_status()["source"] == "no_signal"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_peer_real_scan_is_taken(monkeypatch, manager):
    payload = {"scan": [{"a": 0.5, "r": 1.5}, {"a": -0.5, "r": 2.5}], "demo": False,
               "connected": True,
               "detected": {"port": "/dev/ttyUSB1", "protocol": "lds01",
                            "baud": 230400, "motor": "none"}}
    server, thread = peer_server(payload)
    try:
        monkeypatch.setenv(
            "RC_LIDAR_PEER_URLS", "http://127.0.0.1:%d/api/lidar" % server.server_port)
        manager.peer_urls = lidar.peer_urls()
        assert manager._peer_cycle() is True
        status = manager.get_status()
        assert status["source"] == "peer"
        assert status["demo"] is False
        assert status["detected"]["protocol"] == "lds01"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_peer_urls_skip_own_port(monkeypatch, self_ports, manager):
    """Бортовой экран (8090) не ходит за сканом к себе же."""
    monkeypatch.setenv("RC_LIDAR_PEER_URLS",
                       "http://127.0.0.1:8090/api/lidar,http://127.0.0.1:8080/api/lidar")
    manager.peer_urls = lidar.peer_urls()
    lidar.set_self_port(8090)
    assert manager._peer_targets() == ["http://127.0.0.1:8080/api/lidar"]
    lidar.set_self_port("не-число")          # мусор не ломает
    assert manager._peer_targets() == ["http://127.0.0.1:8080/api/lidar"]


# ---------------------------------------------------------------------------
# 4. API обоих экранов: реальный скан доходит, демо помечено
# ---------------------------------------------------------------------------


def test_console_api_marks_demo_scan(client, operator, manager):
    """Стенд без железа: /api/state и /api/lidar честно говорят «демо»."""
    from conftest import login

    assert login(client).status_code in (302, 303)
    state = client.get("/api/state").get_json()["data"]
    assert state["lidar"], "симуляция должна отдавать точки для стенда"
    assert state["lidarDemo"] is True
    assert state["lidarStatus"]["demo"] is True
    assert state["lidarStatus"]["connected"] is False
    assert state["lidarStatus"]["reason"].startswith("демо-скан источника данных")

    status = client.get("/api/lidar").get_json()
    assert status["demo"] is True
    assert status["connected"] is False
    assert status["reason"] == state["lidarStatus"]["reason"]


def test_console_api_serves_real_scan_without_demo_flag(client, operator, manager):
    """Точки с USB-лидара: те же ответы, но уже без пометки демо."""
    from conftest import login

    assert login(client).status_code in (302, 303)
    scan = [{"a": round(i * 0.01, 4), "r": 1.5} for i in range(360)]
    assert manager.feed_points(scan, source_label="serial:ttyUSB1") == 360
    manager.detected = {"port": "/dev/ttyUSB1", "protocol": "lds01",
                        "baud": 230400, "motor": "none"}

    state = client.get("/api/state").get_json()["data"]
    assert len(state["lidar"]) == 360
    assert state["lidarDemo"] is False
    assert state["lidarStatus"]["connected"] is True
    assert state["lidarStatus"]["source"] == "serial:ttyUSB1"
    assert state["lidarStatus"]["detected"]["baud"] == 230400

    status = client.get("/api/lidar").get_json()
    assert status["demo"] is False
    assert status["count"] == 360
    assert status["points_count"] == 360


def test_console_kiosk_state_carries_lidar_reason(client, operator, manager):
    """Экран робота на 8080: причина «скана нет» в ответе есть (robot.js её показывает)."""
    body = client.get("/api/kiosk_state").get_json()
    assert body["ok"] is True
    data = body["data"]
    assert data["lidarStatus"]["reason"]
    assert "lidarDemo" in data


def test_both_screens_get_the_same_real_scan(client, operator, kiosk_client, manager):
    """Пульт и бортовой экран в одном процессе видят один и тот же скан."""
    from conftest import login

    assert login(client).status_code in (302, 303)
    scan = [{"a": 0.1, "r": 2.0}, {"a": -0.1, "r": 3.0}]
    manager.feed_points(scan, source_label="serial:ttyUSB1")

    console = client.get("/api/state").get_json()["data"]
    kiosk = kiosk_client.get("/api/kiosk_state").get_json()["data"]
    assert console["lidar"] == scan
    assert kiosk["lidar"] == scan
    assert console["lidarDemo"] is False and kiosk["lidarDemo"] is False
    assert console["lidarStatus"]["connected"] is True
    assert kiosk["lidarStatus"]["connected"] is True
    # ответ /api/lidar — то, что берёт соседний процесс робота
    assert kiosk_client.get("/api/lidar").get_json()["scan"] == scan


def test_status_reports_interpreter_and_pyserial(manager):
    """В статусе видно, каким интерпретатором запущен процесс и есть ли pyserial."""
    status = manager.get_status()
    assert status["python"] == sys.executable
    assert status["pyserial"] == lidar.pyserial_version()
    summary_lines = lidar.startup_summary()
    assert any(sys.executable in line for line in summary_lines)
    assert any("RC_LIDAR_PROTOCOL" in line for line in summary_lines)


def test_diagnostics_reports_missing_pyserial(monkeypatch):
    """pyserial нет в этом интерпретаторе — причина видна на экране, а не в догадках."""
    monkeypatch.setattr(lidar, "pyserial_version", lambda: None)
    monkeypatch.setattr(lidar, "discover_ports", lambda **kw: ["/dev/ttyUSB1"])
    monkeypatch.setattr(lidar, "excluded_ports", lambda: set())
    monkeypatch.setattr(lidar.os, "access", lambda *args: True)
    diagnostics = lidar.usb_diagnostics()
    assert diagnostics["pyserial"] is None
    assert "pyserial не установлен" in diagnostics["summary"]
    assert "ensure_deps.sh" in diagnostics["summary"]
    assert any("ensure_deps" in line for line in lidar.startup_summary())


# ---------------------------------------------------------------------------
# 5. Менеджер стартует вместе с приложением и читает .env до создания
# ---------------------------------------------------------------------------


def test_start_lidar_reads_settings_and_remembers_own_port(monkeypatch, self_ports, caplog):
    monkeypatch.setenv("RC_LIDAR_PORT", "/dev/ttyUSB1")
    monkeypatch.setenv("RC_LIDAR_PROTOCOL", "lds01")
    monkeypatch.setenv("RC_LIDAR_BAUD", "230400")
    monkeypatch.setenv("RC_LIDAR_MOTOR", "none")
    monkeypatch.delenv("PYTEST_CURRENT_TEST", raising=False)
    created = []
    monkeypatch.setattr(
        LidarManager, "get_instance",
        classmethod(lambda cls, port=None: created.append(port) or LidarManager()),
    )
    with caplog.at_level("INFO", logger="robot_control.slam"):
        assert slam.start_lidar(port=8080) is not None
    assert created == [None]
    assert 8080 in lidar._SELF_PORTS
    text = "\n".join(record.getMessage() for record in caplog.records)
    assert "/dev/ttyUSB1" in text and "lds01" in text and "230400" in text


def test_start_lidar_is_silent_in_tests_and_can_be_switched_off(monkeypatch):
    monkeypatch.setenv("RC_LIDAR_AUTOSTART", "0")
    assert slam.start_lidar(force=False) is None
    monkeypatch.delenv("RC_LIDAR_AUTOSTART", raising=False)
    # в тестах (PYTEST_CURRENT_TEST) ранний старт не срабатывает без force
    assert slam.start_lidar() is None
    assert slam.start_lidar(testing=True) is None


def test_create_app_starts_lidar_after_env_is_loaded(monkeypatch, config):
    """Фабрика приложения поднимает лидар сама, со своим портом HTTP."""
    from robot_control import app as app_module

    calls = []
    monkeypatch.setattr(
        app_module, "start_lidar",
        lambda **kwargs: calls.append(kwargs) or None,
    )
    app_module.create_app(config)
    assert calls and calls[0]["port"] == config.port
    assert calls[0]["testing"] is config.testing


def test_create_kiosk_app_starts_lidar_with_its_own_port(monkeypatch, config):
    from robot_control import kiosk_app as kiosk_module

    calls = []
    monkeypatch.setattr(
        kiosk_module, "start_lidar",
        lambda **kwargs: calls.append(kwargs) or None,
    )
    kiosk_module.create_kiosk_app(config)
    assert calls and calls[0]["port"] == kiosk_module.kiosk_port()


# ---------------------------------------------------------------------------
# 6. Сквозная проверка на псевдотерминале: порт → парсер → менеджер → API
# ---------------------------------------------------------------------------


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="нужен POSIX-псевдотерминал")
def test_lds01_scan_reaches_both_screens_over_http(monkeypatch, client, operator,
                                                   kiosk_client, backend):
    """LDS-01 на настоящем pyserial: скан доезжает до API пульта и бортового экрана."""
    pytest.importorskip("serial")
    from conftest import login

    assert login(client).status_code in (302, 303)
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    stop = threading.Event()

    def device():
        sent = 0
        started = time.monotonic()
        while not stop.is_set():
            allowed = int((time.monotonic() - started) * 12600.0)   # ~230400 бод
            while sent * 42 < allowed:
                try:
                    os.write(master, encode_lds01_packet(sent % 60, 1.5))
                except OSError:
                    time.sleep(0.05)
                    break
                sent += 1
            time.sleep(0.005)

    thread = threading.Thread(target=device, daemon=True)
    thread.start()

    monkeypatch.setenv("RC_LIDAR_PORT", port)
    monkeypatch.setenv("RC_LIDAR_PROTOCOL", "lds01")
    monkeypatch.setenv("RC_LIDAR_BAUD", "230400")
    monkeypatch.setenv("RC_LIDAR_MOTOR", "none")
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")
    worker_manager = LidarManager()
    worker = threading.Thread(target=worker_manager._serial_worker, daemon=True)
    LidarManager._instance = worker_manager
    worker.start()
    try:
        assert wait_for(lambda: len(worker_manager.get_scan() or []) >= 300, timeout=10.0), \
            "реальный скан с порта не дошёл до менеджера"

        console = client.get("/api/state").get_json()["data"]
        assert console["lidarDemo"] is False
        assert len(console["lidar"]) >= 300
        assert console["lidarStatus"]["detected"]["protocol"] == "lds01"
        assert console["lidarStatus"]["detected"]["baud"] == 230400

        kiosk = kiosk_client.get("/api/kiosk_state").get_json()["data"]
        assert kiosk["lidarDemo"] is False
        assert len(kiosk["lidar"]) >= 300

        api = kiosk_client.get("/api/lidar").get_json()
        assert api["connected"] is True and api["demo"] is False
        assert api["count"] >= 300
    finally:
        worker_manager.stop()
        worker.join(timeout=3.0)
        LidarManager._instance = None
        stop.set()
        thread.join(timeout=2.0)
        os.close(slave)
        os.close(master)


# ---------------------------------------------------------------------------
# 7. Порты лидара и порты модулей шасси: приоритет и отметки в data/
# ---------------------------------------------------------------------------


@pytest.fixture
def state_dir(tmp_path, monkeypatch):
    """Общий журнал процессов робота (``data/ports``) — во временной папке.

    Кэш журнала (доли секунды, чтобы не сканировать каталог на каждый запрос)
    в тестах выключен: иначе проверка читала бы снимок соседнего теста.
    """
    path = tmp_path / "state"
    path.mkdir()
    monkeypatch.setattr(lidar, "DATA_DIR", path)
    monkeypatch.setattr(lidar, "PORTS_STATE_CACHE_S", 0.0)
    return path


def silent_serial(monkeypatch):
    """Подменяет pyserial: порт открывается и молчит, открытия записаны в список."""
    opened: list = []

    class FakeDevice:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def write(self, data):
            return len(data)

        def read(self, size):  # noqa: ARG002 — данных на порту нет
            time.sleep(0.01)
            return b""

    fake = types.ModuleType("serial")
    fake.Serial = lambda port, *args, **kwargs: (opened.append(port), FakeDevice())[1]
    fake.SerialException = Exception
    monkeypatch.setitem(sys.modules, "serial", fake)
    return opened


def test_lidar_prioritises_ports_while_it_searches(state_dir, tmp_path):
    """Пока порт не найден — порты уступают лидару; после находки — только его порт.

    Это ответ на робот с двумя адаптерами: источник данных шасси брал порты на свою
    проверку ровно тогда, когда лидар начинал перебор, и лидар сообщал «все порты
    заняты другой программой» про порты, которые держало это же приложение.
    """
    other = tmp_path / "ttyUSB0"
    other.write_text("")
    own = tmp_path / "ttyUSB1"
    own.write_text("")
    assert lidar.lidar_holds_port(str(other)) is False   # никто ничего не ищет

    lidar.publish_lidar_state("searching", since=time.time())
    assert lidar.lidar_search_priority() > 0.0
    assert lidar.lidar_holds_port(str(other)) is True

    lidar.publish_lidar_state("found", port=str(own))
    assert lidar.lidar_search_priority() == 0.0
    assert lidar.lidar_holds_port(str(own)) is True
    assert lidar.lidar_holds_port(str(other)) is False

    lidar.clear_lidar_state()
    assert lidar.lidar_holds_port(str(own)) is False


def test_search_priority_window_is_bounded(state_dir, monkeypatch):
    """Окно приоритета кончается: если лидара на роботе нет, порты достаются модулям."""
    monkeypatch.setenv("RC_LIDAR_PRIORITY_S", "5")
    lidar.publish_lidar_state("searching", since=time.time() - 30)
    assert lidar.lidar_search_priority() == 0.0
    assert lidar.lidar_holds_port("/dev/ttyUSB0") is False

    monkeypatch.setenv("RC_LIDAR_PRIORITY_S", "0")           # приоритет выключен
    lidar.publish_lidar_state("searching", since=time.time())
    assert lidar.lidar_holds_port("/dev/ttyUSB0") is False


def test_state_of_a_dead_process_does_not_hold_ports(state_dir):
    """Файл упавшего процесса порты не держит — иначе лидар ждал бы его вечно."""
    process = subprocess.Popen([sys.executable, "-c", "pass"])
    process.wait()
    payload = {"pid": process.pid, "state": "searching", "port": None,
               "at": time.time(), "since": time.time()}
    path = lidar.lidar_state_file(process.pid)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload))
    assert lidar.lidar_search_priority() == 0.0
    assert lidar.lidar_holds_port("/dev/ttyUSB0") is False


def test_state_of_another_searcher_does_not_erase_the_live_search(state_dir):
    """Запись второго сеанса поиска в том же процессе не гасит живой поиск.

    Так лидар «не находился» на роботе: менеджеров в процессе бывает несколько
    (пульт и бортовой экран в одном процессе, менеджеры в тестах), а поле ``since``
    в файле состояния было одно на процесс. Запись соседа, который начал искать
    раньше и уже исчерпал окно ``RC_LIDAR_PRIORITY_S``, затирала свежую — и модули
    шасси занимали порты посреди перебора лидара.
    """
    expired = LidarManager()
    expired._search_since = time.time() - 60.0             # окно приоритета кончилось
    expired._publish_state("searching")
    live = LidarManager()
    live._publish_state("searching")                       # этот искать только начал
    try:
        records = json.loads(lidar.lidar_state_file().read_text("utf-8"))["records"]
        assert len(records) == 2, "у каждого сеанса поиска своя запись"
        assert lidar.lidar_search_priority() > 0.0, "живой поиск виден в журнале"
        assert lidar.lidar_holds_port("/dev/ttyUSB0") is True
    finally:
        expired.stop()
        live.stop()
    assert lidar.lidar_search_priority() == 0.0, "остановленный поиск держит порты"


def test_own_writes_are_visible_through_the_journal_cache(state_dir, monkeypatch):
    """Своя запись видна сразу: кэш журнала сбрасывается, ожидать TTL не нужно.

    Кэш нужен на роботе (статус читают оба экрана, каталог на SD-карте), но
    решение «уступать ли порт» нельзя принимать по устаревшему снимку после
    собственной записи — от этого зависит, достаётся ли порт лидару.
    """
    monkeypatch.setattr(lidar, "PORTS_STATE_CACHE_S", 5.0)
    assert lidar.lidar_search_priority() == 0.0            # первый снимок журнала

    lidar.publish_lidar_state("searching", since=time.time())
    assert lidar.lidar_search_priority() > 0.0             # видно без ожидания кэша

    port = state_dir.parent / "ttyUSB0"
    port.write_text("")
    lidar.tag_module_port(str(port))
    assert lidar.module_port_confirmed(str(port)) is True
    lidar.untag_module_port(str(port))
    assert lidar.module_port_confirmed(str(port)) is False


def test_serial_source_yields_ports_while_the_lidar_searches(state_dir, tmp_path,
                                                             monkeypatch, backend):
    """Источник шасси не занимает порты своей проверкой, пока лидар ищет свой.

    Правка A из разбора на роботе: без неё лидару не доставалось ни секунды —
    модули брали оба порта на двухсекундную проверку как раз в момент перебора.
    """
    port = tmp_path / "ttyUSB0"
    port.write_text("")
    other = tmp_path / "ttyUSB1"
    other.write_text("")
    opened: list = []

    class BusySerial:
        SerialException = Exception

        @staticmethod
        def Serial(name, *args, **kwargs):  # noqa: N802 — интерфейс pyserial
            opened.append(name)
            raise OSError(16, "Device or resource busy")

    fake = types.ModuleType("serial")
    fake.Serial = BusySerial.Serial
    fake.SerialException = BusySerial.SerialException
    monkeypatch.setitem(sys.modules, "serial", fake)
    monkeypatch.setattr(backend.SerialSource, "YIELD_SLEEP_S", 0.05)

    lidar.publish_lidar_state("searching", since=time.time())
    source = backend.SerialSource([str(port), str(other)], yield_to_lidar=lidar.lidar_holds_port)
    try:
        time.sleep(0.4)
        assert opened == [], "источник шасси занял порты, пока лидар искал свой"
        lidar.publish_lidar_state("found", port=str(other))
        assert wait_for(lambda: str(port) in opened, timeout=5.0), \
            "после поиска лидара источник шасси не вернулся к своим портам"
        time.sleep(0.2)
        assert str(other) not in opened, "источник шасси полез на порт, который читает лидар"
    finally:
        source._stop = True


@pytest.mark.skipif(not os.path.isdir("/proc/self/fd"), reason="нужен /proc")
def test_chassis_source_tags_the_port_where_telemetry_was_confirmed(state_dir, monkeypatch, backend):
    """Порт с кадрами телеметрии отмечается как порт модулей — и снимается при тишине.

    Правка B: отметку видит лидар и не перебирает этот порт (он всё равно занят
    источником данных шасси). Отметка снимается, как только порт замолчал: если
    адаптеры поменяли местами, лидар должен получить этот порт.
    """
    pytest.importorskip("serial")
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    monkeypatch.setattr(backend.SerialSource, "PROBE_WINDOW_S", 1.0)
    source = backend.SerialSource(
        [port],
        claim=lidar.try_lock_port,
        release=lidar.unlock_port,
        yield_to_lidar=lidar.lidar_holds_port,
        tag_module=lidar.tag_module_port,
        untag_module=lidar.untag_module_port,
    )
    try:
        assert wait_for(lambda: open_fds_to(port) >= 1, timeout=5.0)
        for _ in range(20):
            os.write(master, encode_telemetry(backend))
            time.sleep(0.02)
        assert wait_for(lambda: lidar.module_port_confirmed(port), timeout=5.0), \
            "источник шасси не отметил порт, на котором подтвердилась телеметрия"

        # Порт модулей не уступается лидару даже во время его поиска.
        lidar.publish_lidar_state("searching", since=time.time())
        assert lidar.lidar_holds_port(port) is False
        assert lidar.lidar_holds_port("/dev/ttyUSB7") is True

        # Порт замолчал (адаптер выдернули / заменили лидаром) — отметка уходит.
        assert wait_for(lambda: not lidar.module_port_confirmed(port), timeout=8.0), \
            "отметка порта модулей осталась после тишины — лидар его больше не откроет"
    finally:
        source._stop = True
        time.sleep(0.4)      # дочитываем до закрытия мастера: иначе порт шумит в логе
        os.close(master)


def test_lidar_does_not_probe_the_port_of_the_chassis_modules(state_dir, fake_usb_like,
                                                              monkeypatch, manager):
    """Лидар не открывает порт модулей шасси и видно, почему: ``tried`` и ``chassis_ports``."""
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")
    monkeypatch.setattr(lidar, "LIDAR_RESCAN_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_PROBE_TIMEOUT_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_ATTEMPT_MAX_S", 0.2)
    module_port, *others = fake_usb_like
    lidar.tag_module_port(module_port)
    opened = silent_serial(monkeypatch)

    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: any(port in opened for port in others), timeout=5.0), \
            "лидар не попробовал ни один порт"
        assert wait_for(
            lambda: any(item["note"] == "модуль шасси" for item in manager.tried), timeout=5.0
        ), "пропущенный порт модулей не виден в tried"
        time.sleep(0.3)
        assert module_port not in opened, "лидар открыл порт, который читает источник шасси"
        status = manager.get_status()
        assert module_port in status["chassis_ports"]
        assert status["priority_s"] > 0.0, "состояние «ищу порт» не опубликовано"
        modules = [p for p in status["usb_diagnostics"]["ports"] if p["chassis_module"]]
        assert [p["port"] for p in modules] == [module_port]
    finally:
        manager._stop = True
        worker.join(timeout=3.0)


def test_all_ports_as_modules_is_reported_honestly(state_dir, fake_usb_like, monkeypatch, manager):
    """Все порты заняты модулями шасси — так и написано, а не «проверь кабель лидара»."""
    monkeypatch.setattr(lidar, "LIDAR_RESCAN_S", 0.05)
    for port in fake_usb_like:
        lidar.tag_module_port(port)

    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: "не найден" in manager.reason, timeout=5.0)
        status = manager.get_status()
        assert status["candidates"] == []
        assert "модулями шасси" in status["hint"]
        assert sorted(status["chassis_ports"]) == sorted(fake_usb_like)
    finally:
        manager._stop = True
        worker.join(timeout=3.0)


def test_manager_publishes_the_port_it_found(state_dir, manager):
    """Найденный порт публикуется: источник шасси его больше не занимает."""
    port = state_dir.parent / "ttyUSB1"
    port.write_text("")
    manager._mark_found({"port": str(port), "protocol": "lds01", "baud": 230400, "motor": "none"})
    assert lidar.lidar_active_port() == str(port)
    assert lidar.lidar_holds_port(str(port)) is True
    manager.stop()
    assert lidar.lidar_active_port() is None


def test_state_stays_fresh_while_the_scan_goes(state_dir, manager):
    """Пока идут витки, состояние обновляется — иначе порт лидара «протухнет» для модулей."""
    port = state_dir.parent / "ttyUSB1"
    port.write_text("")
    manager._mark_found({"port": str(port), "protocol": "rplidar", "baud": 115200, "motor": "dtr-low"})
    manager._state_written = time.time() - 10          # как будто писали давно
    manager._store_scan([{"a": 0.0, "r": 1.5}], "serial:ttyUSB1")
    assert lidar.lidar_active_port() == str(port)
    written = json.loads(lidar.lidar_state_file().read_text("utf-8"))
    assert time.time() - written["at"] < lidar.LIDAR_STATE_TTL_S
