"""Проверки чтения USB/serial лидара: подбор порта, протокола и скорости.

Железа в тестах нет, поэтому pyserial подменяется виртуальными устройствами:
«RPLIDAR» (отвечает на SCAN и отдаёт точки, пока крутится мотор) и «LD06/LD19»
(односторонний поток пакетов 0x54 0x2C). Это позволяет проверить ровно те
ситуации, из-за которых на экране было «скана лидара нет»: мотор не включён
линией DTR/RTS, другая скорость порта (A3/S1 — 256000), порт занят вторым
процессом робота.
"""

from __future__ import annotations

import importlib.util
import json
import math
import pathlib
import sys
import threading
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from robot_control import lidar
from robot_control.lidar import (
    Ld06Stream,
    LidarManager,
    RplidarStream,
    ScanAccumulator,
    discover_ports,
    ld06_crc_ok,
    lidar_profiles,
    parse_rplidar_point,
    probe_port,
)

# ---------------------------------------------------------------------------
# Кодирование потоков «устройств»
# ---------------------------------------------------------------------------


def encode_rplidar_node(angle_deg: float, dist_m: float, *, start: bool = False, quality: int = 10) -> bytes:
    """5-байтовый пакет RPLIDAR (обратный к parse_rplidar_point)."""
    angle_q6 = int(round((angle_deg % 360.0) * 64.0)) & 0x7FFF
    dist_q2 = int(round(dist_m * 1000.0 * 4.0)) & 0x3FFF
    b0 = ((quality & 0x3F) << 2) | (0x01 if start else 0x02)
    b1 = ((angle_q6 & 0x7F) << 1) | 0x01
    b2 = (angle_q6 >> 7) & 0xFF
    b3 = dist_q2 & 0xFF
    b4 = (dist_q2 >> 8) & 0xFF
    return bytes([b0, b1, b2, b3, b4])


def encode_ld06_packet(start_deg: float, dist_m: float = 1.0, *, crc_ok: bool = True) -> bytes:
    """47-байтовый пакет LD06/LD19 с 12 точками."""
    pkt = bytearray(47)
    pkt[0], pkt[1] = 0x54, 0x2C
    pkt[2], pkt[3] = 0xE8, 0x03          # скорость, °/с
    start = int(round((start_deg % 360.0) * 100.0)) & 0xFFFF
    pkt[4], pkt[5] = start & 0xFF, start >> 8
    for i in range(12):
        idx = 6 + i * 3
        dist_mm = int(round(dist_m * 1000.0))
        pkt[idx] = dist_mm & 0xFF
        pkt[idx + 1] = (dist_mm >> 8) & 0xFF
        pkt[idx + 2] = 200                # достоверность
    end = int(round(((start_deg + 27.0) % 360.0) * 100.0)) & 0xFFFF
    pkt[42], pkt[43] = end & 0xFF, end >> 8
    crc = 0
    for byte in pkt[:46]:
        crc ^= byte
    pkt[46] = crc if crc_ok else (crc ^ 0xFF)
    return bytes(pkt)


# ---------------------------------------------------------------------------
# Виртуальные устройства и модуль serial
# ---------------------------------------------------------------------------


class FakeDevice:
    """Базовое устройство: умеет отдавать байты и запоминать запись."""

    def __init__(self, *, baud: int) -> None:
        self.baud = baud
        self.lines = {"dtr": True, "rts": True}
        self.buffer = bytearray()
        self.written: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if port.baudrate != self.baud:
            return b""  # устройство живёт на своей скорости
        if not self.buffer:
            return b""
        chunk = bytes(self.buffer[:size])
        del self.buffer[:size]
        return chunk

    def flush_input(self) -> None:
        pass


class FakeRplidar(FakeDevice):
    """Виртуальный RPLIDAR: мотор включается линией USB-переходника."""

    def __init__(
        self,
        *,
        baud: int = 115200,
        motor_line: str = "dtr",
        motor_level: bool = False,
        points_per_rev: int = 30,
        revolutions: int = 3,
        respond: bool = True,
    ) -> None:
        super().__init__(baud=baud)
        self.motor_line = motor_line
        self.motor_level = motor_level
        self.points_per_rev = points_per_rev
        self.revolutions = revolutions
        self.respond = respond
        self.scanning = False
        self.emitted = 0
        self.scan_commands = 0

    @property
    def motor_on(self) -> bool:
        return bool(self.lines.get(self.motor_line)) is bool(self.motor_level)

    def write(self, data: bytes) -> int:
        size = super().write(data)
        if bytes(data) == lidar.RPLIDAR_CMD_SCAN:
            self.scan_commands += 1
            if self.respond:
                self.scanning = True
                self.buffer.extend(bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81]))
        elif bytes(data) == lidar.RPLIDAR_CMD_STOP:
            self.scanning = False
        return size

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if port.baudrate != self.baud:
            return b""
        if self.scanning and self.motor_on and self.emitted < self.revolutions:
            self._emit_revolution()
        return super().read(size, port)

    def _emit_revolution(self) -> None:
        self.emitted += 1
        for i in range(self.points_per_rev):
            self.buffer.extend(
                encode_rplidar_node(
                    angle_deg=i * 360.0 / self.points_per_rev,
                    dist_m=1.0 + i * 0.01,
                    start=(i == 0),
                )
            )
        # точка с битом начала закрывает текущий виток у читателя
        self.buffer.extend(encode_rplidar_node(0.0, 2.0, start=True))


class FakeLd06(FakeDevice):
    """Виртуальный LD06/LD19: поток пакетов без команд управления."""

    def __init__(
        self,
        *,
        baud: int = 230400,
        revolutions: int = 2,
        packets_per_rev: int = 12,
        crc_ok: bool = True,
    ) -> None:
        super().__init__(baud=baud)
        self.revolutions = revolutions
        self.packets_per_rev = packets_per_rev
        self.crc_ok = crc_ok
        self.sent = 0

    def read(self, size: int, port: "FakeSerial") -> bytes:
        if port.baudrate != self.baud:
            return b""
        while len(self.buffer) < size and self.sent < self.revolutions * self.packets_per_rev:
            step = 360.0 / self.packets_per_rev
            start_deg = (self.sent % self.packets_per_rev) * step
            self.buffer.extend(
                encode_ld06_packet(start_deg, dist_m=1.0 + 0.01 * self.sent, crc_ok=self.crc_ok)
            )
            self.sent += 1
        return super().read(size, port)


class FakeSerialError(Exception):
    """Аналог serial.SerialException."""


class FakeSerial:
    """Минимальный порт: только то, что использует robot_control.lidar."""

    devices: dict[str, FakeDevice] = {}

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2, exclusive: bool = True) -> None:
        device = FakeSerial.devices.get(port)
        if device is None:
            raise FakeSerialError("could not open port %s" % port)
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.exclusive = exclusive
        self.device = device
        self.closed = False

    @property
    def dtr(self) -> bool:
        return bool(self.device.lines["dtr"])

    @dtr.setter
    def dtr(self, value: bool) -> None:
        self.device.lines["dtr"] = bool(value)

    @property
    def rts(self) -> bool:
        return bool(self.device.lines["rts"])

    @rts.setter
    def rts(self, value: bool) -> None:
        self.device.lines["rts"] = bool(value)

    def read(self, size: int = 1) -> bytes:
        return self.device.read(size, self)

    def write(self, data: bytes) -> int:
        return self.device.write(bytes(data))

    def reset_input_buffer(self) -> None:
        self.device.flush_input()

    def close(self) -> None:
        self.closed = True

    def __enter__(self) -> "FakeSerial":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


@pytest.fixture
def fake_serial(monkeypatch):
    """Подменяет pyserial и даёт чистую таблицу устройств."""
    module = types.ModuleType("serial")
    module.Serial = FakeSerial
    module.SerialException = FakeSerialError
    monkeypatch.setitem(sys.modules, "serial", module)
    FakeSerial.devices = {}
    yield module
    FakeSerial.devices = {}


@pytest.fixture
def fake_port(tmp_path):
    """Путь порта, который существует в файловой системе (иначе его не найти)."""
    path = tmp_path / "ttyUSB7"
    path.write_text("")
    return str(path)


@pytest.fixture(autouse=True)
def fast_lidar(monkeypatch):
    """Короткие таймауты и никаких соседних процессов по умолчанию."""
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "")
    monkeypatch.setattr(lidar, "LIDAR_PROBE_TIMEOUT_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_DATA_TIMEOUT_S", 0.05)
    monkeypatch.setattr(lidar, "LIDAR_RESCAN_S", 0.05)
    yield


def wait_for(predicate, timeout: float = 3.0, pause: float = 0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(pause)
    return False


# ---------------------------------------------------------------------------
# Разбор потоков
# ---------------------------------------------------------------------------


def test_rplidar_node_round_trip():
    point = parse_rplidar_point(encode_rplidar_node(90.0, 2.0, start=True))
    assert point is not None
    assert point["r"] == 2.0
    assert abs(point["a"] + math.pi / 2) < 0.001  # угол по часовой → против часовой


def test_rplidar_stream_joins_split_packets():
    """Пакет, разрезанный между чтениями порта, собирается обратно."""
    stream = RplidarStream()
    node = encode_rplidar_node(45.0, 3.25, start=True)
    assert stream.feed(node[:2]) == []
    points = stream.feed(node[2:])
    assert len(points) == 1
    assert points[0][0]["r"] == 3.25 and points[0][1] is True


def test_rplidar_stream_recovers_after_garbage():
    """После мусора поток продолжает отдавать настоящие точки витка."""
    stream = RplidarStream()
    nodes = b"".join(
        encode_rplidar_node(i * 30.0, 1.0 + i * 0.1, start=(i == 0)) for i in range(12)
    )
    points = stream.feed(b"\xf7\x00\x9a" + nodes)
    radii = [p["r"] for p, _ in points]
    # на стыке с мусором один пакет может «съесться» ложным совпадением (так же,
    # как у любого драйвера RPLIDAR), но виток восстанавливается и точек хватает
    assert len([r for r in radii if r <= 2.2]) >= 11
    assert len(points) >= 11


def test_ld06_crc_and_revolution_detection():
    good = encode_ld06_packet(0.0)
    assert ld06_crc_ok(good) is True
    assert ld06_crc_ok(encode_ld06_packet(0.0, crc_ok=False)) is False

    stream = Ld06Stream()
    packets = []
    for start_deg in (0.0, 30.0, 60.0, 0.0, 30.0):  # второй виток начинается с 0°
        packets.extend(stream.feed(encode_ld06_packet(start_deg)))
    assert len(packets) == 5
    assert [revolution for _, revolution in packets] == [False, False, False, True, False]
    assert all(len(pts) == 12 for pts, _ in packets)


def test_ld06_stream_tolerates_broken_crc():
    stream = Ld06Stream(check_crc=True, crc_bad_limit=3)
    for _ in range(4):
        stream.feed(encode_ld06_packet(0.0, crc_ok=False))
    assert stream.check_crc is False           # перешли в режим без CRC
    packets = stream.feed(encode_ld06_packet(90.0, crc_ok=False))
    assert len(packets) == 1 and len(packets[0][0]) == 12


def test_scan_accumulator_publishes_revolution_and_gap():
    published = []
    acc = ScanAccumulator(published.append, min_points=3, gap_s=0.0)
    acc.add([{"a": 0.0, "r": 1.0}, {"a": 0.1, "r": 1.1}])
    assert published == []
    acc.add([{"a": 0.2, "r": 1.2}], revolution_done=True)
    assert len(published) == 1 and len(published[0]) == 3
    # виток без бита начала: отдаём накопленное по паузе
    acc.add([{"a": 0.3, "r": 1.3}, {"a": 0.4, "r": 1.4}, {"a": 0.5, "r": 1.5}])
    assert len(published) == 2


# ---------------------------------------------------------------------------
# Поиск портов и профилей
# ---------------------------------------------------------------------------


def test_lidar_profiles_order_and_overrides():
    assert lidar_profiles("auto")[0] == ("rplidar", 115200)
    assert ("ld06", 230400) in lidar_profiles("auto")
    assert lidar_profiles("ld06") == [("ld06", 230400)]
    assert lidar_profiles("auto", 230400)[0] == ("ld06", 230400)
    assert lidar_profiles("rplidar", 256000)[0] == ("rplidar", 256000)


def test_discover_ports_configured_glob_and_exclusions(tmp_path, monkeypatch):
    new = tmp_path / "ttyUSB7"
    other = tmp_path / "ttyUSB9"
    new.write_text("")
    other.write_text("")
    ports = discover_ports(configured=str(tmp_path / "ttyUSB*"))
    assert [p for p in ports if p.endswith(("ttyUSB7", "ttyUSB9"))] == [str(new), str(other)]

    # порт мастера шасси лидару не отдаём
    ports = discover_ports(configured=str(tmp_path / "ttyUSB*"), excluded={str(new)})
    assert str(new) not in ports and str(other) in ports


def test_discover_ports_reads_master_and_modules_from_env(tmp_path, monkeypatch):
    master = tmp_path / "ttyACM0"
    module = tmp_path / "ttyUSB1"
    for path in (master, module):
        path.write_text("")
    monkeypatch.setenv("RC_SLAM_PORT", str(master))
    monkeypatch.setenv("RC_SLAM_PORTS", "%s,%s" % (module, tmp_path / "ttyUSB2"))
    excluded = lidar.excluded_ports()
    assert str(master) in excluded
    assert str(module) in excluded


def test_discover_ports_skips_autoprobe(tmp_path, monkeypatch):
    port = tmp_path / "ttyUSB3"
    port.write_text("")
    monkeypatch.setattr(lidar.glob, "glob", lambda pattern, **_: [str(port)] if "ttyUSB" in pattern else [])
    assert discover_ports(autoprobe=False) == []
    assert str(port) in discover_ports(autoprobe=True)


# ---------------------------------------------------------------------------
# Подбор параметров на порту
# ---------------------------------------------------------------------------


def test_probe_port_finds_rplidar_hidden_behind_motor_line(fake_serial, fake_port):
    """Мотор включается только линией rts-low — перебор обязан до неё дойти."""
    device = FakeRplidar(motor_line="rts", motor_level=False)
    FakeSerial.devices[fake_port] = device
    published = []
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=published.append)
    assert found is not None
    assert found["protocol"] == "rplidar"
    assert found["baud"] == 115200
    assert found["motor"] == "rts-low"          # DTR-варианты не помогли
    assert published and len(published[0]) >= lidar.LIDAR_MIN_POINTS
    assert device.motor_on is True


def test_probe_port_leaves_motor_running(fake_serial, fake_port):
    """RPLIDAR A1/A2: мотор включается DTR=низкий, и линия НЕ глушится в конце.

    Старый код в конце выставлял DTR=True (сброс после открытия порта) и тем
    самым останавливал мотор — скан так и не появлялся.
    """
    device = FakeRplidar(motor_line="dtr", motor_level=False)
    FakeSerial.devices[fake_port] = device
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05)
    assert found is not None and found["motor"] == "dtr-low"
    assert device.lines["dtr"] is False
    assert device.motor_on is True


def test_probe_port_keeps_trying_bauds_for_rplidar_a3(fake_serial, fake_port):
    """RPLIDAR A3 отвечает только на 256000 — как на USB-адаптере с переключателем."""
    FakeSerial.devices[fake_port] = FakeRplidar(baud=256000)
    published = []
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=published.append)
    assert found is not None and found["baud"] == 256000
    assert published


def test_probe_port_finds_ld06(fake_serial, fake_port):
    FakeSerial.devices[fake_port] = FakeLd06()
    published = []
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, publish=published.append)
    assert found is not None
    assert found["protocol"] == "ld06"
    assert found["baud"] == 230400
    assert published and len(published[0]) >= lidar.LIDAR_MIN_POINTS


def test_probe_port_publishes_nothing_without_real_points(fake_serial, fake_port):
    """Ответ на SCAN есть, но мотор стоит: «скана» из чужих байт быть не должно."""
    FakeSerial.devices[fake_port] = FakeRplidar(motor_line="dtr", motor_level=False)
    published = []
    with FakeSerial(fake_port) as ser:
        found = probe_port(ser, fake_port, timeout=0.05, motor="none", publish=published.append)
    assert found is None
    assert published == []


def test_probe_port_ignores_port_without_answer(fake_serial, fake_port):
    FakeSerial.devices[fake_port] = FakeDevice(baud=9600)
    with FakeSerial(fake_port) as ser:
        assert probe_port(ser, fake_port, timeout=0.05) is None


# ---------------------------------------------------------------------------
# Менеджер: рабочий поток, статус, замок порта, второй процесс
# ---------------------------------------------------------------------------


def test_manager_reads_fake_rplidar_and_reports_detected(fake_serial, fake_port, monkeypatch):
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    FakeSerial.devices[fake_port] = FakeRplidar()
    manager = LidarManager()
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: bool(manager.get_scan()))
        status = manager.get_status()
        assert status["connected"] is True
        assert status["detected"]["port"] == fake_port
        assert status["detected"]["motor"] == "dtr-low"
        assert status["reason"].startswith("скан идёт")
        assert manager.last_error == ""
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_reports_missing_pyserial(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", None)  # import serial → ImportError
    manager = LidarManager()
    manager._serial_worker()                          # один проход и выход
    status = manager.get_status()
    assert "pyserial" in status["reason"]
    assert "pyserial" in status["hint"]


def test_manager_reports_unanswered_port(fake_serial, fake_port, monkeypatch):
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    FakeSerial.devices[fake_port] = FakeDevice(baud=9600)
    manager = LidarManager()
    assert manager._read_port(fake_serial, fake_port) is False
    status = manager.get_status()
    assert status["connected"] is False
    assert "не отвечает" in status["reason"]
    assert "256000" in status["hint"]      # подсказка про скорость модели
    assert status["detected"] is None


def test_manager_report_has_help_hint_when_no_port(fake_serial, monkeypatch):
    """Порта нет — в статусе должна быть причина и подсказка, а не тишина."""
    monkeypatch.setenv("RC_LIDAR_PORT", "/dev/ttyNOPE")
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    manager = LidarManager()
    assert manager.candidate_ports() == []
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: "не найден" in manager.get_status()["reason"])
        status = manager.get_status()
        assert "RC_LIDAR_PORT" in status["hint"]
        assert "lidar_check" in status["hint"]
        assert status["candidates"] == []
    finally:
        manager.stop()
        worker.join(timeout=2.0)


def test_manager_skips_port_of_another_robot_process(fake_serial, fake_port, monkeypatch):
    """Занятый вторым процессом порт не перехватываем, а просим скан у соседа."""
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    monkeypatch.setenv("RC_LIDAR_PEER_URLS", "http://127.0.0.1:1/api/lidar")  # соседа нет
    FakeSerial.devices[fake_port] = FakeRplidar()
    owner = LidarManager()
    manager = LidarManager()
    assert owner._acquire_port_lock(fake_port) is True
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    try:
        assert wait_for(lambda: "занят" in manager.get_status()["reason"])
        assert manager.get_scan() is None
    finally:
        manager.stop()
        owner._release_port_lock()
        worker.join(timeout=2.0)


def test_port_lock_between_processes(fake_serial, monkeypatch):
    """Порт лидара открывается один раз: второй процесс робота замок не получает."""
    monkeypatch.setenv("RC_LIDAR_PORT", "/dev/ttyTEST")
    FakeSerial.devices["/dev/ttyTEST"] = FakeRplidar()
    first, second = LidarManager(), LidarManager()
    try:
        assert first._acquire_port_lock("/dev/ttyTEST") is True
        assert second._acquire_port_lock("/dev/ttyTEST") is False
        first._release_port_lock()
        assert second._acquire_port_lock("/dev/ttyTEST") is True
    finally:
        first._release_port_lock()
        second._release_port_lock()


def test_second_process_takes_scan_over_http(fake_serial, monkeypatch):
    """Второй процесс получает скан по HTTP — экран не остаётся пустым."""
    payload = {
        "scan": [{"a": 0.5, "r": 1.5}, {"a": -0.5, "r": 2.5}],
        "detected": {"port": "/dev/ttyTEST", "protocol": "rplidar", "baud": 115200, "motor": "dtr-low"},
    }

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):                                     # noqa: N802
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):                         # тишина в тестах
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        monkeypatch.setenv("RC_LIDAR_PEER_URLS", "http://127.0.0.1:%d/api/lidar" % server.server_port)
        manager = LidarManager()
        assert manager._peer_cycle() is True
        assert manager.get_scan() == payload["scan"]
        assert manager.detected["baud"] == 115200
        assert "второго процесса" in manager.reason
        assert manager.get_status()["source"] == "peer"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Диагностический скрипт scripts/lidar_check.py
# ---------------------------------------------------------------------------

CLI_SCRIPT = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "lidar_check.py"


def load_lidar_check():
    """Загружает скрипт как модуль (он рассчитан на запуск с робота)."""
    spec = importlib.util.spec_from_file_location("lidar_check_cli", CLI_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_lidar_check_cli_lists_ports(fake_serial, fake_port, monkeypatch, capsys):
    monkeypatch.setenv("RC_LIDAR_PORT", fake_port)
    monkeypatch.setenv("RC_LIDAR_AUTOPROBE", "0")
    module = load_lidar_check()
    assert module.main(["--list"]) == 0
    assert fake_port in capsys.readouterr().out


def test_lidar_check_cli_finds_rplidar(fake_serial, fake_port, capsys):
    FakeSerial.devices[fake_port] = FakeRplidar(baud=256000)
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--seconds", "0.2"]) == 0
    out = capsys.readouterr().out
    assert "256000" in out and "RC_LIDAR_PORT" in out


def test_lidar_check_cli_reports_dead_port(fake_serial, fake_port, capsys):
    FakeSerial.devices[fake_port] = FakeDevice(baud=9600)
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--seconds", "0.1"]) == 1
    out = capsys.readouterr().out
    assert "не найден" in out
    assert "115200" in out          # подсказка про скорости моделей


def test_lidar_check_cli_json_for_scripts(fake_serial, fake_port, capsys):
    FakeSerial.devices[fake_port] = FakeLd06()
    module = load_lidar_check()
    assert module.main(["--port", fake_port, "--json", "--seconds", "0.2"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["ok"] is True
    assert report["reports"][0]["protocol"] == "ld06"
    assert report["reports"][0]["baud"] == 230400
