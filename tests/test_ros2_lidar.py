"""Лидар через ROS 2: окружение, подписка на /scan и статус для экранов.

Железа и ROS 2 в тестах нет, поэтому проверяется то, что ломалось по-настоящему
на роботе:

  * ``rclpy`` не виден из ``.venv`` приложения (ROS 2 ставится системно в
    ``/opt/ros``) — путь и переменные окружения должны добавляться сами;
  * «ROS 2 есть, а скана нет» раньше выглядело как обычное «лидар не найден»:
    теперь причина и подсказка говорят про драйвер, пакет и команду запуска;
  * при источнике ``ros`` приложение не должно открывать USB-порт (его держит
    драйвер) — иначе скан теряли оба;
  * статус драйвера (``/zapret/lidar/status``) обязан доходить до
    ``GET /api/lidar`` → ``ros.supervisor``.

ROS 2 подменяется фальшивым префиксом ``/opt/ros/<дистрибутив>`` во временном
каталоге и (где нужно) фальшивым модулем ``rclpy`` в ``sys.modules``.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from pathlib import Path

import pytest

from robot_control import ros_env
from robot_control.lidar import LidarManager, normalize_lidar_settings, normalize_source
from robot_control.ros_lidar import RosLidar

from conftest import login

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(autouse=True)
def _clean_ros_env(monkeypatch):
    """Каждый тест начинает с чистого кэша: префикс ROS 2 задаётся заново."""
    for key in ("RC_ROS_PREFIX", "ROS_DISTRO", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH",
                "RC_LIDAR_SOURCE", "RC_LIDAR_ROS_TOPIC", "RC_LIDAR_PORT",
                "RC_LIDAR_PROTOCOL", "RC_LIDAR_BAUD", "RC_LIDAR_MOTOR",
                "RC_LIDAR_AUTOPROBE"):
        monkeypatch.delenv(key, raising=False)
    ros_env.reset_cache()
    yield
    ros_env.reset_cache()


def make_prefix(root: Path, distro: str = "humble", *,
                drivers: tuple[str, ...] = ("rplidar_ros",),
                python: str | None = None) -> Path:
    """Создаёт каталог, похожий на установленный ROS 2 (для тестов)."""
    prefix = root / distro
    (prefix / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    version = python or "%d.%d" % sys.version_info[:2]
    (prefix / "lib" / ("python%s" % version) / "site-packages").mkdir(parents=True)
    (prefix / "setup.bash").write_text("# ROS 2 %s\n" % distro, encoding="utf-8")
    for package in ("rclpy", "sensor_msgs", "std_msgs", "std_srvs", *drivers):
        (prefix / "share" / "ament_index" / "resource_index" / "packages" / package
         ).write_text("", encoding="utf-8")
    return prefix


@pytest.fixture
def ros_prefix(tmp_path, monkeypatch):
    """ROS 2 во временном каталоге + фальшивый ``rclpy`` (чтобы «импорт» сработал)."""
    prefix = make_prefix(tmp_path)
    monkeypatch.setenv("RC_ROS_PREFIX", str(prefix))
    rclpy = type(sys)("rclpy")
    rclpy.ok = lambda: True
    rclpy.init = lambda args=None: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    ros_env.reset_cache()
    return prefix


# --------------------------------------------------------------------------- #
# Окружение ROS 2 (robot_control/ros_env.py)
# --------------------------------------------------------------------------- #


def test_prefix_and_distro_are_found_from_the_environment(tmp_path, monkeypatch):
    prefix = make_prefix(tmp_path, "jazzy")
    monkeypatch.setenv("RC_ROS_PREFIX", str(prefix))
    assert ros_env.prefix() == prefix
    assert ros_env.distro() == "jazzy"
    assert str(prefix) in [str(item) for item in ros_env.candidate_prefixes()]


def test_non_ros_directory_is_not_taken_as_prefix(tmp_path, monkeypatch):
    """Каталог без setup.bash и библиотек не считается ROS 2 — иначе подсказки врут."""
    empty = tmp_path / "opt" / "humble"
    empty.mkdir(parents=True)
    monkeypatch.setenv("RC_ROS_PREFIX", str(empty))
    assert ros_env.prefix() is None or ros_env.prefix() != empty


def test_ensure_python_path_makes_site_packages_importable(ros_prefix, monkeypatch):
    """Ключевая поломка: .venv не видит системный rclpy, пока пути не добавлены."""
    entries = ros_env.python_path_entries(ros_prefix)
    assert any("site-packages" in entry for entry in entries)
    for entry in entries:
        sys.path[:] = [item for item in sys.path if item != entry]
    assert ros_env.ensure_python_path(ros_prefix) is True
    assert all(entry in sys.path for entry in entries if "site-packages" in entry)
    # Переменные нужны уже загруженным библиотекам ROS 2, а не только sys.path.
    assert str(ros_prefix) in os.environ.get("AMENT_PREFIX_PATH", "")
    assert str(ros_prefix) in os.environ.get("LD_LIBRARY_PATH", "")
    assert str(ros_prefix) in os.environ.get("PYTHONPATH", "")


def test_installed_packages_are_read_from_the_ament_index(ros_prefix):
    packages = ros_env.installed_packages(ros_prefix)
    assert {"rplidar_ros", "sensor_msgs"} <= packages
    assert "ldlidar_stl_ros2" not in packages


def test_describe_reports_distro_packages_and_hint(ros_prefix):
    ros_env.reset_cache()
    info = ros_env.describe()
    assert info["available"] is True
    assert info["distro"] == "humble"
    assert "rplidar_ros" in info["packages"]
    assert info["drivers"]["rplidar"] == ["rplidar_ros"]
    assert info["drivers"]["lds01"] == []
    assert info["topic"] == "/scan"
    # Пакеты лидаров, которых нет, видны как missing для конкретного типа.
    assert ros_env.driver_packages("lds01") == ("hls_lfcd_lds_driver",)
    assert "ros-humble-hls-lfcd-lds-driver" in ros_env.install_hint("turtlebot3")
    assert ros_env.launch_command(model="lds01", port="/dev/ttyUSB0") \
        == "./scripts/ros2_lidar.sh --model lds01 --port /dev/ttyUSB0"


def test_describe_is_cached_until_reset(ros_prefix):
    ros_env.reset_cache()
    first = ros_env.describe()
    (ros_prefix / "share" / "ament_index" / "resource_index" / "packages" / "hls_lfcd_lds_driver"
     ).write_text("", encoding="utf-8")
    assert ros_env.describe()["packages"] == first["packages"], "кэш статуса не работает"
    ros_env.reset_cache()
    assert "hls_lfcd_lds_driver" in ros_env.describe()["packages"]


def test_missing_ros_is_explained_with_the_setup_command(tmp_path):
    missing = ros_env.missing_reason() if ros_env.prefix() is None else ""
    assert "ROS 2 не установлен" in missing
    assert ros_env.SETUP_COMMAND in missing


def test_import_rclpy_without_ros_raises_with_a_hint(monkeypatch):
    monkeypatch.setattr(sys, "modules", dict(sys.modules))
    sys.modules.pop("rclpy", None)
    ros_env.reset_cache()
    if ros_env.prefix() is not None:  # pragma: no cover — на машине с ROS 2
        pytest.skip("на этой машине ROS 2 установлен")
    with pytest.raises(ros_env.RosUnavailable) as excinfo:
        ros_env.import_rclpy()
    assert ros_env.SETUP_COMMAND in str(excinfo.value)


# --------------------------------------------------------------------------- #
# Подписка на скан и состояние (robot_control/ros_lidar.py)
# --------------------------------------------------------------------------- #


def laserscan(points: int = 360, step: float = 0.0174) -> dict:
    return {
        "ranges": [1.5] * points,
        "angle_min": -3.14,
        "angle_max": -3.14 + step * points,
        "angle_increment": step,
        "range_min": 0.05,
        "range_max": 12.0,
    }


def test_ros_lidar_counts_points_and_reports_freshness():
    seen = {}

    def on_scan(**fields):
        seen.update(fields)
        return len(fields["ranges"])

    ros = RosLidar(on_scan, topic="/scan")
    assert ros.fresh() is False
    assert ros.feed_laserscan(**laserscan(360)) == 360
    assert seen["source_label"] == "ros"
    assert seen["range_max"] == 12.0
    assert ros.fresh() is True
    assert ros.points == 360
    assert ros.age() is not None and ros.age() < 1.0
    status = ros.status()
    assert status["connected"] is True and status["points"] == 360
    assert status["topic"] == "/scan"
    assert status["rescan_service"] == "/lidar_supervisor/rescan"
    assert "ROS 2" in status["reason"] and "360" in status["reason"]


def test_ros_lidar_rejects_foreign_json_in_the_status_topic():
    """Чужой JSON в топике состояния не должен выдаваться за состояние нашего лидара."""
    ros = RosLidar()
    # Подписка поднята (в тесте ROS 2 нет, поэтому состояние задаём явно).
    ros.available = True
    ros.started = True
    assert ros.feed_supervisor_json("не json") is False
    assert ros.feed_supervisor_json('{"backend": "other", "state": "running"}') is False
    assert ros.supervisor == {}
    payload = json.dumps({"backend": "zapret_lidar", "state": "failed",
                          "reason": "ни один драйвер не отдал /scan",
                          "hint": "поставь пакет ros-humble-rplidar-ros",
                          "tried": [{"driver": "rplidar", "port": "/dev/ttyUSB0",
                                     "note": "нет /scan за 8 с"}]}, ensure_ascii=False)
    assert ros.feed_supervisor_json(payload) is True
    assert ros.supervisor_fresh() is True
    assert "ни один драйвер" in ros.summary()
    assert "ros-humble-rplidar-ros" in ros.hint()


def test_ros_lidar_tells_apart_no_driver_and_dead_driver():
    ros = RosLidar()
    ros.available = True
    ros.started = True
    ros.note_publishers(0)
    assert "никто не публикует" in ros.summary()
    assert "./scripts/ros2_lidar.sh" in ros.hint()

    ros.note_publishers(2, ["rplidar_node", "lidar_supervisor"])
    assert "публикуют /scan" in ros.summary()
    assert "без точек" in ros.hint()

    # Драйвер работал и пропал: это отдельная причина, а не «драйвер не запущен».
    ros.feed_laserscan(**laserscan(120))
    ros.scan_at = time.time() - 10.0
    assert "оборвался" in ros.summary()


def test_ros_lidar_without_ros_2_explains_what_to_install():
    ros = RosLidar()
    assert ros.start() is False, "без rclpy подписка не должна подниматься"
    assert ros.available is False
    assert ros.error
    status = ros.status()
    assert status["connected"] is False
    assert "./scripts/ros2_setup.sh" in status["hint"]


def test_ros_lidar_rescan_without_supervisor_is_reported():
    """Кнопка «Пересканировать» без запущенного супервизора говорит об этом прямо."""
    ros = RosLidar()
    assert ros.request_rescan() is False
    assert "запусти" in ros.rescan_result


# --------------------------------------------------------------------------- #
# Источник данных в менеджере лидара (robot_control/lidar.py)
# --------------------------------------------------------------------------- #


@pytest.fixture
def manager():
    previous = LidarManager._instance
    instance = LidarManager()
    LidarManager._instance = instance
    try:
        yield instance
    finally:
        instance.stop()
        LidarManager._instance = previous


def test_source_synonyms_and_refusal():
    assert normalize_source("ros2") == "ros"
    assert normalize_source("scan") == "ros"
    assert normalize_source("USB") == "serial"
    assert normalize_source("что угодно") == "auto"
    assert normalize_lidar_settings({"source": "ros"})["source"] == "ros"
    with pytest.raises(ValueError, match="Источник данных лидара"):
        normalize_lidar_settings({"source": "spi"})


def test_serial_source_never_uses_ros(manager, monkeypatch):
    """RC_LIDAR_SOURCE=serial отключает ROS 2: это стенд без ROS и отладка USB."""
    monkeypatch.setenv("RC_LIDAR_SOURCE", "serial")
    settings = normalize_lidar_settings({**manager.current_settings(), "source": "serial"})
    manager._apply_settings(settings)
    assert manager.source_mode == "serial"
    assert manager._ros_owns_lidar() is False
    assert manager._start_ros() is False
    assert manager.current_settings()["source"] == "serial"


def test_ros_source_owns_the_lidar_even_without_a_serial_port(manager, monkeypatch):
    """При RC_LIDAR_SOURCE=ros USB-порт лидара не перебирается вовсе."""
    monkeypatch.setenv("RC_LIDAR_SOURCE", "ros")
    manager.source_mode = "ros"
    assert manager._ros_owns_lidar() is True
    status = manager.get_status()
    assert status["source_mode"] == "ros"
    ros = status["ros"]
    assert ros["topic"] == "/scan"
    assert ros["owns_lidar"] is True
    assert ros["rescan_service"] == "/lidar_supervisor/rescan"


def test_auto_uses_ros_when_ros_is_available(manager, monkeypatch):
    class FakeRos:
        available = True
        topic = "/scan"

        def fresh(self):
            return False

        def summary(self):
            return "ROS 2 есть, но /scan никто не публикует"

        def hint(self):
            return "запусти драйвер: ./scripts/ros2_lidar.sh"

        def status(self):
            return {"available": True, "topic": self.topic, "connected": False,
                    "reason": self.summary(), "hint": self.hint(), "publishers": 0}

    manager.ros = FakeRos()
    manager.source_mode = "auto"
    assert manager._ros_owns_lidar() is True, "auto обязан предпочитать ROS 2"
    # Воркер в этом режиме на порты не ходит: их держит драйвер ROS 2.
    monkeypatch.setattr("robot_control.lidar.LIDAR_RESCAN_S", 0.05)
    monkeypatch.setattr("robot_control.lidar.open_serial",
                        lambda *_a, **_k: pytest.fail("лидар полез на USB-порт при ROS 2"))
    monkeypatch.setattr(manager, "candidate_ports", lambda **_kw: ["/dev/ttyUSB0"])
    worker = threading.Thread(target=manager._serial_worker, daemon=True)
    worker.start()
    time.sleep(0.4)
    manager._stop = True
    worker.join(timeout=3.0)
    assert "ROS 2" in manager.reason
    assert "./scripts/ros2_lidar.sh" in manager.hint


def test_manager_rescan_asks_the_ros_supervisor(manager, monkeypatch):
    calls = []

    class FakeRos:
        available = True
        topic = "/scan"
        rescan_result = "перебор драйверов перезапущен (состояние: probing)"

        def request_rescan(self):
            calls.append("rescan")
            return True

    manager.ros = FakeRos()
    manager.source_mode = "ros"
    monkeypatch.setattr(manager, "start", lambda: None)
    monkeypatch.setattr(manager, "_stop_worker", lambda: None)
    result = manager.restart_search()
    assert calls == ["rescan"]
    assert "перебор драйверов" in result["ros_rescan"]


# --------------------------------------------------------------------------- #
# API приложения: панель «Лидар» видит ROS 2
# --------------------------------------------------------------------------- #


def test_web_lidar_status_exposes_ros_block(client, operator):
    login(client)
    status = client.get("/api/lidar").get_json()
    assert status["source_mode"] in ("auto", "ros", "serial")
    ros = status["ros"]
    for key in ("available", "distro", "topic", "packages", "drivers", "launch",
                "supervisor", "publishers", "rescan_service", "reason", "hint"):
        assert key in ros, key
    assert ros["topic"] == "/scan"


def test_web_lidar_settings_expose_and_save_the_source(client, operator, tmp_path,
                                                       monkeypatch):
    manager = LidarManager()
    previous = LidarManager._instance
    LidarManager._instance = manager
    env = tmp_path / ".env"
    env.write_text("RC_SECRET_KEY=secret\nRC_LIDAR_SOURCE=auto\n", encoding="utf-8")
    monkeypatch.setattr("robot_control.lidar_settings.env_file_path", lambda: env)
    monkeypatch.setattr("robot_control.lidar.env_file_path", lambda: env)
    try:
        login(client)
        page = client.get("/api/lidar/settings")
        assert page.status_code == 200
        body = page.get_json()
        assert "ros" in body["options"]["sources"] and "serial" in body["options"]["sources"]
        assert body["settings"]["source"] == "auto"
        assert body["status"]["ros"]["topic"] == "/scan"
        saved = client.post("/api/lidar/settings", json={
            "source": "ros", "port": "", "protocol": "lds01", "baud": "",
            "motor": "auto", "autoprobe": True,
        }, headers={"X-CSRF-Token": body["csrfToken"]})
        assert saved.status_code == 200, saved.get_data(as_text=True)
        assert saved.get_json()["settings"]["source"] == "ros"
        assert "RC_LIDAR_SOURCE=ros" in env.read_text(encoding="utf-8")
    finally:
        manager.stop()
        LidarManager._instance = previous


def test_kiosk_rescan_reports_the_ros_supervisor_answer(config, monkeypatch):
    """«Пересканировать» на борту отвечает, что именно сделал супервизор ROS 2."""
    from robot_control.kiosk_app import create_kiosk_app

    manager = LidarManager()
    previous = LidarManager._instance
    LidarManager._instance = manager
    calls = []

    class FakeRos:
        available = True
        topic = "/scan"
        rescan_result = "перебор драйверов перезапущен (состояние: probing)"

        def fresh(self):
            return False

        def summary(self):
            return "ROS 2: драйвер подбирается"

        def hint(self):
            return ""

        def request_rescan(self):
            calls.append("rescan")
            return True

        def status(self):
            return {"available": True, "topic": "/scan", "connected": False,
                    "reason": self.summary(), "hint": ""}

    manager.ros = FakeRos()
    manager.source_mode = "ros"
    monkeypatch.setattr(manager, "start", lambda: None)
    monkeypatch.setattr(manager, "_stop_worker", lambda: None)
    try:
        client = create_kiosk_app(config).test_client()
        body = client.post("/api/lidar/rescan", json={}).get_json()
        assert body["ok"] is True
        assert calls == ["rescan"]
        assert "перебор драйверов" in body["ros_rescan"]
        assert body["status"]["ros"]["topic"] == "/scan"
    finally:
        manager.stop()
        LidarManager._instance = previous


def test_kiosk_state_carries_ros_diagnostics_for_the_screen(client, operator):
    """Бортовой экран должен видеть, что скан идёт из ROS 2, а не «просто нет»."""
    login(client)
    data = client.get("/api/state").get_json()["data"]
    ros = data["lidarStatus"]["ros"]
    assert ros["topic"] == "/scan"
    # Публичного статуса супервизора нет — значит, драйвер не запущен: подсказка
    # обязана вести к запуску драйвера, а не к «проверь кабель».
    assert "ros2_lidar.sh" in ros["hint"] or "ros2_setup.sh" in ros["hint"]
