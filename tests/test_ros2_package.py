"""Пакет ROS 2 ``zapret_lidar`` и скрипты установки/запуска.

ROS 2 в тестах нет, и это правильно: пакет построен так, чтобы план запуска,
таблица драйверов и опознание лидара по кадрам проверялись без ROS. Проверяется:

  * таблица драйверов покрывает все лидары, а её пакеты совпадают с тем, что
    обещает приложение (``robot_control/ros_env.py``) — иначе подсказка «что
    поставить» уводила бы оператора не туда;
  * ``--print-plan`` собирает порядок попыток с учётом установленных пакетов,
    портов и результатов проверки порта;
  * опознание кадров (LDS-01 ``0xFA``, LD06 ``0x54 0x2C``, RPLIDAR после SCAN)
    выбирает правильный драйвер автоматически — это то, что раньше делал
    самодельный перебор протоколов внутри приложения;
  * package.xml/setup.py/launch-файл ссылаются на существующие файлы, а apt-имена
    пакетов драйверов совпадают с ``ros-env``;
  * скрипты ``ros2_setup.sh``/``ros2_lidar.sh`` печатают понятные команды и не
    трогают систему без флагов.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
WS = ROOT / "ros2_ws"
PKG = WS / "src" / "zapret_lidar"
SCRIPTS = ROOT / "scripts"

# Пакет ROS 2 без зависимостей от rclpy можно импортировать напрямую: он на
# чистой стандартной библиотеке, поэтому проверяется и на машине без ROS.
# sys.path — корень пакета (ros2_ws/src/zapret_lidar), как это делает colcon.
sys.path.insert(0, str(WS / "src" / "zapret_lidar"))
from zapret_lidar import drivers as ros_drivers  # noqa: E402
from zapret_lidar import frames as ros_frames  # noqa: E402
from zapret_lidar import supervisor as ros_supervisor  # noqa: E402

from robot_control import ros_env  # noqa: E402


# --------------------------------------------------------------------------- #
# Таблица драйверов
# --------------------------------------------------------------------------- #


def test_every_driver_is_importable_from_the_cli_table():
    assert set(ros_drivers.DRIVER_ORDER) == set(ros_drivers.DRIVERS)
    for name, driver in ros_drivers.DRIVERS.items():
        assert driver.package and driver.executable
        assert driver.title and driver.protocols
        assert driver.apt or driver.source, name


def test_driver_packages_match_what_the_web_app_promises():
    """Пакеты драйверов в пакете ROS 2 и в приложении — один и тот же список.

    Приложение показывает оператору «поставь ros-humble-…», супервизор ищет
    пакет в ament-индексе, а проба драйвера запускает узел. Если списки
    разъедутся, подсказка будет вести к пакету, которого супервизор не знает.
    """
    by_protocol: dict[str, set[str]] = {}
    for driver in ros_drivers.DRIVERS.values():
        for protocol in driver.protocols:
            by_protocol.setdefault(protocol, set()).add(driver.package)
    for protocol, packages in ros_env.LIDAR_DRIVER_PACKAGES.items():
        assert by_protocol.get(protocol), "протокол %s не покрыт драйверами" % protocol
        assert set(packages) & by_protocol[protocol], (
            "пакеты протокола %s в приложении (%s) не совпадают с пакетом ROS 2 (%s)"
            % (protocol, ", ".join(packages), ", ".join(sorted(by_protocol[protocol]))))
    # Каждое apt-имя, которое называет приложение, есть и в таблице драйверов.
    for driver in ros_drivers.DRIVERS.values():
        if driver.apt:
            assert driver.package in ros_env.APT_PACKAGES
            assert ros_env.APT_PACKAGES[driver.package] == driver.apt


def test_lds01_driver_is_first_for_turtlebot_burger():
    """Лидар TurtleBot3 на этом роботе основной: он должен пробоваться первым."""
    assert ros_drivers.DRIVER_ORDER[0] == "lds01"
    assert ros_drivers.get_driver("turtlebot3") is ros_drivers.DRIVERS["lds01"]
    assert ros_drivers.get_driver("hls-lfcd2") is ros_drivers.DRIVERS["lds01"]
    assert ros_drivers.get_driver("opencr") is ros_drivers.DRIVERS["opencr"]


def test_plan_prefers_installed_packages_and_probed_ports(tmp_path):
    port = tmp_path / "ttyUSB0"
    port.write_text("")
    other = tmp_path / "ttyUSB1"
    other.write_text("")
    plan = ros_drivers.build_plan(
        model="auto", ports=[str(port), str(other)],
        installed={"rplidar_ros", "hls_lfcd_lds_driver"},
        scores={"lds01@%s" % other: 2.0})
    assert plan, "план не должен быть пустым"
    assert plan[0].driver.name == "lds01", [i.driver.name for i in plan]
    assert plan[0].port == str(other), "результат проверки порта не учтён"
    # Драйверы без своего пакета остаются в плане, но помечены как неустановленные:
    # оператор должен видеть, чего не хватает, а не «лидар не найден».
    missing = [item for item in plan if not item.installed]
    assert missing, "неустановленные драйверы пропали из плана"
    assert all(item.driver.package not in {"rplidar_ros", "hls_lfcd_lds_driver"}
               for item in missing)
    not_installed_last = [item.driver.name for item in plan][-1]
    assert not_installed_last in {item.driver.name for item in missing}


def test_plan_can_be_limited_by_model(tmp_path):
    """Явно названный лидар: только его драйверы (и запасной для того же железа)."""
    port = tmp_path / "ttyUSB0"
    port.write_text("")
    plan = ros_drivers.build_plan(model="rplidar", ports=[str(port)],
                                  installed={"rplidar_ros"})
    assert plan[0].driver.name == "rplidar"
    assert {item.driver.name for item in plan} <= {"rplidar", "sllidar"}
    assert all(item.driver.package in {"rplidar_ros", "sllidar_ros2"} for item in plan)
    lds = ros_drivers.build_plan(model="turtlebot3", ports=[str(port)],
                                 installed={"hls_lfcd_lds_driver"})
    assert lds[0].driver.name == "lds01"


def test_reserved_ports_are_never_offered_to_the_lidar(tmp_path, monkeypatch):
    """Порт мастера руления и BMS лидару не отдаём — там другое устройство."""
    lidar_port = tmp_path / "ttyUSB0"
    lidar_port.write_text("")
    master_port = tmp_path / "ttyACM0"
    master_port.write_text("")
    monkeypatch.setenv("RC_SLAM_PORT", str(master_port))
    monkeypatch.setenv("RC_BMS_PORT", "")
    candidates = ros_drivers.candidate_ports([str(lidar_port), str(master_port)])
    assert candidates == [str(lidar_port)]


def test_driver_command_uses_direct_executable_and_ros_parameters(tmp_path):
    executable = tmp_path / "rplidar_composition"
    executable.write_text("#!/bin/sh\n")
    executable.chmod(0o755)
    argv = ros_drivers.driver_command(
        ros_drivers.DRIVERS["rplidar"], port="/dev/ttyUSB0", baud=256000,
        topic="/scan", frame_id="base_scan", executable=executable)
    assert argv[0] == str(executable), "исполняемый файл должен запускаться напрямую"
    assert "--ros-args" in argv
    assert "serial_port:=/dev/ttyUSB0" in argv
    assert "serial_baudrate:=256000" in argv
    # У LDS-01 параметры называются иначе — это то, на чём легко ошибиться.
    lds = ros_drivers.driver_command(ros_drivers.DRIVERS["lds01"], port="/dev/ttyUSB1")
    assert lds[:3] == ["ros2", "run", "hls_lfcd_lds_driver"]
    assert "port:=/dev/ttyUSB1" in lds


def test_opencr_driver_passes_the_board_port_and_parameters_file():
    driver = ros_drivers.DRIVERS["opencr"]
    argv = ros_drivers.driver_command(driver, port="/dev/ttyACM0")
    assert argv[:3] == ["ros2", "run", "turtlebot3_node"]
    assert "-i" in argv and "/dev/ttyACM0" in argv
    assert os.environ.get("TURTLEBOT3_MODEL", "burger") in \
        ros_drivers._substitute(driver.param_file, port="/dev/ttyACM0")


def test_find_executable_looks_into_the_built_workspace(tmp_path):
    install = tmp_path / "install"
    lib = install / "lib" / "rplidar_ros"
    lib.mkdir(parents=True)
    exe = lib / "rplidar_node"
    exe.write_text("#!/bin/sh\n")
    exe.chmod(0o755)
    assert ros_drivers.find_executable(ros_drivers.DRIVERS["rplidar"],
                                       install=install) == exe
    assert ros_drivers.find_executable(ros_drivers.DRIVERS["ld06"], install=install) is None


def test_install_lines_know_apt_and_source_builds():
    apt = ros_drivers.install_lines(ros_drivers.DRIVERS["rplidar"], "humble")
    assert apt == ["sudo apt install ros-humble-rplidar-ros"]
    source = ros_drivers.install_lines(ros_drivers.DRIVERS["ld06"], "humble")
    assert any("ldlidar_stl_ros2" in line and "colcon" in line for line in source)


# --------------------------------------------------------------------------- #
# Опознание лидара по кадрам (zapret_lidar/frames.py)
# --------------------------------------------------------------------------- #


def lds01_packet(index: int, distance_m: float = 1.5) -> bytes:
    """Кадр LDS-01: 0xFA, номер пакета, 6 измерений, контрольная сумма."""
    packet = bytearray(42)
    packet[0] = 0xFA
    packet[1] = 0xA0 + (index % 60)
    packet[2], packet[3] = 200, 0
    for point in range(6):
        offset = 4 + point * 6
        distance = int(distance_m * 1000) + point
        packet[offset], packet[offset + 1] = 200, 0
        packet[offset + 2] = distance & 0xFF
        packet[offset + 3] = (distance >> 8) & 0xFF
    checksum = ros_frames.lds01_checksum(bytes(packet[:40]))
    packet[40], packet[41] = checksum & 0xFF, (checksum >> 8) & 0xFF
    return bytes(packet)


def ld06_packet(start_deg: float) -> bytes:
    """Кадр LD06/LD19: синхрослово 0x54 0x2C и XOR-контрольная сумма."""
    packet = bytearray(47)
    packet[0], packet[1] = 0x54, 0x2C
    start = int(round(start_deg * 100.0)) & 0xFFFF
    packet[4], packet[5] = start & 0xFF, start >> 8
    for point in range(12):
        offset = 6 + point * 3
        packet[offset], packet[offset + 1], packet[offset + 2] = 0xE8, 0x03, 200
    crc = 0
    for byte in packet[:46]:
        crc ^= byte
    packet[46] = crc
    return bytes(packet)


def rplidar_stream() -> bytes:
    """Ответ RPLIDAR: дескриптор A5 5A и точки с ровно растущим углом."""
    data = bytearray(b"\xa5\x5a\x05\x00\x00\x40\x81")
    for index in range(60):
        angle_q6 = int(((index * 6) % 360) * 64)
        packet = bytearray(5)
        packet[0] = (50 << 2) | 0x02
        packet[1] = ((angle_q6 & 0x7F) << 1) | 0x01
        packet[2] = (angle_q6 >> 7) & 0xFF
        distance_q2 = int(2.5 * 1000 * 4)
        packet[3] = distance_q2 & 0xFF
        packet[4] = (distance_q2 >> 8) & 0xFF
        data += packet
    return bytes(data)


def test_frames_identify_each_lidar():
    lds01 = b"".join(lds01_packet(index) for index in range(6))
    assert ros_frames.score_lds01(lds01) >= ros_frames.SCORE_STRONG
    assert ros_frames.score_ld06(lds01) == 0.0
    assert ros_frames.best_protocol(lds01)[0] == "lds01"

    ld06 = b"".join(ld06_packet(start) for start in (0.0, 27.0, 54.0))
    assert ros_frames.score_ld06(ld06) >= ros_frames.SCORE_STRONG
    assert ros_frames.best_protocol(ld06)[0] == "ld06"

    rplidar = rplidar_stream()
    assert ros_frames.score_rplidar(rplidar) >= ros_frames.SCORE_STRONG
    assert ros_frames.best_protocol(rplidar)[0] == "rplidar"


def test_frames_do_not_take_garbage_for_a_lidar():
    """Чужие байты (или чужой протокол на этой скорости) — это не лидар."""
    garbage = bytes(range(256)) * 3
    protocol, score = ros_frames.best_protocol(garbage)
    assert score < ros_frames.SCORE_WEAK, (protocol, score)
    # LDS-01 разобранный как LD06 не должен давать «сильное» совпадение.
    lds01 = b"".join(lds01_packet(index) for index in range(6))
    assert ros_frames.score_ld06(lds01) < ros_frames.SCORE_STRONG


def test_probe_reads_real_bytes_from_a_pseudo_terminal():
    """Проверка порта на настоящем потоке байт (pty), а не на заглушке.

    Лидар сыплет кадры непрерывно — писатель в тесте делает то же самое, пока
    ``probe_port`` читает порт.
    """
    pytest.importorskip("serial")
    from zapret_lidar import probe as ros_probe

    master, slave = os.openpty()
    port = os.ttyname(slave)
    stop = threading.Event()

    def writer():
        index = 0
        while not stop.is_set():
            try:
                os.write(master, lds01_packet(index))
            except OSError:  # pragma: no cover — порт закрыли
                return
            index += 1
            time.sleep(0.01)

    thread = threading.Thread(target=writer, daemon=True)
    thread.start()
    try:
        report = ros_probe.probe_port(port, bauds={"lds01": (230400,)})
        assert report["protocol"] == "lds01", report
        assert report["scores"]["lds01"] >= 1.0
        assert ros_probe.scores_for_plan({port: report})["lds01@%s" % port] >= 1.0
    finally:
        stop.set()
        thread.join(timeout=2.0)
        os.close(master)
        os.close(slave)


# --------------------------------------------------------------------------- #
# CLI пакета (--print-plan / --list-drivers)
# --------------------------------------------------------------------------- #


def run_supervisor(*args: str) -> subprocess.CompletedProcess:
    """Запускает CLI пакета тем же интерпретатором, что и тесты (без ROS 2)."""
    return subprocess.run(
        [sys.executable, str(PKG / "zapret_lidar" / "supervisor.py"), *args],
        capture_output=True, text=True, timeout=60, cwd=str(ROOT),
    )


def test_print_plan_shows_order_and_missing_packages(tmp_path):
    port = tmp_path / "ttyUSB0"
    port.write_text("")
    result = run_supervisor("--print-plan", "--model", "auto", "--port", str(port),
                            "--installed", "hls_lfcd_lds_driver", "--prefix", "/nonexistent")
    assert result.returncode == 0, result.stderr
    assert "План запуска драйверов лидара" in result.stdout
    assert "lds01" in result.stdout
    assert "НЕТ пакета rplidar_ros" in result.stdout
    assert "поставить: sudo apt install ros-humble-rplidar-ros" in result.stdout


def test_print_plan_json_is_machine_readable(tmp_path):
    port = tmp_path / "ttyUSB0"
    port.write_text("")
    result = run_supervisor("--print-plan", "--json", "--model", "rplidar",
                            "--port", str(port), "--installed", "rplidar_ros")
    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload["plan"][0]["driver"] == "rplidar"
    assert payload["plan"][0]["installed"] is True
    assert payload["plan"][0]["port"] == str(port)


def test_list_drivers_mentions_every_supported_lidar():
    result = run_supervisor("--list-drivers")
    assert result.returncode == 0, result.stderr
    for title in ("LDS-01", "RPLIDAR", "LD06", "turtlebot3_node"):
        assert title in result.stdout, title


def test_cli_without_ros_explains_how_to_install_it():
    """Запуск супервизора без ROS 2 не должен падать трейсбеком."""
    result = run_supervisor("--model", "auto", "--port", "")
    if "rclpy" in result.stdout + result.stderr:
        assert result.returncode == 2
        assert "./scripts/ros2_setup.sh" in result.stderr
    else:  # pragma: no cover — на машине с ROS 2 супервизор действительно запустится
        assert result.returncode in (0, 2)


# --------------------------------------------------------------------------- #
# Упаковка пакета ROS 2
# --------------------------------------------------------------------------- #


def test_package_xml_declares_an_ament_python_package():
    root = ET.parse(PKG / "package.xml").getroot()
    assert root.findtext("name") == "zapret_lidar"
    assert root.findtext("export/build_type") == "ament_python"
    dependencies = {item.text for item in root.findall("exec_depend")}
    assert {"rclpy", "sensor_msgs", "std_msgs"} <= dependencies


def test_setup_py_entry_points_and_data_files_exist():
    text = (PKG / "setup.py").read_text(encoding="utf-8")
    assert "supervisor_node = zapret_lidar.supervisor:main" in text
    assert (PKG / "resource" / "zapret_lidar").is_file()
    for relative in ("launch/lidar.launch.py", "config/lidar.yaml", "package.xml"):
        assert (PKG / relative).is_file(), relative
    # Модули, на которые ссылаются точки входа и launch, на месте.
    for module in ("supervisor.py", "node.py", "drivers.py", "frames.py", "probe.py"):
        assert (PKG / "zapret_lidar" / module).is_file(), module


def test_launch_file_declares_the_parameters_the_supervisor_uses():
    text = (PKG / "launch" / "lidar.launch.py").read_text(encoding="utf-8")
    for name in ("model", "port", "baud", "scan_topic", "status_topic", "autostart"):
        assert 'DeclareLaunchArgument("%s"' % name in text, name
    assert "supervisor_node" in text
    # Вендорский драйвер и наш супервизор не должны запускаться одновременно.
    assert "UnlessCondition" in text and "driver_launch" in text


def test_config_yaml_has_defaults_matching_the_module():
    text = (PKG / "config" / "lidar.yaml").read_text(encoding="utf-8")
    assert "scan_topic: /scan" in text
    assert "status_topic: /zapret/lidar/status" in text
    assert "model: auto" in text
    assert ros_drivers.DEFAULT_SCAN_TOPIC in text
    assert ros_drivers.DEFAULT_STATUS_TOPIC in text


# --------------------------------------------------------------------------- #
# Скрипты установки и запуска
# --------------------------------------------------------------------------- #


def fake_ros_prefix(tmp_path: Path, distro: str = "humble",
                    packages: tuple[str, ...] = ("rplidar_ros",)) -> Path:
    """Мини-ROS 2: setup.bash, ament-индекс и каталог python-пакетов."""
    prefix = tmp_path / distro
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    version = "%d.%d" % sys.version_info[:2]
    (prefix / "lib" / ("python%s" % version) / "site-packages").mkdir(parents=True)
    (prefix / "setup.bash").write_text("# ROS 2\n", encoding="utf-8")
    for package in ("rclpy", "sensor_msgs", *packages):
        (index / package).write_text("", encoding="utf-8")
    return prefix


def test_ros2_setup_check_reports_what_is_installed(tmp_path):
    prefix = fake_ros_prefix(tmp_path)
    env = dict(os.environ, RC_ROS_PREFIX=str(prefix), ROS_DISTRO="")
    result = subprocess.run(["bash", str(SCRIPTS / "ros2_setup.sh"), "--check", "--json"],
                            capture_output=True, text=True, timeout=120, cwd=str(ROOT), env=env)
    payload = json.loads(result.stdout)
    assert payload["prefix"] == str(prefix)
    assert payload["distro"] == "humble"
    assert "rplidar_ros" in payload["packages"]
    # В фальшивом префиксе нет настоящего rclpy: проверка обязана быть честной.
    assert payload["rclpy"] is False
    assert result.returncode == 1


def test_ros2_setup_check_without_ros_says_how_to_install(tmp_path):
    env = dict(os.environ, RC_ROS_PREFIX=str(tmp_path / "нет"), ROS_DISTRO="")
    result = subprocess.run(["bash", str(SCRIPTS / "ros2_setup.sh"), "--check"],
                            capture_output=True, text=True, timeout=120, cwd=str(ROOT), env=env)
    assert result.returncode == 1
    assert "ROS 2" in result.stdout
    assert "не найден" in result.stdout


def test_ros2_setup_help_documents_the_flags():
    result = subprocess.run(["bash", str(SCRIPTS / "ros2_setup.sh"), "--help"],
                            capture_output=True, text=True, timeout=60, cwd=str(ROOT))
    assert result.returncode == 0
    for flag in ("--install", "--drivers", "--service", "--check", "--no-env"):
        assert flag in result.stdout, flag


def test_ros2_lidar_print_shows_the_exact_command(tmp_path):
    """--print печатает команду запуска и ничего не запускает (проверка на стенде)."""
    prefix = fake_ros_prefix(tmp_path)
    install = tmp_path / "install"
    (install / "zapret_lidar" / "lib").mkdir(parents=True)
    (install / "setup.bash").write_text("# ws\n", encoding="utf-8")
    env = dict(os.environ, RC_ROS_PREFIX=str(prefix),
               RC_ROS_WS_INSTALL=str(install),
               RC_LIDAR_PORT="/dev/ttyUSB0", RC_LIDAR_PROTOCOL="lds01")
    result = subprocess.run(
        ["bash", str(SCRIPTS / "ros2_lidar.sh"), "--print"],
        capture_output=True, text=True, timeout=120, cwd=str(ROOT), env=env)
    assert result.returncode == 0, result.stderr
    assert "model:=lds01" in result.stdout
    assert "port:=/dev/ttyUSB0" in result.stdout
    assert "supervisor" in result.stdout
    assert str(prefix) in result.stdout


def test_ros2_lidar_help_lists_modes():
    result = subprocess.run(["bash", str(SCRIPTS / "ros2_lidar.sh"), "--help"],
                            capture_output=True, text=True, timeout=60, cwd=str(ROOT))
    assert result.returncode == 0
    for flag in ("--status", "--print", "--model", "--port"):
        assert flag in result.stdout, flag


def test_lidar_service_unit_starts_the_ros_supervisor():
    text = (SCRIPTS / "robot-lidar.service").read_text(encoding="utf-8")
    assert "ExecStart=/opt/robot-control/scripts/ros2_lidar.sh" in text
    assert "SupplementaryGroups=dialout" in text
    # Драйвер лидара должен подняться раньше приложений: иначе они не увидят скан.
    assert "Before=robot-control.service" in text
    assert "EnvironmentFile=-/opt/robot-control/.env" in text


def test_installer_and_autostart_scripts_know_about_ros2():
    installer = (SCRIPTS / "install_ubuntu.sh").read_text(encoding="utf-8")
    assert "ros2_setup.sh" in installer
    assert "--ros2" in installer and "--no-ros2" in installer
    autostart = (SCRIPTS / "autostart.sh").read_text(encoding="utf-8")
    assert "robot-lidar" in autostart
    stop = (SCRIPTS / "stop.sh").read_text(encoding="utf-8")
    assert 'stop_service robot-lidar' in stop


def test_env_example_documents_the_source_and_topic():
    text = (ROOT / ".env.example").read_text(encoding="utf-8")
    assert "RC_LIDAR_SOURCE=auto" in text
    assert "RC_LIDAR_ROS_TOPIC=/scan" in text
    assert "ros2_setup.sh" in text and "ros2_lidar.sh" in text


def test_workspace_readme_lists_every_driver_package():
    text = (WS / "README.md").read_text(encoding="utf-8")
    for driver in ros_drivers.DRIVERS.values():
        assert driver.package in text, driver.package
    assert "--print-plan" in text


@pytest.mark.skipif(shutil.which("bash") is None, reason="нужен bash")
def test_scripts_pass_bash_syntax_check():
    for name in ("ros2_setup.sh", "ros2_lidar.sh", "autostart.sh", "stop.sh",
                 "install_ubuntu.sh"):
        result = subprocess.run(["bash", "-n", str(SCRIPTS / name)],
                                capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, "%s: %s" % (name, result.stderr)
