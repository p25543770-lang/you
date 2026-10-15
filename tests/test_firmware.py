"""Прошивки руля: совпадение с проверенным скетчем, общий блок и симуляция.

* Пины, уровни и скорости берутся из скетча, проверенного на железе
  (firmware/reference/steering_tested) — расхождение ловится здесь, а не на роботе.
* Блок STEERING CORE одинаков в мастере и слейве: правка в одном месте
  без второго — ошибка.
* tests/firmware_sim — хост-симуляция всех четырёх плат (TB6600, оптодатчик,
  I2C): калибровка из разных положений, ошибки датчика, команды монитора,
  тяга и watchdog. Нужен g++; без него тест пропускается.
* Сборка настоящим avr-gcc — если есть arduino-cli или AVR_ROOT/CORE_ROOT
  (см. firmware/tools/avr_build.py).
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
MASTER = ROOT / "firmware" / "motor_master" / "motor_master.ino"
NODE = ROOT / "firmware" / "motor_node" / "motor_node.ino"
REFERENCE = ROOT / "firmware" / "reference" / "steering_tested" / "steering_tested.ino"
SIM_DIR = ROOT / "tests" / "firmware_sim"
CORE_BEGIN = "// =========================== STEERING CORE BEGIN ==========================="
CORE_END = "// ============================ STEERING CORE END ============================"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _core(path: Path) -> str:
    text = _text(path)
    assert text.count(CORE_BEGIN) == 1 and text.count(CORE_END) == 1, path
    return text[text.index(CORE_BEGIN):text.index(CORE_END) + len(CORE_END)]


def _const(text: str, name: str) -> str:
    match = re.search(r"\bconst\s+[\w ]+?\b%s\s*=\s*([^;]+);" % re.escape(name), text)
    assert match, name
    return match.group(1).strip().rstrip("f").rstrip("UL").rstrip("L")


def test_steering_core_is_identical_in_master_and_node():
    assert _core(MASTER) == _core(NODE)


def test_sketches_compile_for_avr_headers_only():
    """В avr-libc нет <strings.h>: прежняя прошивка из-за него не собиралась."""
    for path in (MASTER, NODE):
        text = _text(path)
        assert "#include <strings.h>" not in text
        assert "String " not in text          # без кучи: только char-буферы
        assert "Serial.print" not in text     # печать только через неблокирующий буфер


def test_pins_levels_and_speeds_match_the_tested_sketch():
    reference = _text(REFERENCE)
    core = _core(MASTER)
    assert _const(reference, "PUL") == _const(core, "STEP_PUL_PIN") == "6"
    assert _const(reference, "DIR") == _const(core, "STEP_DIR_PIN") == "7"
    assert _const(reference, "ENA") == _const(core, "STEP_ENA_PIN") == "8"
    assert _const(reference, "OPTO_PIN") == _const(core, "OPTO_PIN") == "18"
    assert _const(reference, "STEP_PERIOD_US") == _const(core, "STEP_PERIOD_US") == "100"
    assert _const(reference, "CALIB_STEP_PERIOD_US") == _const(core, "CALIB_STEP_PERIOD_US") == "2000"
    assert float(_const(reference, "STEPS_PER_DEGREE")) == float(_const(core, "STEPS_PER_DEGREE"))
    assert float(_const(reference, "CALIB_SEARCH_DEGREES")) == float(_const(core, "CALIB_SEARCH_DEGREES"))
    assert float(_const(reference, "CALIB_SLOW_SEARCH_DEGREES")) == float(
        _const(core, "CALIB_SLOW_SEARCH_DEGREES"))
    assert _const(reference, "OPTO_ACTIVE_LOW") == _const(core, "OPTO_ACTIVE_LOW") == "true"
    # DIR LOW — шаги растут («f»), ENA = HIGH с момента включения, D18 с подтяжкой.
    assert "digitalWrite(DIR, LOW)" in reference and _const(core, "STEP_DIR_PLUS_LEVEL") == "LOW"
    assert "digitalWrite(ENA, HIGH)" in reference and _const(core, "STEP_ENABLE_LEVEL") == "HIGH"
    assert "pinMode(OPTO_PIN, INPUT_PULLUP)" in reference and "pinMode(OPTO_PIN, INPUT_PULLUP)" in core
    assert "Wire.setClock(400000)" in reference and _const(_text(MASTER), "I2C_CLOCK_HZ") == "400000"


def test_master_accepts_monitor_commands_of_the_tested_sketch():
    master = _text(MASTER)
    core = _core(MASTER)
    for command in ("k", "s", "g", "h", "z", "d", "c"):
        assert 'PSTR("%s")' % command in master
    for message in ("Калибровка уже запущена!", "Цель достигнута. Угол:", "ПАУЗА. Угол:",
                    "Шаблон: d [градусы] [f/b]", "Ошибка направления! f или b", "ОБНУЛЕНА.",
                    "ЦИКЛ качания на", "ЦИКЛ ВЫКЛ. Угол:", "Цикл и так выключен.", "Уже в цели."):
        assert message in core, message
    # Тяга — отдельная команда «t»: «d» осталась поворотом, как в проверенном скетче.
    assert 'PSTR("t %d")' in master
    assert '"d %d"' not in master


def _compiler() -> str | None:
    return shutil.which(os.environ.get("CXX", "g++")) or shutil.which("clang++")


@pytest.fixture(scope="module")
def simulator(tmp_path_factory):
    compiler = _compiler()
    if not compiler:
        pytest.skip("нет g++/clang++ — симуляция прошивок пропущена")
    binary = tmp_path_factory.mktemp("firmware-sim") / "sim"
    result = subprocess.run(
        [compiler, "-std=c++17", "-O1", "-w", "-I", str(SIM_DIR / "stubs"),
         str(SIM_DIR / "sim.cpp"), "-o", str(binary)],
        capture_output=True, text=True, timeout=300,
    )
    assert result.returncode == 0, result.stderr[-4000:]
    return binary


def test_firmware_simulation_scenarios(simulator):
    result = subprocess.run([str(simulator)], capture_output=True, text=True, timeout=600)
    report = result.stdout + "\n" + result.stderr[-6000:]
    assert result.returncode == 0, report
    assert "22/22 сценариев прошли" in result.stdout, report


def test_server_parses_simulated_master_telemetry(simulator):
    """Строки @TLM настоящей прошивки (из симуляции) понимает сервер."""
    import importlib.util

    spec = importlib.util.spec_from_file_location("slam_backend_fw", ROOT / "slam_gui" / "backend.py")
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)

    result = subprocess.run([str(simulator), "tlm_dump"], capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr[-4000:]
    frames = {}
    messages = []
    for line in result.stdout.splitlines():
        frame = backend.parse_master_line(line)
        if frame:
            frames[frame["mod"]] = frame
        elif line.startswith("# "):
            messages.append(line)
    assert set(frames) == {"FL", "FR", "RL", "RR"}
    for module, frame in frames.items():
        assert frame["cal"] is True and frame["home"] == "opto", (module, frame)
        assert frame["firmware"] == 2 and frame["calibError"] == 0
        assert frame["drivePercent"] == 0 and frame["rpm"] == 0.0
    assert frames["FR"]["deg"] == 12.5 and frames["FR"]["moving"] is False
    assert any("FR: Калибровка ЗАВЕРШЕНА!" in line for line in messages)

    import queue
    import threading

    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    source.frames = {}
    source.drive_percent = 0
    source.drive_last_command = 0.0
    source.commands = queue.Queue(maxsize=4)
    source.serial_ready = threading.Event()
    source.serial_ready.set()
    for line in result.stdout.splitlines():
        source.feed(line, now=100.0)
    status = source.drive_status(now=100.2)
    assert status["reason"] == "steering_misaligned"   # FR стоит на 12.5°
    motors = {motor["id"]: motor for motor in source.read(now=100.2)["motors"]}
    assert all(motor["homed"] and motor["online"] for motor in motors.values())
    assert motors["FR"]["homeSource"] == "opto"


def _avr_toolchain_available() -> bool:
    if shutil.which("arduino-cli"):
        return True
    return bool(os.environ.get("AVR_ROOT")) and bool(os.environ.get("CORE_ROOT"))


@pytest.mark.skipif(not _avr_toolchain_available(),
                    reason="нет arduino-cli и AVR_ROOT/CORE_ROOT — сборка avr-gcc пропущена")
@pytest.mark.parametrize("sketch", [MASTER, NODE])
def test_sketch_builds_with_avr_gcc(sketch, tmp_path):
    if shutil.which("arduino-cli") and not os.environ.get("AVR_ROOT"):
        command = ["arduino-cli", "compile", "--fqbn", "arduino:avr:mega", "--warnings", "all",
                   str(sketch.parent)]
    else:
        command = ["python3", str(ROOT / "firmware" / "tools" / "avr_build.py"), str(sketch)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    assert "warning" not in (result.stdout + result.stderr).lower()
