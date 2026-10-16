"""Предел поворота руля: ±85° для каждого модуля, одинаково в Python, прошивках и интерфейсе.

Предел проверяется программно: если кто-то поменяет значение только в одном месте,
тест падает. Левый и правый поворот — симметричные границы.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
LIMIT = 85.0
MASTER = ROOT / "firmware" / "motor_master" / "motor_master.ino"
NODE = ROOT / "firmware" / "motor_node" / "motor_node.ino"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_limit_is_the_same_in_python_layers():
    from robot_control import motion, slam

    assert slam.STEERING_ANGLE_LIMIT_DEG == LIMIT
    assert motion.STEER_LIMIT_DEG == LIMIT


def test_backend_limit_is_the_same():
    text = _read(ROOT / "slam_gui" / "backend.py")
    assert re.search(r"^\s*MAX_STEER_DEG = 85\.0\b", text, re.M)


@pytest.mark.parametrize("path", [MASTER, NODE], ids=["master", "node"])
def test_firmware_limit_constant_and_messages(path):
    text = _read(path)
    assert re.search(r"const float STEER_LIMIT_DEG = 85\.0f;", text), path.name
    assert "вне ±85°" in text, path.name
    assert "вне ±32°" not in text and "предел ±32°" not in text, path.name


def test_firmware_master_and_node_agree():
    master = re.search(r"STEER_LIMIT_DEG = ([0-9.]+)f", _read(MASTER)).group(1)
    node = re.search(r"STEER_LIMIT_DEG = ([0-9.]+)f", _read(NODE)).group(1)
    assert master == node == "85.0"


def test_interface_limits_are_symmetric_85():
    html = _read(ROOT / "slam_gui" / "robot.html")
    js = _read(ROOT / "slam_gui" / "robot.js")
    steering = _read(ROOT / "slam_gui" / "steering.js")
    assert html.count('min="-85" max="85"') == 4
    assert "−85°" in html and "+85°" in html
    assert 'min="-32"' not in html
    assert "Math.max(-85, Math.min(85," in js and "Math.max(-32" not in js
    assert "Math.max(-85, Math.min(85," in steering and "Math.max(-32" not in steering


def test_motion_plan_accepts_inside_and_refuses_beyond_85():
    from robot_control.motion import Geometry, PlanError, Twist, plan_chassis

    # Умеренный поворот: все углы внутри ±85°.
    plan = plan_chassis(Twist(0.2, 0.2), Geometry(wheelbase=0.6, track=0.5))
    assert all(abs(a) <= LIMIT for a in plan.steer_deg.values())
    # Резкий поворот: угол переднего модуля за 85° — отказ, а не обрезка.
    with pytest.raises(PlanError):
        plan_chassis(Twist(0.2, 0.9), Geometry(wheelbase=0.6, track=0.5))
