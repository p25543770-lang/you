"""Гейт команд движения и мост Twist → шасси (без железа: фальшивый источник)."""

from __future__ import annotations

import math

import pytest

from robot_control.motion import (
    ChassisPlan,
    Geometry,
    GateInput,
    GateLimits,
    MotionExecutor,
    MotionGate,
    PlanError,
    Twist,
    corridor_clearance,
    plan_chassis,
    steer_change_needs_stop,
)

NOW = 100.0


def fresh(**kw) -> GateInput:
    base = dict(now=NOW, scan_time=NOW, scan_points=[{"a": 0.0, "r": 5.0}],
                telemetry_time=NOW, soc_pct=80.0)
    base.update(kw)
    return GateInput(**base)


# ------------------------------- гейт ----------------------------------- #
def test_gate_passes_fresh_small_command():
    gate = MotionGate()
    result = gate.check(Twist(0.1, 0.0), fresh())
    assert result.reason == "ok"
    assert result.twist.vx == pytest.approx(0.1)


@pytest.mark.parametrize("state, reason", [
    (dict(estop=True), "estop"),
    (dict(fault=True), "fault"),
    (dict(telemetry_time=None), "no_telemetry"),
    (dict(telemetry_time=NOW - 1.0), "no_telemetry"),
    (dict(scan_time=None), "no_scan"),
    (dict(scan_time=NOW - 0.5), "no_scan"),
    (dict(soc_pct=None), "soc_unknown"),
    (dict(soc_pct=10.0), "low_soc"),
])
def test_gate_refuses_with_reason(state, reason):
    result = MotionGate().check(Twist(0.1, 0.0), fresh(**state))
    assert result.twist.is_zero()
    assert result.reason == reason


def test_gate_refuses_nan_and_reverse():
    gate = MotionGate()
    assert gate.check(Twist(float("nan"), 0.0), fresh()).reason == "invalid_cmd"
    assert gate.check(Twist(-0.1, 0.0), fresh()).reason == "reverse_not_supported"


def test_gate_clamps_speed_and_yaw_to_limits():
    lim = GateLimits()
    result = MotionGate(lim).check(Twist(5.0, 9.0), fresh(scan_points=[]))
    assert result.twist.vx <= lim.max_vx + 1e-9
    assert result.twist.wz <= lim.max_wz + 1e-9
    assert result.reason in ("limited", "accel_limited")


def test_gate_stops_when_obstacle_inside_stop_distance():
    result = MotionGate().check(Twist(0.2, 0.0), fresh(scan_points=[{"a": 0.0, "r": 0.5}]))
    assert result.twist.is_zero()
    assert result.reason == "obstacle_stop"


def test_gate_slows_linearly_between_stop_and_slow_distance():
    lim = GateLimits()
    mid = (lim.stop_clearance_m + lim.slow_clearance_m) / 2.0
    result = MotionGate(lim).check(Twist(0.2, 0.0), fresh(scan_points=[{"a": 0.0, "r": mid}]))
    assert result.reason == "obstacle_slow"
    assert 0.0 < result.twist.vx < 0.2


def test_gate_ignores_obstacle_outside_corridor():
    # Препятствие далеко сбоку (90°, 1 м): вне полосы движения 0.45 м.
    result = MotionGate().check(Twist(0.2, 0.0), fresh(scan_points=[{"a": math.pi / 2, "r": 1.0}]))
    assert result.reason not in ("obstacle_stop", "obstacle_slow")
    assert result.twist.vx > 0


def test_gate_acceleration_is_ramped():
    lim = GateLimits(max_accel=0.5)
    gate = MotionGate(lim)
    gate.check(Twist(0.0, 0.0), fresh(now=NOW))            # ноль, фиксируем время
    result = gate.check(Twist(0.3, 0.0), fresh(now=NOW + 0.1))
    assert result.twist.vx <= 0.5 * 0.1 + 1e-9             # не быстрее 0.5 м/с²


def test_gate_timeout_stops_stale_command():
    gate = MotionGate()
    gate.check(Twist(0.1, 0.0), fresh(now=NOW))
    assert gate.timeout(NOW + 0.3) is None
    stop = gate.timeout(NOW + 0.7)
    assert stop is not None and stop.reason == "cmd_timeout" and stop.twist.is_zero()


def test_gate_default_timeout_matches_firmware_watchdog():
    assert GateLimits().cmd_timeout_s == pytest.approx(0.65)


# --------------------------- препятствия ------------------------------- #
def test_corridor_clearance_takes_nearest_point_ahead():
    lim = GateLimits()
    pts = [{"a": 0.0, "r": 3.0}, {"a": 0.1, "r": 1.5}, {"a": math.pi, "r": 0.2}]
    assert corridor_clearance(pts, 1.0, lim) == pytest.approx(1.5 * math.cos(0.1), rel=1e-6)


def test_corridor_clearance_ignores_bad_points_and_empty_scan():
    lim = GateLimits()
    assert corridor_clearance([], 1.0, lim) is None
    assert corridor_clearance([{"a": "x", "r": 1}, {"r": 1}, {"a": 0, "r": -1}], 1.0, lim) is None


def test_corridor_clearance_uses_lidar_offset():
    lim = GateLimits()
    assert corridor_clearance([{"a": 0.0, "r": 1.0}], 1.0, lim, lidar_x=0.3) == pytest.approx(1.3)


# ----------------------------- кинематика ------------------------------- #
def test_straight_motion_has_zero_steer_and_scaled_drive():
    geo = Geometry(full_scale_mps=0.7)
    plan = plan_chassis(Twist(0.35, 0.0), geo)
    assert all(angle == 0.0 for angle in plan.steer_deg.values())
    assert plan.drive_percent == pytest.approx(10.0, abs=0.01)


def test_drive_percent_never_exceeds_limit():
    plan = plan_chassis(Twist(0.3, 0.0), Geometry(full_scale_mps=0.1))
    assert plan.drive_percent == 20.0


def test_gentle_yaw_gives_front_and_rear_steer_within_limit():
    plan = plan_chassis(Twist(0.2, 0.1), Geometry(wheelbase=0.6, track=0.5))
    assert plan.steer_deg["FL"] > 0 and plan.steer_deg["RL"] < 0
    assert all(abs(a) <= 85.0 for a in plan.steer_deg.values())


def test_too_sharp_turn_is_refused_not_clipped():
    # Передний модуль уходит за 85°: команда отказывается целиком, а не обрезается.
    with pytest.raises(PlanError):
        plan_chassis(Twist(0.2, 0.9), Geometry(wheelbase=0.6))


def test_rotation_in_place_and_reverse_are_refused():
    with pytest.raises(PlanError):
        plan_chassis(Twist(0.0, 0.3))
    with pytest.raises(PlanError):
        plan_chassis(Twist(-0.1, 0.0))


def test_steer_change_detection_needs_stop_while_driving():
    assert steer_change_needs_stop({"FL": 0.0}, {"FL": 5.0})
    assert not steer_change_needs_stop({"FL": 0.0}, {"FL": 0.2})


# ------------------------------ исполнитель ----------------------------- #
class FakeSource:
    def __init__(self, drive_ok=True, steer_ok=True):
        self.calls = []
        self.drive_ok = drive_ok
        self.steer_ok = steer_ok

    def drive(self, percent):
        self.calls.append(("drive", round(percent, 2)))
        return {"ok": self.drive_ok, "percent": percent}

    def steer_module(self, module, angle):
        self.calls.append(("steer", module, round(angle, 2)))
        return {"ok": self.steer_ok}


def test_executor_sends_steer_then_drive_for_straight_motion():
    src = FakeSource()
    ex = MotionExecutor(src, Geometry(full_scale_mps=0.7))
    out = ex.command(Twist(0.2, 0.0), fresh())
    assert out["ok"] is True
    # Первая команда после простоя — мягкий старт: 0.1 м/с → 0.1/0.7·20 % ≈ 2.86 %.
    assert len(src.calls) == 1 and src.calls[0][0] == "drive"
    assert src.calls[0][1] == pytest.approx(2.86, abs=0.01)


def test_executor_stops_drive_on_refusal_and_on_gate_zero():
    src = FakeSource()
    ex = MotionExecutor(src)
    ex.command(Twist(0.2, 0.0), fresh())
    src.calls.clear()
    out = ex.command(Twist(0.2, 0.0), fresh(estop=True))
    assert out["reason"] == "estop"
    assert src.calls == [("drive", 0)]
    assert ex.current_percent == 0.0


def test_executor_refuses_steer_change_while_moving():
    src = FakeSource()
    ex = MotionExecutor(src, Geometry(wheelbase=0.6))
    ex.command(Twist(0.2, 0.0), fresh())            # едем прямо
    src.calls.clear()
    out = ex.command(Twist(0.2, 0.1), fresh(now=NOW + 0.05))  # нужен поворот руля
    assert out["reason"] == "steer_wait_for_stop"
    # Руль не поворачивался, пока тяга не снята.
    assert src.calls == [("drive", 0)]


def test_executor_reports_failed_steer_and_stops():
    src = FakeSource(steer_ok=False)
    ex = MotionExecutor(src, Geometry(wheelbase=0.6, track=0.5))
    out = ex.command(Twist(0.2, 0.05), fresh())
    assert out["ok"] is False and out["reason"] == "steer_failed"


def test_executor_reports_drive_refusal_from_source():
    src = FakeSource(drive_ok=False)
    out = MotionExecutor(src).command(Twist(0.2, 0.0), fresh())
    assert out["ok"] is False
    assert out["drive"]["ok"] is False


def test_geometry_defaults_are_marked_unmeasured():
    # Защита от ложной уверенности: значения по умолчанию должны быть явно помечены в исходнике.
    import inspect
    import robot_control.motion as motion
    assert "НЕ ИЗМЕРЕНЫ" in inspect.getsource(motion.Geometry)
