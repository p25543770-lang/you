"""Гейт команд движения и мост Twist → шасси 4WIS/4WID (без ROS и без HTTP).

Модуль не открывает новых путей к моторам. Все команды проходят:

1. :class:`MotionGate` — свежесть скана и телеметрии, таймаут команд, зазор до
   препятствия по коридору движения, низкий заряд, аварийный флаг, ограничение
   скорости и ускорения. Любой отказ даёт нулевой Twist с причиной.
2. :func:`plan_chassis` — кинематика 4WIS: углы модулей и тяга в процентах.
   Углы вне ±32° и задний ход не планируются (отказ, а не «обрезка»).
3. :class:`MotionExecutor` — отправка через методы источника (``steer_module``,
   ``drive``). Источник сам проверяет выравнивание, нулевую отметку, телеметрию
   и watchdog. Руль не поворачивается при ненулевой тяге: сначала стоп.

Геометрия шасси (база, колея, положение лидара) — **не измерена**. Значения по
умолчанию помечены в :class:`Geometry` и должны быть заменены измерениями на роботе.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

#: Предел руления модуля, как в прошивке и в backend.py.
STEER_LIMIT_DEG = 32.0
#: Предел тяги, как в прошивке (DRIVE_MAX_PERCENT) и в backend.py.
DRIVE_LIMIT_PERCENT = 20.0
#: Таймаут команды движения; совпадает с watchdog прошивки (650 мс).
CMD_TIMEOUT_S = 0.65
#: Мягкий старт после простоя: сколько секунд «разгона» учитывать на первой команде.
SOFT_START_S = 0.2


@dataclass(frozen=True)
class Twist:
    """Команда движения в системе шасси: vx — вперёд (м/с), wz — рыскание (рад/с)."""

    vx: float = 0.0
    wz: float = 0.0

    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.wz == 0.0


ZERO = Twist(0.0, 0.0)


@dataclass(frozen=True)
class Geometry:
    """Размеры шасси в метрах. ЗНАЧЕНИЯ ПО УМОЛЧАНИЮ НЕ ИЗМЕРЕНЫ.

    wheelbase — расстояние между осями; track — колея; lidar_x — смещение лидара
    вперёд от центра шасси; full_scale_mps — скорость, которой соответствует 20 %.
    """

    wheelbase: float = 0.60   # TODO измерить на роботе
    track: float = 0.50       # TODO измерить на роботе
    lidar_x: float = 0.0      # TODO измерить на роботе
    full_scale_mps: float = 0.7  # TODO измерить на роботе (SimSource использует 0.7)

    def module_positions(self) -> dict[str, tuple[float, float]]:
        half_l = self.wheelbase / 2.0
        half_w = self.track / 2.0
        return {
            "FL": (half_l, half_w),
            "FR": (half_l, -half_w),
            "RL": (-half_l, half_w),
            "RR": (-half_l, -half_w),
        }


@dataclass(frozen=True)
class GateLimits:
    """Пороги гейта. Консервативные значения; подлежат подтверждению на полигоне."""

    max_vx: float = 0.3            # м/с
    max_wz: float = 0.5            # рад/с
    max_accel: float = 0.5         # м/с² — ограничение нарастания vx
    max_decel: float = 1.0         # м/с² — торможение можно быстрее
    stop_clearance_m: float = 0.6  # зазор ниже — немедленный стоп
    slow_clearance_m: float = 1.2  # зазор ниже — скорость ограничивается линейно
    corridor_half_width_m: float = 0.45  # полуширина коридора движения
    corridor_length_m: float = 2.0       # длина коридора перед шасси
    scan_max_age_s: float = 0.2
    telemetry_max_age_s: float = 0.2
    cmd_timeout_s: float = CMD_TIMEOUT_S
    min_soc_pct: float = 18.0


@dataclass(frozen=True)
class GateInput:
    """Снимок состояния на момент команды. None означает «неизвестно» → отказ."""

    now: float
    scan_time: Optional[float] = None
    scan_points: Optional[list] = None      # [{"a": рад, "r": м}, ...] в системе лидара
    telemetry_time: Optional[float] = None
    soc_pct: Optional[float] = None
    fault: bool = False
    estop: bool = False


@dataclass(frozen=True)
class GateResult:
    twist: Twist
    reason: str  # "ok" или причина отказа/ограничения


def corridor_clearance(points: Iterable[dict], forward_sign: float, limits: GateLimits,
                       lidar_x: float = 0.0) -> Optional[float]:
    """Минимальное расстояние до препятствия в коридоре перед шасси, м.

    Точки лидара переводятся в систему шасси (смещение lidar_x). Учитываются
    только точки впереди по направлению движения (forward_sign +1/-1) внутри
    полосы шириной 2·corridor_half_width и длиной corridor_length. None — если
    точек нет или все вне коридора: по скану нельзя доказать, что путь свободен,
    поэтому гейт всё равно решит по остальным проверкам (см. MotionGate.check).
    """
    best: Optional[float] = None
    for point in points or []:
        try:
            angle = float(point["a"])
            dist = float(point["r"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(angle) or not math.isfinite(dist) or dist <= 0:
            continue
        x = dist * math.cos(angle) + lidar_x
        y = dist * math.sin(angle)
        x *= forward_sign
        if x <= 0 or x > limits.corridor_length_m:
            continue
        if abs(y) > limits.corridor_half_width_m:
            continue
        best = x if best is None else min(best, x)
    return best


class MotionGate:
    """Фильтр команд движения с памятью (ускорение, таймаут)."""

    def __init__(self, limits: GateLimits = GateLimits()):
        self.limits = limits
        self._last = ZERO
        self._last_cmd_time: Optional[float] = None

    @property
    def last(self) -> Twist:
        return self._last

    def reset(self) -> None:
        self._last = ZERO
        self._last_cmd_time = None

    def _zero(self, reason: str, now: float) -> GateResult:
        self._last = ZERO
        self._last_cmd_time = now
        return GateResult(ZERO, reason)

    def check(self, requested: Twist, state: GateInput) -> GateResult:
        """Пропустить команду, ограничить или заменить нулём с причиной."""
        lim = self.limits
        now = state.now

        if state.estop:
            return self._zero("estop", now)
        if state.fault:
            return self._zero("fault", now)
        if not (math.isfinite(requested.vx) and math.isfinite(requested.wz)):
            return self._zero("invalid_cmd", now)
        if state.telemetry_time is None or now - state.telemetry_time > lim.telemetry_max_age_s:
            return self._zero("no_telemetry", now)
        if state.scan_time is None or now - state.scan_time > lim.scan_max_age_s:
            return self._zero("no_scan", now)
        if state.soc_pct is None:
            return self._zero("soc_unknown", now)
        if state.soc_pct < lim.min_soc_pct:
            return self._zero("low_soc", now)

        if requested.is_zero():
            self._last = ZERO
            self._last_cmd_time = now
            return GateResult(ZERO, "ok")

        # Ограничение по скорости и рысканию.
        vx = max(-lim.max_vx, min(lim.max_vx, requested.vx))
        wz = max(-lim.max_wz, min(lim.max_wz, requested.wz))
        reason = "ok"
        if vx != requested.vx or wz != requested.wz:
            reason = "limited"

        # Задний ход по этому гейту не разрешён: проверки зазора сзади нет.
        if vx < 0:
            return self._zero("reverse_not_supported", now)

        # Зазор по коридору перед шасси.
        clearance = corridor_clearance(state.scan_points or [], 1.0, lim, state.lidar_x_hint()
                                       if hasattr(state, "lidar_x_hint") else 0.0)
        if clearance is not None and clearance <= lim.stop_clearance_m:
            return self._zero("obstacle_stop", now)
        if clearance is not None and clearance < lim.slow_clearance_m:
            span = lim.slow_clearance_m - lim.stop_clearance_m
            factor = max(0.0, (clearance - lim.stop_clearance_m) / span)
            vx *= factor
            reason = "obstacle_slow"

        # Ограничение ускорения. После простоя или таймаута разгон начинается от нуля
        # с мягким стартом: первая команда не может дать больше max_accel·SOFT_START_S.
        stale = self._last_cmd_time is None or now - self._last_cmd_time > lim.cmd_timeout_s
        prev = 0.0 if stale else self._last.vx
        dt = SOFT_START_S if stale else max(0.0, now - self._last_cmd_time)
        before_ramp = vx
        if vx > prev:
            vx = min(vx, prev + lim.max_accel * dt)
        else:
            vx = max(vx, prev - lim.max_decel * dt)
        if vx != before_ramp and reason in ("ok", "limited"):
            reason = "accel_limited"

        out = Twist(vx, wz)
        self._last = out
        self._last_cmd_time = now
        return GateResult(out, reason)

    def timeout(self, now: float) -> Optional[GateResult]:
        """Если команда не обновлялась дольше таймаута — стоп."""
        if self._last.is_zero() or self._last_cmd_time is None:
            return None
        if now - self._last_cmd_time > self.limits.cmd_timeout_s:
            return self._zero("cmd_timeout", now)
        return None


@dataclass(frozen=True)
class ChassisPlan:
    """План для шасси: углы модулей (градусы) и тяга (проценты)."""

    steer_deg: dict
    drive_percent: float


class PlanError(ValueError):
    """Команду нельзя выполнить: отказ, а не обрезка."""


def plan_chassis(twist: Twist, geometry: Geometry = Geometry()) -> ChassisPlan:
    """Кинематика 4WIS для движения вперёд с рысканием.

    Скорость точки модуля в системе шасси: (vx - wz·y, wz·x). Угол руления —
    направление этой скорости, тяга — модуль скорости, пересчитанный в проценты
    через full_scale_mps. Угол вне ±32° — PlanError (поворот слишком крутой для
    этого шасси на данной скорости).
    """
    if twist.vx < 0:
        raise PlanError("задний ход не планируется")
    if not (math.isfinite(twist.vx) and math.isfinite(twist.wz)):
        raise PlanError("некорректная команда")

    steer: dict[str, float] = {}
    speeds: list[float] = []
    for name, (x, y) in geometry.module_positions().items():
        vx_m = twist.vx - twist.wz * y
        vy_m = twist.wz * x
        speed = math.hypot(vx_m, vy_m)
        if speed < 1e-9:
            angle = 0.0
        else:
            angle = math.degrees(math.atan2(vy_m, vx_m))
        if abs(angle) > STEER_LIMIT_DEG + 1e-9:
            raise PlanError("модуль %s: угол %.1f° вне ±%.0f°" % (name, angle, STEER_LIMIT_DEG))
        steer[name] = round(angle, 2) + 0.0  # +0.0 убирает "-0.0"
        speeds.append(speed)

    fastest = max(speeds)
    if geometry.full_scale_mps <= 0:
        raise PlanError("некорректная геометрия: full_scale_mps")
    percent = min(DRIVE_LIMIT_PERCENT, fastest / geometry.full_scale_mps * DRIVE_LIMIT_PERCENT)
    return ChassisPlan(steer_deg=steer, drive_percent=round(percent, 2))


def steer_change_needs_stop(current: dict, target: dict, tolerance_deg: float = 0.5) -> bool:
    """Руль нельзя поворачивать при ненулевой тяге: нужен стоп перед поворотом."""
    for name, angle in target.items():
        if abs(float(current.get(name, 0.0)) - float(angle)) > tolerance_deg:
            return True
    return False


@dataclass
class MotionExecutor:
    """Выполняет план через методы источника шасси. Сам моторы не трогает."""

    source: object
    geometry: Geometry = field(default_factory=Geometry)
    gate: MotionGate = field(default_factory=MotionGate)
    current_steer: dict = field(default_factory=dict)
    current_percent: float = 0.0

    def _send_stop(self) -> dict:
        result = self.source.drive(0)
        if result.get("ok"):
            self.current_percent = 0.0
        return result

    def command(self, requested: Twist, state: GateInput) -> dict:
        gated = self.gate.check(requested, state)
        if gated.twist.is_zero():
            if self.current_percent != 0.0:
                stop = self._send_stop()
                return {"ok": bool(stop.get("ok")), "reason": gated.reason, "stop": stop}
            return {"ok": True, "reason": gated.reason, "stop": None}

        try:
            plan = plan_chassis(gated.twist, self.geometry)
        except PlanError as exc:
            stop = self._send_stop() if self.current_percent != 0.0 else None
            return {"ok": False, "reason": "plan_refused", "detail": str(exc), "stop": stop}

        if self.current_percent != 0.0 and steer_change_needs_stop(self.current_steer, plan.steer_deg):
            stop = self._send_stop()
            if not stop.get("ok"):
                return {"ok": False, "reason": "stop_failed", "stop": stop}
            # Повторный разбор: руль поворачиваем только на нулевой тяге.
            return {"ok": True, "reason": "steer_wait_for_stop", "stop": stop}

        for name, angle in plan.steer_deg.items():
            if abs(float(self.current_steer.get(name, 0.0)) - angle) <= 0.5:
                continue
            result = self.source.steer_module(name, angle)
            if not result.get("ok"):
                stop = self._send_stop() if self.current_percent != 0.0 else None
                return {"ok": False, "reason": "steer_failed", "module": name,
                        "detail": result.get("reason"), "stop": stop}
            self.current_steer[name] = angle

        result = self.source.drive(plan.drive_percent)
        if result.get("ok"):
            self.current_percent = plan.drive_percent
        return {"ok": bool(result.get("ok")), "reason": gated.reason,
                "plan": plan, "drive": result}
