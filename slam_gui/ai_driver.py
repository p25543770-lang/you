#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ai_driver.py — ИИ-водитель стенда RUS SLAM.

Здесь живёт настоящая нейросеть, а не украшение:

* **Сеть** — полносвязный перцептрон с tanh-активацией, 10 входов → 24 скрытых
  нейрона → 8 выходных = **32 нейрона** (входы нейронами не считаются: это
  сигналы датчиков и задания). Обучение — обратное распространение ошибки,
  один шаг на каждый такт управления, прямо на ходу.
* **Учитель** — геометрический решатель: он знает, как поставить колёса
  4WIS-машины, чтобы доехать до цели (прямо, боком, разворотом на месте), и
  сеть учится его повторять. В управление идут **выходы сети**, а не учителя:
  разница между ними видна на экране как «ошибка».
* **ROS 2** — равноправный участник: те же числа (вперёд/боком/угловая
  скорость), которыми живёт стенд, уходят в изолированный ``/sim/cmd_vel`` через
  :class:`RosCommandSink`, если рядом есть rclpy. Без ROS стенд продолжает
  ехать, и панель честно пишет, что ROS рядом нет.

Модуль подключается и как часть ``gui/backend.py``, и в тестах напрямую;
он не тянет ничего, кроме стандартной библиотеки.
"""

from __future__ import annotations

import math
import random

#: Входы сети: задание в системе робота, препятствия и состояние машины.
INPUTS = ("dx", "dy", "dth", "crab", "obstL", "obstC", "obstR", "soc", "cargo", "speed")

#: Выходы: четыре угла колёс, тяга и три оценки манёвра.
OUTPUTS = ("FL", "FR", "RL", "RR", "throttle", "s_fwd", "s_crab", "s_rot")

HIDDEN = 24
#: Нейронов ровно столько — 24 скрытых + 8 выходных. Это проверяется тестом.
NEURONS = HIDDEN + len(OUTPUTS)                       # = 32

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
ANGLE_LIMIT = 90.0        # ° — больше рулевой модуль не встаёт
V_MAX = 1.0               # м/с — скорость стенда
W_MAX = 1.4               # рад/с — разворот на месте

#: Дальность дальномера, к которой нормированы входы-препятствия (0…1):
#: близость = 1 − расстояние / SENSOR_RANGE. Так же считает и сам стенд.
SENSOR_RANGE = 3.0

#: Слова для экрана: знак угла — «вправо», как договорились в интерфейсе.
STRAIGHT_DEG = 3.0
AVOID_DEG = 45.0                     # угол объезда, °
SOFT_DEG = 7.0                       # ширина перехода между манёврами, °
SPIN_DEG = 25.0                      # разница перед/зад, после которой это разворот
CRAB_DEG = 40.0                      # угол, после которого параллельные колёса — краб


def _sig(x: float, edge: float, width: float = SOFT_DEG) -> float:
    """Плавный порог: 0 ниже кромки, 1 выше — без ступеньки для обучения."""
    return 1.0 / (1.0 + math.exp(-(x - edge) / max(1e-3, width)))
TURN_DEADBAND_DEG = 4.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _norm_angle(a: float) -> float:
    """Приводит угол к (−π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def steer_word(angle_deg: float) -> str:
    """«прямо» / «вправо» / «влево» — направление поворота колеса."""
    if abs(angle_deg) < STRAIGHT_DEG:
        return "прямо"
    return "вправо" if angle_deg > 0 else "влево"


def _ahead_m(obst) -> float:
    """Расстояние до препятствия по центру, м — из близости 0…1."""
    return (1.0 - _clamp(float(obst[1]), 0.0, 1.0)) * SENSOR_RANGE


def geometric_target(dx: float, dy: float, dth: float, crab: bool = False,
                     obst=None) -> dict:
    """Как встать колёсами, чтобы доехать: учитель сети.

    ``dx``, ``dy`` — цель в системе робота, м (x — вперёд, y — влево);
    ``dth`` — рассогласование курса, рад; ``obst`` — близость препятствий
    (слева, по центру, справа) с дальномера, 0…1. Возвращает углы колёс
    в градусах, тягу и вектор для обучения (−1…1 по углам и тяге плюс
    оценки манёвра).
    """
    # «вправо-положительное» рассогласование: так же, как подписан угол колеса
    turn = -math.degrees(dth)
    dist = math.hypot(dx, dy)

    def _throttle(ceil: float, floor: float = 0.16) -> float:
        """Тяга по остатку пути: далеко — быстрее, у цели — подкрадываемся."""
        return _clamp(floor + 0.55 * dist, floor, ceil)

    mode = "вперёд"
    throttle = _throttle(0.60)
    # «в стороне» считаем по направлению на цель, а не по рассогласованию курса:
    # у крабовых заданий курс держат, и рассогласование там нулевое
    bear = abs(math.degrees(math.atan2(dy, max(dx, 0.001))))
    near = dist <= 0.30

    # Оценки манёвров — мягкие: у сеть должна быть гладкая цель, иначе пороги
    # «70°» и «105°» превращаются для неё в шум, и обучение не сходится.
    # разворот — по рассогласованию курса, краб — по направлению на цель:
    # у крабового задания курс держат, там рассогласование нулевое
    # Разворот на месте — только если цель за спиной (или это задание
    # «развернуться»: тогда dist = 0 и нужно выставить курс). Всё остальное —
    # езда передом: колесо в сторону поворота, перед доворачивает больше зада.
    s_rot = _sig(abs(turn), 115.0)
    if dist <= 0.05:
        s_rot = max(s_rot, _sig(abs(turn), 4.0))
    front_m = _ahead_m(obst) if obst is not None else SENSOR_RANGE
    if front_m < 0.8:
        # Нос почти в стене: доворачиваться дугой некуда — у 4WIS для этого есть
        # разворот на месте, центр при этом не сдвигается. Так и выходят из угла:
        # довернулся на месте — и поехал носом, а не скребёшь стену дугой.
        s_rot = max(s_rot, _sig(abs(turn), 35.0, 20.0))
    # Вблизи цели доворачиваться дугой невыгодно: радиус дуги (~0,25 м) сравним
    # с остатком пути, и машина начинает кружить вокруг точки. Настоящая 4WIS в
    # такой ситуации доворачивается на месте, а потом едет прямо. Так и делаем.
    if dist <= 0.8:
        s_rot = max(s_rot, _sig(abs(turn), 50.0))
    s_crab = _sig(bear, 40.0) * (1.0 - _sig(bear, 105.0)) if crab else 0.0
    if near:
        s_crab *= 0.0 if dist <= 0.12 else 1.0
    s_fwd = max(0.0, 1.0 - max(s_rot, s_crab))
    total = s_rot + s_crab + s_fwd or 1.0
    s_rot, s_crab, s_fwd = s_rot / total, s_crab / total, s_fwd / total
    mode = max((("разворот", s_rot), ("краб", s_crab), ("вперёд", s_fwd)), key=lambda kv: kv[1])[0]

    side = mode == "краб"
    if mode == "разворот":
        # смотрим не туда: доворот на месте, пока курс не сойдётся
        sign = 1.0 if turn > 0 else -1.0
        angles = dict.fromkeys(WHEEL_NAMES)
        angles["FL"] = angles["FR"] = ANGLE_LIMIT * sign
        angles["RL"] = angles["RR"] = -ANGLE_LIMIT * sign
        throttle = 0.4
    elif dx < -0.05 and abs(dy) <= 0.6 and dist <= 1.5 and abs(turn) <= 30.0:
        # Цель прямо за кормой и недалеко: машина сдаёт назад, как автомобиль.
        # Ехать задним ходом далеко невыгодно — там уже разворот на месте.
        steer = _clamp(turn * 0.45, -35.0, 35.0)
        angles = {"FL": steer, "FR": steer, "RL": -steer, "RR": -steer}
        mode, throttle = "назад", -min(0.45, _throttle(0.45))
    elif obst is not None and not side and _ahead_m(obst) < 0.6 \
            and _ahead_m(obst) < 0.6 * max(dist, 0.35):
        # Впереди ближе, чем до цели: нос в свободную сторону, ход придерживаем.
        # Объезжаем только то, что действительно мешает ехать к цели: стена в
        # конце прохода не повод вилять, если сама цель ближе препятствия.
        # Контрфаза доворачивает корпус, а не сносит его боком — груз не гуляет.
        side_dir = -1.0 if obst[2] > obst[0] else 1.0
        steer = 0.7 * AVOID_DEG * side_dir
        angles = {"FL": steer, "FR": steer, "RL": -steer, "RR": -steer}
        mode, throttle = "объезд", 0.35
    elif side:
        # все четыре колеса параллельно: машина едет вбок, курс не меняется
        steer = _clamp(math.degrees(math.atan2(-dy, max(dx, 0.03))), -90.0, 90.0)
        angles = {name: steer for name in WHEEL_NAMES}
        throttle = _throttle(0.50)
    else:
        # Ход передом: перед и зад в контрфазе — корпус идёт носом и доворачивает
        # дугой, боковой снос нулевой (см. body_velocity). Так ведёт себя
        # настоящая 4WIS-машина: к цели она едет носом, а не боком, и груз не
        # гуляет. Чем больше рассогласование курса, тем круче контрфаза — вплоть
        # до разворота «в несколько приёмов», как на машине.
        steer = _clamp(turn * 0.55, -AVOID_DEG - 10.0, AVOID_DEG + 10.0)
        angles = {"FL": steer, "FR": steer, "RL": -steer, "RR": -steer}
        # на крутом довороте придерживаем ход, к цели — подкрадываемся
        throttle = _throttle(0.60) * (1.0 - 0.55 * min(1.0, abs(turn) / 90.0))
    vector = [angles[name] / ANGLE_LIMIT for name in WHEEL_NAMES]
    vector.append(_clamp(throttle, -1.0, 1.0))
    vector += [s_fwd, s_crab, s_rot]
    return {"angles": angles, "throttle": throttle, "mode": mode, "vector": vector}


def command_from_outputs(y) -> dict:
    """Превращает выходы сети в команду машине: углы, тяга, слово манёвра.

    Манёвр читается по самой геометрии колёс, а не по отдельному выходу-оценке:
    передняя и задняя пары в противоходе — разворот на месте; все четыре
    параллельно и круто — краб; параллельно и почти прямо — ход. Оценки
    (``вперёд``/``краб``/``разворот``) остаются намерением сети и идут на экран,
    но машину ведут колёса: классификатор ошибался бы, и шасси замирало.
    Тут же согласование 4WIS: в переводе четыре модуля выравниваются, в
    развороте идут парами, иначе остаточный «клин» уводит корпус вбок.
    """
    raw = {name: _clamp(float(y[i]) * ANGLE_LIMIT, -ANGLE_LIMIT, ANGLE_LIMIT)
           for i, name in enumerate(WHEEL_NAMES)}
    throttle = _clamp(float(y[4]), -1.0, 1.0)
    front = (raw["FL"] + raw["FR"]) / 2.0
    rear = (raw["RL"] + raw["RR"]) / 2.0
    mean = sum(raw.values()) / len(raw)
    spread = max(raw.values()) - min(raw.values())
    if abs(front - rear) > 2.0 * (AVOID_DEG + 20.0):      # пары в противоход, круто
        mode = "разворот"
        angles = {"FL": front, "FR": front, "RL": rear, "RR": rear}
    elif spread <= 4.0:                                   # все четыре параллельно
        angles = dict.fromkeys(WHEEL_NAMES, mean)
        mode = "краб" if abs(mean) > CRAB_DEG else ("назад" if throttle < -0.05 else "вперёд")
    else:
        # контрфаза: перед в одну сторону, зад в другую — ход передом с доворотом
        # (или задним ходом, если тяга отрицательная)
        angles = dict(raw)
        mode = "назад" if throttle < -0.05 else "вперёд"
    scores = {"вперёд": float(y[5]), "краб": float(y[6]), "разворот": float(y[7])}
    # слово «вправо/влево» — по тому, куда машина на самом деле поедет: у
    # контрфазы средний угол нулевой, и по нему направление не прочитать
    vx, vy, wz = body_velocity(angles, throttle)
    if abs(wz) > 0.12:                                    # машина доворачивает
        word = "вправо" if wz < 0.0 else "влево"
    elif math.hypot(vx, vy) > 0.03:                       # едем: смотрим, куда
        word = steer_word(math.degrees(math.atan2(-vy, vx)))
    else:
        word = "прямо"
    mean_angle = math.degrees(math.atan2(-vy, vx)) if math.hypot(vx, vy) > 0.03 else 0.0
    intent = max(scores, key=scores.get)
    pieces = [mode]
    if word != "прямо":
        pieces.append(word)
    speed = abs(throttle) * V_MAX
    label = " · ".join(pieces) + " · " + f"{speed:.1f} м/с".replace(".", ",")
    return {"angles": angles, "throttle": throttle, "mode": mode,
            "steer": word, "meanAngle": mean_angle, "intent": intent,
            "intents": {k: round(v, 3) for k, v in scores.items()}, "label": label}


#: База машины — расстояние между осями модулей, м (по раме робота).
WHEELBASE = 0.62


def body_velocity(angles: dict, throttle: float) -> tuple:
    """Скорости твёрдого корпуса 4WIS: (вперёд, вбок, угловая).

    Модель как у настоящей машины с четырьмя поворотными модулями: передняя и
    задняя оси едут каждая туда, куда смотрят её колёса, а корпус — твёрдое
    тело между ними. Отсюда всё поведение:

    * **передом** — перед и зад смотрят почти одинаково: машина едет вперёд и
      доворачивает дугой (угловая скорость — из разницы перед/зад и базы);
    * **разворот на месте** — пары в противоход (±90°): корпус крутится, но
      не едет (боковой снос нулевой);
    * **краб** — все четыре параллельно под углом: едем боком, курс держим.

    Угловая скорость пропорциональна скорости хода, поэтому на месте машина не
    крутится сама по себе — как и настоящая. Положительный угол колеса —
    «вправо», ось y смотрит влево (как в ROS), поэтому боковая скорость и
    угловая идут со знаком минус.
    """
    front = (angles["FL"] + angles["FR"]) / 2.0
    rear = (angles["RL"] + angles["RR"]) / 2.0
    phi = (front + rear) / 2.0            # куда смотрит машина в среднем
    delta = (front - rear) / 2.0          # насколько перед крутит больше зада
    speed = throttle * V_MAX
    rad = math.radians(phi)
    slip = math.cos(math.radians(delta))  # на дуге часть хода уходит в поворот
    vx = speed * math.cos(rad) * slip
    vy = -speed * math.sin(rad) * slip
    wz = -speed * 2.0 * math.sin(math.radians(delta)) * math.cos(rad) / WHEELBASE
    return vx, vy, wz


class SafetySupervisor:
    """Независимый ограничитель движения между нейросетью и приводом.

    Сеть выбирает команду, но не может отменить этот слой: он проверяет свежий
    круговой /scan и свежую /odom, оценивает свободный коридор в направлении
    фактического движения (включая задний ход и «краб») и снижает тягу по
    тормозному пути.
    Если датчик неисправен/устарел или команда ведёт в слишком тесное место,
    на привод уходит нулевая тяга. Геометрия корпуса — консервативная стартовая
    оценка от колёсной базы; перед реальной машиной её обязательно сверить с
    габаритами и тормозами конкретного шасси.
    """

    SCAN_MAX_AGE_S = 0.25       # два с половиной периода лидара 10 Гц
    MIN_RAYS = 72               # полный круг, не грубее примерно 5° на луч
    REACTION_S = 0.20           # выдержка на обновление лидара и реакцию привода
    BRAKE_DECEL_MPS2 = 0.65     # консервативное замедление, м/с²
    MARGIN_M = 0.08             # дополнительный отступ от расчётного контура
    HALF_LENGTH_M = WHEELBASE / 2.0
    HALF_WIDTH_M = 0.23         # временная оценка; откалибровать по корпусу
    ODOM_MAX_AGE_S = 0.25       # без свежей скорости нельзя безопасно учесть инерцию
    MIN_COMMAND_SPEED = 0.025   # ниже — считаем поступательное движение нулевым
    MIN_CURRENT_SPEED = 0.035

    def __init__(self, scan_max_age_s: float = SCAN_MAX_AGE_S):
        self.scan_max_age_s = float(scan_max_age_s)

    def _scan_points(self, scan, now):
        """Проверяет свежесть/полноту /scan и возвращает точки в координатах базы."""
        if not isinstance(scan, dict):
            return None, None, None, "нет данных лидара"
        ranges = scan.get("ranges")
        if not isinstance(ranges, (list, tuple)) or len(ranges) < self.MIN_RAYS:
            return None, None, None, "неполный скан лидара"
        try:
            stamp = float(scan["stamp"])
            angle_min = float(scan["angle_min"])
            angle_inc = float(scan["angle_inc"])
            range_max = float(scan["range_max"])
            now = float(now)
        except (KeyError, TypeError, ValueError, OverflowError):
            return None, None, None, "неверные метки лидара"
        if not all(math.isfinite(v) for v in (stamp, angle_min, angle_inc, range_max, now)) \
                or angle_inc == 0.0 or range_max <= 0.0:
            return None, None, None, "неверные параметры лидара"
        if abs(angle_inc) > math.radians(5.5):
            return None, None, None, "слишком редкий скан лидара"
        coverage = abs(angle_inc) * len(ranges)
        if coverage < 2.0 * math.pi - max(0.10, 1.5 * abs(angle_inc)):
            return None, None, None, "лидар не видит кругом"
        age = now - stamp
        if age < -0.05:
            return None, None, None, "время лидара не синхронизировано"
        age = max(0.0, age)          # время отметки /scan может опережать входной now на доли мс
        if age > self.scan_max_age_s:
            return None, None, age, "лидар устарел"

        points = []
        nearest = range_max
        for i, raw in enumerate(ranges):
            try:
                distance = float(raw)
            except (TypeError, ValueError, OverflowError):
                return None, None, age, "повреждён скан лидара"
            if math.isnan(distance) or distance == -math.inf or distance < 0.0:
                return None, None, age, "повреждён скан лидара"
            # В ROS бесконечность означает «эхо не найдено до range_max».
            if distance == math.inf:
                distance = range_max
            else:
                distance = min(distance, range_max)
            angle = angle_min + i * angle_inc
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append((x, y))
            if distance < nearest:
                nearest = distance
        return points, nearest, age, ""

    def _path_clearance(self, points, vx, vy, wz, speed):
        """Проверяет дугу на длину тормозного пути с раздутым контуром корпуса.

        Интегрируем командную дугу с шагом 4 см — также для бокового хода. Эхо
        должно оставаться вне прямоугольного корпуса на всём пути реакции и
        торможения, а не только на текущем луче «вперёд».
        """
        if speed <= 1e-6:
            return None
        a, reaction = self.BRAKE_DECEL_MPS2, self.REACTION_S
        stop_path = reaction * speed + speed * speed / (2.0 * a)
        path_step = 0.04
        steps = max(1, int(math.ceil(stop_path / path_step)))
        half_length = self.HALF_LENGTH_M + self.MARGIN_M
        half_width = self.HALF_WIDTH_M + self.MARGIN_M
        for index in range(steps + 1):
            distance = min(index * path_step, stop_path)
            theta = wz * distance / speed
            if abs(wz) <= 1e-6:
                px, py = vx / speed * distance, vy / speed * distance
            else:
                sin_t, cos_t = math.sin(theta), math.cos(theta)
                px = (vx * sin_t + vy * (cos_t - 1.0)) / wz
                py = (vx * (1.0 - cos_t) + vy * sin_t) / wz
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            for ox, oy in points:
                rx, ry = ox - px, oy - py
                local_x = rx * cos_t + ry * sin_t
                if abs(local_x) > half_length:
                    continue
                local_y = -rx * sin_t + ry * cos_t
                if abs(local_y) <= half_width:
                    # Недооценка на один шаг компенсирует дискретизацию дуги.
                    return max(0.0, distance - path_step)
        return None

    def _safe_speed(self, clearance):
        """Максимум скорости по расстоянию до первого пересечения корпуса."""
        available = max(0.0, clearance)
        a, t = self.BRAKE_DECEL_MPS2, self.REACTION_S
        # Решение v*t + v²/(2a) <= свободный путь.
        safe_speed = max(0.0, math.sqrt((a * t) ** 2 + 2.0 * a * available) - a * t)
        return min(V_MAX, safe_speed)

    def filter(self, command: dict, scan: dict, now: float,
               current_velocity=None, current_velocity_age_s=None) -> tuple:
        """Возвращает (безопасная команда, отчёт защиты); не меняет углы руления."""
        command_error = not isinstance(command, dict)
        safe = dict(command) if isinstance(command, dict) else {}
        raw_angles = safe.get("angles")
        if not isinstance(raw_angles, dict) or any(name not in raw_angles for name in WHEEL_NAMES):
            command_error = True
            raw_angles = raw_angles if isinstance(raw_angles, dict) else {}
        safe_angles = {}
        for name in WHEEL_NAMES:
            try:
                angle = float(raw_angles.get(name, 0.0))
                if not math.isfinite(angle):
                    raise ValueError("non-finite wheel angle")
                safe_angles[name] = _clamp(angle, -ANGLE_LIMIT, ANGLE_LIMIT)
            except (AttributeError, TypeError, ValueError, OverflowError):
                safe_angles[name] = 0.0
                command_error = True
        safe["angles"] = safe_angles
        try:
            requested_throttle = _clamp(float(safe.get("throttle", 0.0)), -1.0, 1.0)
            if not math.isfinite(requested_throttle):
                raise ValueError("non-finite throttle")
        except (TypeError, ValueError, OverflowError):
            requested_throttle = 0.0
            command_error = True
        if command_error:
            requested_throttle = 0.0
        safe["throttle"] = requested_throttle
        requested_v = body_velocity(safe["angles"], requested_throttle)
        requested_speed = math.hypot(requested_v[0], requested_v[1])
        try:
            if current_velocity is None or current_velocity_age_s is None:
                raise ValueError("missing odometry")
            odom_age = float(current_velocity_age_s)
            if not math.isfinite(odom_age) or odom_age < -0.05 or odom_age > self.ODOM_MAX_AGE_S:
                raise ValueError("stale odometry")
            current_vx, current_vy = float(current_velocity[0]), float(current_velocity[1])
            current_wz = float(current_velocity[2]) if len(current_velocity) > 2 else 0.0
            if not all(math.isfinite(v) for v in (current_vx, current_vy, current_wz)):
                raise ValueError("non-finite odometry")
        except (TypeError, ValueError, IndexError, OverflowError):
            current_vx = current_vy = current_wz = 0.0
            velocity_error = True
        else:
            velocity_error = False
        current_speed = math.hypot(current_vx, current_vy)

        if command_error or velocity_error:
            error = "неверная команда колёс" if command_error else "нет достоверной одометрии"
            safe["throttle"] = 0.0
            self._update_label(safe, "stop")
            return safe, {"state": "stop", "active": True, "reason": error,
                          "clearanceM": None, "requestedMps": round(requested_speed, 2),
                          "safeMps": 0.0, "limitMps": 0.0, "scanAgeMs": None}

        points, nearest, age, error = self._scan_points(scan, now)
        result = {
            "state": "ok", "active": False, "reason": "контроль активен",
            "clearanceM": None if nearest is None else round(nearest, 2),
            "requestedMps": round(requested_speed, 2),
            "limitMps": round(V_MAX, 2), "scanAgeMs": None if age is None else round(age * 1000.0),
        }
        if error:
            safe["throttle"] = 0.0
            result.update(state="stop", active=True, reason=error, limitMps=0.0, safeMps=0.0)
            self._update_label(safe, "stop")
            return safe, result

        command_limits = []
        clearances = []
        must_brake = False
        # Проверяем и выбранный нейросетью путь, и текущую инерцию: при смене
        # направления машина сначала продолжает катиться по старому курсу.
        paths = []
        if requested_speed >= self.MIN_COMMAND_SPEED:
            paths.append((requested_v[0], requested_v[1], requested_v[2],
                          requested_speed, "команда"))
        if current_speed >= self.MIN_CURRENT_SPEED:
            paths.append((current_vx, current_vy, current_wz, current_speed, "инерция"))

        for vx, vy, wz, speed, path_name in paths:
            clearance = self._path_clearance(points, vx, vy, wz, speed)
            # Проверяем до полной остановки при текущей скорости. Если дуга
            # свободна на всём тормозном пути, текущая уставка проходит.
            safe_limit = self._safe_speed(clearance) if clearance is not None else V_MAX
            if clearance is not None:
                clearances.append(clearance)
            if path_name == "команда":
                command_limits.append(safe_limit)
            elif current_speed > safe_limit + 0.02:
                must_brake = True
                result["reason"] = "торможение перед препятствием"

        # При развороте на месте корпус заметает габаритный круг, а не только
        # точку центра. Запрещаем спин, если лидар видит препятствие в этом круге.
        spinning = (requested_speed < self.MIN_COMMAND_SPEED and abs(requested_v[2]) > 0.02) \
            or (current_speed < self.MIN_CURRENT_SPEED and abs(current_wz) > 0.02)
        if spinning:
            turn_clearance = float(nearest)
            turn_buffer = math.hypot(self.HALF_LENGTH_M, self.HALF_WIDTH_M) + self.MARGIN_M
            clearances.append(turn_clearance)
            if turn_clearance <= turn_buffer:
                must_brake = True
                result["reason"] = "мало места для разворота"
                command_limits.append(0.0)
        elif not paths and nearest <= math.hypot(self.HALF_LENGTH_M, self.HALF_WIDTH_M) + self.MARGIN_M:
            # Даже при нулевой уставке показываем, что корпус стоит у препятствия.
            must_brake = True
            result["reason"] = "препятствие близко к корпусу"
            command_limits.append(0.0)

        command_limit = min(command_limits) if command_limits else V_MAX
        if command_limit <= 1e-3 and requested_speed >= self.MIN_COMMAND_SPEED:
            must_brake = True
            result["reason"] = "препятствие в стоп-зоне"
        if must_brake:
            safe["throttle"] = 0.0
        elif safe["throttle"] != 0.0 and requested_speed > command_limit + 1e-3:
            safe["throttle"] *= command_limit / max(requested_speed, 1e-9)
            result["reason"] = "скорость снижена по тормозному пути"
        result["limitMps"] = round(max(0.0, command_limit), 2)

        if clearances:
            result["clearanceM"] = round(min(clearances), 2)
        if must_brake:
            result.update(state="stop", active=True, limitMps=0.0)
        elif abs(safe["throttle"]) < 1e-3 and abs(requested_throttle) >= 1e-3:
            result.update(state="stop", active=True, limitMps=0.0)
            if result["reason"] == "контроль активен":
                result["reason"] = "защитная остановка"
        elif abs(safe["throttle"]) + 1e-3 < abs(requested_throttle):
            result.update(state="slow", active=True)
        self._update_label(safe, result["state"])
        result["safeMps"] = round(math.hypot(*body_velocity(safe["angles"], safe["throttle"])[:2]), 2)
        return safe, result

    @staticmethod
    def _update_label(command, state):
        """Скорость и состояние на этикетке соответствуют уставке после защиты."""
        parts = [part for part in str(command.get("label") or "").split(" · ")
                 if part not in {"СТОП защиты", "ограничение защиты"}]
        if parts and "м/с" in parts[-1]:
            speed = abs(float(command.get("throttle", 0.0))) * V_MAX
            parts[-1] = ("%.1f м/с" % speed).replace(".", ",")
        if state == "stop":
            parts.append("СТОП защиты")
        elif state == "slow":
            parts.append("ограничение защиты")
        if parts and command.get("label"):
            command["label"] = " · ".join(parts)


def scenario_inputs(dx: float, dy: float, dth: float, crab: bool, obst, soc: float = 0.8,
                    cargo: float = 1.0, speed: float = 0.3) -> dict:
    """Входы сети по обстановке: и для очной тренировки, и для езды."""
    return {
        "dx": round(_clamp(dx / 2.0, -1.0, 1.0), 3),
        "dy": round(_clamp(dy / 2.0, -1.0, 1.0), 3),
        "dth": round(_clamp(-math.degrees(dth) / 90.0, -1.0, 1.0), 3),
        "crab": 1.0 if crab else 0.0,
        "obstL": float(obst[0]), "obstC": float(obst[1]), "obstR": float(obst[2]),
        "soc": round(_clamp(soc, 0.0, 1.0), 3),
        "cargo": round(_clamp(cargo, 0.0, 1.0), 3),
        "speed": round(_clamp(speed, 0.0, 1.0), 3),
    }


#: Веса после очного урока: урок делается один раз на процесс, дальше каждая
#: сеть начинает с готовых весов и доучивается на ходу сама.
_WARM_CACHE = {}


class NeuralDriver:
    """Перцептрон 10 → 24 → 8 с обучением на ходу (обратное распространение).

    Обучение — стохастический градиентный спуск с инерцией: без неё веса
    успевают за сменой манёвра только через полсекунды, и машина едет «по
    старой команде», пока сеть догоняет учителя.
    """

    def __init__(self, seed: int = 7, lr: float = 0.12, momentum: float = 0.0):
        self.rnd = random.Random(seed)
        self.lr = lr
        self.momentum = momentum
        self.w1 = [[self.rnd.uniform(-0.35, 0.35) for _ in INPUTS] for _ in range(HIDDEN)]
        self.b1 = [0.0] * HIDDEN
        self.w2 = [[self.rnd.uniform(-0.35, 0.35) for _ in range(HIDDEN)] for _ in OUTPUTS]
        self.b2 = [0.0] * len(OUTPUTS)
        self.vw1 = [[0.0] * len(INPUTS) for _ in range(HIDDEN)]
        self.vb1 = [0.0] * HIDDEN
        self.vw2 = [[0.0] * HIDDEN for _ in OUTPUTS]
        self.vb2 = [0.0] * len(OUTPUTS)
        self.steps = 0
        self.loss = 0.0
        self.loss_avg = None
        self.last_h = [0.0] * HIDDEN
        self.last_y = [0.0] * len(OUTPUTS)

    # --- счёт ----------------------------------------------------------------
    @property
    def neurons(self) -> int:
        return HIDDEN + len(self.w2)

    @property
    def layers(self) -> list:
        return [len(INPUTS), HIDDEN, len(self.w2)]

    def forward(self, x):
        x = list(x)
        h = [math.tanh(sum(w * xi for w, xi in zip(row, x)) + b)
             for row, b in zip(self.w1, self.b1)]
        y = [math.tanh(sum(w * hi for w, hi in zip(row, h)) + b)
             for row, b in zip(self.w2, self.b2)]
        self.last_h, self.last_y = h, y
        return h, y

    def step(self, x, target):
        """Один такт: прямой проход, шаг обучения на ошибке, выходы наружу."""
        h, y = self.forward(x)
        err = [t - o for t, o in zip(target, y)]
        loss = sum(e * e for e in err) / len(err)

        d_out = [e * (1.0 - o * o) for e, o in zip(err, y)]      # производная tanh
        d_hidden = []
        for j in range(HIDDEN):
            acc = sum(d_out[k] * self.w2[k][j] for k in range(len(self.w2)))
            d_hidden.append(acc * (1.0 - h[j] * h[j]))

        mu = self.momentum
        for k in range(len(self.w2)):
            for j in range(HIDDEN):
                step = self.lr * d_out[k] * h[j]
                self.vw2[k][j] = mu * self.vw2[k][j] + step
                self.w2[k][j] += self.vw2[k][j]
            self.vb2[k] = mu * self.vb2[k] + self.lr * d_out[k]
            self.b2[k] += self.vb2[k]
        for j in range(HIDDEN):
            for i, xi in enumerate(x):
                step = self.lr * d_hidden[j] * xi
                self.vw1[j][i] = mu * self.vw1[j][i] + step
                self.w1[j][i] += self.vw1[j][i]
            self.vb1[j] = mu * self.vb1[j] + self.lr * d_hidden[j]
            self.b1[j] += self.vb1[j]

        self.steps += 1
        self.loss = loss
        self.loss_avg = loss if self.loss_avg is None else self.loss_avg * 0.97 + loss * 0.03
        return {"y": y, "h": h, "loss": loss, "loss_avg": self.loss_avg, "steps": self.steps}

    # --- веса ---
    def weights(self) -> dict:
        return {"w1": [row[:] for row in self.w1], "b1": self.b1[:],
                "w2": [row[:] for row in self.w2], "b2": self.b2[:]}

    def load(self, pack: dict) -> None:
        self.w1 = [row[:] for row in pack["w1"]]
        self.b1 = pack["b1"][:]
        self.w2 = [row[:] for row in pack["w2"]]
        self.b2 = pack["b2"][:]

    def pretrain(self, steps: int = 4000, seed: int = 11) -> float:
        """Очный урок на учителе перед выездом в цех.

        Робот не выезжает с нулевыми весами: сеть проходит тренировку на
        геометрическом учителе по случайным обстановкам, а дальше доучивается
        на ходу. Урок считается один раз на процесс, дальше веса берутся из
        кэша: иначе каждый стенд и каждый тест заново учили бы одно и то же.
        """
        key = (steps, seed)
        if key not in _WARM_CACHE:
            net = NeuralDriver(seed=seed, lr=0.12, momentum=0.0)
            net.load({"w1": self.w1, "b1": self.b1, "w2": self.w2, "b2": self.b2})
            rnd = random.Random(seed)
            for _ in range(steps):
                dx, dy = rnd.uniform(-2.0, 2.0), rnd.uniform(-2.0, 2.0)
                dth = rnd.uniform(-math.pi, math.pi)
                crab = rnd.random() < 0.5
                obst = tuple(round(rnd.uniform(0.0, 1.0), 2) for _ in range(3))
                target = geometric_target(dx, dy, dth, crab=crab, obst=obst)
                scene = scenario_inputs(dx, dy, dth, crab, obst)
                net.step([scene[name] for name in INPUTS], target["vector"])
            _WARM_CACHE[key] = (net.weights(), float(net.loss_avg or 0.0))
        pack, loss = _WARM_CACHE[key]
        self.load(pack)
        return loss

    def activations(self) -> list:
        """Все 32 нейрона — для картинки на экране."""
        return [round(v, 3) for v in (self.last_h + self.last_y)]


class RosCommandSink:
    """Изолированный мост симулятора в ROS 2: ``/sim/cmd_vel``, ``/sim/scan``, ``/sim/odom``.

    Типы сообщений совместимы с ходовой ROS: Twist, LaserScan и Odometry,
    однако все синтетические данные находятся в ``/sim``. Даже если вызывающий
    код попросит ``/cmd_vel``, sink добавит изолирующий префикс: обычный драйвер
    физического робота не получит команду случайно.

    Публикация включается только явно (``RC_AI_ROS=1``); по умолчанию стенд
    никому не командует. Для аппаратного робота нужен отдельный узел с реальными
    датчиками, E-STOP и проверенным watchdog. Если rclpy рядом нет — панель так
    и говорит.
    """

    namespace = "/sim"
    topic = "/sim/cmd_vel"
    scan_topic = "/sim/scan"
    odom_topic = "/sim/odom"

    @classmethod
    def _isolate_topic(cls, topic: str) -> str:
        """Topic override никогда не снимает обязательный namespace ``/sim``."""
        name = str(topic or "cmd_vel").strip("/") or "cmd_vel"
        if name == "sim" or name.startswith("sim/"):
            return "/" + name
        return cls.namespace + "/" + name

    def __init__(self, enabled: bool = False, topic: str = "/cmd_vel"):
        self.topic = self._isolate_topic(topic)
        self.enabled = bool(enabled)
        self.available = False
        self.reason = ""
        self.published = 0
        self.published_scan = 0
        self.published_odom = 0
        self.last_error = ""
        self.extra_error = ""
        self.scan_ready = False
        self.odom_ready = False
        self.node = None
        self.publisher = None
        self.publisher_scan = None
        self.publisher_odom = None
        self._Twist = None
        self._LaserScan = None
        self._Odometry = None
        try:
            import rclpy                                   # noqa: F401
            from geometry_msgs.msg import Twist
        except ImportError:
            self.reason = "rclpy не найден — ИИ ведёт стенд"
            return
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node("rus_slam_ai")
            self.publisher = self.node.create_publisher(Twist, self.topic, 10)
            self._Twist = Twist
            self.available = True
            self.reason = (f"ROS есть: команды уходят в {self.topic}" if self.enabled
                           else "ROS есть, публикация выключена (RC_AI_ROS=1)")
        except Exception as exc:                           # noqa: BLE001 — ROS не должен валить пульт
            self.reason = f"ROS есть, но мост не поднялся: {exc}"
            self.last_error = str(exc)
            return
        # --- синтетические дальномер и поза остаются изолированы в /sim ---
        try:
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import LaserScan
            self.publisher_scan = self.node.create_publisher(LaserScan, self.scan_topic, 5)
            self.publisher_odom = self.node.create_publisher(Odometry, self.odom_topic, 10)
            self._LaserScan, self._Odometry = LaserScan, Odometry
            self.scan_ready = self.odom_ready = True
        except Exception as exc:                           # noqa: BLE001 — без /scan стенд живёт
            self.extra_error = str(exc)

    def publish_scan(self, scan) -> bool:
        """Оборот симулятора в ``/sim/scan`` (sensor_msgs/LaserScan)."""
        if not (self.scan_ready and self.enabled and self.publisher_scan) or not scan:
            return False
        try:
            msg = self._LaserScan()
            msg.header.frame_id = str(scan.get("frame_id", "laser"))
            msg.angle_min = float(scan.get("angle_min", -math.pi))
            msg.angle_increment = float(scan.get("angle_inc", 0.0))
            ranges = [float(r) for r in (scan.get("ranges") or ())]
            msg.angle_max = msg.angle_min + msg.angle_increment * max(0, len(ranges) - 1)
            msg.range_max = float(scan.get("range_max", 3.0))
            msg.ranges = ranges
            self.publisher_scan.publish(msg)
            self.published_scan += 1
            return True
        except Exception as exc:                           # noqa: BLE001
            self.last_error = str(exc)
            return False

    def publish_odom(self, odom) -> bool:
        """Синтетическая поза в ``/sim/odom`` (nav_msgs/Odometry)."""
        if not (self.odom_ready and self.enabled and self.publisher_odom) or not odom:
            return False
        try:
            msg = self._Odometry()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = float(odom.get("x", 0.0))
            msg.pose.pose.position.y = float(odom.get("y", 0.0))
            half = float(odom.get("th", 0.0)) * 0.5
            msg.pose.pose.orientation.z = math.sin(half)
            msg.pose.pose.orientation.w = math.cos(half)
            msg.twist.twist.linear.x = float(odom.get("vx", 0.0))
            msg.twist.twist.linear.y = float(odom.get("vy", 0.0))
            msg.twist.twist.angular.z = float(odom.get("wz", 0.0))
            self.publisher_odom.publish(msg)
            self.published_odom += 1
            return True
        except Exception as exc:                           # noqa: BLE001
            self.last_error = str(exc)
            return False

    def publish(self, vx: float, vy: float, wz: float) -> bool:
        """Отдаёт скорости в ROS. Возвращает False, если публикация не идёт."""
        if not (self.available and self.enabled and self.publisher):
            return False
        try:
            msg = self._Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(wz)
            self.publisher.publish(msg)
            self.published += 1
            return True
        except Exception as exc:                           # noqa: BLE001
            self.last_error = str(exc)
            return False

    def status(self) -> dict:
        return {
            "available": self.available,
            "enabled": self.enabled,
            "topic": self.topic,
            "scanTopic": self.scan_topic,
            "odomTopic": self.odom_topic,
            "published": self.published,
            "publishedScan": self.published_scan,
            "publishedOdom": self.published_odom,
            "scanReady": self.scan_ready,
            "odomReady": self.odom_ready,
            "reason": self.reason,
            "error": self.last_error or self.extra_error,
        }
