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
  скорость), которыми живёт стенд, уходят в ``/cmd_vel`` через
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
    s_rot = _sig(abs(turn), 105.0)
    if not crab:
        s_rot = max(s_rot, _sig(abs(turn), 70.0))
    s_crab = _sig(bear, 40.0) * (1.0 - _sig(bear, 105.0)) if crab else 0.0
    if near:
        s_rot = max(s_rot, _sig(abs(turn), 4.0))
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
        throttle = 0.3
    elif obst is not None and obst[1] > 0.5 and not side:
        # впереди близко: объезжаем с более свободной стороны
        side_dir = -1.0 if obst[2] > obst[0] else 1.0
        angles = {name: AVOID_DEG * side_dir for name in WHEEL_NAMES}
        mode, throttle = "объезд", 0.3
    elif side:
        # все четыре колеса параллельно: машина едет вбок, курс не меняется
        steer = _clamp(math.degrees(math.atan2(-dy, max(dx, 0.03))), -90.0, 90.0)
        angles = {name: steer for name in WHEEL_NAMES}
        throttle = _throttle(0.50)
    elif dx < -0.05:
        # цель за спиной и разворачиваться не нужно — задним ходом
        angles = dict.fromkeys(WHEEL_NAMES, 0.0)
        mode, throttle = "назад", -_throttle(0.50)
    else:
        # ход к цели: колёса параллельно направлению на цель
        steer = _clamp(math.degrees(math.atan2(-dy, max(dx, 0.03))), -60.0, 60.0)
        angles = {name: steer for name in WHEEL_NAMES}
        throttle = _throttle(0.60)
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
    if abs(front - rear) > SPIN_DEG:
        mode = "разворот"
        angles = {"FL": front, "FR": front, "RL": rear, "RR": rear}
        mean_angle = (front - rear) / 2.0
    else:
        angles = dict.fromkeys(WHEEL_NAMES, mean)
        mean_angle = mean
        if abs(mean) > CRAB_DEG:
            mode = "краб"
        elif throttle < -0.05:
            mode = "назад"
        else:
            mode = "вперёд"
    scores = {"вперёд": float(y[5]), "краб": float(y[6]), "разворот": float(y[7])}
    word = steer_word(mean_angle)
    intent = max(scores, key=scores.get)
    pieces = [mode]
    if word != "прямо":
        pieces.append(word)
    speed = abs(throttle) * V_MAX
    label = " · ".join(pieces) + " · " + f"{speed:.1f} м/с".replace(".", ",")
    return {"angles": angles, "throttle": throttle, "mode": mode,
            "steer": word, "meanAngle": mean_angle, "intent": intent,
            "intents": {k: round(v, 3) for k, v in scores.items()}, "label": label}


def body_velocity(angles: dict, throttle: float) -> tuple:
    """Скорости в системе робота: (вперёд, вбок, угловая).

    Модель 4WIS: все колёса повёрнуты на средний угол — машина едет туда;
    если передние и задние смотрят в разные стороны — крутится вокруг центра.
    Положительный угол колеса — «вправо», поэтому боковая скорость и угловая
    берутся со знаком минус (ось y смотрит влево, как в ROS).
    """
    phi = sum(angles[name] for name in WHEEL_NAMES) / len(WHEEL_NAMES)
    spin = ((angles["FL"] + angles["FR"]) - (angles["RL"] + angles["RR"])) / 2.0
    speed = throttle * V_MAX
    sat = 1.0 - min(1.0, abs(spin) / ANGLE_LIMIT)      # на развороте машина не едет
    rad = math.radians(phi)
    vx = speed * math.cos(rad) * sat
    vy = -speed * math.sin(rad) * sat
    wz = -_clamp(spin / ANGLE_LIMIT, -1.0, 1.0) * W_MAX * max(0.35, min(1.0, abs(throttle)))
    return vx, vy, wz


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
    """Команды ИИ в ROS 2: ``/cmd_vel`` (geometry_msgs/Twist).

    Публикация включается только явно (``RC_AI_ROS=1``): по умолчанию стенд
    никому не командует. Если rclpy рядом нет — панель так и говорит, а стенд
    продолжает жить на своей модели.
    """

    topic = "/cmd_vel"

    def __init__(self, enabled: bool = False, topic: str = "/cmd_vel"):
        self.topic = topic
        self.enabled = bool(enabled)
        self.available = False
        self.reason = ""
        self.published = 0
        self.last_error = ""
        self.node = None
        self.publisher = None
        self._Twist = None
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
            "published": self.published,
            "reason": self.reason,
            "error": self.last_error,
        }
