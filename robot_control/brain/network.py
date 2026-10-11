"""ИИ-контроллер: рекуррентная сеть с утечкой на 32–48 нейронах (чистый Python).

Вход (20): 16 лучей лидара (нормированы на 4 м), синус и косинус
относительного курса на цель, расстояние до цели, текущая скорость.
Скрытый слой: N нейронов (по умолчанию 40), состояние переносится между шагами:
    h ← (1−α)·h + α·tanh(W_in·x + W_rec·h + b)
Выход (2): tanh → скорость вперёд (0…V_MAX) и угол руля (±30°).
Все веса — один плоский список; его подбирает train.py эволюционной стратегией.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from .world import LIDAR_BEAMS, LIDAR_RANGE_M, STEER_LIMIT_DEG, V_MAX

N_INPUTS = LIDAR_BEAMS + 4
N_OUTPUTS = 2
MIN_NEURONS, MAX_NEURONS = 32, 48
DEFAULT_NEURONS = 40
ALPHA = 0.5                       # скорость утечки (доля обновления за шаг)
WEIGHTS_FILE = Path(__file__).with_name("weights.json")


def param_count(n_hidden: int) -> int:
    return n_hidden * N_INPUTS + n_hidden * n_hidden + N_OUTPUTS * n_hidden + n_hidden + N_OUTPUTS


def features(lidar, rel_bearing: float, distance: float, speed: float) -> list[float]:
    x = [min(r, LIDAR_RANGE_M) / LIDAR_RANGE_M for r in lidar]
    x += [math.sin(rel_bearing), math.cos(rel_bearing)]
    x += [min(distance, 10.0) / 10.0, speed / V_MAX]
    return x


class Brain:
    def __init__(self, n_hidden: int = DEFAULT_NEURONS, weights: list[float] | None = None):
        if not MIN_NEURONS <= n_hidden <= MAX_NEURONS:
            raise ValueError(f"число нейронов {n_hidden} вне диапазона {MIN_NEURONS}–{MAX_NEURONS}")
        self.n = n_hidden
        self.h = [0.0] * n_hidden
        self.out = [0.0] * N_OUTPUTS
        if weights is None:
            weights = [0.0] * param_count(n_hidden)
        self.set_weights(weights)

    def set_weights(self, flat: list[float]) -> None:
        need = param_count(self.n)
        if len(flat) != need:
            raise ValueError(f"ожидалось {need} весов, получено {len(flat)}")
        n = self.n
        i = 0
        self.w_in = [flat[i + k * N_INPUTS:i + (k + 1) * N_INPUTS] for k in range(n)]
        i += n * N_INPUTS
        self.w_rec = [flat[i + k * n:i + (k + 1) * n] for k in range(n)]
        i += n * n
        self.w_out = [flat[i + k * n:i + (k + 1) * n] for k in range(N_OUTPUTS)]
        i += N_OUTPUTS * n
        self.b_h = flat[i:i + n]
        i += n
        self.b_out = flat[i:i + N_OUTPUTS]

    def reset(self) -> None:
        self.h = [0.0] * self.n

    def step(self, x: list[float]) -> tuple[float, float]:
        """Один шаг сети. Возвращает (скорость 0…V_MAX, руль ±30°)."""
        h_new = []
        for k in range(self.n):
            s = self.b_h[k]
            row_in = self.w_in[k]
            for j in range(N_INPUTS):
                s += row_in[j] * x[j]
            row_rec = self.w_rec[k]
            for j in range(self.n):
                s += row_rec[j] * self.h[j]
            h_new.append((1.0 - ALPHA) * self.h[k] + ALPHA * math.tanh(s))
        self.h = h_new
        o = []
        for m in range(N_OUTPUTS):
            s = self.b_out[m] + sum(self.w_out[m][j] * self.h[j] for j in range(self.n))
            o.append(math.tanh(s))
        self.out = o
        speed = (o[0] + 1.0) / 2.0 * V_MAX
        steer = o[1] * STEER_LIMIT_DEG
        return speed, steer


def load_weights(path: Path = WEIGHTS_FILE) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data


def reflex_prior(n_hidden: int = DEFAULT_NEURONS) -> list[float]:
    """Стартовые веса: часть нейронов реализует базовые рефлексы.

    Группы (нейроны 0…n-1):
      0–3   «свободно слева»  → руль влево  (лучи 1…7, весом +)
      4–7   «свободно справа» → руль вправо (лучи 9…15)
      8–15  «свободно впереди» → скорость    (лучи 15, 0, 1)
      16–23 «цель слева»      → руль влево  (вход sin курса на цель)
      остальные — малые случайные веса, их дообучает ES.
    Так сеть стартует с разумного поведения, а не с шума.
    """
    import random

    rng = random.Random(0)
    w_in = [[rng.gauss(0.0, 0.05) for _ in range(N_INPUTS)] for _ in range(n_hidden)]
    w_rec = [[rng.gauss(0.0, 0.02) for _ in range(n_hidden)] for _ in range(n_hidden)]
    w_out = [[0.0] * n_hidden for _ in range(N_OUTPUTS)]
    b_h = [0.0] * n_hidden
    b_out = [0.5, 0.0]                 # крейсерская скорость ~0.5 м/с при «всё свободно»

    left_beams = range(1, 8)
    right_beams = range(9, 16)
    front_beams = (15, 0, 1)
    for k in range(0, 4):
        for i in left_beams:
            w_in[k][i] = 3.0 / 7.0
        b_h[k] = -2.1
        w_out[1][k] = 0.5
    for k in range(4, 8):
        for i in right_beams:
            w_in[k][i] = 3.0 / 7.0
        b_h[k] = -2.1
        w_out[1][k] = -0.5
    for k in range(8, 16):
        for i in front_beams:
            w_in[k][i] = 1.0
        b_h[k] = -1.0
        w_out[0][k] = 0.25
    for k in range(16, 24):
        w_in[k][LIDAR_BEAMS] = 2.0            # sin(курс на цель)
        w_out[1][k] = 0.6 / 8.0 * 4.0
    flat = []
    for row in w_in:
        flat += row
    for row in w_rec:
        flat += row
    for row in w_out:
        flat += row
    flat += b_h + b_out
    return flat
