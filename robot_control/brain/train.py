"""Обучение весов ИИ-контроллера эволюционной стратегией (чистый Python).

    python3 -m robot_control.brain.train                 # 40 нейронов, ~5 мин
    python3 -m robot_control.brain.train --neurons 48 --gens 120

Алгоритм: (μ, λ)-ES с усреднением элиты и затуханием шага. На каждом
поколении все особи проходят одну и ту же случайную раскладку склада
(seed меняется от поколения к поколению), поэтому сеть учится не одной
трассе, а общему поведению: объезд стеллажей и дойти до точек.
В конце — проверка на раскладках, которых не было при обучении.
"""

from __future__ import annotations

import argparse
import json
import random
import statistics
import time
from multiprocessing import Pool
from pathlib import Path

from .episode import run_episode
from .network import DEFAULT_NEURONS, N_INPUTS, WEIGHTS_FILE, Brain, param_count, reflex_prior


def _eval(args):
    """Оценка особи на двух раскладках: базовый склад и случайная (меньше шума)."""
    weights, n, seed = args
    brain = Brain(n, weights)
    total = 0.0
    for s in (None, seed):
        total += run_episode(brain, s, noise_seed=s or 0).score
    return total / 2.0


def _evaluate_many(pool, n, population, seed):
    jobs = [(w, n, seed) for w in population]
    return pool.map(_eval, jobs, chunksize=2)


def validate(n: int, weights, seeds) -> dict:
    """Проверка на незнакомых раскладках: доля миссий и средние показатели."""
    brain = Brain(n, weights)
    results = [run_episode(brain, s, noise_seed=s or 0) for s in seeds]
    full = sum(1 for r in results if r.reached == 3 and r.collisions == 0)
    return {
        "layouts": len(results),
        "missionsFullyDone": full,
        "avgGoalsReached": round(statistics.mean(r.reached for r in results), 2),
        "avgCollisions": round(statistics.mean(r.collisions for r in results), 2),
        "avgScore": round(statistics.mean(r.score for r in results), 1),
    }


def train(n: int, gens: int, pop: int, mu: int, seed: int, out: str,
          init: str | None = None, sigma0: float = 0.15) -> dict:
    rng = random.Random(seed)
    dim = param_count(n)
    # старт от рефлексов (network.reflex_prior), ES дообучает всю сеть, включая память
    if init:
        center = list(json.loads(Path(init).read_text(encoding="utf-8"))["weights"])
    else:
        center = reflex_prior(n)
    sigma1 = 0.02
    t0 = time.time()
    with Pool() as pool:
        for g in range(gens):
            sigma = sigma0 * (sigma1 / sigma0) ** (g / max(1, gens - 1))
            # случайная раскладка меняется от поколения к поколению
            layout_seed = 100 + g // 2  # _eval сам добавляет базовый склад
            pop_w = []
            noise = []
            for _ in range(pop):
                eps = [rng.gauss(0.0, 1.0) for _ in range(dim)]
                noise.append(eps)
                pop_w.append([c + sigma * e for c, e in zip(center, eps)])
            scores = _evaluate_many(pool, n, pop_w, layout_seed)
            order = sorted(range(pop), key=lambda i: scores[i], reverse=True)
            elite = order[:mu]
            center = [sum(pop_w[i][k] for i in elite) / mu for k in range(dim)]
            if g % 5 == 0 or g == gens - 1:
                print(f"gen {g:3d}  σ={sigma:.3f}  best={scores[order[0]]:7.1f}  "
                      f"mean={statistics.mean(scores):7.1f}  {time.time() - t0:5.0f}s",
                      flush=True)
    stats = validate(n, center, seeds=list(range(9000, 9020)))
    stats["baseLayout"] = validate(n, center, seeds=[None])
    print("проверка на 20 незнакомых раскладках:", json.dumps(stats, ensure_ascii=False))
    payload = {
        "neurons": n,
        "inputs": N_INPUTS,
        "generations": gens,
        "population": pop,
        "seed": seed,
        "validation": stats,
        "weights": [round(w, 5) for w in center],
    }
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)
    print("веса сохранены:", out)
    return payload


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--neurons", type=int, default=DEFAULT_NEURONS)
    ap.add_argument("--gens", type=int, default=60)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--mu", type=int, default=6)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=str(WEIGHTS_FILE))
    ap.add_argument("--init", default=None, help="продолжить обучение с весов этого файла")
    ap.add_argument("--sigma0", type=float, default=0.15)
    a = ap.parse_args()
    train(a.neurons, a.gens, a.pop, a.mu, a.seed, a.out, init=a.init, sigma0=a.sigma0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
