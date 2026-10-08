"""Замер отклика: сеть, один шаг обучения и полный такт стенда.

ПК робота — i5-13400 / 16 ГБ (заказчик назвал бюджет отклика 5 мс). Здесь
меряем на этой машине и печатаем, сколько остаётся на сети при таком бюджете.
"""
import statistics
import sys
import time

sys.path.insert(0, ".")
import slam_gui.ai_driver as A
import slam_gui.backend as B


def ms(values):
    return [v * 1e3 for v in values]


def report(title, samples):
    m = ms(samples)
    print("%-34s сред. %7.3f  медиана %7.3f  p95 %7.3f  макс %7.3f мс" % (
        title, statistics.mean(m), statistics.median(m),
        sorted(m)[int(len(m) * 0.95)], max(m)))


# --- 1. Сеть сама по себе ----------------------------------------------------
net = A.NeuralDriver()
x = [0.4, -0.2, 0.3, 0.0, 0.12, 0.4, 0.2, 0.72, 0.0, 0.5]
target = A.geometric_target(1.2, -0.4, 0.3, crab=False, obst=(0.1, 0.4, 0.2))["vector"]

for _ in range(200):
    net.forward(x)
n = 20000
t = []
for _ in range(n):
    t0 = time.perf_counter()
    net.forward(x)
    t.append(time.perf_counter() - t0)
report("сеть: прямой проход (32 нейрона)", t)

t = []
for _ in range(20000):
    t0 = time.perf_counter()
    net.step(x, target)
    t.append(time.perf_counter() - t0)
report("сеть: такт с обучением", t)

# --- 2. Урок перед выездом из кэша и с нуля ---------------------------------
t0 = time.perf_counter()
A._WARM_CACHE.clear()
fresh = A.NeuralDriver()
loss = fresh.pretrain()
print("урок перед выездом: %.1f мс, ошибка %.5f" % ((time.perf_counter() - t0) * 1e3, loss))

t0 = time.perf_counter()
cached = A.NeuralDriver()
cached.pretrain()
print("тот же урок из кэша: %.2f мс" % ((time.perf_counter() - t0) * 1e3))

# --- 3. Полный такт стенда --------------------------------------------------
clock = {"t": 1000.0}
B.time.time = lambda: clock["t"]
src = B.SimSource()
for _ in range(300):
    src.read()
    clock["t"] += 1.0 / 60.0

ticks, rays = [], []
for _ in range(3000):
    clock["t"] += 1.0 / 60.0
    t0 = time.perf_counter()
    src.read()
    ticks.append(time.perf_counter() - t0)
    t0 = time.perf_counter()
    src._obstacles()
    rays.append(time.perf_counter() - t0)
report("такт стенда (сеть + карта + ROS)", ticks)
report("из них: три луча дальномера", rays)
m = ms(ticks)
print("бюджет 5 мс: средний такт оставляет %.2f мс запаса, макс %.2f мс" % (
    5.0 - statistics.mean(m), 5.0 - max(m)))
print("нейронов: %d (%s) · шагов обучения: %d · ошибка: %.5f" % (
    src.driver.neurons, " → ".join(map(str, src.driver.layers)), src.driver.steps,
    src.driver.loss_avg or 0.0))
