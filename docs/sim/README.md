# Симуляция: робот 9:16 едет по лидару

- `seed1.html` — открыть в браузере (без сервера): склад, робот 0.45 × 0.80 м с хитбоксом
  (красный при касании), оценка позы (зелёный пунктир), лучи лидара, план, касания (×).
- `seed1.json` — данные миссии (кадры, пути, раскладка, касания).

Пересборка:
```
.venv/bin/python -c "import sys,json; sys.path.insert(0,'.'); from robot_control.brain.rect_sim import run_rect_mission as R; ..."
.venv/bin/python scripts/render_sim.py docs/sim/seed1.json docs/sim/seed1.html
```

Итог seed 1: все 8 точек пройдены, касаний 13, время 270 с симуляции.
Ошибка оценки позы — см. поля `loc_err_mean` и `loc_err_max` в JSON.
Проверка: JS-вьюер проверен на синтаксис (node), визуально — через matplotlib-превью тех же данных.
