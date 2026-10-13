#!/usr/bin/env python3
"""Экран робота (киоск) одной командой.

    ./run_kiosk.py                 # сервер на 8090 + экран в браузере (если есть дисплей)
    ./run_kiosk.py --no-open       # только сервер (для systemd/без дисплея)
    ./run_kiosk.py --source sim    # стенд: данные симуляции

Отдаёт экран киоска: публичная телеметрия, а поворот рулевых модулей — после PIN
и только с ``--source master``. Порт по умолчанию — 8090 (``RC_KIOSK_PORT``).
Подробности — ``--help``.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.kiosk_app import main

if __name__ == "__main__":
    sys.exit(main())
