"""Сервер для дисплея на борту робота (порт 8090 по умолчанию).

Отдаёт только экран киоска: АКБ + шасси + лидар. Без авторизации, без пульта.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.kiosk_app import main

if __name__ == "__main__":
    main()
