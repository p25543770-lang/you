# Copyright 2026 Zapret robot control
"""Пакет ROS 2 ``zapret_lidar``: лидар для робота Zapret.

Модули:

  * :mod:`zapret_lidar.drivers` — таблица драйверов и план запуска (без ROS 2);
  * :mod:`zapret_lidar.frames` — опознание протокола лидара по байтам;
  * :mod:`zapret_lidar.probe` — проверка портов перед запуском драйвера;
  * :mod:`zapret_lidar.supervisor` — CLI (``--print-plan``, ``--list-drivers``,
    ``--probe``) и точка входа узла;
  * :mod:`zapret_lidar.node` — узел ``lidar_supervisor`` (нужен rclpy).
"""

__version__ = "1.0.0"
