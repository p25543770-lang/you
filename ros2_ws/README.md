# Рабочее пространство ROS 2: лидар робота Zapret

```
ros2_ws/
  src/zapret_lidar/        наш пакет: супервизор драйвера лидара
    zapret_lidar/
      drivers.py           таблица драйверов и план запуска (без ROS 2)
      frames.py            опознание лидара по байтам порта
      probe.py             быстрая проверка порта перед запуском драйвера
      supervisor.py        CLI: --print-plan / --list-drivers / --probe, вход узла
      node.py              узел lidar_supervisor (rclpy)
    launch/lidar.launch.py запуск всего лидара одной командой
    config/lidar.yaml      параметры супервизора
```

Зачем это нужно: раньше приложение само читало USB-порт лидара и подбирало
протокол, скорость и линию включения мотора. На живом роботе этот подбор
постоянно спорил с источником данных шасси за те же порты, а лидар TurtleBot3
(LDS-01), подключённый к плате OpenCR, вообще не виден как USB-устройство —
поэтому скана не было ни на пульте, ни на бортовом экране. Теперь лидар отдаёт
**ROS 2-пакет драйвера** (как в документации производителя), а приложение
подписывается на `/scan`.

## Что делает наш пакет

`lidar_supervisor` — не драйвер, а «надсмотрщик»: драйверы лидаров уже есть в
ROS 2, но их больше десятка, и для конкретного железа подходит один.

1. Собирает план: какие драйверы вообще поставлены (ament-индекс), какие порты
   есть (`/dev/serial/by-id/*lidar*`, `/dev/ttyUSB*`, `/dev/ttyACM*` — без портов
   мастера руления, шасси и BMS).
2. При `model:=auto` быстро проверяет порты (`probe`) и ставит вперёд драйвер
   того лидара, чьи кадры видно на порту: LDS-01 (`0xFA`), LD06/LD19
   (`0x54 0x2C`), RPLIDAR (ответ на команду SCAN + ровный виток).
3. Запускает драйвер, ждёт `sensor_msgs/LaserScan` в `/scan` и следит за
   свежестью: если процесс упал или точки пропали — перезапускает, а после
   нескольких неудач переходит к следующему драйверу.
4. Публикует состояние в `/zapret/lidar/status` (JSON): состояние, драйвер, порт,
   точки, Гц, что уже пробовали, каких пакетов не хватает и что поставить.
   Веб-пульт показывает это в панели «Лидар» и в `GET /api/lidar` → `ros`.

## Установка и запуск на роботе

Руками — ничего: ярлык «Запустить робот» (`scripts/start_robot.sh`) сам
докачивает ROS 2 и драйверы, компилирует это рабочее пространство и запускает
супервизор. То же по шагам:

```bash
./scripts/ros2_setup.sh         # поставит ROS 2 и драйверы (apt или исходники), rosdep, colcon build
./scripts/ros2_lidar.sh         # запустит супервизор (модель и порт — из .env); сам пересоберёт ws,
                                # если исходники изменились
./scripts/ros2_lidar.sh --status   # что видно: топик /scan, частоту, статус
./scripts/ros2_setup.sh --status   # как прошла последняя установка/сборка (журнал var/ros2_setup.log)
```

Сборка повторяется, только когда исходники в `src/` изменились (отпечаток —
`install/.zapret_build_stamp`); `./scripts/ros2_setup.sh --rebuild` — с нуля.
Пакеты драйверов, склонированные из исходников, в git не попадают (`.gitignore`
игнорирует всё в `src/`, кроме `zapret_lidar`).

Вручную, без наших скриптов:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install            # из ros2_ws
source install/setup.bash
ros2 launch zapret_lidar lidar.launch.py model:=auto
```

Диагностика без ROS 2 (работает даже там, где ROS не поставлен):

```bash
python3 ros2_ws/src/zapret_lidar/zapret_lidar/supervisor.py --print-plan
python3 ros2_ws/src/zapret_lidar/zapret_lidar/supervisor.py --list-drivers
.venv/bin/python -m zapret_lidar.probe --probe     # после сборки ws
```

## Поддерживаемые драйверы

| Лидар | Пакет ROS 2 | Узел | Порт/скорость |
| --- | --- | --- | --- |
| LDS-01/LDS-02 (TurtleBot3 Burger, HLS-LFCD2) | `hls_lfcd_lds_driver` | `hlds_laser_publisher` | `/dev/ttyUSB*`, 230400, кадры `0xFA` |
| RPLIDAR A1/A2M8 | `rplidar_ros` | `rplidar_composition` / `rplidar_node` | 115200 |
| RPLIDAR A2M7/A2M12/A3/S1 | `rplidar_ros` | то же | 256000 |
| RPLIDAR C1 | `rplidar_ros` | то же | 460800 |
| RPLIDAR (альтернатива) | `sllidar_ros2` | `sllidar_node` | по модели |
| LD06/LD19 | `ldlidar_stl_ros2` | `ldlidar_stl_ros2_node` | 230400, кадры `0x54 0x2C` |
| STL-27L | `ldlidar_stl_ros2` | `ldlidar_stl_ros2_node` | 921600 |
| LDS-01 через плату OpenCR (заводская сборка Burger) | `turtlebot3_node` | `turtlebot3_ros` | `/dev/ttyACM0` (OpenCR), параметры — `param/burger.yaml` |

`ldlidar_stl_ros2` и `sllidar_ros2` не публикуются в apt: `scripts/ros2_setup.sh`
клонирует их в `ros2_ws/src` и собирает вместе с нашим пакетом (у
`ldlidar_stl_ros2` заодно правится известная ошибка сборки на Ubuntu 24.04+ —
нет `#include <pthread.h>`). Драйверы из apt (`hls_lfcd_lds_driver`,
`rplidar_ros`, `turtlebot3_node`), если для дистрибутива пакета нет или он не
ставится, тоже клонируются и компилируются здесь; `sllidar_ros2` — по
`RC_ROS_DRIVERS=sllidar_ros2` или модели `sllidar`, `turtlebot3_node` из
исходников — только при модели `opencr`.

Если драйвер производителя уже запускается сам (например, через свой
`turtlebot3_bringup` или чужой launch), наш супервизор не нужен вовсе: веб-
приложение читает `/scan` напрямую. Достаточно, чтобы в `.env` стоял
`RC_LIDAR_SOURCE=ros` (или `auto`).
