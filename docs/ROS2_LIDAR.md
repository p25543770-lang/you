# Лидар через ROS 2: установка, запуск, диагностика

Документ для робота: как лидар отдаёт скан **пакетом ROS 2**, а пульт (8080) и
бортовой экран (8090) читают его из топика `/scan`.

Коротко: **руками ничего ставить не нужно**. Ярлык «Запустить робот»
(`scripts/start_robot.sh`) и установщик (`scripts/install_ubuntu.sh`) сами
докачивают ROS 2, драйверы лидаров и компилируют `ros2_ws`, а потом запускают
драйвер лидара. В первый раз это несколько минут и один вопрос пароля sudo в
окне терминала, дальше — секунды.

```bash
./scripts/ros2_setup.sh                       # то же вручную: ROS 2 + драйверы + сборка ws
./scripts/ros2_setup.sh --status              # идёт ли установка и чем закончилась
./scripts/ros2_lidar.sh --status              # топик /scan, частота, состояние драйвера
./scripts/stop.sh && ./scripts/start_robot.sh # перезапустить всё (и лидар)
```

## Зачем это вместо чтения USB-порта приложением

Раньше приложение само открывало USB-порт лидара и подбирало протокол, скорость
и линию включения мотора. На живом роботе это давало три проблемы:

1. **Порт один, а читателей несколько.** Пульт и бортовой экран, источник данных
   шасси и сам лидар по очереди занимали один и тот же `/dev/ttyUSB*`; пока шёл
   подбор, скан пропадал, а в журнале появлялось «порт занят другой программой».
2. **LDS-01 через плату OpenCR** (заводская сборка TurtleBot3 Burger) вообще не
   виден как USB-устройство лидара: скан отдаёт плата робота.
3. **`rclpy` не виден из `.venv`.** ROS 2 ставится системно в `/opt/ros`, а
   приложение работает интерпретатором из `.venv` — без подготовки путей импорт
   `rclpy` падал, и ветка «взять скан из ROS 2» не включалась никогда, хотя
   `ros2 topic echo /scan` в терминале работал.

Теперь порт занимает **драйвер производителя** (пакет ROS 2), а приложение только
подписывается на топик: спорить за порт больше некому. Путь и переменные
окружения для `rclpy` добавляет `robot_control/ros_env.py`.

## Что стоит на роботе

| Лидар | Пакет ROS 2 | Узел | Откуда скан |
| --- | --- | --- | --- |
| LDS-01/LDS-02 (TurtleBot3 Burger, HLS-LFCD2) | `hls_lfcd_lds_driver` | `hlds_laser_publisher` | USB-переходник лидара, 230400 |
| RPLIDAR A1/A2M8 | `rplidar_ros` | `rplidar_composition` / `rplidar_node` | 115200 |
| RPLIDAR A2M7/A2M12/A3/S1 | `rplidar_ros` | то же | 256000 |
| RPLIDAR C1 | `rplidar_ros` | то же | 460800 |
| RPLIDAR (альтернатива) | `sllidar_ros2` | `sllidar_node` | по модели |
| LD06/LD19 | `ldlidar_stl_ros2` | `ldlidar_stl_ros2_node` | 230400 |
| STL-27L | `ldlidar_stl_ros2` | `ldlidar_stl_ros2_node` | 921600 |
| LDS-01 через плату OpenCR | `turtlebot3_node` | `turtlebot3_ros` | `/dev/ttyACM0`, `param/burger.yaml` |

Таблицу знает супервизор `zapret_lidar`: он и решает, какой пакет запускать.
`ldlidar_stl_ros2` и `sllidar_ros2` в apt не публикуются — `ros2_setup.sh`
клонирует их в `ros2_ws/src` и компилирует сам.

## Установка: всё само

```bash
./scripts/ros2_setup.sh                 # всё сразу: ROS 2, драйверы, rosdep, colcon build
./scripts/ros2_setup.sh --check --json  # что найдено (дистрибутив, rclpy, пакеты), ничего не ставит
./scripts/ros2_setup.sh --status        # идёт ли установка сейчас, последний результат, конец журнала
./scripts/ros2_setup.sh --no-install    # только собрать ros2_ws (без apt и sudo)
./scripts/ros2_setup.sh --rebuild       # пересобрать ros2_ws с нуля
./scripts/ros2_setup.sh --service       # ещё и служба robot-lidar (лидар поднимается до приложений)
```

Что делает `ros2_setup.sh` без флагов (каждый шаг — только если он нужен,
повторный запуск занимает секунды):

1. **ROS 2.** Ищет `/opt/ros/<дистрибутив>`. Нет — подключает официальный
   репозиторий (включает Ubuntu `universe`, ставит пакет `ros2-apt-source` с
   GitHub, как велит docs.ros.org; при неудаче — ключ `ros.key`) и ставит
   `ros-<дистрибутив>-ros-base` и `ros-dev-tools`. Дистрибутив — по
   `UBUNTU_CODENAME` (так верно и на Linux Mint/Pop!_OS): 22.04 → `humble`,
   24.04 → `jazzy`, 26.04 → `lyrical`; вручную — `--distro`. apt ждёт чужую
   блокировку до 10 минут, никогда не удаляет пакеты (`--no-remove`), а при
   сбое доделывает `dpkg --configure -a`, обновляет систему и повторяет. На
   Ubuntu 24.04 добавляет `noble-updates`/`noble-backports` в `ubuntu.sources`,
   если их нет (без них `ros-dev-tools` не ставится — так в официальной инструкции).
2. **Где готовых пакетов ROS 2 нет** (Debian 12/13, Raspberry Pi OS, не-LTS
   Ubuntu вроде 25.04/25.10, 32-битный ARM) — ROS 2 скачивается исходниками с
   GitHub (`vcs import`) и **компилируется сам** (`colcon build` только нужных
   пакетов: rclpy, rclcpp, сообщения, ros2 launch/run/topic — без RViz и демо)
   в `/opt/ros/<дистрибутив>`. С ярлыка это идёт в фоне (`--background-build`):
   робот работает, лидар пока по USB; ход — `./scripts/ros2_setup.sh --status`.
   Повторный запуск продолжает с того же места.
3. **Драйверы лидаров**: `hls_lfcd_lds_driver`, `rplidar_ros`, `turtlebot3_node`
   — из apt; чего в apt нет или что не поставилось — клонируется в `ros2_ws/src`
   (нужная ветка под дистрибутив) и компилируется. Известные ошибки сборки
   правятся сами (например, `ldlidar_stl_ros2` без `#include <pthread.h>` не
   компилируется на Ubuntu 24.04+). Свой набор — `RC_ROS_DRIVERS` в `.env`.
4. **rosdep** ставит зависимости исходников, **colcon** компилирует весь
   `ros2_ws` (`--symlink-install --continue-on-error`, потоки — по памяти, чтобы
   сборка не вешала машину с 2–4 ГБ). Отпечаток исходников хранится в
   `ros2_ws/install/.zapret_build_stamp`: пересборка — только если они изменились.
   Драйвер, который не скомпилировался, не мешает остальным.
5. **`.env`**: `RC_LIDAR_SOURCE=ros`, когда `rclpy`, сборка и хотя бы один
   драйвер проверены (выбранный оператором `serial` не трогается).

Пароль sudo спрашивается **один раз** и только если что-то ставится. Без
терминала (служба systemd, автозапуск сеанса) пароль спросить негде: скрипт
ничего не ломает, выходит с кодом 3, а панель «Лидар» показывает «нужен пароль
администратора — запусти ярлык «Запустить робот»». Журнал всех шагов —
`var/ros2_setup.log`, состояние для панели — `var/ros2_setup.json`.

Кто вызывает установку сам:

| Где | Что происходит |
| --- | --- |
| ярлык «Запустить робот» (`start_robot.sh`) | `ros2_setup.sh --background-build`, затем супервизор лидара в фоне (`var/lidar.pid`, журнал `var/lidar.log`) — до запуска пульта и экрана; если пульт и экран работают службами systemd, после первой установки ROS 2 они перезапускаются, чтобы увидеть `rclpy` |
| `install_ubuntu.sh` | ROS 2 по умолчанию (`--no-ros2` — не ставить и запомнить `RC_ROS_AUTO=0`) |
| `ros2_lidar.sh` / служба `robot-lidar` | нет ROS 2 — ставит; изменились исходники `ros2_ws` — пересобирает |
| «Обновить с GitHub» в панели | после обновления докомпилирует `ros2_ws` (без sudo) и перезапускает супервизор лидара с новым кодом |

Выключить автоустановку: `RC_ROS_AUTO=0` в `.env` (или `RC_LIDAR_SOURCE=serial`
— тогда ROS 2 не нужен вовсе).

`--service` ставит юнит `robot-lidar.service` с `Before=robot-control.service`:
драйвер лидара поднимается раньше приложений, и они сразу читают `/scan`.
`./scripts/autostart.sh off` гасит и эту службу.

### Почему раньше «не ставилось и не компилировалось»

- у `scripts/ros2_setup.sh` и `scripts/ros2_lidar.sh` в git не было бита
  исполнения: `./scripts/ros2_setup.sh` давал «Permission denied», а служба
  `robot-lidar` — `203/EXEC`;
- без `--install` ROS 2 не ставился вовсе, драйверы — только с `--drivers`,
  `ldlidar_stl_ros2` — только с `--ldlidar`; установщик ставил ROS 2 лишь с
  `--ros2`, а ярлык и обновление ROS 2 не трогали;
- сборка шла `--packages-select zapret_lidar` (драйверы из исходников не
  компилировались никогда) и обрывалась на `source setup.bash` под `set -u`
  (скрипты colcon читают незаданный `$COLCON_TRACE`);
- `LD_LIBRARY_PATH` выставлялся изнутри уже работающего приложения, а загрузчик
  читает его только при старте процесса — `rclpy` не загружался. Теперь
  `run.py`/`run_kiosk.py` перезапускаются один раз с окружением `source setup.bash`;
- мост в приложении создавал узел ROS 2 до `rclpy.init()` — подписка на `/scan`
  падала («context is invalid») даже при работающем ROS 2;
- в `ros_env` пакет `sllidar_ros2` числился в apt, хотя в rosdistro его нет, а
  список дистрибутивов не знал `lyrical` (Ubuntu 26.04).

## Запуск и проверка

```bash
./scripts/ros2_lidar.sh                        # модель и порт — из .env
./scripts/ros2_lidar.sh --model lds01 --port /dev/ttyUSB0
./scripts/ros2_lidar.sh --status               # список топиков, состояние, частота /scan
./scripts/ros2_lidar.sh --print                # показать команду, ничего не запуская
```

Вручную, «как в документации производителя»:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch zapret_lidar lidar.launch.py model:=auto
```

Диагностика без ROS 2 (план запуска считается чистой стандартной библиотекой):

```bash
python3 ros2_ws/src/zapret_lidar/zapret_lidar/supervisor.py --print-plan
python3 ros2_ws/src/zapret_lidar/zapret_lidar/supervisor.py --list-drivers
```

## Как супервизор выбирает драйвер

1. Собирает план: какие пакеты драйверов реально установлены (ament-индекс),
   какие есть порты (`/dev/serial/by-id/*lidar*`, `/dev/ttyUSB*`, `/dev/ttyACM*`;
   порты мастера руления, шасси и BMS исключены).
2. При `model:=auto` быстро проверяет порты и ставит вперёд драйвер того лидара,
   чьи кадры видны: LDS-01 (`0xFA`, 42 байта), LD06/LD19 (`0x54 0x2C`), RPLIDAR
   (ответ `A5 5A` на команду SCAN и ровный виток точек).
3. Запускает драйвер напрямую из `lib/<package>/` (или `ros2 run`), ждёт
   `sensor_msgs/LaserScan` до `probe_timeout` (8 с) и следит за свежестью:
   процесс упал или точки пропали — перезапуск, потом следующий драйвер из плана.
4. Публикует состояние в `/zapret/lidar/status` (JSON). Оно попадает в
   `GET /api/lidar` → `ros.supervisor` и в панель «Лидар» обоих экранов:
   драйвер, порт, точки, Гц, что уже пробовали и чего не хватает.

Флаги и параметры — в `ros2_ws/src/zapret_lidar/config/lidar.yaml`
(`model`, `port`, `baud`, `scan_topic`, `status_topic`, `probe_timeout`,
`scan_timeout`, `autostart`, `autoprobe`).

## Настройки `.env`

| Переменная | Значение | Смысл |
| --- | --- | --- |
| `RC_LIDAR_SOURCE` | `auto` | `auto` — ROS 2, если он есть, иначе USB; `ros` — только ROS 2; `serial` — только USB (приложением) |
| `RC_LIDAR_ROS_TOPIC` | `/scan` | топик скана, который слушают оба экрана |
| `RC_LIDAR_PROTOCOL` | `auto` | в ROS 2-режиме это модель лидара: `lds01`, `rplidar`, `ld06`, `turtlebot3`… — супервизор запускает соответствующий пакет |
| `RC_LIDAR_PORT` | пусто | порт для драйвера; пусто — автопоиск |
| `RC_LIDAR_BAUD` | пусто | скорость порта; пусто — по умолчанию драйвера |
| `RC_LIDAR_ROS_STATUS_TOPIC` | `/zapret/lidar/status` | топик состояния супервизора |
| `RC_ROS_PREFIX` | `/opt/ros/<дистрибутив>` | где искать ROS 2, если он не в `/opt/ros` |
| `RC_ROS_AUTO` | `1` | `0` — ярлык и `ros2_lidar.sh` не ставят и не собирают ROS 2 сами |
| `RC_ROS_DRIVERS` | пусто | свой набор драйверов вместо `hls_lfcd_lds_driver rplidar_ros ldlidar_stl_ros2 turtlebot3_node` |
| `RC_ROS_SOURCE_BUILD` | `auto` | сборка самого ROS 2 из исходников: `auto` — где нет пакетов, `0` — никогда, `1` — всегда |
| `RC_ROS_BUILD_JOBS` | по памяти | сколько потоков компилятора давать colcon |
| `RC_ROS_APT_MIRROR` | пусто | зеркало вместо `http://packages.ros.org/ros2/ubuntu` |
| `RC_ROS_REEXEC` | `1` | `0` — не перезапускать приложения с окружением ROS 2 |

Все они доступны и из панели «Лидар» (веб-пульт — после входа, бортовой экран —
без PIN): в списке «Откуда берётся скан» есть `auto`, `ros` и `serial`.

## Если скана нет

| Что видно | Что делать |
| --- | --- |
| `ROS 2 не установлен (нет /opt/ros/…)` | запустить ярлык «Запустить робот» (или `./scripts/ros2_setup.sh`) — всё докачается и скомпилируется само |
| `… сейчас идёт автоустановка ROS 2: …` | подождать; ход — `./scripts/ros2_setup.sh --status` |
| `… нужен пароль администратора …` | ярлык «Запустить робот» открывает терминал — там ввести пароль sudo один раз |
| `ROS 2 … есть, но это приложение запущено без его окружения` | ROS 2 поставили после запуска: `./scripts/stop.sh` и снова ярлык |
| `rclpy не импортируется из /opt/ros/…` | `./scripts/ros2_setup.sh --check`; Python в `.venv` должен совпадать с Python ROS 2 |
| `ROS 2 есть, но /scan никто не публикует — драйвер лидара не запущен` | ярлык запускает супервизор сам; вручную — `./scripts/ros2_lidar.sh` (или служба: `./scripts/ros2_setup.sh --service`) |
| `нет ROS 2-пакетов драйверов лидара: …` | `./scripts/ros2_setup.sh` (поставит из apt или скомпилирует из исходников) |
| `ни один драйвер не отдал /scan (rplidar@/dev/ttyUSB0 — нет /scan за 8 с)` | проверить порт и права: `./scripts/lidar_check.py --ros`, `sudo usermod -aG dialout $USER`, перезайти |
| `драйвер встал: нет точек 3.0 с` | драйвер перезапустится сам; если повторяется — смотрите `journalctl -u robot-lidar -f` |
| `на портах … не видно ни одного известного лидара` | лидар подключён к плате робота (OpenCR): задайте `--model opencr` или `RC_LIDAR_PROTOCOL=turtlebot3` |
| скан идёт, но хочется старый путь | `RC_LIDAR_SOURCE=serial` (только USB) — прежнее чтение порта приложением целиком сохранено |

Проверка «глазами ROS 2»:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic list | grep -E 'scan|zapret'
ros2 topic hz /scan
ros2 topic echo --once /zapret/lidar/status
```

Проверка со стороны приложения:

```bash
.venv/bin/python scripts/lidar_check.py --ros         # дистрибутив, rclpy, пакеты драйверов
.venv/bin/python scripts/lidar_web_check.py           # дошёл ли скан до обоих экранов
curl -s http://127.0.0.1:8080/api/lidar | python3 -m json.tool | head -40
```

## Что осталось на прежнем пути

Чтение USB-порта самим приложением (`RC_LIDAR_SOURCE=serial` и `auto` без ROS 2)
никуда не делось: на стенде без ROS 2 и при отладке оно работает как раньше —
перебор портов, протоколов и скоростей, межпроцессный замок порта, обмен сканом
между пультом и бортом по HTTP (`peer`). ROS 2-путь просто стоит первым, когда
ROS 2 в системе есть.
