# Установка на машину с Ubuntu

Инструкция для ноутбука/ПК на Ubuntu, который будет бортовым компьютером робота.

## Быстрая установка — рекомендуемый путь

```bash
git clone https://github.com/p25543770-lang/you.git robot-control
cd robot-control
./scripts/install_ubuntu.sh
```

Установщик создаёт окружение, `.env`, оператора и ровно один ярлык
**«Запустить робот»**. Он сразу запускает веб-пульт (порт 8080) и экран робота
(8090); при следующих запусках используется тот же ярлык. Для запуска вручную
из терминала: `./scripts/start_robot.sh`; для остановки: `./scripts/stop.sh`.
По умолчанию источник — USB-мастер `RC_SLAM_SOURCE=master`, порт
`/dev/ttyACM0`; поменяйте `RC_SLAM_PORT` в `.env`, если Linux назначил другой.
ROS 2 для лидара ставится **сам** — флаги не нужны: установщик (и потом каждый
запуск ярлыком) докачивает ROS 2 и пакеты драйверов лидара, компилирует
`ros2_ws` и включает чтение скана из `/scan` (см. раздел 4b). Не нужен ROS 2 —
`./scripts/install_ubuntu.sh --no-ros2`.
Если установщик добавил пользователя в `dialout`, выйдите из Ubuntu и войдите
снова, чтобы группа USB-портов вступила в силу.

Дальнейшие разделы — для ручной настройки и отладки.

## 1. Зависимости

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip network-manager iw
```

## 2. Код и окружение

```bash
sudo useradd --system --home /opt/robot-control --shell /usr/sbin/nologin robotctl
sudo mkdir -p /opt/robot-control
sudo cp -r . /opt/robot-control
cd /opt/robot-control
sudo ./scripts/ensure_deps.sh          # в каталоге, принадлежащем root
```

В своей домашней копии (без `sudo`) скрипт запускается просто как
`./scripts/ensure_deps.sh` — `sudo` он зовёт сам, и только если понадобится
`apt install python3-venv`.

`ensure_deps.sh` сам создаёт `.venv`, ставит пакеты (при отсутствии сети — из
`vendor/`, где уже лежит pyserial) и **проверяет** результат: если в готовом
окружении нет `flask` или `serial`, ставит заново, а не рапортует «всё
установлено». Вручную `pip install pyserial` делать не нужно. Проверить себя:

```bash
.venv/bin/python -c "import flask, serial; print(flask.__version__, serial.VERSION)"
```

## 3. Настройки и учётка оператора

```bash
sudo cp .env.example .env
python3 -c "import secrets; print(secrets.token_urlsafe(48))"
sudo nano .env                 # вписать RC_SECRET_KEY
sudo .venv/bin/python scripts/create_operator.py --username operator
sudo chown -R robotctl:robotctl /opt/robot-control
sudo chmod 600 /opt/robot-control/.env
```

Проверьте, что `RC_PBKDF2_ITERATIONS` вам подходит: на слабом одноплатнике
можно снизить до 100000, ниже — не стоит.

## 4. Автозапуск (по желанию)

По умолчанию после загрузки Ubuntu приложение само не стартует. Ручной запуск
обоих интерфейсов — одним ярлыком **«Запустить робот»** или командой
`./scripts/start_robot.sh`.

Чтобы после загрузки стартовали оба сервера, а после входа в графический сеанс
открывались оба экрана, используйте установщик:

```bash
./scripts/install_ubuntu.sh --autostart
journalctl -u robot-control -f
journalctl -u robot-kiosk -f
```

Он включает `robot-control` (8080), `robot-kiosk` (8090) и одну запись
`~/.config/autostart/robot-control.desktop`. Systemd держит серверы, ярлык
рабочего стола открывает оба интерфейса и не дублирует процессы.

Выключить автозапуск обратно (службы, общий ярлык сеанса, точку доступа):

```bash
./scripts/autostart.sh status              # что поднимается само
./scripts/autostart.sh off                 # выключить автозапуск И остановить всё сейчас
./scripts/autostart.sh off --keep-running  # только выключить автозапуск
./scripts/autostart.sh on                  # вернуть автозапуск
```

Полностью погасить приложение, не трогая автозапуск, — `./scripts/stop.sh`
(серверы по pid-файлам, службы, окно киоска).

## 4b. Лидар: ROS 2 (рекомендуемый путь) и USB

Скан 360° отдаёт **драйвер ROS 2** (`hls_lfcd_lds_driver` для LDS-01/LDS-02 —
лидар TurtleBot3 Burger, `rplidar_ros` для RPLIDAR, `ldlidar_stl_ros2` для
LD06/LD19, `turtlebot3_node` — если лидар подключён к плате OpenCR). Пульт (8080)
и бортовой экран (8090) подписываются на `/scan`; порт занимает драйвер, поэтому
приложения больше не спорят за USB-устройство.

Ставить ничего не нужно: `install_ubuntu.sh` и ярлык «Запустить робот» сами
вызывают `./scripts/ros2_setup.sh`, а он подключает официальный репозиторий
ROS 2 (пакет `ros2-apt-source`), ставит `ros-<дистрибутив>-ros-base` и
`ros-dev-tools` (дистрибутив — по `UBUNTU_CODENAME`: 22.04 → `humble`,
24.04 → `jazzy`, 26.04 → `lyrical`; Linux Mint тоже), драйверы лидаров из apt,
а чего в apt нет (`ldlidar_stl_ros2` и др.) — клонирует и компилирует;
зависимости исходников ставит `rosdep`, рабочее пространство компилирует
`colcon`. Первый раз — несколько минут и один вопрос пароля sudo, потом —
секунды. На Debian/Raspberry Pi OS/не-LTS Ubuntu готовых пакетов ROS 2 нет —
он компилируется из исходников в фоне. Вручную и для проверки:

```bash
./scripts/ros2_setup.sh                       # всё само: ROS 2, драйверы, rosdep, colcon build
./scripts/ros2_setup.sh --status              # идёт ли установка и чем закончилась
./scripts/ros2_setup.sh --service             # служба robot-lidar: драйвер до приложений
./scripts/ros2_lidar.sh --status              # топик /scan, частота, состояние драйвера
.venv/bin/python scripts/lidar_check.py --ros # дистрибутив ROS 2, rclpy, пакеты драйверов
```

Почему это работает из `.venv`: ROS 2 стоит системно в `/opt/ros`, а приложения
работают интерпретатором из `.venv`. `run.py` и `run_kiosk.py` при старте один
раз перезапускаются с окружением `source /opt/ros/<дистрибутив>/setup.bash`
(`LD_LIBRARY_PATH` для `rclpy` загрузчик читает только при запуске процесса),
поэтому `import rclpy` в приложении работает без ручного `source`. Проверить:
`.venv/bin/python scripts/lidar_check.py --ros`.

Кто именно запустит драйвер, решает супервизор `zapret_lidar`
(`ros2_ws/src/zapret_lidar`): он перебирает только те пакеты, что реально
установлены, при `model:=auto` опознаёт лидар по кадрам на порту, следит за
свежестью `/scan` и публикует состояние в `/zapret/lidar/status` — оно видно в
панели «Лидар» обоих экранов и в `GET /api/lidar` → `ros.supervisor`. Полный
разбор — `docs/ROS2_LIDAR.md`.

Если ROS 2 не нужен (стенд, отладка), лидар по-прежнему читается напрямую:

```bash
RC_LIDAR_SOURCE=serial        # в .env: только USB-порт, без ROS 2
```

Тогда работает всё описанное ниже (перебор портов и протоколов самим приложением).

### Лидар на USB-порту (режим `serial`)

В режиме `serial` лидар читает само приложение (`robot_control/lidar.py`): порт,
протокол и скорость подбираются перебором, мотор RPLIDAR включается линией
USB-переходника.
Перебор идёт по **всем** USB-портам (`/dev/serial/by-id/*`, `/dev/ttyUSB*`,
`/dev/ttyACM*`), каждый круг начинается со следующего порта; порт из
`RC_LIDAR_PORT` — только первый в очереди, а не единственный. Ход перебора виден
под картой и в `GET /api/lidar` (`reason` и список `tried` по каждому порту).
Ничего запускать вручную не нужно, но пользователю сервиса нужен доступ к порту:

```bash
sudo usermod -aG dialout robotctl              # если сервис работает не от root
.venv/bin/python scripts/lidar_check.py        # что видно на портах
.venv/bin/python scripts/lidar_check.py --save-env   # записать найденное в .env
.venv/bin/python scripts/lidar_web_check.py    # скан дошёл до пульта и борта?
```

`--save-env` пишет найденные `RC_LIDAR_PORT` (постоянный путь
`/dev/serial/by-id/…`, если он есть), `RC_LIDAR_PROTOCOL`, `RC_LIDAR_BAUD` и
`RC_LIDAR_MOTOR`, не трогая остальные строки `.env`, и создаёт `RC_SECRET_KEY`,
если файла ещё не было. После этого приложения перезапускают
(`./scripts/stop.sh && ./scripts/start_robot.sh` либо
`sudo systemctl restart robot-control robot-kiosk`).

`lidar_web_check.py` — последняя проверка пути «порт → API → экран»: оба
приложения запущены, запущены интерпретатором с pyserial, и их `GET /api/lidar`
отдаёт свежий настоящий скан (не демо). Код возврата `0` — лидар виден на обоих
экранах, `1` — приложения работают, но скана нет (причина печатается), `2` —
приложение не запущено. Флаги: `--wait 20`, `--only console|kiosk`, `--json`.

Службы в `scripts/robot-control.service` и `scripts/robot-kiosk.service`
стартуют python'ом из `.venv` проекта (`/opt/robot-control/.venv/bin/python`) и
входят в группу `dialout`: в системном python3 нет ни Flask, ни pyserial, а без
`dialout` порт не открывается. `scripts/install_ubuntu.sh` подставляет в юниты
фактический путь к `.venv`.

Скорость зависит от модели: A1/A2M8 — 115200, A2M7/A2M12/A3/S1 — 256000,
C1 — 460800, LD06/LD19 — 230400. Лидар TurtleBot3 Burger (LDS-01 / LDS-02, он же
HLS-LFCD2) тоже идёт на 230400, но другими кадрами (`0xFA` по 42 байта): протокол
`lds01` определяется сам, вручную — `.venv/bin/python scripts/lidar_check.py
--protocol lds01`. Если такой лидар подключён к плате OpenCR (заводская сборка
Burger), напрямую по USB его не видно — скан должен отдавать ROS 2 (`/scan`) или
нужен отдельный USB-переходник. У RPLIDAR A3 на USB-адаптере есть переключатель
115200/256000 — он должен совпадать. Чтобы не тратить время на перебор, закрепите
порт в `.env`: `RC_LIDAR_PORT=/dev/ttyUSB1` (остальные USB всё равно будут
проверены, если здесь данных нет), при необходимости `RC_LIDAR_BAUD` и
`RC_LIDAR_MOTOR` (см. `GET /api/lidar`, поле `detected` — там найденные параметры).
Порты мастера и модулей шасси (`RC_SLAM_PORT`/`RC_SLAM_PORTS`) лидар не трогает —
если лидар подключён туда, уберите порт из этих переменных.

Постоянное имя порта (если лидаров на машине несколько):

```bash
sudo tee /etc/udev/rules.d/60-rplidar.rules >/dev/null <<'RULE'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
RULE
sudo udevadm control --reload && sudo udevadm trigger
```

После этого порт всегда `/dev/rplidar` — приложение находит его автоматически.

## 5. Бортовой экран и управление

Киоск на порту 8090 показывает телеметрию и функции без PIN; PIN запрашивается
только при открытии кейса. Руление и тяга киоска могут физически перемещать
робот без PIN — сначала проверяйте их с вывешенными колёсами и физическим
аварийным выключателем. Веб-пульт оператора сохраняет собственную авторизацию.
В обычном режиме киоск не запускают отдельно: единая команда
`./scripts/start_robot.sh` поднимает и пульт 8080, и киоск 8090. Установщик
создаёт один ярлык на рабочем столе; не создавайте отдельные ярлыки пульта и
киоска.

Для диагностики или запуска без рабочего стола доступны отдельные команды:

```bash
./scripts/start.sh              # только веб-пульт
./scripts/kiosk.sh --no-open    # только сервер киоска
./scripts/kiosk.sh              # киоск + открытие браузера
```

Systemd-вариант ставится через `./scripts/install_ubuntu.sh --autostart`;
службы не открывают браузер сами, общий ярлык автозапуска сеанса открывает оба
экрана после входа. Отключение: `./scripts/autostart.sh off`.

USB-порт лидара один, а приложений два, поэтому второй процесс порт **не
отбирает**: пульт (8080) держит лидар, а бортовой экран берёт готовый виток у
него по HTTP (`RC_LIDAR_PEER_URLS`, свой адрес из списка исключается) — и
наоборот, если первым поднялся экран. На карте это подписано как
`скан от пульта`, в `GET /api/lidar` — `source: peer`; это штатный режим, а не
ошибка. Если реального скана нет, экран показывает демо-скан источника данных с
подписью `демо · N точек (не лидар)` и причиной под картой — за работающий
лидар он не выдаётся.

Проверка: `curl http://127.0.0.1:8090/healthz` → `{"mode":"kiosk","ok":true,…}`.
Остановить всё запущенное — `./scripts/stop.sh` (пульт, экран, службы, окно
киоска). Выключить ещё и автозапуск, чтобы робот больше не поднимал приложение
сам, — `./scripts/autostart.sh off`.

Служба лидара: `sudo systemctl status robot-lidar`, журнал —
`journalctl -u robot-lidar -f`. Если лидар не нужен вовсе (или подключён не к
ROS 2-драйверу), службу можно не ставить, а лидар читать напрямую
(`RC_LIDAR_SOURCE=serial`).

Если вместо экрана Firefox пишет «Your Firefox profile cannot be loaded.
It may be missing or inaccessible» — это snap-сборка: ей нельзя отдать
профиль в `~/.cache` или `/tmp`. Запуск сам создаёт каталог в
`~/snap/firefox/common/robot-kiosk-browser`. Закройте окно с ошибкой и
запустите `./scripts/start_robot.sh` ещё раз.

## 6. Точка доступа Wi-Fi

```bash
sudo RC_AP_SSID=ROBOT RC_AP_PSK='придумайте-8-и-больше' ./scripts/setup_ap.sh
```

Чтобы точка поднималась при загрузке:

```bash
sudo nmcli connection modify robot-ap connection.autoconnect yes
```

Проверка, что AP поднялся:

```bash
nmcli connection show robot-ap | grep -i 'connection.state'
ip addr show | grep 10.42.0.1
```

## 7. Проверка

С телефона, подключённого к сети `ROBOT`: откройте `http://10.42.0.1:8080/` —
должна появиться форма входа. После входа откроется основной экран робота.
Киоск доступен на дисплее машины через `http://127.0.0.1:8090/kiosk`.

С самой машины:

```bash
curl -i http://127.0.0.1:8080/healthz
curl -i -o /dev/null -s -w '%{http_code} -> %{redirect_url}\n' http://127.0.0.1:8080/
```

Ожидается `200` с `{"status":"ok", ...}` и `302 -> .../login?next=/`.

Лидар на обоих экранах (приложения должны работать):

```bash
.venv/bin/python scripts/lidar_web_check.py --wait 20
```

Ожидается `✔ реальный скан: N точек` по каждому приложению и итог
`реальный лидар виден на обоих экранах (источники: peer, serial:0)`; код
возврата `0`. Если скана нет, команда печатает причину из API, интерпретатор
(должен быть из `.venv`), наличие pyserial и диагностику USB-портов
(права, исключения портов шасси).

## 8. HTTPS (необязательно)

Кошелёк самоподписанного сертификата и запуск через HTTPS:

```bash
sudo apt install -y python3-certbot-apache   # или сгенерировать самоподписанный
```

Проще всего поставить перед приложением `caddy` или `nginx` с TLS и включить
`RC_COOKIE_SECURE=true`. Отдельно gunicorn настраивать не нужно: `run.py` по
умолчанию сам поднимает production WSGI-сервер gunicorn (1 процесс, 4 потока;
размер регулируется `RC_WORKERS`/`RC_THREADS`), а `run.py --dev` оставляет
встроенный сервер Flask для отладки.

## Если что-то не так

| Симптом | Что проверить |
| --- | --- |
| «Учётки операторов не настроены» | запущен ли `scripts/create_operator.py`, путь `RC_OPERATORS_FILE` |
| Адаптер не поддерживает AP | `iw list \| grep -A20 'Supported interface modes'` |
| Точка не поднимается | `nmcli device status`, свободен ли адаптер от другой сети |
| Сессии слетают после рестарта | не задан `RC_SECRET_KEY` |
| Порт занят | смените `RC_PORT` в `.env` |
| На экране «скана лидара нет» | под картой написана причина; подробнее — `scripts/lidar_check.py`, `GET /api/lidar` (поле `tried` — какие порты проверены), группа `dialout`, скорость под модель (см. раздел 4b) |
| Приложение запускается само, а не нужно | `./scripts/autostart.sh status`, затем `./scripts/autostart.sh off` |
| Лидар отвечает, но точек нет | мотор: перебрать `RC_LIDAR_MOTOR` (`dtr-low`, `rts-low`, …) или проверить питание лидара |
| Лидар TurtleBot3 (LDS-01) не найден | проверьте `.venv/bin/python scripts/lidar_check.py --ros` и `./scripts/ros2_lidar.sh --status`: скан должен отдавать драйвер ROS 2 (`hls_lfcd_lds_driver`, а через плату OpenCR — `turtlebot3_node`, `--model opencr`). Для USB-режима — `scripts/lidar_check.py --protocol lds01` (см. раздел 4b) |
| Карта лидара пустая, хотя `lidar_check.py` лидар находит | веб-приложение запущено не из `.venv` (нет pyserial) или `.env` не перечитан: `./scripts/ensure_deps.sh`, `.venv/bin/python scripts/lidar_check.py --save-env`, перезапуск; проверка — `.venv/bin/python scripts/lidar_web_check.py` |
| Экран показывает `демо · N точек (не лидар)` | реального скана нет: причина написана под картой, подробности — `GET /api/lidar` (поля `reason`, `usb_diagnostics`) |
| Бортовой экран без скана, пульт со сканом | второй процесс берёт скан у первого по HTTP: `source: peer` — норма. Если `peer` не работает, проверьте `RC_LIDAR_PEER_URLS` и что оба приложения на одной машине |
| `Permission denied` на `/dev/ttyUSB*` у службы | `sudo usermod -aG dialout robotctl` и перезапуск службы; в юнитах `SupplementaryGroups=dialout` уже есть (см. раздел 4b) |
| После обновления кода пропал pyserial | `./scripts/ensure_deps.sh` — скрипт проверяет импорты в `.venv` и доустанавливает недостающее |
