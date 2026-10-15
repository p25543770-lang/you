# Установка на машину с Ubuntu

Инструкция для ноутбука/ПК на Ubuntu, который будет бортовым компьютером робота.

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

## 4. Запуск веб-пульта (автозапуск — по желанию)

По умолчанию приложение **не** стартует само: после загрузки робота пульт
поднимают вручную — `./scripts/start.sh` (или ярлык «Робот-пульт»).

Если нужен автозапуск при загрузке:

```bash
sudo cp scripts/robot-control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-control
journalctl -u robot-control -f
```

Юнит запускает сервис под непривилегированным пользователем `robotctl` с
`ProtectSystem=strict` и записью только в `data/` и `var/`.

Выключить автозапуск обратно (и вообще весь автозапуск: службы, ярлык экрана,
автоподъём точки доступа):

```bash
./scripts/autostart.sh status              # что поднимается само
./scripts/autostart.sh off                 # выключить автозапуск И остановить всё сейчас
./scripts/autostart.sh off --keep-running  # только выключить автозапуск
./scripts/autostart.sh on                  # вернуть автозапуск
```

Полностью погасить приложение, не трогая автозапуск, — `./scripts/stop.sh`
(серверы по pid-файлам, службы, окно киоска).

## 4b. Лидар (USB/UART)

Лидар читает само приложение (`robot_control/lidar.py`): порт, протокол и
скорость подбираются перебором, мотор RPLIDAR включается линией USB-переходника.
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
(`./scripts/stop.sh && ./scripts/start.sh`, `./scripts/kiosk.sh` или
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

## 5. Бортовой экран (киоск, порт 8090)

Дисплей робота показывает только телеметрию: АКБ, мини-шасси, лидар. Вход не
нужен — экран открыт только на чтение.

```bash
sudo cp scripts/robot-kiosk.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-kiosk
journalctl -u robot-kiosk -f
```

Служба держит сервер на 8090 с момента загрузки и перезапускается при падении.
Ярлыки (меню и рабочий стол) ставятся так — под пользователем рабочего стола
робота (не root):

```bash
./scripts/make_shortcut.sh              # ярлыки, без автозапуска
./scripts/make_shortcut.sh --autostart  # + открывать экран сам после входа
./scripts/autostart.sh off              # выключить любой автозапуск
```

Запуск вручную (одной командой, поднимает и сервер, и браузер):

```bash
./scripts/kiosk.sh
```

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

Если вместо экрана Firefox пишет «Your Firefox profile cannot be loaded.
It may be missing or inaccessible» — это snap-сборка: ей нельзя отдать
профиль в `~/.cache` или `/tmp`. Запуск сам создаёт каталог в
`~/snap/firefox/common/robot-kiosk-browser`. Закройте окно с ошибкой и
запустите `./scripts/kiosk.sh` ещё раз.

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
должна появиться форма входа. После входа — страница-заглушка.

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
| Лидар TurtleBot3 (LDS-01) не найден | проверьте `scripts/lidar_check.py --protocol lds01`; если лидар подключён через OpenCR, напрямую он недоступен (см. раздел 4b) |
| Карта лидара пустая, хотя `lidar_check.py` лидар находит | веб-приложение запущено не из `.venv` (нет pyserial) или `.env` не перечитан: `./scripts/ensure_deps.sh`, `.venv/bin/python scripts/lidar_check.py --save-env`, перезапуск; проверка — `.venv/bin/python scripts/lidar_web_check.py` |
| Экран показывает `демо · N точек (не лидар)` | реального скана нет: причина написана под картой, подробности — `GET /api/lidar` (поля `reason`, `usb_diagnostics`) |
| Бортовой экран без скана, пульт со сканом | второй процесс берёт скан у первого по HTTP: `source: peer` — норма. Если `peer` не работает, проверьте `RC_LIDAR_PEER_URLS` и что оба приложения на одной машине |
| `Permission denied` на `/dev/ttyUSB*` у службы | `sudo usermod -aG dialout robotctl` и перезапуск службы; в юнитах `SupplementaryGroups=dialout` уже есть (см. раздел 4b) |
| После обновления кода пропал pyserial | `./scripts/ensure_deps.sh` — скрипт проверяет импорты в `.venv` и доустанавливает недостающее |
