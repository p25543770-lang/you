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
sudo python3 -m venv .venv
sudo .venv/bin/pip install -r requirements.txt
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

## 4. Автозапуск веб-пульта

```bash
sudo cp scripts/robot-control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-control
journalctl -u robot-control -f
```

Юнит запускает сервис под непривилегированным пользователем `robotctl` с
`ProtectSystem=strict` и записью только в `data/` и `var/`.

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
Чтобы после входа в систему экран открывался сам (браузер без рамок), поставьте
ярлык и его автозапуск — под пользователем рабочего стола робота (не root):

```bash
./scripts/make_shortcut.sh
```

Запуск вручную (одной командой, поднимает и сервер, и браузер):

```bash
./scripts/kiosk.sh
```

Проверка: `curl http://127.0.0.1:8090/healthz` → `{"mode":"kiosk","ok":true,…}`.
Остановить всё — `./scripts/stop.sh`.

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
