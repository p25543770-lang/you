#!/usr/bin/env bash
#
# Установка и запуск пульта оператора на Ubuntu одной командой:
#
#   git clone https://github.com/p25543770-lang/you.git robot-control
#   cd robot-control
#   ./scripts/install_ubuntu.sh
#
# Скрипт: ставит зависимости, создаёт .env с секретным ключом, спрашивает
# логин/пароль оператора, открывает порт 8080 в ufw (если он включён)
# и запускает сервер. После этого пульт доступен с других устройств в той же
# сети по адресу http://<IP-этой-машины>:8080/
#
# Автозапуск при загрузке по умолчанию НЕ ставится: после установки пульт и
# бортовой экран поднимаются вручную (./scripts/start.sh, ./scripts/kiosk.sh).
# Нужен автозапуск — флаг --autostart (или позже: ./scripts/autostart.sh on).
#
# Флаги:
#   --no-run         установить, но не запускать сервер
#   --autostart      включить автозапуск: служба systemd robot-control (пульт),
#                    автозапуск бортового экрана в сеансе; точка доступа Wi-Fi —
#                    см. --ap-autostart
#   --systemd        то же, что --autostart (устаревшее имя флага)
#   --ap             поднять на этой машине точку доступа Wi-Fi (робот сам раздаёт сеть)
#   --ap-autostart   поднимать точку доступа и при каждой загрузке
#
set -euo pipefail
cd "$(dirname "$0")/.."

RUN=1
AUTOSTART=0
AP=0
AP_AUTOSTART=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-run)       RUN=0; shift ;;
    --autostart)    AUTOSTART=1; shift ;;
    --systemd)      AUTOSTART=1; shift ;;                 # старое имя флага
    --ap)           AP=1; shift ;;
    --ap-autostart) AP=1; AP_AUTOSTART=1; shift ;;
    -h|--help)      sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "неизвестный аргумент: $1" >&2; exit 1 ;;
  esac
done

log() { printf '\033[1;32m[install]\033[0m %s\n' "$*"; }

# --- 0. версия Python ------------------------------------------------------- #
if ! python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)' 2>/dev/null; then
  echo "Нужен Python 3.9 или новее, а у вас $(python3 --version 2>&1)." >&2
  echo "На Ubuntu 22.04+ подходящий python3 уже стоит; на 20.04 обновите Python." >&2
  exit 1
fi
log "python3: $(python3 --version 2>&1) (нужен 3.9+)"

# --- 1-2. python и зависимости (работает офлайн через vendor/) -------------- #
./scripts/ensure_deps.sh

# --- 3. .env с секретным ключом ---------------------------------------------- #
if [[ ! -f .env ]]; then
  log "создаю .env с новым RC_SECRET_KEY…"
  {
    echo "RC_SECRET_KEY=$(python3 -c 'import secrets;print(secrets.token_urlsafe(48))')"
    echo "RC_BIND_HOST=0.0.0.0"
    echo "RC_PORT=8080"
  } > .env
  chmod 600 .env
else
  log ".env уже есть — не трогаю"
fi

# --- 4. учётка оператора ------------------------------------------------------ #
if [[ ! -f data/operators.json ]]; then
  if [[ -t 0 ]]; then
    log "создаю учётку оператора (логин по умолчанию admin)…"
    .venv/bin/python scripts/create_operator.py \
      --username "${RC_OPERATOR:-admin}" --min-length "${RC_MIN_PASSWORD_LENGTH:-4}"
  else
    log "терминала для ввода нет — создаю учётку автоматически…"
    ./scripts/create_default_operator.sh "${RC_OPERATOR:-admin}"
  fi
else
  log "учётки уже созданы — пропускаю"
fi

# --- 4b. проверка: что реально записалось ------------------------------------- #
# Fail fast: если учётка почему-то не создалась, сервер потом будет отвергать
# любой пароль как «неверный» — лучше упасть здесь с понятной причиной.
log "проверяю записанные учётки…"
.venv/bin/python scripts/create_operator.py --list

# --- 5. порт в фаерволе -------------------------------------------------------- #
if command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | grep -q "Status: active"; then
  if command -v sudo >/dev/null 2>&1 && [[ $EUID -ne 0 ]]; then SUDO=sudo; else SUDO=; fi
  $SUDO ufw allow 8080/tcp >/dev/null
  log "ufw: порт 8080 открыт"
fi

# --- 6. автозапуск (только по флагу) ----------------------------------------- #
# По умолчанию не ставим ничего: робот после включения не поднимает приложение
# само. Хочешь автозапуск — --autostart (или позже ./scripts/autostart.sh on).
if [[ $AUTOSTART -eq 1 ]]; then
  log "ставлю systemd-юнит и включаю автозапуск…"
  if command -v sudo >/dev/null 2>&1 && [[ $EUID -ne 0 ]]; then SUDO=sudo; else SUDO=; fi
  $SUDO cp scripts/robot-control.service /etc/systemd/system/
  $SUDO sed -i "s|/opt/robot-control|$PWD|g" /etc/systemd/system/robot-control.service
  $SUDO sed -i "s|User=robotctl|User=$USER|g; s|Group=robotctl|Group=$USER|g" /etc/systemd/system/robot-control.service
  # Служба обязана стартовать python'ом из .venv (там Flask и pyserial):
  # с системным python3 пульт поднялся бы без лидара вообще.
  if ! grep -q "$PWD/.venv/bin/python" /etc/systemd/system/robot-control.service; then
    $SUDO sed -i "s|^ExecStart=.*|ExecStart=$PWD/.venv/bin/python $PWD/run.py|" \
      /etc/systemd/system/robot-control.service
  fi
  $SUDO systemctl daemon-reload
  $SUDO systemctl enable --now robot-control
  log "служба robot-control запущена (автозапуск включён)"
else
  log "автозапуск не ставлю: пульт и экран запускаются вручную"
  log "  включить: sudo ./scripts/install_ubuntu.sh --autostart (или ./scripts/autostart.sh on)"
fi

# --- 6b. точка доступа Wi-Fi (опция) ---------------------------------------- #
if [[ $AP -eq 1 ]]; then
  if [[ $AP_AUTOSTART -eq 1 ]]; then
    log "поднимаю точку доступа Wi-Fi (и буду поднимать при загрузке)…"
    ./scripts/setup_ap.sh --autostart || log "точку доступа поднять не удалось — см. текст выше"
  else
    log "поднимаю точку доступа Wi-Fi (без автоподъёма при загрузке)…"
    ./scripts/setup_ap.sh || log "точку доступа поднять не удалось — см. текст выше"
  fi
fi

# --- 7. ярлык на рабочем столе (если есть графика) --------------------------- #
if [[ -n "${DISPLAY:-}" ]]; then
  if [[ $AUTOSTART -eq 1 ]]; then
    ./scripts/make_shortcut.sh --autostart || log "ярлык создать не удалось — не критично"
  else
    ./scripts/make_shortcut.sh || log "ярлык создать не удалось — не критично"
  fi
fi

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
log "готово"
echo
echo "  Локально : http://127.0.0.1:8080/"
[[ -n "$IP" ]] && echo "  В сети   : http://${IP}:8080/   (с телефона/ноутбука в той же сети)"
echo
echo "  Подключение: логин/пароль оператора, которые вы задали на шаге 4."
if [[ $AP -eq 1 ]]; then
  echo "  Через Wi-Fi робота: подключитесь к сети из data/wifi.txt"
  echo "    и откройте http://10.42.0.1:8080/"
else
  echo "  Точка доступа Wi-Fi (если машина робота раздаёт сеть):"
  echo "    sudo ./scripts/setup_ap.sh            # поднять сейчас"
  echo "    sudo ./scripts/setup_ap.sh --autostart  # + поднимать при загрузке"
fi
echo
if [[ $AUTOSTART -eq 1 ]]; then
  echo "  Автозапуск включён: после загрузки пульт поднимается сам."
  echo "    выключить: ./scripts/autostart.sh off"
else
  echo "  Автозапуска нет — приложение запускается вручную:"
  echo "    ./scripts/start.sh    пульт оператора (8080)"
  echo "    ./scripts/kiosk.sh    бортовой экран робота (8090)"
  echo "    ./scripts/autostart.sh status   # что вообще стартует само"
fi

if [[ $RUN -eq 1 && $AUTOSTART -eq 0 ]]; then
  echo
  log "запускаю сервер (Ctrl+C для остановки)…"
  exec .venv/bin/python run.py
fi
