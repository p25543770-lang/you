#!/usr/bin/env bash
#
# Ярлык-запуск пульта оператора. При каждом старте:
#   1. автообновление кода с git (reset на origin текущей ветки);
#   2. подтягивание зависимостей (venv создаётся, pip ставит только при изменении
#      requirements.txt);
#   3. создание .env и подсказка по учётке, если их нет;
#   4. перезапуск сервера (если уже работал — аккуратно перезапускается);
#   5. открытие сайта в браузере (если есть графика).
#
# Использование: ./scripts/start.sh   (или ярлык «Робот-пульт» на рабочем столе)
# Остановка:     ./scripts/stop.sh
# Логи:          var/server.log
#
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"

log() { printf '\033[1;32m[start]\033[0m %s\n' "$*"; }

# при падении — сразу видно, где и на чём
trap 'echo "✗ Ошибка на строке $LINENO (команда: $BASH_COMMAND)" >&2' ERR

PORT="$(grep -E '^RC_PORT=' .env 2>/dev/null | cut -d= -f2 || true)"
PORT="${PORT:-8080}"

# --- 1. автообновление с git ------------------------------------------------- #
if [[ -d .git ]] && command -v git >/dev/null 2>&1; then
  BR="$(git rev-parse --abbrev-ref HEAD)"
  if git fetch -q origin "$BR" 2>/dev/null; then
    LOCAL="$(git rev-parse HEAD)"
    REMOTE="$(git rev-parse "origin/$BR")"
    if [[ "$LOCAL" != "$REMOTE" ]]; then
      log "есть обновления с git — применяю (ветка $BR)…"
      git reset --hard -q "origin/$BR"
      # перезапускаем уже обновлённый скрипт (один раз)
      if [[ -z "${RC_UPDATED:-}" ]]; then
        RC_UPDATED=1 exec "$0" "$@"
      fi
    else
      log "код актуален (ветка $BR, $(git rev-parse --short HEAD))"
    fi
  else
    log "git недоступен — пропускаю автообновление"
  fi
else
  log "это не git-клон — автообновление пропущено"
fi

# --- 2. python и зависимости -------------------------------------------------- #
# Работает и без интернета: pip и колёса лежат в vendor/ (см. ensure_deps.sh).
./scripts/ensure_deps.sh

# --- 3. .env и учётка ----------------------------------------------------------- #
if [[ ! -f .env ]]; then
  log "создаю .env…"
  {
    echo "RC_SECRET_KEY=$(python3 -c 'import secrets;print(secrets.token_urlsafe(48))')"
    echo "RC_BIND_HOST=0.0.0.0"
    echo "RC_PORT=8080"
  } > .env
  chmod 600 .env
  PORT=8080
fi
if [[ ! -f data/operators.json ]]; then
  if [[ -t 0 ]]; then
    log "создаю учётку оператора…"
    .venv/bin/python scripts/create_operator.py --username operator || true
  fi
  if [[ ! -f data/operators.json ]]; then
    log "спросить пароль негде — создаю учётку автоматически…"
    ./scripts/create_default_operator.sh operator
  fi
fi

# --- 4. перезапуск сервера -------------------------------------------------------- #
mkdir -p var
PIDFILE=var/run.pid
if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  log "останавливаю предыдущий запуск (pid $(cat "$PIDFILE"))…"
  kill "$(cat "$PIDFILE")" 2>/dev/null || true
  sleep 1
fi
rm -f "$PIDFILE"

# --- 4b. порт мог занять старый сервер, не отмеченный в pidfile ------------------- #
# (запущенный вручную или старой установкой): его код в памяти не совпадает с
# новыми шаблонами на диске — это даёт 500. Убиваем таких автоматически.
port_busy() {
  .venv/bin/python -c '
import socket, sys
s = socket.socket()
try:
    s.connect(("127.0.0.1", int(sys.argv[1]))); sys.exit(0)
except Exception:
    sys.exit(1)
finally:
    s.close()
' "$PORT" 2>/dev/null
}
if port_busy; then
  log "порт $PORT занят старым сервером — завершаю его…"
  pkill -f "robot_control.app:create_app" 2>/dev/null || true
  pkill -f "$ROOT/run.py" 2>/dev/null || true
  pkill -f "$ROOT/.venv/bin/python" 2>/dev/null || true
  command -v fuser >/dev/null 2>&1 && fuser -k "${PORT}/tcp" 2>/dev/null || true
  sleep 1
  # крайний случай: убиваем любого слушателя порта по pid из ss
  if port_busy; then
    PIDS="$(ss -ltnp 2>/dev/null | grep -E "[:.]${PORT}\s" | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u)"
    for pid in $PIDS; do
      log "завершаю процесс $pid, держащий порт $PORT…"
      kill "$pid" 2>/dev/null || true
    done
    sleep 1
  fi
  if port_busy; then
    echo "Порт $PORT всё ещё занят другим процессом — освободите его и повторите." >&2
    exit 1
  fi
fi

# --- 4c. фаервол: открыть порт, чтобы подключались другие устройства ---------- #
if command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | grep -q "Status: active"; then
  if ! ufw status 2>/dev/null | grep -q "${PORT}/tcp"; then
    log "ufw активен — открываю порт $PORT…"
    if [[ $EUID -eq 0 ]]; then
      ufw allow "${PORT}/tcp" >/dev/null
    elif [[ -t 0 ]] && command -v sudo >/dev/null 2>&1; then
      sudo ufw allow "${PORT}/tcp" >/dev/null
    elif command -v sudo >/dev/null 2>&1; then
      sudo -n ufw allow "${PORT}/tcp" >/dev/null 2>&1 \
        || log "нет прав sudo: выполните сами 'sudo ufw allow ${PORT}/tcp'"
    else
      log "выполните сами: sudo ufw allow ${PORT}/tcp"
    fi
  fi
fi

# --- 5. запуск: сервер привязан к этому окну ---------------------------------- #
# Закрытие окна терминала (или Ctrl+C) останавливает сервер — старых висящих
# процессов с устаревшим кодом больше не будет.
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
log "адреса: http://127.0.0.1:${PORT}/  (в сети: http://${IP:-?}:${PORT}/)"
log "ВАЖНО: открывайте именно http://…, а не https:// — TLS не настроен."
log "сервер работает в этом окне: закроете окно или Ctrl+C — он остановится."

SERVER_PID=""
cleanup() {
  if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    # gunicorn живёт в своей сессии (setsid): убиваем всю его группу
    kill -TERM -- -"$SERVER_PID" 2>/dev/null || kill -TERM "$SERVER_PID" 2>/dev/null || true
  fi
  rm -f "$PIDFILE"
}
trap 'cleanup; exit 130' INT TERM
trap 'cleanup; exit 129' HUP

# setsid: сервер в своей сессии — терминальный HUP приходит только сюда,
# а мы сами решаем, когда остановить сервер (закрытие окна / Ctrl+C)
if command -v setsid >/dev/null 2>&1; then
  setsid .venv/bin/python run.py &
else
  .venv/bin/python run.py &
fi
SERVER_PID=$!
echo "$SERVER_PID" > "$PIDFILE"

# --- 6. проверка и браузер ------------------------------------------------------ #
health() {
  RC_PORT_CHECK="$PORT" .venv/bin/python -c '
import os, sys, urllib.request
try:
    r = urllib.request.urlopen(
        "http://127.0.0.1:%s/healthz" % os.environ["RC_PORT_CHECK"], timeout=1)
    sys.exit(0 if r.status == 200 else 1)
except Exception:
    sys.exit(1)
' 2>/dev/null
}

ok=""
for _ in $(seq 1 15); do
  if health; then ok=1; break; fi
  sleep 1
done
if [[ -n "$ok" ]]; then
  log "работает. Логи идут прямо в это окно."
  if [[ -n "${DISPLAY:-}" ]] && command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://127.0.0.1:${PORT}/" >/dev/null 2>&1 || true
  fi
else
  echo "Сервер не поднялся (ошибка выше)." >&2
  cleanup
  exit 1
fi

wait "$SERVER_PID"
STATUS=$?
SERVER_PID=""
rm -f "$PIDFILE"
exit "$STATUS"
