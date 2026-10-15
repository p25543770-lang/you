#!/usr/bin/env bash
# Единственный запуск робота: веб-пульт (8080) + экран на борту (8090) + лидар
# через ROS 2. Зависимости ставятся/проверяются один раз, окна и серверы не
# дублируются. ROS 2, драйверы лидаров и ros2_ws докачиваются и компилируются
# сами (scripts/ros2_setup.sh): в первый раз — несколько минут и один вопрос
# пароля sudo в этом окне, дальше — секунды. Выключить: RC_ROS_AUTO=0 в .env.
set -uo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"

log() { printf '\033[1;32m[robot]\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31m[robot]\033[0m %s\n' "$*" >&2; exit 1; }

mkdir -p var
exec 9>var/start-all.lock
if command -v flock >/dev/null 2>&1 && ! flock -n 9; then
  log "робот уже запущен этим ярлыком — второй комплект серверов не создаю."
  exit 0
fi

if [[ "${RC_SKIP_DEPS:-0}" != "1" ]]; then
  ./scripts/ensure_deps.sh || fail "не удалось установить зависимости."
fi
export RC_SKIP_DEPS=1

read_env() {
  local key="$1" fallback="$2" value
  value="$(grep -E "^${key}=" .env 2>/dev/null | tail -n 1 | cut -d= -f2- || true)"
  printf '%s' "${value:-$fallback}"
}
PORT="$(read_env RC_PORT 8080)"
KIOSK_PORT="$(read_env RC_KIOSK_PORT 8090)"

# --- ROS 2 и лидар: докачать, скомпилировать, запустить ----------------------- #
ROS_READY=0
ROS_WAS_READY=0
LIDAR_PID=""
prepare_ros() {
  local auto lidar_source status
  if [[ -f scripts/ros2_setup.sh ]] && bash scripts/ros2_setup.sh --check >/dev/null 2>&1; then
    ROS_WAS_READY=1
  fi
  auto="${RC_ROS_AUTO:-$(read_env RC_ROS_AUTO 1)}"
  lidar_source="${RC_LIDAR_SOURCE:-$(read_env RC_LIDAR_SOURCE auto)}"
  if [[ "$auto" == "0" ]]; then
    log "ROS 2: автоустановка выключена (RC_ROS_AUTO=0) — только проверяю."
  elif [[ "$lidar_source" == "serial" ]]; then
    log "лидар читается по USB (RC_LIDAR_SOURCE=serial) — ROS 2 не трогаю."
    return 0
  elif [[ -f scripts/ros2_setup.sh ]]; then
    log "ROS 2 и лидар: проверяю; чего не хватает — докачаю и скомпилирую сам…"
    # --no-wait: если установка уже идёт (другое окно или фоновая сборка),
    # робот запускается сразу, а не ждёт её конца.
    # bash явно: скрипт работает, даже если у файла потерян бит исполнения.
    bash scripts/ros2_setup.sh --background-build --no-wait --quiet
    status=$?
    case "$status" in
      0)  ;;
      3)  log "ROS 2 ждёт пароля администратора: запусти ярлык «Запустить робот» (откроет терминал)" \
              "или ./scripts/ros2_setup.sh — лидар пока читается по USB." ;;
      4)  log "ROS 2 компилируется в фоне — лидар пока читается по USB (ход: ./scripts/ros2_setup.sh --status)." ;;
      75) log "установка ROS 2 уже идёт в другом окне — лидар пока читается по USB." ;;
      *)  log "ROS 2 настроен не до конца (код $status) — подробности: var/ros2_setup.log;" \
              "робот работает дальше." ;;
    esac
  fi
  if [[ -f scripts/ros2_setup.sh ]] && bash scripts/ros2_setup.sh --check >/dev/null 2>&1; then
    ROS_READY=1
  fi
}

lidar_running() {
  local pid
  pid="$(cat var/lidar.pid 2>/dev/null || true)"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null
}

# Супервизор лидара (пакет zapret_lidar) поднимаем ДО приложений: тогда пульт и
# бортовой экран сразу читают /scan, а USB-порт занимает драйвер ROS 2.
start_lidar() {
  [[ $ROS_READY -eq 1 && -f scripts/ros2_lidar.sh ]] || return 0
  if command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet robot-lidar 2>/dev/null; then
    log "лидар ROS 2: работает служба robot-lidar."
    return 0
  fi
  if lidar_running; then
    log "лидар ROS 2: супервизор уже запущен (pid $(cat var/lidar.pid))."
    return 0
  fi
  if [[ -f var/lidar.log ]] && (( $(stat -c %s var/lidar.log 2>/dev/null || echo 0) > 2000000 )); then
    mv -f var/lidar.log var/lidar.log.1
  fi
  # Отдельная сессия (setsid): Ctrl+C в этом окне не рвёт драйвер на полуслове —
  # супервизор останавливается по TERM из cleanup и сам гасит драйвер.
  if command -v setsid >/dev/null 2>&1; then
    setsid bash scripts/ros2_lidar.sh --keep-alive --no-setup >>var/lidar.log 2>&1 </dev/null &
  else
    bash scripts/ros2_lidar.sh --keep-alive --no-setup >>var/lidar.log 2>&1 </dev/null &
  fi
  LIDAR_PID=$!
  log "лидар ROS 2: супервизор запущен (журнал var/lidar.log, статус: ./scripts/ros2_lidar.sh --status)."
}

health() {
  local port="$1"
  .venv/bin/python -c '
import sys, urllib.request
try:
    response = urllib.request.urlopen("http://127.0.0.1:%s/healthz" % sys.argv[1], timeout=1)
    sys.exit(0 if response.status == 200 else 1)
except Exception:
    sys.exit(1)
' "$port" >/dev/null 2>&1
}
port_open() {
  .venv/bin/python -c '
import socket, sys
sock = socket.socket(); sock.settimeout(.3)
try:
    sock.connect(("127.0.0.1", int(sys.argv[1]))); sys.exit(0)
except OSError:
    sys.exit(1)
finally:
    sock.close()
' "$1" >/dev/null 2>&1
}

# ROS 2 поставлен только что, а пульт и экран уже работают службами systemd
# (автозапуск): их процессы стартовали без окружения ROS 2 и rclpy не увидят —
# перезапускаем их один раз (пароль sudo только что вводили для установки).
restart_services_for_ros() {
  local units=() unit
  [[ $ROS_READY -eq 1 && $ROS_WAS_READY -eq 0 ]] || return 0
  command -v systemctl >/dev/null 2>&1 || return 0
  for unit in robot-control robot-kiosk; do
    systemctl is-active --quiet "$unit" 2>/dev/null && units+=("$unit")
  done
  [[ ${#units[@]} -gt 0 ]] || return 0
  log "ROS 2 только что поставлен — перезапускаю службы ${units[*]}, чтобы они увидели лидар из ROS 2…"
  if sudo -n systemctl restart "${units[@]}" 2>/dev/null \
     || { [[ -t 0 ]] && sudo systemctl restart "${units[@]}"; }; then
    for _ in $(seq 1 30); do health "$PORT" && break; sleep 1; done
  else
    log "перезапусти службы сам: sudo systemctl restart ${units[*]}"
  fi
}

CONTROL_PID=""
KIOSK_PID=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "$LIDAR_PID" ]] && kill -0 "$LIDAR_PID" 2>/dev/null; then
    kill -TERM "$LIDAR_PID" 2>/dev/null || true
  fi
  if [[ -n "$KIOSK_PID" ]] && kill -0 "$KIOSK_PID" 2>/dev/null; then
    # Фоновый bash child игнорирует SIGINT в неинтерактивной оболочке; SIGTERM
    # гарантированно прекращает run_kiosk.py и его HTTP-сервер.
    kill -TERM "$KIOSK_PID" 2>/dev/null || true
  fi
  if [[ -n "$CONTROL_PID" ]] && kill -0 "$CONTROL_PID" 2>/dev/null; then
    kill -TERM "$CONTROL_PID" 2>/dev/null || true
  fi
  [[ -z "$KIOSK_PID" ]] || wait "$KIOSK_PID" 2>/dev/null || true
  [[ -z "$CONTROL_PID" ]] || wait "$CONTROL_PID" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

prepare_ros
start_lidar
restart_services_for_ros

log "запускаю оба интерфейса одним комплектом."

if health "$PORT"; then
  log "веб-пульт уже работает: http://127.0.0.1:${PORT}/"
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] && command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://127.0.0.1:${PORT}/" >/dev/null 2>&1 || true
  fi
elif port_open "$PORT"; then
  fail "порт $PORT занят не веб-пультом. Освободите порт или задайте RC_PORT в .env."
else
  ./scripts/start.sh &
  CONTROL_PID=$!
  ready=""
  for _ in $(seq 1 45); do
    if health "$PORT"; then ready=1; break; fi
    if ! kill -0 "$CONTROL_PID" 2>/dev/null; then
      wait "$CONTROL_PID"; status=$?
      fail "веб-пульт не запустился (код $status)."
    fi
    sleep 1
  done
  [[ -n "$ready" ]] || fail "веб-пульт не ответил за 45 секунд."
fi

if health "$KIOSK_PORT"; then
  log "экран робота уже работает: http://127.0.0.1:${KIOSK_PORT}/kiosk"
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    ./scripts/kiosk.sh &
    KIOSK_PID=$! # подключается к готовому серверу и открывает только экран
  fi
elif port_open "$KIOSK_PORT"; then
  fail "порт киоска $KIOSK_PORT занят другим приложением."
else
  ./scripts/kiosk.sh &
  KIOSK_PID=$!
fi

log "готово: пульт http://127.0.0.1:${PORT}/ · киоск http://127.0.0.1:${KIOSK_PORT}/kiosk"
log "для остановки нажмите Ctrl+C; можно также выполнить ./scripts/stop.sh."

children=()
[[ -z "$CONTROL_PID" ]] || children+=("$CONTROL_PID")
[[ -z "$KIOSK_PID" ]] || children+=("$KIOSK_PID")
if [[ ${#children[@]} -eq 0 ]]; then
  log "оба сервера уже обслуживаются отдельно; ярлыки браузеров открыты."
  # Серверы живут отдельно — и лидар оставляем работать (остановит ./scripts/stop.sh).
  LIDAR_PID=""
  exit 0
fi
set +e
wait -n "${children[@]}"
status=$?
set -e
exit "$status"
