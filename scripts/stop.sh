#!/usr/bin/env bash
# Останавливает серверы, поднятые scripts/start.sh (пульт, 8080) и
# scripts/kiosk.sh (бортовой экран, 8090).
set -euo pipefail
cd "$(dirname "$0")/.."

stop_by_pidfile() {
  local pidfile="$1" label="$2"
  if [[ -f "$pidfile" ]] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
    local PID
    PID="$(cat "$pidfile")"
    # gunicorn мастер поднимает воркеров — убиваем всю группу процессов
    kill -- -"$(ps -o pgid= -p "$PID" | tr -d ' ')" 2>/dev/null || kill "$PID"
    sleep 1
    rm -f "$pidfile"
    echo "Остановлено: $label (pid $PID)."
  else
    rm -f "$pidfile"
    echo "Не запущено: $label."
  fi
}

stop_by_pidfile var/run.pid "пульт оператора (8080)"
stop_by_pidfile var/kiosk.pid "бортовой экран (8090)"
