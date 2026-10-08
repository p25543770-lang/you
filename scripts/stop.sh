#!/usr/bin/env bash
# Останавливает сервер, поднятый scripts/start.sh.
set -euo pipefail
cd "$(dirname "$0")/.."

PIDFILE=var/run.pid
if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  PID="$(cat "$PIDFILE")"
  # gunicorn мастер поднимает воркеров — убиваем всю группу процессов
  kill -- -"$(ps -o pgid= -p "$PID" | tr -d ' ')" 2>/dev/null || kill "$PID"
  sleep 1
  rm -f "$PIDFILE"
  echo "Остановлено (pid $PID)."
else
  rm -f "$PIDFILE"
  echo "Сервер не запущен."
fi
