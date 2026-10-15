#!/usr/bin/env bash
#
# Останавливает всё, что подняло приложение:
#   * пульт оператора (8080) и бортовой экран (8090) — по pid-файлам из var/
#     (их пишут start.sh и kiosk.sh; гасится вся группа процессов, поэтому
#     закрывается и окно браузера);
#   * супервизор лидара ROS 2, запущенный ярлыком «Запустить робот»
#     (var/lidar.pid; он сам гасит драйвер лидара и освобождает USB-порт);
#   * службы systemd robot-control, robot-kiosk и robot-lidar (лидар через ROS 2),
#     если они сейчас запущены;
#   * оставшиеся окна киоск-браузера (профиль robot-kiosk-browser).
#
# Автозапуск это НЕ выключает: если робот должен перестать поднимать
# приложение сам после включения, нужен ./scripts/autostart.sh off
# (он и выключит автозапуск, и остановит всё запущенное разом).
#
# Использование:
#   ./scripts/stop.sh              остановить всё перечисленное
#   ./scripts/stop.sh --servers    только серверы по pid-файлам (пульт и экран)
#   ./scripts/stop.sh --help
#
# Каталог с pid-файлами: var/ (можно переопределить: RC_VAR_DIR=/путь).
#
set -euo pipefail
cd "$(dirname "$0")/.."

VAR_DIR="${RC_VAR_DIR:-var}"
ONLY_SERVERS=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --servers)  ONLY_SERVERS=1; shift ;;
    -h|--help)  sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "неизвестный аргумент: $1 (см. --help)" >&2; exit 1 ;;
  esac
done

# Для systemctl нужен root: сами зовём sudo, если он есть.
SUDO=""
if [[ $EUID -ne 0 ]] && command -v sudo >/dev/null 2>&1; then
  SUDO="sudo"
fi

have_systemd() {
  command -v systemctl >/dev/null 2>&1 || return 1
  $SUDO systemctl list-unit-files --no-pager >/dev/null 2>&1
}

STOPPED=0

stop_server() {
  local pidfile="$1" label="$2" pid pgid
  if [[ -f "$pidfile" ]] && pid="$(cat "$pidfile" 2>/dev/null)" && [[ -n "$pid" ]] \
     && kill -0 "$pid" 2>/dev/null; then
    # Сервер поднимает воркеров и окно браузера — убиваем всю группу процессов.
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    mypgid="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ' || true)"
    if [[ -n "$pgid" && "$pgid" != "$mypgid" ]]; then
      kill -- "-$pgid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    else
      # Группа та же, что у нас (сервер запущен из этой же оболочки без
      # job control): группу не трогаем, иначе убьём и себя, и терминал.
      kill "$pid" 2>/dev/null || true
    fi
    sleep 1
    rm -f "$pidfile"
    echo "Остановлено: $label (pid $pid)."
    STOPPED=1
    return 0
  fi
  rm -f "$pidfile"
  echo "Не запущено: $label."
  return 1
}

stop_service() {
  local unit="$1" title="$2" state
  if ! have_systemd; then
    echo "systemd недоступен — службу $unit не проверяю."
    return 1
  fi
  state="$($SUDO systemctl is-active "$unit.service" 2>/dev/null || true)"
  if [[ "$state" != "active" ]]; then
    echo "Не запущено: служба $unit ($title)."
    return 1
  fi
  if $SUDO systemctl stop "$unit.service" >/dev/null 2>&1; then
    echo "Остановлено: служба $unit ($title)."
    STOPPED=1
    return 0
  fi
  echo "Не удалось остановить службу $unit — проверь: systemctl status $unit"
  return 1
}

stop_browsers() {
  local pids
  if ! command -v pgrep >/dev/null 2>&1; then
    echo "pgrep не найден — окно киоска не проверяю."
    return 1
  fi
  pids="$(pgrep -f "robot-kiosk-browser" 2>/dev/null || true)"
  if [[ -z "$pids" ]]; then
    echo "Не запущено: окно киоск-браузера."
    return 1
  fi
  # shellcheck disable=SC2086 — список pid без кавычек нужен намеренно
  kill $pids 2>/dev/null || true
  sleep 1
  pids="$(pgrep -f "robot-kiosk-browser" 2>/dev/null || true)"
  if [[ -n "$pids" ]]; then
    # shellcheck disable=SC2086
    kill -9 $pids 2>/dev/null || true
  fi
  echo "Остановлено: окно киоск-браузера."
  STOPPED=1
  return 0
}

# «нечего останавливать» — это не ошибка: функция возвращает 1, но скрипт
# должен дойти до конца и отчитаться по всем пунктам (set -e → || true).
stop_server "$VAR_DIR/run.pid" "пульт оператора (8080)" || true
stop_server "$VAR_DIR/kiosk.pid" "бортовой экран (8090)" || true
# Супервизор лидара в своей группе процессов: TERM получает цикл перезапуска,
# а он — супервизор, который останавливает драйвер и освобождает порт.
stop_server "$VAR_DIR/lidar.pid" "лидар ROS 2 (супервизор ярлыка)" || true

if [[ $ONLY_SERVERS -eq 0 ]]; then
  stop_service robot-control "пульт оператора, служба systemd" || true
  stop_service robot-kiosk "бортовой экран, служба systemd" || true
  # Драйвер лидара держит USB-порт: останавливаем его вместе с приложениями,
  # иначе при следующем запуске порт занят и скан не поднимается.
  stop_service robot-lidar "лидар ROS 2, служба systemd" || true
  stop_browsers || true
fi

echo
if [[ $STOPPED -eq 1 ]]; then
  echo "Всё остановлено."
else
  echo "Запущенного приложения не найдено."
fi
echo "Автозапуск: ./scripts/autostart.sh status   (выключить — ./scripts/autostart.sh off)"
