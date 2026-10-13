#!/usr/bin/env bash
#
# Автозапуск приложения на роботе: посмотреть, выключить или вернуть.
#
# Приложение по умолчанию НЕ стартует само — и пульт оператора (8080), и
# бортовой экран (8090) поднимаются вручную:
#
#   ./scripts/start.sh     пульт оператора
#   ./scripts/kiosk.sh     бортовой экран робота
#
# Этот скрипт нужен, если автозапуск когда-то включали (install_ubuntu.sh
# --autostart/--systemd, `systemctl enable robot-control robot-kiosk`,
# make_shortcut.sh --autostart, setup_ap.sh --autostart): одной командой он
# выключает всё это, а `on` возвращает обратно.
#
# Использование:
#   ./scripts/autostart.sh status   # что поднимается само (по умолчанию)
#   ./scripts/autostart.sh off      # выключить автозапуск целиком
#   ./scripts/autostart.sh on       # вернуть автозапуск
#
# Что проверяет и меняет:
#   * службы systemd: robot-control (пульт), robot-kiosk (экран) — enable/disable;
#   * ярлык экрана в автозапуске сеанса: ~/.config/autostart/robot-kiosk.desktop;
#   * автоподъём точки доступа Wi-Fi robot-ap (connection.autoconnect).
#
# Уже запущенные серверы скрипт не трогает поодиночке: `off` выключает именно
# службы (systemctl disable --now), а вручную запущенные останавливает
# ./scripts/stop.sh.
#
set -euo pipefail
cd "$(dirname "$0")/.."

ACTION="${1:-status}"
UNITS=(robot-control robot-kiosk)
UNIT_TITLES=("пульт оператора (8080)" "бортовой экран (8090)")
AUTOSTART_DIR="${HOME}/.config/autostart"
AUTOSTART_NAMES=(robot-kiosk robot-console)
AP_CONN="robot-ap"

log()  { printf '\033[1;32m[autostart]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[autostart]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[autostart]\033[0m %s\n' "$*" >&2; exit 1; }

# Для systemctl и nmcli нужен root: сами зовём sudo, если он есть.
SUDO=""
if [[ $EUID -ne 0 ]] && command -v sudo >/dev/null 2>&1; then
  SUDO="sudo"
fi

have_systemd() {
  # Не «есть ли бинарь», а «отвечает ли systemd»: на контейнере без systemd
  # команды все равно не сработают, и лучше сказать об этом прямо.
  command -v systemctl >/dev/null 2>&1 || return 1
  $SUDO systemctl list-unit-files --no-pager >/dev/null 2>&1
}

unit_installed() {
  have_systemd && $SUDO systemctl cat "$1.service" >/dev/null 2>&1
}

unit_enabled() {
  local value
  value="$($SUDO systemctl is-enabled "$1.service" 2>/dev/null || true)"
  echo "${value:-not-found}"
}

unit_active() {
  local value
  value="$($SUDO systemctl is-active "$1.service" 2>/dev/null || true)"
  echo "${value:-не запущена}"
}

autostart_file() {
  # Печатает путь к ярлыку автозапуска сеанса, если он есть.
  local name
  for name in "${AUTOSTART_NAMES[@]}"; do
    if [[ -f "$AUTOSTART_DIR/$name.desktop" ]]; then
      echo "$AUTOSTART_DIR/$name.desktop"
      return 0
    fi
  done
  return 0
}

ap_exists() {
  command -v nmcli >/dev/null 2>&1 && nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$AP_CONN"
}

ap_autoconnect() {
  nmcli -g connection.autoconnect connection show "$AP_CONN" 2>/dev/null | head -1 || true
}

print_manual_hint() {
  cat <<'EOF'

Приложение запускается вручную:
  ./scripts/start.sh     пульт оператора (http://<IP>:8080/)
  ./scripts/kiosk.sh     бортовой экран робота (http://127.0.0.1:8090/)
  ./scripts/stop.sh      остановить запущенное
EOF
}

# ------------------------------------------------------------------ status --- #
if [[ $ACTION == "status" ]]; then
  echo "Автозапуск приложения при загрузке робота:"
  auto_count=0
  if ! have_systemd; then
    echo "  • systemd недоступен — службы не проверяю"
  else
    for index in "${!UNITS[@]}"; do
      unit="${UNITS[$index]}"
      title="${UNIT_TITLES[$index]}"
      if ! unit_installed "$unit"; then
        echo "  • $title: служба не установлена"
        continue
      fi
      enabled="$(unit_enabled "$unit")"
      active="$(unit_active "$unit")"
      if [[ "$enabled" == "enabled" || "$enabled" == "enabled-runtime" ]]; then
        echo "  • $title: автозапуск ВКЛЮЧЁН ($enabled, сейчас: $active)"
        auto_count=$((auto_count + 1))
      else
        echo "  • $title: автозапуск выключен ($enabled, сейчас: $active)"
      fi
    done
  fi

  entry="$(autostart_file)"
  if [[ -n "$entry" ]]; then
    echo "  • ярлык экрана в автозапуске сеанса: $entry (ВКЛЮЧЁН)"
    auto_count=$((auto_count + 1))
  else
    echo "  • ярлык экрана в автозапуске сеанса: нет"
  fi

  if ! command -v nmcli >/dev/null 2>&1; then
    echo "  • точка доступа Wi-Fi: nmcli не найден — не проверяю"
  elif ! ap_exists; then
    echo "  • точка доступа Wi-Fi robot-ap: не настроена"
  elif [[ "$(ap_autoconnect)" == "yes" ]]; then
    echo "  • точка доступа Wi-Fi robot-ap: автоподъём ВКЛЮЧЁН"
    auto_count=$((auto_count + 1))
  else
    echo "  • точка доступа Wi-Fi robot-ap: автоподъём выключен"
  fi

  echo
  if [[ $auto_count -eq 0 ]]; then
    echo "Итог: само ничего не запускается."
    print_manual_hint
  else
    echo "Итог: автозапуск включён ($auto_count шт.)."
    echo "Выключить: ./scripts/autostart.sh off"
  fi
  exit 0
fi

# --------------------------------------------------------------------- off --- #
if [[ $ACTION == "off" || $ACTION == "disable" ]]; then
  log "выключаю автозапуск приложения…"

  if have_systemd; then
    for index in "${!UNITS[@]}"; do
      unit="${UNITS[$index]}"
      title="${UNIT_TITLES[$index]}"
      if ! unit_installed "$unit"; then
        echo "  служба $unit не установлена — пропускаю"
        continue
      fi
      if $SUDO systemctl disable --now "$unit.service" >/dev/null 2>&1; then
        log "$title: автозапуск выключен, служба остановлена"
      else
        warn "$title: не удалось выключить — проверь: systemctl status $unit"
      fi
    done
  else
    warn "systemd недоступен — службы не выключаю"
  fi

  removed=0
  for name in "${AUTOSTART_NAMES[@]}"; do
    if [[ -f "$AUTOSTART_DIR/$name.desktop" ]]; then
      rm -f "$AUTOSTART_DIR/$name.desktop"
      log "ярлык автозапуска сеанса удалён: $AUTOSTART_DIR/$name.desktop"
      removed=1
    fi
  done
  [[ $removed -eq 0 ]] && echo "  ярлыка автозапуска сеанса не было — пропускаю"

  if ! command -v nmcli >/dev/null 2>&1; then
    echo "  nmcli не найден — точку доступа не трогаю"
  elif ! ap_exists; then
    echo "  точка доступа robot-ap не настроена — пропускаю"
  else
    if $SUDO nmcli connection modify "$AP_CONN" connection.autoconnect no >/dev/null 2>&1; then
      log "точка доступа robot-ap: автоподъём выключен (поднимать вручную: sudo ./scripts/setup_ap.sh)"
    else
      warn "точку доступа robot-ap не удалось изменить — выключи руками: sudo nmcli connection modify $AP_CONN connection.autoconnect no"
    fi
  fi

  log "готово: после загрузки робота приложение само не запускается"
  print_manual_hint
  exit 0
fi

# ---------------------------------------------------------------------- on --- #
if [[ $ACTION == "on" || $ACTION == "enable" ]]; then
  log "включаю автозапуск приложения…"
  enabled_any=0

  if have_systemd; then
    for index in "${!UNITS[@]}"; do
      unit="${UNITS[$index]}"
      title="${UNIT_TITLES[$index]}"
      if ! unit_installed "$unit"; then
        echo "  служба $unit не установлена — поставь: sudo ./scripts/install_ubuntu.sh --autostart"
        continue
      fi
      if $SUDO systemctl enable --now "$unit.service" >/dev/null 2>&1; then
        log "$title: автозапуск включён"
        enabled_any=1
      else
        warn "$title: не удалось включить — нужен root: sudo ./scripts/autostart.sh on"
      fi
    done
  else
    warn "systemd недоступен — службы не включаю"
  fi

  if [[ -x ./scripts/make_shortcut.sh ]]; then
    ./scripts/make_shortcut.sh --autostart
    enabled_any=1
  fi

  if ! command -v nmcli >/dev/null 2>&1; then
    echo "  nmcli не найден — точку доступа не трогаю"
  elif ! ap_exists; then
    echo "  точка доступа robot-ap не настроена — поднять: sudo ./scripts/setup_ap.sh --autostart"
  elif $SUDO nmcli connection modify "$AP_CONN" connection.autoconnect yes \
       && $SUDO nmcli connection modify "$AP_CONN" connection.autoconnect-priority 10 >/dev/null 2>&1; then
    log "точка доступа robot-ap: автоподъём включён"
  else
    warn "точку доступа robot-ap не удалось изменить — включи руками: sudo nmcli connection modify $AP_CONN connection.autoconnect yes"
  fi

  echo
  if [[ $enabled_any -eq 0 ]]; then
    warn "ничего включить не удалось — см. сообщения выше"
    exit 1
  fi
  log "готово: после загрузки робота пульт и экран поднимаются сами"
  echo "Проверить: ./scripts/autostart.sh status"
  echo "Логи:      journalctl -u robot-control -f   ·   journalctl -u robot-kiosk -f"
  exit 0
fi

die "неизвестное действие: $ACTION (status | off | on)"
