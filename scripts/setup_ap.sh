#!/usr/bin/env bash
#
# Поднимает на машине с Ubuntu точку доступа Wi-Fi, к которой подключается
# телефон/ноутбук, и по которой открывается веб-пульт робота.
#
# Работает через NetworkManager (nmcli) — стандарт для Ubuntu Desktop/Server.
# Требуется Wi-Fi-адаптер с поддержкой режима AP.
#
# Использование:
#   sudo ./scripts/setup_ap.sh                      # автоопределение адаптера
#   sudo ./scripts/setup_ap.sh --iface wlp3s0       # конкретный адаптер
#   RC_AP_SSID=Robot RC_AP_PSK='оченьсекретно' sudo -E ./scripts/setup_ap.sh
#   sudo ./scripts/setup_ap.sh --down               # выключить точку доступа
#
set -euo pipefail

CON_NAME="robot-ap"
IFACE=""
ACTION="up"

RC_AP_SSID="${RC_AP_SSID:-ROBOT-CONSOLE}"
RC_AP_PSK="${RC_AP_PSK:-}"
RC_AP_IP="${RC_AP_IP:-10.42.0.1/24}"

log() { printf '\033[1;32m[ap]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[ap]\033[0m %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface) IFACE="${2:-}"; shift 2 ;;
    --down)  ACTION="down"; shift ;;
    -h|--help)
      sed -n '2,20p' "$0"; exit 0 ;;
    *) die "неизвестный аргумент: $1" ;;
  esac
done

command -v nmcli >/dev/null 2>&1 || die "nmcli не найден. Установите NetworkManager: apt install network-manager"

if [[ $ACTION == "down" ]]; then
  nmcli connection down "$CON_NAME" 2>/dev/null && log "точка доступа выключена" || log "точка доступа не была запущена"
  exit 0
fi

if [[ $EUID -ne 0 ]]; then
  die "нужны права root: sudo ./scripts/setup_ap.sh"
fi

# --- находим Wi-Fi-адаптер -------------------------------------------------- #
if [[ -z "$IFACE" ]]; then
  IFACE="$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')"
fi
[[ -n "$IFACE" ]] || die "Wi-Fi-адаптер не найден. Укажите его через --iface"
log "адаптер: $IFACE"

# --- проверяем поддержку режима AP ------------------------------------------ #
if command -v iw >/dev/null 2>&1; then
  if ! iw list 2>/dev/null | grep -A20 'Supported interface modes' | grep -q '\* AP'; then
    die "адаптер $IFACE не поддерживает режим точки доступа (AP)"
  fi
  log "режим AP поддерживается"
else
  log "iw не установлен — пропускаю проверку поддержки AP (apt install iw)"
fi

# --- пароль ------------------------------------------------------------------ #
if [[ -z "$RC_AP_PSK" ]]; then
  RC_AP_PSK="$(head -c 12 /dev/urandom | base64 | tr -dc 'a-zA-Z0-9' | head -c 12)"
  GENERATED=1
fi
if [[ ${#RC_AP_PSK} -lt 8 ]]; then
  die "пароль Wi-Fi короче 8 символов — WPA2 его не примет"
fi

# --- пересоздаём соединение -------------------------------------------------- #
nmcli connection delete "$CON_NAME" >/dev/null 2>&1 || true

nmcli connection add \
  type wifi ifname "$IFACE" con-name "$CON_NAME" autoconnect no \
  ssid "$RC_AP_SSID" \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  ipv4.method shared \
  ipv4.addresses "$RC_AP_IP" \
  >/dev/null

nmcli connection modify "$CON_NAME" \
  802-11-wireless-security.key-mgmt wpa-psk \
  802-11-wireless-security.psk "$RC_AP_PSK" \
  >/dev/null

nmcli connection up "$CON_NAME" >/dev/null

GATEWAY="${RC_AP_IP%%/*}"
PORT="${RC_PORT:-8080}"

log "готово"
echo
echo "  Сеть Wi-Fi : $RC_AP_SSID"
echo "  Пароль     : $RC_AP_PSK${GENERATED:+   (сгенерирован автоматически)}"
echo "  Адрес      : http://${GATEWAY}:${PORT}/"
echo
echo "  Подключитесь к этой сети с телефона/ноутбука и откройте адрес выше."
echo "  Выключить: sudo ./scripts/setup_ap.sh --down"
