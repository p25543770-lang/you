#!/usr/bin/env bash
#
# Поднимает на роботе (Ubuntu) точку доступа Wi-Fi. Телефон/ноутбук
# подключается к этой сети и открывает веб-пульт робота в браузере.
#
# Работает через NetworkManager (nmcli) — стандарт для Ubuntu Desktop/Server.
# Нужен Wi-Fi-адаптер с поддержкой режима AP (почти все современные).
#
# Использование:
#   sudo ./scripts/setup_ap.sh                     # поднять (адаптер найдётся сам)
#   sudo ./scripts/setup_ap.sh --autostart         # + поднимать при каждой загрузке
#   sudo ./scripts/setup_ap.sh --iface wlp3s0      # конкретный адаптер
#   sudo ./scripts/setup_ap.sh --status            # что сейчас поднято, пароль, адрес
#   sudo ./scripts/setup_ap.sh --down              # выключить точку доступа
#   sudo RC_AP_SSID=ROBOT RC_AP_PSK='мой-пароль-8+' -E ./scripts/setup_ap.sh
#
set -euo pipefail
cd "$(dirname "$0")/.."

CON_NAME="robot-ap"
IFACE=""
ACTION="up"
AUTOSTART=0

RC_AP_SSID="${RC_AP_SSID:-ROBOT-CONSOLE}"
RC_AP_PSK="${RC_AP_PSK:-}"
RC_AP_IP="${RC_AP_IP:-10.42.0.1/24}"

log()  { printf '\033[1;32m[ap]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[ap]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[ap]\033[0m %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface)     IFACE="${2:-}"; shift 2 ;;
    --down)      ACTION="down"; shift ;;
    --status)    ACTION="status"; shift ;;
    --autostart) AUTOSTART=1; shift ;;
    -h|--help)   sed -n '2,16p' "$0"; exit 0 ;;
    *) die "неизвестный аргумент: $1 (см. --help)" ;;
  esac
done

command -v nmcli >/dev/null 2>&1 \
  || die "nmcli не найден. Поставьте NetworkManager: sudo apt install network-manager"

GATEWAY="${RC_AP_IP%%/*}"
PORT="${RC_PORT:-8080}"
CRED_FILE="data/wifi.txt"

# --- пароль: берём сохранённый, иначе генерируем ----------------------------- #
load_psk() {
  [[ -n "$RC_AP_PSK" ]] && return 0
  if [[ -f "$CRED_FILE" ]]; then
    RC_AP_PSK="$(awk -F': *' '/^Пароль/{print $2; exit}' "$CRED_FILE" 2>/dev/null || true)"
    [[ -n "$RC_AP_PSK" ]] && { log "пароль взят из $CRED_FILE"; return 0; }
  fi
  RC_AP_PSK="$(head -c 16 /dev/urandom | base64 | tr -dc 'a-zA-Z0-9' | head -c 12)"
  GENERATED=1
}

save_psk() {
  mkdir -p data
  {
    echo "Сеть Wi-Fi: $RC_AP_SSID"
    echo "Пароль: $RC_AP_PSK"
    echo "Адрес пульта: http://${GATEWAY}:${PORT}/"
    echo "Обновлено: $(date '+%Y-%m-%d %H:%M')"
  } > "$CRED_FILE"
  chmod 600 "$CRED_FILE"
  log "пароль Wi-Fi сохранён в $CRED_FILE"
}

# --- --status ---------------------------------------------------------------- #
if [[ $ACTION == "status" ]]; then
  if nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "$CON_NAME"; then
    log "точка доступа РАБОТАЕТ"
  else
    warn "точка доступа сейчас не поднята"
  fi
  nmcli -g 802-11-wireless.ssid,802-11-wireless-security.psk connection show "$CON_NAME" 2>/dev/null \
    | awk -F: '{printf "  Сеть   : %s\n  Пароль : %s\n", $1, $2}' || true
  echo "  Адрес  : http://${GATEWAY}:${PORT}/"
  [[ -f "$CRED_FILE" ]] && { echo; echo "  ($CRED_FILE)"; cat "$CRED_FILE" | sed 's/^/  /'; }
  exit 0
fi

# --- --down ------------------------------------------------------------------ #
if [[ $ACTION == "down" ]]; then
  nmcli connection down "$CON_NAME" 2>/dev/null \
    && log "точка доступа выключена" \
    || log "точка доступа не была запущена"
  exit 0
fi

[[ $EUID -eq 0 ]] || die "нужны права root: sudo ./scripts/setup_ap.sh"

# --- находим Wi-Fi-адаптер --------------------------------------------------- #
if [[ -z "$IFACE" ]]; then
  IFACE="$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi" && $1!="" {print $1; exit}')"
fi
[[ -n "$IFACE" ]] || die "Wi-Fi-адаптер не найден. Укажите его: sudo ./scripts/setup_ap.sh --iface <имя>"
log "адаптер: $IFACE"

# --- предупреждение: адаптер уже в другой сети ------------------------------- #
CUR="$(nmcli -t -f DEVICE,CONNECTION device | awk -F: -v i="$IFACE" '$1==i{print $2; exit}')"
if [[ -n "$CUR" && "$CUR" != "$CON_NAME" && "$CUR" != "--" ]]; then
  warn "адаптер $IFACE сейчас в сети «$CUR» — она отключится, когда поднимется точка доступа."
  warn "(интернет по этому Wi-Fi пропадёт; по кабелю — останется)"
fi

# --- поддержка режима AP ----------------------------------------------------- #
if command -v iw >/dev/null 2>&1; then
  if ! iw list 2>/dev/null | grep -A20 'Supported interface modes' | grep -q '\* AP'; then
    die "адаптер $IFACE не поддерживает режим точки доступа (AP)"
  fi
  log "режим AP поддерживается"
else
  warn "iw не установлен — проверку поддержки AP пропускаю (sudo apt install iw)"
fi

load_psk
[[ ${#RC_AP_PSK} -ge 8 ]] || die "пароль Wi-Fi короче 8 символов — WPA2 его не примет"

# --- captive portal: телефон сам откроет пульт при подключении --------------- #
DN_CONF="/etc/NetworkManager/dnsmasq-shared.d/robot.conf"
if [[ ! -f "$DN_CONF" ]]; then
  mkdir -p /etc/NetworkManager/dnsmasq-shared.d
  echo "address=/#/$GATEWAY" > "$DN_CONF"
  log "captive portal включён ($DN_CONF)"
fi

# --- создаём соединение ------------------------------------------------------ #
nmcli connection delete "$CON_NAME" >/dev/null 2>&1 || true

AC=no
[[ $AUTOSTART -eq 1 ]] && AC=yes

nmcli connection add \
  type wifi ifname "$IFACE" con-name "$CON_NAME" autoconnect "$AC" \
  ssid "$RC_AP_SSID" \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  ipv4.method shared \
  ipv4.addresses "$RC_AP_IP" \
  >/dev/null || die "не удалось создать соединение точки доступа"

nmcli connection modify "$CON_NAME" \
  802-11-wireless-security.key-mgmt wpa-psk \
  802-11-wireless-security.psk "$RC_AP_PSK" \
  >/dev/null || die "не удалось задать пароль WPA2"

if [[ $AUTOSTART -eq 1 ]]; then
  nmcli connection modify "$CON_NAME" connection.autoconnect-priority 10 >/dev/null 2>&1 || true
  log "автоподъём при загрузке включён"
fi

log "поднимаю точку доступа…"
nmcli connection up "$CON_NAME" >/dev/null \
  || die "точка доступа не поднялась. Проверьте: sudo journalctl -u NetworkManager -n 30"

save_psk

log "готово"
echo
echo "  Сеть Wi-Fi : $RC_AP_SSID"
echo "  Пароль     : $RC_AP_PSK${GENERATED:+   (сгенерирован)}"
echo "  Пульт      : http://${GATEWAY}:${PORT}/"
echo
echo "  1. На телефоне/ноутбуке подключитесь к сети «$RC_AP_SSID»."
echo "  2. Откройте в браузере http://${GATEWAY}:${PORT}/ (http, не https)."
echo "  3. Войдите: логин и пароль оператора — в data/credentials.txt"
echo
echo "  Пароль Wi-Fi сохранён в $CRED_FILE"
echo "  Статус    : sudo ./scripts/setup_ap.sh --status"
echo "  Выключить : sudo ./scripts/setup_ap.sh --down"
