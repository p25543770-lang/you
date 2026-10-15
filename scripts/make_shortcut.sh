#!/usr/bin/env bash
# Создаёт ровно один ярлык на рабочем столе: «Запустить робот».
# Он поднимает вместе веб-пульт 8080 и бортовой экран 8090.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"
NAME="robot-control"
AUTOSTART=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --autostart)    AUTOSTART=1; shift ;;
    --no-autostart) AUTOSTART=0; shift ;;
    -h|--help)
      echo "Использование: ./scripts/make_shortcut.sh [--autostart|--no-autostart]"
      echo "Создаёт один ярлык для совместного запуска пульта и киоска."
      exit 0
      ;;
    *) echo "неизвестный аргумент: $1 (см. --help)" >&2; exit 1 ;;
  esac
done

DESK=""
if command -v xdg-user-dir >/dev/null 2>&1; then
  DESK="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
[[ -z "$DESK" || ! -d "$DESK" ]] && DESK="$HOME/Desktop"
mkdir -p "$DESK"

make_entry() {
  local target="$1" terminal="$2" autostart="${3:-no}"
  cat > "$target" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Запустить робот
Comment=Одновременно запускает веб-пульт и бортовой экран робота
Exec="$ROOT/scripts/start_desktop.sh"
Icon=applications-system
Terminal=$terminal
Categories=Utility;
EOF
  if [[ "$autostart" == "yes" ]]; then
    printf 'X-GNOME-Autostart-enabled=true\nStartupNotify=false\n' >> "$target"
  fi
  chmod +x "$target"
}

# Удаляем старые раздельные значки/автозапуски, чтобы после обновления
# на рабочем столе остался только один общий ярлык.
for old in robot-console robot-kiosk; do
  rm -f "$DESK/$old.desktop" \
        "$HOME/.local/share/applications/$old.desktop" \
        "$HOME/.config/autostart/$old.desktop"
done
rm -f "$HOME/.local/share/applications/$NAME.desktop"

make_entry "$DESK/$NAME.desktop" true
if command -v gio >/dev/null 2>&1; then
  gio set "$DESK/$NAME.desktop" metadata::trusted true 2>/dev/null || true
fi

AUTOSTART_DIR="$HOME/.config/autostart"
if [[ $AUTOSTART -eq 1 ]]; then
  mkdir -p "$AUTOSTART_DIR"
  make_entry "$AUTOSTART_DIR/$NAME.desktop" false yes
  echo "Автозапуск включён: веб-пульт и киоск стартуют после входа в систему."
else
  rm -f "$AUTOSTART_DIR/$NAME.desktop"
  echo "Автозапуск выключен; запуск — одним ярлыком вручную."
fi

echo "Готово. Единственный ярлык: $DESK/$NAME.desktop"
echo "Двойной щелчок запускает оба интерфейса; Ctrl+C закрывает их."
