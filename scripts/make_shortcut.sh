#!/usr/bin/env bash
#
# Создаёт ярлыки в меню приложений и на рабочем столе Ubuntu:
#   * «Робот-пульт»         — scripts/start.sh (пульт, порт 8080);
#   * «Экран робота (киоск)» — scripts/kiosk.sh (бортовой экран, порт 8090).
# Плюс ставит бортовой экран в автозапуск сеанса: после включения робота
# экран открывается сам, без команд в терминале.
#
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"
NAME="robot-console"
KIOSK_NAME="robot-kiosk"

make_desktop() {
  local target="$1"
  cat > "$target" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Робот-пульт
Comment=Автообновление с git и запуск пульта оператора
Exec=$ROOT/scripts/start_desktop.sh
Icon=utilities-terminal
Terminal=true
Categories=Utility;
EOF
  chmod +x "$target"
}

# --- бортовой экран робота (киоск, порт 8090) --- #
make_kiosk_desktop() {
  local target="$1" autostart="${2:-no}"
  {
    cat <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Экран робота (киоск)
Comment=Бортовой экран: АКБ, шасси, лидар — порт 8090
Exec=$ROOT/scripts/kiosk.sh
Icon=video-display
EOF
    if [[ "$autostart" == "yes" ]]; then
      # Тихо, без окна терминала: экран должен появиться сам после входа.
      printf 'Terminal=false\nX-GNOME-Autostart-enabled=true\nStartupNotify=false\n'
    else
      printf 'Terminal=true\nCategories=Utility;\n'
    fi
  } > "$target"
  chmod +x "$target"
}

# --- меню приложений --- #
mkdir -p "$HOME/.local/share/applications"
make_desktop "$HOME/.local/share/applications/$NAME.desktop"
make_kiosk_desktop "$HOME/.local/share/applications/$KIOSK_NAME.desktop"
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$HOME/.local/share/applications" 2>/dev/null || true
fi
echo "Ярлык в меню приложений: $HOME/.local/share/applications/$NAME.desktop"
echo "Ярлык экрана робота:    $HOME/.local/share/applications/$KIOSK_NAME.desktop"

# --- автозапуск экрана при входе в систему --- #
# Чтобы бортовой экран открывался сам сразу после включения робота, как только
# поднялась графическая сессия. Сервер на 8090 при этом может уже работать
# службой robot-kiosk — тогда команда только откроет браузер.
AUTOSTART_DIR="$HOME/.config/autostart"
mkdir -p "$AUTOSTART_DIR"
make_kiosk_desktop "$AUTOSTART_DIR/$KIOSK_NAME.desktop" yes
echo "Автозапуск экрана робота: $AUTOSTART_DIR/$KIOSK_NAME.desktop"

# --- рабочий стол --- #
DESK=""
if command -v xdg-user-dir >/dev/null 2>&1; then
  DESK="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
[[ -z "$DESK" || ! -d "$DESK" ]] && DESK="$HOME/Desktop"
if mkdir -p "$DESK" 2>/dev/null; then
  make_desktop "$DESK/$NAME.desktop"
  make_kiosk_desktop "$DESK/$KIOSK_NAME.desktop"
  # GNOME требует пометить файл доверенным, иначе двойной клик не сработает
  if command -v gio >/dev/null 2>&1; then
    gio set "$DESK/$NAME.desktop" metadata::trusted true 2>/dev/null || true
    gio set "$DESK/$KIOSK_NAME.desktop" metadata::trusted true 2>/dev/null || true
  fi
  echo "Ярлык на рабочем столе: $DESK/$NAME.desktop"
  echo "Ярлык экрана:            $DESK/$KIOSK_NAME.desktop"
fi

echo
echo "Готово:"
echo "  • «Робот-пульт» — пульт оператора на порту 8080."
echo "  • «Экран робота (киоск)» — бортовой экран на порту 8090;"
echo "    он же добавлен в автозапуск и откроется сам после входа в систему."
