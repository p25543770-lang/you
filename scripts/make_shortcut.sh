#!/usr/bin/env bash
#
# Создаёт ярлык «Робот-пульт» в меню приложений и на рабочем столе Ubuntu.
# Ярлык запускает scripts/start.sh в терминале: автообновление с git,
# зависимости, старт сервера, открытие браузера.
#
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"
NAME="robot-console"

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

# --- меню приложений --- #
mkdir -p "$HOME/.local/share/applications"
make_desktop "$HOME/.local/share/applications/$NAME.desktop"
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$HOME/.local/share/applications" 2>/dev/null || true
fi
echo "Ярлык в меню приложений: $HOME/.local/share/applications/$NAME.desktop"

# --- рабочий стол --- #
DESK=""
if command -v xdg-user-dir >/dev/null 2>&1; then
  DESK="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
[[ -z "$DESK" || ! -d "$DESK" ]] && DESK="$HOME/Desktop"
if mkdir -p "$DESK" 2>/dev/null; then
  make_desktop "$DESK/$NAME.desktop"
  # GNOME требует пометить файл доверенным, иначе двойной клик не сработает
  if command -v gio >/dev/null 2>&1; then
    gio set "$DESK/$NAME.desktop" metadata::trusted true 2>/dev/null || true
  fi
  echo "Ярлык на рабочем столе: $DESK/$NAME.desktop"
fi

echo
echo "Готово. Кликните «Робот-пульт» — откроется терминал, обновится с git,"
echo "подтянет зависимости, запустит сервер и откроет сайт в браузере."
