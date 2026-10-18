#!/usr/bin/env bash
# patch-mod.sh — добавляет minGameVersion в mod.json/mod.hjson мода Campaign Threat Radar.
# Использование: ./patch-mod.sh <путь-к-mod.json-или-папке-мода>
#
# Примеры:
#   ./patch-mod.sh ~/.local/share/Mindustry/mods/campaign-threat-radar/mod.json
#   ./patch-mod.sh ~/.local/share/Mindustry/mods/campaign-threat-radar   (папка)

set -euo pipefail

TARGET_MIN="154"   # подсказка из Mindustry на скриншоте
LATEST_MIN="154"   # последнее значение, которое просит игра (build 154+ / v8)

if [[ $# -lt 1 ]]; then
  echo "Использование: $0 <path-to-mod.json-or-mod-folder>"
  exit 1
fi

PATHARG="$1"

if [[ -d "$PATHARG" ]]; then
# Ищем mod.json или mod.hjson в корне папки
if [[ -f "$PATHARG/mod.json" ]]; then
  FILE="$PATHARG/mod.json"
elif [[ -f "$PATHARG/mod.hjson" ]]; then
  FILE="$PATHARG/mod.hjson"
elif compgen -G "$PATHARG"/*/mod.json > /dev/null; then
  # импортнутый мод может лежать во вложенной папке с тем же именем
  FILE=$(compgen -G "$PATHARG"/*/mod.json | head -n1)
else
  echo "В папке $PATHARG не найден mod.json или mod.hjson"
  exit 2
fi
else
  FILE="$PATHARG"
fi

if [[ ! -f "$FILE" ]]; then
  echo "Файл не найден: $FILE"
  exit 2
fi

# Делаем бэкап
BAK="$FILE.bak.$(date +%Y%m%d-%H%M%S)"
cp "$FILE" "$BAK"
echo "Бэкап: $BAK"

if grep -qiE '"?minGameVersion"?\s*[:=]' "$FILE"; then
  echo "Поле minGameVersion уже присутствует; заменяю значение на $TARGET_MIN."
  python3 - "$FILE" "$TARGET_MIN" <<'PY'
import re, sys
p, val = sys.argv[1], sys.argv[2]
s = open(p, encoding='utf-8').read()
# json: "minGameVersion": "123"
s = re.sub(r'("minGameVersion"\s*:\s*)"?[^",}\n]+"?', r'\1"' + val + '"', s, flags=re.I)
# hjson: minGameVersion: "123"  /  minGameVersion: 123
s = re.sub(r'(?m)^(\s*minGameVersion\s*:\s*)"?[^",}\n]+"?', r'\1"' + val + '"', s, flags=re.I)
open(p, 'w', encoding='utf-8').write(s)
PY
else
  echo "Добавляю поле minGameVersion: \"$TARGET_MIN\""
  python3 - "$FILE" "$TARGET_MIN" <<'PY'
import sys, re
p, val = sys.argv[1], sys.argv[2]
s = open(p, encoding='utf-8').read()
# Стиль: ключи в кавычках → JSON (иначе HJSON без кавычек у ключей)
use_quotes = bool(re.search(r'^\s*\{?\s*"', s))
if use_quotes:
    insert = '"minGameVersion": "' + val + '",'
else:
    insert = 'minGameVersion: "' + val + '"'

# 1) Если есть открывающая фигурная скобка — вставляем после неё на новой строке.
s2, n = re.subn(r'(\{\s*\r?\n)', r'\1  ' + insert + '\n', s, count=1)
if n == 0:
    s2, n = re.subn(r'(\{)', r'\1' + insert + '\n', s, count=1)

# 2) Если открывающей скобки нет (HJSON с неявным root-объектом) — вставляем
#    ПЕРЕД первым ключом (name, displayName и т.п.) на новой строке.
if n == 0:
    s2, n = re.subn(r'()', insert + '\n', s, count=1)  # в начало файла

open(p, 'w', encoding='utf-8').write(s2)
PY
fi

echo "Готово. Перезапустите Mindustry, чтобы изменения подействовали."
