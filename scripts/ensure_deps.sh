#!/usr/bin/env bash
# Создаёт .venv и ставит зависимости. Работает БЕЗ интернета: pip и все
# пакеты лежат в vendor/ (колёса), онлайн-установка — если сеть есть.
set -euo pipefail
cd "$(dirname "$0")/.."

log() { printf '\033[1;32m[deps]\033[0m %s\n' "$*"; }

python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 9) else 1)' 2>/dev/null \
  || { echo "Нужен Python 3.9+ (у вас $(python3 --version 2>&1))." >&2; exit 1; }

if [[ ! -x .venv/bin/python ]]; then
  log "создаю .venv…"
  if ! python3 -m venv .venv 2>/dev/null && ! python3 -m venv --without-pip .venv 2>/dev/null; then
    if command -v apt >/dev/null 2>&1; then
      log "ставлю python3-venv через apt…"
      if command -v sudo >/dev/null 2>&1 && [[ $EUID -ne 0 ]]; then SUDO=sudo; else SUDO=; fi
      $SUDO apt update -y && $SUDO apt install -y python3-venv
      python3 -m venv --without-pip .venv
    else
      echo "python3-venv не установлен, apt недоступен." >&2
      exit 1
    fi
  fi
fi

# pip в venv: сначала ensurepip, иначе — колесо pip из vendor/ (офлайн)
if [[ ! -x .venv/bin/pip ]]; then
  .venv/bin/python -m ensurepip --default-pip 2>/dev/null || true
fi
if [[ ! -x .venv/bin/pip ]]; then
  PIPWHL="$(ls vendor/pip-*.whl 2>/dev/null | head -1 || true)"
  if [[ -n "$PIPWHL" ]]; then
    log "ставлю pip из vendor (офлайн)…"
    PYTHONPATH="$PIPWHL" .venv/bin/python -m pip install -q --no-index --find-links vendor pip
  fi
fi
[[ -x .venv/bin/pip ]] || { echo "Не удалось поставить pip в .venv." >&2; exit 127; }

# Какие модули обязаны импортироваться в .venv: flask/gunicorn — веб-пульт,
# serial (pyserial) — USB-лидар и UART-модули. Проверяем именно импорт, а не
# только хэш requirements.txt: при прерванной установке хэш уже записан, а
# pyserial нет — и тогда лидар «не работает» молча, хотя скрипт установки
# отрапортовал об успехе.
REQUIRED_MODULES=(flask gunicorn serial)

missing_deps() {
  local missing=() module
  for module in "${REQUIRED_MODULES[@]}"; do
    .venv/bin/python -c "import $module" >/dev/null 2>&1 || missing+=("$module")
  done
  printf '%s\n' "${missing[@]}"
}

install_deps() {
  log "ставлю зависимости…"
  if ! .venv/bin/pip install -q -r requirements.txt 2>/dev/null; then
    log "сети нет — ставлю из vendor (офлайн)…"
    .venv/bin/pip install -q --no-index --find-links vendor -r requirements.txt
  fi
}

REQ_HASH="$(sha256sum requirements.txt | awk '{print $1}')"
CACHED="$(cat .venv/.req-hash 2>/dev/null || true)"
MISSING="$(missing_deps)"
if [[ "$REQ_HASH" != "$CACHED" ]]; then
  install_deps
elif [[ -n "$MISSING" ]]; then
  log "в .venv не хватает: $(echo "$MISSING" | tr '\n' ' ')— ставлю заново"
  install_deps
else
  log "зависимости уже установлены"
fi

MISSING="$(missing_deps)"
if [[ -n "$MISSING" ]]; then
  echo "Не удалось установить в .venv: $(echo "$MISSING" | tr '\n' ' ')" >&2
  echo "Без pyserial USB-лидар не читается. Поставь вручную и повтори запуск:" >&2
  echo "  .venv/bin/pip install --no-index --find-links vendor -r requirements.txt" >&2
  exit 1
fi
if [[ "$REQ_HASH" != "$CACHED" ]]; then
  echo "$REQ_HASH" > .venv/.req-hash
fi
log "проверка: $(.venv/bin/python -c 'import serial
from importlib.metadata import version
print("flask", version("flask"), "· pyserial", serial.VERSION)')"
