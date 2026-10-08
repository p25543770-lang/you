#!/usr/bin/env bash
#
# Минимальный запуск веб-пульта: сам ставит зависимости (офлайн из vendor/,
# если сети нет) и поднимает сервер. Ничего лишнего — без git-обновления,
# ufw и ярлыков (это делает scripts/install_ubuntu.sh).
#
# Сервер при старте сам создаёт .env и учётку admin/1234, если их нет
# (см. run.py), поэтому команда работает и на пустом месте.
#
#   ./scripts/serve.sh
#
set -euo pipefail
cd "$(dirname "$0")/.."

./scripts/ensure_deps.sh
exec .venv/bin/python run.py "$@"
