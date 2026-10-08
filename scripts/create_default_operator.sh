#!/usr/bin/env bash
# Создаёт учётку оператора БЕЗ вопросов: пароль генерируется и сохраняется в
# data/credentials.txt (права 600, в git не попадает). Вызывается установщиком
# и start.sh, когда спросить пароль интерактивно нельзя/не нужно.
set -euo pipefail
cd "$(dirname "$0")/.."

USER_NAME="${1:-operator}"

PASS="$(python3 -c '
import secrets, string
abc = string.ascii_letters + string.digits
print("".join(secrets.choice(abc) for _ in range(12)))
')"

printf '%s\n%s\n' "$PASS" "$PASS" | \
  .venv/bin/python scripts/create_operator.py --username "$USER_NAME" 2>/dev/null

mkdir -p data
{
  echo "login: $USER_NAME"
  echo "password: $PASS"
} > data/credentials.txt
chmod 600 data/credentials.txt

echo "Учётка создана сама: $USER_NAME / $PASS"
echo "  (копия — в data/credentials.txt, покажи/смени при необходимости)"
