#!/usr/bin/env bash
# Создаёт учётку оператора БЕЗ вопросов. Вызывается установщиком и start.sh,
# когда спросить пароль интерактивно нельзя/не нужно.
#
# По умолчанию — admin / 1234 (как просил заказчик). Переопределяется:
#   RC_OPERATOR_USER=robot RC_OPERATOR_PASSWORD='свой-пароль' ./scripts/create_default_operator.sh
#   ./scripts/create_default_operator.sh robot          # только другое имя
#   RC_OPERATOR_PASSWORD=random ./scripts/create_default_operator.sh   # случайный пароль
#
# Логин/пароль дублируются в data/credentials.txt (права 600, в git не попадает).
set -euo pipefail
cd "$(dirname "$0")/.."

USER_NAME="${RC_OPERATOR_USER:-${1:-admin}}"
PASS="${RC_OPERATOR_PASSWORD:-1234}"

# Случайный пароль — если явно попросили.
if [[ "$PASS" == "random" ]]; then
  PASS="$(python3 -c '
import secrets, string
abc = string.ascii_letters + string.digits
print("".join(secrets.choice(abc) for _ in range(12)))
')"
fi

MIN_LEN=${#PASS}
[[ $MIN_LEN -lt 4 ]] && MIN_LEN=4

printf '%s\n%s\n' "$PASS" "$PASS" | \
  .venv/bin/python scripts/create_operator.py \
    --username "$USER_NAME" --min-length "$MIN_LEN" 2>/dev/null

mkdir -p data
{
  echo "login: $USER_NAME"
  echo "password: $PASS"
} > data/credentials.txt
chmod 600 data/credentials.txt

echo "Учётка оператора: $USER_NAME / $PASS"
echo "  (копия — в data/credentials.txt)"
if [[ ${#PASS} -lt 8 ]]; then
  echo "  ⚠ Пароль короткий. Для точки доступа, открытой всем вокруг, лучше длиннее:"
  echo "    RC_OPERATOR_PASSWORD='свой-пароль' ./scripts/create_default_operator.sh"
fi
