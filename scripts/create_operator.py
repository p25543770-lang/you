#!/usr/bin/env python3
"""Создание и правка учётки оператора.

Пароль спрашивается скрытым вводом и в файл не попадает — сохраняется только
PBKDF2-хеш. Файл учёткок (по умолчанию data/operators.json) в git не кладётся.

Примеры:
    python3 scripts/create_operator.py --username operator
    python3 scripts/create_operator.py --list
    python3 scripts/create_operator.py --username operator --disable
"""

from __future__ import annotations

import argparse
import getpass
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_control.auth import OperatorStore  # noqa: E402
from robot_control.config import Config  # noqa: E402


def read_password(min_length: int = 8) -> str:
    first = getpass.getpass("Пароль оператора: ")
    second = getpass.getpass("Повторите пароль: ")
    if first != second:
        raise SystemExit("Пароли не совпадают, ничего не изменено.")
    if len(first) < min_length:
        raise SystemExit(f"Пароль короче {min_length} символов — так нельзя.")
    if min_length < 8:
        print(
            f"ВНИМАНИЕ: пароль короче 8 символов. "
            f"Для точки доступа, открытой всем вокруг, это опасно."
        )
    return first


def main() -> int:
    parser = argparse.ArgumentParser(description="Управление учётками операторов")
    parser.add_argument("--username", help="логин оператора")
    parser.add_argument("--file", help="файл учёткок (по умолчанию из настроек)")
    parser.add_argument("--list", action="store_true", help="показать список операторов")
    parser.add_argument("--disable", action="store_true", help="отключить учётку")
    parser.add_argument("--enable", action="store_true", help="включить учётку")
    parser.add_argument(
        "--min-length",
        type=int,
        default=8,
        help="минимальная длина пароля (по умолчанию 8)",
    )
    args = parser.parse_args()

    config = Config.from_env()
    path = Path(args.file) if args.file else config.operators_file
    store = OperatorStore(path)

    if args.list:
        if not store.exists():
            print(f"Файла {path} нет — операторы ещё не созданы.")
            return 1
        for username in store.usernames():
            operator = store.get(username)
            state = "активен" if operator and operator.active else "отключён"
            print(f"{operator.username:20s} роль={operator.role:9s} {state}")
        return 0

    if not args.username:
        parser.error("нужно указать --username (или использовать --list)")

    operators = store.load()
    key = args.username.strip().lower()

    if args.disable or args.enable:
        if key not in operators:
            raise SystemExit(f"Оператора {args.username} нет в {path}")
        operator = operators[key]
        operators[key] = type(operator)(
            username=operator.username,
            role=operator.role,
            password_hash=operator.password_hash,
            active=args.enable,
        )
        store.save(operators.values())
        print(f"{'Включён' if args.enable else 'Отключён'}: {operator.username}")
        return 0

    password = read_password(min_length=args.min_length)
    operator = store.add_or_update(
        args.username, password, iterations=config.pbkdf2_iterations
    )
    print(f"Сохранено: {operator.username} (роль {operator.role}) -> {path}")
    print(f"Итераций PBKDF2: {config.pbkdf2_iterations}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
