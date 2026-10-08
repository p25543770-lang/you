"""Хеширование паролей, хранилище операторов и ограничение подбора пароля.

Пароли хранятся только как PBKDF2-HMAC-SHA256 с индивидуальной солью.
В репозиторий не попадает ни один файл с учётками (см. .gitignore).
"""

from __future__ import annotations

import hashlib
import hmac
import json
import secrets
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

PBKDF2_ALGORITHM = "pbkdf2_sha256"
ROLE_OPERATOR = "operator"
ALLOWED_ROLES = (ROLE_OPERATOR,)


# --------------------------------------------------------------------------- #
# Пароли
# --------------------------------------------------------------------------- #
def hash_password(password: str, iterations: int = 200_000, salt: bytes | None = None) -> str:
    """Вернуть строку-хеш вида `pbkdf2_sha256$итерации$соль$хеш`."""
    if not password:
        raise ValueError("пароль не может быть пустым")
    salt = salt if salt is not None else secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, iterations)
    return f"{PBKDF2_ALGORITHM}${iterations}${salt.hex()}${digest.hex()}"


def verify_password(password: str, stored: str, iterations: int | None = None) -> bool:
    """Сверка в постоянном времени. Любая ошибка формата => False."""
    try:
        algorithm, raw_iterations, salt_hex, hash_hex = stored.split("$")
        if algorithm != PBKDF2_ALGORITHM:
            return False
        rounds = int(raw_iterations) if iterations is None else iterations
        salt = bytes.fromhex(salt_hex)
        expected = bytes.fromhex(hash_hex)
    except (AttributeError, ValueError):
        return False
    if not password or rounds <= 0:
        return False
    actual = hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, rounds)
    return hmac.compare_digest(actual, expected)


# --------------------------------------------------------------------------- #
# Операторы
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Operator:
    username: str
    role: str
    password_hash: str
    active: bool = True

    @property
    def is_operator(self) -> bool:
        return self.active and self.role == ROLE_OPERATOR


def _normalize(username: str) -> str:
    return username.strip().lower()


class OperatorStore:
    """JSON-хранилище операторов. Файл создаётся скриптом create_operator.py."""

    def __init__(self, path: Path):
        self.path = Path(path)

    def exists(self) -> bool:
        return self.path.is_file()

    def load(self) -> dict[str, Operator]:
        if not self.exists():
            return {}
        data = json.loads(self.path.read_text(encoding="utf-8"))
        operators = data.get("operators", [])
        return {
            _normalize(item["username"]): Operator(
                username=item["username"],
                role=item["role"],
                password_hash=item["password_hash"],
                active=bool(item.get("active", True)),
            )
            for item in operators
        }

    def get(self, username: str) -> Operator | None:
        return self.load().get(_normalize(username))

    def usernames(self) -> list[str]:
        return sorted(operator.username for operator in self.load().values())

    def save(self, operators: Iterable[Operator]) -> None:
        payload = {
            "version": 1,
            "operators": [
                {
                    "username": op.username,
                    "role": op.role,
                    "password_hash": op.password_hash,
                    "active": op.active,
                }
                for op in operators
            ],
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        self.path.chmod(0o600)

    def add_or_update(
        self, username: str, password: str, role: str = ROLE_OPERATOR, iterations: int = 200_000
    ) -> Operator:
        if role not in ALLOWED_ROLES:
            raise ValueError(f"недопустимая роль: {role}")
        name = username.strip()
        if not name:
            raise ValueError("имя оператора не может быть пустым")
        operator = Operator(
            username=name,
            role=role,
            password_hash=hash_password(password, iterations=iterations),
            active=True,
        )
        operators = self.load()
        operators[_normalize(name)] = operator
        self.save(operators.values())
        return operator


# --------------------------------------------------------------------------- #
# Ограничение подбора пароля
# --------------------------------------------------------------------------- #
class LoginThrottle:
    """Блокировка по (логин, IP) после серии неудачных попыток."""

    def __init__(self, max_attempts: int = 5, lockout_seconds: int = 300):
        self.max_attempts = max_attempts
        self.lockout_seconds = lockout_seconds
        self._failures: dict[str, list[float]] = {}
        self._lock = threading.Lock()

    @staticmethod
    def _key(username: str, ip: str) -> str:
        return f"{_normalize(username)}|{ip}"

    def locked_for(self, username: str, ip: str, now: float | None = None) -> int:
        """Сколько секунд осталось до разблокировки (0 — не заблокирован)."""
        now = time.time() if now is None else now
        with self._lock:
            attempts = self._failures.get(self._key(username, ip), [])
            recent = [t for t in attempts if now - t < self.lockout_seconds]
            self._failures[self._key(username, ip)] = recent
            if len(recent) < self.max_attempts:
                return 0
            return max(0, int(self.lockout_seconds - (now - recent[0])) + 1)

    def register_failure(self, username: str, ip: str, now: float | None = None) -> None:
        now = time.time() if now is None else now
        with self._lock:
            key = self._key(username, ip)
            attempts = [t for t in self._failures.get(key, []) if now - t < self.lockout_seconds]
            attempts.append(now)
            self._failures[key] = attempts[-self.max_attempts :]

    def reset(self, username: str, ip: str) -> None:
        with self._lock:
            self._failures.pop(self._key(username, ip), None)


def authenticate(
    store: OperatorStore,
    throttle: LoginThrottle,
    username: str,
    password: str,
    ip: str,
    timing_hash: str,
) -> tuple[Operator | None, str]:
    """Проверить вход.

    ``timing_hash`` — заранее посчитанный хеш, который подставляется вместо
    настоящего, когда логина нет в хранилище: тогда время ответа не выдаёт,
    существует ли такой оператор.

    Возвращает (оператор, код причины). Коды:
    ``ok``, ``locked``, ``no_store``, ``bad_credentials``, ``inactive``.
    """
    wait = throttle.locked_for(username, ip)
    if wait:
        return None, f"locked:{wait}"
    if not store.exists():
        return None, "no_store"
    operator = store.get(username)
    reference_hash = operator.password_hash if operator else timing_hash
    if not verify_password(password, reference_hash):
        throttle.register_failure(username, ip)
        return None, "bad_credentials"
    if operator is None:
        return None, "bad_credentials"
    if not operator.is_operator:
        return None, "inactive"
    throttle.reset(username, ip)
    return operator, "ok"
