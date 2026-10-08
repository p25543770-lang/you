"""Настройки приложения: переменные окружения + файл .env."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _load_dotenv(path: Path) -> None:
    """Мини-загрузчик .env: KEY=VALUE построчно, без сторонних зависимостей.

    Уже заданные переменные окружения имеют приоритет над файлом.
    """
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class Config:
    """Все параметры, которые нужны приложению."""

    secret_key: str = ""
    operators_file: Path = PROJECT_ROOT / "data" / "operators.json"
    bind_host: str = "0.0.0.0"
    port: int = 8080
    cookie_secure: bool = False
    cookie_samesite: str = "Lax"
    # Пароль на вход: по умолчанию выключен — пульт открывается сразу, как
    # на дисплее робота, так и по сети. Включается RC_REQUIRE_LOGIN=1.
    require_login: bool = False
    max_failed_attempts: int = 5
    lockout_seconds: int = 300
    session_lifetime_minutes: int = 60
    pbkdf2_iterations: int = 200_000
    testing: bool = False
    # --- интерфейс RUS SLAM ---
    slam_source: str = "auto"          # auto | sim | serial | ros
    slam_ports: tuple[str, ...] = ()   # UART-порты для serial
    slam_lock_file: Path = PROJECT_ROOT / "data" / "lock.json"
    slam_pin: str = ""                 # пусто → заводской PIN из backend.py
    slam_max_attempts: int = 5
    slam_lock_seconds: int = 30
    _loaded_from: str = field(default="", repr=False)

    @classmethod
    def from_env(cls, project_root: Path = PROJECT_ROOT) -> "Config":
        _load_dotenv(project_root / ".env")
        operators_file = os.environ.get("RC_OPERATORS_FILE") or "data/operators.json"
        path = Path(operators_file)
        if not path.is_absolute():
            path = project_root / path
        return cls(
            secret_key=os.environ.get("RC_SECRET_KEY", ""),
            operators_file=path,
            bind_host=os.environ.get("RC_BIND_HOST", "0.0.0.0"),
            port=int(os.environ.get("RC_PORT", "8080")),
            cookie_secure=_env_bool("RC_COOKIE_SECURE", False),
            cookie_samesite=os.environ.get("RC_COOKIE_SAMESITE", "Lax"),
            require_login=_env_bool("RC_REQUIRE_LOGIN", False),
            max_failed_attempts=int(os.environ.get("RC_MAX_FAILED_ATTEMPTS", "5")),
            lockout_seconds=int(os.environ.get("RC_LOCKOUT_SECONDS", "300")),
            session_lifetime_minutes=int(os.environ.get("RC_SESSION_MINUTES", "60")),
            pbkdf2_iterations=int(os.environ.get("RC_PBKDF2_ITERATIONS", "200000")),
            testing=_env_bool("RC_TESTING", False),
            slam_source=(os.environ.get("RC_SLAM_SOURCE") or "auto").strip().lower(),
            slam_ports=tuple(
                p.strip()
                for p in (os.environ.get("RC_SLAM_PORTS") or "").split(",")
                if p.strip()
            ),
            slam_lock_file=(
                Path(os.environ["RC_SLAM_LOCK_FILE"])
                if os.environ.get("RC_SLAM_LOCK_FILE")
                else project_root / "data" / "lock.json"
            ),
            slam_pin=(os.environ.get("RC_SLAM_PIN") or "").strip(),
            slam_max_attempts=int(os.environ.get("RC_SLAM_MAX_ATTEMPTS", "5")),
            slam_lock_seconds=int(os.environ.get("RC_SLAM_LOCK_SEC", "30")),
            _loaded_from=str(project_root / ".env"),
        )
