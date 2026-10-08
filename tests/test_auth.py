"""Проверки самого модуля авторизации — без HTTP."""

from __future__ import annotations

import pytest

from robot_control.auth import (
    LoginThrottle,
    OperatorStore,
    authenticate,
    hash_password,
    verify_password,
)

TIMING_HASH = hash_password("timing-equalizer", iterations=1_000)


def test_hash_password_roundtrip():
    stored = hash_password("s3cret-pass", iterations=1_000)
    assert stored.startswith("pbkdf2_sha256$1000$")
    assert verify_password("s3cret-pass", stored)
    assert not verify_password("S3CRET-PASS", stored)


def test_hash_password_rejects_empty():
    with pytest.raises(ValueError):
        hash_password("")


def test_hash_is_salted():
    assert hash_password("same", iterations=1_000) != hash_password("same", iterations=1_000)


@pytest.mark.parametrize(
    "broken",
    ["", "не-хеш", "md5$1$aa$bb", "pbkdf2_sha256$не-число$aa$bb", "pbkdf2_sha256$1$zz$bb"],
)
def test_verify_password_survives_garbage(broken):
    assert verify_password("whatever", broken) is False


def test_store_add_and_get(tmp_path):
    store = OperatorStore(tmp_path / "operators.json")
    assert store.exists() is False
    store.add_or_update("Operator", "пароль-123", iterations=1_000)
    found = store.get("operator")
    assert found is not None
    assert found.username == "Operator"
    assert found.role == "operator"
    assert store.usernames() == ["Operator"]
    assert (tmp_path / "operators.json").stat().st_mode & 0o077 == 0


def test_store_rejects_unknown_role(tmp_path):
    with pytest.raises(ValueError):
        OperatorStore(tmp_path / "o.json").add_or_update("x", "пароль-123", role="root")


def test_authenticate_ok(tmp_path):
    store = OperatorStore(tmp_path / "o.json")
    store.add_or_update("operator", "пароль-123", iterations=1_000)
    throttle = LoginThrottle(max_attempts=3, lockout_seconds=60)
    operator, reason = authenticate(
        store, throttle, "OPERATOR", "пароль-123", "10.0.0.5", timing_hash=TIMING_HASH
    )
    assert reason == "ok"
    assert operator.username == "operator"


def test_authenticate_wrong_password(tmp_path):
    store = OperatorStore(tmp_path / "o.json")
    store.add_or_update("operator", "пароль-123", iterations=1_000)
    throttle = LoginThrottle(max_attempts=3, lockout_seconds=60)
    operator, reason = authenticate(
        store, throttle, "operator", "не-тот", "10.0.0.5", timing_hash=TIMING_HASH
    )
    assert operator is None
    assert reason == "bad_credentials"


def test_authenticate_unknown_login(tmp_path):
    store = OperatorStore(tmp_path / "o.json")
    store.add_or_update("operator", "пароль-123", iterations=1_000)
    throttle = LoginThrottle(max_attempts=3, lockout_seconds=60)
    operator, reason = authenticate(
        store, throttle, "ghost", "пароль-123", "10.0.0.5", timing_hash=TIMING_HASH
    )
    assert operator is None
    assert reason == "bad_credentials"


def test_authenticate_without_store(tmp_path):
    store = OperatorStore(tmp_path / "missing.json")
    throttle = LoginThrottle()
    _, reason = authenticate(
        store, throttle, "operator", "x", "10.0.0.5", timing_hash=TIMING_HASH
    )
    assert reason == "no_store"


def test_authenticate_inactive_operator(tmp_path):
    store = OperatorStore(tmp_path / "o.json")
    store.add_or_update("operator", "пароль-123", iterations=1_000)
    operator = store.get("operator")
    store.save([type(operator)(operator.username, operator.role, operator.password_hash, False)])
    throttle = LoginThrottle()
    _, reason = authenticate(
        store, throttle, "operator", "пароль-123", "10.0.0.5", timing_hash=TIMING_HASH
    )
    assert reason == "inactive"


def test_throttle_locks_and_releases():
    throttle = LoginThrottle(max_attempts=3, lockout_seconds=60)
    now = 1_000.0
    assert throttle.locked_for("op", "1.1.1.1", now=now) == 0
    for _ in range(3):
        throttle.register_failure("op", "1.1.1.1", now=now)
    assert throttle.locked_for("op", "1.1.1.1", now=now) > 0
    # другое сочетание логин+IP блокировкой не задето
    assert throttle.locked_for("op", "2.2.2.2", now=now) == 0
    # через минуту счётчик обнуляется
    assert throttle.locked_for("op", "1.1.1.1", now=now + 61) == 0


def test_throttle_reset_after_success():
    throttle = LoginThrottle(max_attempts=2, lockout_seconds=60)
    throttle.register_failure("op", "1.1.1.1", now=5.0)
    throttle.reset("op", "1.1.1.1")
    assert throttle.locked_for("op", "1.1.1.1", now=5.0) == 0
