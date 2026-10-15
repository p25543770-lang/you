"""Журнал событий бортового экрана: кольцевой буфер настоящих событий.

Панель «Логи» на экране киоска раньше показывала выдуманный поток строк.
Здесь закреплено то, чем он заменён:

* в журнал попадают настоящие записи логирования ``robot_control.*`` и
  явные события маршрутов, а не демо-строки;
* повторяющиеся события сворачиваются в счётчик, а не забивают экран;
* значения PIN/паролей в журнал не попадают, одна запись — одна строка;
* буфер ограничен: старые записи вытесняются, и это видно в счётчике.
"""

from __future__ import annotations

import logging

from robot_control import eventlog
from robot_control.eventlog import EventLog, EventLogHandler, sanitize


# ---------------------------------- санитайзер ------------------------------- #
def test_pin_digits_are_masked():
    assert sanitize("pin=7232") == "pin=***"
    assert sanitize("PIN: 7232") == "PIN: ***"
    assert sanitize("проверка PIN 7232 отклонена") == "проверка PIN *** отклонена"
    assert sanitize("password = hunter2") == "password = ***"
    assert sanitize("token:abcdef") == "token:***"


def test_ordinary_sentences_are_not_mangled():
    """Фраза «PIN принят» — не секрет: маскировать в ней нечего."""
    assert sanitize("PIN не принят (бортовой экран): неверный PIN") == \
        "PIN не принят (бортовой экран): неверный PIN"
    assert sanitize("руление FL → +12.5°") == "руление FL → +12.5°"


def test_one_record_is_one_line():
    line = sanitize("лидар: скана нет\n\tпорт занят\x07 ")
    assert line == "лидар: скана нет порт занят"
    assert "\n" not in line


def test_long_message_is_trimmed():
    line = sanitize("о" * 400)
    assert len(line) <= eventlog.MAX_MESSAGE
    assert line.endswith("…")


# ------------------------------- кольцевой буфер ----------------------------- #
def test_empty_journal_is_honest():
    journal = EventLog(limit=20)
    payload = journal.payload()
    assert payload["entries"] == []
    assert payload["count"] == 0
    assert payload["demo"] is False
    assert payload["levels"]["warn"] == 0


def test_entries_are_newest_last_and_counted():
    journal = EventLog(limit=20)
    journal.record("info", "первое")
    journal.record("warn", "второе")
    journal.record("ok", "третье")
    entries = journal.entries()
    assert [e["msg"] for e in entries] == ["первое", "второе", "третье"]
    assert [e["seq"] for e in entries] == [1, 2, 3]
    assert all(e["time"].count(":") == 2 for e in entries)
    payload = journal.payload(limit=2)
    assert [e["msg"] for e in payload["entries"]] == ["второе", "третье"]
    assert payload["count"] == 3 and payload["shown"] == 2


def test_repeats_collapse_into_a_counter():
    """Лидар не отвечает, тяга удерживается — строка одна со счётчиком ×N."""
    journal = EventLog(limit=20)
    for _ in range(5):
        journal.record("warn", "лидар: скана нет — идёт поиск порта")
    journal.record("info", "между ними")
    for _ in range(3):
        journal.record("warn", "лидар: скана нет — идёт поиск порта")

    entries = journal.entries()
    assert len(entries) == 2
    # Обновлённая запись уходит в конец: её время — самое свежее.
    assert entries[0]["msg"] == "между ними"
    assert entries[1]["repeat"] == 8
    assert journal.count == 2


def test_different_messages_do_not_collapse():
    journal = EventLog(limit=20)
    journal.record("info", "тяга +8% (вперёд)")
    journal.record("info", "тяга -8% (назад)")
    journal.record("warn", "тяга +8% (вперёд)")
    assert len(journal.entries()) == 3


def test_old_entries_are_dropped_and_counted():
    journal = EventLog(limit=20)          # MIN_LIMIT — буфер всегда не меньше
    for index in range(30):
        journal.record("info", "событие %d" % index)
    payload = journal.payload()
    assert payload["count"] == 20
    assert payload["entries"][0]["msg"] == "событие 10"
    assert payload["dropped"] == 10


def test_every_level_is_kept_as_is():
    journal = EventLog(limit=20)
    journal.record("info", "а")
    journal.record("ok", "б")
    journal.record("warning", "в")
    journal.record("error", "г")
    levels = [e["level"] for e in journal.entries()]
    assert levels == ["info", "ok", "warn", "err"]
    assert journal.totals()["err"] == 1


def test_empty_message_is_not_recorded():
    journal = EventLog(limit=20)
    assert journal.record("info", "") is None
    assert journal.record("info", "   ") is None
    assert journal.count == 0


def test_clear_empties_the_buffer():
    journal = EventLog(limit=20)
    journal.record("info", "событие")
    assert journal.clear() == 1
    assert journal.count == 0


# --------------------------------- обработчик -------------------------------- #
def test_handler_feeds_journal_from_robot_logging():
    journal = EventLog(limit=20)
    logger = logging.getLogger("robot_control.test_journal")
    logger.setLevel(logging.INFO)
    handler = EventLogHandler(journal)
    logger.addHandler(handler)
    logger.propagate = False
    try:
        logger.info("Лидар: найден на /dev/ttyUSB1 (%s %s бод, мотор: %s)",
                    "lds01", 230400, "none")
        logger.warning("Камера 2: нет устройства")
    finally:
        logger.removeHandler(handler)

    entries = journal.entries()
    assert [e["level"] for e in entries] == ["info", "warn"]
    assert entries[0]["msg"] == "Лидар: найден на /dev/ttyUSB1 (lds01 230400 бод, мотор: none)"
    assert entries[0]["source"] == "test_journal"      # короткое имя журнала
    assert entries[1]["source"] == "test_journal"


def test_handler_ignores_its_own_module():
    """Запись о записи не должна порождать саму себя."""
    journal = EventLog(limit=20)
    handler = EventLogHandler(journal)
    record = logging.LogRecord("robot_control.eventlog", logging.INFO, __file__, 1,
                               "внутренняя запись", None, None)
    handler.emit(record)
    assert journal.count == 0


def test_install_is_idempotent():
    eventlog.reset()
    try:
        first = eventlog.install(target=EventLog(limit=20))
        second = eventlog.install(target=first.target)
        assert first is second
        assert logging.getLogger(eventlog.ROOT_LOGGER).handlers.count(first) == 1
    finally:
        eventlog.reset()
