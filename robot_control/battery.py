"""Read-only JBD UART telemetry, following danilka-revin/ploter_a1.

See docs/battery.md for protocol provenance and wiring. Only read requests
(``DD A5 <reg> 00 …``) are ever sent: basic info 0x03 and cells 0x04 every
second, plus — as optional extras that never break telemetry — the device name
0x05, protection trigger counters 0xAA and the protection thresholds
0x10…0x2F (plain reads, exactly like ploter_a1; boards that only answer them
in factory mode report them as unavailable). No factory-mode, settings, FET or
capacity-write commands exist in this module.
"""
from __future__ import annotations

import atexit
import copy
import csv
import io
import logging
import os
import threading
import time

log = logging.getLogger(__name__)
MAX_AGE = 5.0
FAULTS = (
    "Перенапряжение банки", "Пониженное напряжение банки",
    "Перенапряжение АКБ", "Пониженное напряжение АКБ",
    "Перегрев при заряде", "Переохлаждение при заряде",
    "Перегрев при разряде", "Переохлаждение при разряде",
    "Перегрузка заряда", "Перегрузка разряда", "Короткое замыкание",
    "Ошибка AFE", "Программная защита",
)


class ProtocolError(ValueError):
    pass


class NoAnswer(ProtocolError):
    """Плата промолчала (а не отказала статусом 0x80)."""


def reserved_ports():
    port = os.environ.get("RC_BMS_PORT", "").strip()
    return {port, os.path.realpath(port)} if port else set()


#: Счётчики срабатываний защит (регистр 0xAA, по u16 на счётчик), порядок bms-tools.
ERROR_COUNTERS = (
    ("sc", "Короткое замыкание"), ("chgoc", "Перегрузка заряда"), ("dsgoc", "Перегрузка разряда"),
    ("covp", "Перенапряжение банки"), ("cuvp", "Пониженное напряжение банки"),
    ("chgot", "Перегрев при заряде"), ("chgut", "Переохлаждение при заряде"),
    ("dsgot", "Перегрев при разряде"), ("dsgut", "Переохлаждение при разряде"),
    ("povp", "Перенапряжение АКБ"), ("puvp", "Пониженное напряжение АКБ"),
)

#: Пороги защит из EEPROM (только чтение). Масштабы — как в ploter_a1 / bms-tools:
#: mv — мВ (показ в В), mvraw — мВ как есть, mv10 — ×10 мВ, ma10 — ×10 мА,
#: temp — 0,1 К, mah10 — ×10 мАч.
THRESHOLDS = (
    (0x24, "covp", "Перенапряжение банки", "mv"),
    (0x25, "covp_rel", "Возврат после перенапряжения банки", "mv"),
    (0x26, "cuvp", "Пониженное напряжение банки", "mv"),
    (0x27, "cuvp_rel", "Возврат после понижения банки", "mv"),
    (0x20, "povp", "Перенапряжение АКБ", "mv10"),
    (0x21, "povp_rel", "Возврат после перенапряжения АКБ", "mv10"),
    (0x22, "puvp", "Пониженное напряжение АКБ", "mv10"),
    (0x23, "puvp_rel", "Возврат после понижения АКБ", "mv10"),
    (0x28, "chgoc", "Перегрузка по заряду", "ma10"),
    (0x29, "dsgoc", "Перегрузка по разряду", "ma10"),
    (0x18, "chgot", "Перегрев при заряде", "temp"),
    (0x19, "chgot_rel", "Возврат после перегрева при заряде", "temp"),
    (0x1A, "chgut", "Переохлаждение при заряде", "temp"),
    (0x1B, "chgut_rel", "Возврат после переохлаждения при заряде", "temp"),
    (0x1C, "dsgot", "Перегрев при разряде", "temp"),
    (0x1D, "dsgot_rel", "Возврат после перегрева при разряде", "temp"),
    (0x1E, "dsgut", "Переохлаждение при разряде", "temp"),
    (0x1F, "dsgut_rel", "Возврат после переохлаждения при разряде", "temp"),
    (0x2A, "bal_start", "Старт балансировки", "mv"),
    (0x2B, "bal_window", "Окно балансировки", "mvraw"),
    (0x10, "design_cap", "Номинальная ёмкость", "mah10"),
    (0x2F, "cell_cnt", "Число банок в настройках", "count"),
)
DEVICE_NAME_REGISTER = 0x05
ERROR_COUNTERS_REGISTER = 0xAA
#: Только чтение: телеметрия, имя, счётчики и пороги. Ни одного регистра записи.
READ_REGISTERS = frozenset({0x03, 0x04, DEVICE_NAME_REGISTER, ERROR_COUNTERS_REGISTER}
                           | {reg for reg, *_ in THRESHOLDS})


def read_request(register):
    if register not in READ_REGISTERS:
        raise ProtocolError("Разрешено только чтение телеметрии, имени, счётчиков и порогов JBD")
    checksum = (-register) & 0xFFFF
    return bytes([0xDD, 0xA5, register, 0, checksum >> 8, checksum & 255, 0x77])


def decode_date(raw):
    """Дата выпуска JBD: биты 0–4 день, 5–8 месяц, 9–15 год от 2000. Нули → None."""
    day, month, year = raw & 0x1F, (raw >> 5) & 0x0F, ((raw >> 9) & 0x7F) + 2000
    if not raw or not 1 <= month <= 12 or not 1 <= day <= 31:
        return None
    return f"{year:04d}-{month:02d}-{day:02d}"


def parse_device_name(data):
    text = bytes(data).replace(b"\x00", b"").decode("utf-8", "replace")
    text = "".join(ch for ch in text if ch.isprintable()).strip()
    return text[:40] or None


def parse_error_counters(data):
    if len(data) < 2:
        raise ProtocolError("Пустой ответ счётчиков защит")
    counts = {}
    for index, (key, _label) in enumerate(ERROR_COUNTERS):
        offset = index * 2
        counts[key] = int.from_bytes(data[offset:offset + 2], "big") if offset + 2 <= len(data) else None
    return counts


def decode_threshold(kind, data):
    """Значение порога в единицах экрана: В, А, °C, Ач, шт."""
    if len(data) < 2:
        raise ProtocolError("Короткий ответ порога")
    raw = int.from_bytes(data[:2], "big")
    if kind == "mv":
        return round(raw / 1000, 3), "В"
    if kind == "mvraw":
        return raw, "мВ"
    if kind == "mv10":
        return round(raw / 100, 2), "В"
    if kind == "ma10":
        signed = raw - 0x10000 if raw & 0x8000 else raw   # разряд у части плат со знаком
        return round(abs(signed) / 100, 2), "А"
    if kind == "temp":
        return round((raw - 2731) / 10, 1), "°C"
    if kind == "mah10":
        return round(raw / 100, 2), "Ач"
    return raw, "шт"


def extract_frames(buffer):
    """Accept fragmented frames, noise, and both JBD status checksum variants."""
    frames = []
    while buffer:
        start = buffer.find(b"\xdd")
        if start < 0:
            return frames, b""
        buffer = buffer[start:]
        if len(buffer) < 7:
            break
        size = buffer[3] + 7
        if len(buffer) < size:
            break
        frame = buffer[:size]
        checksum = int.from_bytes(frame[-3:-1], "big")
        if frame[-1] != 0x77 or checksum not in (
            (-sum(frame[2:-3])) & 0xFFFF, (-sum(frame[3:-3])) & 0xFFFF,
        ):
            buffer = buffer[1:]
            continue
        frames.append((frame[1], frame[2], frame[4:-3]))
        buffer = buffer[size:]
    return frames, buffer


def parse_snapshot(basic, cells, now=None):
    if len(basic) < 23 or len(basic) < 23 + basic[22] * 2:
        raise ProtocolError("Неполный ответ общих параметров/температур")
    if not cells or len(cells) % 2:
        raise ProtocolError("Неполный ответ напряжений банок")
    count = basic[21]
    if not 1 <= count <= 127:
        raise ProtocolError("Некорректное количество банок BMS")
    word = lambda offset: int.from_bytes(basic[offset:offset + 2], "big")
    millivolts = [int.from_bytes(cells[i:i + 2], "big") for i in range(0, len(cells), 2)]
    balance = word(12) | (word(14) << 16)
    average = sum(millivolts) / len(millivolts)
    faults = word(16)
    current = int.from_bytes(basic[2:4], "big", signed=True) / 100
    voltage = word(0) / 100
    complete = len(millivolts) == count
    remaining_ah, full_ah = word(4) / 100, word(6) / 100
    # Направление тока — как в ploter_a1: шум ±0,05 А считается покоем.
    direction = "charge" if current > 0.05 else ("discharge" if current < -0.05 else "idle")
    hours_to_empty = round(remaining_ah / -current, 2) if current <= -0.1 else None
    hours_to_full = (round(max(0.0, full_ah - remaining_ah) / current, 2)
                     if current >= 0.1 and full_ah else None)
    manufactured = decode_date(word(10))
    lowest, highest = min(millivolts), max(millivolts)
    return {
        "updatedAt": time.time() if now is None else now,
        "volts": voltage, "amps": current, "watts": round(voltage * current, 2),
        "direction": direction,
        "directionText": {"charge": "заряд", "discharge": "разряд", "idle": "покой"}[direction],
        "soc": basic[19], "remainingAh": remaining_ah, "fullAh": full_ah,
        "remainingWh": round(remaining_ah * voltage, 1), "fullWh": round(full_ah * voltage, 1),
        "hoursToEmpty": hours_to_empty, "hoursToFull": hours_to_full,
        "cycles": word(8), "firmwareVersion": basic[18], "firmwareText": f"0x{basic[18]:02X}",
        "manufactureDate": manufactured,
        "manufactureDateText": ".".join(reversed(manufactured.split("-"))) if manufactured else None,
        "ntcCount": basic[22],
        "temperatures": [round((word(23 + i * 2) - 2731) / 10, 1) for i in range(basic[22])],
        "chargeOn": bool(basic[20] & 1), "dischargeOn": bool(basic[20] & 2),
        "faultRaw": faults,
        "faults": [label for bit, label in enumerate(FAULTS) if faults & (1 << bit)]
                  + ([f"Неизвестные защиты: 0x{faults & ~0x1FFF:04X}"] if faults & ~0x1FFF else []),
        "balanceMask": balance,
        "balancingCells": [i + 1 for i in range(max(count, len(millivolts))) if balance & (1 << i)],
        "cellCount": count, "receivedCount": len(millivolts), "complete": complete,
        "minMv": lowest, "maxMv": highest,
        "minCell": millivolts.index(lowest) + 1, "maxCell": millivolts.index(highest) + 1,
        "sumMv": sum(millivolts), "sumVolts": round(sum(millivolts) / 1000, 3),
        "deltaMv": highest - lowest, "averageMv": round(average, 1),
        # Never silently truncate extra banks or omit missing ones.
        "cells": [
            {"number": i + 1, "mv": millivolts[i] if i < len(millivolts) else None,
             "deviationMv": round(millivolts[i] - average, 1) if i < len(millivolts) else None,
             "balancing": bool(balance & (1 << i))}
            for i in range(max(count, len(millivolts)))
        ],
    }


NOT_CONFIGURED = ("BMS не настроена: выберите порт на этой странице («Подключение BMS») "
                  "или задайте RC_BMS_PORT в .env")
OPTIONAL_TIMEOUT = 0.35      # имя/счётчики/пороги: не ответила быстро — не страшно
COUNTERS_EVERY = 10          # счётчики срабатываний — раз в 10 опросов, как в ploter_a1
THRESHOLDS_PER_POLL = 2      # пороги дочитываются по 2 регистра за опрос, не тормозя телеметрию
THRESHOLD_SILENCE_LIMIT = 4  # столько порогов подряд без ответа — плата их не отдаёт, хватит


class BatteryManager:
    """One polling thread on the kiosk server; HTTP requests never touch UART."""
    def __init__(self, port="", opener=None):
        self.port = port
        self._opener = opener or self._open
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sample = None
        self._error = "Ожидание BMS" if port else NOT_CONFIGURED
        self._connected_since = None
        self._reset_extras()

    def _reset_extras(self):
        """Имя, счётчики и пороги читаются заново на каждом новом подключении."""
        self._polls = 0
        self._device_name = None
        self._name_attempts = 0
        self._counters = None
        self._counters_at = None
        self._counter_failures = 0
        self._thresholds = {}
        self._threshold_queue = [reg for reg, *_ in THRESHOLDS]
        self._threshold_silence = 0

    def _open(self):
        import serial
        return serial.Serial(
            self.port, baudrate=9600, bytesize=8, parity="N", stopbits=1,
            timeout=0.05, write_timeout=0.8, xonxoff=False, rtscts=False, dsrdtr=False,
        )

    def start(self):
        if self.port and not (self._thread and self._thread.is_alive()):
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="jbd-bms", daemon=True)
            self._thread.start()
            atexit.register(self.stop)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def reconfigure(self, port):
        """Сменить порт на ходу (страница «Подключение BMS»): поток перезапускается."""
        port = (port or "").strip()
        self.stop()
        with self._lock:
            self.port = port
            self._sample = None
            self._connected_since = None
            self._error = "Ожидание BMS" if port else NOT_CONFIGURED
            self._reset_extras()
        self._thread = None
        self.start()

    def _read(self, transport, register, timeout=0.8):
        transport.reset_input_buffer()
        transport.write(read_request(register))
        buffer = b""
        deadline = time.monotonic() + timeout
        while not self._stop.is_set() and time.monotonic() < deadline:
            buffer += transport.read(256)
            frames, buffer = extract_frames(buffer)
            for reg, status, data in frames:
                if reg == register:
                    if status:
                        raise ProtocolError(f"BMS: статус 0x{status:02X}, регистр 0x{reg:02X}")
                    return data
        raise NoAnswer(f"BMS не ответила на регистр 0x{register:02X}")

    def poll(self, transport):
        sample = parse_snapshot(self._read(transport, 3), self._read(transport, 4))
        with self._lock:
            self._sample = sample
            self._error = ""
            if self._connected_since is None:
                self._connected_since = sample["updatedAt"]

    def poll_extras(self, transport):
        """Необязательные данные после телеметрии: имя, счётчики, пороги.

        Только запросы чтения. Ошибка или молчание платы здесь не рвут связь и
        не портят телеметрию: поле просто помечается недоступным.
        """
        self._polls += 1
        if self._device_name is None and self._name_attempts < 2:
            self._name_attempts += 1
            try:
                name = parse_device_name(self._read(transport, DEVICE_NAME_REGISTER, OPTIONAL_TIMEOUT))
                with self._lock:
                    self._device_name = name or ""
            except (ProtocolError, OSError) as exc:
                log.debug("BMS: имя устройства не прочитано: %s", exc)
        if self._counter_failures < 3 and (self._polls - 1) % COUNTERS_EVERY == 0:
            try:
                counts = parse_error_counters(
                    self._read(transport, ERROR_COUNTERS_REGISTER, OPTIONAL_TIMEOUT))
                with self._lock:
                    self._counters = counts
                    self._counters_at = time.time()
                    self._counter_failures = 0
            except (ProtocolError, OSError) as exc:
                self._counter_failures += 1
                log.debug("BMS: счётчики защит не прочитаны: %s", exc)
        for _ in range(THRESHOLDS_PER_POLL):
            if not self._threshold_queue:
                break
            register = self._threshold_queue.pop(0)
            definition = next(item for item in THRESHOLDS if item[0] == register)
            try:
                value, unit = decode_threshold(
                    definition[3], self._read(transport, register, OPTIONAL_TIMEOUT))
                self._threshold_silence = 0
            except NoAnswer:
                self._threshold_silence += 1
                value, unit = None, None
            except (ProtocolError, OSError):
                self._threshold_silence = 0       # отказ статусом — плата живая, читаем дальше
                value, unit = None, None
            with self._lock:
                self._thresholds[definition[1]] = (value, unit)
                if self._threshold_silence >= THRESHOLD_SILENCE_LIMIT:
                    # Молчит на пороги — не тратим на них каждый опрос.
                    for rest in self._threshold_queue:
                        key = next(item[1] for item in THRESHOLDS if item[0] == rest)
                        self._thresholds[key] = (None, None)
                    self._threshold_queue = []

    def extras(self):
        """Имя, счётчики и пороги в виде, готовом для экрана и CSV."""
        with self._lock:
            name = self._device_name
            name_attempts = self._name_attempts
            counters = dict(self._counters) if self._counters is not None else None
            counters_at = self._counters_at
            counter_failures = self._counter_failures
            thresholds = dict(self._thresholds)
            pending = len(self._threshold_queue)
        read = [key for key, (value, _unit) in thresholds.items() if value is not None]
        if pending:
            thresholds_state = "reading"
        elif read:
            thresholds_state = "ok" if len(read) == len(THRESHOLDS) else "partial"
        else:
            thresholds_state = "unavailable"
        return {
            "deviceName": name or None,
            "deviceNameState": "ok" if name else ("unavailable" if name_attempts >= 2 else "reading"),
            "errorCounters": None if counters is None else [
                {"key": key, "label": label, "count": counters.get(key)} for key, label in ERROR_COUNTERS
            ],
            "errorCountersAt": counters_at,
            "errorCountersState": "ok" if counters is not None else (
                "unavailable" if counter_failures >= 3 else "reading"),
            "thresholds": [
                {"key": key, "label": label, "value": thresholds[key][0], "unit": thresholds[key][1]}
                for _reg, key, label, _kind in THRESHOLDS if key in thresholds
            ],
            "thresholdsState": thresholds_state,
        }

    def _loop(self):
        from .lidar import try_lock_port, unlock_port
        while not self._stop.is_set():
            transport = None
            fd = None
            try:
                fd = try_lock_port(self.port)
                if fd is None:
                    raise OSError("Порт BMS занят другим процессом")
                transport = self._opener()
                log.info("BMS: подключение к %s, JBD 9600 8N1", self.port)
                with self._lock:
                    self._reset_extras()
                    self._connected_since = None
                while not self._stop.is_set():
                    self.poll(transport)
                    self.poll_extras(transport)
                    self._stop.wait(1)
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                log.warning("BMS: %s", exc)
            finally:
                try:
                    if transport is not None:
                        transport.close()
                finally:
                    unlock_port(fd)
            self._stop.wait(3)

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        with self._lock:
            sample = copy.deepcopy(self._sample)
            error = self._error
        age = max(0, now - sample["updatedAt"]) if sample else None
        connected = sample is not None and age <= MAX_AGE and not error
        complete = bool(sample and sample["complete"])
        reason = error or ("Данные устарели" if not connected else (
            "Все банки прочитаны" if complete else
            f"Неполный отчёт: получено {sample['receivedCount']} из {sample['cellCount']} банок"
        ))
        if sample is not None:
            sample.update(self.extras())
        with self._lock:
            since = self._connected_since
        return {"configured": bool(self.port), "connected": connected,
                "complete": complete, "reason": reason, "ageS": age, "data": sample,
                "port": self.port or None, "connectedSince": since if connected else None}


def report_response(snapshot):
    """CSV contains every bank from a single snapshot, including missing banks."""
    from flask import Response, jsonify
    if not snapshot.get("connected") or not snapshot.get("data"):
        return jsonify({"ok": False, "reason": snapshot["reason"]}), 503
    data = snapshot["data"]
    out = io.StringIO(newline="")
    writer = csv.writer(out, delimiter=";")
    writer.writerow(["Отчёт JBD BMS", "Время Unix", data["updatedAt"]])
    writer.writerow(["Полнота", snapshot["reason"], "Получено", data["receivedCount"], "Ожидалось", data["cellCount"]])
    for label, key in (("Заряд, %", "soc"), ("Напряжение, В", "volts"), ("Ток, А", "amps"),
                       ("Мощность, Вт", "watts"), ("Остаток, Ач", "remainingAh"),
                       ("Полная ёмкость, Ач", "fullAh"), ("Циклы", "cycles"),
                       ("Минимум, мВ", "minMv"), ("Максимум, мВ", "maxMv"),
                       ("Разброс, мВ", "deltaMv")):
        writer.writerow([label, data[key]])
    writer.writerow(["Температуры, °C", *data["temperatures"]])
    writer.writerow(["Заряд разрешён", data["chargeOn"], "Разряд разрешён", data["dischargeOn"]])
    writer.writerow(["Защиты", *data["faults"]] if data["faults"] else ["Защиты", "Нет активных защит"])
    writer.writerow(["Устройство", data.get("deviceName") or "нет данных"])
    writer.writerow(["Дата выпуска", data.get("manufactureDateText") or "нет данных"])
    writer.writerow(["Прошивка BMS", data.get("firmwareText") or ""])
    writer.writerow(["Датчиков температуры", data.get("ntcCount")])
    writer.writerow(["Мин. банка", data.get("minCell"), "Макс. банка", data.get("maxCell")])
    writer.writerow(["Сумма банок, В", data.get("sumVolts")])
    writer.writerow(["Балансируются банки", *(data.get("balancingCells") or ["нет"])])
    counters = data.get("errorCounters")
    if counters:
        writer.writerow(["Счётчики срабатываний", *[f"{item['label']}: {item['count']}" for item in counters]])
    for item in data.get("thresholds") or []:
        writer.writerow(["Порог: " + item["label"],
                         "нет данных" if item["value"] is None else f"{item['value']} {item['unit']}"])
    writer.writerow(["Банка", "Напряжение, мВ", "Отклонение от среднего, мВ", "Балансировка", "Данные"])
    for cell in data["cells"]:
        writer.writerow([cell["number"], cell["mv"], cell["deviationMv"],
                         "Да" if cell["balancing"] else "Нет", "Нет данных" if cell["mv"] is None else "Прочитано"])
    return Response("\ufeff" + out.getvalue(), content_type="text/csv; charset=utf-8",
                    headers={"Content-Disposition": 'attachment; filename="battery-report.csv"',
                             "Cache-Control": "no-store"})


def kiosk_snapshot():
    """The operator server relays kiosk telemetry, never opens the BMS UART."""
    import json
    import urllib.request
    try:
        port = int(os.environ.get("RC_KIOSK_PORT") or "8090")
        if not 1 <= port <= 65535:
            raise ValueError("invalid kiosk port")
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/battery", timeout=1) as response:
            return json.load(response)["battery"]
    except (OSError, ValueError, KeyError) as exc:
        return {"configured": bool(reserved_ports()), "connected": False, "complete": False,
                "reason": "Нет связи с BMS: запустите бортовой сервер (8090)", "ageS": None, "data": None}
