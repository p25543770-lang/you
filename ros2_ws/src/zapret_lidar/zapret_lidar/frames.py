# Copyright 2026 Zapret robot control
#
# Опознание лидара по байтам порта: нужно, чтобы ``model:=auto`` запускал
# правильный ROS 2-драйвер сразу, а не перебирал пакеты по таймауту.
#
# Модуль без зависимостей (кроме stdlib): тот же код работает и в супервизоре на
# роботе, и в тестах — на байтах, которые в тестах же и собраны.
"""Опознание протокола лидара по байтам: rplidar, LDS-01, LD06/LD19."""

from __future__ import annotations

#: Синхрослово RPLIDAR: драйвер посылает SCAN и получает дескриптор A5 5A.
RPLIDAR_CMD_SCAN = b"\xa5\x20"
RPLIDAR_ANSWER_SYNC = b"\xa5\x5a"
RPLIDAR_BAUDS = (115200, 256000, 460800)

#: Заголовки пакетов, которые лидары сыплют сами (без команд).
LD06_SYNC = b"\x54\x2c"
LD06_PACKET_LEN = 47
LDS01_HEADER = 0xFA
LDS01_PACKET_LEN = 42
LDS01_BAUD = 230400
LD06_BAUD = 230400

#: Сколько очков нужно, чтобы считать протокол опознанным (витки не обязательны:
#: проверка идёт доли секунды, до полного оборота).
SCORE_STRONG = 2.0
SCORE_WEAK = 1.0

#: Протокол → скорость, на которой его проверяют при ``model:=auto``.
PROTOCOL_BAUDS: dict[str, tuple[int, ...]] = {
    "lds01": (LDS01_BAUD,),
    "ld06": (LD06_BAUD,),
    "rplidar": RPLIDAR_BAUDS,
}


def lds01_checksum(data: bytes) -> int:
    """Контрольная сумма пакета LDS-01/LDS-02 (сумма двух чередующихся полусумм)."""
    low = 0
    high = 0
    for index, byte in enumerate(data):
        if index % 2 == 0:
            low = (low + byte) & 0xFFFF
        else:
            high = (high + byte) & 0xFFFF
    return ((high << 8) | (low & 0xFF)) & 0xFFFF


def score_lds01(data: bytes) -> float:
    """Очки за кадры LDS-01: заголовок 0xFA, длина 42, номера и КС сходятся."""
    score = 0.0
    limit = len(data) - LDS01_PACKET_LEN
    index = 0
    while index <= limit:
        if data[index] != LDS01_HEADER:
            index += 1
            continue
        packet = data[index:index + LDS01_PACKET_LEN]
        number = packet[1]
        index_ok = 0xA0 <= number < 0xA0 + 60
        checksum = packet[40] | (packet[41] << 8)
        crc_ok = checksum == lds01_checksum(packet[:40])
        if index_ok and crc_ok:
            score += 1.0
            index += LDS01_PACKET_LEN
            continue
        if index_ok:
            # Клоны с другой формулой контрольной суммы: считаем слабым признаком.
            score += 0.25
        index += 1
    return score


def ld06_crc_ok(packet: bytes) -> bool:
    """Контрольная сумма пакета LD06/LD19: XOR всех байт, кроме последнего."""
    crc = 0
    for byte in packet[:LD06_PACKET_LEN - 1]:
        crc ^= byte
    return crc == packet[LD06_PACKET_LEN - 1]


def score_ld06(data: bytes) -> float:
    """Очки за кадры LD06/LD19: синхрослово 0x54 0x2C и верная контрольная сумма."""
    score = 0.0
    index = 0
    limit = len(data) - LD06_PACKET_LEN
    while index <= limit:
        if data[index:index + 2] != LD06_SYNC:
            index += 1
            continue
        packet = data[index:index + LD06_PACKET_LEN]
        if ld06_crc_ok(packet):
            score += 1.0
            index += LD06_PACKET_LEN
            continue
        score += 0.25
        index += 1
    return score


def score_rplidar(data: bytes) -> float:
    """Очки за RPLIDAR: ответ на команду SCAN и ровный виток точек.

    Точки RPLIDAR — 5 байт: качество/флаг, угол (в 1/64°), дистанция (в 1/4 мм).
    Проверяем не одну точку, а виток: у настоящего лидара угол растёт шагами по
    кругу, а у случайных байт — скачет. Это тот же признак, что и в приложении
    (``robot_control/lidar.py``), только здесь он нужен один раз, при запуске.

    Без синхрослова ответа (``A5 5A`` на нашу команду SCAN) очки не начисляются
    вовсе: байтовый «пандус» с монотонно растущим содержимым иначе выглядел бы
    как вращение — на этом и строится проверка мусора в тестах.
    """
    if RPLIDAR_ANSWER_SYNC not in data[:32]:
        return 0.0
    angles: list[float] = []
    index = 0
    while index + 5 <= len(data):
        packet = data[index:index + 5]
        start_bit = packet[0] & 0x01
        quality = packet[0] >> 2
        angle = ((packet[1] >> 1) | (packet[2] << 7)) / 64.0
        if start_bit == 0 and quality > 0:
            angles.append(angle % 360.0)
            index += 5
            continue
        index += 1
    if len(angles) < 8:
        # Дескриптор ответа уже доказательство, что это RPLIDAR на связи.
        return SCORE_WEAK
    steps = [(current - previous) % 360.0 for previous, current in zip(angles, angles[1:])]
    median = sorted(steps)[len(steps) // 2]
    spread = max(angles) - min(angles)
    if median > 15.0 or spread <= 30.0:
        return SCORE_WEAK
    score = 1.5
    if sum(1 for step in steps if step > 180.0) == 1:
        # Ровно один оборот за окно: так и идёт виток настоящего лидара.
        score += 0.5
    return score


def score_bytes(data: bytes, protocol: str) -> float:
    """Очки за конкретный протокол по уже прочитанным байтам."""
    if not data:
        return 0.0
    if protocol == "lds01":
        return score_lds01(data)
    if protocol in ("ld06", "ld19", "stl27l"):
        return score_ld06(data)
    if protocol == "rplidar":
        return score_rplidar(data)
    return 0.0


def best_protocol(data: bytes) -> tuple[str, float]:
    """Лучший протокол по байтам порта и его оценка (``("", 0.0)`` — не опознан)."""
    results = [(name, score_bytes(data, name)) for name in ("lds01", "ld06", "rplidar")]
    results.sort(key=lambda item: -item[1])
    name, score = results[0]
    return (name, score) if score >= SCORE_WEAK else ("", 0.0)
