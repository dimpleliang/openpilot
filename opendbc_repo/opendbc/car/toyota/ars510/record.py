"""0x80 object record: CRC check and slot extraction."""
from __future__ import annotations

import zlib
from collections.abc import Iterator

from .constants import (
    ID80_CRC_END,
    ID80_CRC_FIELD,
    ID80_CRC_START,
    ID80_IDLE_SLOT,
    ID80_OBJECT_COUNT,
    ID80_OBJECT_LEN,
    ID80_OBJECT_START,
    ID80_RECORD_LEN,
)


def id80_crc_ok(record: bytes) -> bool:
    """zlib CRC32 over record[1:737], stored little-endian at record[737:741]."""
    if len(record) != ID80_RECORD_LEN:
        return False
    observed = int.from_bytes(record[ID80_CRC_FIELD[0]:ID80_CRC_FIELD[1]], "little")
    return observed == (zlib.crc32(record[ID80_CRC_START:ID80_CRC_END]) & 0xFFFFFFFF)


def id80_crc(record_body: bytes) -> int:
    """CRC value for record[1:737] (useful for building synthetic records)."""
    return zlib.crc32(record_body) & 0xFFFFFFFF


def slot_bytes(record: bytes, slot: int) -> bytes:
    start = ID80_OBJECT_START + slot * ID80_OBJECT_LEN
    return record[start:start + ID80_OBJECT_LEN]


def occupied_slots(record: bytes) -> Iterator[tuple[int, bytes]]:
    """(slot index, 36 slot bytes) for every slot that differs from the idle template."""
    for slot in range(ID80_OBJECT_COUNT):
        b = slot_bytes(record, slot)
        if b != ID80_IDLE_SLOT:
            yield slot, b


def live_object_count(record: bytes) -> int:
    """Header field (record bits 115..119, little-endian): number of allocated slots: slots whose index field equals
    their position, which are the slots with age >= 1.

    Exact on every record of the bundled samples (header_allocation_count.json); an integrity cross-check (docs/02).
    """
    return (int.from_bytes(record[14:16], "little") >> 3) & 0x1F


# dimpleliang/ars510, 11/02 offline analysis of 420k records (tools/ars510/mac, analysis/bits2):
def header_timestamp_s(record: bytes) -> float:
    """Radar measurement timestamp of this object list: record bytes 1..4 little-endian in 100 us (wraps every ~5 days).
    Advances 60.28 ms per record on this unit, independent of the host's USB batching jitter."""
    return int.from_bytes(record[1:5], "little") * 1e-4


def header_latency_ms(record: bytes) -> int:
    """Record byte 13: age of the object list when the radar started sending it, in ms (146-241 on this unit).
    = time the first 0x80 frame left the radar minus header_timestamp_s, quantised to the radar's 10 ms tick: within a
    segment corr 0.93 and slope 0.96 ms/count against the host-side delay, which drops from 6.3 to 1.9 ms std once this is
    subtracted. More objects -> one tick later (the 0.60 correlation with the object count). Not temperature, not a detection
    count, not part of a wider field (byte 12 and the low bits of byte 14 are constant). So the objects in a record describe
    the scene at about (host time of the first frame) - header_latency_ms(record) / 1000."""
    return record[13]
