"""Frame synchronization and payload interpretation, with no hardware access.

The vendor log shows frames that are expected to be 2057 bytes but frequently
arrive misaligned. This module therefore never assumes a single fixed length:
it reports every observed length and only accepts a frame whose tail byte
matches at a plausible distance from the header.

No assumption is made about the array size. Both candidate layouts are
computed for each frame so the observed data can decide which one is real.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Iterable, Sequence

FRAME_HEAD = 0x98
FRAME_TAIL = 0x81
EXPECTED_TOTAL = 2057
EXPECTED_PAYLOAD = 2048
EXPECTED_HEADER = 7

#: Candidate frame lengths to accept, widest first. A frame is accepted when the
#: byte at that distance from the header is the tail byte.
LENGTH_CANDIDATES = (2057, 2056, 2058)

#: Do not accept absurdly short "frames"; the payload is known to be large.
MIN_FRAME = 64
#: Buffered bytes that cannot yield another frame are dropped.
MAX_BUFFER = 1 << 20


@dataclass
class Frame:
    length: int
    header_size: int
    payload: bytes
    checksum: bytes
    raw: bytes


@dataclass
class ScanStats:
    bytes_in: int = 0
    frames: int = 0
    discarded_bytes: int = 0
    length_histogram: Counter = field(default_factory=Counter)
    tail_mismatches: int = 0

    def as_dict(self) -> dict:
        total = self.frames + (1 if self.discarded_bytes else 0)
        return {
            "bytes_in": self.bytes_in,
            "frames": self.frames,
            "discarded_bytes": self.discarded_bytes,
            "length_histogram": {str(k): v for k, v in sorted(self.length_histogram.items())},
            "tail_mismatches": self.tail_mismatches,
        }


class FrameSynchronizer:
    """Incremental byte-stream to frame converter.

    Feed arbitrary chunks; complete frames come out. Bytes that cannot start a
    valid frame are discarded and counted, never silently included in a frame.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.stats = ScanStats()

    def feed(self, data: bytes) -> list[Frame]:
        if not data:
            return []
        self._buf += data
        self.stats.bytes_in += len(data)
        frames: list[Frame] = []

        while True:
            start = self._buf.find(bytes([FRAME_HEAD]))
            if start < 0:
                self._drop(len(self._buf))
                break
            if start > 0:
                self._drop(start)

            frame = self._try_extract()
            if frame is None:
                if len(self._buf) < MIN_FRAME:
                    break
                # Header present but no tail at a candidate distance yet: either
                # the frame is still arriving, or this 0x98 was payload noise.
                if len(self._buf) < max(LENGTH_CANDIDATES):
                    break
                self._drop(1)
                continue

            frames.append(frame)
            del self._buf[: frame.length]

        if len(self._buf) > MAX_BUFFER:
            self._drop(len(self._buf) - MAX_BUFFER)
        return frames

    def _try_extract(self) -> Frame | None:
        for length in LENGTH_CANDIDATES:
            if len(self._buf) < length:
                continue
            if self._buf[length - 1] != FRAME_TAIL:
                if len(self._buf) >= max(LENGTH_CANDIDATES):
                    self.stats.tail_mismatches += 1
                continue
            raw = bytes(self._buf[:length])
            header = EXPECTED_HEADER
            payload_end = min(header + EXPECTED_PAYLOAD, length - 1)
            payload = raw[header:payload_end]
            checksum = raw[payload_end : length - 1]
            self.stats.frames += 1
            self.stats.length_histogram[length] += 1
            return Frame(
                length=length,
                header_size=header,
                payload=payload,
                checksum=checksum,
                raw=raw,
            )
        return None

    def _drop(self, count: int) -> None:
        if count <= 0:
            return
        del self._buf[:count]
        self.stats.discarded_bytes += count


def nibble_cells(payload: bytes) -> list[int]:
    """REJECTED hypothesis: 64x64, one byte holds two 4-bit cells (0..15).

    Kept only so the layout comparison stays reproducible. The evidence that
    rules this out is in ``calibration-findings.md`` section 1.1: the even-index
    byte high nibble is non-zero 0 times out of 204800, while the odd-index byte
    high nibble is non-zero 955 times. Packed nibbles cannot produce that parity
    split.
    """
    cells: list[int] = []
    for byte in payload:
        cells.append(byte >> 4)
        cells.append(byte & 0x0F)
    return cells


def word_cells(payload: bytes) -> list[int]:
    """Confirmed layout: 32x32, one cell per **big-endian** 16-bit word.

    The device sends the high byte first. An earlier little-endian version of
    this helper cross-wired the pairs and produced meaningless values like 6144
    from a single byte of 24.
    """
    usable = len(payload) - (len(payload) % 2)
    return [
        (payload[i] << 8) | payload[i + 1]
        for i in range(0, usable, 2)
    ]



def grid_shape(cells: Sequence[int]) -> tuple[int, int] | None:
    side = int(round(len(cells) ** 0.5))
    if side * side != len(cells):
        return None
    return side, side


@dataclass
class LayoutStats:
    name: str
    cells: int
    shape: tuple[int, int] | None
    minimum: int
    maximum: int
    mean: float
    active_cells: int
    centroid: tuple[float, float] | None

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "cells": self.cells,
            "shape": list(self.shape) if self.shape else None,
            "min": self.minimum,
            "max": self.maximum,
            "mean": round(self.mean, 3),
            "active_cells": self.active_cells,
            "centroid": list(self.centroid) if self.centroid else None,
        }


def analyze_layout(cells: Sequence[int], name: str, threshold: int) -> LayoutStats:
    shape = grid_shape(cells)
    if not cells:
        return LayoutStats(name, 0, None, 0, 0, 0.0, 0, None)
    active = [(i, v) for i, v in enumerate(cells) if v >= threshold]
    centroid = None
    if active and shape:
        cols = shape[1]
        total = sum(v for _, v in active)
        if total > 0:
            cx = sum((i % cols) * v for i, v in active) / total
            cy = sum((i // cols) * v for i, v in active) / total
            centroid = (round(cx, 2), round(cy, 2))
    return LayoutStats(
        name=name,
        cells=len(cells),
        shape=shape,
        minimum=min(cells),
        maximum=max(cells),
        mean=sum(cells) / len(cells),
        active_cells=len(active),
        centroid=centroid,
    )


def analyze_payload(payload: bytes, threshold: int) -> list[LayoutStats]:
    """Both candidate interpretations of one payload, side by side.

    The second entry is the confirmed layout; the first is a rejected
    hypothesis kept for regression evidence (see ``calibration-findings.md``).
    """
    return [
        analyze_layout(word_cells(payload), "32x32_16bit_be", threshold),
        analyze_layout(nibble_cells(payload), "64x64_4bit_rejected", threshold),
    ]


def summarize(frames: Iterable[Frame], threshold: int) -> dict:
    frames = list(frames)
    if not frames:
        return {"frames": 0}
    word_max = 0
    nibble_max = 0
    word = []
    nick = []
    for frame in frames:
        a, b = analyze_payload(frame.payload, threshold)
        word.append(a)
        nick.append(b)
        word_max = max(word_max, a.maximum)
        nibble_max = max(nibble_max, b.maximum)

    def avg(values: list[float]) -> float:
        return round(sum(values) / len(values), 3) if values else 0.0

    return {
        "frames": len(frames),
        "layouts": {
            "32x32_16bit_be": {
                "observed_max": word_max,
                "mean_active_cells": avg([s.active_cells for s in word]),
                "mean_centroid": _avg_centroid([s.centroid for s in word]),
            },
            "64x64_4bit_rejected": {
                "observed_max": nibble_max,
                "mean_active_cells": avg([s.active_cells for s in nick]),
                "mean_centroid": _avg_centroid([s.centroid for s in nick]),
            },
        },
    }



def _avg_centroid(points: list[tuple[float, float] | None]) -> list[float] | None:
    valid = [p for p in points if p is not None]
    if not valid:
        return None
    return [
        round(sum(p[0] for p in valid) / len(valid), 2),
        round(sum(p[1] for p in valid) / len(valid), 2),
    ]
