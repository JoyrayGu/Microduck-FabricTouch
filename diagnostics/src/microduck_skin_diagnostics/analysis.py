"""Offline analysis of raw captured frames.

Reads a .bin of back-to-back frames and reports the value histogram, per-frame
activity, a spatial map, and centroid drift. This distinguishes a persistent
local bias (some region always high) from transient contact (only some frames).
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from . import frame_parser as fp


def load_frames(path: Path) -> list[bytes]:
    """Split a .bin of concatenated frames using the same sync rules."""
    data = Path(path).read_bytes()
    sync = fp.FrameSynchronizer()
    frames = sync.feed(data)
    return [frame.payload for frame in frames]


@dataclass
class FrameReport:
    index: int
    active: int
    maximum: int
    centroid: tuple[float, float] | None


def value_histogram(payloads: Sequence[bytes]) -> Counter:
    hist: Counter = Counter()
    for payload in payloads:
        hist.update(fp.nibble_cells(payload))
    return hist


def per_frame(payloads: Sequence[bytes], threshold: int) -> list[FrameReport]:
    out: list[FrameReport] = []
    for i, payload in enumerate(payloads):
        stats = fp.analyze_layout(fp.nibble_cells(payload), "64x64", threshold)
        out.append(FrameReport(i, stats.active_cells, stats.maximum, stats.centroid))
    return out


def hotspot_map(payloads: Sequence[bytes], rows: int = 8, cols: int = 8, side: int = 64) -> list[list[int]]:
    """Coarse map: max cell value per block, to localise a persistent hotspot."""
    grid = [[0] * cols for _ in range(rows)]
    block_r = side // rows
    block_c = side // cols
    for payload in payloads:
        cells = fp.nibble_cells(payload)
        for r in range(rows):
            for c in range(cols):
                base_r, base_c = r * block_r, c * block_c
                peak = 0
                for y in range(base_r, base_r + block_r):
                    row_off = y * side
                    for x in range(base_c, base_c + block_c):
                        v = cells[row_off + x]
                        if v > peak:
                            peak = v
                if peak > grid[r][c]:
                    grid[r][c] = peak
    return grid


def render_map(grid: list[list[int]]) -> str:
    lines = []
    for row in grid:
        lines.append(" ".join(f"{v:2d}" if v else " ." for v in row))
    return "\n".join(lines)


def report(path: Path, threshold: int) -> dict:
    payloads = load_frames(path)
    if not payloads:
        return {"path": str(path), "frames": 0}
    hist = value_histogram(payloads)
    total_cells = sum(hist.values())
    reports = per_frame(payloads, threshold)
    actives = [r.active for r in reports]
    maxes = [r.maximum for r in reports]
    centroids = [r.centroid for r in reports if r.centroid]
    moving = 0
    if len(centroids) > 1:
        for a, b in zip(centroids, centroids[1:]):
            if abs(a[0] - b[0]) > 0.5 or abs(a[1] - b[1]) > 0.5:
                moving += 1
    return {
        "path": str(path),
        "frames": len(payloads),
        "threshold": threshold,
        "value_histogram": {str(k): v for k, v in sorted(hist.items())},
        "nonzero_cell_fraction": round(
            (total_cells - hist.get(0, 0)) / total_cells, 5
        ) if total_cells else 0.0,
        "active_cells": {
            "min": min(actives),
            "max": max(actives),
            "mean": round(sum(actives) / len(actives), 2),
            "frames_above_zero": sum(1 for a in actives if a > 0),
        },
        "frame_max": {
            "min": min(maxes),
            "max": max(maxes),
            "mean": round(sum(maxes) / len(maxes), 2),
        },
        "centroid": {
            "first": list(centroids[0]) if centroids else None,
            "last": list(centroids[-1]) if centroids else None,
            "frames_with_centroid": len(centroids),
            "frames_with_motion": moving,
        },
        "hotspot_max_map": hotspot_map(payloads),
    }
