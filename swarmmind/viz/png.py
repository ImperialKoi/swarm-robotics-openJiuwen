"""Minimal PNG writer, stdlib only.

Deliberately not Pillow: this is a handful of bytes of header around `zlib`, it needs no
install, and it keeps the 8 GB budget and the dependency surface untouched. It exists so
visual debugging is possible long before the Godot dashboard is (docs/PLAN.md D5).
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np


def _chunk(tag: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + tag
        + data
        + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    )


def encode(rgb: np.ndarray) -> bytes:
    """(H, W, 3) uint8 -> PNG bytes."""
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"expected (H, W, 3), got {rgb.shape}")
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    h, w, _ = rgb.shape
    # Each scanline is prefixed with filter type 0 (None).
    raw = np.hstack([np.zeros((h, 1), np.uint8), rgb.reshape(h, w * 3)]).tobytes()
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(raw, 6))
        + _chunk(b"IEND", b"")
    )


def write(path: str | Path, rgb: np.ndarray) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(encode(rgb))
    return p


def upscale(rgb: np.ndarray, factor: int) -> np.ndarray:
    """Nearest-neighbour zoom, so a 1 m grid cell is actually visible."""
    return np.repeat(np.repeat(rgb, factor, axis=0), factor, axis=1)
