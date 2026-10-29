"""Encodings shared with the C++ writer (see schema.sql header)."""

from __future__ import annotations

import json
from typing import Iterable, Sequence

import numpy as np

SEQ_BITS = 48
SEQ_MASK = (1 << SEQ_BITS) - 1
SYMBOL_BITS = 56
SYMBOL_INDEX_MASK = (1 << SYMBOL_BITS) - 1

#: Track ids at or above this value are not Khronos tracks. SAM3-discovered objects use
#: ``SAM3_TRACK_BASE | object_index`` and legacy imports of Khronos object folders use
#: ``LEGACY_TRACK_BASE | object_index`` (the original track id is unknown).
SAM3_TRACK_BASE = 1 << 40
LEGACY_TRACK_BASE = 1 << 41
TRACK_INDEX_MASK = (1 << 40) - 1


def make_id(robot_id: int, seq: int) -> int:
    """Frame / observation / sensor id: ``(robot_id << 48) | seq``."""
    if not 0 <= seq <= SEQ_MASK:
        raise ValueError(f"sequence {seq} out of range")
    return (int(robot_id) << SEQ_BITS) | int(seq)


def id_robot(id_: int) -> int:
    return int(id_) >> SEQ_BITS


def id_seq(id_: int) -> int:
    return int(id_) & SEQ_MASK


def node_symbol(node_id: int) -> tuple[str, int]:
    """spark_dsg NodeSymbol decomposition: (category char, index)."""
    node_id = int(node_id)
    return chr((node_id >> SYMBOL_BITS) & 0xFF), node_id & SYMBOL_INDEX_MASK


def make_node_id(category: str, index: int) -> int:
    return (ord(category) << SYMBOL_BITS) | int(index)


def node_label(node_id: int, sep: str = "_") -> str:
    """``O_12`` for NodeSymbol('O', 12); the decimal id if the category is not a letter."""
    cat, idx = node_symbol(node_id)
    return f"{cat}{sep}{idx}" if cat.isalpha() else str(int(node_id))


def sam3_track_id(node_id: int) -> int:
    """Track id used for SAM3 observations bound to ``node_id`` (one track per node)."""
    return SAM3_TRACK_BASE | (int(node_id) & TRACK_INDEX_MASK)


# --- transforms -------------------------------------------------------------------------


def transform_to_blob(T) -> bytes:
    arr = np.asarray(T, dtype="<f8")
    if arr.size != 16:
        raise ValueError(f"expected a 4x4 transform, got shape {arr.shape}")
    return arr.reshape(16).tobytes()


def blob_to_transform(blob) -> np.ndarray | None:
    if blob is None:
        return None
    if len(blob) != 128:
        raise ValueError(f"transform blob has {len(blob)} bytes, expected 128")
    return np.frombuffer(blob, dtype="<f8").reshape(4, 4).copy()


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    q = np.array([w, x, y, z], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_matrix(
    position: Sequence[float], rotation: np.ndarray | None = None
) -> np.ndarray:
    T = np.eye(4)
    if rotation is not None:
        T[:3, :3] = rotation
    T[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return T


# --- masks ------------------------------------------------------------------------------


def rle_encode(mask: np.ndarray) -> list[int]:
    """Row-major run lengths, starting with a (possibly empty) background run."""
    flat = np.asarray(mask, dtype=bool).reshape(-1)
    if flat.size == 0:
        return []
    change = np.flatnonzero(flat[1:] != flat[:-1]) + 1
    bounds = np.concatenate(([0], change, [flat.size]))
    counts = np.diff(bounds).tolist()
    if flat[0]:
        counts.insert(0, 0)
    return [int(c) for c in counts]


def rle_decode(counts: Iterable[int] | str, shape: tuple[int, int]) -> np.ndarray:
    if isinstance(counts, str):
        counts = json.loads(counts)
    counts = np.asarray(list(counts), dtype=np.int64)
    h, w = int(shape[0]), int(shape[1])
    total = int(counts.sum()) if counts.size else 0
    if total != h * w:
        raise ValueError(f"RLE covers {total} pixels but mask is {h}x{w}={h * w}")
    values = (np.arange(counts.size) % 2).astype(bool)
    return np.repeat(values, counts).reshape(h, w)
