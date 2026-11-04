"""Minimal OpenEXR reader (numpy + zlib only).

Hydra's ``InputData::save`` writes depth/range images with OpenCV as single-part
scanline EXR (``IMWRITE_EXR_TYPE_FLOAT``) with no, RLE or ZIP compression
(``SaveOptions::float_compression``). Reading EXR through OpenCV needs
``OPENCV_IO_ENABLE_OPENEXR=1`` in the environment *before* ``cv2`` is first used, which
a library cannot guarantee (the host process may have imported cv2 already), so the
indexer reads EXR with this module. None of imageio's backends reads EXR out of the box
either (its ``opencv`` plugin has the same restriction, ``freeimage`` needs a binary
download), so :func:`~hydra_python.image_store.inputdata.decode_exr` only falls back to
imageio's ``opencv`` plugin for encodings this module does not handle (PIZ, PXR24, B44,
DWA, tiled or multi-part files), which Hydra never writes.
"""

from __future__ import annotations

import struct
import zlib

import numpy as np

MAGIC = 20000630

_COMPRESSION = {0: "none", 1: "rle", 2: "zips", 3: "zip"}
_LINES_PER_CHUNK = {0: 1, 1: 1, 2: 1, 3: 16}
_PIXEL_DTYPE = {0: np.dtype("<u4"), 1: np.dtype("<f2"), 2: np.dtype("<f4")}


class UnsupportedExr(ValueError):
    """The file is valid EXR but uses a feature this reader does not implement."""


def _cstr(buf: bytes, pos: int) -> tuple[str, int]:
    end = buf.index(b"\0", pos)
    return buf[pos:end].decode("latin-1"), end + 1


def _parse_header(buf: bytes) -> tuple[dict, int]:
    if len(buf) < 8 or struct.unpack_from("<i", buf, 0)[0] != MAGIC:
        raise ValueError("not an OpenEXR file")
    flags = struct.unpack_from("<I", buf, 4)[0]
    if flags & 0x200:
        raise UnsupportedExr("tiled EXR")
    if flags & 0x1000:
        raise UnsupportedExr("multi-part EXR")
    if flags & 0x800:
        raise UnsupportedExr("deep EXR")
    pos = 8
    header: dict = {}
    while True:
        name, pos = _cstr(buf, pos)
        if not name:
            break
        typ, pos = _cstr(buf, pos)
        size = struct.unpack_from("<i", buf, pos)[0]
        pos += 4
        value = buf[pos : pos + size]
        pos += size
        if typ == "chlist":
            chans = []
            p = 0
            while value[p] != 0:
                cname, p = _cstr(value, p)
                ptype, _plinear, xs, ys = struct.unpack_from("<iB3xii", value, p)
                p += 16
                chans.append((cname, ptype, xs, ys))
            header[name] = chans
        elif typ == "compression":
            header[name] = value[0]
        elif typ == "box2i":
            header[name] = struct.unpack("<iiii", value)
        else:
            header[name] = value
    return header, pos


def _predictor(data: bytes) -> np.ndarray:
    # OpenEXR: t[i] = t[i-1] + t[i] - 128 (mod 256), for i >= 1
    t = np.frombuffer(data, dtype=np.uint8).astype(np.int64)
    if t.size:
        t[1:] -= 128
        t = np.cumsum(t) & 0xFF
    return t.astype(np.uint8)


def _deinterleave(t: np.ndarray) -> bytes:
    half = (t.size + 1) // 2
    out = np.empty_like(t)
    out[0::2] = t[:half]
    out[1::2] = t[half:]
    return out.tobytes()


def _rle_decode(data: bytes, expected: int) -> bytes:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        count = struct.unpack_from("b", data, i)[0]
        i += 1
        if count < 0:
            out += data[i : i + -count]
            i += -count
        else:
            out += data[i : i + 1] * (count + 1)
            i += 1
    if len(out) != expected:
        raise ValueError(
            f"EXR RLE chunk decoded to {len(out)} bytes, expected {expected}"
        )
    return bytes(out)


def read_exr(data: bytes) -> np.ndarray:
    """Decode an EXR file's bytes. One channel gives an ``HxW`` array, several give
    ``HxWxC`` in OpenCV order (B, G, R, A when those are the channels, else the file's
    alphabetical order). FLOAT/HALF become float32, UINT uint32."""
    buf = bytes(data)
    header, pos = _parse_header(buf)
    comp = header.get("compression", 0)
    if comp not in _COMPRESSION:
        raise UnsupportedExr(f"EXR compression {comp}")
    chans = header["channels"]
    if any(xs != 1 or ys != 1 for _, _, xs, ys in chans):
        raise UnsupportedExr("subsampled EXR channels")
    x0, y0, x1, y1 = header["dataWindow"]
    width, height = x1 - x0 + 1, y1 - y0 + 1
    lines = _LINES_PER_CHUNK[comp]
    n_chunks = (height + lines - 1) // lines
    offsets = struct.unpack_from(f"<{n_chunks}Q", buf, pos)
    dtypes = [_PIXEL_DTYPE[ptype] for _, ptype, _, _ in chans]
    line_bytes = sum(dt.itemsize for dt in dtypes) * width
    planes = [np.empty((height, width), dtype=dt) for dt in dtypes]
    for off in offsets:
        y, size = struct.unpack_from("<ii", buf, off)
        chunk = buf[off + 8 : off + 8 + size]
        n_lines = min(lines, y1 - y + 1)
        expected = line_bytes * n_lines
        if comp == 0 or size == expected:
            raw = chunk  # stored uncompressed (also when compression did not help)
        elif comp == 1:
            raw = _deinterleave(_predictor(_rle_decode(chunk, expected)))
        else:
            raw = _deinterleave(_predictor(zlib.decompress(chunk)))
        if len(raw) != expected:
            raise ValueError(
                f"EXR chunk at line {y}: {len(raw)} bytes, expected {expected}"
            )
        p = 0
        for li in range(n_lines):
            row = y - y0 + li
            for ci, dt in enumerate(dtypes):
                nb = dt.itemsize * width
                planes[ci][row] = np.frombuffer(raw, dtype=dt, count=width, offset=p)
                p += nb
    out = [pl.astype(np.float32) if pl.dtype.kind == "f" else pl for pl in planes]
    if len(out) == 1:
        return out[0]
    names = [c[0] for c in chans]
    order = [n for n in ("B", "G", "R", "A") if n in names]
    if len(order) != len(names):
        order = names
    return np.stack([out[names.index(n)] for n in order], axis=-1)


__all__ = ["UnsupportedExr", "read_exr"]
