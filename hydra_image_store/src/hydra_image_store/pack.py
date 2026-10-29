"""Pack a run's image store into one compressed archive and rebuild it elsewhere.

``pack_store`` (source machine) writes a tar stream (optionally zstd-wrapped)::

    manifest.json            what was packed and how (first member)
    frames/<frame_id>.hisf   one encoded frame per shipped frame (transfer.encode_frame)
    files/<rel path>         kept observation mask PNGs, verbatim
    store.sqlite             consistent snapshot of the index (last member)

The snapshot keeps every row (sensors, frames, observations, bindings, aliases); image
paths of frames that were not shipped are set to NULL (exactly what ``gc`` does), and
shipped frames point at the paths ``unpack_store`` writes (``transfer.decoded_paths``).
``unpack_store`` (destination) streams the archive into ``<dest>/image_store.partial``,
restores downscaled depth to full resolution so the sensor calibration stays valid,
runs :func:`check_store` and only then renames it to ``<dest>/image_store``.

Both ends stream, so the console can run::

    ssh robot hydra-image-store pack RUN - --compression fast | hydra-image-store unpack - DEST

or in python ``pack_store(run, proc.stdin)`` / ``unpack_store(proc.stdout, dest)``.
"""

from __future__ import annotations

import io
import json
import os
import pathlib
import shutil
import sqlite3
import sys
import tarfile
import tempfile
import time
from typing import BinaryIO, Callable

from . import transfer
from .store import DB_NAME, ImageStore, _Db, _resolve_db

MANIFEST = "manifest.json"
FORMAT = "hydra-image-store-pack"
FORMAT_VERSION = 1
_ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"


def resolve_store_dir(path: str | os.PathLike) -> pathlib.Path:
    """``<run>/image_store`` for a run directory, else the store directory itself."""
    p = pathlib.Path(path).expanduser()
    if (p / "image_store" / DB_NAME).exists():
        return (p / "image_store").absolute()
    return _resolve_db(p).parent


def _snapshot(src_db: pathlib.Path, dst: pathlib.Path) -> None:
    import urllib.parse

    uri = "file:" + urllib.parse.quote(str(src_db)) + "?mode=ro"
    src = sqlite3.connect(uri, uri=True)
    try:
        out = sqlite3.connect(str(dst))
        with out:
            src.backup(out)
        out.execute("PRAGMA journal_mode = DELETE")
        out.close()
    finally:
        src.close()


class _TarOut:
    def __init__(self, out, zstd_level: int | None):
        self._close: list[Callable] = []
        if isinstance(out, (str, os.PathLike)) and str(out) == "-":
            raw = sys.stdout.buffer
        elif isinstance(out, (str, os.PathLike)):
            raw = open(out, "wb")
            self._close.append(raw.close)
        else:
            raw = out
        self._counter = raw = _CountingWriter(raw)  # bytes on the wire (after zstd)
        self._zw = None
        if zstd_level is not None:
            zstd = transfer._zstd()
            self._zw = zstd.ZstdCompressor(level=zstd_level, threads=-1).stream_writer(
                raw, closefd=False
            )
            raw = self._zw
        self.tar = tarfile.open(fileobj=raw, mode="w|", format=tarfile.PAX_FORMAT)

    def add(self, name: str, data: bytes) -> None:
        info = tarfile.TarInfo(name)
        info.size = len(data)
        info.mtime = int(time.time())
        info.mode = 0o644
        self.tar.addfile(info, io.BytesIO(data))

    def close(self) -> int:
        self.tar.close()
        if self._zw is not None:
            self._zw.flush(transfer._zstd().FLUSH_FRAME)
            self._zw.close()
        self._counter.flush()
        for c in self._close:
            c()
        return self._counter.n


class _CountingWriter(io.RawIOBase):
    def __init__(self, raw):
        self.raw, self.n = raw, 0

    def writable(self):
        return True

    def write(self, b):
        self.raw.write(b)
        self.n += len(b)
        return len(b)

    def flush(self):
        self.raw.flush()


def pack_store(
    src: str | os.PathLike,
    out: str | os.PathLike | BinaryIO,
    *,
    frames: str = "keyframes",
    compression: str = "fast",
    zstd_level: int | None = None,
    include_masks: bool = True,
    progress: Callable[[int, int], None] | None = None,
) -> dict:
    """Pack the image store of ``src`` (run dir or store dir) into ``out``.

    ``out``: a path (``.zst`` / ``.tzst`` suffix turns on outer zstd), ``"-"`` for
    stdout, or a writable binary file object. ``frames="keyframes"`` ships keyframes
    plus the frames of kept object observations (object images); ``"all"`` ships every
    frame with images. ``compression``: see :mod:`hydra_image_store.transfer`.
    ``zstd_level`` wraps the tar in zstd (mainly shrinks the sqlite snapshot; images are
    already compressed); default 3 for ``.zst`` paths, else off. ``progress(done,
    total)`` is called after each frame. Returns stats.
    """
    if frames not in transfer.FRAME_SELECTIONS:
        raise ValueError(f"frames must be one of {transfer.FRAME_SELECTIONS}")
    if compression not in transfer.COMPRESSION_MODES:
        raise ValueError(f"compression must be one of {transfer.COMPRESSION_MODES}")
    store_dir = resolve_store_dir(src)
    db_path = store_dir / DB_NAME
    if not db_path.exists():
        raise FileNotFoundError(f"no image store at {store_dir}")
    if (
        zstd_level is None
        and isinstance(out, (str, os.PathLike))
        and str(out).endswith((".zst", ".tzst"))
    ):
        zstd_level = 3
    t0 = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="his_pack_") as tmp:
        snap = pathlib.Path(tmp) / DB_NAME
        _snapshot(db_path, snap)
        db = _Db(snap, readonly=False)
        db.root = store_dir  # rows of the snapshot, files of the source store
        store = ImageStore([db])
        selected = transfer.select_frames(store, frames, include_kept_observations=True)
        manifest = {
            "format": FORMAT,
            "version": FORMAT_VERSION,
            "source": str(store_dir),
            "frames": frames,
            "compression": compression,
            "created": time.time(),
            "frame_count": len(selected),
            "info": store.info()[0],
        }
        manifest["info"]["path"] = str(store_dir)
        tar = _TarOut(out, zstd_level)
        stats = {
            "frames": 0,
            "frame_bytes": 0,
            "rgb_bytes": 0,
            "depth_bytes": 0,
            "masks": 0,
            "mask_bytes": 0,
            "missing_files": 0,
        }
        try:
            tar.add(MANIFEST, json.dumps(manifest, indent=1).encode())
            conn = db.conn
            conn.execute("BEGIN")
            shipped = set()
            for i, frame in enumerate(selected):
                blob = transfer.encode_frame(store, frame, compression)
                header = transfer.decode_header(blob)
                if not header["parts"]:
                    stats["missing_files"] += 1
                    continue
                rgb_rel, depth_rel = _paths_from_header(header)
                conn.execute(
                    "UPDATE frames SET rgb_path = ?, depth_path = ? WHERE frame_id = ?",
                    (rgb_rel, depth_rel, frame.frame_id),
                )
                shipped.add(frame.frame_id)
                tar.add(f"frames/{frame.frame_id}.hisf", blob)
                stats["frames"] += 1
                stats["frame_bytes"] += len(blob)
                for p in header["parts"]:
                    stats[f"{p['name']}_bytes"] += int(p["size"])
                if progress is not None:
                    progress(i + 1, len(selected))
            # rows whose images were not shipped lose their paths
            conn.execute("CREATE TEMP TABLE shipped (frame_id INTEGER PRIMARY KEY)")
            conn.executemany("INSERT INTO shipped VALUES (?)", [(f,) for f in shipped])
            conn.execute(
                "UPDATE frames SET rgb_path = NULL, depth_path = NULL "
                "WHERE frame_id NOT IN (SELECT frame_id FROM shipped)"
            )
            masks = conn.execute(
                "SELECT obs_id, mask_path FROM observations WHERE mask_path IS NOT NULL"
            ).fetchall()
            for obs_id, rel in masks:
                path = store_dir / rel if rel else None
                safe = transfer._safe_rel(rel)
                if include_masks and safe and path.exists():
                    data = path.read_bytes()
                    tar.add(f"files/{safe}", data)
                    stats["masks"] += 1
                    stats["mask_bytes"] += len(data)
                    if safe != rel:
                        conn.execute(
                            "UPDATE observations SET mask_path = ? WHERE obs_id = ?",
                            (safe, obs_id),
                        )
                else:
                    conn.execute(
                        "UPDATE observations SET mask_path = NULL WHERE obs_id = ?",
                        (obs_id,),
                    )
            for k, v in (
                ("transfer_source", str(store_dir)),
                ("transfer_frames", frames),
                ("transfer_compression", compression),
                ("transfer_depth_downscale", "2" if compression == "fast" else "1"),
                ("transfer_packed_at", f"{time.time():.3f}"),
            ):
                conn.execute(
                    "INSERT OR REPLACE INTO meta (key, value) VALUES (?, ?)", (k, v)
                )
            conn.execute("COMMIT")
            conn.execute("DROP TABLE shipped")
            conn.execute("VACUUM")
            db.conn.close()
            tar.add(DB_NAME, snap.read_bytes())
        finally:
            stats["archive_bytes"] = tar.close()
    stats["seconds"] = round(time.monotonic() - t0, 3)
    stats["selected"] = len(selected)
    return stats


def _paths_from_header(header: dict) -> tuple[str | None, str | None]:
    fake = {
        "meta": header["meta"],
        "rgb": None,
        "rgb_format": None,
        "depth": None,
        "depth_png": None,
    }
    for p in header["parts"]:
        if p["name"] == "rgb":
            fake["rgb"], fake["rgb_format"] = b"", p["codec"]
        elif p["name"] == "depth":
            if p["codec"] == "delta-zstd":
                fake["depth"] = True
            else:
                fake["depth_png"], fake["depth_ext"] = b"", p.get("ext") or ".png"
    return transfer.decoded_paths(fake)


class _Prefixed(io.RawIOBase):
    """A readable stream with some already-read bytes put back in front."""

    def __init__(self, prefix: bytes, raw):
        self.prefix, self.raw = prefix, raw

    def readable(self):
        return True

    def readinto(self, b):
        if self.prefix:
            n = min(len(b), len(self.prefix))
            b[:n] = self.prefix[:n]
            self.prefix = self.prefix[n:]
            return n
        data = self.raw.read(len(b))
        n = len(data)
        b[:n] = data
        return n


def _safe_member(name: str) -> str:
    p = pathlib.PurePosixPath(name)
    if p.is_absolute() or ".." in p.parts or not p.parts:
        raise ValueError(f"unsafe archive member {name!r}")
    return str(p)


def unpack_store(
    src: str | os.PathLike | BinaryIO,
    dest: str | os.PathLike,
    *,
    store_name: str = "image_store",
    overwrite: bool = False,
    check: bool = True,
    dsg: str | os.PathLike | None = None,
    progress: Callable[[int, int], None] | None = None,
) -> dict:
    """Rebuild a packed store as ``<dest>/<store_name>`` (``dest`` is the run directory).

    ``src`` is an archive path, ``"-"`` (stdin) or a readable binary stream; zstd is
    detected from the magic. The store is assembled in ``<store>.partial`` and renamed
    into place only after :func:`check_store` passes (``check=False`` skips it; ``dsg``,
    default ``<dest>/hydra/backend/dsg.json`` when present, adds the scene graph checks).
    Returns stats including the check report.
    """
    dest = pathlib.Path(dest).expanduser().absolute()
    final = dest / store_name
    if final.exists() and not overwrite:
        raise FileExistsError(f"{final} exists (pass overwrite=True / --overwrite)")
    staging = dest / f"{store_name}.partial"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    close = []
    if isinstance(src, (str, os.PathLike)) and str(src) == "-":
        raw = sys.stdin.buffer
    elif isinstance(src, (str, os.PathLike)):
        raw = open(src, "rb")
        close.append(raw)
    else:
        raw = src
    t0 = time.monotonic()
    stats = {"frames": 0, "files": 0, "bytes_in": 0}
    manifest = None
    try:
        head = raw.read(4)
        stream = _Prefixed(head, raw)
        if head == _ZSTD_MAGIC:
            stream = (
                transfer._zstd()
                .ZstdDecompressor()
                .stream_reader(stream, read_across_frames=True)
            )
        stream = io.BufferedReader(stream, 1 << 20)
        got_db = False
        with tarfile.open(fileobj=stream, mode="r|") as tar:
            for member in tar:
                if not member.isfile():
                    continue
                name = _safe_member(member.name)
                data = tar.extractfile(member).read()
                stats["bytes_in"] += len(data)
                if name == MANIFEST:
                    manifest = json.loads(data)
                    if manifest.get("format") != FORMAT:
                        raise ValueError("not a hydra-image-store pack archive")
                    if int(manifest.get("version", 0)) > FORMAT_VERSION:
                        raise ValueError(
                            f"archive version {manifest['version']} is newer than {FORMAT_VERSION}"
                        )
                elif name.startswith("frames/") and name.endswith(".hisf"):
                    transfer.write_decoded_images(staging, transfer.decode_frame(data))
                    stats["frames"] += 1
                    if progress is not None and manifest:
                        progress(stats["frames"], int(manifest.get("frame_count", 0)))
                elif name.startswith("files/"):
                    rel = name[len("files/") :]
                    out = staging / _safe_member(rel)
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_bytes(data)
                    stats["files"] += 1
                elif name == DB_NAME:
                    (staging / DB_NAME).write_bytes(data)
                    got_db = True
        if manifest is None or not got_db:
            raise ValueError("incomplete archive (missing manifest or store.sqlite)")
        # the snapshot was stored with journal_mode=DELETE; switch the copy back to WAL
        conn = sqlite3.connect(str(staging / DB_NAME))
        conn.execute("PRAGMA journal_mode = WAL")
        conn.execute(
            "INSERT OR REPLACE INTO meta (key, value) VALUES ('transfer_unpacked_at', ?)",
            (f"{time.time():.3f}",),
        )
        conn.commit()
        conn.close()
        stats["manifest"] = {
            k: manifest[k] for k in ("source", "frames", "compression", "frame_count")
        }
        if check:
            if dsg is None and (dest / "hydra/backend/dsg.json").exists():
                dsg = dest / "hydra/backend/dsg.json"
            report = check_store(staging, dsg=dsg)
            stats["check"] = report
            if not report["ok"]:
                raise RuntimeError(
                    "unpacked store failed checks: "
                    + ", ".join(k for k, v in report["checks"].items() if not v)
                    + f" (left in {staging})"
                )
        if final.exists():
            shutil.rmtree(final)
        os.replace(staging, final)
    finally:
        for f in close:
            f.close()
    stats["store"] = str(final)
    stats["seconds"] = round(time.monotonic() - t0, 3)
    return stats


def check_store(store: str | os.PathLike, *, dsg=None, sample_images: int = 20) -> dict:
    """Structural consistency checks of one store (cheap; used after ``unpack``).

    Always: schema opens; every frame's sensor exists; every observation's frame exists;
    every referenced file exists; keyframes are anchored; sampled RGB/depth images have
    the sensor's resolution (needs cv2 or Pillow, else skipped). With ``dsg`` (path or
    loaded graph): keyframe anchors are DSG nodes and bound nodes are DSG nodes or merge
    children. Returns ``{"ok", "checks": {name: bool}, "stats": {...}}``.
    """
    from .poses import JsonDsg, load_dsg

    store_dir = resolve_store_dir(store)
    checks: dict[str, bool] = {}
    stats: dict = {}
    with ImageStore.open(store_dir) as s:
        conn = s._dbs[0].conn
        one = lambda sql: conn.execute(sql).fetchone()[0]  # noqa: E731
        stats["info"] = s.info()[0]
        stats["frames_without_sensor"] = one(
            "SELECT COUNT(*) FROM frames WHERE sensor_id NOT IN (SELECT sensor_id FROM sensors)"
        )
        checks["frames reference existing sensors"] = (
            stats["frames_without_sensor"] == 0
        )
        stats["observations_without_frame"] = one(
            "SELECT COUNT(*) FROM observations WHERE frame_id NOT IN (SELECT frame_id FROM frames)"
        )
        checks["observations reference existing frames"] = (
            stats["observations_without_frame"] == 0
        )
        refs = [
            r[0]
            for r in conn.execute(
                "SELECT rgb_path FROM frames WHERE rgb_path IS NOT NULL UNION ALL "
                "SELECT depth_path FROM frames WHERE depth_path IS NOT NULL UNION ALL "
                "SELECT mask_path FROM observations WHERE mask_path IS NOT NULL"
            )
        ]
        missing = [r for r in refs if not (store_dir / r).exists()]
        stats["referenced_files"] = len(refs)
        stats["referenced_files_missing"] = len(missing)
        stats["missing_examples"] = missing[:5]
        checks["all referenced files exist"] = not missing
        stats["keyframes_unanchored"] = one(
            "SELECT COUNT(*) FROM frames WHERE kind = 0 AND anchor_node_id IS NULL"
        )
        checks["keyframes anchored"] = stats["keyframes_unanchored"] == 0
        # image sizes vs calibration
        try:
            transfer._need_image_lib("checking image sizes")
            have_lib = True
        except transfer.MissingDependency:
            have_lib = False
        if have_lib and sample_images:
            bad = []
            rows = conn.execute(
                "SELECT f.frame_id, f.rgb_path, f.depth_path, s.width, s.height FROM frames f "
                "JOIN sensors s ON s.sensor_id = f.sensor_id "
                "WHERE f.rgb_path IS NOT NULL OR f.depth_path IS NOT NULL "
                f"ORDER BY f.frame_id LIMIT {int(sample_images)}"
            ).fetchall()
            for fid, rgb, depth, w, h in rows:
                for rel, kind in ((rgb, "rgb"), (depth, "depth")):
                    if rel is None or not (store_dir / rel).exists():
                        continue
                    shape = _image_hw(store_dir / rel, kind)
                    if shape is not None and tuple(shape) != (h, w):
                        bad.append(
                            {"frame_id": fid, kind: list(shape), "calib": [h, w]}
                        )
            stats["image_size_mismatches"] = bad[:5]
            stats["images_sampled"] = len(rows)
            checks["sampled images match calibration size"] = not bad
        if dsg is not None:
            graph = load_dsg(dsg) if isinstance(dsg, (str, os.PathLike)) else dsg
            if isinstance(graph, JsonDsg):
                has = graph.has_node
                merges = graph.merges
            else:  # spark_dsg
                has = lambda n: graph.has_node(int(n))  # noqa: E731
                merges = {}
            aliases = dict(
                conn.execute("SELECT child_id, parent_id FROM node_aliases").fetchall()
            )
            kf_anchors = [
                r[0]
                for r in conn.execute(
                    "SELECT DISTINCT anchor_node_id FROM frames WHERE kind = 0 AND anchor_node_id IS NOT NULL"
                )
            ]
            bad_kf = [
                n
                for n in kf_anchors
                if not has(n) and n not in aliases and n not in merges
            ]
            stats["keyframe_anchors_missing_in_dsg"] = len(bad_kf)
            checks["keyframe anchors exist in DSG"] = not bad_kf
            bound = [
                r[0]
                for r in conn.execute("SELECT DISTINCT node_id FROM track_bindings")
            ]
            bad_bound = [
                n for n in bound if not has(n) and n not in aliases and n not in merges
            ]
            stats["bound_nodes_missing_in_dsg"] = len(bad_bound)
            checks["bound nodes exist in DSG or are merge children"] = not bad_bound
    return {
        "ok": all(checks.values()),
        "checks": checks,
        "stats": stats,
        "store": str(store_dir),
    }


def _image_hw(path: pathlib.Path, kind: str):
    cv2 = transfer._cv2()
    if cv2 is not None:
        img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        return None if img is None else img.shape[:2]
    from PIL import Image

    with Image.open(path) as im:
        return im.height, im.width


__all__ = ["check_store", "pack_store", "resolve_store_dir", "unpack_store"]
