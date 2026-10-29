"""Garbage collection of image files that no kept observation needs."""

from __future__ import annotations

import os
import pathlib
from collections import Counter

from .store import FrameKind, ImageStoreWriter


def _remove(root: pathlib.Path, rel: str | None, stats: Counter, dry_run: bool) -> None:
    if rel is None or rel in stats.setdefault("_seen", set()):
        return
    stats["_seen"].add(rel)
    path = root / rel
    if os.path.lexists(path):
        stats["bytes_freed"] += path.lstat().st_size
        stats["files_removed"] += 1
        if not dry_run:
            path.unlink()


def gc_store(
    writer: ImageStoreWriter,
    max_images_per_object: int | None = None,
    *,
    orphans: bool = False,
    dry_run: bool = False,
) -> dict:
    """Drop pixels nobody keeps.

    1. With ``max_images_per_object``: per merged node set (canonical node + aliases),
       only the best N observations that still have pixels stay ``kept``.
    2. Masks of unkept observations are deleted; rgb/depth of observation frames
       (kind 2) that no kept observation references are deleted. Keyframe and
       sub-keyframe images are never touched. Rows are kept (paths set to NULL).
    3. ``orphans``: files under ``frames/`` and ``masks/`` no row references are deleted.
    """
    stats: Counter = Counter()
    stats["_seen"] = set()
    root = writer.root
    reader = writer.reader()
    conn = writer.conn
    with writer.batch():
        conn.execute(
            "SAVEPOINT gc"
        )  # dry runs apply the row updates, then roll them back
        if max_images_per_object is not None:
            n = max(0, int(max_images_per_object))
            for node in reader.object_nodes():
                obs = [
                    o
                    for o in reader.observations(node, order="score")
                    if o.kept or o.mask_path is not None
                ]
                for o in obs[n:]:
                    if o.kept:
                        stats["observations_unkept"] += 1
                    conn.execute(
                        "UPDATE observations SET kept = 0 WHERE obs_id = ?", (o.obs_id,)
                    )
        rows = conn.execute(
            "SELECT obs_id, mask_path FROM observations WHERE mask_path IS NOT NULL AND kept = 0"
        ).fetchall()
        for oid, rel in rows:
            _remove(root, rel, stats, dry_run)
            stats["masks_dropped"] += 1
            conn.execute(
                "UPDATE observations SET mask_path = NULL WHERE obs_id = ?", (oid,)
            )
        rows = conn.execute(
            "SELECT f.frame_id, f.rgb_path, f.depth_path FROM frames f "
            "WHERE f.kind = ? AND (f.rgb_path IS NOT NULL OR f.depth_path IS NOT NULL) "
            "AND NOT EXISTS (SELECT 1 FROM observations o WHERE o.frame_id = f.frame_id AND o.kept != 0)",
            (int(FrameKind.OBSERVATION),),
        ).fetchall()
        for fid, rgb, depth in rows:
            _remove(root, rgb, stats, dry_run)
            _remove(root, depth, stats, dry_run)
            stats["frames_dropped"] += 1
            conn.execute(
                "UPDATE frames SET rgb_path = NULL, depth_path = NULL WHERE frame_id = ?",
                (fid,),
            )
        if orphans:
            referenced = set()
            for rgb, depth in conn.execute("SELECT rgb_path, depth_path FROM frames"):
                referenced.update(p for p in (rgb, depth) if p)
            referenced.update(
                r[0]
                for r in conn.execute(
                    "SELECT mask_path FROM observations WHERE mask_path IS NOT NULL"
                )
            )
            for sub in ("frames", "masks"):
                base = root / sub
                if not base.exists():
                    continue
                for dirpath, _, files in os.walk(base):
                    for name in files:
                        rel = os.path.relpath(os.path.join(dirpath, name), root)
                        if rel not in referenced and rel not in stats["_seen"]:
                            _remove(root, rel, stats, dry_run)
                            stats["orphans"] += 1
        if dry_run:
            conn.execute("ROLLBACK TO gc")
        conn.execute("RELEASE gc")
    if not dry_run and max_images_per_object is not None:
        writer.set_meta("max_images_per_object", int(max_images_per_object))
    stats.pop("_seen")
    return dict(stats)
