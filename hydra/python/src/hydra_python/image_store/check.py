"""Structural consistency checks of a store (``hydra image-store check``)."""

from __future__ import annotations

import os
import pathlib

from .codec import EXTERNAL_TRACK_BASE
from .store import DB_NAME, ImageStore, _image_shape, _resolve_db


def resolve_store_dir(path: str | os.PathLike) -> pathlib.Path:
    """``<run>/image_store`` for a run directory, else the store directory itself."""
    from .store import _store_path

    p = pathlib.Path(_store_path(path, True)).expanduser()
    if (p / "image_store" / DB_NAME).exists():
        return (p / "image_store").absolute()
    return _resolve_db(p).parent


def check_store(store: str | os.PathLike, *, dsg=None, sample_images: int = 20) -> dict:
    """Structural consistency checks of one store (cheap).

    Always: schema opens; every frame's sensor exists; every observation's frame exists;
    every referenced file exists; sampled RGB/depth images have
    the sensor's resolution. With ``dsg`` (path or
    loaded graph): keyframe anchors are DSG nodes and bound nodes are DSG nodes or merge
    children. Returns ``{"ok", "checks": {name: bool}, "stats": {...}, "store"}``.
    """
    from .poses import JsonDsg, load_dsg

    store_dir = resolve_store_dir(store)
    checks: dict[str, bool] = {}
    stats: dict = {}
    with ImageStore.open(store_dir) as s:
        conn = s._dbs[0].conn

        def one(sql):
            return conn.execute(sql).fetchone()[0]

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
        # informational: a keyframe far from every agent node stays unanchored
        stats["keyframes_unanchored"] = one(
            "SELECT COUNT(*) FROM frames WHERE kind = 0 AND anchor_node_id IS NULL"
        )
        # image sizes vs calibration
        if sample_images:
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
                    shape = _image_shape(store_dir / rel)
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
            else:  # spark_dsg

                def has(n):
                    return graph.has_node(int(n))

            # node_aliases holds the backend merges (merges.json)
            aliases = dict(
                conn.execute("SELECT child_id, parent_id FROM node_aliases").fetchall()
            )

            def known(n):
                return has(n) or n in aliases

            kf_anchors = [
                r[0]
                for r in conn.execute(
                    "SELECT DISTINCT anchor_node_id FROM frames WHERE kind = 0 AND anchor_node_id IS NOT NULL"
                )
            ]
            bad_kf = [n for n in kf_anchors if not known(n)]
            stats["keyframe_anchors_missing_in_dsg"] = len(bad_kf)
            checks["keyframe anchors exist in DSG"] = not bad_kf
            # the indexer's (Khronos) tracks only: other writers' tracks (external ids,
            # e.g. objects a detector created in a database) may bind nodes the DSG
            # never had
            bound = [
                r[0]
                for r in conn.execute(
                    "SELECT DISTINCT node_id FROM track_bindings WHERE track_id < ?",
                    (EXTERNAL_TRACK_BASE,),
                )
            ]
            bad_bound = [n for n in bound if not known(n)]
            stats["bound_nodes_missing_in_dsg"] = len(bad_bound)
            stats["external_bound_nodes_not_in_dsg"] = sum(
                1
                for (n,) in conn.execute(
                    "SELECT DISTINCT node_id FROM track_bindings WHERE track_id >= ?",
                    (EXTERNAL_TRACK_BASE,),
                )
                if not known(n)
            )
            checks["bound nodes exist in DSG or are merge children"] = not bad_bound
    return {
        "ok": all(checks.values()),
        "checks": checks,
        "stats": stats,
        "store": str(store_dir),
    }


__all__ = ["check_store", "resolve_store_dir"]
