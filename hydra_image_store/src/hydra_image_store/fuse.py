"""Fuse per-robot / per-session image stores into one store keyed by a fused scene graph.

Hydra-Multi renumbers node ids when it fuses scene graphs (each robot's nodes get new
indices, agent layers move to the fused robot's prefix) and writes the bookkeeping to
``backend/fusion_node_map.json``::

    {"robots": {"<fused robot id>": [[original node id, fused node id], ...]},
     "merges": [[fused child id, fused parent id], ...]}

``fuse_stores`` copies every source store into a new store where

* frame / sensor / observation ids and ``robot_id`` are re-keyed to the fused robot id
  (ids are ``robot_id << 48 | seq``, so the sequence part is kept),
* ``anchor_node_id``, ``track_bindings.node_id`` and node aliases are rewritten into fused
  node ids (source merges are resolved first, the fused merges are added as aliases),
* odometric poses are moved into the fused world frame (``world_T_robot``) so the
  fallback pose of unanchored frames stays meaningful,
* image files are hardlinked (or copied / symlinked) under ``robot_<id>/``,
* a ``sources`` table records which run / session each fused robot came from.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import os
import pathlib
import sqlite3
from collections.abc import Mapping, Sequence

import numpy as np

from .codec import SEQ_MASK, blob_to_transform, make_id, transform_to_blob
from .store import DB_NAME, _place_file, _resolve_db, schema_sql

logger = logging.getLogger(__name__)

#: robot id -> agent layer prefix used by Kimera-PGMO / Hydra
ROBOT_PREFIXES = "abcdefgh"

SOURCES_SQL = """
CREATE TABLE IF NOT EXISTS sources (
  robot_id INTEGER PRIMARY KEY,
  source_robot_id INTEGER NOT NULL,
  source_store TEXT NOT NULL,
  session TEXT
);
"""


@dataclasses.dataclass
class FuseSource:
    """One store to fuse."""

    store: pathlib.Path
    #: robot id of this source in the fused scene graph
    robot_id: int
    #: free-form session label (e.g. run name / day)
    session: str | None = None
    #: transform of the robot frame into the fused world frame (default identity)
    world_T_robot: np.ndarray | None = None


def _rekey(entity_id: int | None, robot_id: int) -> int | None:
    return None if entity_id is None else make_id(robot_id, entity_id & SEQ_MASK)


def _symbol(node_id: int) -> tuple[str, int]:
    return chr(node_id >> 56), node_id & ((1 << 56) - 1)


def _with_prefix(node_id: int, prefix: str) -> int:
    return (ord(prefix) << 56) | (node_id & ((1 << 56) - 1))


def load_node_map(path: str | os.PathLike) -> tuple[dict[int, dict[int, int]], dict[int, int]]:
    """Read ``fusion_node_map.json`` as ({robot: {original: fused}}, {child: parent})."""
    with open(path) as fin:
        data = json.load(fin)

    robots = {int(r): {int(o): int(f) for o, f in pairs} for r, pairs in data["robots"].items()}
    merges = {int(c): int(p) for c, p in data.get("merges", [])}
    return robots, merges


def load_world_T_robot(path: str | os.PathLike) -> np.ndarray:
    """Read a 4x4 transform written as whitespace separated text (``world_T_robot.txt``)."""
    values = np.loadtxt(path).reshape(-1)
    if values.size != 16:
        raise ValueError(f"{path}: expected 16 values, got {values.size}")
    return values.reshape(4, 4)


class _NodeMapper:
    """Maps node ids of one source store into fused node ids."""

    def __init__(self, conn: sqlite3.Connection, node_map: Mapping[int, int], robot_id: int):
        self.node_map = node_map
        self.prefix = ROBOT_PREFIXES[robot_id]
        self.parents = dict(conn.execute("SELECT child_id, parent_id FROM node_aliases"))
        self.missing: set[int] = set()

    def canonical(self, node_id: int) -> int:
        seen = set()
        while node_id in self.parents and node_id not in seen:
            seen.add(node_id)
            node_id = self.parents[node_id]
        return node_id

    def __call__(self, node_id: int | None) -> int | None:
        if node_id is None:
            return None

        node_id = self.canonical(node_id)
        fused = self.node_map.get(node_id)
        if fused is None:
            # agent layers are moved to the fused robot's prefix before renumbering
            category, _ = _symbol(node_id)
            if category in ROBOT_PREFIXES:
                fused = self.node_map.get(_with_prefix(node_id, self.prefix))

        if fused is None:
            self.missing.add(node_id)
        return fused


def _source_robot_id(conn: sqlite3.Connection, store: pathlib.Path) -> int:
    ids = {r[0] for r in conn.execute("SELECT DISTINCT robot_id FROM sensors")}
    ids |= {r[0] for r in conn.execute("SELECT DISTINCT robot_id FROM frames")}
    if len(ids) > 1:
        raise ValueError(f"{store} holds several robots {sorted(ids)}; fuse one robot per source")
    return ids.pop() if ids else 0


def fuse_stores(
    out: str | os.PathLike,
    sources: Sequence[FuseSource],
    node_map: Mapping[int, Mapping[int, int]],
    merges: Mapping[int, int] | None = None,
    *,
    link: str = "hardlink",
    overwrite: bool = False,
) -> dict:
    """Write the fused store to ``out`` and return per-source statistics."""
    out = pathlib.Path(out)
    db_path = out / DB_NAME
    if db_path.exists():
        if not overwrite:
            raise FileExistsError(f"{db_path} exists (pass overwrite=True)")
        for suffix in ("", "-wal", "-shm"):
            pathlib.Path(f"{db_path}{suffix}").unlink(missing_ok=True)

    fused_ids = [s.robot_id for s in sources]
    if len(set(fused_ids)) != len(fused_ids):
        raise ValueError(f"fused robot ids must be unique: {fused_ids}")

    out.mkdir(parents=True, exist_ok=True)
    dst = sqlite3.connect(db_path)
    dst.executescript(schema_sql())
    dst.executescript(SOURCES_SQL)
    stats = {"store": str(out), "sources": []}
    with dst:
        for source in sources:
            stats["sources"].append(
                _copy_source(dst, out, source, node_map.get(source.robot_id, {}), link)
            )

        merges = merges or {}
        dst.executemany(
            "INSERT OR REPLACE INTO node_aliases (child_id, parent_id) VALUES (?, ?)",
            sorted(merges.items()),
        )
        stats["fused_merges"] = len(merges)

    dst.close()
    return stats


def _copy_source(
    dst: sqlite3.Connection,
    out: pathlib.Path,
    source: FuseSource,
    node_map: Mapping[int, int],
    link: str,
) -> dict:
    src_db = _resolve_db(source.store)
    src_root = src_db.parent
    src = sqlite3.connect(f"file:{src_db}?mode=ro", uri=True)
    robot = source.robot_id
    src_robot = _source_robot_id(src, src_db)
    mapper = _NodeMapper(src, node_map, robot)
    world_T_robot = np.eye(4) if source.world_T_robot is None else source.world_T_robot
    file_prefix = pathlib.Path(f"robot_{robot}")
    num_files = 0

    def place(rel: str | None) -> str | None:
        nonlocal num_files
        if rel is None:
            return None
        src_file = src_root / rel
        new_rel = file_prefix / rel
        if src_file.exists() and not (out / new_rel).exists():
            _place_file(src_file, out / new_rel, link)
            num_files += 1
        return str(new_rel)

    sensor_cols = "sensor_id, robot_id, name, width, height, fx, fy, cx, cy, depth_scale, body_T_sensor"
    for row in src.execute(f"SELECT {sensor_cols} FROM sensors"):
        row = list(row)
        row[0] = _rekey(row[0], robot)
        row[1] = robot
        dst.execute(f"INSERT INTO sensors ({sensor_cols}) VALUES ({','.join('?' * 11)})", row)

    frame_cols = (
        "frame_id, robot_id, sensor_id, timestamp_ns, kind, world_T_body_odom, "
        "anchor_node_id, anchor_T_body, rgb_path, depth_path"
    )
    num_frames = num_unanchored = 0
    for row in src.execute(f"SELECT {frame_cols} FROM frames"):
        (frame_id, _, sensor_id, stamp, kind, odom, anchor, anchor_T_body, rgb, depth) = row
        fused_anchor = mapper(anchor)
        if anchor is not None and fused_anchor is None:
            anchor_T_body = None
        num_unanchored += fused_anchor is None
        odom = transform_to_blob(world_T_robot @ blob_to_transform(odom))
        dst.execute(
            f"INSERT INTO frames ({frame_cols}) VALUES ({','.join('?' * 10)})",
            (
                _rekey(frame_id, robot),
                robot,
                _rekey(sensor_id, robot),
                stamp,
                kind,
                odom,
                fused_anchor,
                anchor_T_body,
                place(rgb),
                place(depth),
            ),
        )
        num_frames += 1

    obs_cols = (
        "obs_id, robot_id, track_id, frame_id, timestamp_ns, bbox2d_min_x, bbox2d_min_y, "
        "bbox2d_max_x, bbox2d_max_y, bbox3d_min_x, bbox3d_min_y, bbox3d_min_z, bbox3d_max_x, "
        "bbox3d_max_y, bbox3d_max_z, mask_rle, mask_path, score, kept"
    )
    num_obs = 0
    for row in src.execute(f"SELECT {obs_cols} FROM observations"):
        row = list(row)
        row[0] = _rekey(row[0], robot)
        row[1] = robot
        row[3] = _rekey(row[3], robot)
        # 3d boxes are in the robot's world frame
        if row[9] is not None:
            corners = np.array([row[9:12], row[12:15]], dtype=float)
            moved = (world_T_robot[:3, :3] @ corners.T).T + world_T_robot[:3, 3]
            row[9:12] = moved.min(axis=0).tolist()
            row[12:15] = moved.max(axis=0).tolist()
        row[16] = place(row[16])
        dst.execute(f"INSERT INTO observations ({obs_cols}) VALUES ({','.join('?' * 19)})", row)
        num_obs += 1

    num_bindings = num_dropped = 0
    for track_id, node_id in src.execute("SELECT track_id, node_id FROM track_bindings"):
        fused = mapper(node_id)
        if fused is None:
            num_dropped += 1
            continue
        dst.execute(
            "INSERT INTO track_bindings (robot_id, track_id, node_id) VALUES (?, ?, ?)",
            (robot, track_id, fused),
        )
        num_bindings += 1

    dst.execute(
        "INSERT INTO sources (robot_id, source_robot_id, source_store, session) VALUES (?, ?, ?, ?)",
        (robot, src_robot, str(src_root.resolve()), source.session),
    )
    src.close()

    if mapper.missing:
        logger.warning(
            "%s: %d node ids are not in the fused graph (bindings / anchors dropped)",
            src_root, len(mapper.missing),
        )

    return {
        "store": str(src_root),
        "robot_id": robot,
        "source_robot_id": src_robot,
        "frames": num_frames,
        "unanchored_frames": num_unanchored,
        "observations": num_obs,
        "track_bindings": num_bindings,
        "dropped_bindings": num_dropped,
        "missing_nodes": len(mapper.missing),
        "files": num_files,
    }
