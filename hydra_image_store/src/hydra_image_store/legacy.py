"""Convert pre-store run directories (images/, agents/, subkeyframes/, sam3_objects/)."""

from __future__ import annotations

import bisect
import json
import logging
import os
import pathlib
import re
from collections import Counter

import numpy as np

from .codec import LEGACY_TRACK_BASE, make_node_id, pose_matrix, quat_to_rot, sam3_track_id
from .poses import JsonDsg
from .store import FrameKind, ImageStoreWriter

logger = logging.getLogger(__name__)

_OBJECT_DIR = re.compile(r"^([A-Za-z])_?(\d+)$")
_SUBKF_TYPES = ("SubKeyframeNodeAttributes",)


def _load_json(path: pathlib.Path) -> dict:
    with open(path) as f:
        return json.load(f)


def _matrix16(values) -> np.ndarray | None:
    if values is None or len(values) != 16:
        return None
    return np.asarray(values, dtype=float).reshape(4, 4)


def _existing(path: pathlib.Path) -> pathlib.Path | None:
    """The real file behind ``path`` (symlinks resolved) or None if it is missing."""
    if not os.path.lexists(path):
        return None
    real = pathlib.Path(os.path.realpath(path))
    return real if real.is_file() else None


class _DsgIndex:
    def __init__(self, dsg: JsonDsg | None):
        self.dsg = dsg
        self.agent_by_ts: dict[int, int] = {}
        self.agent_by_folder: dict[str, int] = {}
        self.subkf_by_ts: dict[int, int] = {}
        self.subkf_by_folder: dict[str, int] = {}
        self.object_by_folder: dict[str, int] = {}
        if dsg is None:
            return
        for nid, node in dsg.nodes.items():
            attrs = node.get("attributes", {})
            kind = attrs.get("type", "")
            folder = os.path.basename(str(attrs.get("image_folder") or "").rstrip("/"))
            if kind in _SUBKF_TYPES:
                self.subkf_by_ts[int(attrs.get("timestamp", -1))] = nid
                if folder:
                    self.subkf_by_folder[folder] = nid
            elif "world_R_body" in attrs and "timestamp" in attrs:
                self.agent_by_ts.setdefault(int(attrs["timestamp"]), nid)
                if folder:
                    self.agent_by_folder[folder] = nid
            elif folder:
                self.object_by_folder[folder] = nid

    def has(self, nid: int) -> bool:
        return self.dsg is not None and self.dsg.has_node(nid)

    def subkf_anchor(self, stem: str, ts: int):
        """(anchor_node_id, anchor_T_body, world_T_body) from a SubKeyframe node."""
        nid = self.subkf_by_folder.get(stem, self.subkf_by_ts.get(ts))
        if nid is None:
            return None
        a = self.dsg.attributes(nid)
        anchor = a.get("anchor_node_id")
        if anchor is None:
            return None
        q = a.get("anchor_R_subframe") or {"w": 1, "x": 0, "y": 0, "z": 0}
        anchor_T_body = pose_matrix(a.get("anchor_t_subframe", [0, 0, 0]), quat_to_rot(q["w"], q["x"], q["y"], q["z"]))
        world_T_anchor = self.dsg.pose(anchor)
        if world_T_anchor is not None:
            world_T_body = world_T_anchor @ anchor_T_body
        else:
            world_T_body = pose_matrix(a.get("position", [0, 0, 0]))
        return int(anchor), anchor_T_body, world_T_body


class _AnchorIndex:
    """Time-sorted (ts, anchor, anchor_T_body, world_T_body_odom) of imported frames."""

    def __init__(self):
        self.ts: list[int] = []
        self.rows: list[tuple] = []

    def add(self, ts, anchor, anchor_T_body, odom):
        if anchor is None and odom is None:
            return
        i = bisect.bisect_left(self.ts, ts)
        self.ts.insert(i, ts)
        self.rows.insert(i, (anchor, anchor_T_body, odom))

    def nearest(self, ts: int):
        if not self.ts:
            return None, None
        i = bisect.bisect_left(self.ts, ts)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.ts)]
        j = min(cands, key=lambda k: abs(self.ts[k] - ts))
        return self.rows[j], abs(self.ts[j] - ts)


def _calib(run: pathlib.Path) -> dict | None:
    for rel in ("agents/camera_calib.json", "subkeyframes/camera_calib.json", "camera_calib.json"):
        p = run / rel
        if p.exists():
            return _load_json(p)
    return None


def find_dsg(run: pathlib.Path) -> pathlib.Path | None:
    for rel in ("hydra/backend/dsg.json", "hydra/backend/dsg_with_mesh.json", "hydra/frontend/dsg.json", "dsg.json"):
        if (run / rel).exists():
            return run / rel
    return None


def import_legacy(
    run_dir,
    out,
    dsg=None,
    *,
    robot_id: int = 0,
    link: str = "hardlink",
    sam3_dirs: tuple[str, ...] = ("sam3_objects",),
) -> dict:
    """Convert a legacy run directory into an image store at ``out`` (never writes to the run).

    Agents bind to DSG agent nodes by ``image_folder`` or timestamp. Sub-keyframes use the
    DSG SubKeyframe node's anchor when present, else the nearest agent keyframe in time.
    Object observation frames carried no pose, so they are anchored like the nearest
    imported keyframe / sub-keyframe in time (``meta.legacy_obs_max_dt_ns`` records the
    worst gap). Khronos object folders bind ``LEGACY_TRACK_BASE | index`` to the node whose
    ``image_folder`` names the folder (else NodeSymbol(<letter>, index)); SAM3 folders
    bind ``sam3_track_id(node)``.
    """
    run = pathlib.Path(run_dir).expanduser().resolve()
    stats: Counter = Counter()
    dsg_path = pathlib.Path(dsg) if dsg is not None else find_dsg(run)
    index = _DsgIndex(JsonDsg.load(dsg_path) if dsg_path is not None else None)
    calib = _calib(run)

    writer = ImageStoreWriter(out, robot_id)
    anchors = _AnchorIndex()
    max_dt = 0
    try:
        with writer.batch():
            if calib is None:
                logger.warning("%s: no camera_calib.json; sensor intrinsics are zero", run)
                calib = {}
            sensor = writer.add_sensor(
                "camera",
                calib.get("width", 0),
                calib.get("height", 0),
                calib.get("fx", 0.0),
                calib.get("fy", 0.0),
                calib.get("cx", 0.0),
                calib.get("cy", 0.0),
                calib.get("depth_scale", 0.001),
                _matrix16(calib.get("body_T_sensor")),
            )

            # --- agent keyframes ---------------------------------------------------------
            for meta_path in sorted((run / "agents").glob("agent_*_meta.json")):
                meta = _load_json(meta_path)
                stem = meta_path.name[: -len("_meta.json")]
                node_ts = int(meta["timestamp_ns"])
                frame_ts = int(meta.get("frame_timestamp_ns", node_ts))
                node = index.agent_by_folder.get(stem, index.agent_by_ts.get(node_ts))
                odom = _matrix16(meta.get("world_T_body"))
                odom = np.eye(4) if odom is None else odom
                rgb = _existing(meta_path.parent / meta.get("rgb_file", f"{stem}_rgb.jpg"))
                depth = _existing(meta_path.parent / meta.get("depth_file", f"{stem}_depth.png"))
                writer.add_frame_ref(
                    sensor, frame_ts, kind=FrameKind.KEYFRAME, world_T_body_odom=odom, rgb=rgb,
                    depth=depth, anchor_node_id=node, anchor_T_body=np.eye(4) if node is not None else None,
                    link=link,
                )
                stats["keyframes"] += 1
                if node is None:
                    stats["keyframes_unbound"] += 1
                anchors.add(frame_ts, node, np.eye(4), odom)

            # --- sub-keyframes -----------------------------------------------------------
            subkf_rows = []
            for meta_path in sorted((run / "subkeyframes").glob("subkf_*_meta.json")):
                meta = _load_json(meta_path)
                stem = meta_path.name[: -len("_meta.json")]
                ts = int(meta["timestamp_ns"])
                rgb = _existing(meta_path.parent / meta.get("rgb_file", f"{stem}_rgb.jpg"))
                depth = _existing(meta_path.parent / meta.get("depth_file", f"{stem}_depth.png"))
                found = index.subkf_anchor(stem, ts) if index.dsg is not None else None
                own_odom = _matrix16(meta.get("world_T_body"))
                if found is not None:
                    anchor, anchor_T_body, odom = found
                    odom = own_odom if own_odom is not None else odom
                    stats["subkeyframes_dsg_anchor"] += 1
                else:
                    near, dt = anchors.nearest(ts)
                    anchor, anchor_T_body, odom = None, None, own_odom
                    if near is not None:
                        anchor, a_T_b, a_odom = near
                        odom = own_odom if own_odom is not None else a_odom
                        anchor_T_body = np.linalg.inv(a_odom) @ odom if own_odom is not None else a_T_b
                    stats["subkeyframes_nearest_anchor" if anchor is not None else "subkeyframes_unanchored"] += 1
                subkf_rows.append((ts, anchor, anchor_T_body, odom))
                if _frame_kind(writer, sensor, ts) is not None:
                    stats["subkeyframes_deduped"] += 1  # same timestamp as an imported keyframe
                writer.add_frame_ref(
                    sensor, ts, kind=FrameKind.SUBKEYFRAME, world_T_body_odom=odom, rgb=rgb, depth=depth,
                    anchor_node_id=anchor, anchor_T_body=anchor_T_body, link=link,
                )
                stats["subkeyframes"] += 1
            for row in subkf_rows:
                anchors.add(*row)

            # --- Khronos object folders ----------------------------------------------------
            images = run / "images"
            for folder in sorted(images.iterdir()) if images.is_dir() else []:
                m = _OBJECT_DIR.match(folder.name)
                if not folder.is_dir() or m is None:
                    if folder.is_dir():
                        stats["object_dirs_skipped"] += 1
                    continue
                idx = int(m.group(2))
                node = index.object_by_folder.get(folder.name, make_node_id(m.group(1), idx))
                if index.dsg is not None and not index.has(node):
                    stats["object_dirs_without_node"] += 1
                track = LEGACY_TRACK_BASE | idx
                writer.bind_track(track, node)
                stats["objects"] += 1
                n, dt = _import_obs_dir(writer, sensor, folder, track, anchors, link, stats, "object_observations")
                max_dt = max(max_dt, dt)

            # --- SAM3-discovered objects -----------------------------------------------------
            for name in sam3_dirs:
                base = run / name
                for folder in sorted(base.iterdir()) if base.is_dir() else []:
                    m = _OBJECT_DIR.match(folder.name)
                    if not folder.is_dir() or m is None:
                        continue
                    node = make_node_id(m.group(1), int(m.group(2)))
                    track = sam3_track_id(node)
                    writer.bind_track(track, node)
                    stats["sam3_objects"] += 1
                    n, dt = _import_obs_dir(writer, sensor, folder, track, anchors, link, stats, "sam3_observations")
                    max_dt = max(max_dt, dt)

            if index.dsg is not None and index.dsg.merges:
                writer.record_merges(index.dsg.merges)
                stats["merges"] = len(index.dsg.merges)
            writer.set_meta("imported_from", str(run))
            if dsg_path is not None:
                writer.set_meta("imported_dsg", str(dsg_path))
            writer.set_meta("legacy_obs_max_dt_ns", max_dt)
    finally:
        writer.close()
    stats["obs_anchor_max_dt_ms"] = round(max_dt / 1e6, 1)
    return dict(stats)


def _frame_kind(writer: ImageStoreWriter, sensor: int, ts: int):
    row = writer.conn.execute("SELECT kind FROM frames WHERE sensor_id = ? AND timestamp_ns = ?", (sensor, ts)).fetchone()
    return None if row is None else FrameKind(row[0])


def _import_obs_dir(writer, sensor, folder: pathlib.Path, track: int, anchors: _AnchorIndex,
                    link: str, stats: Counter, counter: str) -> tuple[int, int]:
    n, max_dt = 0, 0
    for meta_path in sorted(folder.glob("frame_*_meta.json")):
        meta = _load_json(meta_path)
        prefix = meta_path.name[: -len("_meta.json")]
        ts = int(meta["timestamp_ns"])
        rgb = _existing(folder / meta.get("rgb_file", f"{prefix}_rgb.jpg"))
        depth = _existing(folder / meta.get("depth_file", f"{prefix}_depth.png"))
        mask = _existing(folder / meta.get("mask_file", f"{prefix}_mask.png"))
        near, dt = anchors.nearest(ts)
        anchor, anchor_T_body, odom = (None, None, None) if near is None else near
        max_dt = max(max_dt, dt or 0)
        fid = writer.add_frame_ref(
            sensor, ts, kind=FrameKind.OBSERVATION, world_T_body_odom=odom, rgb=rgb, depth=depth,
            anchor_node_id=anchor, anchor_T_body=anchor_T_body, link=link,
        )
        b2 = meta.get("bbox_2d")
        bbox2d = None if not b2 else (b2["min_x"], b2["min_y"], b2["max_x"], b2["max_y"])
        bbox3d = None
        if meta.get("bbox_min") is not None and meta.get("bbox_max") is not None:
            bbox3d = (meta["bbox_min"], meta["bbox_max"])
        writer.add_observation(
            track, fid, timestamp_ns=ts, bbox2d=bbox2d, bbox3d=bbox3d, mask_rle=meta.get("mask_rle"),
            mask_path=mask, score=float(meta.get("score", 0.0)), kept=True, link=link,
        )
        n += 1
        stats[counter] += 1
    return n, max_dt
