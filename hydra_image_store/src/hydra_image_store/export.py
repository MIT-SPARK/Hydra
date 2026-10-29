"""Folder export: browsable per-node folders that look like the pre-store layout."""

from __future__ import annotations

import json
import os
import pathlib
import shutil
from collections import Counter

import numpy as np

from .codec import node_label
from .store import Frame, FrameKind, ImageStore, Observation


def _link(src: pathlib.Path, dst: pathlib.Path, copy: bool, stats: Counter) -> bool:
    if src is None or not src.exists():
        return False
    if os.path.lexists(dst):
        os.unlink(dst)
    if copy:
        shutil.copy2(src, dst)
        stats["copied"] += 1
        return True
    try:
        os.link(src, dst)
        stats["hardlinked"] += 1
    except OSError:
        os.symlink(os.path.abspath(src), dst)
        stats["symlinked"] += 1
    return True


class _Exporter:
    def __init__(self, store: ImageStore, out: pathlib.Path, pose_source, copy: bool):
        self.store = store
        self.out = out
        self.pose_source = pose_source
        self.copy = copy
        self.stats: Counter = Counter()
        self._calib_cache: dict[int, dict | None] = {}

    def calib_json(self, sensor_id: int):
        if sensor_id not in self._calib_cache:
            c = self.store.calib(sensor_id)
            self._calib_cache[sensor_id] = None if c is None else c.to_json()
        return self._calib_cache[sensor_id]

    def pose_meta(self, frame: Frame) -> dict:
        world_T_body, source = self.store.body_pose(frame, self.pose_source)
        world_T_sensor, _ = self.store.frame_pose(
            frame, self.pose_source, with_source=True
        )
        return {
            "world_T_body": world_T_body.reshape(16).tolist(),
            "world_T_sensor": world_T_sensor.reshape(16).tolist(),
            "pose_source": source,
            "anchor_node_id": frame.anchor_node_id,
        }

    def write_frame(
        self,
        folder: pathlib.Path,
        prefix: str,
        frame: Frame,
        extra: dict,
        obs: Observation | None = None,
    ) -> None:
        meta = {
            "timestamp_ns": frame.timestamp_ns,
            "frame_id": frame.frame_id,
            "frame_kind": FrameKind(frame.kind).name.lower(),
            "robot_id": frame.robot_id,
            **self.pose_meta(frame),
            "calib": self.calib_json(frame.sensor_id),
            **extra,
        }
        if frame.rgb_path is not None and _link(
            frame.rgb_path,
            folder / f"{prefix}_rgb{frame.rgb_path.suffix}",
            self.copy,
            self.stats,
        ):
            meta["rgb_file"] = f"{prefix}_rgb{frame.rgb_path.suffix}"
        if frame.depth_path is not None and _link(
            frame.depth_path,
            folder / f"{prefix}_depth{frame.depth_path.suffix}",
            self.copy,
            self.stats,
        ):
            meta["depth_file"] = f"{prefix}_depth{frame.depth_path.suffix}"
        if obs is not None:
            mask_file = f"{prefix}_mask.png"
            if obs.mask_path is not None and _link(
                obs.mask_path, folder / mask_file, self.copy, self.stats
            ):
                meta["mask_file"] = mask_file
            elif obs.mask_rle:
                written = False
                try:
                    from PIL import Image

                    m = self.store.mask(obs)
                    if m is not None:
                        Image.fromarray(m.astype(np.uint8) * 255).save(
                            folder / mask_file
                        )
                        written = True
                        self.stats["masks_decoded"] += 1
                except ImportError:
                    pass
                if written:
                    meta["mask_file"] = mask_file
                else:
                    meta["mask_rle"] = json.loads(obs.mask_rle)
        with open(folder / f"{prefix}_meta.json", "w") as f:
            json.dump(meta, f, indent=2)
        self.stats["frames"] += 1

    def export_objects(self, kept_only: bool) -> dict[str, int]:
        index = {}
        for node in self.store.object_nodes():
            observations = self.store.observations(
                node, kept_only=kept_only, order="time"
            )
            observations = [
                o
                for o in observations
                if o.kept or o.rgb_path is not None or o.mask_path is not None
            ]
            if not observations:
                continue
            name = node_label(node)
            folder = self.out / name
            folder.mkdir(parents=True, exist_ok=True)
            per_ts: Counter = Counter()
            for o in observations:
                i = per_ts[o.timestamp_ns]
                per_ts[o.timestamp_ns] += 1
                prefix = (
                    f"frame_{o.timestamp_ns}"
                    if i == 0
                    else f"frame_{o.timestamp_ns}_{i}"
                )
                extra = {
                    "node_id": node,
                    "source_node_id": o.node_id,
                    "obs_id": o.obs_id,
                    "track_id": o.track_id,
                    "score": o.score,
                    "kept": o.kept,
                }
                if o.bbox2d is not None:
                    extra["bbox_2d"] = dict(
                        zip(("min_x", "min_y", "max_x", "max_y"), o.bbox2d)
                    )
                if o.bbox3d is not None:
                    extra["bbox_min"] = o.bbox3d[0].tolist()
                    extra["bbox_max"] = o.bbox3d[1].tolist()
                self.write_frame(folder, prefix, o.frame, extra, obs=o)
                self.stats["observations"] += 1
            index[name] = node
            self.stats["object_folders"] += 1
        return index

    def export_agents(self) -> dict[str, int]:
        index = {}
        for node in self.store.agent_nodes(include_subkeyframe_anchors=True):
            frames = self.store.frames_for_agent(node, include_subkeyframes=True)
            keyframes = [f for f in frames if f.kind == FrameKind.KEYFRAME]
            name = (
                f"agent_{keyframes[0].timestamp_ns}"
                if keyframes
                else f"agent_{node_label(node)}"
            )
            folder = self.out / name
            folder.mkdir(parents=True, exist_ok=True)
            for f in frames:
                self.write_frame(
                    folder, f"frame_{f.timestamp_ns}", f, {"node_id": node}
                )
                self.stats[
                    "subkeyframes" if f.kind == FrameKind.SUBKEYFRAME else "keyframes"
                ] += 1
            index[name] = node
            self.stats["agent_folders"] += 1
        return index


def export_store(
    store: ImageStore,
    out,
    pose_source=None,
    *,
    copy: bool = False,
    kept_only: bool = False,
) -> dict:
    """Write per-node folders under ``out``.

    Objects: ``<label>/frame_<ts>_{rgb,depth,mask}.*`` + ``_meta.json`` (merged children
    under their surviving parent). Agents: ``agent_<ts>/`` with the keyframe and its
    sub-keyframes. Files are hardlinked (symlinked across filesystems) unless ``copy``.
    ``meta.json`` carries ``world_T_sensor`` corrected through ``pose_source`` (odometric
    when there is none) plus bbox, timestamp and calib. Returns counters.
    """
    out = pathlib.Path(out)
    out.mkdir(parents=True, exist_ok=True)
    ex = _Exporter(store, out, pose_source, copy)
    objects = ex.export_objects(kept_only)
    agents = ex.export_agents()
    with open(out / "index.json", "w") as f:
        json.dump(
            {
                "stores": [str(r) for r in store.roots],
                "objects": objects,
                "agents": agents,
                "sensors": [c.to_json() for c in store.sensors()],
            },
            f,
            indent=1,
        )
    return dict(ex.stats)
