"""Pose sources for composing frame poses: spark_dsg graphs, DSG json files, callables."""

from __future__ import annotations

import json
import pathlib
from collections.abc import Callable, Mapping
from typing import Any

import numpy as np

from .codec import pose_matrix, quat_to_rot

PoseLookup = Callable[[int], "np.ndarray | None"]


def _quat_from_json(q: Any) -> np.ndarray | None:
    if isinstance(q, dict) and all(k in q for k in "wxyz"):
        return quat_to_rot(q["w"], q["x"], q["y"], q["z"])
    return None


class JsonDsg:
    """Minimal parser for spark_dsg json files (no spark_dsg dependency).

    Only what this package needs: node attributes by id, agent poses, object/agent
    ``image_folder`` strings left behind by older Hydra versions.
    """

    def __init__(self, data: dict):
        self.nodes: dict[int, dict] = {}
        for node in data.get("nodes", []):
            self.nodes[int(node["id"])] = node
        self.merges: dict[int, int] = {}
        for key in ("merges", "node_merges"):
            raw = data.get(key) or data.get("metadata", {}).get(key)
            if isinstance(raw, dict):
                self.merges.update({int(k): int(v) for k, v in raw.items()})
            elif isinstance(raw, list):
                for item in raw:
                    if isinstance(item, (list, tuple)) and len(item) == 2:
                        self.merges[int(item[0])] = int(item[1])

    @classmethod
    def load(cls, path: str | pathlib.Path) -> "JsonDsg":
        with open(path) as f:
            return cls(json.load(f))

    def has_node(self, node_id: int) -> bool:
        return int(node_id) in self.nodes

    def attributes(self, node_id: int) -> dict | None:
        node = self.nodes.get(int(node_id))
        return None if node is None else node.get("attributes", {})

    def pose(self, node_id: int) -> np.ndarray | None:
        attrs = self.attributes(node_id)
        if not attrs or "position" not in attrs:
            return None
        rot = _quat_from_json(attrs.get("world_R_body"))
        if rot is None:
            return None  # only agent-like nodes carry a body orientation
        return pose_matrix(attrs["position"], rot)

    def __call__(self, node_id: int) -> np.ndarray | None:
        return self.pose(node_id)


def _spark_lookup(graph) -> PoseLookup:
    def lookup(node_id: int):
        node_id = int(node_id)
        if not graph.has_node(node_id):
            return None
        attrs = graph.get_node(node_id).attributes
        q = getattr(attrs, "world_R_body", None)
        if q is None:
            return None
        return pose_matrix(attrs.position, quat_to_rot(q.w, q.x, q.y, q.z))

    return lookup


def as_pose_lookup(source) -> PoseLookup | None:
    """Normalize a pose source.

    Accepts ``None``, a spark_dsg ``DynamicSceneGraph``, a :class:`JsonDsg`, a mapping
    ``node_id -> 4x4`` or a callable ``node_id -> 4x4 | None``. A missing node must map
    to ``None`` (callables may also raise ``KeyError``).
    """
    if source is None:
        return None
    if isinstance(source, JsonDsg):
        return source.pose
    if hasattr(source, "has_node") and hasattr(source, "get_node"):
        return _spark_lookup(source)
    if isinstance(source, Mapping):
        return lambda nid: None if source.get(int(nid)) is None else np.asarray(source[int(nid)], float)
    if callable(source):
        def lookup(nid):
            try:
                T = source(int(nid))
            except KeyError:
                return None
            return None if T is None else np.asarray(T, dtype=float).reshape(4, 4)

        return lookup
    raise TypeError(f"unsupported pose source {type(source)!r}")


def load_dsg(path: str | pathlib.Path):
    """Load a DSG file: ``.json`` via :class:`JsonDsg`, anything else via spark_dsg."""
    path = pathlib.Path(path)
    if path.suffix == ".json":
        return JsonDsg.load(path)
    import spark_dsg  # optional dependency

    return spark_dsg.DynamicSceneGraph.load(str(path))
