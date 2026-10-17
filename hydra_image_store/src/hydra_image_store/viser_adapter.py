"""Adapter for ``spark_dsg.viser``'s ``image_provider(node_id) -> list[ImageRef]``.

spark_dsg does not depend on this package, so the provider returns plain dicts whose
keys match the fields of ``spark_dsg.viser.ImageRef``: ``rgb_path``, ``depth_path``,
``mask_path``, ``timestamp_ns`` and ``meta``. ``ImageRef(**d)`` works on each one.
"""

from __future__ import annotations

from typing import Callable

from .store import ImageStore


def make_image_provider(
    store: ImageStore, kept_only: bool = True
) -> Callable[[int], list[dict]]:
    """Return ``provider(node_id)``: object observations (best score first), else the
    keyframes and sub-keyframes anchored to an agent node. Entries without an rgb file
    are skipped."""

    def provider(node_id: int) -> list[dict]:
        out: list[dict] = []
        for o in store.observations(int(node_id), kept_only=kept_only, order="score"):
            if o.rgb_path is None:
                continue
            out.append(
                dict(
                    rgb_path=str(o.rgb_path),
                    depth_path=str(o.depth_path) if o.depth_path else None,
                    mask_path=str(o.mask_path) if o.mask_path else None,
                    timestamp_ns=o.timestamp_ns,
                    meta={
                        "kind": "observation",
                        "obs_id": o.obs_id,
                        "frame_id": o.frame_id,
                        "track_id": o.track_id,
                        "score": o.score,
                        "kept": o.kept,
                        "bbox2d": list(o.bbox2d) if o.bbox2d is not None else None,
                    },
                )
            )
        if out:
            return out
        for f in store.frames_for_agent(int(node_id), include_subkeyframes=True):
            if f.rgb_path is None:
                continue
            out.append(
                dict(
                    rgb_path=str(f.rgb_path),
                    depth_path=str(f.depth_path) if f.depth_path else None,
                    mask_path=None,
                    timestamp_ns=f.timestamp_ns,
                    meta={
                        "kind": "keyframe" if int(f.kind) == 0 else "subkeyframe",
                        "frame_id": f.frame_id,
                    },
                )
            )
        return out

    return provider
