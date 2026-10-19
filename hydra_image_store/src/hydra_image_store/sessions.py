"""Discover the Hydra sessions of a run directory.

Scene graph node ids and track ids restart with every Hydra process, so a store only
describes the session that wrote it. When Hydra first opens a non-empty store with
``image_store.on_existing: rotate`` (the default is ``append``) it moves it aside::

    <run>/image_store/                live store of the current session
    <run>/hydra/backend/dsg.json      its scene graph (written when Hydra shuts down)
    <run>/sessions/<k>/image_store/   store of the k-th earlier session
    <run>/sessions/<k>/hydra/...      copy of that session's saved scene graphs
    <run>/sessions/<k>/session.json   rotation time, source paths, counts

so ``<run>/sessions/<k>`` has the layout of a run directory. Writers outside Hydra
(:class:`~hydra_image_store.ImageStoreWriter`) never rotate.
"""

from __future__ import annotations

import dataclasses
import json
import os
import pathlib

SESSIONS_DIR = "sessions"
STORE_DIR = "image_store"
DSG_DIR = "hydra"
SESSION_INFO = "session.json"


@dataclasses.dataclass(frozen=True)
class Session:
    """One Hydra session of a run: a store and the scene graph its node ids refer to."""

    #: 1, 2, ... for rotated sessions (oldest first), ``None`` for the live session
    index: int | None
    #: Directory with the run layout (``<run>`` or ``<run>/sessions/<k>``)
    root: pathlib.Path
    #: Store directory
    store: pathlib.Path
    #: Contents of ``session.json`` (empty for the live session)
    info: dict = dataclasses.field(default_factory=dict, compare=False)

    @property
    def live(self) -> bool:
        return self.index is None

    @property
    def name(self) -> str:
        return "live" if self.live else str(self.index)

    def dsg_path(self, stage: str = "backend", with_mesh: bool = False) -> pathlib.Path | None:
        """Saved scene graph of the session (``stage`` is ``backend`` or ``frontend``)."""
        path = self.root / DSG_DIR / stage / ("dsg_with_mesh.json" if with_mesh else "dsg.json")
        return path if path.exists() else None

    def open(self, readonly: bool = True):
        """Open the session's store (see :meth:`ImageStore.open`)."""
        from .store import ImageStore

        return ImageStore.open(self.store, readonly=readonly)

    def load_dsg(self, stage: str = "backend"):
        """Load the session's scene graph with :func:`load_dsg` (``None`` if not saved)."""
        from .poses import load_dsg

        path = self.dsg_path(stage)
        return load_dsg(path) if path else None

    def to_json(self) -> dict:
        return {
            "session": self.name,
            "root": str(self.root),
            "store": str(self.store),
            "store_exists": (self.store / "store.sqlite").exists(),
            "backend_dsg": str(p) if (p := self.dsg_path("backend")) else None,
            "frontend_dsg": str(p) if (p := self.dsg_path("frontend")) else None,
            "info": self.info,
        }


def _run_dir(path: str | os.PathLike) -> pathlib.Path:
    path = pathlib.Path(path)
    if path.name == "store.sqlite":
        path = path.parent
    if (path / "store.sqlite").exists() and not (path / STORE_DIR).exists():
        return path.parent  # a store directory
    return path


def list_sessions(run_dir: str | os.PathLike, store_name: str = STORE_DIR,
                  include_live: bool = True) -> list[Session]:
    """Sessions of a run, oldest first: ``sessions/1``, ``sessions/2``, ..., then live.

    ``run_dir`` may also be the live store directory (or its ``store.sqlite``). The live
    session is listed when ``include_live`` and its store directory exists.
    """
    run = _run_dir(run_dir)
    result = []
    sessions = run / SESSIONS_DIR
    if sessions.is_dir():
        for entry in sessions.iterdir():
            if not entry.name.isdigit() or not (entry / store_name).is_dir():
                continue
            info = {}
            try:
                info = json.loads((entry / SESSION_INFO).read_text())
            except (OSError, ValueError):
                pass
            result.append(Session(int(entry.name), entry, entry / store_name, info))
    result.sort(key=lambda s: s.index)
    if include_live and (run / store_name).is_dir():
        result.append(Session(None, run, run / store_name))
    return result
