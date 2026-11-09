"""Reader and small writer for the Hydra image store (schema_version 1)."""

from __future__ import annotations

import contextlib
import dataclasses
import enum
import json
import logging
import os
import pathlib
import shutil
import sqlite3
import urllib.parse
from collections.abc import Iterable, Iterator, Mapping, Sequence
from importlib import resources

import imageio.v3 as iio
import numpy as np

from .codec import (
    SEQ_MASK,
    blob_to_transform,
    make_id,
    rle_decode,
    rle_encode,
    transform_to_blob,
)
from .poses import as_pose_lookup

logger = logging.getLogger(__name__)

SCHEMA_VERSION = 1
DB_NAME = "store.sqlite"


class SchemaVersionError(RuntimeError):
    pass


class FrameKindError(ValueError):
    """A frame row holds a kind this schema version does not define."""


class FrameKind(enum.IntEnum):
    #: a Hydra keyframe archive, anchored to its nearest agent node
    KEYFRAME = 0
    #: a frame only an object observation brought in (no keyframe archive)
    OBSERVATION = 2


def _frame_kind(value, where) -> FrameKind:
    try:
        return FrameKind(value)
    except ValueError:
        kinds = ", ".join(f"{k.value} = {k.name.lower()}" for k in FrameKind)
        raise FrameKindError(
            f"{where}: unknown frame kind {value!r} (valid: {kinds})"
        ) from None


def schema_sql() -> str:
    """The schema shipped with this package."""
    return resources.files(__package__).joinpath("schema.sql").read_text()


@dataclasses.dataclass(frozen=True)
class Calib:
    sensor_id: int
    robot_id: int
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float
    body_T_sensor: np.ndarray

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def to_json(self) -> dict:
        return {
            "sensor_id": self.sensor_id,
            "name": self.name,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "width": self.width,
            "height": self.height,
            "depth_scale": self.depth_scale,
            "body_T_sensor": self.body_T_sensor.reshape(16).tolist(),
        }


@dataclasses.dataclass(frozen=True)
class Frame:
    frame_id: int
    robot_id: int
    sensor_id: int
    timestamp_ns: int
    kind: FrameKind
    world_T_body_odom: np.ndarray
    anchor_node_id: int | None
    anchor_T_body: np.ndarray | None
    rgb_path: pathlib.Path | None  # absolute
    depth_path: pathlib.Path | None  # absolute
    store_dir: pathlib.Path


@dataclasses.dataclass(frozen=True)
class Observation:
    obs_id: int
    robot_id: int
    track_id: int
    node_id: int | None  # node the track is bound to (may be a merged child)
    frame_id: int
    timestamp_ns: int
    bbox2d: tuple[int, int, int, int] | None  # min_x, min_y, max_x, max_y
    bbox3d: tuple[np.ndarray, np.ndarray] | None  # (min, max)
    mask_rle: str | None
    mask_path: pathlib.Path | None  # absolute
    score: float
    kept: bool
    frame: Frame

    @property
    def rgb_path(self) -> pathlib.Path | None:
        return self.frame.rgb_path

    @property
    def depth_path(self) -> pathlib.Path | None:
        return self.frame.depth_path

    @property
    def sensor_id(self) -> int:
        return self.frame.sensor_id


_FRAME_COLS = (
    "frame_id, robot_id, sensor_id, timestamp_ns, kind, world_T_body_odom, "
    "anchor_node_id, anchor_T_body, rgb_path, depth_path"
)
_OBS_COLS = (
    "o.obs_id, o.robot_id, o.track_id, o.frame_id, o.timestamp_ns, "
    "o.bbox2d_min_x, o.bbox2d_min_y, o.bbox2d_max_x, o.bbox2d_max_y, "
    "o.bbox3d_min_x, o.bbox3d_min_y, o.bbox3d_min_z, "
    "o.bbox3d_max_x, o.bbox3d_max_y, o.bbox3d_max_z, "
    "o.mask_rle, o.mask_path, o.score, o.kept"
)


def _resolve_db(path: str | os.PathLike) -> pathlib.Path:
    p = pathlib.Path(path).expanduser()
    if p.is_dir() or p.suffix != ".sqlite":
        p = p / DB_NAME
    return p.absolute()


def _connect(
    db: pathlib.Path, readonly: bool, create: bool = False
) -> sqlite3.Connection:
    if not db.exists() and (readonly or not create):
        raise FileNotFoundError(
            f"no image store at {db}"
            + ("" if readonly else " (pass create=True to start a new one)")
        )
    if readonly:
        uri = "file:" + urllib.parse.quote(str(db)) + "?mode=ro"
        try:
            conn = sqlite3.connect(
                uri, uri=True, check_same_thread=False, isolation_level=None
            )
            conn.execute("SELECT 1 FROM meta LIMIT 1").fetchall()
        except sqlite3.OperationalError:
            # WAL readers need to create the -shm file; on a read-only medium fall back
            # to immutable mode (safe only because nobody can be writing there).
            uri = "file:" + urllib.parse.quote(str(db)) + "?mode=ro&immutable=1"
            conn = sqlite3.connect(
                uri, uri=True, check_same_thread=False, isolation_level=None
            )
    else:
        db.parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(str(db), check_same_thread=False, isolation_level=None)
    # another writer (e.g. the indexer) may hold the lock
    conn.execute("PRAGMA busy_timeout = 5000")
    if not readonly:
        conn.executescript(schema_sql())
        conn.execute("PRAGMA synchronous = NORMAL")
    row = conn.execute("SELECT value FROM meta WHERE key = 'schema_version'").fetchone()
    version = int(row[0]) if row else None
    if version != SCHEMA_VERSION:
        conn.close()
        raise SchemaVersionError(
            f"{db}: schema_version {version}, this reader supports {SCHEMA_VERSION}"
        )
    return conn


class _Db:
    def __init__(self, path, readonly: bool, create: bool = False):
        self.db_path = _resolve_db(path)
        self.root = self.db_path.parent
        self.conn = _connect(self.db_path, readonly, create)

    def abs(self, rel: str | None) -> pathlib.Path | None:
        return None if rel is None else self.root / rel

    def frame_from_row(self, r) -> Frame:
        return Frame(
            frame_id=r[0],
            robot_id=r[1],
            sensor_id=r[2],
            timestamp_ns=r[3],
            kind=_frame_kind(r[4], self.db_path),
            world_T_body_odom=blob_to_transform(r[5]),
            anchor_node_id=r[6],
            anchor_T_body=blob_to_transform(r[7]),
            rgb_path=self.abs(r[8]),
            depth_path=self.abs(r[9]),
            store_dir=self.root,
        )


def _store_path(path: str | os.PathLike, index: bool) -> str | os.PathLike:
    """Store directory for ``path``: maps a run directory to its ``image_store/`` and,
    with ``index``, builds or refreshes the index of a run with raw image logs (see
    :func:`hydra_python.image_store.indexer.ensure_indexed`)."""
    from .indexer import STORE_DIR, ensure_indexed, has_raw_inputs

    if index:
        try:
            store = ensure_indexed(path)
        except (
            OSError,
            sqlite3.Error,
        ) as exc:  # e.g. read-only medium: open what is there
            logger.warning("could not index %s: %s", path, exc)
            store = None
        if store is not None:
            return store
    p = pathlib.Path(path).expanduser()
    if p.is_dir() and not (p / DB_NAME).exists():
        if (p / STORE_DIR / DB_NAME).exists() or has_raw_inputs(p):
            return p / STORE_DIR
    return path


def _in_clause(values: Sequence) -> str:
    return "(" + ",".join("?" * len(values)) + ")"


def _chunks(values: Sequence, n: int = 900) -> Iterator[Sequence]:
    for i in range(0, len(values), n):
        yield values[i : i + n]


class ImageStore:
    """Read access to one or several robot image stores.

    Each path is a store directory (containing ``store.sqlite``) or the sqlite file
    itself. Read-only connections use ``mode=ro`` and see the latest committed state of
    a store a robot is still writing (WAL).
    """

    def __init__(self, dbs: list[_Db]):
        self._dbs = dbs
        self._alias_cache: tuple[tuple, dict[int, int], dict[int, set[int]]] | None = (
            None
        )

    # --- construction -------------------------------------------------------------------

    @classmethod
    def open(
        cls,
        *paths: str | os.PathLike,
        readonly: bool = True,
        create: bool = False,
        index: bool = True,
    ) -> "ImageStore":
        """Open stores. A missing store raises ``FileNotFoundError``; with
        ``readonly=False, create=True`` an empty one is created instead.

        A path may also be a run directory (its ``image_store/`` is opened). When the
        run holds raw Hydra/Khronos image logs, the store is built first with
        :func:`hydra_python.image_store.indexer.index` if it was never indexed or its inputs
        changed since (see :func:`hydra_python.image_store.indexer.ensure_indexed`). That
        writes the store even with ``readonly=True``; ``index=False`` never writes."""
        if not paths:
            raise ValueError("ImageStore.open needs at least one store path")
        if create and readonly:
            raise ValueError("create=True needs readonly=False")
        return cls([_Db(_store_path(p, index), readonly, create) for p in paths])

    @staticmethod
    def open_writer(
        path: str | os.PathLike, robot_id: int = 0, *, create: bool = True
    ) -> "ImageStoreWriter":
        """Writer into the store at ``path`` (a store directory, its ``store.sqlite`` or a
        run directory, whose ``image_store/`` is used). Never runs the indexer.

        Creates an empty store when there is none (``create=False`` raises instead).
        """
        return ImageStoreWriter(_store_path(path, False), robot_id, create=create)

    def close(self) -> None:
        for db in self._dbs:
            db.conn.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def roots(self) -> list[pathlib.Path]:
        return [db.root for db in self._dbs]

    def _query(self, sql: str, params: Sequence = ()) -> Iterator[tuple[_Db, tuple]]:
        for db in self._dbs:
            for row in db.conn.execute(sql, params):
                yield db, row

    # --- merges -------------------------------------------------------------------------

    def _aliases(self) -> tuple[dict[int, int], dict[int, set[int]]]:
        version = tuple(
            db.conn.execute("PRAGMA data_version").fetchone()[0] for db in self._dbs
        )
        if self._alias_cache is None or self._alias_cache[0] != version:
            parents = {
                c: p
                for _, (c, p) in self._query(
                    "SELECT child_id, parent_id FROM node_aliases"
                )
            }
            members: dict[int, set[int]] = {}
            canon = {c: self._follow(parents, c) for c in parents}
            for child, root in canon.items():
                members.setdefault(root, {root}).add(child)
            self._alias_cache = (version, canon, members)
        return self._alias_cache[1], self._alias_cache[2]

    @staticmethod
    def _follow(parents: Mapping[int, int], node: int) -> int:
        seen = {node}
        while node in parents:
            nxt = parents[node]
            if nxt in seen:
                logger.warning("node_aliases cycle through %d; stopping", nxt)
                break
            seen.add(nxt)
            node = nxt
        return node

    def canonical(self, node_id: int) -> int:
        """The node that ``node_id`` was (transitively) merged into, or itself."""
        canon, _ = self._aliases()
        return canon.get(int(node_id), int(node_id))

    def members(self, node_id: int) -> set[int]:
        """All node ids merged into ``canonical(node_id)``, including the canonical id."""
        root = self.canonical(node_id)
        _, members = self._aliases()
        return set(members.get(root, {root}))

    # --- frames ---------------------------------------------------------------------------

    def frame(self, frame_id: int) -> Frame | None:
        for db, row in self._query(
            f"SELECT {_FRAME_COLS} FROM frames WHERE frame_id = ?", (int(frame_id),)
        ):
            return db.frame_from_row(row)
        return None

    def frames(
        self,
        robot_id: int | None = None,
        t0: int | None = None,
        t1: int | None = None,
        kind: FrameKind | int | Iterable[int] | None = None,
    ) -> list[Frame]:
        """Frames ordered by time, optionally filtered (``t0 <= ts <= t1``)."""
        where, params = [], []
        if robot_id is not None:
            where.append("robot_id = ?")
            params.append(int(robot_id))
        if t0 is not None:
            where.append("timestamp_ns >= ?")
            params.append(int(t0))
        if t1 is not None:
            where.append("timestamp_ns <= ?")
            params.append(int(t1))
        if kind is not None:
            kinds = [int(kind)] if isinstance(kind, int) else [int(k) for k in kind]
            where.append(f"kind IN {_in_clause(kinds)}")
            params.extend(kinds)
        sql = f"SELECT {_FRAME_COLS} FROM frames"
        if where:
            sql += " WHERE " + " AND ".join(where)
        out = [db.frame_from_row(r) for db, r in self._query(sql, params)]
        out.sort(key=lambda f: (f.timestamp_ns, f.frame_id))
        return out

    def frames_for_agent(
        self, node_id: int, timestamp_ns: int | None = None
    ) -> list[Frame]:
        """Keyframes anchored to an agent node, in time order, or nearest to
        ``timestamp_ns`` (e.g. the node's own stamp) first."""
        sql = f"SELECT {_FRAME_COLS} FROM frames WHERE anchor_node_id = ? AND kind = ?"
        out = [
            db.frame_from_row(r)
            for db, r in self._query(sql, (int(node_id), int(FrameKind.KEYFRAME)))
        ]
        if timestamp_ns is None:
            out.sort(key=lambda f: (f.timestamp_ns, f.frame_id))
        else:
            t = int(timestamp_ns)
            out.sort(
                key=lambda f: (abs(f.timestamp_ns - t), f.timestamp_ns, f.frame_id)
            )
        return out

    def agent_frame(
        self, node_id: int, timestamp_ns: int, max_dt_ns: int = 50_000_000
    ) -> Frame | None:
        """The keyframe taken at an agent node: of the keyframes anchored to it, the one
        nearest to the node's stamp ``timestamp_ns``, if within ``max_dt_ns``."""
        frames = self.frames_for_agent(node_id, timestamp_ns)
        if frames and abs(frames[0].timestamp_ns - int(timestamp_ns)) <= max_dt_ns:
            return frames[0]
        return None

    # --- descriptors ------------------------------------------------------------------------

    def _descriptor_rows(self, sql: str, params: Sequence = ()):
        for db in self._dbs:
            for row in db.conn.execute(sql, params).fetchall():
                yield db, row

    def descriptor(self, frame_id: int) -> np.ndarray | None:
        """The descriptor stored for a frame (e.g. a keyframe archive's feature vector),
        or ``None``."""
        for _, (dim, dtype, data) in self._descriptor_rows(
            "SELECT dim, dtype, data FROM frame_descriptors WHERE frame_id = ?",
            (int(frame_id),),
        ):
            return _decode_descriptor(dim, dtype, data)
        return None

    def descriptors(
        self,
        robot_id: int | None = None,
        kind: FrameKind | int | Iterable[int] | None = None,
    ) -> Iterator[tuple[Frame, np.ndarray]]:
        """``(frame, descriptor)`` for every frame with a descriptor, in time order,
        optionally filtered by robot and frame kind."""
        where, params = [], []
        if robot_id is not None:
            where.append("f.robot_id = ?")
            params.append(int(robot_id))
        if kind is not None:
            kinds = [int(kind)] if isinstance(kind, int) else [int(k) for k in kind]
            where.append(f"f.kind IN {_in_clause(kinds)}")
            params.extend(kinds)
        cols = ", ".join("f." + c.strip() for c in _FRAME_COLS.split(","))
        sql = (
            f"SELECT {cols}, d.dim, d.dtype, d.data FROM frame_descriptors d "
            "JOIN frames f ON f.frame_id = d.frame_id"
        )
        if where:
            sql += " WHERE " + " AND ".join(where)
        out = [
            (db.frame_from_row(r[:10]), _decode_descriptor(*r[10:]))
            for db, r in self._descriptor_rows(sql, params)
        ]
        out.sort(key=lambda x: (x[0].timestamp_ns, x[0].frame_id))
        yield from out

    def agent_nodes(self) -> list[int]:
        """Ids of the agent nodes keyframes are anchored to."""
        rows = self._query(
            "SELECT DISTINCT anchor_node_id FROM frames "
            "WHERE kind = ? AND anchor_node_id IS NOT NULL",
            (int(FrameKind.KEYFRAME),),
        )
        return sorted({r[0] for _, r in rows})

    # --- sensors / poses --------------------------------------------------------------------

    def calib(self, sensor_id: int) -> Calib | None:
        sql = (
            "SELECT sensor_id, robot_id, name, width, height, fx, fy, cx, cy, depth_scale, "
            "body_T_sensor FROM sensors WHERE sensor_id = ?"
        )
        for _, r in self._query(sql, (int(sensor_id),)):
            return Calib(*r[:10], body_T_sensor=blob_to_transform(r[10]))
        return None

    def sensors(self) -> list[Calib]:
        rows = self._query("SELECT sensor_id FROM sensors")
        return [self.calib(r[0]) for _, r in rows]

    def frame_pose(self, frame: Frame, pose_lookup=None, *, with_source: bool = False):
        """``world_T_sensor`` of a frame.

        ``pose_lookup`` is a spark_dsg graph, a :class:`~hydra_python.image_store.JsonDsg`, a
        mapping or a callable ``node_id -> 4x4``. The pose is
        ``world_T_anchor @ anchor_T_body @ body_T_sensor``; when the anchor is missing or
        cannot be resolved (also after following merges) it falls back to
        ``world_T_body_odom @ body_T_sensor``. With ``with_source=True`` returns
        ``(T, "anchor" | "odom")``.
        """
        calib = self.calib(frame.sensor_id)
        body_T_sensor = np.eye(4) if calib is None else calib.body_T_sensor
        world_T_body, source = self.body_pose(frame, pose_lookup)
        T = world_T_body @ body_T_sensor
        return (T, source) if with_source else T

    def body_pose(self, frame: Frame, pose_lookup=None) -> tuple[np.ndarray, str]:
        lookup = as_pose_lookup(pose_lookup)
        if lookup is not None and frame.anchor_node_id is not None:
            anchor_T_body = (
                np.eye(4) if frame.anchor_T_body is None else frame.anchor_T_body
            )
            for nid in dict.fromkeys(
                (frame.anchor_node_id, self.canonical(frame.anchor_node_id))
            ):
                world_T_anchor = lookup(nid)
                if world_T_anchor is not None:
                    return np.asarray(world_T_anchor, float) @ anchor_T_body, "anchor"
        return frame.world_T_body_odom, "odom"

    # --- observations ---------------------------------------------------------------------

    def tracks(self, node_id: int) -> list[tuple[int, int, int]]:
        """``(robot_id, track_id, bound_node_id)`` for every track of the merged set."""
        ids = sorted(self.members(node_id))
        out = []
        for chunk in _chunks(ids):
            sql = f"SELECT robot_id, track_id, node_id FROM track_bindings WHERE node_id IN {_in_clause(chunk)}"
            out.extend(r for _, r in self._query(sql, chunk))
        return sorted(set(out))

    def unbound_tracks(self) -> list[tuple[int, int, str]]:
        """``(robot_id, track_id, reason)`` of tracks with observations but no node.

        The indexer records why (``extraction_failed``: Khronos logged the track but
        extraction failed, so no node exists; ``no_binding``: no frontend binding; ``node_deleted``: its node was deleted by the frontend);
        other unbound tracks report ``unbound``."""
        out = set()
        for db in self._dbs:
            reasons: dict[tuple[int, int], str] = {}
            for key, value in db.conn.execute(
                "SELECT key, value FROM meta WHERE key LIKE 'indexer.r%.unbound_tracks'"
            ):
                robot = int(key.split(".")[1][1:])
                for t, why in json.loads(value).items():
                    reasons[(robot, int(t))] = why
            for robot, track in db.conn.execute(
                "SELECT DISTINCT o.robot_id, o.track_id FROM observations o "
                "LEFT JOIN track_bindings b ON b.robot_id = o.robot_id AND b.track_id = o.track_id "
                "WHERE b.node_id IS NULL"
            ):
                out.add((robot, track, reasons.get((robot, track), "unbound")))
        return sorted(out)

    def object_nodes(self) -> list[int]:
        """Canonical ids of every node some track is bound to."""
        rows = self._query("SELECT DISTINCT node_id FROM track_bindings")
        return sorted({self.canonical(r[0]) for _, r in rows})

    def observations(
        self,
        node_id: int,
        kept_only: bool = False,
        limit: int | None = None,
        order: str = "score",
    ) -> list[Observation]:
        """Observations of a node and everything merged into it.

        ``order="score"``: best first (ties: earliest first); ``order="time"``: oldest first.
        """
        if order not in ("score", "time"):
            raise ValueError("order must be 'score' or 'time'")
        ids = sorted(self.members(node_id))
        out: list[Observation] = []
        for db in self._dbs:
            for chunk in _chunks(ids):
                sql = (
                    f"SELECT {_OBS_COLS}, b.node_id, "
                    + ", ".join("f." + c.strip() for c in _FRAME_COLS.split(","))
                    + " FROM track_bindings b"
                    " JOIN observations o ON o.robot_id = b.robot_id AND o.track_id = b.track_id"
                    " JOIN frames f ON f.frame_id = o.frame_id"
                    f" WHERE b.node_id IN {_in_clause(chunk)}"
                )
                if kept_only:
                    sql += " AND o.kept != 0"
                for r in db.conn.execute(sql, chunk):
                    out.append(self._obs_from_row(db, r))
        if order == "score":
            out.sort(key=lambda o: (-o.score, o.timestamp_ns, o.obs_id))
        else:
            out.sort(key=lambda o: (o.timestamp_ns, o.obs_id))
        return out if limit is None else out[:limit]

    def track_observations(self, robot_id: int, track_id: int) -> list[Observation]:
        """Observations of one track, bound or not (time order)."""
        out = []
        for db in self._dbs:
            sql = (
                f"SELECT {_OBS_COLS}, b.node_id, "
                + ", ".join("f." + c.strip() for c in _FRAME_COLS.split(","))
                + " FROM observations o JOIN frames f ON f.frame_id = o.frame_id"
                " LEFT JOIN track_bindings b ON b.robot_id = o.robot_id AND b.track_id = o.track_id"
                " WHERE o.robot_id = ? AND o.track_id = ?"
            )
            out.extend(
                self._obs_from_row(db, r)
                for r in db.conn.execute(sql, (int(robot_id), int(track_id)))
            )
        out.sort(key=lambda o: (o.timestamp_ns, o.obs_id))
        return out

    @staticmethod
    def _obs_from_row(db: _Db, r) -> Observation:
        bbox2d = None if r[5] is None else (r[5], r[6], r[7], r[8])
        bbox3d = None
        if r[9] is not None:
            bbox3d = (np.array(r[9:12], dtype=float), np.array(r[12:15], dtype=float))
        return Observation(
            obs_id=r[0],
            robot_id=r[1],
            track_id=r[2],
            node_id=r[19],
            frame_id=r[3],
            timestamp_ns=r[4],
            bbox2d=bbox2d,
            bbox3d=bbox3d,
            mask_rle=r[15],
            mask_path=db.abs(r[16]),
            score=float(r[17]),
            kept=bool(r[18]),
            frame=db.frame_from_row(r[20:]),
        )

    def mask(self, obs: Observation) -> np.ndarray | None:
        """Boolean HxW mask: decoded RLE (needs the sensor size) or the mask PNG."""
        if obs.mask_rle:
            calib = self.calib(obs.frame.sensor_id)
            shape = None if calib is None else (calib.height, calib.width)
            if shape is None or shape[0] * shape[1] == 0:
                shape = _image_shape(obs.mask_path or obs.frame.rgb_path)
            if shape is not None:
                return rle_decode(obs.mask_rle, shape)
        if obs.mask_path is not None and obs.mask_path.exists():
            img = _read_image(obs.mask_path)
            return (img[..., 0] if img.ndim == 3 else img) > 0
        return None

    # --- stats ---------------------------------------------------------------------------------

    def info(self) -> list[dict]:
        out = []
        for db in self._dbs:
            c = db.conn
            one = lambda sql: c.execute(sql).fetchone()[0]  # noqa: E731
            kinds = dict(
                c.execute("SELECT kind, COUNT(*) FROM frames GROUP BY kind").fetchall()
            )
            out.append(
                {
                    "path": str(db.root),
                    "meta": dict(c.execute("SELECT key, value FROM meta").fetchall()),
                    "robots": [
                        r[0]
                        for r in c.execute(
                            "SELECT DISTINCT robot_id FROM frames UNION SELECT DISTINCT robot_id FROM sensors"
                        )
                    ],
                    "sensors": one("SELECT COUNT(*) FROM sensors"),
                    "frames": {
                        _frame_kind(k, db.db_path).name.lower(): v
                        for k, v in sorted(kinds.items())
                    },
                    "frames_with_rgb": one(
                        "SELECT COUNT(*) FROM frames WHERE rgb_path IS NOT NULL"
                    ),
                    "frames_unanchored": one(
                        "SELECT COUNT(*) FROM frames WHERE anchor_node_id IS NULL"
                    ),
                    "observations": one("SELECT COUNT(*) FROM observations"),
                    "observations_kept": one(
                        "SELECT COUNT(*) FROM observations WHERE kept != 0"
                    ),
                    "tracks": one(
                        "SELECT COUNT(*) FROM (SELECT DISTINCT robot_id, track_id FROM observations)"
                    ),
                    "track_bindings": one("SELECT COUNT(*) FROM track_bindings"),
                    "bound_nodes": one(
                        "SELECT COUNT(DISTINCT node_id) FROM track_bindings"
                    ),
                    "node_aliases": one("SELECT COUNT(*) FROM node_aliases"),
                }
            )
        return out


def _image_shape(path) -> tuple[int, int] | None:
    """(rows, cols) of an image file from its header; None if it does not exist."""
    if path is None or not pathlib.Path(path).exists():
        return None
    return tuple(iio.improps(path).shape[:2])


def _read_image(path) -> np.ndarray:
    return iio.imread(path)


def _decode_descriptor(dim: int, dtype: str, data: bytes) -> np.ndarray:
    arr = np.frombuffer(data, dtype=np.dtype(dtype))
    if arr.size != int(dim):
        raise ValueError(f"descriptor has {arr.size} values, expected {dim}")
    return arr.astype(arr.dtype.newbyteorder("="), copy=True)


def frame_stem(timestamp_ns: int, sensor_id: int) -> str:
    """File stem of a frame's images, as the indexer names them: ``<ts>`` for the
    robot's first sensor (sequence 0), else ``<ts>_s<sensor seq>``."""
    seq = int(sensor_id) & SEQ_MASK
    return f"{int(timestamp_ns)}_s{seq}" if seq else str(int(timestamp_ns))


def _place_file(src: pathlib.Path, dst: pathlib.Path, link: str) -> None:
    """Materialize ``src`` at ``dst`` by hardlink (falling back to a copy) or copy."""
    if link not in ("hardlink", "copy"):
        raise ValueError(f"unknown link mode {link!r}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    if link == "hardlink":
        try:
            os.link(src, dst)
            return
        except OSError:
            pass
    shutil.copy2(src, dst)


class ImageStoreWriter:
    """Small writer, used by the indexer and by tools that add their own rows (see
    :data:`~hydra_python.image_store.codec.EXTERNAL_TRACK_BASE` for their track ids).

    Ids are ``(robot_id << 48) | seq``; the next sequence number is taken inside the
    insert transaction as ``max(existing seq for this robot) + 1``, so reopening a store
    resumes numbering and concurrent writers (e.g. the indexer) never collide.
    """

    def __init__(
        self, path: str | os.PathLike, robot_id: int = 0, *, create: bool = True
    ):
        self._db = _Db(path, readonly=False, create=create)
        self.robot_id = int(robot_id)
        self.root = self._db.root
        self._batch_depth = 0

    @property
    def conn(self) -> sqlite3.Connection:
        return self._db.conn

    def close(self) -> None:
        if self._batch_depth:
            self.conn.execute("COMMIT")
            self._batch_depth = 0
        self.conn.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def reader(self) -> ImageStore:
        """A reader sharing this writer's connection (sees uncommitted batch rows)."""
        return ImageStore([self._db])

    @contextlib.contextmanager
    def batch(self):
        """Group many writes in one transaction."""
        if self._batch_depth == 0:
            self.conn.execute("BEGIN IMMEDIATE")
        self._batch_depth += 1
        try:
            yield self
        except BaseException:
            self._batch_depth -= 1
            if self._batch_depth == 0:
                self.conn.execute("ROLLBACK")
            raise
        self._batch_depth -= 1
        if self._batch_depth == 0:
            self.conn.execute("COMMIT")

    def _next_id(self, table: str, column: str) -> int:
        lo = make_id(self.robot_id, 0)
        hi = lo | SEQ_MASK
        row = self.conn.execute(
            f"SELECT MAX({column}) FROM {table} WHERE {column} BETWEEN ? AND ?",
            (lo, hi),
        ).fetchone()
        return lo if row[0] is None else row[0] + 1

    def set_meta(self, key: str, value) -> None:
        with self.batch():
            self.conn.execute(
                "INSERT OR REPLACE INTO meta (key, value) VALUES (?, ?)",
                (key, str(value)),
            )

    # --- rows -------------------------------------------------------------------------------

    def add_sensor(
        self,
        name: str,
        width: int,
        height: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        depth_scale: float = 0.001,
        body_T_sensor=None,
    ) -> int:
        """Insert or update the sensor ``(robot_id, name)``; returns its sensor_id."""
        blob = transform_to_blob(np.eye(4) if body_T_sensor is None else body_T_sensor)
        with self.batch():
            row = self.conn.execute(
                "SELECT sensor_id FROM sensors WHERE robot_id = ? AND name = ?",
                (self.robot_id, name),
            ).fetchone()
            vals = (
                int(width),
                int(height),
                float(fx),
                float(fy),
                float(cx),
                float(cy),
                float(depth_scale),
                blob,
            )
            if row:
                self.conn.execute(
                    "UPDATE sensors SET width=?, height=?, fx=?, fy=?, cx=?, cy=?, depth_scale=?, "
                    "body_T_sensor=? WHERE sensor_id=?",
                    (*vals, row[0]),
                )
                return row[0]
            sid = self._next_id("sensors", "sensor_id")
            self.conn.execute(
                "INSERT INTO sensors (sensor_id, robot_id, name, width, height, fx, fy, cx, cy, "
                "depth_scale, body_T_sensor) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (sid, self.robot_id, name, *vals),
            )
            return sid

    def _import_file(self, src, rel: str, link: str) -> str | None:
        if src is None:
            return None
        src = pathlib.Path(src)
        try:
            return str(src.resolve().relative_to(self.root.resolve()))
        except ValueError:
            pass
        if not src.exists():
            raise FileNotFoundError(src)
        dst = self.root / rel
        if not os.path.lexists(dst):
            _place_file(src, dst, link)
        return rel

    def add_frame_ref(
        self,
        sensor_id: int,
        timestamp_ns: int,
        *,
        kind: FrameKind | int = FrameKind.OBSERVATION,
        world_T_body_odom=None,
        rgb=None,
        depth=None,
        anchor_node_id: int | None = None,
        anchor_T_body=None,
        link: str = "hardlink",
    ) -> int:
        """Add a frame that references existing image files; returns its frame_id.

        ``rgb`` / ``depth`` are paths to existing files. Files already inside the store
        are referenced in place; others are hardlinked (a copy when that fails, e.g.
        across filesystems) or copied (``link="copy"``) to
        ``frames/r<robot>/<stem>_{rgb,depth}<ext>``,
        where ``<stem>`` is ``<ts>`` for the robot's first sensor and ``<ts>_s<seq>`` for
        the others (the indexer's naming, so two sensors at one stamp never share a
        file).
        A frame with the same (sensor, timestamp) is reused: missing image paths and a
        missing anchor are filled in, nothing else changes.
        """
        ts = int(timestamp_ns)
        base = f"frames/r{self.robot_id}/{frame_stem(ts, sensor_id)}"
        with self.batch():
            row = self.conn.execute(
                "SELECT frame_id, rgb_path, depth_path, anchor_node_id FROM frames "
                "WHERE sensor_id = ? AND timestamp_ns = ?",
                (int(sensor_id), ts),
            ).fetchone()
            rgb_rel = (
                None
                if rgb is None or (row and row[1])
                else self._import_file(
                    rgb, f"{base}_rgb{pathlib.Path(rgb).suffix}", link
                )
            )
            depth_rel = (
                None
                if depth is None or (row and row[2])
                else self._import_file(
                    depth, f"{base}_depth{pathlib.Path(depth).suffix}", link
                )
            )
            anchor_blob = None
            if anchor_node_id is not None:
                anchor_blob = transform_to_blob(
                    np.eye(4) if anchor_T_body is None else anchor_T_body
                )
            if row:
                fid = row[0]
                self.conn.execute(
                    "UPDATE frames SET rgb_path = COALESCE(rgb_path, ?), depth_path = COALESCE(depth_path, ?) "
                    "WHERE frame_id = ?",
                    (rgb_rel, depth_rel, fid),
                )
                if row[3] is None and anchor_node_id is not None:
                    self.set_anchor(fid, anchor_node_id, anchor_T_body)
                return fid
            fid = self._next_id("frames", "frame_id")
            odom = np.eye(4) if world_T_body_odom is None else world_T_body_odom
            self.conn.execute(
                f"INSERT INTO frames ({_FRAME_COLS}) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    fid,
                    self.robot_id,
                    int(sensor_id),
                    ts,
                    int(kind),
                    transform_to_blob(odom),
                    None if anchor_node_id is None else int(anchor_node_id),
                    anchor_blob,
                    rgb_rel,
                    depth_rel,
                ),
            )
            return fid

    def set_anchor(self, frame_id: int, node_id: int, anchor_T_body=None) -> None:
        blob = transform_to_blob(np.eye(4) if anchor_T_body is None else anchor_T_body)
        with self.batch():
            self.conn.execute(
                "UPDATE frames SET anchor_node_id = ?, anchor_T_body = ? WHERE frame_id = ?",
                (int(node_id), blob, int(frame_id)),
            )

    def set_descriptor(self, frame_id: int, descriptor) -> None:
        """Store (replace) the descriptor of a frame; ``None`` removes it."""
        with self.batch():
            if descriptor is None:
                self.conn.execute(
                    "DELETE FROM frame_descriptors WHERE frame_id = ?", (int(frame_id),)
                )
                return
            arr = np.asarray(descriptor).reshape(-1)
            if arr.dtype.kind != "f":
                arr = arr.astype(np.float32)
            arr = arr.astype(arr.dtype.newbyteorder("<"), copy=False)
            self.conn.execute(
                "INSERT OR REPLACE INTO frame_descriptors (frame_id, dim, dtype, data) "
                "VALUES (?, ?, ?, ?)",
                (int(frame_id), int(arr.size), arr.dtype.str, arr.tobytes()),
            )

    def add_observation(
        self,
        track_id: int,
        frame_id: int,
        *,
        timestamp_ns: int | None = None,
        bbox2d: Sequence[int] | None = None,
        bbox3d: tuple[Sequence[float], Sequence[float]] | None = None,
        mask: np.ndarray | None = None,
        mask_rle: Sequence[int] | str | None = None,
        mask_path=None,
        score: float = 0.0,
        kept: bool = False,
        link: str = "hardlink",
    ) -> int:
        """Record one observation of ``track_id`` in ``frame_id``; returns its obs_id.

        ``mask`` (bool HxW) is stored as RLE (``mask_rle`` gives the run lengths
        directly); ``mask_path`` references an existing mask image, placed at
        ``masks/r<robot>/<obs_id><ext>`` like frame images. Tools other than the
        indexer use track ids from
        :data:`~hydra_python.image_store.codec.EXTERNAL_TRACK_BASE` up.
        """
        with self.batch():
            if timestamp_ns is None:
                row = self.conn.execute(
                    "SELECT timestamp_ns FROM frames WHERE frame_id = ?",
                    (int(frame_id),),
                ).fetchone()
                if row is None:
                    raise KeyError(f"unknown frame {frame_id}")
                timestamp_ns = row[0]
            oid = self._next_id("observations", "obs_id")
            rle = mask_rle
            if rle is None and mask is not None:
                rle = rle_encode(mask)
            if rle is not None and not isinstance(rle, str):
                rle = json.dumps([int(c) for c in rle], separators=(",", ":"))
            rel_mask = None
            if mask_path is not None:
                rel_mask = self._import_file(
                    mask_path,
                    f"masks/r{self.robot_id}/{oid}{pathlib.Path(mask_path).suffix}",
                    link,
                )
            b2 = [None] * 4 if bbox2d is None else [int(v) for v in bbox2d]
            b3 = (
                [None] * 6
                if bbox3d is None
                else [float(v) for v in (*bbox3d[0], *bbox3d[1])]
            )
            self.conn.execute(
                "INSERT INTO observations (obs_id, robot_id, track_id, frame_id, timestamp_ns, "
                "bbox2d_min_x, bbox2d_min_y, bbox2d_max_x, bbox2d_max_y, bbox3d_min_x, bbox3d_min_y, "
                "bbox3d_min_z, bbox3d_max_x, bbox3d_max_y, bbox3d_max_z, mask_rle, mask_path, score, kept) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    oid,
                    self.robot_id,
                    int(track_id),
                    int(frame_id),
                    int(timestamp_ns),
                    *b2,
                    *b3,
                    rle,
                    rel_mask,
                    float(score),
                    int(bool(kept)),
                ),
            )
            return oid

    def bind_track(
        self, track_id: int, node_id: int, robot_id: int | None = None
    ) -> None:
        """Bind a track to a scene graph node (replaces its previous binding). The
        indexer recomputes the bindings of the tracks it indexes on every run; it keeps
        those from :data:`~hydra_python.image_store.codec.EXTERNAL_TRACK_BASE` up."""
        robot = self.robot_id if robot_id is None else int(robot_id)
        with self.batch():
            self.conn.execute(
                "INSERT OR REPLACE INTO track_bindings (robot_id, track_id, node_id) VALUES (?, ?, ?)",
                (robot, int(track_id), int(node_id)),
            )

    def record_merges(
        self, merges: Mapping[int, int] | Iterable[tuple[int, int]]
    ) -> None:
        """Record ``child -> parent`` merges (upsert)."""
        items = merges.items() if isinstance(merges, Mapping) else merges
        with self.batch():
            self.conn.executemany(
                "INSERT OR REPLACE INTO node_aliases (child_id, parent_id) VALUES (?, ?)",
                [(int(c), int(p)) for c, p in items if int(c) != int(p)],
            )
