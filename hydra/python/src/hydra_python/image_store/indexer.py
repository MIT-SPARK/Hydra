"""Build an image store from Hydra/Khronos image logs (FORMAT.md, "indexer").

Inputs (any may be missing):

- keyframe ``InputData`` archives ``<keyframes>/<timestamp_ns>[_<sensor>].zip``
  (FORMAT.md §1; optionally with a feature vector, stored as the frame's descriptor)
- per-track object logs ``<tracks>/<track_id>/track.json`` + images (§2)
- frontend ``track_bindings.json`` (§3) and backend ``merges.json`` (§4)
- the backend (and frontend) scene graph ``dsg.json``

:func:`index` writes them into ``store.sqlite`` (schema_version 1) through
:class:`~hydra_python.image_store.ImageStoreWriter`. It is idempotent and incremental: running
it again only adds what is new (keyframe archives not indexed yet, ``track.json`` files
whose size or mtime changed), recomputes track bindings and node aliases, and anchors
the frames that are still pending (``final=False`` keeps frames after the newest agent
node pending, for a run Hydra is still writing).

Representation (see README "Indexer"):

- Each keyframe archive becomes a KEYFRAME frame. Colour is stored as a JPEG
  (``<stem>_rgb.jpg``) and metric depth as a uint16 millimetre PNG
  (``<stem>_depth.png``, ``depth_scale`` 0.001).
- Track observations become OBSERVATION frames (or share the KEYFRAME frame of the same
  sensor and stamp) and ``observations`` rows; kept ones reference their colour, depth
  and mask files (hardlinked into the store).
- Track -> node: the newest binding of the track that absorbed it (transitively) in
  ``track_bindings.json``, else the track's own newest binding. Tracks without one
  keep their observations but get no ``track_bindings`` row; they are listed with a
  reason in the ``meta`` key ``indexer.r<robot>.unbound_tracks`` (see
  :meth:`ImageStore.unbound_tracks`): ``extraction_failed`` (no binding, or only an
  ended one, and ``extracted: false``), ``node_deleted`` (the frontend deleted the
  node), ``node_missing`` (the node, after merges, is not in the backend scene graph)
  or ``no_binding``.
- Node aliases: backend ``merges.json`` (final parents). Re-indexing replaces the
  aliases this robot's previous indexing recorded.
- Anchoring: of this robot's agent nodes just before and just after the frame in time,
  the nearer in time (ties: the earlier); none when its odometric position is more than
  ``max_anchor_dist_m`` from the frame's (the frame stays unanchored and readers fall
  back to odometry). ``anchor_T_body = odom_world_T_anchor^-1 @ world_T_body_odom``.
  Odometric agent poses come from the frontend scene graph (the backend only keeps its
  optimized copy), else from the keyframe archive taken at the node's stamp, else (with
  a warning) the backend pose.
"""

from __future__ import annotations

import contextlib
import dataclasses
import json
import logging
import os
import pathlib
from collections import Counter
from collections.abc import Iterable

import numpy as np

from .codec import (
    EXTERNAL_TRACK_BASE,
    blob_to_transform,
    node_symbol,
    transform_to_blob,
)
from .poses import JsonDsg
from .store import DB_NAME, FrameKind, ImageStoreWriter, frame_stem

logger = logging.getLogger(__name__)

INDEXER_VERSION = 1
SIGNATURE_KEY = "indexer.input_signature"
STORE_DIR = "image_store"
KEYFRAME_DIRS = ("keyframes",)
TRACK_DIRS = ("tracks",)
HYDRA_DIRS = ("hydra",)
BINDINGS_FILE = "track_bindings.json"
MERGES_FILE = "merges.json"
DSG_FILES = ("dsg.json", "dsg_with_mesh.json", "dsg.sparkdsg")
#: an archive within this of an agent node's stamp gives the node's odometric pose
STAMP_TOL_NS = 1_000_000


# --- inputs -------------------------------------------------------------------------


@dataclasses.dataclass
class IndexInputs:
    """Where the raw artifacts of one robot's run are (any may be ``None``)."""

    store: pathlib.Path
    keyframes: pathlib.Path | None = None
    tracks: pathlib.Path | None = None
    hydra: pathlib.Path | None = None
    backend_dsg: pathlib.Path | None = None
    frontend_dsg: pathlib.Path | None = None
    bindings: pathlib.Path | None = None
    merges: pathlib.Path | None = None

    def to_json(self) -> dict:
        return {
            k: None if v is None else str(v)
            for k, v in dataclasses.asdict(self).items()
        }


def _first_dir(run: pathlib.Path, rels: Iterable[str]) -> pathlib.Path | None:
    for rel in rels:
        if (run / rel).is_dir():
            return (run / rel).absolute()
    return None


def _first_file(
    directory: pathlib.Path | None, names: Iterable[str]
) -> pathlib.Path | None:
    if directory is None:
        return None
    for name in names:
        if (directory / name).is_file():
            return directory / name
    return None


def find_inputs(
    run=None,
    *,
    keyframes=None,
    tracks=None,
    hydra=None,
    store=None,
    backend_dsg=None,
    frontend_dsg=None,
) -> IndexInputs:
    """Resolve the indexer inputs. Explicit arguments win; the rest is looked up in
    ``run`` (``keyframes/``, ``tracks/``, ``hydra/{backend,frontend}``, see FORMAT.md §5).
    The store defaults to ``<run>/image_store``."""
    run_p = None if run is None else pathlib.Path(run).expanduser().absolute()

    def p(v):
        return None if v is None else pathlib.Path(v).expanduser().absolute()

    kf = p(keyframes) or (run_p and _first_dir(run_p, KEYFRAME_DIRS))
    tr = p(tracks) or (run_p and _first_dir(run_p, TRACK_DIRS))
    hy = p(hydra) or (run_p and _first_dir(run_p, HYDRA_DIRS))
    if store is None:
        if run_p is None:
            raise ValueError("index needs a run directory or an explicit store")
        store = run_p / STORE_DIR
    backend = hy / "backend" if hy is not None else None
    frontend = hy / "frontend" if hy is not None else None
    return IndexInputs(
        store=p(store),
        keyframes=kf or None,
        tracks=tr or None,
        hydra=hy or None,
        # Hydra's backend saves into <log>/backend, Khronos' backend into <log> itself
        backend_dsg=p(backend_dsg)
        or _first_file(backend, DSG_FILES)
        or _first_file(hy, DSG_FILES),
        frontend_dsg=p(frontend_dsg) or _first_file(frontend, DSG_FILES),
        bindings=_first_file(frontend, [BINDINGS_FILE]),
        merges=_first_file(backend, [MERGES_FILE]) or _first_file(hy, [MERGES_FILE]),
    )


def has_raw_inputs(run) -> bool:
    """Whether ``run`` holds keyframe archives or track logs to index."""
    run = pathlib.Path(run)
    return bool(_first_dir(run, KEYFRAME_DIRS) or _first_dir(run, TRACK_DIRS))


def is_indexed(store_dir) -> bool:
    """Whether the store at ``store_dir`` was built (at least once) by the indexer."""
    import sqlite3
    import urllib.parse

    db = pathlib.Path(store_dir) / DB_NAME
    if not db.exists():
        return False
    try:
        uri = "file:" + urllib.parse.quote(str(db.absolute())) + "?mode=ro"
        with contextlib.closing(sqlite3.connect(uri, uri=True)) as conn:
            row = conn.execute(
                "SELECT value FROM meta WHERE key = 'indexer_version'"
            ).fetchone()
    except sqlite3.Error:
        return False
    return row is not None


# --- scene graph agents -------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class AgentNode:
    node_id: int
    timestamp_ns: int
    world_T_body: np.ndarray


def _agents_from_json(data: dict) -> list[AgentNode]:
    from .codec import pose_matrix, quat_to_rot

    out = []
    for node in data.get("nodes", []):
        attrs = node.get("attributes") or {}
        if attrs.get("type") != "AgentNodeAttributes" or "timestamp" not in attrs:
            continue
        q = attrs.get("world_R_body") or {"w": 1, "x": 0, "y": 0, "z": 0}
        T = pose_matrix(attrs["position"], quat_to_rot(q["w"], q["x"], q["y"], q["z"]))
        out.append(AgentNode(int(node["id"]), int(attrs["timestamp"]), T))
    return out


def _agents_from_spark(graph) -> list[AgentNode]:
    import datetime

    from .codec import pose_matrix, quat_to_rot

    out = []
    for node in graph.nodes:
        attrs = node.attributes
        stamp = getattr(attrs, "timestamp", None)
        q = getattr(attrs, "world_R_body", None)
        if stamp is None or q is None or not hasattr(attrs, "dbow_ids"):
            continue
        if isinstance(stamp, datetime.timedelta):  # pybind: microsecond resolution
            stamp = stamp // datetime.timedelta(microseconds=1) * 1000
        T = pose_matrix(attrs.position, quat_to_rot(q.w, q.x, q.y, q.z))
        out.append(AgentNode(int(node.id.value), int(stamp), T))
    return out


def load_agent_nodes(source, prefix: str | None = None) -> list[AgentNode]:
    """Agent nodes (time order) of a DSG: a ``.json`` path (parsed directly, exact ns
    stamps), another path (loaded with spark_dsg; its python stamps have microsecond
    resolution), a :class:`JsonDsg`, a spark_dsg graph or a list of :class:`AgentNode`.
    ``prefix`` keeps only nodes whose NodeSymbol category is that robot prefix."""
    if source is None:
        return []
    if isinstance(source, (str, os.PathLike)):
        path = pathlib.Path(source)
        if path.suffix == ".json":
            with open(path) as f:
                nodes = _agents_from_json(json.load(f))
        else:
            import spark_dsg

            nodes = _agents_from_spark(spark_dsg.DynamicSceneGraph.load(str(path)))
    elif isinstance(source, JsonDsg):
        nodes = _agents_from_json({"nodes": list(source.nodes.values())})
    elif isinstance(source, list):
        nodes = list(source)
    else:
        nodes = _agents_from_spark(source)
    if prefix is not None:
        nodes = [n for n in nodes if node_symbol(n.node_id)[0] == prefix]
    nodes.sort(key=lambda n: (n.timestamp_ns, n.node_id))
    return nodes


def dsg_node_ids(source) -> set[int]:
    """Ids of every node of a scene graph file (``.json`` parsed directly, other files
    with spark_dsg)."""
    path = pathlib.Path(source)
    if path.suffix == ".json":
        with open(path) as f:
            return {int(n["id"]) for n in json.load(f).get("nodes", [])}
    import spark_dsg

    graph = spark_dsg.DynamicSceneGraph.load(str(path))
    return {int(n.id.value) for n in graph.nodes}


def robot_prefix(robot_id: int) -> str:
    """Hydra's default agent prefix for a robot id (kimera_pgmo robot_id_to_prefix)."""
    return chr(ord("a") + int(robot_id))


def infer_robot_id(inputs: IndexInputs, agent_prefix: str | None = None) -> int:
    """Robot id of a run: from ``agent_prefix``, else the ``robot_id`` the Khronos track
    logger recorded, else the one the indexer used for this store before, else from
    the only agent prefix of the frontend scene graph (a robot's frontend holds only
    its own agent), else of the backend one, else 0 (with a warning when the backend
    shows several robots)."""

    def valid(c: str) -> bool:
        return "a" <= c <= "z"

    if agent_prefix is not None and valid(agent_prefix):
        return ord(agent_prefix) - ord("a")
    logged = _logged_robot(inputs.tracks)
    if logged is not None:
        return logged
    previous = _previous_robot(inputs.store)
    if previous is not None:
        return previous
    for dsg in (inputs.frontend_dsg, inputs.backend_dsg):
        if dsg is None:
            continue
        try:
            prefixes = {node_symbol(n.node_id)[0] for n in load_agent_nodes(dsg)}
        except Exception as exc:  # noqa: BLE001 - unreadable graph: try the next one
            logger.warning("%s: cannot read agent nodes: %s", dsg, exc)
            continue
        prefixes = {c for c in prefixes if valid(c)}
        if len(prefixes) == 1:
            return ord(prefixes.pop()) - ord("a")
        if len(prefixes) > 1:
            logger.warning(
                "%s has agents of several robots %s; pass robot_id",
                dsg,
                sorted(prefixes),
            )
    return 0


def _logged_robot(tracks: pathlib.Path | None) -> int | None:
    """``robot_id`` recorded by the Khronos track logger (first readable track.json)."""
    if tracks is None or not tracks.is_dir():
        return None
    for d in sorted(tracks.iterdir(), key=lambda p: p.name):
        try:
            return int(_load_json(d / "track.json")["robot_id"])
        except (OSError, ValueError, TypeError, KeyError):
            continue
    return None


def _previous_robot(store: pathlib.Path) -> int | None:
    import sqlite3
    import urllib.parse

    db = store / DB_NAME
    if not db.exists():
        return None
    try:
        uri = "file:" + urllib.parse.quote(str(db)) + "?mode=ro"
        with contextlib.closing(sqlite3.connect(uri, uri=True)) as conn:
            rows = conn.execute(
                "SELECT key FROM meta WHERE key LIKE 'indexer.r%.sources'"
            ).fetchall()
    except sqlite3.Error:
        return None
    robots = {int(k.split(".")[1][1:]) for (k,) in rows}
    return robots.pop() if len(robots) == 1 else None


# --- helpers ------------------------------------------------------------------------


def archive_sensor_suffix(sensor_name: str) -> str:
    """Sensor suffix of a keyframe archive name as Hydra's KeyframeSelector writes it:
    characters outside ``[A-Za-z0-9_.-]`` become ``_``."""
    return "".join(
        c if (c.isascii() and c.isalnum()) or c in "_.-" else "_" for c in sensor_name
    )


def parse_archive_name(name: str) -> tuple[int, str | None] | None:
    """``(timestamp_ns, sensor suffix or None)`` of ``<ts>.zip`` / ``<ts>_<sensor>.zip``
    (FORMAT.md §1), ``None`` for anything else (e.g. Hydra's temporary files)."""
    if not name.endswith(".zip"):
        return None
    stem = name[: -len(".zip")]
    head, sep, suffix = stem.partition("_")
    if not (head.isascii() and head.isdigit()) or (sep and not suffix):
        return None
    return int(head), (suffix if sep else None)


def _load_json(path: pathlib.Path):
    with open(path) as f:
        return json.load(f)


def _matrix(values) -> np.ndarray | None:
    if values is None:
        return None
    arr = np.asarray(values, dtype=float)
    return arr.reshape(4, 4) if arr.size == 16 else None


def _track_observation(obs: dict):
    """``(obs, sensor, timestamp_ns, kept, mask_rle)`` of a ``track.json`` observation
    (FORMAT.md §2); raises ``KeyError`` / ``ValueError`` / ``TypeError`` when malformed."""
    from .inputdata import SensorInfo

    keys = ("world_T_body", "bbox_2d", "score", "color", "depth", "mask")
    missing = [k for k in keys if k not in obs]  # required, possibly null
    if missing:
        raise KeyError(f"observation without {missing}")
    s = obs["sensor"]
    body_T_sensor = _matrix(s["body_T_sensor"])
    info = SensorInfo(
        name=str(s["name"]),
        width=int(s["width"]),
        height=int(s["height"]),
        # null for non-pinhole sensors
        fx=float(s["fx"] or 0),
        fy=float(s["fy"] or 0),
        cx=float(s["cx"] or 0),
        cy=float(s["cy"] or 0),
        body_T_sensor=np.eye(4) if body_T_sensor is None else body_T_sensor,
    )
    rle = obs["mask_rle"]
    if rle is not None and not isinstance(rle, str):
        raise TypeError(f"mask_rle must be a string or null, got {type(rle).__name__}")
    return obs, info, int(obs["timestamp_ns"]), bool(obs["kept"]), rle or None


def _sig(path: pathlib.Path) -> list[int]:
    st = path.stat()
    return [int(st.st_mtime_ns), int(st.st_size)]


@contextlib.contextmanager
def _locked(store_dir: pathlib.Path):
    """Serialize indexers of one store (several processes may open a run at once)."""
    store_dir.mkdir(parents=True, exist_ok=True)
    with open(store_dir / ".index.lock", "w") as f:
        try:
            import fcntl

            fcntl.flock(f, fcntl.LOCK_EX)
        except ImportError:  # pragma: no cover - non-POSIX
            pass
        yield


class _Indexer:
    def __init__(self, writer: ImageStoreWriter, inputs: IndexInputs, opts: dict):
        self.w = writer
        self.conn = writer.conn
        self.inp = inputs
        self.opts = opts
        self.robot = writer.robot_id
        self.stats: Counter = Counter()
        self.warnings: list[str] = []
        self._sensors: dict[str, int] = {}
        self.key = f"indexer.r{self.robot}."
        self.created: set[int] = set()

    # meta ---------------------------------------------------------------------------
    def meta(self, name: str, default=None):
        row = self.conn.execute(
            "SELECT value FROM meta WHERE key = ?", (self.key + name,)
        ).fetchone()
        return default if row is None else json.loads(row[0])

    def set_meta(self, name: str, value) -> None:
        self.w.set_meta(self.key + name, json.dumps(value, separators=(",", ":")))

    def warn(self, msg: str, *args) -> None:
        text = msg % args if args else msg
        logger.warning(text)
        if len(self.warnings) < 50:
            self.warnings.append(text)

    # sensors ------------------------------------------------------------------------
    def sensor_id(self, info) -> int:
        """Sensor row for ``(robot, name)``; the first calibration seen is kept."""
        name = info.name or "camera"
        if name in self._sensors:
            return self._sensors[name]
        row = self.conn.execute(
            "SELECT sensor_id FROM sensors WHERE robot_id = ? AND name = ?",
            (self.robot, name),
        ).fetchone()
        if row is None:
            sid = self.w.add_sensor(
                name,
                info.width,
                info.height,
                info.fx,
                info.fy,
                info.cx,
                info.cy,
                0.001,
                info.body_T_sensor,
            )
            self.stats["sensors_added"] += 1
        else:
            sid = row[0]
        self._sensors[name] = sid
        return sid

    def _frame_row(self, sid: int, ts: int):
        return self.conn.execute(
            "SELECT frame_id, kind, rgb_path, depth_path FROM frames "
            "WHERE sensor_id = ? AND timestamp_ns = ?",
            (sid, ts),
        ).fetchone()

    # keyframes ----------------------------------------------------------------------
    def index_keyframes(self) -> None:
        from .inputdata import depth_to_mm, read_input_zip, write_jpeg, write_png16

        kf_dir = self.inp.keyframes
        if kf_dir is None or not kf_dir.is_dir():
            return
        # (sanitized sensor name, stamp) of every archive frame already indexed
        done_by_sensor: dict[int, set[str]] = {}
        for name, t in self.conn.execute(
            "SELECT s.name, f.timestamp_ns FROM frames f JOIN sensors s "
            "ON s.sensor_id = f.sensor_id WHERE f.robot_id = ? AND f.kind = ?",
            (self.robot, int(FrameKind.KEYFRAME)),
        ):
            done_by_sensor.setdefault(t, set()).add(archive_sensor_suffix(name))
        # archives that could not be read, with their (mtime, size): not read (and
        # warned about) again until they change
        failed = self.meta("failed_keyframes", {})
        zips = []
        for path in kf_dir.glob("*.zip"):
            parsed = parse_archive_name(path.name)
            if parsed is not None:
                zips.append((*parsed, path))
        # <ts>.zip is the first sensor Hydra saved, <ts>_<sensor>.zip any other one
        suffixed: dict[int, set[str]] = {}
        for t, suffix, _ in zips:
            if suffix is not None:
                suffixed.setdefault(t, set()).add(suffix)

        def is_done(t: int, suffix: str | None) -> bool:
            have = done_by_sensor.get(t, set())
            if suffix is not None:
                return suffix in have
            return bool(have - suffixed.get(t, set()))

        zips.sort(key=lambda z: (z[0], z[1] or ""))

        def unchanged_failure(path) -> bool:
            if failed.get(path.name) is None:
                return False
            try:
                return _sig(path) == failed[path.name]
            except OSError:
                return False

        pending = [
            (t, p)
            for t, suffix, p in zips
            if not is_done(t, suffix) and not unchanged_failure(p)
        ]
        self.stats["keyframes_failed"] += len(
            [p for t, s, p in zips if not is_done(t, s) and unchanged_failure(p)]
        )
        self.stats["keyframe_archives"] = len(zips)
        for i in range(0, len(pending), 50):
            with self.w.batch():
                for name_ts, path in pending[i : i + 50]:
                    try:
                        rec = read_input_zip(path)
                    except Exception as exc:  # noqa: BLE001 - corrupt / unsupported archive
                        self.warn("%s: cannot read keyframe archive: %s", path, exc)
                        self.stats["keyframes_failed"] += 1
                        with contextlib.suppress(OSError):
                            failed[path.name] = _sig(path)
                        continue
                    failed.pop(path.name, None)
                    ts = rec.timestamp_ns
                    if ts != name_ts:
                        self.warn(
                            "%s: metadata timestamp %d differs from name", path, ts
                        )
                    sid = self.sensor_id(rec.sensor)
                    row = self._frame_row(sid, ts)
                    base = self.w.root / f"frames/r{self.robot}/{frame_stem(ts, sid)}"
                    rgb = depth = None
                    if rec.color is not None and not (row and row[2]):
                        rgb = base.with_name(base.name + "_rgb.jpg")
                        write_jpeg(rgb, rec.color, self.opts["jpeg_quality"])
                    if rec.depth is not None and not (row and row[3]):
                        depth = base.with_name(base.name + "_depth.png")
                        write_png16(depth, depth_to_mm(rec.depth))
                    fid = self.w.add_frame_ref(
                        sid,
                        ts,
                        kind=FrameKind.KEYFRAME,
                        world_T_body_odom=rec.world_T_body,
                        rgb=rgb,
                        depth=depth,
                    )
                    if rec.feature is not None:
                        self.w.set_descriptor(fid, rec.feature)
                        self.stats["descriptors_indexed"] += 1
                    if row is not None and row[1] != FrameKind.KEYFRAME:
                        # an observation frame indexed before its archive
                        self.conn.execute(
                            "UPDATE frames SET kind = ? WHERE frame_id = ?",
                            (int(FrameKind.KEYFRAME), fid),
                        )
                    self.created.add(fid)
                    self.stats["keyframes_indexed"] += 1
        self.set_meta("failed_keyframes", dict(sorted(failed.items())))

    # tracks -------------------------------------------------------------------------
    def _delete_track(self, track_id: int) -> None:
        rows = self.conn.execute(
            "SELECT obs_id, mask_path FROM observations WHERE robot_id = ? AND track_id = ?",
            (self.robot, track_id),
        ).fetchall()
        self.conn.execute(
            "DELETE FROM observations WHERE robot_id = ? AND track_id = ?",
            (self.robot, track_id),
        )
        for _, mask in rows:
            if mask and mask.startswith("masks/"):
                with contextlib.suppress(OSError):
                    (self.w.root / mask).unlink()

    def index_tracks(self) -> dict:
        tracks = self.meta("tracks", {})
        tr_dir = self.inp.tracks
        if tr_dir is None or not tr_dir.is_dir():
            return tracks
        link = self.opts["link"]
        for d in sorted(tr_dir.iterdir(), key=lambda p: p.name):
            tj = d / "track.json"
            if not d.is_dir() or not tj.is_file():
                continue
            sig = _sig(tj)
            prev = tracks.get(d.name)
            if prev is not None and prev.get("sig") == sig:
                continue
            try:
                data = _load_json(tj)
                track_id = int(data["track_id"])
                if int(data["robot_id"]) != self.robot:
                    self.warn(
                        "%s: robot_id %s, indexing robot %d",
                        tj,
                        data["robot_id"],
                        self.robot,
                    )
                observations = [_track_observation(o) for o in data["observations"]]
                info = {
                    "sig": sig,
                    "track_id": track_id,
                    "absorbed": [int(t) for t in data["absorbed_track_ids"]],
                    "extracted": bool(data["extracted"]),
                    "semantic_label": data["semantic_label"],
                }
            except (OSError, ValueError, TypeError, KeyError) as exc:
                self.warn("%s: cannot read track log: %s", tj, exc)
                self.stats["tracks_failed"] += 1
                continue
            with self.w.batch():
                self._delete_track(track_id)
                for obs, info_s, ts, kept, rle in observations:
                    sid = self.sensor_id(info_s)

                    def existing(key, obs=obs, kept=kept, d=d):
                        name = obs.get(key)
                        if not (kept and name):
                            return None
                        path = d / name
                        if not path.is_file():
                            self.warn("%s: missing %s", d, name)
                            self.stats["track_files_missing"] += 1
                            return None
                        return path

                    fid = self.w.add_frame_ref(
                        sid,
                        ts,
                        kind=FrameKind.OBSERVATION,
                        world_T_body_odom=_matrix(obs["world_T_body"]),
                        rgb=existing("color"),
                        depth=existing("depth"),
                        link=link,
                    )
                    self.created.add(fid)
                    bbox = obs["bbox_2d"]
                    self.w.add_observation(
                        track_id,
                        fid,
                        timestamp_ns=ts,
                        bbox2d=None if not bbox else bbox,
                        mask_rle=rle,
                        mask_path=existing("mask"),
                        score=float(obs["score"] or 0.0),
                        kept=kept,
                        link=link,
                    )
                    self.stats["observations_indexed"] += 1
                    self.stats[
                        "observations_kept" if kept else "observations_not_kept"
                    ] += 1
            tracks[d.name] = info
            self.stats["tracks_indexed"] += 1
        self.set_meta("tracks", tracks)
        return tracks

    # bindings / merges --------------------------------------------------------------
    def _load_merges(self) -> dict[int, int]:
        """Backend ``child -> final parent`` merges (merges.json)."""
        merges: dict[int, int] = {}
        if self.inp.merges is not None:
            try:
                for m in _load_json(self.inp.merges).get("merges") or []:
                    merges[int(m["child"])] = int(m["parent"])
            except (OSError, ValueError, TypeError, KeyError) as exc:
                self.warn("%s: cannot read merges: %s", self.inp.merges, exc)
        return merges

    def _backend_nodes(self) -> set[int] | None:
        """Node ids of the backend scene graph, ``None`` when it is not available."""
        path = self.inp.backend_dsg
        if path is None:
            return None
        try:
            return dsg_node_ids(path)
        except Exception as exc:  # noqa: BLE001 - unreadable graph: skip the check
            self.warn("%s: cannot read node ids: %s", path, exc)
            return None

    def index_bindings(self, tracks: dict) -> None:
        latest: dict[int, dict] = {}
        deleted_nodes: set[int] = set()
        if self.inp.bindings is not None:
            try:
                data = _load_json(self.inp.bindings)
            except (OSError, ValueError) as exc:
                self.warn("%s: cannot read bindings: %s", self.inp.bindings, exc)
                data = {}
            layers = data.get("layers") or {}
            # keys are graph_updater layer names ("OBJECTS" by default): use the object
            # layer(s) when present, whatever the case, else every layer
            objects = [v for k, v in layers.items() if str(k).lower() == "objects"]
            for layer in objects or list(layers.values()):
                for b in layer or []:
                    t = int(b["track_id"])
                    if b.get("end") == "deleted":
                        deleted_nodes.add(int(b["node_id"]))
                    key = (
                        bool(b.get("active", False)),
                        int(b.get("last_ns", 0) or 0),
                        int(b.get("first_ns", 0) or 0),
                    )
                    if t not in latest or key >= latest[t]["_key"]:
                        latest[t] = {**b, "_key": key}
        # a binding to a node the frontend deleted points at nothing: drop it so the track
        # resolves elsewhere or is unbound
        dropped_deleted = {
            t for t, b in latest.items() if int(b["node_id"]) in deleted_nodes
        }
        for t in dropped_deleted:
            del latest[t]
        absorbed_into: dict[int, int] = {}
        extracted: dict[int, bool] = {}
        for info in tracks.values():
            extracted[info["track_id"]] = info["extracted"]
            for child in info["absorbed"]:
                absorbed_into[child] = info["track_id"]

        def resolve(t: int):
            chain, seen = [t], {t}
            while chain[-1] in absorbed_into and absorbed_into[chain[-1]] not in seen:
                seen.add(absorbed_into[chain[-1]])
                chain.append(absorbed_into[chain[-1]])
            root = chain[-1]
            failed = not extracted.get(root, True)
            for c in reversed(chain):  # the track that absorbed it, else its own
                if c in latest:
                    # an ended binding of a track whose extraction failed: Khronos never
                    # produced its object, the node belongs to the other tracks bound to it
                    if failed and latest[c].get("end") is not None:
                        return None, "extraction_failed"
                    return int(latest[c]["node_id"]), None
            if failed:
                return None, "extraction_failed"
            if any(c in dropped_deleted for c in chain):
                return None, "node_deleted"
            return None, "no_binding"

        merges = self._load_merges()

        def canonical(node: int) -> int:
            seen = {node}
            while node in merges and merges[node] not in seen:
                node = merges[node]
                seen.add(node)
            return node

        backend_nodes = self._backend_nodes()
        all_tracks = set(extracted) | set(latest) | set(absorbed_into)
        bound: dict[int, int] = {}
        unbound: dict[str, str] = {}
        for t in sorted(all_tracks):
            node, reason = resolve(t)
            if (
                node is not None
                and backend_nodes is not None
                and canonical(node) not in backend_nodes
            ):
                node, reason = None, "node_missing"
            if node is not None:
                bound[t] = node
            elif t in extracted:  # only tracks with a log carry observations
                unbound[str(t)] = reason
        with self.w.batch():
            stale = [
                r[0]
                for r in self.conn.execute(
                    "SELECT track_id FROM track_bindings WHERE robot_id = ? AND track_id < ?",
                    (self.robot, EXTERNAL_TRACK_BASE),
                )
                if r[0] not in bound
            ]
            self.conn.executemany(
                "DELETE FROM track_bindings WHERE robot_id = ? AND track_id = ?",
                [(self.robot, t) for t in stale],
            )
            self.conn.executemany(
                "INSERT OR REPLACE INTO track_bindings (robot_id, track_id, node_id) VALUES (?, ?, ?)",
                [(self.robot, t, n) for t, n in bound.items()],
            )
            self.set_meta("unbound_tracks", unbound)
            # aliases this robot's previous indexing recorded that merges.json no longer
            # has (e.g. it was rewritten by a later save) are stale
            previous = set(self.meta("aliases", []))
            self.conn.executemany(
                "DELETE FROM node_aliases WHERE child_id = ?",
                [(c,) for c in sorted(previous - set(merges))],
            )
            self.w.record_merges(merges)
            self.set_meta("aliases", sorted(merges))
        self.stats["track_bindings"] = len(bound)
        self.stats["tracks_unbound"] = len(unbound)
        self.stats["node_aliases"] = len(merges)

    # anchoring ----------------------------------------------------------------------
    def _odom_poses(
        self, agents: list[AgentNode], from_frontend: bool
    ) -> dict[int, np.ndarray]:
        poses: dict[int, np.ndarray] = {}
        if from_frontend:  # the agents were read from the frontend graph itself
            poses = {a.node_id: a.world_T_body for a in agents}
        elif self.inp.frontend_dsg is not None:
            for n in load_agent_nodes(self.inp.frontend_dsg):
                poses[n.node_id] = n.world_T_body
        rows = self.conn.execute(
            "SELECT timestamp_ns, world_T_body_odom FROM frames "
            "WHERE robot_id = ? AND kind = ? ORDER BY timestamp_ns",
            (self.robot, int(FrameKind.KEYFRAME)),
        ).fetchall()
        stamps = np.array([r[0] for r in rows], dtype=np.int64)
        for a in agents:
            if a.node_id in poses:
                self.stats["anchor_odom_frontend"] += 1
                continue
            if stamps.size:
                i = int(np.argmin(np.abs(stamps - a.timestamp_ns)))
                if abs(int(stamps[i]) - a.timestamp_ns) <= STAMP_TOL_NS:
                    poses[a.node_id] = blob_to_transform(rows[i][1])
                    self.stats["anchor_odom_keyframe"] += 1
                    continue
            poses[a.node_id] = a.world_T_body
            self.stats["anchor_odom_backend"] += 1
        if self.stats["anchor_odom_backend"]:
            self.warn(
                "%d agent nodes have no odometric pose (no frontend DSG or keyframe at "
                "their stamp); anchored with backend poses",
                self.stats["anchor_odom_backend"],
            )
        return poses

    def anchor(self, agent_source, final: bool, from_frontend: bool = False) -> None:
        pending = set(self.meta("pending_frames", [])) | self.created
        if not pending:
            return
        prefix = self.opts["agent_prefix"]
        if agent_source is None:
            self.warn("no scene graph: %d frames stay unanchored for now", len(pending))
            self.set_meta("pending_frames", sorted(pending))
            self.stats["frames_pending"] = len(pending)
            return
        agents = load_agent_nodes(agent_source, prefix)
        if not agents:
            self.warn("scene graph has no agent nodes with prefix %r", prefix)
        odom = self._odom_poses(agents, from_frontend)
        stamps = np.array([a.timestamp_ns for a in agents], dtype=np.int64)
        newest = int(stamps[-1]) if stamps.size else None
        frames = {}
        for chunk in [
            sorted(pending)[i : i + 900] for i in range(0, len(pending), 900)
        ]:
            q = ",".join("?" * len(chunk))
            for fid, ts, odom_blob, anchor in self.conn.execute(
                f"SELECT frame_id, timestamp_ns, world_T_body_odom, anchor_node_id "
                f"FROM frames WHERE frame_id IN ({q})",
                chunk,
            ):
                if anchor is None:
                    frames[fid] = (ts, blob_to_transform(odom_blob))
        decided: dict[int, tuple | None] = {}
        max_dist = self.opts["max_anchor_dist_m"]
        for fid, (ts, T) in frames.items():
            if newest is None or (ts > newest and not final):
                continue  # wait for the next agent node
            i = int(np.searchsorted(stamps, ts))  # neighbours: before and at/after
            cands = [j for j in (i - 1, i) if 0 <= j < stamps.size]
            best = min(cands, key=lambda j: (abs(int(stamps[j]) - ts), j))
            a = agents[best]
            W = odom[a.node_id]
            if np.linalg.norm(W[:3, 3] - T[:3, 3]) > max_dist:
                decided[fid] = None  # too far from any agent node: stays unanchored
                continue
            decided[fid] = (a.node_id, np.linalg.inv(W) @ T)
        with self.w.batch():
            for fid, d in decided.items():
                if d is None:
                    self.stats["frames_unanchored"] += 1
                    continue
                node, rel = d
                self.conn.execute(
                    "UPDATE frames SET anchor_node_id = ?, anchor_T_body = ? WHERE frame_id = ?",
                    (int(node), transform_to_blob(rel), fid),
                )
                self.stats["frames_anchored"] += 1
            still = sorted(set(frames) - set(decided))
            self.set_meta("pending_frames", still)
        self.stats["frames_pending"] = len(still)


def index(
    run=None,
    *,
    keyframes=None,
    tracks=None,
    hydra=None,
    store=None,
    dsg=None,
    frontend_dsg=None,
    robot_id: int | None = None,
    agent_prefix: str | None = None,
    max_anchor_dist_m: float = 2.0,
    jpeg_quality: int = 90,
    link: str = "hardlink",
    final: bool = True,
) -> dict:
    """Build or update the image store of a run from its raw Hydra/Khronos logs.

    ``run`` is the run directory (inputs looked up as in FORMAT.md §5); ``keyframes``,
    ``tracks``, ``hydra`` (Hydra's log dir with ``backend/`` and ``frontend/``),
    ``dsg`` / ``frontend_dsg`` (scene graph files) and ``store`` (default
    ``<run>/image_store``) override the lookup. ``robot_id`` namespaces ids
    (``robot << 48 | seq``) and picks the agent prefix (``a`` + robot_id unless
    ``agent_prefix`` is given); ``None`` infers it from the scene graphs (see
    :func:`infer_robot_id`). ``final=False`` leaves frames newer than the newest agent
    node pending (Hydra still running); ``final=True`` decides them with the newest
    agent node.

    Returns statistics. Safe to call repeatedly: only new inputs are added.
    """
    inputs = find_inputs(
        run,
        keyframes=keyframes,
        tracks=tracks,
        hydra=hydra,
        store=store,
        backend_dsg=dsg,
        frontend_dsg=frontend_dsg,
    )
    with _locked(inputs.store):
        return _index(
            inputs,
            robot_id=robot_id,
            agent_prefix=agent_prefix,
            max_anchor_dist_m=max_anchor_dist_m,
            jpeg_quality=jpeg_quality,
            link=link,
            final=final,
            signature=input_signature(inputs),
        )


def _index(
    inputs: IndexInputs,
    *,
    robot_id: int | None = None,
    agent_prefix: str | None = None,
    max_anchor_dist_m: float = 2.0,
    jpeg_quality: int = 90,
    link: str = "hardlink",
    final: bool = True,
    signature: dict | None = None,
) -> dict:
    """:func:`index` for resolved inputs; the caller holds the store's index lock.
    ``signature`` (taken before reading the inputs) is recorded for
    :func:`ensure_indexed`."""
    if robot_id is None:
        robot_id = infer_robot_id(inputs, agent_prefix)
    opts = {
        "agent_prefix": agent_prefix
        if agent_prefix is not None
        else robot_prefix(robot_id),
        "max_anchor_dist_m": float(max_anchor_dist_m),
        "jpeg_quality": int(jpeg_quality),
        "link": link,
    }
    writer = ImageStoreWriter(inputs.store, robot_id)
    try:
        ix = _Indexer(writer, inputs, opts)
        ix.index_keyframes()
        track_info = ix.index_tracks()
        ix.index_bindings(track_info)
        # the backend graph's agent nodes; while it is not saved yet, the frontend's
        # (same ids, odometric poses)
        agent_source = inputs.backend_dsg or inputs.frontend_dsg
        ix.anchor(agent_source, final, from_frontend=inputs.backend_dsg is None)
        writer.set_meta("indexer_version", INDEXER_VERSION)
        if signature is not None:
            writer.set_meta(SIGNATURE_KEY, json.dumps(signature, separators=(",", ":")))
        ix.set_meta("sources", {**inputs.to_json(), **{k: v for k, v in opts.items()}})
    finally:
        writer.close()
    stats = dict(sorted(ix.stats.items()))
    stats["store"] = str(inputs.store)
    stats["inputs"] = inputs.to_json()
    if ix.warnings:
        stats["warnings"] = ix.warnings
    return stats


def input_signature(inputs: IndexInputs) -> dict:
    """Cheap fingerprint of a run's raw inputs: (mtime, size) of the scene graphs,
    bindings and merges, the keyframe archive count and directory mtime (archives are
    renamed into place) and (mtime, size) of every ``track.json``."""

    def sig(path: pathlib.Path | None):
        try:
            return None if path is None else _sig(path)
        except OSError:
            return None

    out: dict = {
        k: sig(getattr(inputs, k))
        for k in ("backend_dsg", "frontend_dsg", "bindings", "merges")
    }
    kf = inputs.keyframes
    if kf is not None and kf.is_dir():
        out["keyframes"] = [
            sum(1 for p in kf.iterdir() if parse_archive_name(p.name)),
            *_sig(kf),
        ]
    tr = inputs.tracks
    if tr is not None and tr.is_dir():
        tracks = {}
        for d in tr.iterdir():
            s = sig(d / "track.json")
            if s is not None:
                tracks[d.name] = s
        out["tracks"] = sorted(tracks.items())
    return json.loads(json.dumps(out))  # tuples -> lists, as read back from meta


def _stored_signature(store: pathlib.Path):
    import sqlite3
    import urllib.parse

    db = store / DB_NAME
    uri = "file:" + urllib.parse.quote(str(db.absolute())) + "?mode=ro"
    try:
        with contextlib.closing(sqlite3.connect(uri, uri=True)) as conn:
            row = conn.execute(
                "SELECT value FROM meta WHERE key = ?", (SIGNATURE_KEY,)
            ).fetchone()
    except sqlite3.Error:
        return None
    return None if row is None else json.loads(row[0])


def ensure_indexed(path) -> pathlib.Path | None:
    """If ``path`` is a run directory with raw logs (or the ``image_store`` directory of
    one) whose store is missing or out of date, (re)build it. Returns the store
    directory to open, or ``None`` when ``path`` is not such a run.

    The store is (re)indexed when it was never indexed, or when the raw inputs changed
    since the last indexing (:func:`input_signature`: new archives or track logs,
    rewritten ``track.json``, saved scene graphs, bindings or merges). Frames left
    pending by an earlier pass (e.g. indexed while Hydra was running) are therefore
    anchored as soon as the scene graphs they wait for are saved; re-running on
    unchanged inputs cannot decide them, so it is skipped. Check and indexing happen
    under the store's index lock. Note that this writes the store even for a read-only
    ``ImageStore.open``; pass ``index=False`` there to never write.
    """
    p = pathlib.Path(path).expanduser()
    if p.suffix == ".sqlite" or p.name == DB_NAME:
        p = p.parent
    if p.name == STORE_DIR and has_raw_inputs(p.parent):
        run, store = p.parent, p
    elif p.is_dir() and not (p / DB_NAME).exists() and has_raw_inputs(p):
        run, store = p, p / STORE_DIR
    else:
        return None
    inputs = find_inputs(run, store=store)
    with _locked(store):
        current = input_signature(inputs)
        if is_indexed(store) and _stored_signature(store) == current:
            return store
        logger.info("indexing raw image logs of %s into %s", run, store)
        # without a saved backend graph Hydra is probably still running: keep frames
        # after the newest agent node pending (decided by a later `index`)
        final = inputs.backend_dsg is not None
        _index(inputs, final=final, signature=current)
    return store


__all__ = [
    "AgentNode",
    "IndexInputs",
    "archive_sensor_suffix",
    "dsg_node_ids",
    "input_signature",
    "parse_archive_name",
    "ensure_indexed",
    "find_inputs",
    "has_raw_inputs",
    "index",
    "infer_robot_id",
    "is_indexed",
    "load_agent_nodes",
    "robot_prefix",
]
