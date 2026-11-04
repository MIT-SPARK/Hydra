"""Read Hydra's live event logs (FORMAT.md §6).

Hydra's ``FrontendEventLogger`` (a frontend sink) and ``MergeEventLogger`` (a backend
merge sink) append JSON lines to files in an events directory while Hydra runs:

- ``{"ev": "open", "version": 1, "wall_ns": .., "source": "frontend"|"backend",
  "robot_id": ..}`` each time a process (re)opens the file
- frontend: ``bind`` / ``end`` (track -> node binding changes) and ``agent`` (new agent
  node with its odometric ``world_T_body``)
- backend: ``merge`` (``child`` -> ``parent`` set or changed) and ``unmerge``

:class:`EventLogs` follows every ``*.jsonl`` file of the directory incrementally (byte
offsets; an incomplete last line is left for the next read) and replays the events into
the same structures the save-time files give: the ``track_bindings.json`` layers, the
``merges.json`` child -> parent map and the agent nodes of the scene graph.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import os
import pathlib

import numpy as np

logger = logging.getLogger(__name__)

EVENT_DIRS = ("events",)
EVENT_GLOB = "*.jsonl"


@dataclasses.dataclass
class _FileState:
    path: pathlib.Path
    inode: int | None = None
    offset: int = 0
    session: int | None = None  # wall_ns of the last "open" line
    source: str | None = None
    robot_id: int | None = None
    mtime_ns: int = 0
    head: bytes = b""  # first bytes (the "open" line): detects a recreated file


class _Replay:
    """Replayed state of one log file (since its last "open" line)."""

    def __init__(self):
        self.layers: dict[str, list[dict]] = {}
        self.active: dict[tuple[str, int], dict] = {}
        self.merges: dict[int, int] = {}
        self.agents: dict[int, tuple[int, list]] = {}

    def apply(self, ev: dict) -> None:
        kind = ev.get("ev")
        if kind == "bind":
            layer = str(ev.get("layer", ""))
            key = (layer, int(ev["track_id"]))
            prev = self.active.pop(key, None)
            if prev is not None:  # the writer ends it explicitly; be lenient
                prev["active"] = False
                prev["end"] = prev["end"] or "track_removed"
            stamp = int(ev.get("stamp_ns", 0) or 0)
            entry = {
                "track_id": key[1],
                "node_id": int(ev["node_id"]),
                "first_ns": stamp,
                "last_ns": stamp,
                "active": True,
                "end": None,
            }
            self.layers.setdefault(layer, []).append(entry)
            self.active[key] = entry
        elif kind == "end":
            key = (str(ev.get("layer", "")), int(ev["track_id"]))
            entry = self.active.pop(key, None)
            if entry is not None:
                entry["active"] = False
                entry["end"] = ev.get("reason") or "track_removed"
        elif kind == "agent":
            T = ev.get("world_T_body")
            if T is not None and len(T) == 16:
                self.agents[int(ev["node_id"])] = (int(ev["timestamp_ns"]), T)
        elif kind == "merge":
            self.merges[int(ev["child"])] = int(ev["parent"])
        elif kind == "unmerge":
            self.merges.pop(int(ev["child"]), None)


def _glued_event(raw: bytes) -> dict | None:
    """The complete event at the end of a malformed line, if any.

    A writer killed in the middle of a line leaves it unterminated; a restarted Hydra
    appending to the same file then glues its first line (the ``open`` that starts the
    new session) to that fragment. Losing that ``open`` would mix the two sessions'
    events, so the last parsable ``{"ev"...}`` object of the line is recovered."""
    start = len(raw)
    while True:
        start = max(raw.rfind(b'{"ev"', 0, start), raw.rfind(b'{ "ev"', 0, start))
        if start <= 0:
            return None
        try:
            ev = json.loads(raw[start:])
        except ValueError:
            continue
        return ev if isinstance(ev, dict) and "ev" in ev else None


def find_events_dir(run) -> pathlib.Path | None:
    """The events directory of a run (``events/``)."""
    if run is None:
        return None
    run = pathlib.Path(run)
    for rel in EVENT_DIRS:
        if (run / rel).is_dir():
            return (run / rel).absolute()
    return None


class EventLogs:
    """Incremental reader of the event logs in one directory.

    :meth:`poll` reads what was appended since the last call and returns whether
    anything changed. A file that was replaced (new inode, or shorter than what was
    read) is read again from the start; an "open" line (a restarted Hydra appending to
    the same file) starts a new session for that file: its earlier events are dropped,
    as the save-time files of the new session would not hold them either (node ids
    restart). ``sessions`` counts every session seen.
    """

    def __init__(self, directory):
        self.dir = None if directory is None else pathlib.Path(directory)
        self._files: dict[str, _FileState] = {}
        self._replays: dict[str, _Replay] = {}
        self._partial: dict[str, bytes] = {}
        self.sessions: dict[str, list[int]] = {}
        self.bad_lines = 0
        self.version = 0  # bumped whenever the replayed state changes
        #: mtime (ns) of every file the last poll read new bytes from
        self.read_mtimes: list[int] = []

    # --- reading ------------------------------------------------------------------------
    def poll(self) -> bool:
        if self.dir is None or not self.dir.is_dir():
            return False
        changed = False
        self.read_mtimes = []
        for path in sorted(self.dir.glob(EVENT_GLOB)):
            changed |= self._read(path)
        if changed:
            self.version += 1
        return changed

    def _reset(self, name: str, path: pathlib.Path, inode: int) -> None:
        self._files[name] = _FileState(path, inode)
        self._replays[name] = _Replay()
        self._partial[name] = b""

    def _read(self, path: pathlib.Path) -> bool:
        name = path.name
        try:
            st = path.stat()
        except OSError:
            return False
        state = self._files.get(name)
        if (
            state is not None
            and state.head
            and (st.st_size != state.offset or st.st_mtime_ns != state.mtime_ns)
        ):
            try:  # a file removed and recreated may get the same inode back
                with open(path, "rb") as f:
                    head = f.read(len(state.head))
            except OSError:
                return False
            if head != state.head:
                state.inode = None
        if state is None or state.inode != st.st_ino or st.st_size < state.offset:
            had = state is not None and state.offset > 0
            self._reset(name, path, st.st_ino)
            state = self._files[name]
            if had:
                logger.warning("%s was replaced: reading it again", path)
        if st.st_size == state.offset:
            return False
        with open(path, "rb") as f:
            f.seek(state.offset)
            data = f.read(st.st_size - state.offset)
        if not state.head:
            state.head = data[:64]
        state.offset += len(data)
        state.mtime_ns = st.st_mtime_ns
        self.read_mtimes.append(st.st_mtime_ns)
        data = self._partial[name] + data
        lines = data.split(b"\n")
        self._partial[name] = lines.pop()  # incomplete last line (b"" if complete)
        changed = False
        for raw in lines:
            if not raw.strip():
                continue
            try:
                ev = json.loads(raw)
            except ValueError:
                self.bad_lines += 1
                ev = _glued_event(raw)
                logger.warning(
                    "%s: skipping malformed event line %r%s",
                    path,
                    raw[:200],
                    ""
                    if ev is None
                    else f" (recovered a {ev.get('ev')!r} event after it)",
                )
                if ev is None:
                    continue
            if not isinstance(ev, dict):
                continue
            if ev.get("ev") == "open":
                if state.session is not None:
                    logger.warning(
                        "%s: new session (Hydra restarted); dropping the events of the "
                        "previous one",
                        path,
                    )
                    self._replays[name] = _Replay()
                state.session = int(ev.get("wall_ns", 0) or 0)
                state.source = ev.get("source")
                if ev.get("robot_id") is not None:
                    state.robot_id = int(ev["robot_id"])
                self.sessions.setdefault(name, []).append(state.session)
            else:
                self._replays[name].apply(ev)
            changed = True
        return changed

    # --- replayed state -----------------------------------------------------------------
    @property
    def has_events(self) -> bool:
        return any(s.offset > 0 for s in self._files.values())

    def robot_id(self) -> int | None:
        for state in self._files.values():
            if state.robot_id is not None:
                return state.robot_id
        return None

    def latest_open(self) -> int | None:
        """``wall_ns`` of the newest "open" line of any file."""
        opens = [s.session for s in self._files.values() if s.session is not None]
        return max(opens) if opens else None

    def session(self) -> int | None:
        """Id of the running Hydra session: ``wall_ns`` of the newest "open" line of the
        frontend log(s), else of any log while no frontend log was opened yet.

        The backend log is opened later in the same run (on its first merge), so its
        "open" line does not start a session; a new frontend "open" (Hydra restarted)
        does."""
        frontend = [
            s.session
            for s in self._files.values()
            if s.session is not None and s.source == "frontend"
        ]
        return max(frontend) if frontend else self.latest_open()

    def session_count(self) -> int:
        """Hydra sessions seen: "open" lines of the frontend log(s), else of any log."""
        frontend = [
            len(self.sessions.get(name, []))
            for name, s in self._files.items()
            if s.source == "frontend"
        ]
        if frontend:
            return sum(frontend)
        return sum(len(v) for v in self.sessions.values())

    def bindings(self) -> dict | None:
        """``track_bindings.json``-shaped data, ``None`` without frontend events."""
        layers: dict[str, list[dict]] = {}
        seen = False
        for name, replay in self._replays.items():
            if self._files[name].source not in (None, "frontend"):
                continue
            for layer, entries in replay.layers.items():
                layers.setdefault(layer, []).extend(dict(e) for e in entries)
                seen = True
            seen |= bool(replay.agents)
        return {"version": 1, "layers": layers} if seen else None

    def merges(self) -> dict[int, int] | None:
        """``merges.json``-shaped child -> final parent map, ``None`` without backend
        events."""
        raw: dict[int, int] = {}
        seen = False
        for name, replay in self._replays.items():
            if self._files[name].source not in (None, "backend"):
                continue
            seen |= self._files[name].session is not None or bool(replay.merges)
            raw.update(replay.merges)
        if not seen:
            return None
        out = {}
        for child, parent in raw.items():  # resolve chains like mergesToJson
            final, steps = parent, 0
            while final in raw and raw[final] != final and steps < len(raw):
                final, steps = raw[final], steps + 1
            out[child] = final
        return out

    def agents(self, prefix: str | None = None):
        """Agent nodes (time order) with their odometric poses."""
        from .codec import node_symbol
        from .indexer import AgentNode

        nodes = []
        for replay in self._replays.values():
            for node_id, (stamp, T) in replay.agents.items():
                if prefix is not None and node_symbol(node_id)[0] != prefix:
                    continue
                nodes.append(
                    AgentNode(node_id, stamp, np.asarray(T, dtype=float).reshape(4, 4))
                )
        nodes.sort(key=lambda n: (n.timestamp_ns, n.node_id))
        return nodes

    def progress(self) -> dict:
        """Per-file read state (recorded in the store meta by the tailer)."""
        return {
            name: {
                "offset": s.offset,
                "inode": s.inode,
                "session": s.session,
                "source": s.source,
                "sessions": len(self.sessions.get(name, [])),
            }
            for name, s in sorted(self._files.items())
        }

    def mtimes(self) -> list[int]:
        return [s.mtime_ns for s in self._files.values() if s.mtime_ns]


def read_events(directory) -> EventLogs:
    """Read every event log of ``directory`` once."""
    logs = EventLogs(directory)
    logs.poll()
    return logs


def events_signature(directory) -> list | None:
    """(name, mtime, size) of every event log, for change detection."""
    if directory is None or not os.path.isdir(directory):
        return None
    out = []
    for path in sorted(pathlib.Path(directory).glob(EVENT_GLOB)):
        try:
            st = path.stat()
        except OSError:
            continue
        out.append([path.name, int(st.st_mtime_ns), int(st.st_size)])
    return out


__all__ = ["EventLogs", "events_signature", "find_events_dir", "read_events"]
