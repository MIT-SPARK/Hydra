"""Live ("tail") indexing of a run while Hydra/Khronos are still writing it.

:class:`Tailer` follows a run directory and keeps its ``image_store/`` up to date
(FORMAT.md §6 "Tail mode"):

- new keyframe archives (only ``*.zip``; Hydra writes ``*.zip.tmp.*`` and renames) and
  new or rewritten ``track.json`` files are indexed as by :func:`indexer.index`;
- the live event logs give the track -> node bindings, the node merges (aliases) and the
  agent nodes; frames are anchored to the nearest agent node known so far, frames newer
  than the newest agent node are anchored provisionally and decided again when the next
  agent node arrives, frames before any agent node stay pending;
- when the save-time files appear (backend ``dsg.json``, ``track_bindings.json``,
  ``merges.json``, newer than the running session and unchanged for ``settle_s``), the
  store is reconciled: every frame the indexer made is anchored again from scratch and
  bindings / aliases come from the saved files, so the store ends equal to a one-shot
  ``index`` of the run.

The store stays readable by other processes throughout (SQLite WAL; every pass is a few
short transactions). Rows other writers add (tracks from
:data:`~hydra_python.image_store.codec.EXTERNAL_TRACK_BASE` up, their frames) are left alone. The tailer records its state in the meta key ``indexer.tail`` (see
:func:`indexer.tail_state`); while its heartbeat is fresh, ``ImageStore.open`` does not
re-index the store itself.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import os
import socket
import threading
import time

import numpy as np

from .codec import EXTERNAL_TRACK_BASE
from .events import EventLogs
from .indexer import (
    INDEXER_VERSION,
    SIGNATURE_KEY,
    TAIL_KEY,
    _index,
    _Indexer,
    _locked,
    _logged_robot,
    _previous_robot,
    find_inputs,
    infer_robot_id,
    input_signature,
    robot_prefix,
)
from .store import FrameKind, ImageStoreWriter

logger = logging.getLogger(__name__)


def _percentiles(values: list[float]) -> dict | None:
    if not values:
        return None
    arr = np.asarray(values, dtype=float)
    return {
        "n": int(arr.size),
        "median_s": round(float(np.median(arr)), 3),
        "p95_s": round(float(np.percentile(arr, 95)), 3),
        "max_s": round(float(arr.max()), 3),
    }


class Tailer:
    """Incrementally index a run that is still being written.

    Arguments are those of :func:`indexer.index` (plus ``events``, the event log
    directory, default ``<run>/events`` or ``<run>/hydra/events``). ``settle_s`` is how
    long the save-time files must stay unchanged before the final reconciliation;
    ``finalize=False`` never reconciles (live passes only).

    Use :meth:`poll` for one pass, :meth:`run` to loop until the run is finalized, or
    :meth:`start` / :meth:`stop` for a background thread.
    """

    def __init__(
        self,
        run=None,
        *,
        keyframes=None,
        tracks=None,
        hydra=None,
        events=None,
        store=None,
        dsg=None,
        frontend_dsg=None,
        robot_id: int | None = None,
        agent_prefix: str | None = None,
        max_anchor_dist_m: float = 2.0,
        jpeg_quality: int = 90,
        link: str = "hardlink",
        settle_s: float = 2.0,
        finalize: bool = True,
    ):
        self._where = dict(
            keyframes=keyframes,
            tracks=tracks,
            hydra=hydra,
            store=store,
            backend_dsg=dsg,
            frontend_dsg=frontend_dsg,
            events=events,
        )
        self.run_dir = run
        self.inputs = find_inputs(run, **self._where)
        self.store = self.inputs.store
        self.events = EventLogs(self.inputs.events)
        self.robot_id = robot_id
        self.agent_prefix = agent_prefix
        self.index_opts = dict(
            max_anchor_dist_m=float(max_anchor_dist_m),
            jpeg_quality=int(jpeg_quality),
            link=link,
        )
        self.settle_s = float(settle_s)
        self.finalize = finalize
        self.finalized = False
        self.final_stats: dict | None = None
        self.polls = 0
        self.lags: dict[str, list[float]] = {
            "keyframes": [],
            "tracks": [],
            "events": [],
        }
        self.poll_times: list[float] = []
        self._final_sig = None
        self._final_since = 0.0
        self._bind_key = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    # --- inputs ---------------------------------------------------------------------------
    def _resolve(self):
        inputs = find_inputs(self.run_dir, **self._where)
        if inputs.events != self.events.dir:  # the events directory appeared
            self.events = EventLogs(inputs.events)
        self.inputs = inputs
        return inputs

    def _robot(self, inputs) -> int | None:
        if self.robot_id is not None:
            return self.robot_id
        if self.agent_prefix is not None:
            return infer_robot_id(inputs, self.agent_prefix)
        for candidate in (
            self.events.robot_id(),
            _logged_robot(inputs.tracks),
            _previous_robot(inputs.store),
        ):
            if candidate is not None:
                self.robot_id = candidate
                return candidate
        if inputs.backend_dsg is not None or inputs.frontend_dsg is not None:
            self.robot_id = infer_robot_id(inputs)
            return self.robot_id
        return None  # wait: guessing 0 would namespace the ids of another robot wrongly

    def _opts(self, robot: int) -> dict:
        return {
            "agent_prefix": self.agent_prefix
            if self.agent_prefix is not None
            else robot_prefix(robot),
            "max_anchor_dist_m": self.index_opts["max_anchor_dist_m"],
            "jpeg_quality": self.index_opts["jpeg_quality"],
            "link": self.index_opts["link"],
        }

    def _save_files_ready(self, inputs) -> bool:
        """Whether the save-time files of the running session exist and settled."""
        if not self.finalize:
            return False
        files = (inputs.backend_dsg, inputs.bindings, inputs.merges)
        if any(f is None for f in files):
            self._final_sig = None
            return False
        try:
            mtimes = [f.stat().st_mtime_ns for f in files]
        except OSError:
            return False
        opened = self.events.latest_open()
        if opened is not None and min(mtimes) < opened:
            return False  # left over from an earlier session
        sig = input_signature(inputs)
        now = time.time()
        if sig != self._final_sig:
            self._final_sig = sig
            self._final_since = now
            return self.settle_s <= 0
        return now - self._final_since >= self.settle_s

    # --- passes ---------------------------------------------------------------------------
    def poll(self) -> dict:
        """One pass. Returns its statistics (``final`` is set by the reconciliation)."""
        t0 = time.time()
        inputs = self._resolve()
        events_changed = self.events.poll()
        robot = self._robot(inputs)
        if robot is None:
            return {"waiting": "robot_id"}
        if self.finalized:
            return {"final": True, "skipped": True}
        if self._save_files_ready(inputs):
            stats = self._reconcile(inputs, robot)
        else:
            stats = self._live(inputs, robot, events_changed)
        self.polls += 1
        self.poll_times.append(time.time() - t0)
        return stats

    def _tail_meta(self, writer: ImageStoreWriter, live: bool) -> None:
        state = {
            "live": live,
            "final": not live and self.finalized,
            "heartbeat_wall": time.time(),
            # who tails: a tailer killed hard (no "live": false written) is recognised
            # as gone at once on this host, not only when its heartbeat gets stale
            "pid": os.getpid(),
            "host": socket.gethostname(),
            "robot_id": writer.robot_id,
            "session": self.events.session(),
            "sessions": self.events.session_count(),
            "events": self.events.progress(),
            "polls": self.polls,
        }
        writer.set_meta(TAIL_KEY, json.dumps(state, separators=(",", ":")))

    def _live(self, inputs, robot: int, events_changed: bool) -> dict:
        live_inputs = dataclasses.replace(
            inputs, backend_dsg=None, frontend_dsg=None, bindings=None, merges=None
        )
        opts = self._opts(robot)
        with _locked(self.store):
            writer = ImageStoreWriter(self.store, robot)
            try:
                ix = _Indexer(writer, live_inputs, opts)
                ix.pending_on_create = True
                ix.events = self.events if self.events.has_events else None
                ix.index_keyframes()
                tracks = ix.index_tracks()
                bind_key = (self.events.version, ix.stats["tracks_indexed"] > 0)
                if bind_key != self._bind_key or ix.stats["tracks_indexed"]:
                    ix.index_bindings(tracks)
                    self._bind_key = (self.events.version, False)
                agents = self.events.agents(opts["agent_prefix"])
                ix.anchor(
                    agents or None, final=False, from_frontend=True, provisional=True
                )
                if self.polls == 0:
                    writer.set_meta("indexer_version", INDEXER_VERSION)
                    ix.set_meta("sources", {**live_inputs.to_json(), **opts})
                self._tail_meta(writer, live=True)
            finally:
                writer.close()
        self._record_lags(ix, time.time(), events_changed)
        stats = dict(sorted(ix.stats.items()))
        stats["agents"] = len(agents)
        stats["final"] = False
        if ix.warnings:
            stats["warnings"] = ix.warnings
        return stats

    def _record_lags(self, ix: _Indexer, now: float, events_changed: bool) -> None:
        n_kf = ix.stats["keyframes_indexed"]
        for i, m in enumerate(ix.source_mtimes):
            kind = "keyframes" if i < n_kf else "tracks"
            self.lags[kind].append(now - m / 1e9)
        if events_changed:  # files that grew in this pass (their last write)
            for m in self.events.read_mtimes:
                self.lags["events"].append(now - m / 1e9)

    def _owned_frames(self, conn, robot: int) -> set[int]:
        """Frames the indexer made: keyframes and frames of Khronos tracks (not those
        other writers added on their own)."""
        rows = conn.execute(
            "SELECT frame_id FROM frames WHERE robot_id = ? AND kind = ? UNION "
            "SELECT frame_id FROM observations WHERE robot_id = ? AND track_id < ?",
            (robot, int(FrameKind.KEYFRAME), robot, EXTERNAL_TRACK_BASE),
        )
        return {r[0] for r in rows}

    def _reconcile(self, inputs, robot: int) -> dict:
        """Final pass on the save-time files: equal to a one-shot ``index``."""
        logger.info("save-time files found: reconciling %s", self.store)
        signature = input_signature(inputs)
        with _locked(self.store):
            writer = ImageStoreWriter(self.store, robot)
            try:
                owned = self._owned_frames(writer.conn, robot)
            finally:
                writer.close()
            # every anchor decided live is decided again, as a one-shot index would,
            # in one transaction with the new decisions (readers never see them unset)
            stats = _index(
                inputs,
                robot_id=robot,
                agent_prefix=self.agent_prefix,
                final=True,
                signature=signature,
                reanchor=owned,
                **self.index_opts,
            )
            self.finalized = True
            writer = ImageStoreWriter(self.store, robot)
            try:
                self._tail_meta(writer, live=False)
            finally:
                writer.close()
        stats["final"] = True
        stats["reanchored"] = len(owned)
        self.final_stats = stats
        return stats

    # --- loops ----------------------------------------------------------------------------
    def run(
        self,
        interval_s: float = 1.0,
        *,
        until_final: bool = True,
        timeout_s: float | None = None,
        on_poll=None,
    ) -> dict | None:
        """Poll every ``interval_s`` until the run is finalized (``until_final``), the
        timeout expires or :meth:`stop` is called. Returns the final statistics."""
        start = time.time()
        while not self._stop.is_set():
            try:
                stats = self.poll()
            except Exception:  # noqa: BLE001 - keep tailing; the next pass retries
                logger.exception("tail pass failed")
                stats = {"error": True}
            if on_poll is not None:
                on_poll(stats)
            if until_final and self.finalized:
                break
            if timeout_s is not None and time.time() - start > timeout_s:
                break
            self._stop.wait(interval_s)
        if not self.finalized:
            self._mark_stopped()
        return self.final_stats

    def _mark_stopped(self) -> None:
        """Record that nobody tails the store any more (ImageStore.open may index)."""
        robot = self.robot_id
        if robot is None or not (self.store / "store.sqlite").exists():
            return
        try:
            with _locked(self.store):
                writer = ImageStoreWriter(self.store, robot)
                try:
                    self._tail_meta(writer, live=False)
                    # the live state is not a one-shot index: let ensure_indexed redo it
                    writer.conn.execute(
                        "DELETE FROM meta WHERE key = ?", (SIGNATURE_KEY,)
                    )
                finally:
                    writer.close()
        except Exception:  # noqa: BLE001
            logger.exception("could not mark the tail as stopped")

    def start(self, interval_s: float = 1.0) -> "Tailer":
        """Tail in a background thread (stops after the reconciliation)."""
        self._stop.clear()
        self._thread = threading.Thread(
            target=self.run, args=(interval_s,), name="image-store-tail", daemon=True
        )
        self._thread.start()
        return self

    def stop(self, timeout: float | None = 10.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    def summary(self) -> dict:
        """Lag (raw input written -> committed to the store) and pass statistics."""
        return {
            "polls": self.polls,
            "finalized": self.finalized,
            "lag": {k: _percentiles(v) for k, v in self.lags.items()},
            "poll_time": _percentiles(self.poll_times),
            "event_sessions": {k: len(v) for k, v in self.events.sessions.items()},
            "bad_event_lines": self.events.bad_lines,
        }


def tail(
    run=None, *, interval_s: float = 1.0, timeout_s: float | None = None, **kwargs
):
    """Tail ``run`` until it is finalized (see :class:`Tailer`); returns the summary."""
    t = Tailer(run, **kwargs)
    t.run(interval_s, timeout_s=timeout_s)
    return {**t.summary(), "final_stats": t.final_stats}


__all__ = ["Tailer", "tail"]
