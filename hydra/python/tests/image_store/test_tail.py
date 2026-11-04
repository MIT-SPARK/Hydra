"""Tail mode (live indexing from the event logs) and its equivalence with ``index``.

The run is the indexer tests' reference run (raw_logs.build_run), written step by step
as Hydra would: keyframe archives, track logs and event log lines while it runs, the
save-time files (scene graphs, track_bindings.json, merges.json) at the end.
"""

from __future__ import annotations

import json
import shutil
import sqlite3
import time

import numpy as np
import pytest
from hydra_python.image_store import (
    EXTERNAL_TRACK_BASE,
    FrameKind,
    ImageStore,
    make_node_id,
)
from hydra_python.image_store.cli import main as cli_main
from hydra_python.image_store.events import EventLogs
from hydra_python.image_store.indexer import (
    ensure_indexed,
    index,
    tail_active,
    tail_state,
)
from hydra_python.image_store.tail import Tailer

cv2 = pytest.importorskip("cv2")
pytest.importorskip("yaml")

import raw_logs as R  # noqa: E402

S = R.S
O1, O2, O3, O4, O5, O6 = R.O1, R.O2, R.O3, R.O4, R.O5, R.O6
AGENTS_T = (1.0, 2.0, 3.0, 4.0)


def ts(t):
    return round(t * S)


def agent_id(i, prefix="a"):
    return make_node_id(prefix, i)


# --- event log writers ----------------------------------------------------------------
class Events:
    """Writes event logs the way Hydra's FrontendEventLogger / MergeEventLogger do."""

    def __init__(self, run, robot_id=0):
        self.dir = run / "events"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.robot_id = robot_id
        self.open("frontend")
        self.open("backend")

    def write(self, source, *events, raw=None):
        with open(self.dir / f"{source}.jsonl", "a") as f:
            for ev in events:
                f.write(json.dumps(ev) + "\n")
            if raw is not None:
                f.write(raw)

    def open(self, source):
        self.write(
            source,
            {
                "ev": "open",
                "version": 1,
                "wall_ns": time.time_ns(),
                "source": source,
                "robot_id": self.robot_id,
            },
        )

    def bind(self, stamp, track, node, layer="OBJECTS"):
        self.write(
            "frontend",
            {
                "ev": "bind",
                "stamp_ns": stamp,
                "layer": layer,
                "track_id": track,
                "node_id": node,
            },
        )

    def end(self, stamp, track, node, reason, layer="OBJECTS"):
        self.write(
            "frontend",
            {
                "ev": "end",
                "stamp_ns": stamp,
                "layer": layer,
                "track_id": track,
                "node_id": node,
                "reason": reason,
            },
        )

    def agent(self, i, t, prefix="a"):
        self.write(
            "frontend",
            {
                "ev": "agent",
                "stamp_ns": ts(t),
                "node_id": agent_id(i, prefix),
                "timestamp_ns": ts(t),
                "world_T_body": np.asarray(R.odom(t)).reshape(16).tolist(),
            },
        )

    def merge(self, stamp, child, parent):
        self.write(
            "backend",
            {"ev": "merge", "stamp_ns": stamp, "child": child, "parent": parent},
        )

    def unmerge(self, stamp, child):
        self.write("backend", {"ev": "unmerge", "stamp_ns": stamp, "child": child})


def write_archive(run, t, T=None):
    stamp = ts(t)
    R.write_input_zip(
        run / "keyframes" / f"{stamp}.zip",
        stamp,
        R.odom(t) if T is None else T,
        color=R.color_rgb(),
        depth=R.depth_m(),
    )


def write_tracks(run, final=True):
    """build_run's tracks; ``final=False`` writes track 31's earlier log (another stamp,
    rewritten at the end)."""
    tracks = run / "tracks"
    R.write_track(
        tracks,
        12,
        [
            {"ts": ts(1.5), "T": R.odom(1.5), "kept": True, "score": 0.9},
            {"ts": ts(1.7), "T": R.odom(1.7), "kept": False, "score": 0.3},
        ],
        absorbed=[7, 9],
    )
    R.write_track(
        tracks, 7, [{"ts": ts(1.2), "T": R.odom(1.2), "kept": True, "score": 0.4}]
    )
    R.write_track(
        tracks, 20, [{"ts": ts(2.5), "T": R.odom(2.5), "kept": True}], extracted=False
    )
    R.write_track(tracks, 21, [{"ts": ts(2.6), "T": R.odom(2.6), "kept": False}])
    R.write_track(
        tracks, 30, [{"ts": ts(3.0), "T": R.odom(3.0), "kept": True, "score": 0.8}]
    )
    t31 = 3.1 if final else 3.15
    R.write_track(
        tracks, 31, [{"ts": ts(t31), "T": R.odom(t31), "kept": True, "score": 0.7}]
    )


def write_save_files(run):
    """What Hydra writes at save: identical to build_run's."""
    ref = R.build_run(run.parent / "_ref_files")
    for rel in (
        "hydra/frontend/track_bindings.json",
        "hydra/frontend/dsg.json",
        "hydra/backend/dsg.json",
        "hydra/backend/merges.json",
    ):
        (run / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(ref / rel, run / rel)
    shutil.rmtree(run.parent / "_ref_files")


def live_run(run):
    """Everything Hydra writes during the run (events consistent with build_run's
    save-time files), in order."""
    ev = Events(run)
    steps = []
    steps.append(lambda: write_archive(run, 1.0))
    steps.append(lambda: (ev.agent(0, 1.0), write_archive(run, 1.5)))
    steps.append(
        lambda: (
            ev.bind(1, 12, O1),
            ev.bind(1, 7, O5),
            ev.bind(1, 30, O3),
            ev.bind(1, 31, O4),
            ev.end(1, 7, O5, "deleted"),
            write_tracks(run, final=False),
            write_archive(run, 2.002),
        )
    )
    steps.append(
        lambda: (
            ev.agent(1, 2.0),
            ev.end(2, 30, O3, "track_removed"),
            ev.bind(2, 30, O2),
            ev.merge(2, O2, O6),
            write_archive(run, 3.4),
        )
    )
    steps.append(
        lambda: (
            ev.agent(2, 3.0),
            ev.end(3, 31, O4, "track_removed"),
            ev.merge(3, O4, O2),
            ev.merge(3, O4, O6),  # rewired to the final parent
            write_archive(run, 3.9, T=R.pose(12.0)),
            write_tracks(run, final=True),
        )
    )
    steps.append(lambda: (ev.agent(3, 4.0), write_archive(run, 5.0)))
    return ev, steps


# --- comparison -------------------------------------------------------------------------
def _digest(root, rel):
    import hashlib

    if rel is None:
        return None
    path = root / rel
    return hashlib.md5(path.read_bytes()).hexdigest() if path.exists() else "missing"


def dump(store_dir, skip_external=True):
    """Store content keyed by (sensor, stamp) / (track, stamp): ids and file names differ
    between stores, nothing else may."""
    conn = sqlite3.connect(f"file:{store_dir / 'store.sqlite'}?mode=ro", uri=True)
    sensors = dict(conn.execute("SELECT sensor_id, name FROM sensors"))
    frames, by_id = {}, {}
    for fid, sid, t, kind, odom, anchor, rel, rgb, depth in conn.execute(
        "SELECT frame_id, sensor_id, timestamp_ns, kind, world_T_body_odom, anchor_node_id, "
        "anchor_T_body, rgb_path, depth_path FROM frames"
    ):
        key = (sensors[sid], t)
        by_id[fid] = key
        frames[key] = (
            kind,
            anchor,
            None if rel is None else np.round(np.frombuffer(rel, float), 9).tolist(),
            np.round(np.frombuffer(odom, float), 9).tolist(),
            _digest(store_dir, rgb),
            _digest(store_dir, depth),
        )
    obs = sorted(
        (track, by_id[fid], kept, round(score, 6), rle, mask is not None)
        for track, fid, kept, score, rle, mask in conn.execute(
            "SELECT track_id, frame_id, kept, score, mask_rle, mask_path FROM observations"
        )
        if not (skip_external and track >= EXTERNAL_TRACK_BASE)
    )
    if skip_external:
        external_frames = {o[1] for o in obs}
        external_frames = {
            by_id[r[0]]
            for r in conn.execute(
                "SELECT frame_id FROM observations WHERE track_id >= ?",
                (EXTERNAL_TRACK_BASE,),
            )
        } - external_frames
        frames = {k: v for k, v in frames.items() if k not in external_frames}
    bindings = sorted(
        r
        for r in conn.execute("SELECT robot_id, track_id, node_id FROM track_bindings")
        if not (skip_external and r[1] >= EXTERNAL_TRACK_BASE)
    )
    aliases = sorted(conn.execute("SELECT child_id, parent_id FROM node_aliases"))
    meta = dict(conn.execute("SELECT key, value FROM meta"))
    conn.close()
    return {
        "frames": frames,
        "observations": obs,
        "bindings": bindings,
        "aliases": aliases,
        "unbound": json.loads(meta.get("indexer.r0.unbound_tracks", "{}")),
        "pending": json.loads(meta.get("indexer.r0.pending_frames", "[]")),
    }


def batch_reference(run, tmp_path):
    """A one-shot ``index`` of a copy of the finished run (without its store)."""
    ref = tmp_path / "batch_run"
    shutil.copytree(run, ref, ignore=shutil.ignore_patterns("image_store"))
    index(ref)
    return dump(ref / "image_store")


# --- tests ------------------------------------------------------------------------------
def test_event_logs_replay_and_partial_lines(tmp_path):
    run = tmp_path / "run"
    ev = Events(run)
    ev.bind(1, 30, O3)
    ev.agent(0, 1.0)
    ev.write("frontend", raw='{"ev": "bind", "stamp_ns": 2, "layer": "OBJ')  # mid-write
    logs = EventLogs(run / "events")
    assert logs.poll()
    assert logs.robot_id() == 0
    assert [e["node_id"] for e in logs.bindings()["layers"]["OBJECTS"]] == [O3]
    assert [a.node_id for a in logs.agents()] == [agent_id(0)]
    # the rest of the line arrives
    ev.write("frontend", raw='ECTS", "track_id": 31, "node_id": %d}\n' % O4)
    ev.end(2, 30, O3, "track_removed")
    ev.merge(2, O2, O6)
    ev.merge(3, O6, O1)  # chain: O2 -> O6 -> O1
    assert logs.poll()
    entries = logs.bindings()["layers"]["OBJECTS"]
    assert [(e["track_id"], e["active"], e["end"]) for e in entries] == [
        (30, False, "track_removed"),
        (31, True, None),
    ]
    assert logs.merges() == {O2: O1, O6: O1}
    ev.unmerge(4, O6)
    logs.poll()
    assert logs.merges() == {O2: O6}
    assert not logs.poll()  # nothing new
    assert logs.bad_lines == 0
    assert logs.progress()["frontend.jsonl"]["offset"] > 0


def test_event_logs_new_session_and_replaced_file(tmp_path):
    run = tmp_path / "run"
    ev = Events(run)
    ev.bind(1, 30, O3)
    ev.merge(1, O2, O6)
    logs = EventLogs(run / "events")
    logs.poll()
    ev.open("frontend")  # Hydra restarted, appending: node ids start over
    ev.bind(5, 40, O1)
    logs.poll()
    assert [e["track_id"] for e in logs.bindings()["layers"]["OBJECTS"]] == [40]
    assert logs.sessions["frontend.jsonl"].__len__() == 2
    assert logs.merges() == {O2: O6}  # the backend log is another file
    (run / "events" / "backend.jsonl").unlink()  # e.g. an output directory wipe
    Events.open(ev, "backend")
    ev.merge(6, O4, O1)
    logs.poll()
    assert logs.merges() == {O4: O1}


def test_tail_live_then_final_equals_batch(tmp_path):
    run = tmp_path / "run"
    ev, steps = live_run(run)
    tailer = Tailer(run, settle_s=0.0)
    store_dir = run / "image_store"

    steps[0]()
    stats = tailer.poll()
    assert stats["keyframes_indexed"] == 1 and stats["frames_pending"] == 1
    assert tail_active(store_dir)
    # ImageStore.open leaves a store that is being tailed alone (no re-index)
    assert ensure_indexed(run) == store_dir

    steps[1]()
    tailer.poll()
    with ImageStore.open(store_dir, index=False) as store:
        (f,) = [f for f in store.frames() if f.timestamp_ns == ts(1.0)]
        assert f.kind == FrameKind.KEYFRAME and f.anchor_node_id == agent_id(0)
        # 1.5 s is after the newest agent node (1.0 s): anchored to it for now
        (f,) = [f for f in store.frames() if f.timestamp_ns == ts(1.5)]
        assert f.anchor_node_id == agent_id(0)
    assert tail_state(store_dir)["live"]

    steps[2]()
    tailer.poll()
    with ImageStore.open(store_dir, index=False) as store:
        # object images during the run: track 12 (and 7, absorbed into it) -> O1
        assert {o.track_id for o in store.observations(O1)} == {12, 7}
        assert store.tracks(O4)  # track 31 bound to O4 for now
        # 1.5 s now has agent 1 (2.0 s) after it: decided for good, still agent 0
        (f,) = [f for f in store.frames() if f.timestamp_ns == ts(1.5)]
        assert f.anchor_node_id == agent_id(0)

    # another writer adds its own frame and observation mid-run
    node = make_node_id("O", 77)
    track = EXTERNAL_TRACK_BASE | 77
    with ImageStore.open_writer(store_dir, robot_id=0) as w:
        sid = w.reader().sensors()[0].sensor_id
        src = run / "external.jpg"
        cv2.imwrite(str(src), np.zeros((12, 16, 3), np.uint8))
        fid = w.add_frame_ref(sid, ts(9.0), kind=FrameKind.OBSERVATION, rgb=src)
        with w.batch():
            w.bind_track(track, node)
            w.add_observation(track, fid, score=0.8, kept=True)

    for step in steps[3:]:
        step()
        tailer.poll()
    with ImageStore.open(store_dir, index=False) as store:
        assert store.canonical(O2) == O6 and store.canonical(O4) == O6
        # merged nodes gather the images of every track
        assert {o.track_id for o in store.observations(O6)} == {30, 31}
        # track 31's first log (3.15 s) was rewritten: its frame is gone
        assert not [f for f in store.frames() if f.timestamp_ns == ts(3.15)]

    # Hydra saves: reconcile
    write_save_files(run)
    stats = tailer.poll()
    assert stats["final"] and tailer.finalized
    state = tail_state(store_dir)
    assert not state["live"] and state["final"]
    assert not tail_active(store_dir)
    assert ensure_indexed(run) == store_dir  # signature recorded: nothing to redo

    tailed = dump(store_dir)
    reference = batch_reference(run, tmp_path)
    assert tailed == reference
    with ImageStore.open(store_dir, index=False) as store:  # the external rows survived
        (o,) = store.observations(node)
        assert o.track_id == track
        assert o.frame.anchor_node_id is None
    assert tailer.summary()["lag"]["keyframes"]["n"] == 6


def test_tail_track_logged_before_its_archive(tmp_path):
    """An observation frame indexed before the keyframe archive of the same stamp takes
    the archive's images (as when the archive comes first)."""
    run = tmp_path / "run"
    ev, steps = live_run(run)
    tailer = Tailer(run, settle_s=0.0)
    write_tracks(
        run, final=False
    )  # track 12's kept observation at 1.5 s, no archive yet
    tailer.poll()
    for step in steps:
        step()
        tailer.poll()
    write_save_files(run)
    assert tailer.poll()["final"]
    assert dump(run / "image_store") == batch_reference(run, tmp_path)


def test_tail_waits_for_robot_id(tmp_path):
    run = tmp_path / "run"
    write_archive(run, 1.0)
    tailer = Tailer(run)
    assert tailer.poll() == {"waiting": "robot_id"}
    assert not (run / "image_store").exists()
    Events(run, robot_id=2)
    tailer.poll()
    with ImageStore.open(run / "image_store", index=False) as store:
        assert {f.robot_id for f in store.frames()} == {2}


def test_session_is_the_frontend_open(tmp_path):
    """The backend log is opened at the first merge, later in the same run: that does
    not start a session (consumers would think Hydra restarted). A new frontend open
    (Hydra restarted, appending) does."""
    run = tmp_path / "run"
    write_archive(run, 1.0)
    ev = Events.__new__(Events)  # open the frontend log only, as Hydra does at start
    ev.dir, ev.robot_id = run / "events", 0
    ev.dir.mkdir(parents=True)
    ev.open("frontend")
    ev.agent(0, 1.0)
    tailer = Tailer(run, settle_s=0.0)
    tailer.poll()
    first = tail_state(run / "image_store")
    assert first["session"] is not None and first["sessions"] == 1
    time.sleep(0.01)
    ev.open("backend")  # first backend merge
    ev.merge(2, O2, O6)
    tailer.poll()
    state = tail_state(run / "image_store")
    assert (state["session"], state["sessions"]) == (first["session"], 1)
    assert tailer.events.latest_open() > first["session"]
    time.sleep(0.01)
    ev.open("frontend")  # Hydra restarted
    ev.agent(0, 1.0)
    tailer.poll()
    state = tail_state(run / "image_store")
    assert state["session"] > first["session"] and state["sessions"] == 2


def test_tail_ignores_save_files_of_an_earlier_session(tmp_path):
    run = tmp_path / "run"
    write_archive(run, 1.0)
    write_save_files(run)  # left over from an earlier session
    time.sleep(0.01)
    ev = Events(run)
    ev.agent(0, 1.0)
    tailer = Tailer(run, settle_s=0.0)
    assert not tailer.poll()["final"]
    write_save_files(run)
    assert tailer.poll()["final"]


def test_tail_settles_before_reconciling(tmp_path):
    run = tmp_path / "run"
    ev, steps = live_run(run)
    for step in steps:
        step()
    write_save_files(run)
    tailer = Tailer(run, settle_s=0.3)
    assert not tailer.poll()["final"]
    time.sleep(0.35)
    assert tailer.poll()["final"]


def test_batch_index_uses_events_after_a_crash(tmp_path):
    """No save-time files (Hydra crashed): ``index`` takes bindings, merges and agent
    nodes from the event logs."""
    run = tmp_path / "run"
    ev, steps = live_run(run)
    for step in steps:
        step()
    stats = index(run, final=False)
    assert stats["track_bindings"] == 5 and stats["node_aliases"] == 2  # as at save
    with ImageStore.open(run / "image_store", index=False) as store:
        assert {o.track_id for o in store.observations(O6)} == {30, 31}
        (f,) = [f for f in store.frames() if f.timestamp_ns == ts(2.002)]
        assert f.kind == FrameKind.KEYFRAME and f.anchor_node_id == agent_id(1)


def test_tail_equals_index_on_events_only(tmp_path):
    """Live passes end in the same state as a one-shot index of the same (crashed) run,
    except the anchors still waiting for an agent node after them."""
    run = tmp_path / "run"
    ev, steps = live_run(run)
    tailer = Tailer(run, finalize=False)
    for step in steps:
        step()
        tailer.poll()
    tailed = dump(run / "image_store")
    ref = tmp_path / "batch_run"
    shutil.copytree(run, ref, ignore=shutil.ignore_patterns("image_store"))
    index(ref, final=False)
    reference = dump(ref / "image_store")
    assert tailed["bindings"] == reference["bindings"]
    assert tailed["aliases"] == reference["aliases"]
    assert tailed["observations"] == reference["observations"]
    assert tailed["unbound"] == reference["unbound"]
    late = {("camera", ts(5.0))}  # after the newest agent node: provisional when tailed

    def strip(d):
        return {k: v for k, v in d["frames"].items() if k not in late}

    assert strip(tailed) == strip(reference)


def test_cli_tail(tmp_path, capsys):
    run = tmp_path / "run"
    ev, steps = live_run(run)
    for step in steps:
        step()
    write_save_files(run)
    assert (
        cli_main(
            ["tail", str(run), "--interval", "0.05", "--settle", "0", "--timeout", "30"]
        )
        == 0
    )
    out = capsys.readouterr().out
    summary = json.loads(out[out.index("\n{") + 1 :])  # after the final pass line
    assert summary["finalized"] and summary["final_stats"]["final"]
    assert cli_main(["check", str(run)]) == 0


def test_open_live(tmp_path):
    run = tmp_path / "run"
    ev, steps = live_run(run)
    for step in steps[:3]:
        step()
    with ImageStore.open(run, live=True, live_interval_s=0.05) as store:
        assert {o.track_id for o in store.observations(O1)} == {12, 7}
        for step in steps[3:]:
            step()
        deadline = time.time() + 10
        while time.time() < deadline and store.canonical(O4) != O6:
            time.sleep(0.05)
        assert store.canonical(O4) == O6
    assert not tail_active(run / "image_store")


# --- concurrent readers / crash leftovers (bag recovery tests, 2026-11-03) -------------
def _unmarked_unanchored(store_dir, ignore=()):
    """Indexer-made frames a reader sees without an anchor that the indexer does not
    list as undecided (``pending_frames`` / ``provisional_frames``)."""
    conn = sqlite3.connect(f"file:{store_dir / 'store.sqlite'}?mode=ro", uri=True)
    try:
        meta = dict(conn.execute("SELECT key, value FROM meta"))
        undecided = set(json.loads(meta.get("indexer.r0.pending_frames", "[]")))
        undecided |= set(json.loads(meta.get("indexer.r0.provisional_frames", "[]")))
        rows = conn.execute(
            "SELECT frame_id, timestamp_ns FROM frames WHERE anchor_node_id IS NULL AND "
            "(kind = ? OR frame_id IN (SELECT frame_id FROM observations WHERE track_id < ?))",
            (int(FrameKind.KEYFRAME), EXTERNAL_TRACK_BASE),
        ).fetchall()
    finally:
        conn.close()
    return [(fid, t) for fid, t in rows if fid not in undecided and t not in ignore]


def test_tail_readers_never_see_undecided_frames_unmarked(tmp_path, monkeypatch):
    """Every commit of a tail pass (live and the reconciliation) leaves each frame
    either anchored or listed as pending / provisional. A store mirror holds the
    frames from the lowest undecided one on; a frame committed before it was anchored
    and not listed would be sent without its anchor (or, if it is anchored later,
    skipped)."""
    import contextlib

    from hydra_python.image_store.store import ImageStoreWriter

    run = tmp_path / "run"
    ev, steps = live_run(run)
    store_dir = run / "image_store"
    far = {ts(3.9)}  # too far from every agent node: decided "no anchor"
    seen: list = []
    orig = ImageStoreWriter.batch

    @contextlib.contextmanager
    def checked(self):
        with orig(self):
            yield self
        if self._batch_depth == 0 and (store_dir / "store.sqlite").exists():
            seen.extend(_unmarked_unanchored(store_dir, far))

    monkeypatch.setattr(ImageStoreWriter, "batch", checked)
    tailer = Tailer(run, settle_s=0.0)
    for step in steps:
        step()
        tailer.poll()
    write_save_files(run)
    assert tailer.poll()["final"]
    assert seen == []
    monkeypatch.setattr(ImageStoreWriter, "batch", orig)
    assert dump(store_dir) == batch_reference(run, tmp_path)


def test_event_logs_restart_after_a_cut_line(tmp_path):
    """Hydra killed in the middle of a line, restarted appending: its ``open`` line is
    glued to the fragment. The new session must still start there."""
    run = tmp_path / "run"
    ev = Events(run)
    ev.bind(1, 30, O3)
    ev.agent(0, 1.0)
    ev.write("frontend", raw='{"ev": "agent", "stamp_ns": 5, "node_id": 12, "world_T')
    ev.open("frontend")  # the restarted Hydra
    ev.bind(5, 40, O1)
    logs = EventLogs(run / "events")
    logs.poll()
    assert logs.bad_lines == 1
    assert logs.session_count() == 2
    assert [e["track_id"] for e in logs.bindings()["layers"]["OBJECTS"]] == [40]
    assert logs.agents() == []


def test_unreadable_archive_is_not_reread_until_it_changes(tmp_path, caplog):
    run = tmp_path / "run"
    ev, steps = live_run(run)
    steps[0]()
    bad = run / "keyframes" / f"{ts(1.2)}.zip"
    good = (run / "keyframes" / f"{ts(1.0)}.zip").read_bytes()
    bad.write_bytes(good[: len(good) // 2])  # half written (crash leftover)
    tailer = Tailer(run, settle_s=0.0)
    assert tailer.poll()["keyframes_failed"] == 1
    caplog.clear()
    stats = tailer.poll()
    assert stats["keyframes_failed"] == 1 and not stats.get("keyframes_indexed")
    assert "cannot read keyframe archive" not in caplog.text
    write_archive(run, 1.2)  # replaced by a complete archive: indexed
    assert tailer.poll()["keyframes_indexed"] == 1


def test_tailer_killed_hard_is_not_active(tmp_path):
    """A tailer killed with SIGKILL leaves ``live: true`` and a fresh heartbeat: a
    restarted tailer must not wait for the heartbeat to get stale (30 s) on this host."""
    import os
    import subprocess
    import sys

    run = tmp_path / "run"
    ev, steps = live_run(run)
    steps[0]()
    code = (
        "import sys; from hydra_python.image_store.tail import Tailer; "
        "Tailer(sys.argv[1], settle_s=0.0).poll()"
    )
    subprocess.run([sys.executable, "-c", code, str(run)], check=True, env=os.environ)
    state = tail_state(run / "image_store")
    assert state["live"] and state["pid"] != os.getpid()
    assert not tail_active(run / "image_store")  # its process is gone
    tailer = Tailer(run, settle_s=0.0)
    tailer.poll()
    assert tail_active(run / "image_store")  # this process tails now


def test_cli_tail_refuses_a_second_tailer(tmp_path, capsys):
    run = tmp_path / "run"
    ev, steps = live_run(run)
    steps[0]()
    Tailer(run, settle_s=0.0).poll()  # live, this process
    assert cli_main(["tail", str(run), "--once"]) == 0
    assert "another tailer follows" in capsys.readouterr().out
    assert tail_state(run / "image_store")["polls"] == 0  # untouched


def _undecided_unlisted(store_dir):
    """Timestamps of frames a reader would take as settled although they have no anchor:
    unanchored and not in any ``pending_frames`` / ``provisional_frames`` list."""
    conn = sqlite3.connect(f"file:{store_dir / 'store.sqlite'}?mode=ro", uri=True)
    try:
        listed = set()
        for (value,) in conn.execute(
            "SELECT value FROM meta WHERE key LIKE 'indexer.r%.pending_frames' "
            "OR key LIKE 'indexer.r%.provisional_frames'"
        ):
            listed |= set(json.loads(value))
        return {
            t
            for fid, t in conn.execute(
                "SELECT frame_id, timestamp_ns FROM frames WHERE anchor_node_id IS NULL"
            )
            if fid not in listed
        }
    finally:
        conn.close()


def test_readers_never_see_undecided_frames_as_settled_through_reconciliation(
    tmp_path, monkeypatch
):
    """After every committed transaction of the tailer, a frame without an anchor is
    either listed as pending / provisional or stays unanchored for good (too far from
    any agent node, another writer's frame): readers such as a store mirror send every
    unlisted frame below the lowest listed one as settled. Checked at the two windows
    where a frame could otherwise be visible unlisted: between its creation and the
    anchoring pass, and between the reset of the live anchors and the pending list of
    the final reconciliation."""
    import contextlib

    from hydra_python.image_store.store import ImageStoreWriter

    run = tmp_path / "run"
    store_dir = run / "image_store"
    seen: set[int] = set()
    orig = ImageStoreWriter.batch

    @contextlib.contextmanager
    def checked(self):
        top = self._batch_depth == 0
        with orig(self):
            yield self
        if top and (store_dir / "store.sqlite").exists():
            seen.update(_undecided_unlisted(store_dir))

    monkeypatch.setattr(ImageStoreWriter, "batch", checked)
    ev, steps = live_run(run)
    tailer = Tailer(run, settle_s=0.0)
    for step in steps:
        step()
        tailer.poll()
    write_save_files(run)
    assert tailer.poll()["final"]
    final_unanchored = _undecided_unlisted(store_dir)
    assert seen, "the hook saw no transaction"
    assert seen <= final_unanchored, sorted(seen - final_unanchored)
