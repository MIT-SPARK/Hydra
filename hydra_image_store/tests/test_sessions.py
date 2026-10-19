import json

import numpy as np

from conftest import AGENT0, make_T
from hydra_image_store import FrameKind, ImageStore, list_sessions
from hydra_image_store.cli import main


def _write_session(root, x, ts):
    """A run-layout directory whose store anchors one keyframe to a(0) at (x, 0, 0)."""
    w = ImageStore.open_writer(root / "image_store")
    sid = w.add_sensor("camera", 8, 6, 5.0, 5.0, 4.0, 3.0)
    fid = w.add_frame_ref(sid, ts, kind=FrameKind.KEYFRAME, world_T_body_odom=make_T(t=(0, 0, 0)),
                          anchor_node_id=AGENT0)
    w.close()
    nodes = [{"id": AGENT0, "layer": 2, "partition": 97,
              "attributes": {"position": [x, 0, 0], "world_R_body": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}}]
    dsg = root / "hydra/backend/dsg.json"
    dsg.parent.mkdir(parents=True, exist_ok=True)
    dsg.write_text(json.dumps({"nodes": nodes, "edges": []}))
    return fid


def _make_run(tmp_path):
    run = tmp_path / "run"
    _write_session(run / "sessions/1", 10.0, 100)
    _write_session(run / "sessions/2", 20.0, 200)
    (run / "sessions/2/session.json").write_text(json.dumps({"session": "2", "rotated_at": "2026-10-18T00:00:00Z"}))
    _write_session(run, 30.0, 300)
    (run / "sessions/notes").mkdir()  # ignored
    (run / "sessions/3").mkdir()  # no store: ignored
    return run


def test_sessions_resolve_against_their_own_graph(tmp_path):
    run = _make_run(tmp_path)
    sessions = ImageStore.sessions(run)
    assert [s.name for s in sessions] == ["1", "2", "live"]
    assert sessions[1].info["rotated_at"] == "2026-10-18T00:00:00Z" and sessions[0].info == {}
    assert sessions[-1].live and sessions[-1].store == run / "image_store"
    assert sessions[0].dsg_path() == run / "sessions/1/hydra/backend/dsg.json"
    assert sessions[0].dsg_path("frontend") is None
    # the same node id a(0) means a different agent in every session
    for session, x in zip(sessions, (10.0, 20.0, 30.0)):
        dsg = session.load_dsg()
        with session.open() as store:
            (frame,) = store.frames(kind=FrameKind.KEYFRAME)
            T, src = store.body_pose(frame, dsg)
        assert src == "anchor"
        np.testing.assert_allclose(T[:3, 3], [x, 0, 0])


def test_sessions_from_store_path_and_without_live(tmp_path):
    run = _make_run(tmp_path)
    assert [s.name for s in list_sessions(run / "image_store")] == ["1", "2", "live"]
    assert [s.name for s in list_sessions(run / "image_store/store.sqlite")] == ["1", "2", "live"]
    assert [s.name for s in ImageStore.sessions(run, include_live=False)] == ["1", "2"]
    assert list_sessions(tmp_path / "missing") == []


def test_python_writer_never_rotates(tmp_path):
    run = tmp_path / "run"
    _write_session(run, 30.0, 300)
    w = ImageStore.open_writer(run / "image_store")
    sid = w.add_sensor("camera", 8, 6, 5.0, 5.0, 4.0, 3.0)
    w.add_frame_ref(sid, 400, kind=FrameKind.OBSERVATION)
    w.close()
    assert not (run / "sessions").exists()
    with ImageStore.open(run / "image_store") as store:
        assert len(store.frames()) == 2


def test_cli_sessions(tmp_path, capsys):
    run = _make_run(tmp_path)
    assert main(["sessions", str(run), "--json"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert [s["session"] for s in out] == ["1", "2", "live"]
    assert out[0]["backend_dsg"].endswith("sessions/1/hydra/backend/dsg.json")
    assert out[2]["store_info"]["frames"] == {"keyframe": 1}
    assert main(["sessions", str(run)]) == 0
    text = capsys.readouterr().out
    assert "session 2:" in text and "rotated at 2026-10-18T00:00:00Z" in text
    assert main(["sessions", str(tmp_path / "missing")]) == 1
