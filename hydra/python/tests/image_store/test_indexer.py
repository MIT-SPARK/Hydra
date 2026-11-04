"""Indexer: raw Hydra/Khronos image logs (FORMAT.md) -> image store.

Reference run (raw_logs.build_run), robot 0 / prefix ``a``; odometry moves +x at 1 m/s
from x=0 at t=1 s; backend agent poses are odometry shifted 1 m in y:

- agent nodes a0..a3 at t = 1, 2, 3, 4 s
- keyframe archives at 1.0 (-> a0), 1.5 (tie -> a0), 2.002 (-> a1), 3.4 (-> a2), 3.9
  with a 12 m odometry jump (unanchored) and 5.0 (after the newest node -> a3 when
  final)
- tracks: 12 absorbed 7 and 9 -> O1; 20 extraction failed; 21 never bound; 30 rebound
  O3 -> O2; 31 bound to O4; backend merges O2 -> O6 and O4 -> O6
"""

import json
import os
import pathlib
import shutil

import imageio.v3 as iio
import numpy as np
import pytest
from hydra_python.image_store import (
    EXTERNAL_TRACK_BASE,
    FrameKind,
    ImageStore,
    make_id,
    make_node_id,
)
from hydra_python.image_store.cli import main as cli_main
from hydra_python.image_store.indexer import index, is_indexed, load_agent_nodes

cv2 = pytest.importorskip("cv2")
pytest.importorskip("yaml")

import raw_logs as R  # noqa: E402 (after the importorskip calls)

A = [make_node_id("a", i) for i in range(4)]


@pytest.fixture
def run(tmp_path):
    return R.build_run(tmp_path / "run")


@pytest.fixture
def indexed(run):
    stats = index(run)
    return run, stats


def ts(t):
    return round(t * R.S)


def frame_at(store, t):
    (f,) = [f for f in store.frames() if f.timestamp_ns == ts(t)]
    return f


# --- readers ------------------------------------------------------------------------


@pytest.mark.parametrize("comp", ["NO", "RLE", "ZIP", "ZIPS"])
def test_exr_reader_matches_opencv(comp):
    from hydra_python.image_store.exr import read_exr

    rng = np.random.default_rng(1)
    d = (rng.random((37, 23)) * 5).astype(np.float32)
    d[0, 0], d[1, 1], d[5, :] = np.nan, np.inf, 2.0
    ok, buf = cv2.imencode(
        ".exr",
        d,
        [
            cv2.IMWRITE_EXR_TYPE,
            cv2.IMWRITE_EXR_TYPE_FLOAT,
            cv2.IMWRITE_EXR_COMPRESSION,
            getattr(cv2, f"IMWRITE_EXR_COMPRESSION_{comp}"),
        ],
    )
    assert ok
    np.testing.assert_array_equal(read_exr(buf.tobytes()), d)


def test_read_input_zip(tmp_path):
    from hydra_python.image_store.inputdata import read_input_zip

    T = R.pose(1, 2, 3, yaw=0.3)
    path = R.write_input_zip(
        tmp_path / "5.zip", 5, T, color=R.color_rgb((200, 30, 10)), depth=R.depth_m(1.5)
    )
    rec = read_input_zip(path)
    assert rec.timestamp_ns == 5
    np.testing.assert_allclose(rec.world_T_body, T, atol=1e-9)
    assert rec.sensor.name == "camera" and (rec.sensor.width, rec.sensor.height) == (
        16,
        12,
    )
    assert (rec.sensor.fx, rec.sensor.cx, rec.sensor.cy) == (R.FX, R.CX, R.CY)
    np.testing.assert_allclose(rec.sensor.body_T_sensor, R.body_T_sensor(), atol=1e-9)
    assert tuple(rec.color[5, 5]) == (200, 30, 10)  # RGB, as Hydra holds it
    assert rec.depth.dtype == np.float32 and rec.depth[5, 5] == pytest.approx(1.5)


def test_depth_to_mm():
    from hydra_python.image_store.inputdata import depth_to_mm

    out = depth_to_mm(np.array([[np.nan, np.inf, -1.0, 1.2344, 70.0]], np.float32))
    assert out.tolist() == [[0, 0, 0, 1234, 65535]]


# --- index -------------------------------------------------------------------------


def test_index_frames_kinds_and_anchors(indexed):
    run, stats = indexed
    assert stats["keyframes_indexed"] == 6 and stats.get("keyframes_failed", 0) == 0
    with ImageStore.open(run) as store:
        kinds = {f.timestamp_ns: f.kind for f in store.frames()}
        for t in (1.0, 1.5, 2.002, 3.4, 3.9, 5.0):
            assert kinds[ts(t)] == FrameKind.KEYFRAME
        for t in (1.2, 1.7, 2.5, 2.6, 3.0, 3.1):
            assert kinds[ts(t)] == FrameKind.OBSERVATION
        expect = {
            1.0: 0,
            1.5: 0,
            2.002: 1,
            3.4: 2,
            5.0: 3,
            1.2: 0,
            1.7: 1,
            2.5: 1,
            2.6: 2,
            3.0: 2,
            3.1: 2,
        }
        for t, a in expect.items():
            assert frame_at(store, t).anchor_node_id == A[a], t
        assert frame_at(store, 3.9).anchor_node_id is None  # 9 m from a3
        assert store.agent_nodes() == A


def test_anchor_transforms_use_odometry(indexed):
    run, _ = indexed
    backend = {
        n.node_id: n.world_T_body
        for n in load_agent_nodes(run / "hydra/backend/dsg.json")
    }
    with ImageStore.open(run) as store:
        kf = frame_at(store, 1.0)
        np.testing.assert_allclose(kf.anchor_T_body, np.eye(4), atol=1e-9)
        kf1 = frame_at(store, 2.002)
        np.testing.assert_allclose(
            kf1.anchor_T_body, np.linalg.inv(R.odom(2.0)) @ R.odom(2.002), atol=1e-9
        )
        sub = frame_at(store, 1.5)
        rel = np.linalg.inv(R.odom(1.0)) @ R.odom(1.5)
        np.testing.assert_allclose(sub.anchor_T_body, rel, atol=1e-9)
        np.testing.assert_allclose(sub.world_T_body_odom, R.odom(1.5), atol=1e-9)
        # corrected pose = backend anchor pose @ relative transform
        T, src = store.body_pose(sub, backend)
        assert src == "anchor"
        np.testing.assert_allclose(T, R.pose(0, 1.0) @ R.odom(1.5), atol=1e-9)
        T, src = store.body_pose(frame_at(store, 3.9), backend)
        assert src == "odom"


def test_keyframe_images_and_calib(indexed):
    run, _ = indexed
    with ImageStore.open(run) as store:
        (calib,) = store.sensors()  # archives and track logs share one sensor row
        assert (calib.width, calib.height, calib.fx, calib.cy) == (16, 12, R.FX, R.CY)
        assert calib.depth_scale == 0.001
        np.testing.assert_allclose(calib.body_T_sensor, R.body_T_sensor(), atol=1e-9)
        kf = frame_at(store, 1.0)
        assert kf.rgb_path.name == f"{ts(1.0)}_rgb.jpg"
        assert kf.depth_path.name == f"{ts(1.0)}_depth.png"
        rgb = iio.imread(kf.rgb_path)
        r, _g, b = rgb[6, 8].astype(int)
        assert r > 180 and b < 40  # true colours in the file
        depth = iio.imread(kf.depth_path)
        assert depth.dtype == np.uint16 and depth[6, 8] == 1234 and depth[0, 0] == 0
        assert depth[0, 2] == 65535
        assert store.calib(kf.sensor_id).sensor_id >> 48 == 0


def test_observations_follow_absorbed_tracks_and_merges(indexed):
    run, _ = indexed
    with ImageStore.open(run) as store:
        obs = store.observations(R.O1)
        assert sorted(o.track_id for o in obs) == [7, 12, 12]
        assert [o.score for o in obs] == [0.9, 0.4, 0.3]  # best first
        assert [o.track_id for o in store.observations(R.O1, kept_only=True)] == [12, 7]
        assert [o.timestamp_ns for o in store.observations(R.O1, order="time")] == [
            ts(1.2),
            ts(1.5),
            ts(1.7),
        ]
        kept = store.observations(R.O1, kept_only=True)[0]
        assert kept.frame.kind == FrameKind.KEYFRAME  # shares the 1.5 s keyframe
        assert kept.rgb_path.exists() and kept.mask_path.exists()
        assert kept.bbox2d == (2, 3, 9, 8)
        m = store.mask(kept)
        assert m.shape == (12, 16) and m[5, 5] and not m[0, 0]
        non_kept = next(o for o in obs if not o.kept)
        assert non_kept.mask_path is None and non_kept.rgb_path is None
        assert store.mask(non_kept)[5, 5]  # from mask_rle
        # backend merges: O2 -> O6, O4 -> O6
        assert store.canonical(R.O4) == R.O6 and store.canonical(R.O2) == R.O6
        assert sorted(o.track_id for o in store.observations(R.O6)) == [30, 31]
        assert sorted(o.track_id for o in store.observations(R.O4)) == [30, 31]
        assert R.O3 not in store.object_nodes()  # 30's ended binding was replaced
        assert set(store.object_nodes()) == {R.O1, R.O6}
        assert {t[1] for t in store.tracks(R.O1)} == {7, 9, 12}


def test_unbound_tracks(indexed):
    run, stats = indexed
    assert stats["tracks_unbound"] == 2
    with ImageStore.open(run) as store:
        assert store.unbound_tracks() == [
            (0, 20, "extraction_failed"),
            (0, 21, "no_binding"),
        ]
        (o,) = store.track_observations(0, 20)
        assert o.node_id is None and o.kept and o.rgb_path.exists()


def test_frames_for_agent(indexed):
    run, _ = indexed
    with ImageStore.open(run) as store:
        frames = store.frames_for_agent(A[0])  # keyframes only, time order
        assert [f.timestamp_ns for f in frames] == [ts(1.0), ts(1.5)]
        assert [f.timestamp_ns for f in store.frames_for_agent(A[0], ts(1.4))] == [
            ts(1.5),
            ts(1.0),
        ]
        # the agent node's own image: the keyframe at its stamp
        assert store.agent_frame(A[0], ts(1.0)).timestamp_ns == ts(1.0)
        assert store.agent_frame(A[1], ts(2.0)).timestamp_ns == ts(2.002)
        assert store.agent_frame(A[2], ts(3.0)) is None  # 3.4 s is 400 ms away
        assert len(store.frames(kind=FrameKind.OBSERVATION)) == 6


def test_reindex_is_idempotent_and_incremental(indexed):
    run, _ = indexed
    with ImageStore.open(run) as store:
        before = store.info()[0]
    stats = index(run)
    assert (
        stats.get("keyframes_indexed", 0) == 0 and stats.get("tracks_indexed", 0) == 0
    )
    with ImageStore.open(run) as store:
        assert store.info()[0] == before
    # new archive, a re-logged track (one more observation) and a new track
    R.write_input_zip(
        run / f"keyframes/{ts(3.0)}.zip",
        ts(3.0),
        R.odom(3.0),
        color=R.color_rgb(),
        depth=R.depth_m(),
    )
    os.utime(run / "tracks/12/track.json", ns=(1, 1))
    R.write_track(
        run / "tracks",
        12,
        [
            {"ts": ts(1.5), "T": R.odom(1.5), "kept": True, "score": 0.9},
            {"ts": ts(1.7), "T": R.odom(1.7), "kept": False, "score": 0.3},
            {"ts": ts(1.8), "T": R.odom(1.8), "kept": True, "score": 0.6},
        ],
        absorbed=[7, 9],
    )
    stats = index(run)
    assert stats["keyframes_indexed"] == 1 and stats["tracks_indexed"] == 1
    with ImageStore.open(run) as store:
        info = store.info()[0]
        assert info["observations"] == before["observations"] + 1
        assert len(store.observations(R.O1)) == 4
        # the observation frame at 3.0 s now also is an archive frame: a2's keyframe
        f = frame_at(store, 3.0)
        assert f.kind == FrameKind.KEYFRAME and f.rgb_path is not None
        assert f.anchor_node_id == A[2]
        assert [
            o.track_id for o in store.observations(R.O6) if o.frame_id == f.frame_id
        ] == [30]


def test_new_archive_becomes_keyframe_when_pending(run):
    index(run)
    R.write_input_zip(
        run / f"keyframes/{ts(4.01)}.zip",
        ts(4.01),
        R.odom(4.01),
        color=R.color_rgb(),
        depth=R.depth_m(),
    )
    index(run)
    with ImageStore.open(run) as store:
        f = frame_at(store, 4.01)
        assert f.kind == FrameKind.KEYFRAME and f.anchor_node_id == A[3]


def test_partial_mode_leaves_frames_pending(run):
    stats = index(run, final=False)
    assert stats["frames_pending"] == 1  # the 5.0 s archive, after the newest node
    with ImageStore.open(run) as store:
        assert frame_at(store, 5.0).anchor_node_id is None
    index(run)  # final pass decides it
    with ImageStore.open(run) as store:
        assert frame_at(store, 5.0).anchor_node_id == A[3]


def test_no_dsg_then_dsg(run):
    shutil.move(run / "hydra", run.parent / "hydra_later")
    stats = index(run)
    assert stats["frames_pending"] > 0
    with ImageStore.open(run) as store:
        assert all(f.anchor_node_id is None for f in store.frames())
        assert store.object_nodes() == []
    shutil.move(run.parent / "hydra_later", run / "hydra")
    index(run)
    with ImageStore.open(run) as store:
        assert frame_at(store, 1.0).kind == FrameKind.KEYFRAME
        assert store.canonical(R.O4) == R.O6


def test_odometry_from_keyframes_without_frontend_dsg(tmp_path):
    run = R.build_run(tmp_path / "run", with_frontend=False)
    stats = index(run)
    assert stats["anchor_odom_keyframe"] == 1  # a0 has an archive at its stamp
    assert stats["anchor_odom_backend"] == 3
    with ImageStore.open(run) as store:
        np.testing.assert_allclose(
            frame_at(store, 1.0).anchor_T_body, np.eye(4), atol=1e-9
        )


def test_failed_archive_is_skipped(run):
    (run / "keyframes" / f"{ts(2.5)}.zip").write_bytes(b"PK\x03\x04 broken")
    stats = index(run)
    assert stats["keyframes_failed"] == 1 and stats["keyframes_indexed"] == 6
    assert stats["warnings"]
    # not read (and warned about) again until it changes
    stats = index(run)
    assert stats["keyframes_failed"] == 1 and not stats.get("keyframes_indexed")
    assert not stats.get("warnings")
    R.write_input_zip(
        run / "keyframes" / f"{ts(2.5)}.zip",
        ts(2.5),
        R.odom(2.5),
        color=R.color_rgb(),
        depth=R.depth_m(),
    )
    stats = index(run)
    assert not stats.get("keyframes_failed") and stats["keyframes_indexed"] == 1


def test_robot_namespacing(tmp_path):
    run = R.build_run(tmp_path / "run_b", robot_prefix="b")
    stats = index(run)  # robot id inferred from the agent prefix
    with ImageStore.open(run) as store:
        assert {f.frame_id >> 48 for f in store.frames()} == {1}
        assert {f.robot_id for f in store.frames()} == {1}
        assert store.sensors()[0].sensor_id == make_id(1, 0)
        assert frame_at(store, 1.0).anchor_node_id == make_node_id("b", 0)
    assert stats["store"].endswith("image_store")


# --- opening runs ---------------------------------------------------------------------


def test_open_run_indexes_once(run):
    assert not is_indexed(run / "image_store")
    with ImageStore.open(run) as store:
        assert len(store.frames(kind=FrameKind.KEYFRAME)) == 6
    assert is_indexed(run / "image_store")
    db = run / "image_store" / "store.sqlite"
    mtime = db.stat().st_mtime_ns
    with ImageStore.open(run / "image_store") as store:  # unchanged inputs: just loads
        assert len(store.frames()) == 12
    assert db.stat().st_mtime_ns == mtime
    with ImageStore.open(run, index=False) as store:
        assert len(store.frames()) == 12


def test_open_reindexes_when_inputs_change(run):
    with ImageStore.open(run) as store:
        assert frame_at(store, 3.0).kind == FrameKind.OBSERVATION
    # a new archive (Hydra still writing when the run was first opened)
    R.write_input_zip(
        run / f"keyframes/{ts(3.0)}.zip", ts(3.0), R.odom(3.0), color=R.color_rgb()
    )
    with ImageStore.open(run) as store:
        assert frame_at(store, 3.0).kind == FrameKind.KEYFRAME
    # a rewritten track log
    R.write_track(
        run / "tracks", 31, [{"ts": ts(3.1), "T": R.odom(3.1), "kept": False}]
    )
    os.utime(run / "tracks/31/track.json", ns=(5, 5))
    with ImageStore.open(run) as store:
        assert [o.kept for o in store.track_observations(0, 31)] == [False]
    # a new track directory
    R.write_track(run / "tracks", 50, [{"ts": ts(3.2), "T": R.odom(3.2), "kept": True}])
    with ImageStore.open(run) as store:
        assert (0, 50, "no_binding") in store.unbound_tracks()


def test_open_anchors_pending_frames_once_backend_is_saved(run):
    """Opened while Hydra runs (no backend graph yet): frames after the newest agent node
    stay pending; the next open after the backend save anchors them."""
    backend = run / "hydra/backend"
    shutil.move(backend, run.parent / "backend_later")
    R.write_dsg(  # the frontend graph only knows a0..a2 so far
        run / "hydra/frontend/dsg.json",
        [(i, ts(t), R.odom(t)) for i, t in enumerate((1.0, 2.0, 3.0))],
    )
    with ImageStore.open(run) as store:
        assert frame_at(store, 5.0).anchor_node_id is None
    shutil.move(run.parent / "backend_later", backend)
    with ImageStore.open(run) as store:
        assert frame_at(store, 5.0).anchor_node_id == A[3]


def test_open_store_dir_of_unindexed_run(run):
    with ImageStore.open(run / "image_store") as store:
        assert len(store.frames()) == 12


def test_multi_open(tmp_path):
    a = R.build_run(tmp_path / "ra")
    b = R.build_run(tmp_path / "rb", robot_prefix="b")
    index(b)
    with ImageStore.open(a, b) as store:
        assert len(store.frames()) == 24
        assert {f.robot_id for f in store.frames()} == {0, 1}
        assert len(store.observations(R.O1)) == 6
        assert len(store.frames_for_agent(make_node_id("b", 0))) == 2


def test_external_rows_survive_reindex(indexed):
    """Rows another writer added with a track id from EXTERNAL_TRACK_BASE up."""
    run, _ = indexed
    node = make_node_id("O", 77)
    track = EXTERNAL_TRACK_BASE | 77
    with ImageStore.open_writer(run, robot_id=0) as w:
        sid = w.reader().sensors()[0].sensor_id
        src = run / "external.jpg"
        iio.imwrite(src, np.zeros((12, 16, 3), np.uint8))
        fid = w.add_frame_ref(sid, ts(9.0), kind=FrameKind.OBSERVATION, rgb=src)
        mask = np.zeros((12, 16), bool)
        mask[2:5, 2:5] = True
        with w.batch():
            w.bind_track(track, node)
            w.add_observation(track, fid, mask=mask, score=0.8, kept=True)
    index(run)
    with ImageStore.open(run) as store:
        (o,) = store.observations(node)
        assert o.track_id == track and store.mask(o).sum() == 9
        assert o.frame.anchor_node_id is None  # not touched by the indexer
        assert len(store.observations(R.O1)) == 3


def test_open_writer_does_not_index(run):
    with ImageStore.open_writer(run) as w:
        assert w.root == run / "image_store"
        assert w.reader().frames() == []
    assert not is_indexed(run / "image_store")


def test_cli_index_info_check(run, capsys):
    shutil.move(run / "keyframes", run / "kf_elsewhere")
    assert (
        cli_main(
            [
                "index",
                str(run),
                "--keyframes",
                str(run / "kf_elsewhere"),
                "--tracks",
                str(run / "tracks"),
                "--hydra",
                str(run / "hydra"),
                "--robot-id",
                "0",
            ]
        )
        == 0
    )
    stats = json.loads(capsys.readouterr().out)
    assert stats["keyframes_indexed"] == 6
    assert cli_main(["info", str(run / "image_store"), "--json"]) == 0
    info = json.loads(capsys.readouterr().out)[0]
    assert info["frames"] == {"keyframe": 6, "observation": 6}
    assert info["meta"]["indexer_version"] == "1"
    assert cli_main(["check", str(run)]) == 0


def test_export_indexed(indexed, tmp_path):
    from hydra_python.image_store.export import export_store
    from hydra_python.image_store.poses import load_dsg

    run, _ = indexed
    with ImageStore.open(run) as store:
        stats = export_store(
            store, tmp_path / "out", load_dsg(run / "hydra/backend/dsg.json")
        )
    assert stats
    assert (tmp_path / "out" / "index.json").exists()


def test_load_agent_nodes_spark_graph(run):
    sd = pytest.importorskip("spark_dsg")
    g = sd.DynamicSceneGraph.load(str(run / "hydra/backend/dsg.json"))
    nodes = load_agent_nodes(g, "a")
    assert [n.node_id for n in nodes] == A
    assert [n.timestamp_ns for n in nodes] == [ts(t) for t in (1, 2, 3, 4)]
    assert load_agent_nodes(g, "b") == []


def test_codecs_without_opencv(tmp_path):
    """The indexer's image codecs are imageio only: archives decode and the store's
    JPEG / 16-bit PNG files encode with OpenCV unimportable."""
    import subprocess
    import sys

    path = R.write_input_zip(
        tmp_path / "7.zip",
        7,
        np.eye(4),
        color=R.color_rgb((200, 30, 10)),
        depth=R.depth_m(2.0),
    )
    code = f"""
import pathlib, sys
sys.modules["cv2"] = None  # import cv2 -> ImportError
import imageio.v3 as iio
from hydra_python.image_store import inputdata
rec = inputdata.read_input_zip({str(path)!r})
assert tuple(rec.color[5, 5]) == (200, 30, 10), rec.color[5, 5]
assert abs(rec.depth[5, 5] - 2.0) < 1e-6
inputdata.write_jpeg(pathlib.Path({str(tmp_path / "c.jpg")!r}), rec.color)
r, _g, b = iio.imread({str(tmp_path / "c.jpg")!r})[5, 5].astype(int)
assert r > 180 and b < 40
mm = inputdata.depth_to_mm(rec.depth)
inputdata.write_png16(pathlib.Path({str(tmp_path / "d.png")!r}), mm)
d = iio.imread({str(tmp_path / "d.png")!r})
assert d.dtype == "uint16" and (d == mm).all() and d[5, 5] == 2000
assert "cv2" not in {{k for k, v in sys.modules.items() if v is not None}}
"""
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=False
    )
    assert out.returncode == 0, out.stderr


def test_exr_fallback_reads_other_encodings(tmp_path):
    """EXR encodings Hydra never writes (here PIZ) go through imageio's opencv plugin,
    in OpenCV's channel order like read_exr."""
    from hydra_python.image_store.exr import UnsupportedExr, read_exr
    from hydra_python.image_store.inputdata import decode_exr

    params = [cv2.IMWRITE_EXR_COMPRESSION, cv2.IMWRITE_EXR_COMPRESSION_PIZ]
    depth = np.random.default_rng(0).uniform(0.1, 9, (6, 8)).astype(np.float32)
    ok, buf = cv2.imencode(".exr", depth, params)
    with pytest.raises(UnsupportedExr):
        read_exr(buf.tobytes())
    np.testing.assert_array_equal(decode_exr(buf.tobytes()), depth)
    bgr = np.random.default_rng(1).uniform(0, 1, (6, 8, 3)).astype(np.float32)
    ok, buf = cv2.imencode(".exr", bgr, params)
    np.testing.assert_array_equal(decode_exr(buf.tobytes()), bgr)


# --- reconciliation with the C++ writers (FORMAT.md "[C++ writer]") -------------------


def _cpp_bindings(run, layer="OBJECTS", extra=None):
    """Bindings as the current GraphUpdater writes them: no node merges, several tracks
    may share a node, ends are "deleted" / "track_removed"."""
    R.write_bindings(
        run / "hydra/frontend",
        [
            (12, R.O1, 1, 2, True, None),
            (7, R.O5, 1, 1, False, "deleted"),
            (30, R.O2, 1, 3, True, None),
            (31, R.O2, 2, 3, True, None),
        ],
        layer=layer,
        extra=extra,
    )


def _backend_objects(run, objects):
    """Rewrite the reference backend graph with other object nodes."""
    R.write_dsg(
        run / "hydra/backend/dsg.json",
        [
            (i, ts(t), R.pose(0, 1.0) @ R.odom(t))
            for i, t in enumerate((1.0, 2.0, 3.0, 4.0))
        ],
        objects=objects,
    )


@pytest.mark.parametrize("layer", ["OBJECTS", "objects", "Objects"])
def test_binding_layer_key_any_case(run, layer):
    stray = {
        "track_id": 21,
        "node_id": R.O9,
        "first_ns": 1,
        "last_ns": 9,
        "active": True,
        "end": None,
    }
    _cpp_bindings(run, layer, extra={"PLACES": [stray]})
    index(run)
    with ImageStore.open(run) as store:
        assert {t[1] for t in store.tracks(R.O1)} == {7, 9, 12}
        assert R.O9 not in store.object_nodes()  # other layers are not object tracks
        assert (0, 21, "no_binding") in store.unbound_tracks()


def test_bindings_of_any_layer_when_no_object_layer(run):
    _cpp_bindings(run, "MY_OBJECTS")
    index(run)
    with ImageStore.open(run) as store:
        assert {t[1] for t in store.tracks(R.O1)} == {7, 9, 12}


def test_several_tracks_share_a_node(run):
    _cpp_bindings(run)
    (run / "hydra/backend/merges.json").unlink()
    _backend_objects(run, (R.O1, R.O2))
    stats = index(run)
    assert stats["node_aliases"] == 0
    with ImageStore.open(run) as store:
        obs = store.observations(R.O2)
        assert sorted(o.track_id for o in obs) == [30, 31]
        assert {o.node_id for o in obs} == {R.O2}
        assert store.canonical(R.O2) == R.O2
        assert set(store.object_nodes()) == {R.O1, R.O2}


def test_binding_to_deleted_node_is_dropped(run):
    """A track whose newest binding ended because the frontend deleted its node points at
    nothing: the track is unbound (reason ``node_deleted``)."""
    R.write_track(run / "tracks", 40, [{"ts": ts(1.5), "T": R.odom(1.5), "kept": True}])
    R.write_bindings(
        run / "hydra/frontend",
        [
            (12, R.O1, 1, 2, True, None),
            (40, R.O6, 1, 1, False, "deleted"),
            (41, R.O6, 1, 1, False, "track_removed"),
        ],
    )
    index(run)
    with ImageStore.open(run) as store:
        assert R.O6 not in store.object_nodes()
        assert (0, 40, "node_deleted") in store.unbound_tracks()
        assert not store.tracks(R.O6)


def test_khronos_backend_layout(run):
    """Khronos' backend saves dsg.json and merges.json into the log root."""
    for name in ("dsg.json", "merges.json"):
        shutil.move(run / "hydra/backend" / name, run / "hydra" / name)
    (run / "hydra/backend").rmdir()
    stats = index(run)
    assert stats["inputs"]["backend_dsg"].endswith("hydra/dsg.json")
    assert stats["inputs"]["merges"].endswith("hydra/merges.json")
    with ImageStore.open(run) as store:
        assert frame_at(store, 1.0).kind == FrameKind.KEYFRAME
        assert store.canonical(R.O2) == R.O6


def test_robot_id_from_track_logs(tmp_path):
    from hydra_python.image_store.indexer import find_inputs, infer_robot_id

    run = R.build_run(tmp_path / "run", robot_prefix="c")
    shutil.rmtree(run / "hydra")  # no scene graph to infer it from
    assert infer_robot_id(find_inputs(run)) == 2
    index(run)
    with ImageStore.open(run) as store:
        assert {f.robot_id for f in store.frames()} == {2}
        assert {o.robot_id for o in store.track_observations(2, 12)} == {2}


def test_null_fields_from_cpp(run):
    """Non-pinhole sensor (null intrinsics), null semantic label, empty mask (null
    bbox and mask_rle), mask_rle string for a non-kept observation."""
    R.write_track(
        run / "tracks",
        40,
        [
            {
                "ts": ts(2.2),
                "T": R.odom(2.2),
                "kept": False,
                "bbox": None,
                "mask": np.zeros((R.H, R.W), bool),
                "sensor": {
                    "name": "lidar",
                    "fx": None,
                    "fy": None,
                    "cx": None,
                    "cy": None,
                    "body_T_sensor": None,
                },
            },
            {"ts": ts(2.3), "T": R.odom(2.3), "kept": False, "mask_rle": "[5,1,186]"},
        ],
        label=None,
    )
    stats = index(run)
    assert stats.get("tracks_failed", 0) == 0
    with ImageStore.open(run) as store:
        a, b = store.track_observations(0, 40)
        assert a.bbox2d is None and a.mask_rle is None and store.mask(a) is None
        lidar = store.calib(a.sensor_id)
        assert lidar.name == "lidar" and lidar.fx == 0 and lidar.width == R.W
        assert b.mask_rle == "[5,1,186]" and store.mask(b).sum() == 1


@pytest.mark.parametrize(
    "broken",
    [
        lambda d: d["observations"][0].update(mask_rle=[5, 1, 186]),  # not a string
        lambda d: d.pop("robot_id"),
        lambda d: d.pop("absorbed_track_ids"),
        lambda d: d["observations"][0].pop("sensor"),
    ],
)
def test_malformed_track_log_is_skipped(run, broken):
    d = R.write_track(
        run / "tracks", 40, [{"ts": ts(2.3), "T": R.odom(2.3), "kept": False}]
    )
    data = json.loads((d / "track.json").read_text())
    broken(data)
    (d / "track.json").write_text(json.dumps(data))
    stats = index(run)
    assert stats["tracks_failed"] == 1 and stats["warnings"]
    with ImageStore.open(run) as store:
        assert store.track_observations(0, 40) == []


REAL = pathlib.Path(__file__).parent / "data" / "real_cpp"


def test_real_cpp_artifacts(tmp_path):
    """Files written by the C++ writers' own gtests (see data/real_cpp/README)."""
    from hydra_python.image_store.inputdata import read_input_zip

    rec = read_input_zip(REAL / "keyframes" / "30.zip")
    assert rec.timestamp_ns == 30 and rec.world_T_body[0, 3] == pytest.approx(2.0)
    assert tuple(rec.color[1, 1]) == (10, 20, 30)  # the Mat Hydra saved
    assert rec.depth[1, 1] == pytest.approx(1.5)
    assert (rec.sensor.fx, rec.sensor.fy, rec.sensor.cx, rec.sensor.cy) == (3, 4, 1, 2)

    run = tmp_path / "run"
    shutil.copytree(REAL, run)
    R.write_dsg(
        run / "hydra/backend/dsg.json",
        [(i, t, R.pose(x)) for i, (t, x) in enumerate([(10, 0.0), (40, 4.0)])],
        objects=(R.O1,),
    )
    R.write_bindings(run / "hydra/frontend", [(12, R.O1, 10, 20, True, None)])
    stats = index(run)
    assert stats["keyframes_indexed"] == 3 and stats["observations_indexed"] == 5
    with ImageStore.open(run) as store:
        keyframes = store.frames(kind=FrameKind.KEYFRAME)
        assert [f.timestamp_ns for f in keyframes] == [10, 30, 40]
        assert [f.anchor_node_id for f in keyframes] == [A[0], A[1], A[1]]
        kf = store.frames(kind=FrameKind.KEYFRAME)[0]
        assert tuple(iio.imread(kf.rgb_path)[1, 1]) == pytest.approx(
            (10, 20, 30), abs=3
        )
        obs = store.observations(R.O1)
        assert [o.kept for o in obs] == [True, False]
        assert store.mask(obs[1]).sum() == 1  # "[5,1,6]" on 3x4
        assert store.mask(obs[0]) is not None and obs[0].depth_path.exists()
        assert sorted(store.unbound_tracks()) == [
            (0, 4, "extraction_failed"),
            (0, 5, "extraction_failed"),
            (0, 8, "no_binding"),  # non-finite pose (null in track.json)
        ]
        (o8,) = store.track_observations(0, 8)
        assert o8.kept  # shares track 12's frame at 10 ns (same sensor and stamp)
        assert {c.name for c in store.sensors()} == {"camera", "test_camera"}


# --- bindings, aliases and archive names ----------------------------------------


def test_reindex_replaces_stale_aliases(run):
    index(run)
    with ImageStore.open(run) as store:
        assert store.canonical(R.O4) == R.O6
    # a later save rewrote merges.json: O4 is no longer merged, O3 is
    R.write_merges(run / "hydra/backend", {R.O2: R.O6, R.O3: R.O6})
    _backend_objects(run, (R.O1, R.O4, R.O6))
    stats = index(run)
    assert stats["node_aliases"] == 2
    with ImageStore.open(run) as store:
        assert store.canonical(R.O4) == R.O4
        assert store.canonical(R.O3) == R.O6
        assert store.canonical(R.O2) == R.O6
        assert store.info()[0]["node_aliases"] == 2


def test_reindex_keeps_aliases_of_other_writers(run):
    index(run)
    with ImageStore.open_writer(run / "image_store", robot_id=0) as w:
        w.record_merges({R.O9: R.O1})  # e.g. a manual merge, not from merges.json
    index(run)
    with ImageStore.open(run) as store:
        assert store.canonical(R.O9) == R.O1


def test_binding_to_node_missing_from_backend(run):
    """A binding whose node (after merges) is not in the backend scene graph is
    unbound with reason node_missing; without a backend graph it is kept."""
    _backend_objects(run, (R.O6,))  # O1 never reached the backend
    index(run)
    with ImageStore.open(run) as store:
        assert (0, 12, "node_missing") in store.unbound_tracks()
        assert (0, 7, "node_missing") in store.unbound_tracks()
        assert R.O1 not in store.object_nodes()
        assert set(store.object_nodes()) == {R.O6}
    (run / "hydra/backend/dsg.json").unlink()
    index(run)
    with ImageStore.open(run) as store:
        assert R.O1 in store.object_nodes()


def test_ended_binding_of_failed_extraction_is_not_bound(run):
    """track_removed binding + extracted: false -> extraction_failed, the node is the
    other tracks'; an active binding of such a track is still used."""
    R.write_track(
        run / "tracks",
        41,
        [{"ts": ts(2.1), "T": R.odom(2.1), "kept": False}],
        extracted=False,
    )
    R.write_track(
        run / "tracks",
        42,
        [{"ts": ts(2.2), "T": R.odom(2.2), "kept": False}],
        extracted=False,
    )
    R.write_bindings(
        run / "hydra/frontend",
        [
            (12, R.O1, 1, 2, True, None),
            (41, R.O1, 1, 2, False, "track_removed"),
            (42, R.O1, 1, 2, True, None),
        ],
    )
    index(run)
    with ImageStore.open(run) as store:
        assert (0, 41, "extraction_failed") in store.unbound_tracks()
        assert 41 not in {t[1] for t in store.tracks(R.O1)}
        assert 42 in {t[1] for t in store.tracks(R.O1)}


def test_archive_names():
    from hydra_python.image_store.indexer import (
        archive_sensor_suffix,
        parse_archive_name,
    )

    assert parse_archive_name("123.zip") == (123, None)
    assert parse_archive_name("123_rear_cam_1.zip") == (123, "rear_cam_1")
    for bad in ("123.zip.tmp.ABCDEF", "abc.zip", "123_.zip", "_x.zip", "123.txt"):
        assert parse_archive_name(bad) is None
    assert archive_sensor_suffix("rear cam/1") == "rear_cam_1"
    assert archive_sensor_suffix("front-left.v2_x") == "front-left.v2_x"


def test_archives_of_several_sensors(run):
    """<ts>.zip (first sensor) and <ts>_<sensor>.zip at the same stamp are both
    indexed, once, whatever the order."""
    t = ts(2.6)
    R.write_input_zip(
        run / f"keyframes/{t}_rear_cam_1.zip",
        t,
        R.odom(2.6),
        color=R.color_rgb(),
        sensor="rear cam/1",
    )
    stats = index(run)
    assert stats["keyframes_indexed"] == 7
    R.write_input_zip(run / f"keyframes/{t}.zip", t, R.odom(2.6), color=R.color_rgb())
    stats = index(run)
    assert stats["keyframes_indexed"] == 1
    assert index(run).get("keyframes_indexed", 0) == 0
    with ImageStore.open(run) as store:
        names = {
            store.calib(f.sensor_id).name
            for f in store.frames(kind=FrameKind.KEYFRAME)
            if f.timestamp_ns == t
        }
        assert names == {"camera", "rear cam/1"}


def _descriptor_run(tmp_path, with_features):
    run = tmp_path / ("on" if with_features else "off")
    R.write_input_zip(
        run / "keyframes/1000.zip",
        1000,
        R.pose(0),
        color=R.color_rgb(),
        feature=[0.5, -1.0, 2.0] if with_features else None,
    )
    R.write_input_zip(run / "keyframes/2000.zip", 2000, R.pose(1), color=R.color_rgb())
    return run


@pytest.mark.parametrize("with_features", [False, True])
def test_keyframe_descriptors(tmp_path, with_features):
    run = _descriptor_run(tmp_path, with_features)
    stats = index(run)
    assert stats.get("descriptors_indexed", 0) == (1 if with_features else 0)
    with ImageStore.open(run) as store:
        f1, f2 = store.frames()
        assert store.descriptor(f2.frame_id) is None
        found = list(store.descriptors())
        if not with_features:
            assert store.descriptor(f1.frame_id) is None and found == []
            return
        d = store.descriptor(f1.frame_id)
        assert d.dtype == np.float32 and d.tolist() == [0.5, -1.0, 2.0]
        assert [(f.frame_id, v.tolist()) for f, v in found] == [
            (f1.frame_id, [0.5, -1.0, 2.0])
        ]
        assert list(store.descriptors(kind=FrameKind.OBSERVATION)) == []
        assert len(list(store.descriptors(kind=FrameKind.KEYFRAME))) == 1
    # re-index (unchanged or forced) keeps exactly one descriptor
    index(run)
    (run / "image_store" / "store.sqlite").touch()
    index(run)
    with ImageStore.open(run) as store:
        assert len(list(store.descriptors())) == 1


def test_real_cpp_multi_sensor_and_descriptors(tmp_path):
    from hydra_python.image_store.inputdata import read_input_zip

    rec = read_input_zip(REAL / "keyframes_features" / "10.zip")
    assert rec.feature.tolist() == [1.0, 1.0, 1.0]
    assert rec.label_features[3].tolist() == [0.5, 0.5]
    assert read_input_zip(REAL / "keyframes" / "10.zip").feature is None

    run = tmp_path / "run"
    shutil.copytree(REAL / "keyframes_multi", run / "keyframes")
    shutil.copy(REAL / "keyframes_features" / "10.zip", run / "keyframes" / "5.zip")
    stats = index(run)
    assert stats["keyframes_indexed"] == 4 and stats["descriptors_indexed"] == 1
    with ImageStore.open(run) as store:
        at10 = {
            store.calib(f.sensor_id).name
            for f in store.frames()
            if f.timestamp_ns == 10
        }
        assert at10 == {"front", "rear cam/1", "camera"}
        assert len(list(store.descriptors())) == 1
