import json
import sqlite3

import numpy as np
import pytest
from conftest import BODY_T_SENSOR, make_T, rot_z, write_image
from hydra_image_store.cli import main
from hydra_image_store.codec import SEQ_MASK
from hydra_image_store.fuse import FuseSource, fuse_stores, load_node_map

from hydra_image_store import FrameKind, ImageStore, make_node_id


def obj(index):
    return make_node_id("O", index)


def make_store(root, name, value):
    """A robot-0 store with 2 keyframes on agents a0/a1 and tracks 10/11/12 on O1/O2/O3."""
    src = root / f"{name}_src"
    store_dir = root / name
    w = ImageStore.open_writer(store_dir, robot_id=0)
    sid = w.add_sensor("cam", 8, 6, 5.0, 5.0, 4.0, 3.0, 0.001, BODY_T_SENSOR)
    kf0 = w.add_frame_ref(
        sid,
        100,
        kind=FrameKind.KEYFRAME,
        world_T_body_odom=make_T(t=(1, 0, 0)),
        rgb=write_image(src / "kf0.jpg", value=value),
        anchor_node_id=make_node_id("a", 0),
        link="copy",
    )
    kf1 = w.add_frame_ref(
        sid,
        200,
        kind=FrameKind.KEYFRAME,
        world_T_body_odom=make_T(t=(2, 0, 0)),
        rgb=write_image(src / "kf1.jpg", value=value),
        anchor_node_id=make_node_id("a", 1),
        link="copy",
    )
    w.add_observation(10, kf0, bbox3d=((0, 0, 0), (1, 1, 1)), score=0.9, kept=True)
    w.add_observation(11, kf1, score=0.7, kept=True)
    w.add_observation(12, kf1, score=0.2)
    w.bind_track(10, make_node_id("O", 1))
    w.bind_track(11, make_node_id("O", 2))
    w.bind_track(12, make_node_id("O", 3))
    w.close()
    return store_dir


@pytest.fixture
def two_robots(tmp_path):
    first = make_store(tmp_path, "first", 50)
    second = make_store(tmp_path, "second", 200)
    # the second robot's backend merged O3 into O2 before fusion
    with sqlite3.connect(second / "store.sqlite") as conn:
        conn.execute(
            "INSERT INTO node_aliases (child_id, parent_id) VALUES (?, ?)",
            (make_node_id("O", 3), make_node_id("O", 2)),
        )

    node_map = {
        0: {
            make_node_id("a", 0): make_node_id("a", 0),
            make_node_id("a", 1): make_node_id("a", 1),
            obj(1): obj(0),
            obj(2): obj(1),
            obj(3): obj(2),
        },
        # agent layer moved to prefix b by hydra_multi, O3 is gone (merged into O2)
        1: {
            make_node_id("b", 0): make_node_id("b", 0),
            make_node_id("b", 1): make_node_id("b", 1),
            obj(1): obj(3),
            obj(2): obj(4),
        },
    }
    merges = {obj(3): obj(0)}  # both robots saw the same object
    return {"first": first, "second": second, "node_map": node_map, "merges": merges}


def test_fuse_rekeys_ids_and_nodes(tmp_path, two_robots):
    world_T_second = make_T(rot_z(np.pi / 2), (100, 0, 0))
    out = tmp_path / "fused"
    stats = fuse_stores(
        out,
        [
            FuseSource(two_robots["first"], 0, "day1"),
            FuseSource(two_robots["second"], 1, "day2", world_T_second),
        ],
        two_robots["node_map"],
        two_robots["merges"],
    )

    first, second = stats["sources"]
    assert (
        first["frames"] == 2
        and first["track_bindings"] == 3
        and first["missing_nodes"] == 0
    )
    assert second["robot_id"] == 1 and second["source_robot_id"] == 0
    assert second["track_bindings"] == 3 and second["dropped_bindings"] == 0
    assert stats["fused_merges"] == 1

    with ImageStore.open(out) as store:
        frames = store.frames()
        assert len(frames) == 4
        assert len({f.frame_id for f in frames}) == 4
        assert {f.frame_id >> 48 for f in frames} == {0, 1}
        assert all(f.rgb_path.exists() for f in frames)

        # robot 1's agents moved to prefix b
        b1_frames = store.frames_for_agent(make_node_id("b", 1))
        assert len(b1_frames) == 1
        np.testing.assert_allclose(
            b1_frames[0].world_T_body_odom,
            world_T_second @ make_T(t=(2, 0, 0)),
            atol=1e-12,
        )

        # fused merge joins track 10 of both robots
        obs = store.observations(obj(0))
        assert sorted((o.robot_id, o.track_id) for o in obs) == [(0, 10), (1, 10)]
        # source merge: track 12 of robot 1 follows O3 -> O2 -> fused O4
        obs = store.observations(obj(4))
        assert sorted((o.robot_id, o.track_id) for o in obs) == [(1, 11), (1, 12)]

        # 3d boxes moved into the fused world frame
        moved = [o for o in store.observations(obj(3)) if o.robot_id == 1][0]
        assert moved.bbox3d is not None

    with sqlite3.connect(out / "store.sqlite") as conn:
        rows = conn.execute(
            "SELECT robot_id, session FROM sources ORDER BY robot_id"
        ).fetchall()
        assert rows == [(0, "day1"), (1, "day2")]
        bbox = conn.execute(
            "SELECT bbox3d_min_x, bbox3d_max_x, bbox3d_min_y, bbox3d_max_y FROM observations "
            "WHERE robot_id = 1 AND track_id = 10"
        ).fetchone()
        assert bbox == pytest.approx((99.0, 100.0, 0.0, 1.0))
        seqs = {r[0] & SEQ_MASK for r in conn.execute("SELECT frame_id FROM frames")}
        assert len(seqs) == 2  # sequence numbers are kept, robot bits differ


def test_source_merge_not_in_fused_graph(tmp_path, two_robots):
    # the fused graph kept O3 of the second robot although its backend merged it into O2
    node_map = dict(two_robots["node_map"])
    node_map[1] = dict(node_map[1])
    node_map[1][obj(3)] = obj(5)
    fuse_stores(
        tmp_path / "fused",
        [FuseSource(two_robots["first"], 0), FuseSource(two_robots["second"], 1)],
        node_map,
    )
    with ImageStore.open(tmp_path / "fused") as store:
        assert [(o.robot_id, o.track_id) for o in store.observations(obj(5))] == [
            (1, 12)
        ]
        assert [(o.robot_id, o.track_id) for o in store.observations(obj(4))] == [
            (1, 11)
        ]


def test_fuse_refuses_duplicates_and_existing(tmp_path, two_robots):
    out = tmp_path / "fused"
    sources = [FuseSource(two_robots["first"], 0), FuseSource(two_robots["second"], 0)]
    with pytest.raises(ValueError):
        fuse_stores(out, sources, two_robots["node_map"])

    sources[1].robot_id = 1
    fuse_stores(out, sources, two_robots["node_map"])
    with pytest.raises(FileExistsError):
        fuse_stores(out, sources, two_robots["node_map"])
    fuse_stores(out, sources, two_robots["node_map"], overwrite=True)


def test_unmapped_nodes_are_dropped(tmp_path, two_robots):
    node_map = {0: {}, 1: two_robots["node_map"][1]}
    stats = fuse_stores(
        tmp_path / "fused",
        [FuseSource(two_robots["first"], 0), FuseSource(two_robots["second"], 1)],
        node_map,
    )
    first = stats["sources"][0]
    assert first["dropped_bindings"] == 3
    assert first["unanchored_frames"] == 2
    with ImageStore.open(tmp_path / "fused") as store:
        assert all(
            f.anchor_node_id is None for f in store.frames() if f.frame_id >> 48 == 0
        )


def test_cli(tmp_path, two_robots):
    fused = tmp_path / "multi"
    (fused / "backend").mkdir(parents=True)
    (fused / "interfaces" / "robot_1").mkdir(parents=True)
    node_map = {
        "robots": {
            str(r): [[o, f] for o, f in m.items()]
            for r, m in two_robots["node_map"].items()
        },
        "merges": [[c, p] for c, p in two_robots["merges"].items()],
    }
    (fused / "backend" / "fusion_node_map.json").write_text(json.dumps(node_map))
    np.savetxt(
        fused / "interfaces" / "robot_1" / "world_T_robot.txt", make_T(t=(5, 0, 0))
    )

    robots, merges = load_node_map(fused / "backend" / "fusion_node_map.json")
    assert robots == two_robots["node_map"] and merges == two_robots["merges"]

    out = tmp_path / "fused_store"
    rc = main(
        [
            "fuse",
            str(fused),
            "--out",
            str(out),
            "--source",
            f"0={two_robots['first']}:day1",
            "--source",
            f"1={two_robots['second']}",
        ]
    )
    assert rc == 0
    with ImageStore.open(out) as store:
        frame = store.frames_for_agent(make_node_id("b", 0))[0]
        np.testing.assert_allclose(frame.world_T_body_odom[:3, 3], (6, 0, 0))
