import sqlite3

import numpy as np
import pytest
from conftest import (
    AGENT0,
    AGENT1,
    BODY_T_SENSOR,
    OBJ_A,
    OBJ_B,
    OBJ_C,
    make_T,
    rot_z,
    write_image,
)

from hydra_image_store import (
    FrameKind,
    ImageStore,
    JsonDsg,
    SchemaVersionError,
    make_id,
    make_node_id,
    sam3_track_id,
)


def test_round_trip(synthetic):
    s = synthetic
    with ImageStore.open(s["dir"]) as store:
        f = store.frame(s["kf0"])
        assert (
            f.kind == FrameKind.KEYFRAME
            and f.timestamp_ns == 100
            and f.anchor_node_id == AGENT0
        )
        np.testing.assert_array_equal(f.anchor_T_body, np.eye(4))
        assert (
            f.rgb_path.is_absolute() and f.rgb_path.exists() and f.depth_path.exists()
        )
        assert f.rgb_path == s["dir"] / "frames/r0/100_rgb.jpg"
        c = store.calib(s["sensor"])
        assert (c.width, c.height, c.fx, c.cx, c.depth_scale) == (8, 6, 5.0, 4.0, 0.001)
        np.testing.assert_allclose(c.body_T_sensor, BODY_T_SENSOR)
        obs = store.observations(OBJ_A)
        assert [o.obs_id for o in obs] == [s["obs"]["a1"], s["obs"]["a2"]]  # by score
        o = obs[0]
        assert o.bbox2d == (3, 2, 5, 3) and o.kept and o.node_id == OBJ_A
        np.testing.assert_array_equal(o.bbox3d[1], [1, 1, 1])
        assert o.rgb_path == f.rgb_path and o.mask_path.exists()
        np.testing.assert_array_equal(store.mask(o), s["mask"])
        assert [x.timestamp_ns for x in store.observations(OBJ_B, order="time")] == [
            150,
            200,
        ]
        assert len(store.observations(OBJ_B, kept_only=True)) == 1
        assert len(store.observations(OBJ_A, limit=1)) == 1
        assert [x.frame_id for x in store.frames_for_agent(AGENT0)] == [
            s["kf0"],
            s["skf"],
        ]
        assert [
            x.frame_id
            for x in store.frames_for_agent(AGENT0, include_subkeyframes=False)
        ] == [s["kf0"]]
        assert len(store.frames()) == 4
        assert [x.timestamp_ns for x in store.frames(t0=120, t1=170)] == [150, 160]
        assert [x.frame_id for x in store.frames(kind=FrameKind.OBSERVATION)] == [
            s["obs_frame"]
        ]
        assert store.frames(robot_id=1) == []
        assert store.agent_nodes() == [AGENT0, AGENT1]
        assert store.object_nodes() == [OBJ_A, OBJ_B, OBJ_C]
        info = store.info()[0]
        assert info["observations"] == 5 and info["bound_nodes"] == 3


def test_ids_resume_and_frame_dedup(synthetic, tmp_path):
    s = synthetic
    w = ImageStore.open_writer(s["dir"], robot_id=0)
    fid = w.add_frame_ref(s["sensor"], 300, kind=FrameKind.SUBKEYFRAME)
    assert fid == make_id(0, 4)  # four frames before
    again = w.add_frame_ref(
        s["sensor"],
        300,
        rgb=write_image(tmp_path / "late.jpg"),
        anchor_node_id=AGENT1,
        link="copy",
    )
    assert again == fid
    oid = w.add_observation(10, fid)
    assert oid == make_id(0, 5)
    w.close()
    w2 = ImageStore.open_writer(s["dir"], robot_id=2)
    assert w2.add_frame_ref(s["sensor"], 301) == make_id(2, 0)
    w2.close()
    with ImageStore.open(s["dir"]) as store:
        f = store.frame(fid)
        assert (
            f.kind == FrameKind.SUBKEYFRAME
            and f.rgb_path.exists()
            and f.anchor_node_id == AGENT1
        )


def test_file_inside_store_is_referenced_in_place(tmp_path):
    w = ImageStore.open_writer(tmp_path / "st")
    sid = w.add_sensor("cam", 8, 6, 1, 1, 1, 1)
    img = write_image(tmp_path / "st/custom/x.jpg")
    fid = w.add_frame_ref(sid, 5, rgb=img)
    assert (
        w.conn.execute(
            "SELECT rgb_path FROM frames WHERE frame_id=?", (fid,)
        ).fetchone()[0]
        == "custom/x.jpg"
    )
    assert w.add_sensor("cam", 9, 6, 1, 1, 1, 1) == sid  # upsert by (robot, name)
    w.close()


def test_hardlink_is_default(tmp_path):
    src = write_image(tmp_path / "a.jpg")
    w = ImageStore.open_writer(tmp_path / "st")
    sid = w.add_sensor("cam", 8, 6, 1, 1, 1, 1)
    w.add_frame_ref(sid, 1, rgb=src)
    w.close()
    assert (tmp_path / "st/frames/r0/1_rgb.jpg").stat().st_ino == src.stat().st_ino


def test_schema_version_checked(synthetic):
    conn = sqlite3.connect(synthetic["dir"] / "store.sqlite")
    conn.execute("UPDATE meta SET value='99' WHERE key='schema_version'")
    conn.commit()
    conn.close()
    with pytest.raises(SchemaVersionError):
        ImageStore.open(synthetic["dir"])


def test_readonly_reader_sees_commits_while_writer_open(synthetic):
    w = ImageStore.open_writer(synthetic["dir"])
    store = ImageStore.open(synthetic["dir"] / "store.sqlite")
    with w.batch():
        w.record_merges({OBJ_C: OBJ_A})
        assert store.canonical(OBJ_C) == OBJ_C  # uncommitted: invisible
    assert (
        store.canonical(OBJ_C) == OBJ_A
    )  # committed: cache invalidated via data_version
    with pytest.raises(sqlite3.OperationalError):
        store._dbs[0].conn.execute("DELETE FROM meta")
    store.close()
    w.close()


def test_aliases_and_merged_observations(synthetic):
    w = ImageStore.open_writer(synthetic["dir"])
    w.record_merges({OBJ_C: OBJ_B, OBJ_B: OBJ_A})  # chain C -> B -> A
    w.close()
    with ImageStore.open(synthetic["dir"]) as store:
        assert store.canonical(OBJ_C) == OBJ_A and store.canonical(OBJ_A) == OBJ_A
        assert store.members(OBJ_B) == {OBJ_A, OBJ_B, OBJ_C}
        assert store.members(make_node_id("O", 99)) == {make_node_id("O", 99)}
        obs = store.observations(OBJ_C)
        assert len(obs) == 5 and {o.node_id for o in obs} == {OBJ_A, OBJ_B, OBJ_C}
        assert store.object_nodes() == [OBJ_A]
        assert [t[1] for t in store.tracks(OBJ_A)] == [10, 11, 12]


def test_alias_cycle_is_defensive(synthetic):
    w = ImageStore.open_writer(synthetic["dir"])
    w.record_merges({OBJ_A: OBJ_B, OBJ_B: OBJ_A})
    w.close()
    with ImageStore.open(synthetic["dir"]) as store:
        assert store.canonical(OBJ_A) in (OBJ_A, OBJ_B)


def test_multi_store(synthetic, tmp_path):
    w = ImageStore.open_writer(tmp_path / "robot1", robot_id=1)
    sid = w.add_sensor("cam", 8, 6, 1, 1, 1, 1)
    fid = w.add_frame_ref(
        sid,
        500,
        rgb=write_image(tmp_path / "r1.jpg"),
        anchor_node_id=make_node_id("b", 0),
    )
    obj = make_node_id("O", 1 << 20)
    w.add_observation(7, fid, score=2.0, kept=True)
    w.bind_track(7, obj)
    w.record_merges({obj: OBJ_A})  # robot 1's object merged into robot 0's
    w.close()
    with ImageStore.open(synthetic["dir"], tmp_path / "robot1") as store:
        obs = store.observations(OBJ_A)
        assert (
            obs[0].robot_id == 1
            and obs[0].rgb_path == tmp_path / "robot1/frames/r1/500_rgb.jpg"
        )
        assert len(obs) == 3
        assert store.calib(sid).robot_id == 1
        assert len(store.frames()) == 5 and len(store.frames(robot_id=1)) == 1


def test_sam3_writer(tmp_path):
    w = ImageStore.open_writer(tmp_path / "st")
    sid = w.add_sensor("cam", 4, 2, 1, 1, 1, 1)
    fid = w.add_frame_ref(sid, 1)
    node = make_node_id("O", 40)
    m = np.array([[0, 1, 1, 0], [0, 0, 1, 1]], bool)
    oid = w.add_sam3_observation(node, fid, mask=m, bbox2d=(1, 0, 3, 1), kept=True)
    w.close()
    with ImageStore.open(tmp_path / "st") as store:
        (o,) = store.observations(node)
        assert o.obs_id == oid and o.track_id == sam3_track_id(node)
        assert o.mask_rle == "[1,2,3,2]" and o.mask_path.exists()
        np.testing.assert_array_equal(store.mask(o), m)


def test_pose_composition(synthetic):
    s = synthetic
    world_T_a0 = make_T(rot_z(0.7), (10, 5, 1))
    anchor_T_body = make_T(rot_z(0.1), (0.5, 0, 0))
    with ImageStore.open(s["dir"]) as store:
        skf = store.frame(s["skf"])
        expected = world_T_a0 @ anchor_T_body @ BODY_T_SENSOR
        np.testing.assert_allclose(
            store.frame_pose(skf, {AGENT0: world_T_a0}), expected
        )
        np.testing.assert_allclose(
            store.frame_pose(skf, lambda n: world_T_a0 if n == AGENT0 else None),
            expected,
        )
        T, src = store.frame_pose(skf, {}, with_source=True)  # anchor missing -> odom
        assert src == "odom"
        np.testing.assert_allclose(T, make_T(t=(1.5, 0, 0)) @ BODY_T_SENSOR)
        obs_frame = store.frame(s["obs_frame"])  # unanchored
        T, src = store.frame_pose(obs_frame, {AGENT0: world_T_a0}, with_source=True)
        assert src == "odom"

        def raising(n):
            raise KeyError(n)

        assert store.frame_pose(skf, raising, with_source=True)[1] == "odom"
        q = {"w": np.cos(0.35), "x": 0.0, "y": 0.0, "z": np.sin(0.35)}
        dsg = JsonDsg(
            {
                "nodes": [
                    {
                        "id": AGENT0,
                        "layer": 2,
                        "partition": 97,
                        "attributes": {"position": [10, 5, 1], "world_R_body": q},
                    }
                ]
            }
        )
        np.testing.assert_allclose(store.frame_pose(skf, dsg), expected, atol=1e-12)


def test_pose_through_merged_anchor(synthetic):
    w = ImageStore.open_writer(synthetic["dir"])
    w.record_merges({AGENT0: AGENT1})
    w.close()
    with ImageStore.open(synthetic["dir"]) as store:
        T = store.frame_pose(
            store.frame(synthetic["kf0"]), {AGENT1: make_T(t=(9, 9, 9))}
        )
        np.testing.assert_allclose(T, make_T(t=(9, 9, 9)) @ BODY_T_SENSOR)


def test_pose_from_spark_dsg(synthetic):
    spark_dsg = pytest.importorskip("spark_dsg")
    G = spark_dsg.DynamicSceneGraph()
    attrs = spark_dsg.AgentNodeAttributes()
    attrs.position = np.array([10.0, 5.0, 1.0])
    attrs.world_R_body = spark_dsg.Quaternion(np.cos(0.35), 0.0, 0.0, np.sin(0.35))
    G.add_node(spark_dsg.DsgLayers.AGENTS, AGENT0, attrs)
    with ImageStore.open(synthetic["dir"]) as store:
        skf = store.frame(synthetic["skf"])
        expected = (
            make_T(rot_z(0.7), (10, 5, 1))
            @ make_T(rot_z(0.1), (0.5, 0, 0))
            @ BODY_T_SENSOR
        )
        T, src = store.frame_pose(skf, G, with_source=True)
        assert src == "anchor"
        np.testing.assert_allclose(T, expected, atol=1e-9)
