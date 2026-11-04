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
from hydra_python.image_store import (
    EXTERNAL_TRACK_BASE,
    FrameKind,
    ImageStore,
    JsonDsg,
    SchemaVersionError,
    make_id,
    make_node_id,
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
        assert o.rgb_path == f.rgb_path and o.mask_path is None
        np.testing.assert_array_equal(store.mask(o), s["mask"])
        assert [x.timestamp_ns for x in store.observations(OBJ_B, order="time")] == [
            150,
            200,
        ]
        assert len(store.observations(OBJ_B, kept_only=True)) == 1
        assert len(store.observations(OBJ_A, limit=1)) == 1
        assert [x.frame_id for x in store.frames_for_agent(AGENT0)] == [
            s["kf0"],
            s["mid"],
        ]
        assert [x.frame_id for x in store.frames_for_agent(AGENT0, 140)] == [
            s["mid"],
            s["kf0"],
        ]
        assert store.agent_frame(AGENT0, 100).frame_id == s["kf0"]
        assert store.agent_frame(AGENT0, 149).frame_id == s["mid"]
        assert store.agent_frame(AGENT0, 100 + 10**8) is None  # nothing within 50 ms
        assert store.agent_frame(AGENT1, 200).frame_id == s["kf1"]
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
    fid = w.add_frame_ref(s["sensor"], 300, kind=FrameKind.KEYFRAME)
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
            f.kind == FrameKind.KEYFRAME
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


def test_external_writer(tmp_path):
    """A tool adding its own observation of a node, with a mask and a mask image."""
    w = ImageStore.open_writer(tmp_path / "st")
    sid = w.add_sensor("cam", 4, 2, 1, 1, 1, 1)
    fid = w.add_frame_ref(sid, 1)
    node = make_node_id("O", 40)
    track = EXTERNAL_TRACK_BASE | 40
    m = np.array([[0, 1, 1, 0], [0, 0, 1, 1]], bool)
    png = write_image(tmp_path / "mask.png", w=4, h=2, mode="L", value=255)
    with w.batch():
        w.bind_track(track, node)
        oid = w.add_observation(track, fid, mask=m, bbox2d=(1, 0, 3, 1), kept=True)
        oid2 = w.add_observation(track, fid, mask_path=png, link="copy")
    w.close()
    with ImageStore.open(tmp_path / "st") as store:
        o, o2 = store.observations(node, order="time")
        assert (o.obs_id, o2.obs_id) == (oid, oid2) and o.track_id == track
        assert o.mask_rle == "[1,2,3,2]" and o.mask_path is None
        np.testing.assert_array_equal(store.mask(o), m)
        assert o2.mask_path == tmp_path / f"st/masks/r0/{oid2}.png"
        assert store.mask(o2).all()


def test_unknown_frame_kind_is_rejected(synthetic):
    conn = sqlite3.connect(synthetic["dir"] / "store.sqlite")
    conn.execute("UPDATE frames SET kind = 7 WHERE frame_id = ?", (synthetic["mid"],))
    conn.commit()
    conn.close()
    with ImageStore.open(synthetic["dir"]) as store:
        assert store.frame(synthetic["kf0"]).kind == FrameKind.KEYFRAME
        with pytest.raises(ValueError, match="unknown frame kind 7"):
            store.frame(synthetic["mid"])
        with pytest.raises(ValueError, match="unknown frame kind 7"):
            store.info()


def test_cli_reports_unknown_frame_kind(synthetic, capsys):
    from hydra_python.image_store.cli import main

    conn = sqlite3.connect(synthetic["dir"] / "store.sqlite")
    conn.execute("UPDATE frames SET kind = 7 WHERE frame_id = ?", (synthetic["mid"],))
    conn.commit()
    conn.close()
    assert main(["check", str(synthetic["dir"])]) == 2
    err = capsys.readouterr().err
    assert "Error:" in err and "unknown frame kind 7" in err and "Traceback" not in err


def test_unknown_link_mode(tmp_path):
    w = ImageStore.open_writer(tmp_path / "st")
    sid = w.add_sensor("cam", 8, 6, 1, 1, 1, 1)
    with pytest.raises(ValueError):
        w.add_frame_ref(sid, 1, rgb=write_image(tmp_path / "a.jpg"), link="symlink")
    w.close()


def test_pose_composition(synthetic):
    s = synthetic
    world_T_a0 = make_T(rot_z(0.7), (10, 5, 1))
    anchor_T_body = make_T(rot_z(0.1), (0.5, 0, 0))
    with ImageStore.open(s["dir"]) as store:
        mid = store.frame(s["mid"])
        expected = world_T_a0 @ anchor_T_body @ BODY_T_SENSOR
        np.testing.assert_allclose(
            store.frame_pose(mid, {AGENT0: world_T_a0}), expected
        )
        np.testing.assert_allclose(
            store.frame_pose(mid, lambda n: world_T_a0 if n == AGENT0 else None),
            expected,
        )
        T, src = store.frame_pose(mid, {}, with_source=True)  # anchor missing -> odom
        assert src == "odom"
        np.testing.assert_allclose(T, make_T(t=(1.5, 0, 0)) @ BODY_T_SENSOR)
        obs_frame = store.frame(s["obs_frame"])  # unanchored
        T, src = store.frame_pose(obs_frame, {AGENT0: world_T_a0}, with_source=True)
        assert src == "odom"

        def raising(n):
            raise KeyError(n)

        assert store.frame_pose(mid, raising, with_source=True)[1] == "odom"
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
        np.testing.assert_allclose(store.frame_pose(mid, dsg), expected, atol=1e-12)


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
        mid = store.frame(synthetic["mid"])
        expected = (
            make_T(rot_z(0.7), (10, 5, 1))
            @ make_T(rot_z(0.1), (0.5, 0, 0))
            @ BODY_T_SENSOR
        )
        T, src = store.frame_pose(mid, G, with_source=True)
        assert src == "anchor"
        np.testing.assert_allclose(T, expected, atol=1e-9)


def test_python_writer_names_frames_like_cpp(tmp_path):
    """Second and later sensors get the ``_s<seq>`` suffix, so two sensors
    with the same stamp never share an image file."""
    src = write_image(tmp_path / "a.jpg")
    w = ImageStore.open_writer(tmp_path / "st", robot_id=2)
    s0 = w.add_sensor("front", 8, 6, 5, 5, 4, 3)
    s1 = w.add_sensor("left", 8, 6, 5, 5, 4, 3)
    f0 = w.add_frame_ref(s0, 42, rgb=src, link="copy")
    f1 = w.add_frame_ref(s1, 42, rgb=src, link="copy")
    reader = w.reader()
    assert reader.frame(f0).rgb_path.name == "42_rgb.jpg"
    assert reader.frame(f1).rgb_path.name == "42_s1_rgb.jpg"
    assert reader.frame(f0).rgb_path.parent.name == "r2"
    w.close()


def test_open_does_not_create_a_store(tmp_path):
    with pytest.raises(FileNotFoundError):
        ImageStore.open(tmp_path / "nope", readonly=False)
    with pytest.raises(FileNotFoundError):
        ImageStore.open_writer(tmp_path / "nope", create=False)
    assert not (tmp_path / "nope").exists()
    with pytest.raises(ValueError):
        ImageStore.open(tmp_path / "nope", create=True)
    ImageStore.open(tmp_path / "new", readonly=False, create=True).close()
    assert (tmp_path / "new" / "store.sqlite").exists()
