import json
import os
import pathlib

import numpy as np
import pytest
from conftest import BODY_T_SENSOR, make_T, rot_z, write_image
from hydra_image_store.cli import main
from hydra_image_store.legacy import import_legacy

from hydra_image_store import (
    LEGACY_TRACK_BASE,
    FrameKind,
    ImageStore,
    JsonDsg,
    make_node_id,
    rle_decode,
    rle_encode,
    sam3_track_id,
)

A0, A1 = make_node_id("a", 0), make_node_id("a", 1)
REAL_RUN = pathlib.Path.home() / "adt4_output/bldg45f6_full_tour_hydra_v2"


def _snapshot(root):
    out = {}
    for dirpath, dirs, files in os.walk(root):
        for name in dirs + files:
            p = os.path.join(dirpath, name)
            st = os.lstat(p)
            out[p] = (st.st_mtime_ns, st.st_size, os.path.islink(p))
    return out


def _quat(theta):
    return {
        "w": float(np.cos(theta / 2)),
        "x": 0.0,
        "y": 0.0,
        "z": float(np.sin(theta / 2)),
    }


@pytest.fixture
def legacy_run(tmp_path):
    run = tmp_path / "run"
    calib = {
        "fx": 5.0,
        "fy": 5.0,
        "cx": 4,
        "cy": 3,
        "width": 8,
        "height": 6,
        "depth_scale": 0.001,
        "body_T_sensor": BODY_T_SENSOR.reshape(16).tolist(),
    }
    for d in ("agents", "subkeyframes"):
        (run / d).mkdir(parents=True)
        (run / d / "camera_calib.json").write_text(json.dumps(calib))

    def agent(ts, frame_ts, x):
        stem = run / "agents" / f"agent_{ts}"
        write_image(f"{stem}_rgb.jpg")
        write_image(f"{stem}_depth.png", mode="I;16")
        meta = {
            "timestamp_ns": ts,
            "frame_timestamp_ns": frame_ts,
            "world_T_body": make_T(t=(x, 0, 0)).reshape(16).tolist(),
            "rgb_file": f"agent_{ts}_rgb.jpg",
            "depth_file": f"agent_{ts}_depth.png",
            "calib": "camera_calib.json",
        }
        pathlib.Path(f"{stem}_meta.json").write_text(json.dumps(meta))

    def subkf(ts):
        stem = run / "subkeyframes" / f"subkf_{ts}"
        write_image(f"{stem}_rgb.jpg")
        write_image(f"{stem}_depth.png", mode="I;16")
        pathlib.Path(f"{stem}_meta.json").write_text(
            json.dumps(
                {
                    "timestamp_ns": ts,
                    "rgb_file": f"subkf_{ts}_rgb.jpg",
                    "depth_file": f"subkf_{ts}_depth.png",
                    "calib": "camera_calib.json",
                }
            )
        )

    agent(100, 100, 1.0)
    agent(200, 205, 2.0)
    for ts in (100, 150, 260):
        subkf(ts)

    mask = np.zeros((6, 8), bool)
    mask[1:3, 2:7] = True
    obj = run / "images/O_3"
    write_image(obj / "frame_160_rgb.jpg")
    write_image(obj / "frame_160_depth.png", mode="L")
    write_image(obj / "frame_160_mask.png", mode="L", value=0)
    from PIL import Image

    Image.fromarray(mask.astype(np.uint8) * 255).save(obj / "frame_160_mask.png")
    (obj / "frame_160_meta.json").write_text(
        json.dumps(
            {
                "timestamp_ns": 160,
                "bbox_min": [0, 0, 0],
                "bbox_max": [1, 2, 3],
                "bbox_2d": {"min_x": 2, "min_y": 1, "max_x": 6, "max_y": 2},
                "mask_file": "frame_160_mask.png",
                "mask_rle": rle_encode(mask),
            }
        )
    )
    orphan = run / "images/O_4"
    write_image(orphan / "frame_270_rgb.jpg")
    (orphan / "frame_270_meta.json").write_text(json.dumps({"timestamp_ns": 270}))
    (run / "images/temp").mkdir()

    sam_mask = run / "sam3_masks/sam3_205_a_cup_1-1-3-3_mask.png"
    sam_mask.parent.mkdir()
    Image.fromarray(mask.astype(np.uint8) * 255).save(sam_mask)
    sam = run / "sam3_objects/O7"
    sam.mkdir(parents=True)
    os.symlink(run / "agents/agent_200_rgb.jpg", sam / "frame_205_rgb.jpg")
    os.symlink(sam_mask, sam / "frame_205_mask.png")
    (sam / "frame_205_meta.json").write_text(
        json.dumps(
            {
                "timestamp_ns": 205,
                "rgb_file": "frame_205_rgb.jpg",
                "mask_file": "frame_205_mask.png",
                "bbox_2d": {"min_x": 1, "min_y": 1, "max_x": 3, "max_y": 3},
            }
        )
    )

    nodes = [
        {
            "id": A0,
            "layer": 2,
            "partition": 97,
            "attributes": {
                "type": "AgentNodeAttributes",
                "timestamp": 100,
                "position": [10, 0, 0],
                "world_R_body": _quat(0.5),
                "image_folder": str(run / "agents/agent_100"),
            },
        },
        {
            "id": A1,
            "layer": 2,
            "partition": 97,
            "attributes": {
                "type": "AgentNodeAttributes",
                "timestamp": 200,
                "position": [20, 0, 0],
                "world_R_body": _quat(0.0),
            },
        },
        {
            "id": make_node_id("s", 0),
            "layer": 2,
            "partition": 115,
            "attributes": {
                "type": "SubKeyframeNodeAttributes",
                "timestamp": 150,
                "anchor_node_id": A0,
                "anchor_R_subframe": _quat(0.2),
                "anchor_t_subframe": [0.5, 0, 0],
                "position": [0, 0, 0],
                "image_folder": "subkeyframes/subkf_150",
            },
        },
        {
            "id": make_node_id("O", 30),
            "layer": 2,
            "partition": 0,
            "attributes": {
                "type": "KhronosObjectAttributes",
                "position": [0, 0, 0],
                "image_folder": "images/O_3",
            },
        },
    ]
    (run / "hydra/backend").mkdir(parents=True)
    (run / "hydra/backend/dsg.json").write_text(
        json.dumps({"nodes": nodes, "edges": []})
    )
    return run, mask


def test_import_legacy_fixture(legacy_run, tmp_path):
    run, mask = legacy_run
    before = _snapshot(run)
    out = tmp_path / "store"
    stats = import_legacy(run, out)
    assert _snapshot(run) == before  # source untouched
    assert (
        stats["keyframes"] == 2
        and stats["subkeyframes"] == 3
        and stats["subkeyframes_deduped"] == 1
    )
    assert (
        stats["subkeyframes_dsg_anchor"] == 1
        and stats["subkeyframes_nearest_anchor"] == 2
    )
    assert (
        stats["objects"] == 2
        and stats["object_dirs_without_node"] == 1
        and stats["object_dirs_skipped"] == 1
    )
    assert stats["sam3_objects"] == 1 and stats["sam3_observations"] == 1

    with ImageStore.open(out) as store:
        kf = store.frames_for_agent(A1, include_subkeyframes=False)
        assert [f.timestamp_ns for f in kf] == [
            205
        ]  # frame_timestamp_ns, bound by node timestamp
        a0 = store.frames_for_agent(A0)
        assert [(f.kind, f.timestamp_ns) for f in a0] == [
            (FrameKind.KEYFRAME, 100),
            (FrameKind.SUBKEYFRAME, 150),
        ]
        assert (
            a0[0].rgb_path.stat().st_ino
            == (run / "agents/agent_100_rgb.jpg").stat().st_ino
        )  # hardlinked
        # sub-keyframe 150: DSG anchor + anchor_T_body
        dsg = JsonDsg.load(run / "hydra/backend/dsg.json")
        expected = (
            make_T(rot_z(0.5), (10, 0, 0))
            @ make_T(rot_z(0.2), (0.5, 0, 0))
            @ BODY_T_SENSOR
        )
        np.testing.assert_allclose(store.frame_pose(a0[1], dsg), expected, atol=1e-12)
        # sub-keyframe 260 (no DSG node) anchors to the nearest keyframe (205 -> A1)
        (late,) = store.frames(t0=260, t1=260)
        assert late.anchor_node_id == A1
        np.testing.assert_allclose(late.world_T_body_odom, make_T(t=(2, 0, 0)))
        # Khronos folder O_3 bound through image_folder to O30
        (obs,) = store.observations(make_node_id("O", 30))
        assert (
            obs.track_id == LEGACY_TRACK_BASE | 3
            and obs.kept
            and obs.bbox2d == (2, 1, 6, 2)
        )
        np.testing.assert_array_equal(obs.bbox3d[1], [1, 2, 3])
        np.testing.assert_array_equal(store.mask(obs), mask)
        assert (
            obs.frame.kind == FrameKind.OBSERVATION and obs.frame.anchor_node_id == A0
        )  # nearest = subkf 150
        np.testing.assert_allclose(
            obs.frame.anchor_T_body, make_T(rot_z(0.2), (0.5, 0, 0)), atol=1e-12
        )
        # orphan folder bound to the symbol its name encodes
        assert len(store.observations(make_node_id("O", 4))) == 1
        # SAM3 object reuses the keyframe it was detected in
        (s,) = store.observations(make_node_id("O", 7))
        assert (
            s.track_id == sam3_track_id(make_node_id("O", 7))
            and s.frame.kind == FrameKind.KEYFRAME
        )
        assert (
            s.rgb_path == kf[0].rgb_path
            and s.mask_path.exists()
            and not s.mask_path.is_symlink()
        )
        np.testing.assert_array_equal(store.mask(s), mask)


def test_import_legacy_cli_without_dsg_info(legacy_run, tmp_path, capsys):
    run, _ = legacy_run
    (run / "hydra/backend/dsg.json").unlink()
    assert (
        main(
            [
                "import-legacy",
                "--run-dir",
                str(run),
                "--out",
                str(tmp_path / "s"),
                "--copy",
            ]
        )
        == 0
    )
    stats = json.loads(capsys.readouterr().out)
    assert stats["keyframes_unbound"] == 2 and stats["subkeyframes_unanchored"] == 3
    with ImageStore.open(tmp_path / "s") as store:
        (obs,) = store.observations(make_node_id("O", 3))
        assert (
            obs.frame.anchor_node_id is None
        )  # odometry of the nearest frame still applies
        np.testing.assert_allclose(obs.frame.world_T_body_odom, make_T(t=(1, 0, 0)))
        f = store.frames(t0=100, t1=100)[0]
        assert (
            f.rgb_path.stat().st_ino != (run / "agents/agent_100_rgb.jpg").stat().st_ino
        )


@pytest.mark.skipif(not REAL_RUN.exists(), reason="real legacy run not available")
def test_import_legacy_real_run_smoke(tmp_path):
    stats = import_legacy(REAL_RUN, tmp_path / "store")
    assert (
        stats["keyframes"] == 638
        and stats["subkeyframes"] == 4277
        and stats["objects"] == 29
    )
    with ImageStore.open(tmp_path / "store") as store:
        obj0 = make_node_id("O", 0)
        (obs,) = store.observations(obj0)
        known = (
            np.array(
                __import__("PIL.Image", fromlist=["Image"]).open(
                    REAL_RUN / "images/O_0/frame_1788127479836547276_mask.png"
                )
            )
            > 0
        )
        decoded = rle_decode(obs.mask_rle, (360, 640))
        np.testing.assert_array_equal(decoded, known)
        np.testing.assert_array_equal(store.mask(obs), known)
        info = store.info()[0]
        assert info["frames_unanchored"] == 0
        dsg = JsonDsg.load(REAL_RUN / "hydra/backend/dsg.json")
        for node in store.agent_nodes()[:20]:
            (kf,) = store.frames_for_agent(node, include_subkeyframes=False)
            T, src = store.frame_pose(kf, dsg, with_source=True)
            assert src == "anchor"
            np.testing.assert_allclose(
                T[:3, 3],
                (dsg.pose(node) @ store.calib(kf.sensor_id).body_T_sensor)[:3, 3],
            )
    assert (
        main(
            [
                "export",
                str(tmp_path / "store"),
                "--dsg",
                str(REAL_RUN / "hydra/backend/dsg.json"),
                "--out",
                str(tmp_path / "export"),
                "--kept-only",
            ]
        )
        == 0
    )
    assert (tmp_path / "export/O_0/frame_1788127479836547276_rgb.jpg").exists()


def test_strip_legacy_dsg(tmp_path):
    from hydra_image_store.legacy import strip_legacy_dsg

    src = tmp_path / "dsg.json"
    data = {
        "layer_keys": [{"layer": 2, "partition": 0}, {"layer": 2, "partition": 115}],
        "layer_names": {"AGENTS": {"layer": 2, "partition": 0}},
        "nodes": [
            {
                "id": 1,
                "layer": 2,
                "partition": 0,
                "attributes": {
                    "type": "AgentNodeAttributes",
                    "image_folder": "agents/agent_1",
                },
            },
            {
                "id": 2,
                "layer": 2,
                "partition": 115,
                "attributes": {
                    "type": "SubKeyframeNodeAttributes",
                    "image_folder": "x",
                },
            },
        ],
        "edges": [{"source": 1, "target": 2, "info": {}}],
    }
    src.write_text(json.dumps(data))
    dst = tmp_path / "out.json"
    stats = strip_legacy_dsg(src, dst)
    out = json.loads(dst.read_text())
    assert stats == {
        "subkeyframe_nodes_removed": 1,
        "edges_removed": 1,
        "image_folders_removed": 1,
    }
    assert [n["id"] for n in out["nodes"]] == [1]
    assert "image_folder" not in out["nodes"][0]["attributes"]
    assert out["edges"] == []
    assert out["layer_keys"] == [{"layer": 2, "partition": 0}]
