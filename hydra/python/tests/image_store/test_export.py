import json

import numpy as np
from conftest import AGENT0, AGENT1, OBJ_A, OBJ_C, make_T, rot_z
from hydra_python.image_store import ImageStore
from hydra_python.image_store.cli import main


def _dsg_json(path):
    def q(theta):
        return {
            "w": float(np.cos(theta / 2)),
            "x": 0.0,
            "y": 0.0,
            "z": float(np.sin(theta / 2)),
        }

    nodes = [
        {
            "id": AGENT0,
            "layer": 2,
            "partition": 97,
            "attributes": {
                "position": [10, 5, 1],
                "world_R_body": q(0.7),
                "timestamp": 100,
            },
        },
        {
            "id": AGENT1,
            "layer": 2,
            "partition": 97,
            "attributes": {
                "position": [20, 5, 1],
                "world_R_body": q(0.0),
                "timestamp": 200,
            },
        },
    ]
    path.write_text(json.dumps({"nodes": nodes, "edges": []}))
    return path


def test_export_layout(synthetic, tmp_path):
    w = ImageStore.open_writer(synthetic["dir"])
    w.record_merges({OBJ_C: OBJ_A})
    w.close()
    dsg = _dsg_json(tmp_path / "dsg.json")
    out = tmp_path / "export"
    assert (
        main(["export", str(synthetic["dir"]), "--dsg", str(dsg), "--out", str(out)])
        == 0
    )
    assert sorted(p.name for p in out.iterdir()) == ["agents", "index.json", "objects"]
    assert sorted(p.name for p in (out / "objects").iterdir()) == ["O_1", "O_2"]
    assert sorted(p.name for p in (out / "agents").iterdir()) == ["a_0", "a_1"]
    # merged child C's observation (at kf1, ts 200) lives under its parent A
    assert sorted(p.name for p in (out / "objects/O_1").glob("*_meta.json")) == [
        "100_meta.json",
        "160_meta.json",
        "200_meta.json",
    ]
    meta = json.loads((out / "objects/O_1/100_meta.json").read_text())
    assert meta["bbox_2d"] == {"min_x": 3, "min_y": 2, "max_x": 5, "max_y": 3}
    assert meta["bbox_min"] == [0, 0, 0] and meta["pose_source"] == "anchor"
    assert meta["calib"]["fx"] == 5.0 and meta["node_id"] == OBJ_A
    rgb = out / "objects/O_1" / meta["rgb_file"]
    with ImageStore.open(synthetic["dir"]) as store:
        f = store.frame(synthetic["kf0"])
        assert rgb.stat().st_ino == f.rgb_path.stat().st_ino  # hardlink
        np.testing.assert_allclose(
            np.reshape(meta["world_T_sensor"], (4, 4)),
            store.frame_pose(f, {AGENT0: make_T(rot_z(0.7), (10, 5, 1))}),
        )
    assert (out / "objects/O_1/100_mask.png").exists()  # decoded from the RLE
    assert sorted(p.name for p in (out / "agents/a_0").iterdir()) == [
        "100_depth.png",
        "100_meta.json",
        "100_rgb.jpg",
        "150_meta.json",
        "150_rgb.jpg",
    ]
    second = json.loads((out / "agents/a_0/150_meta.json").read_text())
    assert second["frame_kind"] == "keyframe" and second["anchor_node_id"] == AGENT0
    index = json.loads((out / "index.json").read_text())
    assert index["objects"]["objects/O_1"] == OBJ_A
    assert index["agents"]["agents/a_1"] == AGENT1


def test_export_copy_and_kept_only(synthetic, tmp_path):
    out = tmp_path / "export"
    main(["export", str(synthetic["dir"]), "--out", str(out), "--copy", "--kept-only"])
    assert not (out / "objects/O_3").exists()  # only an unkept observation
    meta = json.loads((out / "objects/O_1/100_meta.json").read_text())
    assert meta["pose_source"] == "odom"
    with ImageStore.open(synthetic["dir"]) as store:
        assert (out / "objects/O_1/100_rgb.jpg").stat().st_ino != store.frame(
            synthetic["kf0"]
        ).rgb_path.stat().st_ino
