import json

import numpy as np

from conftest import AGENT0, AGENT1, OBJ_A, OBJ_B, OBJ_C, make_T, rot_z
from hydra_image_store import ImageStore
from hydra_image_store.cli import main
from hydra_image_store.gc import gc_store


def _dsg_json(path):
    def q(theta):
        return {"w": float(np.cos(theta / 2)), "x": 0.0, "y": 0.0, "z": float(np.sin(theta / 2))}

    nodes = [
        {"id": AGENT0, "layer": 2, "partition": 97, "attributes": {"position": [10, 5, 1], "world_R_body": q(0.7), "timestamp": 100}},
        {"id": AGENT1, "layer": 2, "partition": 97, "attributes": {"position": [20, 5, 1], "world_R_body": q(0.0), "timestamp": 200}},
    ]
    path.write_text(json.dumps({"nodes": nodes, "edges": []}))
    return path


def test_export_layout(synthetic, tmp_path):
    w = ImageStore.open_writer(synthetic["dir"])
    w.record_merges({OBJ_C: OBJ_A})
    w.close()
    dsg = _dsg_json(tmp_path / "dsg.json")
    out = tmp_path / "export"
    assert main(["export", str(synthetic["dir"]), "--dsg", str(dsg), "--out", str(out)]) == 0
    names = sorted(p.name for p in out.iterdir())
    assert names == ["O_1", "O_2", "agent_100", "agent_200", "index.json"]
    # merged child C's observation (at kf1, ts 200) lives under its parent A
    assert sorted(p.name for p in (out / "O_1").glob("*_meta.json")) == [
        "frame_100_meta.json", "frame_160_meta.json", "frame_200_meta.json"]
    meta = json.loads((out / "O_1/frame_100_meta.json").read_text())
    assert meta["bbox_2d"] == {"min_x": 3, "min_y": 2, "max_x": 5, "max_y": 3}
    assert meta["bbox_min"] == [0, 0, 0] and meta["pose_source"] == "anchor"
    assert meta["calib"]["fx"] == 5.0 and meta["node_id"] == OBJ_A
    rgb = out / "O_1" / meta["rgb_file"]
    with ImageStore.open(synthetic["dir"]) as store:
        f = store.frame(synthetic["kf0"])
        assert rgb.stat().st_ino == f.rgb_path.stat().st_ino  # hardlink
        np.testing.assert_allclose(np.reshape(meta["world_T_sensor"], (4, 4)), store.frame_pose(f, {AGENT0: make_T(rot_z(0.7), (10, 5, 1))}))
    assert (out / "O_1/frame_100_mask.png").exists()
    assert sorted(p.name for p in (out / "agent_100").iterdir()) == [
        "frame_100_depth.png", "frame_100_meta.json", "frame_100_rgb.jpg", "frame_150_meta.json", "frame_150_rgb.jpg"]
    sub = json.loads((out / "agent_100/frame_150_meta.json").read_text())
    assert sub["frame_kind"] == "subkeyframe" and sub["anchor_node_id"] == AGENT0
    index = json.loads((out / "index.json").read_text())
    assert index["objects"]["O_1"] == OBJ_A and index["agents"]["agent_200"] == AGENT1


def test_export_copy_and_kept_only(synthetic, tmp_path):
    out = tmp_path / "export"
    main(["export", str(synthetic["dir"]), "--out", str(out), "--copy", "--kept-only"])
    assert not (out / "O_3").exists()  # only an unkept observation
    meta = json.loads((out / "O_1/frame_100_meta.json").read_text())
    assert meta["pose_source"] == "odom"
    with ImageStore.open(synthetic["dir"]) as store:
        assert (out / "O_1/frame_100_rgb.jpg").stat().st_ino != store.frame(synthetic["kf0"]).rgb_path.stat().st_ino


def test_gc_keep_n_over_merged_set(synthetic):
    root = synthetic["dir"]
    w = ImageStore.open_writer(root)
    w.record_merges({OBJ_B: OBJ_A})
    (root / "frames/r0/stray.jpg").write_bytes(b"x")
    dry = gc_store(w, 1, orphans=True, dry_run=True)
    assert dry["observations_unkept"] == 2 and dry["orphans"] == 1
    with ImageStore.open(root) as store:
        assert sum(o.kept for o in store.observations(OBJ_A)) == 3  # dry run changed nothing
    assert (root / "frames/r0/stray.jpg").exists()
    stats = gc_store(w, 1, orphans=True)
    w.close()
    assert stats["observations_unkept"] == 2 and stats["masks_dropped"] == 2
    assert stats["frames_dropped"] == 1 and stats["orphans"] == 1
    with ImageStore.open(root) as store:
        obs = store.observations(OBJ_A)
        assert [o.obs_id for o in obs if o.kept] == [synthetic["obs"]["a1"]]
        assert obs[0].mask_path.exists()
        assert all(o.mask_path is None for o in obs[1:])
        assert store.frame(synthetic["obs_frame"]).rgb_path is None
        assert store.frame(synthetic["kf1"]).rgb_path.exists()  # keyframes untouched
        assert store.info()[0]["meta"]["max_images_per_object"] == "1"
    assert not (root / "frames/r0/stray.jpg").exists()
    assert not (root / "frames/r0/160_rgb.jpg").exists()


def test_cli_info(synthetic, capsys):
    assert main(["info", str(synthetic["dir"])]) == 0
    out = capsys.readouterr().out
    assert "observations 5 (kept 3)" in out and "keyframe 2" in out
    main(["info", "--json", str(synthetic["dir"])])
    assert json.loads(capsys.readouterr().out)[0]["tracks"] == 3
