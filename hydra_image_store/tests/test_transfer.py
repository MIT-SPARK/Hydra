import io
import json
import tarfile

import numpy as np
import pytest

from conftest import BODY_T_SENSOR, make_T

from hydra_image_store import FrameKind, ImageStore, check_store, make_node_id, pack_store, unpack_store
from hydra_image_store import transfer

pytest.importorskip("zstandard")
PIL = pytest.importorskip("PIL.Image")

W, H = 64, 37  # odd height exercises the ceil() of the half-resolution size


def _depth(seed=0):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:H, 0:W]
    d = (1500 + 10 * xx + 5 * yy + rng.integers(0, 20, (H, W))).astype(np.uint16)
    d[:5, :7] = 0  # no measurement
    d[10, 10] = 65535  # wraps in the row delta
    return d


def _rgb(seed=0):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:H, 0:W]
    img = np.stack([xx * 4, yy * 6, (xx + yy) * 2], -1) + rng.integers(0, 8, (H, W, 3))
    return np.clip(img, 0, 255).astype(np.uint8)


@pytest.fixture
def run(tmp_path):
    """A run directory with a 64x37 store: 3 keyframes, 1 sub-keyframe, 1 kept observation."""
    src = tmp_path / "src"
    src.mkdir()
    run = tmp_path / "run"
    w = ImageStore.open_writer(run / "image_store", robot_id=3)
    sid = w.add_sensor("cam", W, H, 50.0, 51.0, 31.5, 18.0, 0.001, BODY_T_SENSOR)
    frames = []
    for i, (kind, ts) in enumerate([(FrameKind.KEYFRAME, 100), (FrameKind.KEYFRAME, 200),
                                     (FrameKind.SUBKEYFRAME, 250), (FrameKind.KEYFRAME, 300),
                                     (FrameKind.OBSERVATION, 320)]):
        rgb, depth = src / f"{i}.jpg", src / f"{i}.png"
        PIL.fromarray(_rgb(i)).save(rgb, quality=95)
        PIL.fromarray(_depth(i)).save(depth)
        anchor = make_node_id("a", i) if kind != FrameKind.OBSERVATION else None
        frames.append(w.add_frame_ref(sid, ts, kind=kind, world_T_body_odom=make_T(t=(i, 0, 0)),
                                      rgb=rgb, depth=depth, anchor_node_id=anchor, link="copy"))
    mask = np.zeros((H, W), bool)
    mask[5:20, 10:30] = True
    obj = make_node_id("O", 7)
    w.add_sam3_observation(obj, frames[4], mask=mask, score=0.8, kept=True, bbox2d=(10, 5, 29, 19))
    w.add_sam3_observation(obj, frames[2], mask=mask, score=0.3)
    w.close()
    return {"dir": run, "frames": frames, "sensor": sid, "obj": obj}


def test_delta_zstd_roundtrip():
    d = _depth()
    assert np.array_equal(transfer.delta_zstd_decode(transfer.delta_zstd_encode(d), d.shape), d)


def test_downscale_restore_and_calib():
    d = _depth()
    half = transfer.downscale_depth(d)
    assert half.shape == (19, 32)
    assert np.array_equal(half, d[::2, ::2])
    full = transfer.restore_depth(half, 2, d.shape)
    assert full.shape == d.shape and np.array_equal(full[::2, ::2], d[::2, ::2])
    c = transfer.scale_calib({"fx": 50.0, "fy": 51.0, "cx": 31.5, "cy": 18.0, "width": W, "height": H}, 2)
    assert (c["fx"], c["fy"], c["cx"], c["cy"], c["width"], c["height"]) == (25.0, 25.5, 15.75, 9.0, 32, 19)
    # a half-resolution pixel back-projects to the same ray as its full-resolution source
    u, v = 7, 5
    ray_half = np.array([(u - c["cx"]) / c["fx"], (v - c["cy"]) / c["fy"]])
    ray_full = np.array([(2 * u - 31.5) / 50.0, (2 * v - 18.0) / 51.0])
    assert np.allclose(ray_half, ray_full)


@pytest.mark.parametrize("mode", ["none", "lossless", "fast"])
def test_encode_decode_modes(run, mode):
    store = ImageStore.open(run["dir"] / "image_store")
    frame = store.frame(run["frames"][0])
    blob = transfer.encode_frame(store, frame, mode)
    dec = transfer.decode_frame(blob)
    meta = dec["meta"]
    assert meta["frame_id"] == frame.frame_id and meta["timestamp_ns"] == 100
    assert meta["kind"] == 0 and meta["anchor_node_id"] == make_node_id("a", 0)
    assert np.allclose(np.reshape(meta["world_T_body_odom"], (4, 4)), frame.world_T_body_odom)
    assert meta["calib"]["fx"] == 50.0 and meta["calib"]["width"] == W
    assert np.allclose(np.reshape(meta["calib"]["body_T_sensor"], (4, 4)), BODY_T_SENSOR)
    assert dec["rgb_format"] == "jpeg"
    stored_rgb = frame.rgb_path.read_bytes()
    src_depth = np.asarray(PIL.open(frame.depth_path))
    if mode == "fast":
        assert dec["depth_downscale"] == 2 and dec["depth"].shape == (19, 32)
        assert np.array_equal(dec["depth"], src_depth[::2, ::2])
        assert meta["depth_calib"]["fx"] == 25.0 and meta["depth_calib"]["width"] == 32
        assert meta["rgb_quality"] == 75 and dec["rgb"] != stored_rgb
        assert np.array_equal(transfer.full_res_depth(dec)[::2, ::2], src_depth[::2, ::2])
    else:
        assert dec["rgb"] == stored_rgb
        assert dec["depth_downscale"] == 1 and np.array_equal(dec["depth"], src_depth)
        assert meta["depth_calib"]["fx"] == 50.0
    if mode == "none":
        assert dec["depth_png"] == frame.depth_path.read_bytes()


def test_encode_by_id_and_bad_input(run):
    store = ImageStore.open(run["dir"] / "image_store")
    assert transfer.decode_header(transfer.encode_frame(store, run["frames"][1], "none"))["meta"]["timestamp_ns"] == 200
    with pytest.raises(ValueError):
        transfer.encode_frame(store, run["frames"][1], "zip")
    with pytest.raises(ValueError):
        transfer.decode_frame(b"XXXX" + bytes(20))


def test_fast_is_smaller_on_real_sized_images(tmp_path):
    """640x360 smooth depth + textured RGB: fast mode must shrink both parts."""
    rng = np.random.default_rng(1)
    yy, xx = np.mgrid[0:360, 0:640]
    depth = (2000 + 3 * xx + 2 * yy + rng.integers(0, 4, (360, 640))).astype(np.uint16)
    rgb = np.clip(np.stack([xx % 256, yy % 256, (xx * yy) % 256], -1) + rng.integers(0, 30, (360, 640, 3)), 0, 255).astype(np.uint8)
    PIL.fromarray(rgb).save(tmp_path / "r.jpg", quality=92)
    PIL.fromarray(depth).save(tmp_path / "d.png")
    w = ImageStore.open_writer(tmp_path / "s")
    sid = w.add_sensor("cam", 640, 360, 300, 300, 320, 180)
    fid = w.add_frame_ref(sid, 1, kind=0, rgb=tmp_path / "r.jpg", depth=tmp_path / "d.png", anchor_node_id=1)
    w.close()
    store = ImageStore.open(tmp_path / "s")
    sizes = {}
    for mode in transfer.COMPRESSION_MODES:
        parts = transfer.decode_header(transfer.encode_frame(store, fid, mode))["parts"]
        sizes[mode] = {p["name"]: p["size"] for p in parts}
    assert sizes["none"]["rgb"] == (tmp_path / "r.jpg").stat().st_size
    assert sizes["fast"]["rgb"] < 0.9 * sizes["none"]["rgb"]
    assert sizes["lossless"]["depth"] < sizes["none"]["depth"]
    assert sizes["fast"]["depth"] < 0.5 * sizes["lossless"]["depth"]


def test_select_frames(run):
    store = ImageStore.open(run["dir"] / "image_store")
    f = run["frames"]
    assert [x.frame_id for x in transfer.select_frames(store, "keyframes")] == [f[0], f[1], f[3]]
    assert [x.frame_id for x in transfer.select_frames(store, "keyframes", include_kept_observations=True)] == [f[0], f[1], f[3], f[4]]
    assert [x.frame_id for x in transfer.select_frames(store, "all")] == f
    assert [x.frame_id for x in transfer.select_frames(store, "all", after=f[2])] == f[3:]
    assert [x.frame_id for x in transfer.select_frames(store, "all", limit=2)] == f[:2]


def test_write_frame_mirror_keeps_ids(run, tmp_path):
    store = ImageStore.open(run["dir"] / "image_store")
    mirror = ImageStore.open_writer(tmp_path / "mirror", robot_id=3)
    for frame in transfer.select_frames(store, "keyframes"):
        transfer.write_frame(mirror, transfer.decode_frame(transfer.encode_frame(store, frame, "fast")))
    # idempotent (resend after reconnect)
    transfer.write_frame(mirror, transfer.decode_frame(transfer.encode_frame(store, run["frames"][0], "fast")))
    mirror.close()
    m = ImageStore.open(tmp_path / "mirror")
    assert [x.frame_id for x in m.frames()] == [run["frames"][i] for i in (0, 1, 3)]
    assert m.calib(run["sensor"]).fx == 50.0
    f0 = m.frame(run["frames"][0])
    depth = np.asarray(PIL.open(f0.depth_path))
    assert depth.shape == (H, W) and np.array_equal(depth[::2, ::2], _depth(0)[::2, ::2])
    assert np.allclose(m.frame_pose(f0), store.frame_pose(store.frame(run["frames"][0])))
    assert check_store(tmp_path / "mirror")["ok"]


@pytest.mark.parametrize("mode,frames,suffix", [("fast", "keyframes", ".tar"), ("lossless", "all", ".tar.zst"),
                                                ("none", "keyframes", ".tar")])
def test_pack_unpack_roundtrip(run, tmp_path, mode, frames, suffix):
    archive = tmp_path / f"pack{suffix}"
    stats = pack_store(run["dir"], archive, frames=frames, compression=mode)
    assert stats["frames"] == (5 if frames == "all" else 4) and stats["masks"] == 1
    assert stats["archive_bytes"] == archive.stat().st_size
    if suffix.endswith(".zst"):
        assert archive.read_bytes()[:4] == b"\x28\xb5\x2f\xfd"
    dest = tmp_path / "dest"
    out = unpack_store(archive, dest)
    assert out["check"]["ok"], out["check"]
    src = ImageStore.open(run["dir"] / "image_store")
    dst = ImageStore.open(dest / "image_store")
    # every row survives
    for key in ("frames", "observations", "observations_kept", "track_bindings", "sensors"):
        assert dst.info()[0][key] == src.info()[0][key], key
    assert dst.info()[0]["meta"]["transfer_compression"] == mode
    shipped = {x.frame_id for x in dst.frames() if x.rgb_path is not None}
    expected = set(run["frames"]) if frames == "all" else {run["frames"][i] for i in (0, 1, 3, 4)}
    assert shipped == expected
    for fid in shipped:
        a, b = src.frame(fid), dst.frame(fid)
        assert np.allclose(src.frame_pose(a), dst.frame_pose(b))
        da, db = np.asarray(PIL.open(a.depth_path)), np.asarray(PIL.open(b.depth_path))
        assert da.shape == db.shape
        if mode == "fast":
            assert np.array_equal(da[::2, ::2], db[::2, ::2])
        else:
            assert np.array_equal(da, db)
            assert a.rgb_path.read_bytes() == b.rgb_path.read_bytes()
    obs = dst.observations(run["obj"], kept_only=True)
    assert len(obs) == 1 and obs[0].mask_path.exists() and dst.mask(obs[0]).sum() == 15 * 20
    # the unpacked store keeps working as a store
    w = ImageStore.open_writer(dest / "image_store", robot_id=3)
    assert w.add_frame_ref(run["sensor"], 999, kind=0, anchor_node_id=1) not in run["frames"]
    w.close()


def test_unpack_stream_and_refuses_existing(run, tmp_path):
    buf = io.BytesIO()
    pack_store(run["dir"], buf, zstd_level=1)
    buf.seek(0)
    unpack_store(buf, tmp_path / "d")
    buf.seek(0)
    with pytest.raises(FileExistsError):
        unpack_store(buf, tmp_path / "d")
    buf.seek(0)
    unpack_store(buf, tmp_path / "d", overwrite=True)
    assert check_store(tmp_path / "d")["ok"]


def test_unpack_rejects_unsafe_members(tmp_path):
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tar:
        data = json.dumps({"format": "hydra-image-store-pack", "version": 1}).encode()
        info = tarfile.TarInfo("manifest.json")
        info.size = len(data)
        tar.addfile(info, io.BytesIO(data))
        info = tarfile.TarInfo("files/../../evil")
        info.size = 1
        tar.addfile(info, io.BytesIO(b"x"))
    buf.seek(0)
    with pytest.raises(ValueError):
        unpack_store(buf, tmp_path / "d")
    assert not (tmp_path / "evil").exists()


def test_check_store_detects_missing_files(run):
    store_dir = run["dir"] / "image_store"
    ImageStore.open(store_dir).frame(run["frames"][0]).rgb_path.unlink()
    report = check_store(run["dir"])
    assert not report["ok"] and not report["checks"]["all referenced files exist"]


def test_cli_pack_unpack_check(run, tmp_path, capsys):
    from hydra_image_store.cli import main

    assert main(["pack", str(run["dir"]), str(tmp_path / "a.tar.zst"), "--quiet"]) == 0
    assert main(["unpack", str(tmp_path / "a.tar.zst"), str(tmp_path / "dest"), "--quiet"]) == 0
    assert main(["check", str(tmp_path / "dest")]) == 0
    assert "ALL PASS" in capsys.readouterr().out


def test_pillow_fallback_without_cv2(run, monkeypatch):
    monkeypatch.setattr(transfer, "_cv2", lambda: None)
    store = ImageStore.open(run["dir"] / "image_store")
    dec = transfer.decode_frame(transfer.encode_frame(store, run["frames"][0], "fast"))
    assert dec["rgb"][:3] == b"\xff\xd8\xff" and dec["depth"].shape == (19, 32)
    png = transfer.encode_png16(dec["depth"])
    assert np.array_equal(transfer.decode_png16(png), dec["depth"])


def test_missing_zstandard_is_a_clear_error(run, monkeypatch):
    import builtins

    real = builtins.__import__

    def fake(name, *a, **k):
        if name == "zstandard":
            raise ImportError("no zstandard")
        return real(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", fake)
    store = ImageStore.open(run["dir"] / "image_store")
    with pytest.raises(transfer.MissingDependency, match="zstandard"):
        transfer.encode_frame(store, run["frames"][0], "lossless")
    assert transfer.encode_frame(store, run["frames"][0], "none")  # none needs no extras
