import os

# EXR fixtures are encoded with OpenCV, which needs this before cv2 is imported.
os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import pathlib

import imageio.v3 as iio
import numpy as np
import pytest
from hydra_python.image_store import FrameKind, ImageStore, make_node_id


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def make_T(R=None, t=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = t
    return T


def write_image(path, w=8, h=6, mode="RGB", value=128):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if mode == "I;16":
        arr = np.full((h, w), value, dtype=np.uint16)
    elif mode == "L":
        arr = np.full((h, w), value, dtype=np.uint8)
    else:
        arr = np.full((h, w, 3), value, dtype=np.uint8)
    iio.imwrite(path, arr)
    return path


BODY_T_SENSOR = make_T(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.0]]), (0.4, 0.0, 0.1))
AGENT0 = make_node_id("a", 0)
AGENT1 = make_node_id("a", 1)
OBJ_A = make_node_id("O", 1)
OBJ_B = make_node_id("O", 2)
OBJ_C = make_node_id("O", 3)


@pytest.fixture
def synthetic(tmp_path):
    """A one-robot store: 3 keyframes (two anchored to a0), 1 observation frame, 3 tracks
    bound to 3 objects."""
    src = tmp_path / "src"
    store_dir = tmp_path / "store"
    w = ImageStore.open_writer(store_dir, robot_id=0)
    sid = w.add_sensor("cam", 8, 6, 5.0, 5.0, 4.0, 3.0, 0.001, BODY_T_SENSOR)
    kf0 = w.add_frame_ref(
        sid,
        100,
        kind=FrameKind.KEYFRAME,
        world_T_body_odom=make_T(t=(1, 0, 0)),
        rgb=write_image(src / "kf0.jpg"),
        depth=write_image(src / "kf0.png", mode="I;16"),
        anchor_node_id=AGENT0,
        link="copy",
    )
    kf1 = w.add_frame_ref(
        sid,
        200,
        kind=FrameKind.KEYFRAME,
        world_T_body_odom=make_T(t=(2, 0, 0)),
        rgb=write_image(src / "kf1.jpg"),
        anchor_node_id=AGENT1,
        link="copy",
    )
    mid = w.add_frame_ref(
        sid,
        150,
        kind=FrameKind.KEYFRAME,
        world_T_body_odom=make_T(t=(1.5, 0, 0)),
        rgb=write_image(src / "mid.jpg"),
        anchor_node_id=AGENT0,
        anchor_T_body=make_T(rot_z(0.1), (0.5, 0, 0)),
        link="copy",
    )
    obs_frame = w.add_frame_ref(
        sid,
        160,
        kind=FrameKind.OBSERVATION,
        world_T_body_odom=make_T(t=(1.6, 0, 0)),
        rgb=write_image(src / "obs.jpg"),
        link="copy",
    )
    mask = np.zeros((6, 8), bool)
    mask[2:4, 3:6] = True
    o = {}
    o["a1"] = w.add_observation(
        10,
        kf0,
        bbox2d=(3, 2, 5, 3),
        bbox3d=((0, 0, 0), (1, 1, 1)),
        mask=mask,
        score=0.9,
        kept=True,
    )
    o["a2"] = w.add_observation(10, obs_frame, mask=mask, score=0.5, kept=True)
    o["b1"] = w.add_observation(11, kf1, mask=~mask, score=0.7, kept=True)
    o["b2"] = w.add_observation(11, mid, score=0.1)
    o["c1"] = w.add_observation(12, kf1, score=0.2)
    w.bind_track(10, OBJ_A)
    w.bind_track(11, OBJ_B)
    w.bind_track(12, OBJ_C)
    w.close()
    return {
        "dir": store_dir,
        "src": src,
        "sensor": sid,
        "kf0": kf0,
        "kf1": kf1,
        "mid": mid,
        "obs_frame": obs_frame,
        "obs": o,
        "mask": mask,
    }
