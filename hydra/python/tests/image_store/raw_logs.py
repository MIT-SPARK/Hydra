"""Writers for synthetic Hydra/Khronos image logs (FORMAT.md), byte-compatible with the
C++ writers: InputData archives as ``input_data_io.cpp`` encodes them (OpenCV PNG of the
RGB Mat, float32 TIFF, metadata.yaml), track logs, track_bindings.json, merges.json and a
scene graph json in spark_dsg's format."""

from __future__ import annotations

import functools
import json
import pathlib
import zipfile

import numpy as np
import yaml
from hydra_python.image_store import make_node_id

cv2 = None


def _cv2():
    global cv2
    if cv2 is None:
        import cv2 as _cv2mod  # OPENCV_IO_ENABLE_OPENEXR (legacy_exr) is set in conftest

        cv2 = _cv2mod
    return cv2


W, H = 16, 12
FX = FY = 10.0
CX, CY = 8.0, 6.0
BODY_P_SENSOR = [0.25, -0.5, 0.75]
BODY_Q_SENSOR = {"w": 0.9800665778412416, "x": 0.0, "y": 0.19866933079506122, "z": 0.0}


def quat_to_R(w, x, y, z):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def body_T_sensor():
    T = np.eye(4)
    T[:3, :3] = quat_to_R(**BODY_Q_SENSOR)
    T[:3, 3] = BODY_P_SENSOR
    return T


def pose(x, y=0.0, z=0.0, yaw=0.0):
    T = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = (x, y, z)
    return T


def _quat_wxyz(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(
        np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2]
    )
    y = np.copysign(
        np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0]
    )
    z = np.copysign(
        np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1]
    )
    return [float(w), float(x), float(y), float(z)]


def color_rgb(value=(200, 30, 10)):
    """An RGB Mat as Hydra holds it (InputData::color_image is RGB)."""
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = value
    return img


def depth_m(value=1.234):
    d = np.full((H, W), value, np.float32)
    d[0, 0] = np.nan
    d[0, 1] = np.inf
    d[0, 2] = 70.0  # saturates in mm
    return d


def write_input_zip(
    path,
    timestamp_ns,
    world_T_body,
    *,
    color=None,
    depth=None,
    sensor="camera",
    legacy_exr=False,
    feature=None,
):
    """``InputData::save`` of a camera input (entry names/encodings of input_data_io.cpp).
    ``legacy_exr`` writes depth as the uncompressed EXR older Hydra versions used."""
    cv = _cv2()
    entries = {}
    images = {}

    def add(name, ext, img, params=()):
        ok, buf = cv.imencode(ext, img, list(params))
        assert ok
        entries[name + ext] = buf.tobytes()
        images[name] = {
            "file": name + ext,
            "rows": int(img.shape[0]),
            "cols": int(img.shape[1]),
            "opencv_type": {np.dtype(np.uint8): 16, np.dtype(np.float32): 5}[img.dtype]
            if img.ndim == 3 or img.dtype == np.float32
            else 0,
        }

    if color is not None:
        add(
            "color",
            ".png",
            color,
            [
                cv.IMWRITE_PNG_COMPRESSION,
                1,
                cv.IMWRITE_PNG_STRATEGY,
                cv.IMWRITE_PNG_STRATEGY_RLE,
            ],
        )
    if depth is not None:
        if legacy_exr:
            exr = [cv.IMWRITE_EXR_TYPE, cv.IMWRITE_EXR_TYPE_FLOAT]
            add("depth", ".exr", depth, exr + [cv.IMWRITE_EXR_COMPRESSION, 0])
        else:
            add("depth", ".tiff", depth)
    meta = {
        "version": 1,
        "timestamp_ns": int(timestamp_ns),
        "world_T_body": pose_record(world_T_body),
        "sensor": sensor_record(sensor),
        "points_in_world_frame": False,
        "min_range_m": 0.0,
        "max_range_m": float("inf"),
        "feature": [] if feature is None else [float(v) for v in feature],
        "label_features": [],
        "images": images,
    }
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w") as zf:
        for name, data in entries.items():
            # zip_archive.cpp deflates every entry except PNG
            stored = name.endswith((".png", ".exr"))
            method = zipfile.ZIP_STORED if stored else zipfile.ZIP_DEFLATED
            zf.writestr(name, data, compress_type=method)
        zf.writestr(
            "metadata.yaml", yaml.safe_dump(meta), compress_type=zipfile.ZIP_DEFLATED
        )
    return path


def pose_record(T):
    """``{translation_m, rotation_wxyz}`` (keyframe metadata and track.json poses);
    NaN entries become null as in the C++ writers."""
    T = np.asarray(T, float)

    def num(v):
        return float(v) if np.isfinite(v) else None

    return {
        "translation_m": [num(v) for v in T[:3, 3]],
        "rotation_wxyz": [num(v) for v in _quat_wxyz(T[:3, :3])],
    }


def sensor_record(name="camera", **config):
    """``hydra::input::writeSensor`` of the test camera (``config`` overrides fields; a
    ``None`` value removes one): the ``sensor`` entry of a keyframe archive's
    metadata.yaml and one entry of a track log's sensors.yaml."""
    base = {
        "type": "camera",
        "min_range": 0.1,
        "max_range": float("inf"),
        "static_mask_fp": "",
        "extrinsics": {
            "type": "param",
            "body_R_sensor": dict(BODY_Q_SENSOR),
            "body_p_sensor": list(BODY_P_SENSOR),
        },
        "invalid_labels": [],
        "width": W,
        "height": H,
        "cx": CX,
        "cy": CY,
        "fx": FX,
        "fy": FY,
    }
    base.update(config)
    return {"name": name, "config": {k: v for k, v in base.items() if v is not None}}


def write_sensors(tracks_dir, *records):
    """Add sensor records to ``<tracks>/sensors.yaml`` (the track logger writes each
    sensor once, before the first track that references it)."""
    path = pathlib.Path(tracks_dir) / "sensors.yaml"
    known = yaml.safe_load(path.read_text())["sensors"] if path.exists() else []
    names = {r["name"] for r in known}
    known += [r for r in records if r["name"] not in names]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump({"version": 1, "sensors": known}))


def write_track(
    tracks_dir,
    track_id,
    observations,
    *,
    absorbed=(),
    extracted=True,
    label=3,
    robot_id=0,
):
    """A Khronos track log as khronos' TrackLogger writes it; ``observations`` are dicts
    with ``ts``, ``T`` and ``kept`` (optional ``score``, ``bbox`` (None = empty mask),
    ``mask`` bool HxW, ``mask_rle``, ``sensor`` record from :func:`sensor_record`,
    ``image_size`` [w, h] or None). Sensors go to ``<tracks>/sensors.yaml``."""
    from hydra_python.image_store import rle_encode

    cv = _cv2()
    d = pathlib.Path(tracks_dir) / str(track_id)
    d.mkdir(parents=True, exist_ok=True)
    obs_json = []
    for k, o in enumerate(observations):
        sensor = o.get("sensor") or sensor_record()
        write_sensors(tracks_dir, sensor)
        entry = {
            "k": k,
            "timestamp_ns": int(o["ts"]),
            "world_T_body": pose_record(o["T"]),
            "sensor": sensor["name"],
            "image_size": o.get("image_size", [W, H]),
            "bbox_2d": None
            if o.get("bbox", 0) is None
            else list(o.get("bbox", [2, 3, 9, 8])),
            "score": float(o.get("score", 0.5)),
            "kept": bool(o["kept"]),
            "color": None,
            "depth": None,
            "mask": None,
            "mask_rle": None,  # null for kept (png) and empty masks
        }
        mask = o.get("mask")
        if mask is None:
            mask = np.zeros((H, W), bool)
            mask[3:8, 2:9] = True
        if o["kept"]:
            cv.imwrite(
                str(d / f"obs_{k}_color.jpg"),
                np.full((H, W, 3), (10, 30, 200), np.uint8),
            )
            cv.imwrite(str(d / f"obs_{k}_depth.png"), np.full((H, W), 1500, np.uint16))
            cv.imwrite(str(d / f"obs_{k}_mask.png"), mask.astype(np.uint8) * 255)
            entry.update(
                color=f"obs_{k}_color.jpg",
                depth=f"obs_{k}_depth.png",
                mask=f"obs_{k}_mask.png",
            )
        elif mask.any():
            entry["mask_rle"] = o.get(
                "mask_rle", json.dumps(rle_encode(mask), separators=(",", ":"))
            )
        obs_json.append(entry)
    data = {
        "version": 1,
        "robot_id": int(robot_id),
        "track_id": int(track_id),
        "absorbed_track_ids": list(absorbed),
        "extracted": bool(extracted),
        "semantic_label": label,
        "observations": obs_json,
    }
    (d / "track.json").write_text(json.dumps(data))
    return d


def write_bindings(frontend_dir, bindings, layer="OBJECTS", extra=None):
    """track_bindings.json; ``layer`` is the graph_updater key ("OBJECTS" by default
    in Hydra), ``extra`` more layers ``{key: [entry dicts]}``."""
    d = pathlib.Path(frontend_dir)
    d.mkdir(parents=True, exist_ok=True)
    data = {
        "version": 1,
        "layers": {
            **(extra or {}),
            layer: [
                {
                    "track_id": t,
                    "node_id": n,
                    "first_ns": f,
                    "last_ns": last,
                    "active": a,
                    "end": e,
                }
                for t, n, f, last, a, e in bindings
            ],
        },
    }
    (d / "track_bindings.json").write_text(json.dumps(data))


def write_merges(backend_dir, merges):
    d = pathlib.Path(backend_dir)
    d.mkdir(parents=True, exist_ok=True)
    data = {
        "version": 1,
        "merges": [{"child": c, "parent": p} for c, p in merges.items()],
    }
    (d / "merges.json").write_text(json.dumps(data))


def write_dsg(path, agents, objects=(), prefix="a"):
    """Scene graph json with agent nodes ``[(index, ts_ns, 4x4)]``: spark_dsg's own
    writer when it is importable, else the same json by hand."""
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import datetime

        import spark_dsg as sd
    except ImportError:
        sd = None
    if sd is not None:
        g = sd.DynamicSceneGraph()
        for idx, ts, T in agents:
            a = sd.AgentNodeAttributes()
            a.position = np.asarray(T, float)[:3, 3]
            w, x, y, z = _quat_wxyz(np.asarray(T, float)[:3, :3])
            a.world_R_body = sd.Quaternion(w, x, y, z)
            a.timestamp = datetime.timedelta(microseconds=ts // 1000)
            g.add_node(2, sd.NodeSymbol(prefix, idx), a, ord(prefix))
        for nid in objects:
            o = sd.ObjectNodeAttributes()
            o.position = np.zeros(3)
            g.add_node(sd.DsgLayers.OBJECTS, sd.NodeSymbol(nid), o)
        g.save(str(path))
        # the python bindings only take microseconds: restore the exact ns stamps
        data = json.loads(path.read_text())
        exact = {make_node_id(prefix, idx): int(ts) for idx, ts, _ in agents}
        for node in data["nodes"]:
            if node["id"] in exact:
                node["attributes"]["timestamp"] = exact[node["id"]]
        path.write_text(json.dumps(data))
        return path
    nodes = []
    for idx, ts, T in agents:
        T = np.asarray(T, float)
        w, x, y, z = _quat_wxyz(T[:3, :3])
        nodes.append(
            {
                "id": make_node_id(prefix, idx),
                "layer": 2,
                "partition": ord(prefix),
                "attributes": {
                    "type": "AgentNodeAttributes",
                    "timestamp": int(ts),
                    "position": T[:3, 3].tolist(),
                    "world_R_body": {"w": w, "x": x, "y": y, "z": z},
                },
            }
        )
    for nid in objects:
        nodes.append(
            {
                "id": nid,
                "layer": 2,
                "partition": 0,
                "attributes": {"type": "ObjectNodeAttributes", "position": [0, 0, 0]},
            }
        )
    path.write_text(json.dumps({"nodes": nodes, "edges": []}))
    return path


S = 1_000_000_000
O1, O2, O3, O4, O5, O6, O9 = (make_node_id("O", i) for i in (1, 2, 3, 4, 5, 6, 9))


def odom(t_s):
    """Odometric body pose at time t (moving +x at 1 m/s from x=0 at t=1 s)."""
    return pose(t_s - 1.0, 0.0, 0.0, yaw=0.1 * t_s)


def build_run(run, *, robot_prefix="a", with_frontend=True):
    """The reference run used by the indexer tests (see test_indexer.py docstring)."""
    run = pathlib.Path(run)
    kf = run / "keyframes"
    for t in (1.0, 1.5, 2.002, 3.4, 5.0):
        ts = int(round(t * S))
        write_input_zip(
            kf / f"{ts}.zip", ts, odom(t), color=color_rgb(), depth=depth_m()
        )
    # a keyframe whose odometry jumped 10 m (no agent node within 2 m)
    ts = int(3.9 * S)
    write_input_zip(
        kf / f"{ts}.zip", ts, pose(12.0), color=color_rgb(), depth=depth_m()
    )
    # Hydra's temporary archive name (must be ignored)
    (kf / "123.zip.tmp.ABCDEF").write_bytes(b"partial")

    wt = functools.partial(write_track, robot_id=ord(robot_prefix) - ord("a"))
    tracks = run / "tracks"
    wt(
        tracks,
        12,
        [
            {"ts": int(1.5 * S), "T": odom(1.5), "kept": True, "score": 0.9},
            {"ts": int(1.7 * S), "T": odom(1.7), "kept": False, "score": 0.3},
        ],
        absorbed=[7, 9],
    )
    wt(tracks, 7, [{"ts": int(1.2 * S), "T": odom(1.2), "kept": True, "score": 0.4}])
    wt(
        tracks,
        20,
        [{"ts": int(2.5 * S), "T": odom(2.5), "kept": True}],
        extracted=False,
    )
    wt(tracks, 21, [{"ts": int(2.6 * S), "T": odom(2.6), "kept": False}])
    wt(tracks, 30, [{"ts": int(3.0 * S), "T": odom(3.0), "kept": True, "score": 0.8}])
    wt(tracks, 31, [{"ts": int(3.1 * S), "T": odom(3.1), "kept": True, "score": 0.7}])

    hydra = run / "hydra"
    write_bindings(
        hydra / "frontend",
        [
            (12, O1, 1, 2, True, None),
            (7, O5, 1, 1, False, "deleted"),
            (30, O3, 1, 2, False, "track_removed"),
            (30, O2, 2, 3, True, None),
            (31, O4, 1, 3, False, "track_removed"),
        ],
    )
    write_merges(hydra / "backend", {O2: O6, O4: O6})
    agents_t = (1.0, 2.0, 3.0, 4.0)
    # backend poses are optimized: shifted by 1 m in y relative to odometry
    write_dsg(
        hydra / "backend" / "dsg.json",
        [
            (i, int(round(t * S)), pose(0, 1.0) @ odom(t))
            for i, t in enumerate(agents_t)
        ],
        objects=(O1, O6),
        prefix=robot_prefix,
    )
    if with_frontend:
        write_dsg(
            hydra / "frontend" / "dsg.json",
            [(i, int(round(t * S)), odom(t)) for i, t in enumerate(agents_t)],
            prefix=robot_prefix,
        )
    return run
