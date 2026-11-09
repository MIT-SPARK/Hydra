"""Reader for Hydra ``InputData::save`` archives (keyframe logs, FORMAT.md §1).

An archive is a ZIP with ``metadata.yaml`` plus one entry per non-empty image, written
by ``hydra/src/input/input_data_io.cpp``:

- ``color.png``  ``CV_8UC3`` holding **RGB** (Hydra's ``InputData::color_image``
  convention; OpenCV encodes the Mat as if it were BGR, so the PNG file itself has its
  red and blue channels swapped)
- ``depth.tiff``  ``CV_32FC1`` depth in metres (uncompressed TIFF; the archive deflates it)
- ``color_mask.png``, ``range.tiff``, ``labels.tiff``, ``instances.tiff``,
  ``vertices.exr``, ``traversability.tiff``, ``sensor_mask.png`` (ignored here)

Archives written before Hydra switched its float images to TIFF hold ``depth.exr``
instead; :func:`decode_exr` reads those through imageio's ``opencv`` plugin.

``metadata.yaml``::

    version: 1
    timestamp_ns: <int>
    world_T_body: {translation_m: [x, y, z], rotation_wxyz: [w, x, y, z]}
    sensor: {name: <str>, config: {type: camera, width, height, fx, fy, cx, cy,
             min_range, max_range, extrinsics: {type: param,
             body_R_sensor: {w, x, y, z}, body_p_sensor: [x, y, z]}, ...}}
    images: {color: {file: color.png, rows, cols, opencv_type}, depth: {...}, ...}
    feature: [f0, f1, ...]          # [] unless saved (KeyframeSelector save_fields: features)
    label_features: [{label: <int>, feature: [...]}, ...]
"""

from __future__ import annotations

import dataclasses
import pathlib
import zipfile

import imageio.v3 as iio
import numpy as np

from .codec import pose_matrix, quat_to_rot

CV_8UC1, CV_8UC3, CV_8UC4, CV_32FC1 = 0, 16, 24, 5


@dataclasses.dataclass(frozen=True)
class SensorInfo:
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    body_T_sensor: np.ndarray
    type: str = "camera"


@dataclasses.dataclass
class InputRecord:
    timestamp_ns: int
    world_T_body: np.ndarray
    sensor: SensorInfo
    color: np.ndarray | None  # HxWx3 uint8 RGB (or HxW gray)
    depth: np.ndarray | None  # HxW float32 metres
    feature: np.ndarray | None = None  # float32 descriptor, None when not saved
    label_features: dict[int, np.ndarray] = dataclasses.field(default_factory=dict)


def _yaml():
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - dependency of the package
        raise ImportError("reading keyframe archives needs PyYAML") from exc
    return yaml


def _pose(node) -> np.ndarray:
    p = node["translation_m"]
    w, x, y, z = node["rotation_wxyz"]
    return pose_matrix(p, quat_to_rot(w, x, y, z))


def _vec3(value) -> list[float]:
    if isinstance(value, dict):  # tolerate {x, y, z}
        return [float(value[k]) for k in "xyz"]
    return np.asarray(value, dtype=float).reshape(3).tolist()  # flat or 3x1 nested


def sensor_from_record(record: dict) -> SensorInfo:
    """``SensorInfo`` from the ``sensor`` entry of ``metadata.yaml``."""
    cfg = record.get("config") or {}
    ext = cfg.get("extrinsics") or {}
    body_T_sensor = np.eye(4)
    if ext.get("type", "param") == "param" and "body_p_sensor" in ext:
        q = ext.get("body_R_sensor") or {"w": 1, "x": 0, "y": 0, "z": 0}
        body_T_sensor = pose_matrix(
            _vec3(ext["body_p_sensor"]), quat_to_rot(q["w"], q["x"], q["y"], q["z"])
        )
    return SensorInfo(
        name=str(record.get("name", "")),
        width=int(cfg.get("width", 0) or 0),
        height=int(cfg.get("height", 0) or 0),
        fx=float(cfg.get("fx", 0) or 0),
        fy=float(cfg.get("fy", 0) or 0),
        cx=float(cfg.get("cx", 0) or 0),
        cy=float(cfg.get("cy", 0) or 0),
        body_T_sensor=body_T_sensor,
        type=str(cfg.get("type", "")),
    )


def decode_png(data: bytes) -> np.ndarray:
    """Decode an archive PNG into the array OpenCV encoded (the original Mat).

    OpenCV writes a 3/4-channel Mat as BGR(A), so the file's first channel is the Mat's
    third: swap it back (Hydra's colour Mat is RGB, see the module docstring)."""
    return _swap_rb(iio.imread(data, extension=".png"))


def decode_tiff(data: bytes) -> np.ndarray:
    """Decode an archive TIFF (float32 depth, int32 labels, ...) as OpenCV wrote it."""
    return np.ascontiguousarray(iio.imread(data, extension=".tiff"))


def decode_exr(data: bytes) -> np.ndarray:
    """Decode an EXR from an archive written before Hydra used TIFF for float images.

    No imageio backend reads EXR out of the box: this uses the ``opencv`` plugin, which
    needs OpenCV and ``OPENCV_IO_ENABLE_OPENEXR=1`` set before ``cv2`` is imported."""
    try:
        # flags=-1 is cv2.IMREAD_UNCHANGED (float32, all channels)
        img = iio.imread(data, extension=".exr", plugin="opencv", flags=-1)
    except Exception as exc:  # noqa: BLE001 - missing cv2 / codec disabled
        raise ValueError(
            f"could not read EXR ({exc}); this archive predates Hydra's TIFF depth: "
            "install opencv and set OPENCV_IO_ENABLE_OPENEXR=1 before cv2 is imported"
        ) from exc
    # the plugin converts to RGB(A); return OpenCV's channel order
    return _swap_rb(img)


def _swap_rb(img: np.ndarray) -> np.ndarray:
    """BGR(A) <-> RGB(A) for 3/4-channel images; others unchanged."""
    if img.ndim == 3 and img.shape[2] in (3, 4):
        img = img[..., [2, 1, 0, *range(3, img.shape[2])]]
    return np.ascontiguousarray(img)


def _feature(value) -> np.ndarray | None:
    if not value:
        return None
    arr = np.asarray(value, dtype=np.float32).reshape(-1)
    return arr if arr.size else None


def _read_image(zf: zipfile.ZipFile, images: dict, name: str) -> np.ndarray | None:
    desc = images.get(name)
    if not desc:
        return None
    data = zf.read(desc["file"])
    suffix = pathlib.PurePosixPath(desc["file"]).suffix.lower()
    if suffix == ".png":
        img = decode_png(data)
    elif suffix in (".tiff", ".tif"):
        img = decode_tiff(data)
    elif suffix == ".exr":
        img = decode_exr(data)
    else:
        raise ValueError(f"{name}: unsupported image file {desc['file']!r}")
    rows, cols = int(desc["rows"]), int(desc["cols"])
    if img.shape[0] != rows or img.shape[1] != cols:
        raise ValueError(
            f"{name}: decoded {img.shape[:2]}, metadata says {(rows, cols)}"
        )
    return img


def read_input_zip(
    path: str | pathlib.Path, *, load_images: bool = True
) -> InputRecord:
    """Read a keyframe archive. Raises on missing/corrupt metadata or images."""
    with zipfile.ZipFile(path) as zf:
        meta = _yaml().safe_load(zf.read("metadata.yaml"))
        if not isinstance(meta, dict) or int(meta.get("version", 0)) != 1:
            raise ValueError(f"{path}: unsupported InputData metadata version")
        images = meta.get("images") or {}
        color = depth = None
        if load_images:
            color = _read_image(zf, images, "color")
            depth = _read_image(zf, images, "depth")
    if depth is not None:
        if depth.ndim == 3:
            depth = depth[..., 0]
        depth = depth.astype(np.float32, copy=False)
    return InputRecord(
        timestamp_ns=int(meta["timestamp_ns"]),
        world_T_body=_pose(meta["world_T_body"]),
        sensor=sensor_from_record(meta.get("sensor") or {}),
        color=color,
        depth=depth,
        feature=_feature(meta.get("feature")),
        label_features={
            int(e["label"]): f
            for e in meta.get("label_features") or []
            if isinstance(e, dict) and (f := _feature(e.get("feature"))) is not None
        },
    )


def depth_to_mm(depth_m: np.ndarray) -> np.ndarray:
    """Metres -> uint16 millimetres (as OpenCV's ``convertTo(CV_16UC1, 1000)``):
    rounded, saturated, non-finite and negative -> 0."""
    d = np.asarray(depth_m, dtype=np.float64) * 1000.0
    d = np.where(np.isfinite(d), d, 0.0)
    return np.clip(np.rint(d), 0, 65535).astype(np.uint16)


def write_jpeg(path: pathlib.Path, rgb: np.ndarray, quality: int = 90) -> None:
    """Write an RGB (or gray) array as a normal JPEG (true colours in the file)."""
    arr = rgb[..., :3] if rgb.ndim == 3 else rgb
    _atomic_write(
        path,
        iio.imwrite(
            "<bytes>", np.ascontiguousarray(arr), extension=".jpg", quality=int(quality)
        ),
    )


def write_png16(path: pathlib.Path, depth_mm: np.ndarray) -> None:
    """Write a uint16 image as a 16-bit PNG. zlib level 1 (OpenCV's default): the
    library default (6) is ~4x slower for ~15% smaller depth images."""
    _atomic_write(
        path,
        iio.imwrite(
            "<bytes>",
            np.ascontiguousarray(depth_mm, dtype=np.uint16),
            extension=".png",
            compress_level=1,
        ),
    )


def _atomic_write(path: pathlib.Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    tmp.replace(path)


__all__ = [
    "InputRecord",
    "SensorInfo",
    "decode_exr",
    "decode_png",
    "decode_tiff",
    "depth_to_mm",
    "read_input_zip",
    "sensor_from_record",
    "write_jpeg",
    "write_png16",
]
