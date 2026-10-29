# hydra_image_store

Pure-python reader, small writer and CLI for the Hydra image store: one SQLite index
(`store.sqlite`, schema in `hydra/resources/image_store/schema.sql`, `schema_version` 1)
per robot process. Image files sit next to the index and are referenced by relative path.
Dependencies: `numpy`. Optional: `Pillow` (mask PNGs, mask export) and `spark_dsg`
(pose lookups from a `DynamicSceneGraph`). The package is not a colcon package
(`COLCON_IGNORE`), so install it with `pip install -e hydra_image_store`.

## Store layout

```
<store>/store.sqlite                 # WAL mode
<store>/frames/r<robot>/<ts>_rgb.jpg  # shared by keyframes, sub-keyframes, observation frames
<store>/frames/r<robot>/<ts>_depth.png
<store>/masks/r<robot>/<obs_id>.png   # kept object observations only
```

### Sessions

Scene graph node ids and track ids restart with every Hydra process, so a store only
describes the session that wrote it. When Hydra's C++ writer first opens a non-empty
store directory, `image_store.on_existing` decides what happens:
- `append` (default) keeps writing into it and logs a warning, because ids can collide.
- `rotate` moves it aside and starts an empty store at the configured path.
- `fail` refuses to open it.

A rotated run looks like this:

```
<run>/image_store/                 # live store (current session); path unchanged
<run>/hydra/backend/dsg.json       # its scene graph (written when Hydra shuts down)
<run>/sessions/<k>/image_store/    # store of the k-th earlier session, k = 1, 2, ...
<run>/sessions/<k>/hydra/{backend,frontend}/dsg*.json, mesh.ply, ...
                                   # copy of that session's saved scene graphs
<run>/sessions/<k>/session.json    # rotated_at, source paths, counts, robot_ids, artifacts
```

`<run>/sessions/<k>` has the same layout as a run directory, so tools that take a run
directory work on it unchanged. Hydra copies the files listed in
`image_store.session_artifacts` (default: backend and frontend `dsg.json`,
`dsg_with_mesh.json` and `mesh.ply`, plus the backend `deformation_graph.dgrf`,
`trajectory.csv` and `loop_closures.csv`) from `image_store.session_artifacts_dir`
(ADT4: `$ADT4_OUTPUT_DIR/hydra`, Hydra's `log_path`). Rotation happens at most once
per process: Khronos and Hydra share the instance, so whichever opens it first
triggers it. The python writer (`open_writer`) never rotates, because SAM3 write-back
deliberately writes into the live store.

```python
for s in ImageStore.sessions("run"):      # oldest first, live session last
    s.index, s.live, s.store, s.info       # info = session.json
    with s.open() as store:
        dsg = s.load_dsg()                 # hydra/backend/dsg.json of that session
        store.frame_pose(frame, dsg)
```

Encodings (shared with the C++ writer): transforms are 16 little-endian float64,
row-major. Frame, observation and sensor ids are `(robot_id << 48) | seq`. Masks are a
JSON array of run lengths over the row-major image, starting with a background run.
`frames.kind` is 0 for a keyframe, 1 for a sub-keyframe and 2 for an observation frame.

Track id namespaces: Khronos tracks use small ids. Ids from `1 << 40` up are reserved.
SAM3 objects use `sam3_track_id(node) = (1 << 40) | (node & (2**40-1))`, which gives one
track per node. `import-legacy` uses `(1 << 41) | object_index` for Khronos folders,
because the original track id is lost.

## Reading

```python
from hydra_image_store import ImageStore, load_dsg

store = ImageStore.open("run/image_store", "robot2/image_store")  # readonly=True
store.canonical(node_id) -> int            # follows node_aliases chains (cycle-safe)
store.members(node_id) -> set[int]          # canonical + everything merged into it
store.observations(node_id, kept_only=False, limit=None, order="score"|"time") -> list[Observation]
store.track_observations(robot_id, track_id) -> list[Observation]
store.frames_for_agent(node_id, include_subkeyframes=True) -> list[Frame]
store.frames(robot_id=None, t0=None, t1=None, kind=None) -> list[Frame]
store.frame(frame_id) -> Frame | None
store.calib(sensor_id) -> Calib | None      # intrinsics, K, depth_scale, body_T_sensor
store.frame_pose(frame, pose_lookup=None, with_source=False) -> 4x4 (, "anchor"|"odom")
store.mask(obs) -> np.ndarray[bool] | None  # RLE decode, else the mask PNG
store.object_nodes(), store.agent_nodes(include_subkeyframe_anchors=False), store.info()
```

`Observation` has `obs_id, robot_id, track_id, node_id` (the node the track is bound to,
which may be a merged child), `frame_id, timestamp_ns, bbox2d, bbox3d, mask_rle,
mask_path, score, kept`, the full `frame`, and the `rgb_path` / `depth_path` properties.
`Frame` has `world_T_body_odom, anchor_node_id, anchor_T_body`, and absolute
`rgb_path` / `depth_path` (or `None`).

`frame_pose` returns `world_T_anchor @ anchor_T_body @ body_T_sensor`. If the anchor (or
its canonical node) cannot be resolved, it falls back to `world_T_body_odom @ body_T_sensor`.
`pose_lookup` can be any of:
- a spark_dsg graph (agent `position` + `world_R_body`)
- `load_dsg("dsg.json")`, a spark_dsg-free json parser
- a mapping or callable `node_id -> 4x4 | None`

Readers open the store with `mode=ro` and see each commit as it lands. The alias cache is
invalidated through `PRAGMA data_version`.

## Writing (SAM3 discovery, tools)

```python
w = ImageStore.open_writer("run/image_store", robot_id=0)
sid = w.add_sensor(name, width, height, fx, fy, cx, cy, depth_scale=0.001, body_T_sensor=None)
fid = w.add_frame_ref(sid, ts, kind=FrameKind.OBSERVATION, world_T_body_odom=None,
                      rgb=path, depth=path, anchor_node_id=None, anchor_T_body=None,
                      link="hardlink"|"symlink"|"copy")
w.set_anchor(fid, node_id, anchor_T_body=None)
oid = w.add_observation(track_id, fid, timestamp_ns=None, bbox2d=None, bbox3d=None,
                        mask=None, mask_rle=None, mask_path=None, score=0.0, kept=False)
w.add_sam3_observation(node_id, fid, **same_kwargs)   # binds sam3_track_id(node)
w.bind_track(track_id, node_id)
w.record_merges({child: parent})
with w.batch(): ...                                     # one transaction
```

`add_frame_ref` deduplicates on (sensor, timestamp). For an existing frame it only fills
in missing image paths and a missing anchor. Image files already inside the store are
referenced in place. Other files are hardlinked by default, with a copy when the
hardlink fails. Ids are allocated inside the insert transaction as
`max(seq of this robot) + 1`. A reopened store resumes numbering, and the C++ writer can
work on the same file at the same time.

## CLI

```
hydra-image-store info <store>... [--json]
hydra-image-store sessions <run> [--json]          # rotated sessions + live store, with DSGs
hydra-image-store export <store>... --out DIR [--dsg dsg.json] [--copy] [--kept-only]
hydra-image-store gc <store> [--max-images-per-object N] [--orphans] [--dry-run]
hydra-image-store import-legacy --run-dir RUN --out STORE [--dsg dsg.json] [--robot-id 0]
                                [--sam3-dir sam3_objects ...] [--copy | --symlink]
```

**export** writes one folder per object, `O_<idx>/frame_<ts>_{rgb.jpg,depth.png,mask.png,meta.json}`.
Merged children go under their surviving parent. It writes one `agent_<ts>/` folder per
agent node, holding the keyframe and its sub-keyframes, or `agent_a_<idx>/` when only
sub-keyframes anchor to that node. `index.json` maps folders to node ids. Files are
hardlinked, with a symlink when the hardlink fails (for example across filesystems), or
copied with `--copy`. Each `meta.json` holds:
- `world_T_sensor` and `world_T_body`, corrected through `--dsg` (`pose_source` says
  whether the anchor or odometry was used)
- the bboxes, timestamp, ids, score and kept flag
- the calibration

**gc** applies keep-N per merged node set when `--max-images-per-object` is given. It
deletes the masks of unkept observations and the rgb/depth images of observation frames
(kind 2) that no kept observation references. With `--orphans` it also deletes
unreferenced files. Rows stay; their paths are set to NULL. Keyframe and sub-keyframe
images are never deleted.

**import-legacy** converts a pre-store run (`agents/`, `subkeyframes/`, `images/O_*`,
`sam3_objects/O*`) without touching it. Images are hardlinked by default. Bindings work
as follows:
- Agents bind to DSG agent nodes by `image_folder`, or else by timestamp.
- Sub-keyframes take the anchor of their DSG SubKeyframe node, or else the nearest
  keyframe.
- Object observation frames had no pose. They take the anchor and odometry of the
  nearest imported frame in time. The worst gap is stored in `meta.legacy_obs_max_dt_ns`.
- Object folders bind by DSG `image_folder`, or else by the NodeSymbol their name encodes.

## Transfer between machines

`hydra_image_store.transfer` encodes one frame (row, calibration, RGB, depth) as a
self-describing blob (`b"HISF"`, version byte, JSON header, payloads; no msgpack).
Extras: `pip install hydra_image_store[transfer]` (`zstandard`, `opencv-python-headless`;
Pillow works instead of cv2). A missing package raises `transfer.MissingDependency`.

| compression | RGB | depth |
|---|---|---|
| `none` | stored bytes | stored PNG bytes |
| `lossless` | stored bytes | uint16 row-delta + zstd-3 (bit-exact) |
| `fast` (default) | JPEG q75 (~40% smaller) | half resolution + row-delta + zstd-3 |

`fast` halves depth by nearest sampling: half-res pixel `(i, j)` is the full-res sample
`(2i, 2j)`, so only measured values survive and the scaled intrinsics `fx/2, fy/2, cx/2,
cy/2` (size `ceil(w/2) x ceil(h/2)`) are exact. The blob carries `depth_downscale` and
`meta["depth_calib"]`. Writing into a store repeats each sample over its 2x2 block, so
the depth file is pixel-aligned with RGB and the sensor calibration is unchanged.

```python
from hydra_image_store import transfer
frames = transfer.select_frames(store, frames="keyframes"|"all", after=None,
                                include_kept_observations=False, require_images=True)
blob = transfer.encode_frame(store, frame_or_id, compression="fast") -> bytes
d = transfer.decode_frame(blob) -> {"meta", "rgb", "rgb_format", "depth", "depth_png",
                                    "depth_downscale"}
transfer.decode_header(blob)                    # meta + parts, no pixel decoding
transfer.full_res_depth(d)                      # uint16 aligned with RGB / calib
transfer.write_frame(writer, d) -> frame_id     # mirror: keeps source sensor/frame ids
transfer.write_decoded_images(store_dir, d)     # just the files -> (rgb_rel, depth_rel)
```

`meta` holds `frame_id, robot_id, sensor_id, timestamp_ns, kind, world_T_body_odom`
(16 floats, row-major), `anchor_node_id, anchor_T_body, rgb_path, depth_path` (source
relative paths), `calib` (`Calib.to_json()` + `robot_id`), `depth_calib`,
`compression`, `depth_downscale` and, in `fast`, `rgb_quality`. `select_frames` orders by
`frame_id` (the sqlite rowid), so the max `frame_id` received is a resume watermark
(ids are per robot). `write_frame` is idempotent: resending a frame only fills missing
paths.

### Prior-map pack / unpack

```
hydra-image-store pack RUN_DIR OUT.tar[.zst] [--frames keyframes|all]
                       [--compression none|lossless|fast] [--zstd-level N] [--no-masks]
hydra-image-store unpack ARCHIVE DEST_RUN_DIR [--overwrite] [--no-check] [--dsg dsg.json]
hydra-image-store check STORE_OR_RUN [--dsg dsg.json] [--json]
```

`OUT` / `ARCHIVE` may be `-` (stdout / stdin), so a copy over ssh streams:
`ssh robot hydra-image-store pack RUN - | hydra-image-store unpack - DEST`. The pack
report goes to stderr. The archive is a tar: `manifest.json`, `frames/<id>.hisf`,
`files/<mask paths>`, then a snapshot of `store.sqlite` with every row kept. Image paths
of frames that were not shipped are NULL (as after `gc`). `--frames keyframes` ships the
keyframes plus the frames of kept object observations. `unpack` builds
`DEST/image_store.partial`, runs `check_store` (with `DEST/hydra/backend/dsg.json` when
present) and only then renames it to `DEST/image_store`.

Python API (for the console Maps -> Transfer job):

```python
from hydra_image_store import pack_store, unpack_store, check_store
stats = pack_store(run_dir, out_path_or_binary_file, frames="keyframes",
                   compression="fast", zstd_level=None, include_masks=True,
                   progress=lambda done, total: ...)
stats = unpack_store(path_or_binary_stream, dest_run_dir, overwrite=False, check=True,
                     dsg=None, progress=None)   # stats["check"] = check_store report
report = check_store(store_or_run_dir, dsg=None)  # {"ok", "checks": {name: bool}, "stats"}
```

Measured on a 640x360 run (`imgstore_lc_on2`, 369 keyframe + kept-observation frames):
`none` 61 MB, `lossless` 59 MB, `fast` 30 MB (RGB 39 KB/frame, depth 25 KB/frame),
pack 2.6 s, unpack 1.8 s. `--frames all` (1305 frames) with `fast` and outer zstd: 84 MB.

## Tests

```
cd hydra_image_store && python -m pytest       # do not source ROS (PYTHONPATH)
```
