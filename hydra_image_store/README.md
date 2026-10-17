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

## Tests

```
cd hydra_image_store && python -m pytest       # do not source ROS (PYTHONPATH)
```
