# hydra_python.image_store

Pure-python indexer and reader for the Hydra image store: one SQLite index
(`store.sqlite`, schema in `schema.sql`, `schema_version` 1) per robot process, built
from the image logs Hydra and Khronos write (`FORMAT.md`). Image files sit next to the
index and are referenced by relative path.

It needs neither the compiled bindings nor OpenCV: dependencies are `numpy`, `PyYAML`,
`imageio` and `click` (all hydra_python dependencies); `spark_dsg` is optional (pose
lookups from a `DynamicSceneGraph`). Without the bindings `import hydra_python` warns
(`BindingsNotBuiltWarning`) and this subpackage works as usual, so either

- source a colcon workspace that built Hydra with `HYDRA_ENABLE_PYTHON=ON`, or
- `pip install hydra/` (builds the bindings when CMake finds Hydra, else installs the
  pure-python parts only), or
- put `hydra/python/src` on `PYTHONPATH`.

## Store layout

```
<store>/store.sqlite                 # WAL mode
<store>/frames/r<robot>/<stem>_rgb.jpg  # keyframes and observation frames
<store>/frames/r<robot>/<stem>_depth.png
<store>/masks/r<robot>/<obs_id>.png   # kept object observations only
```

`<stem>` is the timestamp `<ts>` for the robot's first sensor and `<ts>_s<seq>` for the
others (`seq` = the sensor id's sequence part), so two cameras with the same stamp never
share a file. The indexer and `ImageStoreWriter.add_frame_ref` name files the same way.

Encodings: transforms are 16 little-endian float64,
row-major. Frame, observation and sensor ids are `(robot_id << 48) | seq`. Masks are a
JSON array of run lengths over the row-major image, starting with a background run.
`frames.kind` is 0 for a keyframe (a Hydra keyframe archive) and 2 for an observation
frame (a frame only an object observation brought in); readers reject a frame of any
other kind.

Track ids: Khronos track ids are small. Ids from `EXTERNAL_TRACK_BASE = 1 << 40` up are
reserved for other writers (e.g. a tool that records its own observations of scene graph
nodes); the indexer never rebinds or deletes them.

## Indexer (Hydra/Khronos image logs -> store)

Hydra and Khronos log raw files (contract: `FORMAT.md`): keyframe `InputData` archives
`<keyframes>/<ts>.zip` (`<ts>_<sensor>.zip` for every sensor but the first one Hydra
saved), Khronos track logs `<tracks>/<track_id>/track.json` (+ kept observation images),
the frontend's `track_bindings.json` and the backend's `merges.json`.
`hydra_python.image_store.index` builds the store from them:

```
hydra image-store index RUN [--keyframes DIR] [--tracks DIR] [--hydra DIR] [--dsg FILE]
                        [--store DIR] [--robot-id N] [--max-anchor-dist 2.0]
                        [--copy] [--partial]
```

Defaults: `RUN/keyframes`, `RUN/tracks`, `RUN/hydra/{backend,frontend}` (the backend's
`dsg.json` / `merges.json` may also sit in `RUN/hydra/` itself, where Khronos' backend
saves them), store `RUN/image_store`. Only `*.zip` archives are read (Hydra writes
`<ts>.zip.tmp.XXXXXX` first); archives are told apart by the sensor name and stamp in
their metadata, not by the file name. `ImageStore.open(RUN)` (or
`ImageStore.open(RUN/image_store)`) builds the store when the run was never indexed and
re-indexes it when the raw inputs changed since (new archives or track logs, rewritten
`track.json`, newly saved scene graphs, bindings or merges; a fingerprint is kept in meta
`indexer.input_signature`), so a store first opened while Hydra was running catches up and
its pending frames get anchored once the backend graph is saved. The check and the indexing
run under the store's `.index.lock`. This writes the store even for a read-only open;
`index=False` never writes. Running `index` again is cheap and additive: archives already indexed are
skipped, a `track.json` whose mtime/size changed is re-read (its observations replaced),
bindings and aliases are recomputed, and still-pending frames are anchored. Rows of other
writers (tracks from `EXTERNAL_TRACK_BASE` up, their bindings and frames) are left alone.

What it produces:
- **Keyframe archives** -> KEYFRAME frames (kind 0) with the colour as `<stem>_rgb.jpg`
  and depth as uint16 mm `<stem>_depth.png` (`depth_scale` 0.001; NaN/inf -> 0). Hydra's
  colour Mat is RGB and OpenCV writes it to the archive's PNG unconverted; the indexer
  decodes it back to RGB and writes a normal JPEG (quality 90; depth PNG zlib level 1).
  Images are read and written with `imageio`. Depth is a float32 EXR, read by
  `hydra_python.image_store.exr` (pure numpy): no imageio backend reads EXR out of the
  box (its `opencv` plugin needs `OPENCV_IO_ENABLE_OPENEXR=1` before `cv2` is imported,
  which a library cannot guarantee; `freeimage` needs a binary download). EXR encodings
  Hydra does not write (PIZ, B44, tiled, ...) fall back to imageio's `opencv` plugin.
- **Track observations** -> OBSERVATION frames (kind 2), or the KEYFRAME frame of the same
  sensor and stamp, and `observations` rows, kept and not kept. Kept ones reference the
  logged colour/depth/mask files (hardlinked into the store, `--copy` to copy).
- **Descriptors** (optional): when an archive carries a feature vector (Hydra
  `KeyframeSelector.save_fields` with `features`; off by default) it is stored for the frame
  in the `frame_descriptors` table (`dim`, numpy `dtype`, raw `data`). `label_features`
  are not stored.
- **Bindings**: a track binds to the node of the track that absorbed it (Khronos
  `absorbed_track_ids`, transitively), else its own newest binding. Tracks without one keep
  their observations with no `track_bindings` row; `ImageStore.unbound_tracks()` lists them
  with the reason, recorded in meta `indexer.r<robot>.unbound_tracks`: `extraction_failed`
  (`extracted: false` and no binding or only an ended one, e.g. `track_removed`),
  `node_deleted` (the frontend deleted the node), `node_missing` (the bound node, after
  merges, is not in the backend scene graph; checked only when that graph is saved) or
  `no_binding`.
- **Aliases**: backend `merges.json` (final parents). A re-index replaces the aliases this
  robot's previous indexing recorded (meta `indexer.r<robot>.aliases`); aliases other
  writers recorded are kept.
- **Anchors**: every frame, keyframe or not, is anchored to the nearer in time of the
  robot's agent nodes just before and after it (ties: the earlier); none when its
  odometric position is more than `--max-anchor-dist` from the frame's. `anchor_T_body =
  odom_world_T_anchor^-1 @ world_T_body_odom`, with odometric agent poses from the
  frontend `dsg.json`, else the archive at the node's stamp (within 1 ms), else (warned)
  the backend pose. The keyframe taken at an agent node is the one anchored to it nearest
  to its stamp (`ImageStore.agent_frame`). With `--partial` (Hydra still
  running) frames newer than the newest agent node stay pending for the next run.
- **Ids**: `(robot << 48) | seq` as for every writer. The robot id defaults to the
  `robot_id` in the track logs, else the one the store was indexed with, else the one whose agent prefix (`'a' + id`) the frontend
  (else backend) scene graph holds, else 0.
- **Bindings file**: `layers` keys are graph_updater layer names (`OBJECTS`); the object
  layer is found case-insensitively (else every layer is used). Several tracks may bind to
  one node. Node merges come from `merges.json` only.
- **Track logs** must follow `FORMAT.md` §2 (`robot_id`, `track_id`, `absorbed_track_ids`,
  `extracted`, `semantic_label`, every observation key; `mask_rle` a string or `null`).
  A malformed `track.json` is skipped with a warning (`tracks_failed`).

## Reading

```python
from hydra_python.image_store import ImageStore, load_dsg

store = ImageStore.open("run/image_store", "robot2/image_store")  # readonly=True
# a missing store raises FileNotFoundError; readonly=False, create=True creates one
store.canonical(node_id) -> int            # follows node_aliases chains (cycle-safe)
store.members(node_id) -> set[int]          # canonical + everything merged into it
store.observations(node_id, kept_only=False, limit=None, order="score"|"time") -> list[Observation]
store.track_observations(robot_id, track_id) -> list[Observation]
store.frames_for_agent(node_id, timestamp_ns=None) -> list[Frame]  # keyframes anchored to it
store.agent_frame(node_id, timestamp_ns, max_dt_ns=50_000_000) -> Frame | None  # its own image
store.frames(robot_id=None, t0=None, t1=None, kind=None) -> list[Frame]
store.frame(frame_id) -> Frame | None
store.calib(sensor_id) -> Calib | None      # intrinsics, K, depth_scale, body_T_sensor
store.frame_pose(frame, pose_lookup=None, with_source=False) -> 4x4 (, "anchor"|"odom")
store.mask(obs) -> np.ndarray[bool] | None  # RLE decode, else the mask PNG
store.object_nodes(), store.agent_nodes(), store.info()
store.unbound_tracks() -> list[(robot_id, track_id, reason)]   # observed tracks without a node
store.descriptor(frame_id) -> np.ndarray | None                 # optional frame descriptor
store.descriptors(robot_id=None, kind=None) -> Iterator[(Frame, np.ndarray)]  # time order
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

## Writing (tools)

The indexer writes through `ImageStoreWriter`; other tools can add their own rows with
track ids from `EXTERNAL_TRACK_BASE` up:

```python
w = ImageStore.open_writer("run/image_store", robot_id=0)  # creates it (create=False: raise)
sid = w.add_sensor(name, width, height, fx, fy, cx, cy, depth_scale=0.001, body_T_sensor=None)
fid = w.add_frame_ref(sid, ts, kind=FrameKind.OBSERVATION, world_T_body_odom=None,
                      rgb=path, depth=path, anchor_node_id=None, anchor_T_body=None,
                      link="hardlink"|"copy")
w.set_anchor(fid, node_id, anchor_T_body=None)
w.set_descriptor(fid, vector)                           # replace; None removes it
oid = w.add_observation(track_id, fid, timestamp_ns=None, bbox2d=None, bbox3d=None,
                        mask=None, mask_rle=None, mask_path=None, score=0.0, kept=False)
w.bind_track(track_id, node_id)
w.record_merges({child: parent})
with w.batch(): ...                                     # one transaction
```

`open_writer` never runs the indexer; given a run directory it opens (or creates) its
`image_store/`. `add_frame_ref` deduplicates on (sensor, timestamp). For an existing
frame it only fills in missing image paths and a missing anchor. Image files already
inside the store are referenced in place. Other files are hardlinked by default, with a
copy when the hardlink fails. `mask` (bool HxW) is stored as RLE; `mask_path` places an
existing mask image under `masks/r<robot>/`. Ids are allocated inside the insert
transaction as `max(seq of this robot) + 1`. A reopened store resumes numbering, and the
indexer and other writers can work on the same file at the same time.

## CLI

```
hydra image-store info <store>... [--json]
hydra image-store export <store>... --out DIR [--dsg dsg.json] [--copy] [--kept-only]
hydra image-store check STORE_OR_RUN [--dsg dsg.json] [--json]
hydra image-store index RUN [--keyframes DIR] [--tracks DIR] [--hydra DIR] ...   # see Indexer
```

`python -m hydra_python.image_store ...` is the same CLI without the rest of `hydra`
(whose other commands import matplotlib/seaborn).

**export** writes one folder per node: `objects/<label>/<ts>_{rgb.jpg,depth.png,mask.png,meta.json}`
for objects (merged children go under their surviving parent) and `agents/<label>/` with
the keyframes anchored to each agent node, where `<label>` is the node symbol (`O_12`,
`a_3`). `index.json` maps folders to node ids. Files are hardlinked, with a symlink when
the hardlink fails (for example across filesystems), or copied with `--copy`. Each
`meta.json` holds:
- `world_T_sensor` and `world_T_body`, corrected through `--dsg` (`pose_source` says
  whether the anchor or odometry was used)
- the bboxes, timestamp, ids, score and kept flag
- the calibration

**check** runs cheap structural checks (`check_store(store_or_run_dir, dsg=None)` returns
`{"ok", "checks": {name: bool}, "stats", "store"}`): frames reference existing sensors,
observations existing frames, every referenced file exists, and sampled images match the
calibration size. With `--dsg`, keyframe anchors and bound nodes must
be DSG nodes or merge children (`node_aliases`). `stats` also counts unanchored keyframes.

## Tests

```
cd hydra/python && python -m pytest tests/image_store   # do not source ROS (PYTHONPATH)
```

The indexer tests also need `opencv-python-headless`: they encode archives and track logs
exactly as Hydra's `input_data_io.cpp` and Khronos' track logger do (OpenCV, with
`OPENCV_IO_ENABLE_OPENEXR=1` set in `conftest.py`). `data/real_cpp/` holds files written
by the C++ writers' own tests. spark_dsg (`numpy<2` for the PyPI wheel) is optional.
