# Image log format (v1)

The files Hydra and Khronos write when image logging is enabled, and that
`hydra image-store index` reads. All writers are off by default. Change this file first
if the format changes.

Conventions:
- Ids are decimal integers in JSON. Node ids are `spark_dsg` `NodeId`s (uint64,
  `NodeSymbol` value).
- Poses are 4x4 row-major lists of 16 numbers (`world_T_body`, `body_T_sensor`).
  Timestamps are integer nanoseconds.
- Every file is written to a temporary name in the same directory and renamed into
  place, so readers may look at a directory while Hydra runs. Keyframe archives use
  `<name>.zip.tmp.XXXXXX`, JSON files `<name>.tmp`: read only `*.zip` and `*.json`.
- JSON writers never emit `nan`/`inf` (written as `null`) and escape strings.

## 1. Keyframes: Hydra `KeyframeSelector.log_path`

```
<log_path>/<timestamp_ns>.zip            # InputData::save of each accepted keyframe
<log_path>/<timestamp_ns>_<sensor>.zip   # same, for every sensor but the first one saved
```
- Each keyframe the selector accepts is saved once, by a background thread; pending
  keyframes are flushed when the pipeline saves.
- The first sensor whose keyframe is saved (in this process) keeps `<timestamp_ns>.zip`.
  Every other sensor gets `_<sensor>`, with the characters of the sensor name outside
  `[A-Za-z0-9_.-]` replaced by `_`, so two sensors at one stamp never collide.
  Readers take the sensor and stamp from `metadata.yaml`, not from the file name, and
  deduplicate by (sensor, stamp).
- `save_fields` (default `[color, depth]`) selects the images kept in the archive; the
  others are cleared on a shallow copy. Names: `color`, `color_mask`, `depth`, `range`,
  `labels`, `instances`, `vertices`, `traversability`, `features`. `metadata.yaml`
  (pose, sensor, timestamp) is always present. The archive has no robot id.
- `save_options` sets the encoding (`InputData::SaveOptions`). Colour is a PNG;
  single-channel float images (depth in metres, range, traversability) are float32 TIFF,
  which OpenCV writes uncompressed and the archive deflates (`archive_compression`). The
  vertex map stays EXR (OpenCV cannot round-trip multi-channel float TIFF).
  Archives from older Hydra versions hold `depth.exr` instead; readers dispatch on the
  file name in `metadata.yaml`.
- Descriptors (optional): with `features` in `save_fields` the metadata's `feature`
  (`InputData::feature`) and `label_features` (`[{label, feature}]`) are filled;
  otherwise both are `[]`.
- Keyframe density is set by the keyframe policy (e.g. `DistancePolicy` with
  `min_pose_separation` and `max_time_separation_s`). By default the selector only sees
  the input of each full update; with Hydra `ReconstructionModule.forward_intermediate_inputs`
  or Khronos `ActiveWindow.forward_intermediate_inputs` (both default `false`) it sees
  every input. Forwarded inputs still pending at stop are emitted with a last output
  (Hydra) or the final `finishMapping` output (Khronos).
- `max_active_keyframes` (default 50, 0 = unbounded) caps the keyframes held in memory;
  the oldest are evicted (they are already saved). `max_queue_size` (default 100,
  0 = unbounded) caps the keyframes waiting to be written; when it is full, keyframes are
  dropped with a warning.

## 2. Object tracks: Khronos `active_window.track_logger.log_path`

```
<log_path>/sensors.yaml                   # every sensor the tracks reference
<log_path>/<track_id>/track.json
<log_path>/<track_id>/obs_<k>_color.jpg   # kept observations only
<log_path>/<track_id>/obs_<k>_depth.png   # uint16 millimetres, kept only, with save_depth
<log_path>/<track_id>/obs_<k>_mask.png    # uint8 0/255, full image size, kept only
```
`track.json` is written once the track's extraction result is known, i.e. after object
extraction (rewritten if the track is logged again):
```json
{"version": 1, "robot_id": 0, "track_id": 12, "absorbed_track_ids": [7, 9], "extracted": true,
 "semantic_label": 3, "observations": [
   {"k": 0, "timestamp_ns": 0,
    "world_T_body": {"translation_m": [x, y, z], "rotation_wxyz": [w, x, y, z]},
    "sensor": "camera", "image_size": [640, 480],
    "bbox_2d": [u_min, v_min, u_max, v_max], "mask_rle": null,
    "score": 0.0, "kept": true, "color": "obs_0_color.jpg", "depth": "obs_0_depth.png",
    "mask": "obs_0_mask.png"}]}
```
`sensors.yaml` holds one record per sensor, the same record as the `sensor` entry of a
keyframe archive's `metadata.yaml` (`hydra::input::writeSensor`: `Sensor::dump()` with
the extrinsics as parameters); it is written before the first `track.json` that
references the sensor:
```yaml
version: 1
sensors:
  - name: camera
    config: {type: camera, width: 640, height: 480, fx: 0, fy: 0, cx: 0, cy: 0,
             extrinsics: {type: param, body_R_sensor: {w, x, y, z}, body_p_sensor: [x, y, z]}, ...}
```
- `robot_id` is `GlobalInfo::getRobotPrefix().id` of the Khronos process.
- `absorbed_track_ids`: tracks Khronos merged into this one; the frontend only sees
  deletes for them.
- `extracted: false` if extraction did not produce an object (no scene graph node will
  appear). Such tracks are metadata only by default (all observations `kept: false`, no
  image files); `keep_images_of_dropped_tracks: true` keeps their images.
- `semantic_label` is `null` for tracks without semantics.
- Observations: every observation of the track with a semantic cluster, `k = 0..n-1` in
  observation order (by time), including those whose frame the frame buffer
  (`frame_data_buffer.max_buffer_size`) dropped before the track was logged: when the
  buffer drops a frame, the logger snapshots that frame's observations of live tracks
  (metadata always, encoded images only while the observation ranks within its track's
  best `max_images_per_object`; snapshots follow Khronos track merges and are dropped
  with deleted tracks). `track.json` is therefore the same whatever the buffer size.
- The best `max_images_per_object` by `score` (ties: older `timestamp_ns`, then sensor
  name) are `kept` and have image files; the others are metadata only (`color`, `depth`,
  `mask` are `null`) with an optional `mask_rle`.
- `mask_rle`: run lengths over the row-major full image, starting with a background run,
  serialized to a string such as `"[5,1,6]"`; `null` for kept observations (they have the
  mask PNG), when the mask is empty or with `save_mask_rle: false`. `bbox_2d` is in
  pixels, inclusive, `null` if the mask is empty.
- Every key shown above is always written (`null` where it does not apply); the indexer
  skips a `track.json` without one, or whose `sensor` is not in `sensors.yaml`.
- `world_T_body` has the layout of the keyframe metadata; non-finite values are `null`.
- `image_size` (`[width, height]`, `null` without an image) is the size of the image the
  mask and `bbox_2d` refer to; the indexer uses it when the sensor record has no size
  (sensors other than cameras).
- `obs_<k>_color.jpg` is a normal JPEG (true colours in the file), full image size.
  `sensor` equals the `InputData` sensor name of section 1.
- Config (`active_window.track_logger`): `log_path` (empty = off),
  `max_images_per_object: 1`, `score: segment_size | bbox_volume`, `save_depth: false`,
  `save_mask_rle: true`, `keep_images_of_dropped_tracks: false`, `jpeg_quality: 95`,
  `max_queue_size: 100`.

## 3. Track to node bindings: Hydra frontend save

`<frontend save dir>/track_bindings.json`
```json
{"version": 1, "layers": {"OBJECTS": [
   {"track_id": 12, "node_id": 123, "first_ns": 0, "last_ns": 0, "active": true, "end": null}
]}}
```
- One entry per (track, node) binding made during the run, not only the final state.
  `first_ns`/`last_ns` are the stamps of the frontend updates that created and last
  updated the binding.
- Ended bindings have `active: false` and `end` = `"deleted"` (the track was deleted and
  its node removed) or `"track_removed"` (the track was removed or rebound while the node
  survives through other tracks).
- `layers` keys are the `graph_updater.layer_updates` keys, i.e. the layer name (`OBJECTS`
  by default). Readers match the object layer case-insensitively.
- The frontend never merges two nodes: a new track matched to an existing node is bound to
  that node, so several tracks can share a `node_id`. Node merges come only from the
  backend (section 4).

## 4. Backend merges: backend save

`<backend save dir>/merges.json`
```json
{"version": 1, "merges": [{"child": 124, "parent": 123}]}
```
- The final child to final parent map (`SharedDsgInfo::merges`, chains resolved).
- Hydra writes it to `<log>/backend/merges.json`. The Khronos backend writes all its
  output, including `dsg.json` and `merges.json`, into the directory it is given (the log
  root, not `backend/`).

## 5. Indexer

```
hydra image-store index RUN [--keyframes DIR] [--tracks DIR] [--hydra DIR] [--robot-id N]
```
builds `RUN/image_store/` (`schema_version` 1, see `schema.sql`). Defaults when only `RUN`
is given (also used by `ImageStore.open(RUN)`, which indexes or refreshes the store):
keyframes `RUN/keyframes`, tracks `RUN/tracks`, Hydra output `RUN/hydra` with
`backend/{dsg.json,merges.json}` and `frontend/{dsg.json,track_bindings.json}`; the flags
override them.

- Frames: every keyframe archive is a keyframe frame (kind 0); an observation whose
  sensor and stamp match no archive brings in an observation frame (kind 2).
- Anchoring: each frame is anchored to the nearer in time of the agent nodes before and
  after it; `anchor_T_body = world_T_anchor^-1 * world_T_body`, both odometric. Odometric
  agent poses come from the frontend `dsg.json` (the backend only keeps its optimized
  copy), so keep saving the frontend graph; fallbacks: the keyframe archive at the agent
  node's stamp, then the backend pose. An agent node's own image is the keyframe
  anchored to it nearest to its stamp.
- Track chain: track, then the track that absorbed it, then the frontend node it is bound
  to, then the backend parent from `merges.json`.
- Ids are namespaced per robot (`robot << 48 | seq`). `--robot-id` defaults to the
  `robot_id` in `track.json`, else the robot the store was indexed with, else the robot
  whose agent prefix (`'a' + id`) the frontend (else backend) graph holds.
- Track ids from `1 << 40` up are reserved for writers other than the indexer; it never
  rebinds or deletes them.
- Unbound tracks keep their observations without a `track_bindings` row and are listed
  with a reason (`ImageStore.unbound_tracks()`): `extraction_failed` (`extracted: false`
  and no binding or only an ended one), `node_deleted`, `node_missing` (the bound node,
  after merges, is not in the saved backend graph) or `no_binding`.
- Aliases come from `merges.json` only; a re-index replaces the aliases the previous
  indexing of that robot recorded.
- `ImageStore.open(RUN)` indexes when the store was never indexed or the raw inputs changed
  since (meta `indexer.input_signature`), under the store's `.index.lock`. This writes the
  store even for a read-only open; `index=False` never writes.
- Descriptors: a non-empty archive `feature` is stored in the table
  `frame_descriptors(frame_id PRIMARY KEY, dim, dtype, data BLOB)`. `label_features` are
  not stored.
