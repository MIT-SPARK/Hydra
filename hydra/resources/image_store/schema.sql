-- Hydra image store schema (version 1).
--
-- Images live as files next to this database; every *_path column is relative to the
-- directory containing store.sqlite. Transforms are 16 little-endian float64 values,
-- row-major 4x4. Ids for frames and observations use the same robot prefix as spark_dsg
-- node ids ((robot_id << 48) | sequence) so stores from different robots never collide.
-- Masks are run-length encoded as a JSON array of counts over the row-major image,
-- starting with a run of background pixels.
--
-- Sessions: scene graph node ids and track ids restart with every Hydra process, so a
-- store only describes the session that wrote it. By default (image_store.on_existing:
-- append) Hydra keeps writing into an existing store (ids can collide). With rotate,
-- when Hydra first opens a non-empty store directory, it moves it aside and starts an
-- empty store at the configured path:
--   <run>/image_store/                  live store of the current session
--   <run>/sessions/<k>/image_store/     store of the k-th earlier session (k = 1, 2, ...)
--   <run>/sessions/<k>/hydra/           copy of that session's saved scene graphs/meshes
--                                       (image_store.session_artifacts_dir/_artifacts)
--   <run>/sessions/<k>/session.json     rotation time, source paths, counts, robot ids
-- so <run>/sessions/<k> has the layout of a run directory. on_existing: fail refuses to
-- open a non-empty store. Other writers (e.g. the python package) never rotate.

PRAGMA journal_mode = WAL;

CREATE TABLE IF NOT EXISTS meta (
  key TEXT PRIMARY KEY,
  value TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS sensors (
  sensor_id INTEGER PRIMARY KEY,
  robot_id INTEGER NOT NULL,
  name TEXT NOT NULL,
  width INTEGER NOT NULL,
  height INTEGER NOT NULL,
  fx REAL NOT NULL,
  fy REAL NOT NULL,
  cx REAL NOT NULL,
  cy REAL NOT NULL,
  depth_scale REAL NOT NULL,      -- meters per depth unit (16UC1 mm => 0.001)
  body_T_sensor BLOB NOT NULL,
  UNIQUE (robot_id, name)
);

-- kind: 0 = agent keyframe, 1 = sub-keyframe, 2 = object observation frame.
-- A keyframe is anchored to its own agent node with an identity anchor_T_body. Any other
-- frame is anchored to a nearby agent node once one is available; until then
-- anchor_node_id is NULL and readers fall back to the odometric pose.
CREATE TABLE IF NOT EXISTS frames (
  frame_id INTEGER PRIMARY KEY,
  robot_id INTEGER NOT NULL,
  sensor_id INTEGER NOT NULL REFERENCES sensors (sensor_id),
  timestamp_ns INTEGER NOT NULL,
  kind INTEGER NOT NULL,
  world_T_body_odom BLOB NOT NULL,
  anchor_node_id INTEGER,
  anchor_T_body BLOB,
  rgb_path TEXT,
  depth_path TEXT,
  UNIQUE (sensor_id, timestamp_ns)
);
CREATE INDEX IF NOT EXISTS frames_by_anchor ON frames (anchor_node_id);
CREATE INDEX IF NOT EXISTS frames_by_time ON frames (robot_id, timestamp_ns);

-- One row per time a tracked object was seen. Every observation is recorded; only the
-- best max_images_per_object of each track keep pixels (kept = 1, mask_path and the
-- frame's rgb_path set).
CREATE TABLE IF NOT EXISTS observations (
  obs_id INTEGER PRIMARY KEY,
  robot_id INTEGER NOT NULL,
  track_id INTEGER NOT NULL,
  frame_id INTEGER NOT NULL REFERENCES frames (frame_id),
  timestamp_ns INTEGER NOT NULL,
  bbox2d_min_x INTEGER, bbox2d_min_y INTEGER, bbox2d_max_x INTEGER, bbox2d_max_y INTEGER,
  bbox3d_min_x REAL, bbox3d_min_y REAL, bbox3d_min_z REAL,
  bbox3d_max_x REAL, bbox3d_max_y REAL, bbox3d_max_z REAL,
  mask_rle TEXT,
  mask_path TEXT,
  score REAL NOT NULL DEFAULT 0,
  kept INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS observations_by_track ON observations (robot_id, track_id);
CREATE INDEX IF NOT EXISTS observations_by_frame ON observations (frame_id);

-- Which scene graph node a track became (written by the frontend graph updater).
CREATE TABLE IF NOT EXISTS track_bindings (
  robot_id INTEGER NOT NULL,
  track_id INTEGER NOT NULL,
  node_id INTEGER NOT NULL,
  PRIMARY KEY (robot_id, track_id)
);
CREATE INDEX IF NOT EXISTS track_bindings_by_node ON track_bindings (node_id);

-- Node merges applied by the backend: child was merged into parent (final parent, as in
-- SharedDsgInfo::merges).
CREATE TABLE IF NOT EXISTS node_aliases (
  child_id INTEGER PRIMARY KEY,
  parent_id INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS node_aliases_by_parent ON node_aliases (parent_id);

INSERT OR IGNORE INTO meta (key, value) VALUES ('schema_version', '1');
