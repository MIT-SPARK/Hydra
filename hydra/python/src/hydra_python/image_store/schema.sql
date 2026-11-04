-- Hydra image store schema (version 1).
--
-- Images live as files next to this database; every *_path column is relative to the
-- directory containing store.sqlite. Transforms are 16 little-endian float64 values,
-- row-major 4x4. Ids for frames and observations use the same robot prefix as spark_dsg
-- node ids ((robot_id << 48) | sequence) so stores from different robots never collide.
-- Masks are run-length encoded as a JSON array of counts over the row-major image,
-- starting with a run of background pixels.
--
-- Scene graph node ids and track ids restart with every Hydra process, so a store
-- describes one Hydra run: keep one run directory per Hydra session.

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

-- kind: 0 = keyframe (a Hydra keyframe archive), 2 = observation frame (a frame only an
-- object observation brought in); readers reject other values. Every frame is anchored
-- to the agent node nearest in time once one is available (anchor_T_body = body pose
-- relative to that node, from odometry); until then, or when no node is close enough,
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

-- Which scene graph node a track became. The indexer writes the bindings of Khronos
-- tracks; track ids from 1 << 40 up are reserved for other writers.
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

-- Per-frame descriptor, when one is available (e.g. the InputData feature of a keyframe
-- archive saved with KeyframeSelector save_fields: [..., features]).
CREATE TABLE IF NOT EXISTS frame_descriptors (
  frame_id INTEGER PRIMARY KEY,
  dim INTEGER NOT NULL,
  dtype TEXT NOT NULL,            -- numpy dtype string of data, e.g. '<f4'
  data BLOB NOT NULL
);

INSERT OR IGNORE INTO meta (key, value) VALUES ('schema_version', '1');
