/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/utils/image_store.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <sqlite3.h>

#include <algorithm>
#include <bit>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "hydra/image_store_schema.h"
#include "hydra/utils/image_store_session.h"

namespace hydra {

namespace fs = std::filesystem;
using spark_dsg::NodeId;

void declare_config(ImageStoreConfig& config) {
  using namespace config;
  name("ImageStoreConfig");
  field<Path>(config.path, "path");
  enum_field(config.on_existing,
             "on_existing",
             {{ImageStoreConfig::OnExisting::Append, "append"},
              {ImageStoreConfig::OnExisting::Rotate, "rotate"},
              {ImageStoreConfig::OnExisting::Fail, "fail"}});
  field<Path>(config.session_artifacts_dir, "session_artifacts_dir");
  field(config.session_artifacts, "session_artifacts");
  field(config.max_images_per_object, "max_images_per_object");
  field(config.jpeg_quality, "jpeg_quality");
  field(config.write_depth, "write_depth");
  field(config.flush_period_ms, "flush_period_ms", "ms");
  field(config.max_queue_size, "max_queue_size");
  checkInRange(config.jpeg_quality, 0, 100, "jpeg_quality");
  check(config.max_queue_size, GT, size_t(0), "max_queue_size");
}

namespace {

static_assert(std::endian::native == std::endian::little,
              "transforms are stored as little-endian doubles");

constexpr uint64_t kSequenceMask = (uint64_t(1) << 48) - 1;

std::string transformBlob(const Eigen::Isometry3d& transform) {
  // row-major 4x4
  const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> m = transform.matrix();
  return std::string(reinterpret_cast<const char*>(m.data()), sizeof(double) * 16);
}

struct Statement {
  Statement(sqlite3* db, const std::string& sql) {
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("failed to prepare '" + sql +
                               "': " + sqlite3_errmsg(db));
    }
  }

  ~Statement() { sqlite3_finalize(stmt); }

  Statement(const Statement&) = delete;
  Statement& operator=(const Statement&) = delete;

  Statement& bind(int i, int64_t value) {
    return checkBind(sqlite3_bind_int64(stmt, i, value), i);
  }

  Statement& bind(int i, double value) {
    return checkBind(sqlite3_bind_double(stmt, i, value), i);
  }

  Statement& bind(int i, const std::string& value) {
    return checkBind(
        sqlite3_bind_text(stmt, i, value.data(), value.size(), SQLITE_TRANSIENT), i);
  }

  Statement& bindBlob(int i, const std::string& value) {
    return checkBind(
        sqlite3_bind_blob(stmt, i, value.data(), value.size(), SQLITE_TRANSIENT), i);
  }

  Statement& bindNull(int i) { return checkBind(sqlite3_bind_null(stmt, i), i); }

  template <typename T>
  Statement& bindOptional(int i, const std::optional<T>& value) {
    return value ? bind(i, *value) : bindNull(i);
  }

  //! Run a statement without results (resets it afterwards)
  void run() {
    const auto ret = sqlite3_step(stmt);
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    if (ret != SQLITE_DONE && ret != SQLITE_ROW) {
      throw std::runtime_error(std::string("statement failed: ") +
                               sqlite3_errmsg(sqlite3_db_handle(stmt)));
    }
  }

  bool step() {
    const auto ret = sqlite3_step(stmt);
    if (ret == SQLITE_ROW) {
      return true;
    }

    if (ret != SQLITE_DONE) {
      throw std::runtime_error(std::string("query failed: ") +
                               sqlite3_errmsg(sqlite3_db_handle(stmt)));
    }

    return false;
  }

  int64_t integer(int col) const { return sqlite3_column_int64(stmt, col); }

  std::string text(int col) const {
    const auto ptr = sqlite3_column_text(stmt, col);
    return ptr ? reinterpret_cast<const char*>(ptr) : "";
  }

  bool isNull(int col) const { return sqlite3_column_type(stmt, col) == SQLITE_NULL; }

  sqlite3_stmt* stmt = nullptr;

 private:
  Statement& checkBind(int ret, int i) {
    if (ret != SQLITE_OK) {
      const std::string msg = sqlite3_errmsg(sqlite3_db_handle(stmt));
      sqlite3_clear_bindings(stmt);
      throw std::runtime_error("failed to bind parameter " + std::to_string(i) + ": " +
                               msg);
    }

    return *this;
  }
};

//! Owns a database connection (closed after the statements declared after it)
struct Connection {
  Connection() = default;
  ~Connection() { sqlite3_close(db); }

  Connection(const Connection&) = delete;
  Connection& operator=(const Connection&) = delete;

  sqlite3* db = nullptr;
};

void exec(sqlite3* db, const std::string& sql) {
  char* err = nullptr;
  if (sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
    const std::string msg = err ? err : "unknown error";
    sqlite3_free(err);
    throw std::runtime_error("failed to execute sql: " + msg);
  }
}

bool sameCalib(const CameraCalib& a, const CameraCalib& b) {
  return a.fx == b.fx && a.fy == b.fy && a.cx == b.cx && a.cy == b.cy &&
         a.width == b.width && a.height == b.height &&
         a.body_T_sensor.matrix() == b.body_T_sensor.matrix();
}

}  // namespace

struct ImageStore::Impl {
  //! Work for the writer thread: files are written first, then the rows are committed
  struct Job {
    std::function<void()> files;
    std::function<void()> rows;
  };

  //! In-memory index of a frame
  struct FrameEntry {
    uint64_t frame_id;
    int kind;
    bool has_rgb;
    bool has_depth;
    bool anchored;
  };

  struct SensorEntry {
    uint64_t sensor_id;
    CameraCalib calib;
    double depth_scale;
  };

  explicit Impl(const ImageStore::Config& config);
  ~Impl();

  void initDatabase();
  void loadState();
  void prepareStatements();

  uint64_t nextId(std::map<int, uint64_t>& sequences, int robot_id) {
    return makeId(robot_id, sequences[robot_id]++);
  }

  //! Queue a job (blocks while the queue is full)
  void push(Job&& job);
  void spin();
  void runBatch(std::vector<Job>& jobs);
  void ensureDirectory(const fs::path& dir);
  bool writeImage(const fs::path& relative, const cv::Mat& image) const;
  //! Write an encoded file; with exclusive, fails if the file exists
  bool writeFile(const fs::path& relative,
                 const std::vector<uchar>& bytes,
                 bool exclusive) const;

  const ImageStore::Config config;
  // declared before the statements, so it is closed after they are finalized
  Connection connection;
  sqlite3*& db = connection.db;

  // statements (only used by the writer thread after construction)
  std::unique_ptr<Statement> insert_sensor;
  std::unique_ptr<Statement> insert_frame;
  std::unique_ptr<Statement> update_frame_paths;
  std::unique_ptr<Statement> update_frame_anchor;
  std::unique_ptr<Statement> update_frame_kind;
  std::unique_ptr<Statement> insert_observation;
  std::unique_ptr<Statement> upsert_binding;
  std::unique_ptr<Statement> delete_binding;
  std::unique_ptr<Statement> upsert_alias;
  std::unique_ptr<Statement> delete_alias;
  std::unique_ptr<Statement> find_sensor;
  std::unique_ptr<Statement> sensor_exists;
  std::unique_ptr<Statement> find_frame;
  std::unique_ptr<Statement> frame_exists;
  std::unique_ptr<Statement> observation_exists;
  std::unique_ptr<Statement> frame_state;

  // state of the producers (guarded by state_mutex)
  mutable std::mutex state_mutex;
  std::map<int, uint64_t> sensor_sequences;
  std::map<int, uint64_t> frame_sequences;
  std::map<int, uint64_t> observation_sequences;
  std::map<std::pair<int, std::string>, SensorEntry> sensors;
  std::map<uint64_t, int> sensor_robots;
  std::map<std::pair<uint64_t, uint64_t>, FrameEntry> frames;  // (sensor, stamp)
  std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> frame_keys;
  //! Unanchored frames per robot keyed by (timestamp, frame id)
  std::map<int, std::map<std::pair<uint64_t, uint64_t>, Eigen::Isometry3d>> pending;
  std::map<std::pair<int, uint64_t>, NodeId> bindings;
  std::map<NodeId, NodeId> aliases;

  // writer queue (guarded by queue_mutex)
  std::mutex queue_mutex;
  std::condition_variable queue_cv;   // work available / flush / shutdown
  std::condition_variable space_cv;   // queue has space
  std::condition_variable commit_cv;  // batch committed
  std::deque<Job> queue;
  uint64_t num_queued = 0;
  uint64_t num_committed = 0;
  bool flush_requested = false;
  bool should_shutdown = false;
  std::thread writer;

  // only accessed by the writer thread
  std::set<fs::path> created_dirs;
  //! Ids that were taken by another writer (e.g., the python package) when committed
  std::unordered_map<uint64_t, uint64_t> sensor_remap;
  std::unordered_map<uint64_t, uint64_t> frame_remap;

  static uint64_t resolve(const std::unordered_map<uint64_t, uint64_t>& remap,
                          uint64_t id) {
    const auto iter = remap.find(id);
    return iter == remap.end() ? id : iter->second;
  }

  //! Whether a single-parameter query returns a row
  static bool hasRow(Statement& stmt, int64_t value) {
    stmt.bind(1, value);
    const bool found = stmt.step();
    sqlite3_reset(stmt.stmt);
    sqlite3_clear_bindings(stmt.stmt);
    return found;
  }

  //! Advance the sequences past the ids other writers committed
  void syncSequences();

  //! A new id for the robot of an id that another writer already used
  uint64_t freshId(std::map<int, uint64_t>& sequences, uint64_t taken_id) {
    std::lock_guard<std::mutex> lock(state_mutex);
    return nextId(sequences, static_cast<int>(taken_id >> 48));
  }

  //! Final id of a sensor (registered under the same name by another writer)
  uint64_t commitSensorId(uint64_t sensor_id, int robot_id, const std::string& name);

  //! Final id of a new frame; returns false if the frame exists already (as id)
  bool commitFrameId(uint64_t& frame_id, uint64_t sensor_id, uint64_t timestamp_ns);
};

ImageStore::Impl::Impl(const ImageStore::Config& config) : config(config) {
  std::error_code ec;
  fs::create_directories(config.path, ec);
  if (ec) {
    throw std::runtime_error("failed to create " + config.path.string() + ": " +
                             ec.message());
  }

  // on failure, the members (statements first, then the connection) are released
  const auto db_path = config.path / kDatabaseName;
  if (sqlite3_open_v2(db_path.c_str(),
                      &db,
                      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX,
                      nullptr) != SQLITE_OK) {
    const std::string msg = db ? sqlite3_errmsg(db) : "out of memory";
    throw std::runtime_error("failed to open " + db_path.string() + ": " + msg);
  }

  initDatabase();
  loadState();
  prepareStatements();
  writer = std::thread(&Impl::spin, this);
}

ImageStore::Impl::~Impl() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    should_shutdown = true;
  }

  queue_cv.notify_all();
  space_cv.notify_all();
  if (writer.joinable()) {
    writer.join();
  }

  // the members close the connection after finalizing the statements (the last
  // connection to close checkpoints the write-ahead log)
}

void ImageStore::Impl::initDatabase() {
  sqlite3_busy_timeout(db, 5000);
  exec(db, image_store::kSchema);
  exec(db, "PRAGMA synchronous = NORMAL;");

  Statement version(db, "SELECT value FROM meta WHERE key = 'schema_version'");
  if (!version.step()) {
    throw std::runtime_error("image store has no schema version");
  }

  const auto value = version.text(0);
  if (value != std::to_string(kSchemaVersion)) {
    throw std::runtime_error("image store schema version " + value +
                             " is not supported (expected " +
                             std::to_string(kSchemaVersion) + ")");
  }
}

namespace {

//! Advance per-robot sequences past the largest id of each robot in a table
void loadSequences(sqlite3* db,
                   const std::string& table,
                   const std::string& column,
                   std::map<int, uint64_t>& sequences) {
  Statement query(db,
                  "SELECT " + column + " >> 48, MAX(" + column + " & " +
                      std::to_string(kSequenceMask) + ") FROM " + table +
                      " GROUP BY 1");
  while (query.step()) {
    auto& next = sequences[static_cast<int>(query.integer(0))];
    next = std::max(next, static_cast<uint64_t>(query.integer(1)) + 1);
  }
}

}  // namespace

void ImageStore::Impl::syncSequences() {
  std::map<int, uint64_t> sensors_db, frames_db, observations_db;
  loadSequences(db, "sensors", "sensor_id", sensors_db);
  loadSequences(db, "frames", "frame_id", frames_db);
  loadSequences(db, "observations", "obs_id", observations_db);
  std::lock_guard<std::mutex> lock(state_mutex);
  const auto advance = [](const auto& from, auto& to) {
    for (const auto& [robot, next] : from) {
      to[robot] = std::max(to[robot], next);
    }
  };
  advance(sensors_db, sensor_sequences);
  advance(frames_db, frame_sequences);
  advance(observations_db, observation_sequences);
}

uint64_t ImageStore::Impl::commitSensorId(uint64_t sensor_id,
                                          int robot_id,
                                          const std::string& name) {
  find_sensor->bind(1, static_cast<int64_t>(robot_id)).bind(2, name);
  std::optional<uint64_t> existing;
  if (find_sensor->step()) {
    existing = find_sensor->integer(0);
  }

  sqlite3_reset(find_sensor->stmt);
  sqlite3_clear_bindings(find_sensor->stmt);
  uint64_t final_id = sensor_id;
  if (existing) {
    final_id = *existing;
  } else if (hasRow(*sensor_exists, sensor_id)) {
    final_id = freshId(sensor_sequences, sensor_id);
  }

  if (final_id != sensor_id) {
    sensor_remap[sensor_id] = final_id;
  }

  return final_id;
}

bool ImageStore::Impl::commitFrameId(uint64_t& frame_id,
                                     uint64_t sensor_id,
                                     uint64_t timestamp_ns) {
  find_frame->bind(1, static_cast<int64_t>(sensor_id))
      .bind(2, static_cast<int64_t>(timestamp_ns));
  std::optional<uint64_t> existing;
  if (find_frame->step()) {
    existing = find_frame->integer(0);
  }

  sqlite3_reset(find_frame->stmt);
  sqlite3_clear_bindings(find_frame->stmt);
  const auto original = frame_id;
  bool is_new = true;
  if (existing) {
    frame_id = *existing;
    is_new = false;
  } else if (hasRow(*frame_exists, frame_id)) {
    frame_id = freshId(frame_sequences, frame_id);
  }

  if (frame_id != original) {
    frame_remap[original] = frame_id;
  }

  return is_new;
}

void ImageStore::Impl::loadState() {
  loadSequences(db, "sensors", "sensor_id", sensor_sequences);
  loadSequences(db, "frames", "frame_id", frame_sequences);
  loadSequences(db, "observations", "obs_id", observation_sequences);

  Statement sensor_query(db,
                         "SELECT sensor_id, robot_id, name, width, height, fx, fy, cx, "
                         "cy, depth_scale, body_T_sensor FROM sensors");
  while (sensor_query.step()) {
    SensorEntry entry;
    entry.sensor_id = sensor_query.integer(0);
    const auto robot_id = static_cast<int>(sensor_query.integer(1));
    entry.calib.width = sensor_query.integer(3);
    entry.calib.height = sensor_query.integer(4);
    entry.calib.fx = sqlite3_column_double(sensor_query.stmt, 5);
    entry.calib.fy = sqlite3_column_double(sensor_query.stmt, 6);
    entry.calib.cx = sqlite3_column_double(sensor_query.stmt, 7);
    entry.calib.cy = sqlite3_column_double(sensor_query.stmt, 8);
    entry.depth_scale = sqlite3_column_double(sensor_query.stmt, 9);
    const auto blob = sqlite3_column_blob(sensor_query.stmt, 10);
    if (blob && sqlite3_column_bytes(sensor_query.stmt, 10) == 16 * sizeof(double)) {
      Eigen::Matrix<double, 4, 4, Eigen::RowMajor> m;
      std::memcpy(m.data(), blob, 16 * sizeof(double));
      entry.calib.body_T_sensor.matrix() = m;
    }

    sensor_robots[entry.sensor_id] = robot_id;
    sensors[{robot_id, sensor_query.text(2)}] = entry;
  }

  Statement frame_query(db,
                        "SELECT frame_id, sensor_id, timestamp_ns, kind, rgb_path, "
                        "depth_path, anchor_node_id FROM frames");
  while (frame_query.step()) {
    FrameEntry entry;
    entry.frame_id = frame_query.integer(0);
    entry.kind = frame_query.integer(3);
    entry.has_rgb = !frame_query.isNull(4);
    entry.has_depth = !frame_query.isNull(5);
    entry.anchored = !frame_query.isNull(6);
    const std::pair<uint64_t, uint64_t> key(frame_query.integer(1),
                                            frame_query.integer(2));
    frames[key] = entry;
    frame_keys[entry.frame_id] = key;
  }

  Statement binding_query(db, "SELECT robot_id, track_id, node_id FROM track_bindings");
  while (binding_query.step()) {
    bindings[{static_cast<int>(binding_query.integer(0)), binding_query.integer(1)}] =
        binding_query.integer(2);
  }

  Statement alias_query(db, "SELECT child_id, parent_id FROM node_aliases");
  while (alias_query.step()) {
    aliases[alias_query.integer(0)] = alias_query.integer(1);
  }
}

void ImageStore::Impl::prepareStatements() {
  insert_sensor = std::make_unique<Statement>(
      db,
      "INSERT INTO sensors (sensor_id, robot_id, name, width, height, fx, fy, cx, cy, "
      "depth_scale, body_T_sensor) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?) "
      "ON CONFLICT (sensor_id) DO UPDATE SET width = excluded.width, height = "
      "excluded.height, fx = excluded.fx, fy = excluded.fy, cx = excluded.cx, cy = "
      "excluded.cy, depth_scale = excluded.depth_scale, body_T_sensor = "
      "excluded.body_T_sensor");
  insert_frame = std::make_unique<Statement>(
      db,
      "INSERT INTO frames (frame_id, robot_id, sensor_id, timestamp_ns, kind, "
      "world_T_body_odom, anchor_node_id, anchor_T_body, rgb_path, depth_path) VALUES "
      "(?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
  update_frame_paths = std::make_unique<Statement>(
      db,
      "UPDATE frames SET rgb_path = COALESCE(?, rgb_path), depth_path = COALESCE(?, "
      "depth_path) WHERE frame_id = ?");
  update_frame_anchor = std::make_unique<Statement>(
      db, "UPDATE frames SET anchor_node_id = ?, anchor_T_body = ? WHERE frame_id = ?");
  update_frame_kind =
      std::make_unique<Statement>(db, "UPDATE frames SET kind = ? WHERE frame_id = ?");
  insert_observation = std::make_unique<Statement>(
      db,
      "INSERT INTO observations (obs_id, robot_id, track_id, frame_id, timestamp_ns, "
      "bbox2d_min_x, bbox2d_min_y, bbox2d_max_x, bbox2d_max_y, bbox3d_min_x, "
      "bbox3d_min_y, bbox3d_min_z, bbox3d_max_x, bbox3d_max_y, bbox3d_max_z, mask_rle, "
      "mask_path, score, kept) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, "
      "?, "
      "?, ?)");
  upsert_binding = std::make_unique<Statement>(
      db,
      "INSERT INTO track_bindings (robot_id, track_id, node_id) VALUES (?, ?, ?) ON "
      "CONFLICT (robot_id, track_id) DO UPDATE SET node_id = excluded.node_id");
  delete_binding = std::make_unique<Statement>(
      db, "DELETE FROM track_bindings WHERE robot_id = ? AND track_id = ?");
  upsert_alias = std::make_unique<Statement>(
      db,
      "INSERT INTO node_aliases (child_id, parent_id) VALUES (?, ?) ON CONFLICT "
      "(child_id) DO UPDATE SET parent_id = excluded.parent_id");
  delete_alias =
      std::make_unique<Statement>(db, "DELETE FROM node_aliases WHERE child_id = ?");
  find_sensor = std::make_unique<Statement>(
      db, "SELECT sensor_id FROM sensors WHERE robot_id = ? AND name = ?");
  sensor_exists =
      std::make_unique<Statement>(db, "SELECT 1 FROM sensors WHERE sensor_id = ?");
  find_frame = std::make_unique<Statement>(
      db, "SELECT frame_id FROM frames WHERE sensor_id = ? AND timestamp_ns = ?");
  frame_exists =
      std::make_unique<Statement>(db, "SELECT 1 FROM frames WHERE frame_id = ?");
  observation_exists =
      std::make_unique<Statement>(db, "SELECT 1 FROM observations WHERE obs_id = ?");
  frame_state = std::make_unique<Statement>(
      db, "SELECT kind, anchor_node_id IS NULL FROM frames WHERE frame_id = ?");
}

void ImageStore::Impl::push(Job&& job) {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    space_cv.wait(lock, [this] {
      return queue.size() < config.max_queue_size || should_shutdown;
    });
    queue.push_back(std::move(job));
    ++num_queued;
  }

  queue_cv.notify_one();
}

void ImageStore::Impl::spin() {
  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::milliseconds(config.flush_period_ms);
  auto last_commit = Clock::now();
  while (true) {
    std::vector<Job> jobs;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      // wait for work, then until the batch is due
      queue_cv.wait(lock, [this] { return !queue.empty() || should_shutdown; });
      queue_cv.wait_until(lock, last_commit + period, [this] {
        return should_shutdown || flush_requested ||
               queue.size() >= config.max_queue_size / 2;
      });

      if (queue.empty() && should_shutdown) {
        break;
      }

      flush_requested = false;
      jobs.reserve(queue.size());
      std::move(queue.begin(), queue.end(), std::back_inserter(jobs));
      queue.clear();
    }

    space_cv.notify_all();
    runBatch(jobs);
    last_commit = Clock::now();
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      num_committed += jobs.size();
    }

    commit_cv.notify_all();
  }

  commit_cv.notify_all();
}

void ImageStore::Impl::runBatch(std::vector<Job>& jobs) {
  for (auto& job : jobs) {
    if (!job.files) {
      continue;
    }

    try {
      job.files();
    } catch (const std::exception& e) {
      LOG(WARNING) << "[ImageStore] Failed to write images: " << e.what();
    }
  }

  // the in-memory state (frames, anchors, bindings, aliases) already includes the
  // dropped operations, so later writes that depend on them may be skipped
  const auto dropped = [&](const std::exception& e) {
    LOG(ERROR) << "[ImageStore] " << e.what() << "; dropped a batch of " << jobs.size()
               << " operation(s): " << config.path
               << " is missing their rows (image files written for them are orphaned) "
                  "and no longer matches the state of this writer";
  };

  try {
    // other writers (e.g., the python package) cannot commit until this batch is done,
    // so ids taken by them are detected reliably
    exec(db, "BEGIN IMMEDIATE");
    syncSequences();
  } catch (const std::exception& e) {
    sqlite3_exec(db, "ROLLBACK", nullptr, nullptr, nullptr);
    dropped(e);
    return;
  }

  for (auto& job : jobs) {
    if (!job.rows) {
      continue;
    }

    // a failing job leaves none of its rows behind
    try {
      exec(db, "SAVEPOINT job");
    } catch (const std::exception& e) {
      sqlite3_exec(db, "ROLLBACK", nullptr, nullptr, nullptr);
      dropped(e);
      return;
    }

    try {
      job.rows();
      exec(db, "RELEASE job");
    } catch (const std::exception& e) {
      LOG(WARNING) << "[ImageStore] Dropped an operation: " << e.what();
      sqlite3_exec(db, "ROLLBACK TO job", nullptr, nullptr, nullptr);
      sqlite3_exec(db, "RELEASE job", nullptr, nullptr, nullptr);
    }
  }

  try {
    exec(db, "COMMIT");
  } catch (const std::exception& e) {
    sqlite3_exec(db, "ROLLBACK", nullptr, nullptr, nullptr);
    dropped(e);
  }
}

void ImageStore::Impl::ensureDirectory(const fs::path& dir) {
  if (created_dirs.count(dir)) {
    return;
  }

  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) {
    throw std::runtime_error("failed to create " + dir.string() + ": " + ec.message());
  }

  created_dirs.insert(dir);
}

bool ImageStore::Impl::writeImage(const fs::path& relative,
                                  const cv::Mat& image) const {
  const auto path = config.path / relative;
  std::vector<int> params;
  if (path.extension() == ".jpg") {
    params = {cv::IMWRITE_JPEG_QUALITY, config.jpeg_quality};
  }

  try {
    if (cv::imwrite(path.string(), image, params)) {
      return true;
    }
  } catch (const cv::Exception& e) {
    LOG(WARNING) << "[ImageStore] Failed to write " << path << ": " << e.what();
    return false;
  }

  LOG(WARNING) << "[ImageStore] Failed to write " << path;
  return false;
}

bool ImageStore::Impl::writeFile(const fs::path& relative,
                                 const std::vector<uchar>& bytes,
                                 bool exclusive) const {
  const auto path = config.path / relative;
  std::FILE* file = std::fopen(path.c_str(), exclusive ? "wbx" : "wb");
  if (!file) {
    LOG_IF(WARNING, !exclusive || errno != EEXIST)
        << "[ImageStore] Failed to write " << path << ": " << std::strerror(errno);
    return false;
  }

  const bool written = std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
  const bool closed = std::fclose(file) == 0;
  if (!written || !closed) {
    LOG(WARNING) << "[ImageStore] Failed to write " << path;
    return false;
  }

  return true;
}

namespace {

// stores can outlive static destruction (e.g., when owned by other statics), so the
// registry is never destroyed
struct Registry {
  std::mutex mutex;
  std::condition_variable cv;
  std::map<fs::path, std::weak_ptr<ImageStore>> stores;
  //! Directories whose earlier contents were handled (see Config::on_existing)
  std::set<fs::path> prepared;
};

Registry& registry() {
  static auto* instance = new Registry();
  return *instance;
}

fs::path normalizedPath(const fs::path& path) {
  std::error_code ec;
  auto result = fs::weakly_canonical(fs::absolute(path), ec);
  if (ec) {
    result = fs::absolute(path).lexically_normal();
  }

  // drop a trailing separator so that "a/" and "a" are the same store
  if (!result.has_filename() && result.has_parent_path()) {
    result = result.parent_path();
  }

  return result;
}

}  // namespace

ImageStore::Ptr ImageStore::open(const Config& config) {
  if (config.path.empty()) {
    return nullptr;
  }

  const auto key = normalizedPath(config.path);
  auto& reg = registry();
  std::unique_lock<std::mutex> lock(reg.mutex);
  while (true) {
    auto iter = reg.stores.find(key);
    if (iter == reg.stores.end()) {
      break;
    }

    if (auto store = iter->second.lock()) {
      return store;
    }

    // the previous instance is still draining its queue
    reg.cv.wait(lock);
  }

  Config normalized = config;
  normalized.path = key;
  if (!config::isValid(normalized, true)) {
    LOG(ERROR) << "[ImageStore] Invalid config for " << key;
    return nullptr;
  }

  // ids restart with every session: handle a store left by an earlier process once
  if (!reg.prepared.count(key)) {
    if (!image_store::prepareSession(normalized)) {
      return nullptr;
    }

    reg.prepared.insert(key);
  }

  try {
    Ptr store(new ImageStore(normalized), [key](ImageStore* ptr) {
      delete ptr;  // drains the queue
      auto& reg = registry();
      std::lock_guard<std::mutex> lock(reg.mutex);
      auto iter = reg.stores.find(key);
      if (iter != reg.stores.end() && iter->second.expired()) {
        reg.stores.erase(iter);
      }

      reg.cv.notify_all();
    });
    reg.stores[key] = store;
    LOG(INFO) << "[ImageStore] Opened " << key;
    return store;
  } catch (const std::exception& e) {
    LOG(ERROR) << "[ImageStore] Failed to open " << key << ": " << e.what();
    return nullptr;
  }
}

ImageStore::Ptr ImageStore::open(const fs::path& path) {
  Config config;
  config.path = path;
  return open(config);
}

uint64_t ImageStore::makeId(int robot_id, uint64_t sequence) {
  return (static_cast<uint64_t>(robot_id) << 48) | (sequence & kSequenceMask);
}

std::string ImageStore::encodeMaskRle(const cv::Mat& mask) {
  std::stringstream ss;
  ss << "[";
  if (mask.empty()) {
    ss << "]";
    return ss.str();
  }

  CV_Assert(mask.type() == CV_8UC1);
  bool foreground = false;
  size_t run = 0;
  bool first = true;
  for (int r = 0; r < mask.rows; ++r) {
    const auto row = mask.ptr<uint8_t>(r);
    for (int c = 0; c < mask.cols; ++c) {
      const bool value = row[c] != 0;
      if (value == foreground) {
        ++run;
        continue;
      }

      ss << (first ? "" : ",") << run;
      first = false;
      foreground = value;
      run = 1;
    }
  }

  ss << (first ? "" : ",") << run << "]";
  return ss.str();
}

ImageStore::ImageStore(const Config& config)
    : config(config), impl_(std::make_unique<Impl>(config)) {}

ImageStore::~ImageStore() = default;

uint64_t ImageStore::addSensor(int robot_id,
                               const std::string& name,
                               const CameraCalib& calib,
                               double depth_scale) {
  uint64_t sensor_id;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    auto iter = impl_->sensors.find({robot_id, name});
    if (iter != impl_->sensors.end()) {
      auto& entry = iter->second;
      if (sameCalib(entry.calib, calib) && entry.depth_scale == depth_scale) {
        return entry.sensor_id;
      }

      entry.calib = calib;
      entry.depth_scale = depth_scale;
      sensor_id = entry.sensor_id;
    } else {
      sensor_id = impl_->nextId(impl_->sensor_sequences, robot_id);
      impl_->sensors[{robot_id, name}] = {sensor_id, calib, depth_scale};
      impl_->sensor_robots[sensor_id] = robot_id;
    }
  }

  auto impl = impl_.get();
  impl_->push({nullptr, [=]() {
                 const auto final_id = impl->commitSensorId(sensor_id, robot_id, name);
                 impl->insert_sensor->bind(1, static_cast<int64_t>(final_id))
                     .bind(2, static_cast<int64_t>(robot_id))
                     .bind(3, name)
                     .bind(4, static_cast<int64_t>(calib.width))
                     .bind(5, static_cast<int64_t>(calib.height))
                     .bind(6, calib.fx)
                     .bind(7, calib.fy)
                     .bind(8, calib.cx)
                     .bind(9, calib.cy)
                     .bind(10, depth_scale)
                     .bindBlob(11, transformBlob(calib.body_T_sensor))
                     .run();
               }});
  return sensor_id;
}

uint64_t ImageStore::addFrame(const Frame& frame,
                              const cv::Mat& color,
                              const cv::Mat& depth_in) {
  cv::Mat depth;
  if (config.write_depth && !depth_in.empty()) {
    depth = depth_in.type() == CV_32FC1 ? depthToKeyframe(depth_in) : depth_in;
  }

  const std::pair<uint64_t, uint64_t> key(frame.sensor_id, frame.timestamp_ns);
  const int kind = static_cast<int>(frame.kind);
  bool is_new = false;
  bool write_rgb = false;
  bool write_depth = false;
  bool set_anchor = false;
  bool lower_kind = false;
  uint64_t frame_id;
  uint64_t sensor_seq = 0;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    const auto sensor_iter = impl_->sensor_robots.find(frame.sensor_id);
    if (sensor_iter == impl_->sensor_robots.end()) {
      throw std::invalid_argument("unknown sensor id " +
                                  std::to_string(frame.sensor_id));
    }

    if (sensor_iter->second != frame.robot_id) {
      throw std::invalid_argument("frame of robot " + std::to_string(frame.robot_id) +
                                  " uses sensor " + std::to_string(frame.sensor_id) +
                                  " of robot " + std::to_string(sensor_iter->second));
    }

    sensor_seq = frame.sensor_id & kSequenceMask;
    auto iter = impl_->frames.find(key);
    if (iter == impl_->frames.end()) {
      is_new = true;
      frame_id = impl_->nextId(impl_->frame_sequences, frame.robot_id);
      write_rgb = !color.empty();
      write_depth = !depth.empty();
      impl_->frames[key] = {frame_id, kind, write_rgb, write_depth, false};
      impl_->frame_keys[frame_id] = key;
      iter = impl_->frames.find(key);
    } else {
      frame_id = iter->second.frame_id;
      write_rgb = !color.empty() && !iter->second.has_rgb;
      write_depth = !depth.empty() && !iter->second.has_depth;
      iter->second.has_rgb |= write_rgb;
      iter->second.has_depth |= write_depth;
      lower_kind = kind < iter->second.kind;
      if (lower_kind) {
        iter->second.kind = kind;
      }
    }

    auto& entry = iter->second;
    if (frame.anchor_node && !entry.anchored) {
      entry.anchored = true;
      set_anchor = !is_new;
      auto& pending = impl_->pending[frame.robot_id];
      pending.erase({frame.timestamp_ns, frame_id});
    } else if (is_new && !frame.anchor_node) {
      impl_->pending[frame.robot_id][{frame.timestamp_ns, frame_id}] =
          frame.world_T_body;
    }
  }

  std::string stem = std::to_string(frame.timestamp_ns);
  if (sensor_seq > 0) {
    stem += "_s" + std::to_string(sensor_seq);
  }

  const fs::path dir = fs::path("frames") / ("r" + std::to_string(frame.robot_id));
  const std::optional<std::string> rgb_path =
      write_rgb ? std::optional<std::string>((dir / (stem + "_rgb.jpg")).string())
                : std::nullopt;
  const std::optional<std::string> depth_path =
      write_depth ? std::optional<std::string>((dir / (stem + "_depth.png")).string())
                  : std::nullopt;

  auto impl = impl_.get();
  auto written = std::make_shared<std::pair<bool, bool>>(false, false);
  Impl::Job job;
  if (write_rgb || write_depth) {
    job.files = [=]() {
      impl->ensureDirectory(impl->config.path / dir);
      if (rgb_path) {
        written->first = impl->writeImage(*rgb_path, color);
      }

      if (depth_path) {
        written->second = impl->writeImage(*depth_path, depth);
      }
    };
  }

  const auto anchor_blob = transformBlob(frame.anchor_T_body);
  job.rows = [=]() {
    // only reference files that were written
    const auto rgb = written->first ? rgb_path : std::nullopt;
    const auto depth_file = written->second ? depth_path : std::nullopt;
    const auto sensor_id = Impl::resolve(impl->sensor_remap, frame.sensor_id);
    auto target = Impl::resolve(impl->frame_remap, frame_id);
    bool insert = is_new;
    bool anchor = set_anchor;
    bool kind_update = lower_kind;
    if (is_new && !impl->commitFrameId(target, sensor_id, frame.timestamp_ns)) {
      // another writer added the frame: fill in what this frame adds
      insert = false;
      auto& state = *impl->frame_state;
      state.bind(1, static_cast<int64_t>(target));
      if (state.step()) {
        kind_update = kind < state.integer(0);
        anchor = frame.anchor_node && state.integer(1) != 0;
      }

      sqlite3_reset(state.stmt);
      sqlite3_clear_bindings(state.stmt);
    }

    if (insert) {
      auto& stmt = *impl->insert_frame;
      stmt.bind(1, static_cast<int64_t>(target))
          .bind(2, static_cast<int64_t>(frame.robot_id))
          .bind(3, static_cast<int64_t>(sensor_id))
          .bind(4, static_cast<int64_t>(frame.timestamp_ns))
          .bind(5, static_cast<int64_t>(kind))
          .bindBlob(6, transformBlob(frame.world_T_body));
      if (frame.anchor_node) {
        stmt.bind(7, static_cast<int64_t>(*frame.anchor_node)).bindBlob(8, anchor_blob);
      } else {
        stmt.bindNull(7).bindNull(8);
      }

      stmt.bindOptional(9, rgb).bindOptional(10, depth_file).run();
      return;
    }

    if (rgb || depth_file) {
      impl->update_frame_paths->bindOptional(1, rgb)
          .bindOptional(2, depth_file)
          .bind(3, static_cast<int64_t>(target))
          .run();
    }

    if (anchor) {
      impl->update_frame_anchor->bind(1, static_cast<int64_t>(*frame.anchor_node))
          .bindBlob(2, anchor_blob)
          .bind(3, static_cast<int64_t>(target))
          .run();
    }

    if (kind_update) {
      impl->update_frame_kind->bind(1, static_cast<int64_t>(kind))
          .bind(2, static_cast<int64_t>(target))
          .run();
    }
  };

  impl_->push(std::move(job));
  return frame_id;
}

std::vector<uint64_t> ImageStore::addObservations(
    int robot_id, uint64_t track_id, const std::vector<Observation>& observations) {
  std::vector<uint64_t> ids;
  if (observations.empty()) {
    return ids;
  }

  LOG_IF(WARNING, track_id >= kMaxTrackId)
      << "[ImageStore] Track id " << track_id << " is in a reserved range";
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    for (size_t i = 0; i < observations.size(); ++i) {
      ids.push_back(impl_->nextId(impl_->observation_sequences, robot_id));
    }
  }

  const fs::path dir = fs::path("masks") / ("r" + std::to_string(robot_id));
  const auto mask_file = [dir](uint64_t obs_id) {
    return (dir / (std::to_string(obs_id) + ".png")).string();
  };

  // encoded masks and whether they were written under the observation's id
  struct Mask {
    std::vector<uchar> png;
    bool written = false;
  };

  auto masks = std::make_shared<std::vector<Mask>>(observations.size());
  auto impl = impl_.get();
  Impl::Job job;
  job.files = [=]() {
    for (size_t i = 0; i < observations.size(); ++i) {
      const auto& obs = observations[i];
      if (!obs.kept || obs.mask.empty()) {
        continue;
      }

      auto& mask = masks->at(i);
      try {
        if (!cv::imencode(".png", obs.mask, mask.png)) {
          mask.png.clear();
        }
      } catch (const cv::Exception&) {
        mask.png.clear();
      }

      if (mask.png.empty()) {
        LOG(WARNING) << "[ImageStore] Failed to encode the mask of observation "
                     << ids[i];
        continue;
      }

      impl->ensureDirectory(impl->config.path / dir);
      // never replace a file: another writer may have taken the id in the meantime
      mask.written = impl->writeFile(mask_file(ids[i]), mask.png, true);
    }
  };

  job.rows = [=]() {
    auto& stmt = *impl->insert_observation;
    for (size_t i = 0; i < observations.size(); ++i) {
      const auto& obs = observations[i];
      auto obs_id = ids[i];
      if (Impl::hasRow(*impl->observation_exists, obs_id)) {
        obs_id = impl->freshId(impl->observation_sequences, obs_id);
      }

      // masks are written before the transaction (see files above), except when the
      // id was taken by another writer or a file with its name existed already
      std::optional<std::string> mask_path;
      const auto& mask = masks->at(i);
      if (!mask.png.empty()) {
        const auto path = mask_file(obs_id);
        if ((mask.written && obs_id == ids[i]) ||
            impl->writeFile(path, mask.png, false)) {
          mask_path = path;
        }
      }

      stmt.bind(1, static_cast<int64_t>(obs_id))
          .bind(2, static_cast<int64_t>(robot_id))
          .bind(3, static_cast<int64_t>(track_id))
          .bind(4, static_cast<int64_t>(Impl::resolve(impl->frame_remap, obs.frame_id)))
          .bind(5, static_cast<int64_t>(obs.timestamp_ns));
      for (int j = 0; j < 4; ++j) {
        if (obs.bbox2d) {
          stmt.bind(6 + j, static_cast<int64_t>((*obs.bbox2d)[j]));
        } else {
          stmt.bindNull(6 + j);
        }
      }

      for (int j = 0; j < 3; ++j) {
        if (obs.bbox3d) {
          stmt.bind(10 + j, obs.bbox3d->first(j)).bind(13 + j, obs.bbox3d->second(j));
        } else {
          stmt.bindNull(10 + j).bindNull(13 + j);
        }
      }

      if (obs.mask_rle.empty()) {
        stmt.bindNull(16);
      } else {
        stmt.bind(16, obs.mask_rle);
      }

      stmt.bindOptional(17, mask_path)
          .bind(18, obs.score)
          .bind(19, static_cast<int64_t>(obs.kept ? 1 : 0))
          .run();
    }
  };

  impl_->push(std::move(job));
  return ids;
}

void ImageStore::setAnchor(uint64_t frame_id,
                           NodeId anchor_node,
                           const Eigen::Isometry3d& anchor_T_body) {
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    const auto key_iter = impl_->frame_keys.find(frame_id);
    if (key_iter == impl_->frame_keys.end()) {
      LOG(WARNING) << "[ImageStore] Cannot anchor unknown frame " << frame_id;
      return;
    }

    impl_->frames.at(key_iter->second).anchored = true;
    const int robot_id = static_cast<int>(frame_id >> 48);
    impl_->pending[robot_id].erase({key_iter->second.second, frame_id});
  }

  auto impl = impl_.get();
  const auto blob = transformBlob(anchor_T_body);
  impl_->push(
      {nullptr, [=]() {
         impl->update_frame_anchor->bind(1, static_cast<int64_t>(anchor_node))
             .bindBlob(2, blob)
             .bind(3, static_cast<int64_t>(Impl::resolve(impl->frame_remap, frame_id)))
             .run();
       }});
}

std::vector<ImageStore::PendingFrame> ImageStore::unanchoredFrames(
    int robot_id, uint64_t max_timestamp_ns) const {
  std::vector<PendingFrame> result;
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  const auto iter = impl_->pending.find(robot_id);
  if (iter == impl_->pending.end()) {
    return result;
  }

  for (const auto& [key, pose] : iter->second) {
    if (key.first > max_timestamp_ns) {
      break;
    }

    result.push_back({key.second, key.first, pose});
  }

  return result;
}

void ImageStore::abandonAnchor(uint64_t frame_id) {
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  const auto key_iter = impl_->frame_keys.find(frame_id);
  if (key_iter == impl_->frame_keys.end()) {
    return;
  }

  const int robot_id = static_cast<int>(frame_id >> 48);
  impl_->pending[robot_id].erase({key_iter->second.second, frame_id});
}

void ImageStore::bindTrack(int robot_id, uint64_t track_id, NodeId node) {
  LOG_IF(WARNING, track_id >= kMaxTrackId)
      << "[ImageStore] Track id " << track_id << " is in a reserved range";
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    auto& bound = impl_->bindings[{robot_id, track_id}];
    if (bound == node) {
      return;
    }

    bound = node;
  }

  auto impl = impl_.get();
  impl_->push({nullptr, [=]() {
                 impl->upsert_binding->bind(1, static_cast<int64_t>(robot_id))
                     .bind(2, static_cast<int64_t>(track_id))
                     .bind(3, static_cast<int64_t>(node))
                     .run();
               }});
}

void ImageStore::unbindTrack(int robot_id, uint64_t track_id) {
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    if (!impl_->bindings.erase({robot_id, track_id})) {
      return;
    }
  }

  auto impl = impl_.get();
  impl_->push({nullptr, [=]() {
                 impl->delete_binding->bind(1, static_cast<int64_t>(robot_id))
                     .bind(2, static_cast<int64_t>(track_id))
                     .run();
               }});
}

void ImageStore::recordMerges(const std::map<NodeId, NodeId>& merges,
                              const std::vector<NodeId>& removed) {
  std::vector<std::pair<NodeId, NodeId>> changed;
  std::vector<NodeId> deleted;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    for (const auto& [child, parent] : merges) {
      auto iter = impl_->aliases.find(child);
      if (iter != impl_->aliases.end() && iter->second == parent) {
        continue;
      }

      impl_->aliases[child] = parent;
      changed.emplace_back(child, parent);
    }

    for (const auto child : removed) {
      if (impl_->aliases.erase(child)) {
        deleted.push_back(child);
      }
    }
  }

  if (changed.empty() && deleted.empty()) {
    return;
  }

  auto impl = impl_.get();
  impl_->push({nullptr, [=]() {
                 for (const auto& [child, parent] : changed) {
                   impl->upsert_alias->bind(1, static_cast<int64_t>(child))
                       .bind(2, static_cast<int64_t>(parent))
                       .run();
                 }

                 for (const auto child : deleted) {
                   impl->delete_alias->bind(1, static_cast<int64_t>(child)).run();
                 }
               }});
}

void ImageStore::flush() {
  std::unique_lock<std::mutex> lock(impl_->queue_mutex);
  const auto target = impl_->num_queued;
  if (impl_->num_committed >= target) {
    return;
  }

  impl_->flush_requested = true;
  impl_->queue_cv.notify_all();
  impl_->commit_cv.wait(lock, [&] { return impl_->num_committed >= target; });
}

}  // namespace hydra
