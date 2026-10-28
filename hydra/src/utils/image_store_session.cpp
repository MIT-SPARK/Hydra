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
#include "hydra/utils/image_store_session.h"

#include <glog/logging.h>
#include <sqlite3.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "hydra/utils/image_store.h"

namespace hydra::image_store {

namespace fs = std::filesystem;

namespace {

//! Read-only query (stmt is null if the statement cannot be prepared)
struct Query {
  Query(sqlite3* db, const std::string& sql) {
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
      sqlite3_finalize(stmt);
      stmt = nullptr;
    }
  }

  ~Query() { sqlite3_finalize(stmt); }

  Query(const Query&) = delete;
  Query& operator=(const Query&) = delete;

  bool step() { return stmt && sqlite3_step(stmt) == SQLITE_ROW; }

  sqlite3_stmt* stmt = nullptr;
};

bool isNonEmptyDirectory(const fs::path& path) {
  std::error_code ec;
  if (!fs::is_directory(path, ec)) {
    return false;
  }

  return fs::directory_iterator(path, ec) != fs::directory_iterator();
}

std::string jsonString(const std::string& value) {
  std::stringstream ss;
  ss << '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        ss << "\\\"";
        break;
      case '\\':
        ss << "\\\\";
        break;
      case '\n':
        ss << "\\n";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          ss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c)
             << std::dec;
        } else {
          ss << c;
        }
    }
  }
  ss << '"';
  return ss.str();
}

std::string jsonStrings(const std::vector<std::string>& values) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    ss << (i ? ", " : "") << jsonString(values[i]);
  }
  ss << "]";
  return ss.str();
}

std::string utcTimestamp(std::time_t time) {
  std::tm tm;
  gmtime_r(&time, &tm);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return ss.str();
}

//! JSON object summarizing a store database ("null" if it cannot be read)
std::string summarizeDatabase(const fs::path& db_path) {
  if (!fs::exists(db_path)) {
    return "null";
  }

  sqlite3* db = nullptr;
  if (sqlite3_open_v2(db_path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) !=
      SQLITE_OK) {
    sqlite3_close(db);
    return "null";
  }

  std::stringstream ss;
  ss << "{";
  bool first = true;
  const auto add = [&](const std::string& key, const std::string& sql) {
    // tables of an older or partially created store may be missing
    Query query(db, sql);
    if (query.step() && sqlite3_column_type(query.stmt, 0) != SQLITE_NULL) {
      ss << (first ? "" : ", ") << jsonString(key) << ": "
         << sqlite3_column_int64(query.stmt, 0);
      first = false;
    }
  };

  add("sensors", "SELECT COUNT(*) FROM sensors");
  add("frames", "SELECT COUNT(*) FROM frames");
  add("keyframes", "SELECT COUNT(*) FROM frames WHERE kind = 0");
  add("subkeyframes", "SELECT COUNT(*) FROM frames WHERE kind = 1");
  add("observations", "SELECT COUNT(*) FROM observations");
  add("observations_kept", "SELECT COUNT(*) FROM observations WHERE kept != 0");
  add("track_bindings", "SELECT COUNT(*) FROM track_bindings");
  add("node_aliases", "SELECT COUNT(*) FROM node_aliases");
  add("first_timestamp_ns", "SELECT MIN(timestamp_ns) FROM frames");
  add("last_timestamp_ns", "SELECT MAX(timestamp_ns) FROM frames");
  Query robots(
      db,
      "SELECT robot_id FROM sensors UNION SELECT robot_id FROM frames ORDER BY "
      "robot_id");
  if (robots.stmt) {
    ss << (first ? "" : ", ") << "\"robot_ids\": [";
    for (bool first_id = true; robots.step(); first_id = false) {
      ss << (first_id ? "" : ", ") << sqlite3_column_int64(robots.stmt, 0);
    }
    ss << "]";
  }

  ss << "}";
  sqlite3_close(db);
  return ss.str();
}

//! Create the next session directory <sessions>/<k> (k = 1 + largest existing k)
fs::path claimSessionDirectory(const fs::path& sessions) {
  fs::create_directories(sessions);
  uint64_t last = 0;
  for (const auto& entry : fs::directory_iterator(sessions)) {
    const auto name = entry.path().filename().string();
    if (!name.empty() && name.size() < 16 &&
        std::all_of(name.begin(), name.end(), [](unsigned char c) {
          return std::isdigit(c);
        })) {
      last = std::max<uint64_t>(last, std::stoull(name));
    }
  }

  // create_directory is atomic, so concurrent rotations never share a directory
  for (uint64_t k = last + 1;; ++k) {
    const auto dir = sessions / std::to_string(k);
    if (fs::create_directory(dir)) {
      return dir;
    }
  }
}

//! Move the store of an earlier session out of the way (see Config::on_existing)
bool rotateStore(const ImageStoreConfig& config) {
  const auto& path = config.path;
  const auto session_dir = claimSessionDirectory(path.parent_path() / "sessions");
  const auto archived = session_dir / path.filename();
  fs::rename(path, archived);
  const auto rotated_at = std::time(nullptr);

  std::vector<std::string> copied, missing, failed;
  fs::path artifacts_dest;
  if (!config.session_artifacts_dir.empty()) {
    artifacts_dest = session_dir / config.session_artifacts_dir.filename();
    for (const auto& relative : config.session_artifacts) {
      const auto source = config.session_artifacts_dir / relative;
      std::error_code ec;
      if (!fs::is_regular_file(source, ec)) {
        missing.push_back(relative);
        continue;
      }

      const auto target = artifacts_dest / relative;
      fs::create_directories(target.parent_path(), ec);
      if (!ec) {
        fs::copy_file(source, target, fs::copy_options::overwrite_existing, ec);
      }

      if (ec) {
        LOG(ERROR) << "[ImageStore] Failed to copy " << source << " to " << target
                   << ": " << ec.message();
        failed.push_back(relative);
      } else {
        copied.push_back(relative);
      }
    }
  }

  std::string last_modified = "null";
  std::error_code ec;
  const auto db_time = fs::last_write_time(archived / ImageStore::kDatabaseName, ec);
  if (!ec) {
    const auto sys_time =
        std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            db_time - fs::file_time_type::clock::now() +
            std::chrono::system_clock::now());
    last_modified =
        jsonString(utcTimestamp(std::chrono::system_clock::to_time_t(sys_time)));
  }

  std::ofstream out(session_dir / "session.json");
  out << "{\n"
      << "  \"session\": " << jsonString(session_dir.filename().string()) << ",\n"
      << "  \"store\": " << jsonString(archived.filename().string()) << ",\n"
      << "  \"source_store\": " << jsonString(path.string()) << ",\n"
      << "  \"rotated_at\": " << jsonString(utcTimestamp(rotated_at)) << ",\n"
      << "  \"rotated_by_pid\": " << ::getpid() << ",\n"
      << "  \"store_last_modified\": " << last_modified << ",\n"
      << "  \"counts\": " << summarizeDatabase(archived / ImageStore::kDatabaseName)
      << ",\n"
      << "  \"artifacts_source\": "
      << (config.session_artifacts_dir.empty()
              ? std::string("null")
              : jsonString(config.session_artifacts_dir.string()))
      << ",\n"
      << "  \"artifacts_dir\": "
      << (artifacts_dest.empty() ? std::string("null")
                                 : jsonString(artifacts_dest.filename().string()))
      << ",\n"
      << "  \"artifacts\": " << jsonStrings(copied) << ",\n"
      << "  \"artifacts_missing\": " << jsonStrings(missing) << ",\n"
      << "  \"artifacts_failed\": " << jsonStrings(failed) << "\n"
      << "}\n";

  LOG(WARNING) << "[ImageStore] " << path
               << " held the store of an earlier session (node and track ids restart "
                  "with every session): moved it to "
               << archived << " and copied " << copied.size() << " file(s) of "
               << (config.session_artifacts_dir.empty() ? fs::path("<none>")
                                                        : config.session_artifacts_dir)
               << "; this session starts with an empty store";
  return true;
}

}  // namespace

bool prepareSession(const ImageStoreConfig& config) {
  if (!isNonEmptyDirectory(config.path)) {
    return true;
  }

  switch (config.on_existing) {
    case ImageStoreConfig::OnExisting::Append:
      LOG(WARNING) << "[ImageStore] " << config.path
                   << " holds the store of an earlier session and on_existing is "
                      "'append': node and track ids of this session can collide with "
                      "its anchors, track bindings and node aliases";
      return true;
    case ImageStoreConfig::OnExisting::Fail:
      LOG(ERROR) << "[ImageStore] " << config.path
                 << " holds the store of an earlier session and on_existing is 'fail'; "
                    "not opening it";
      return false;
    case ImageStoreConfig::OnExisting::Rotate:
    default:
      try {
        return rotateStore(config);
      } catch (const std::exception& e) {
        LOG(ERROR) << "[ImageStore] Failed to move the earlier session out of "
                   << config.path << ": " << e.what() << "; not opening it";
        return false;
      }
  }
}

}  // namespace hydra::image_store
