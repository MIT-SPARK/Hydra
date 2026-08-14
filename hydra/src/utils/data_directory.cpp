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
#include "hydra/utils/data_directory.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <glog/logging.h>

#include <ctime>
#include <filesystem>

namespace hydra {

namespace fs = std::filesystem;

namespace {

inline bool makeDirs(const fs::path& path) {
  if (fs::exists(path)) {
    return true;
  }

  std::error_code code;
  return fs::create_directories(path, code);
}

inline std::string getDateTime(const std::string& format) {
  auto now = std::time(nullptr);
  auto tm = *std::localtime(&now);
  std::stringstream ss;
  ss << std::put_time(&tm, format.c_str());
  return ss.str();
}

}  // namespace

void declare_config(DataDirectory::Config& config) {
  using namespace config;
  name("LogConfig");
  field(config.overwrite, "overwrite");
  field(config.allocate, "allocate");
  field(config.use_timestamp, "use_timestamp");
  field(config.timestamp_format, "timestamp_format");
}

DataDirectory::DataDirectory() : valid_(false) {}

DataDirectory::DataDirectory(const std::filesystem::path& output_path,
                             std::optional<Config> _config)
    : config(_config.value_or(Config{})), valid_(false), root_path_(output_path) {
  if (config.use_timestamp) {
    root_path_ /= getDateTime(config.timestamp_format);
  }

  if (std::filesystem::exists(root_path_) && config.overwrite) {
    LOG(WARNING) << "Overwriting existing directory: '" << root_path_.string() << "'.";
    std::filesystem::remove_all(root_path_);
  }

  if (config.allocate && !makeDirs(root_path_)) {
    LOG(WARNING) << "Failed to make dir '" << root_path_ << "'. Logging disabled!";
    return;
  }

  valid_ = true;
}

fs::path DataDirectory::path(const std::filesystem::path& sub_path) const {
  if (!valid_) {
    throw std::runtime_error("logging not configured, unable to get timer filepath");
  }

  auto ns_path = root_path_ / sub_path;
  if (config.allocate && !std::filesystem::exists(ns_path)) {
    makeDirs(ns_path);
  }

  return ns_path;
}

DataDirectory DataDirectory::child(const std::string& sub_path) const {
  Config new_config = config;
  new_config.overwrite = false;  // no need to clear children
  return DataDirectory(sub_path, new_config);
}

bool DataDirectory::valid() const { return valid_; }

DataDirectory::operator bool() const { return valid(); }

}  // namespace hydra
