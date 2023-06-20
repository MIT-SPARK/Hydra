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
#include "hydra/utils/log_utilities.h"

#include <config_utilities/config.h>
#include <glog/logging.h>

#include <filesystem>

namespace fs = std::filesystem;

namespace hydra {

inline bool makeDirs(const fs::path& path) {
  if (fs::exists(path)) {
    return true;
  }

  std::error_code code;
  return fs::create_directories(path, code);
}

void declare_config(LogConfig& config) {
  using namespace config;
  name("LogConfig");
  field(config.log_dir, "log_path");
  field(config.log_timing_incrementally, "log_timing_incrementally");
  field(config.timing_stats_name, "timing_stats_name");
  field(config.timing_suffix, "timing_suffix");
  field(config.log_raw_timers_to_single_dir, "log_raw_timers_to_single_dir");
}

LogSetup::LogSetup(const LogConfig& conf) : valid_(false), config_(conf) {
  if (config_.log_dir == "") {
    return;
  }

  fs::path log_path(config_.log_dir);
  if (!makeDirs(log_path)) {
    LOG(WARNING) << "Failed to make dir: " << log_path << ". logging Disabled!";
    return;
  }

  valid_ = true;
}

LogSetup::LogSetup(const std::string& output_path)
    : LogSetup(LogConfig::fromString(output_path)) {}

LogSetup::~LogSetup() {
  if (!valid_) {
    return;
  }

  for (const auto& func : callbacks_) {
    func(*this);
  }
}

std::string LogSetup::getLogDir() const { return valid_ ? config_.log_dir : ""; }

std::string LogSetup::getLogDir(const std::string& log_namespace) const {
  if (!valid_) {
    return "";
  }

  auto ns_path = fs::path(config_.log_dir) / log_namespace;
  if (!namespaces_.count(log_namespace)) {
    makeDirs(ns_path);
    namespaces_.insert(log_namespace);
  }

  return ns_path.string();
}

std::string LogSetup::getTimerFilepath() const {
  if (!valid_) {
    throw std::runtime_error("logging not configured, unable to get timer filepath");
  }

  const auto log_dir = fs::path(config_.log_dir);
  return (log_dir / config_.timing_stats_name).lexically_normal().string();
}

std::string LogSetup::getTimerFilepath(const std::string& timer_name) const {
  if (!valid_) {
    throw std::runtime_error("unable to save timer: " + timer_name);
  }

  const auto log_dir = fs::path(config_.log_dir);

  std::string used_name = timer_name;
  // If requested, replace all '/' with '_' to avoid creating a directory.
  if (config_.log_raw_timers_to_single_dir) {
    for (char& c : used_name) {
      if (c == '/') {
        c = '_';
      }
    }
  }

  const auto timer_path = fs::path(timer_name);
  const auto timer_ns = timer_path.parent_path();
  const auto filename = timer_path.stem().string() + config_.timing_suffix;
  if (timer_ns.empty()) {
    return (log_dir / filename).lexically_normal().string();
  }

  fs::path full_path(getLogDir(timer_ns.string()));
  return (full_path / filename).lexically_normal().string();
}

bool LogSetup::valid() const { return valid_; }

const LogConfig& LogSetup::config() const { return config_; }

void LogSetup::registerExitCallback(const std::function<void(const LogSetup&)>& func) {
  callbacks_.push_back(func);
}

}  // namespace hydra
