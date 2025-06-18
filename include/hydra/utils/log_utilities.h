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
#pragma once

#include <filesystem>
#include <functional>
#include <list>
#include <set>
#include <string>

namespace hydra {

class LogSetup {
 public:
  struct Config {
    //! Output path for any saved files (empty string disables logging)
    std::filesystem::path log_dir;
    //! Summary statistics file name
    std::string timing_stats_name = "timing_stats.csv";
    //! Suffix for individual timing files
    std::string timing_suffix = "_timing_raw.csv";
    /**
     * If true log all timers into a single directory, replacing '/' with '_' in the
     * names. If false create separate directories for separators '/' (default).
     */
    bool log_raw_timers_to_single_dir = false;

    static Config fromString(const std::string& output_path);
  } const config;

  LogSetup();
  explicit LogSetup(const Config& config);
  explicit LogSetup(const std::string& output_path);
  ~LogSetup();

  std::filesystem::path getLogDir() const;
  std::filesystem::path getLogDir(const std::string& log_namespace) const;

  std::filesystem::path getTimerFilepath() const;
  std::filesystem::path getTimerFilepath(const std::string& timer_name) const;

  bool valid() const;
  operator bool() const;

  void registerExitCallback(const std::function<void(const LogSetup&)>& func);

 private:
  bool valid_;
  mutable std::set<std::string> namespaces_;
  std::list<std::function<void(const LogSetup&)>> callbacks_;
};

void declare_config(LogSetup::Config& config);

}  // namespace hydra
