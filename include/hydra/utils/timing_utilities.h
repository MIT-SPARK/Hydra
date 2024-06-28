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
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "hydra/utils/log_utilities.h"

namespace hydra {

namespace timing {

struct ElapsedStatistics {
  double last_s;
  double mean_s;
  double min_s;
  double max_s;
  double stddev_s;
  size_t num_measurements;
};

std::ostream& operator<<(std::ostream& out, const ElapsedStatistics& stats);

class ElapsedTimeRecorder {
 public:
  static ElapsedTimeRecorder& instance() {
    if (!instance_) {
      instance_.reset(new ElapsedTimeRecorder());
    }
    return *instance_;
  }

  void start(const std::string& timer_name, const uint64_t& timestamp);

  void stop(const std::string& timer_name);

  void record(const std::string& timer_name,
              const uint64_t timestamp,
              const std::chrono::nanoseconds elapsed);

  void reset();

  std::optional<double> getLastElapsed(const std::string& timer_name) const;

  ElapsedStatistics getStats(const std::string& timer_name) const;

  void logElapsed(const std::string& name, const LogSetup& log_config) const;

  void logAllElapsed(const LogSetup& log_config) const;

  std::string getPrintableStats() const;

  void logStats(const std::string& stat_filepath) const;

  void setupIncrementalLogging(const LogSetup::Ptr& log_config);

  bool timing_disabled;

  bool disable_output;

  bool log_to_same_folder = false;

 private:
  using TimeList = std::list<std::chrono::nanoseconds>;
  using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
  using TimeMap = std::map<std::string, TimePoint>;
  using TimeStamps = std::list<uint64_t>;
  using TimeStamp = std::map<std::string, uint64_t>;

  ElapsedTimeRecorder();

  static std::unique_ptr<ElapsedTimeRecorder> instance_;

  TimeMap starts_;
  TimeStamp start_stamps_;
  std::map<std::string, TimeList> elapsed_;
  std::map<std::string, TimeStamps> stamps_;
  mutable std::mutex mutex_;

  bool log_incrementally_;
  LogSetup::Ptr log_setup_;
  std::map<std::string, std::shared_ptr<std::ofstream>> files_;
};

class ScopedTimer {
 public:
  explicit ScopedTimer(const std::string& name, uint64_t timestamp);

  ScopedTimer(const std::string& name,
              uint64_t timestamp,
              bool verbose,
              int verbosity,
              bool elapsed_only = true,
              bool verbosity_disables = false);

  ~ScopedTimer();

  void start();
  void stop();
  void reset(const std::string& name);
  void reset(const std::string& name, uint64_t timestamp);

 private:
  std::string name_;
  uint64_t timestamp_;
  bool verbose_;
  int verbosity_;
  bool elapsed_only_;
  bool verbosity_disables_;
  bool is_running_ = false;
};

}  // namespace timing
}  // namespace hydra
