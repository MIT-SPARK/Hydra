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
#include "hydra/utils/timing_utilities.h"

#include <glog/logging.h>
#include <glog/stl_logging.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

namespace hydra::timing {

bool operator<(const ElapsedTimeRecorder::Entry& lhs,
               const ElapsedTimeRecorder::Entry& rhs) {
  return lhs.elapsed < rhs.elapsed;
}

namespace {

using Entries = std::vector<ElapsedTimeRecorder::Entry>;

ElapsedStatistics computeStats(const Entries& entries) {
  if (entries.empty()) {
    return {};
  }

  const auto last_elapsed = entries.back().elapsed_seconds();
  if (entries.size() == 1) {
    return {last_elapsed, last_elapsed, last_elapsed, last_elapsed, 0.0, 1};
  }

  const size_t N = entries.size();
  const double mean = std::accumulate(
      entries.begin(), entries.end(), 0.0, [&](double total, const auto& entry) {
        return total + (entry.elapsed_seconds() / N);
      });

  const double variance = std::accumulate(
      entries.begin(), entries.end(), 0.0, [&](double total, const auto& entry) {
        const double mean_diff = entry.elapsed_seconds() - mean;
        return total + (mean_diff * mean_diff / N);
      });

  const auto min_entry = std::min_element(entries.begin(), entries.end());
  const auto max_entry = std::max_element(entries.begin(), entries.end());
  return {last_elapsed,
          mean,
          min_entry == entries.end() ? 0.0 : min_entry->elapsed_seconds(),
          max_entry == entries.end() ? 0.0 : max_entry->elapsed_seconds(),
          std::sqrt(variance),
          N};
}

void writeEntries(const std::filesystem::path& output_csv, const Entries& entries) {
  std::ofstream output_file;
  output_file.open(output_csv);
  if (!output_file.is_open()) {
    LOG(ERROR) << "Could not open file " << output_csv << " for writing";
    return;
  }

  std::stringstream ss;
  ss << "timestamp(ns),elapsed(s)\n";
  for (const auto& entry : entries) {
    ss << entry.timestamp << "," << entry.elapsed_seconds() << "\n";
  }

  output_file << ss.str();
  output_file.close();
}

std::filesystem::path getTimerPath(const std::filesystem::path& output,
                                   std::string name,
                                   const std::string& prefix,
                                   const std::string& suffix) {
  // replace all '/' with '_' to avoid creating a directory.
  std::transform(name.cbegin(), name.cend(), name.begin(), [](const char c) {
    return c == '/' ? '_' : c;
  });
  return (output / std::filesystem::path(prefix + name + suffix + ".csv"))
      .lexically_normal();
}

}  // namespace

decltype(ElapsedTimeRecorder::instance_) ElapsedTimeRecorder::instance_;

std::ostream& operator<<(std::ostream& out, const ElapsedStatistics& stats) {
  if (!stats.num_measurements) {
    out << "N/A [s] over 0 call(s)";
    return out;
  }

  out << stats.mean_s;
  if (stats.num_measurements > 1) {
    out << " +/- " << stats.stddev_s;
    out << " [" << stats.min_s << ", " << stats.max_s << "]";
  }

  out << " [s] over " << stats.num_measurements << " call(s) (last: " << stats.last_s
      << " [s])";
  return out;
}

double ElapsedTimeRecorder::Entry::elapsed_seconds() const {
  return std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
}

ElapsedTimeRecorder::ElapsedTimeRecorder()
    : timing_disabled(false), disable_output(true) {}

ElapsedTimeRecorder& ElapsedTimeRecorder::instance() {
  if (!instance_) {
    instance_.reset(new ElapsedTimeRecorder());
  }

  return *instance_;
}

void ElapsedTimeRecorder::reset() { instance_.reset(new ElapsedTimeRecorder()); }

void ElapsedTimeRecorder::start(const std::string& name, const uint64_t timestamp) {
  {  // start critical section
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = starts_.find(name);
    if (iter == starts_.end()) {
      starts_.emplace(name,
                      TimePoint{timestamp, std::chrono::high_resolution_clock::now()});
      return;  // break to avoid error statement
    }
  }  // end critical section

  LOG(ERROR) << "Timer '" << name << "' was already started. Discarding time point!";
}

void ElapsedTimeRecorder::stop(const std::string& name) {
  // we grab the time point first (to not mess up timing with later processing)
  const auto stop_point = std::chrono::high_resolution_clock::now();
  {  // start critical section
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = starts_.find(name);
    if (iter != starts_.end()) {
      add(name, iter->second.timestamp, stop_point - iter->second.now);
      starts_.erase(iter);
      return;  // break to avoid error statement
    }
  }  // end critical section

  LOG(ERROR) << "Timer '" << name << "' was not started. Discarding time point!";
}

void ElapsedTimeRecorder::record(const std::string& name,
                                 const uint64_t timestamp,
                                 const std::chrono::nanoseconds elapsed) {
  std::unique_lock<std::mutex> lock(mutex_);
  add(name, timestamp, elapsed);
}

std::vector<std::string> ElapsedTimeRecorder::timerNames() const {
  std::vector<std::string> names;
  {  // start critical section
    std::unique_lock<std::mutex> lock(mutex_);
    std::transform(elapsed_.begin(),
                   elapsed_.end(),
                   std::back_inserter(names),
                   [](const auto& entry) { return entry.first; });
  }

  return names;
}

std::optional<double> ElapsedTimeRecorder::getLastElapsed(const std::string& n) const {
  {  // start critical section
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = elapsed_.find(n);
    if (iter != elapsed_.end()) {
      // NOTE(nathan) invariant that every timer has at least one sample
      return iter->second.back().elapsed_seconds();
    }
  }  // end critical section

  return std::nullopt;
}

ElapsedStatistics ElapsedTimeRecorder::getStats(const std::string& name) const {
  std::vector<Entry> durations;
  {  // start critical section
    std::unique_lock<std::mutex> lock(mutex_);
    auto iter = elapsed_.find(name);
    if (iter != elapsed_.end()) {
      durations = iter->second;
    }
  }  // end critical section

  return computeStats(durations);
}

std::string ElapsedTimeRecorder::printAllStats() const {
  std::stringstream ss;
  const auto timers = timerNames();
  for (const auto& name : timers) {
    const auto stats = getStats(name);
    ss << name << ": " << stats << std::endl;
  }

  return ss.str();
};

//! Suffix for individual timing files
std::string timing_suffix = "_timing_raw.csv";
/**
 * If true log all timers into a single directory, replacing '/' with '_' in the
 * names. If false create separate directories for separators '/' (default).
 */
bool log_raw_timers_to_single_dir = false;

void ElapsedTimeRecorder::logTimers(const std::filesystem::path& output,
                                    const std::string& name_prefix,
                                    const std::string& name_suffix) const {
  const auto all_timers = timerNames();
  VLOG(5) << "Saving timers: [" << all_timers << "]";
  for (const auto& name : all_timers) {
    VLOG(5) << "Saving timer '" << name << "'";

    std::vector<Entry> entries;
    {  // start critical section
      std::unique_lock<std::mutex> lock(mutex_);
      auto iter = elapsed_.find(name);
      if (iter != elapsed_.end()) {
        entries = iter->second;
      }
    }  // end critical section

    if (entries.empty()) {
      LOG(ERROR) << "Invalid timer encountered while saving '" << name << "'";
      continue;
    }

    const auto output_csv = getTimerPath(output, name, name_prefix, name_suffix);
    VLOG(2) << "Writing " << entries.size() << " measurements for timer '" << name
            << "' to '" << output_csv << "'";
    writeEntries(output_csv, entries);
    VLOG(5) << "Saved timer '" << name << "'";
  }
}

void ElapsedTimeRecorder::logStats(const std::filesystem::path& filename) const {
  std::ofstream output_file;
  output_file.open(filename);

  // file format
  std::stringstream ss;
  ss << "name,mean[s],min[s],max[s],std-dev[s]\n";
  for (const auto& str_timer_pair : elapsed_) {
    const ElapsedStatistics& stats = getStats(str_timer_pair.first);
    ss << str_timer_pair.first << "," << stats.mean_s << "," << stats.min_s << ","
       << stats.max_s << "," << stats.stddev_s << "\n";
  }
  output_file << ss.str();
  output_file.close();
}

// NOTE(nathan) this is intentionally NOT threadsafe, the callee is responsible
// for locking the mutex
void ElapsedTimeRecorder::add(const std::string& name,
                              const uint64_t timestamp,
                              const std::chrono::nanoseconds elapsed) {
  auto iter = elapsed_.find(name);
  if (iter == elapsed_.end()) {
    iter = elapsed_.emplace(name, std::vector<Entry>()).first;
  }

  iter->second.emplace_back(Entry{timestamp, elapsed});
}

ScopedTimer::ScopedTimer(const std::string& name,
                         uint64_t timestamp,
                         bool verbose,
                         int verbosity,
                         bool elapsed_only,
                         bool verbosity_disables)
    : name_(name),
      timestamp_(timestamp),
      verbose_(verbose),
      verbosity_(verbosity),
      elapsed_only_(elapsed_only),
      verbosity_disables_(verbosity_disables) {
  start();
}

ScopedTimer::ScopedTimer(const std::string& name, uint64_t timestamp)
    : ScopedTimer(name, timestamp, false, 1, true, false) {}

ScopedTimer::~ScopedTimer() { stop(); }

void ScopedTimer::start() {
  if (is_running_) {
    return;
  }

  if (ElapsedTimeRecorder::instance().timing_disabled) {
    return;
  }

  if (verbosity_disables_ and !VLOG_IS_ON(verbosity_)) {
    return;
  }

  ElapsedTimeRecorder::instance().start(name_, timestamp_);
  is_running_ = true;
}

void ScopedTimer::stop() {
  if (!is_running_) {
    return;
  }

  if (ElapsedTimeRecorder::instance().timing_disabled) {
    return;
  }

  if (verbosity_disables_ and !VLOG_IS_ON(verbosity_)) {
    return;
  }

  is_running_ = false;
  ElapsedTimeRecorder::instance().stop(name_);
  if (!verbose_) {
    return;
  }

  if (ElapsedTimeRecorder::instance().disable_output) {
    return;
  }

  if (!VLOG_IS_ON(verbosity_)) {
    return;
  }

  if (elapsed_only_) {
    VLOG(verbosity_) << "{Timer " << name_
                     << "}: " << *ElapsedTimeRecorder::instance().getLastElapsed(name_)
                     << " [s] elapsed";
  } else {
    VLOG(verbosity_) << "{Timer " << name_
                     << "}: " << ElapsedTimeRecorder::instance().getStats(name_);
  }
}

void ScopedTimer::reset(const std::string& name) {
  stop();
  name_ = name;
  start();
}

void ScopedTimer::reset(const std::string& name, uint64_t timestamp) {
  stop();
  name_ = name;
  timestamp_ = timestamp;
  start();
}

}  // namespace hydra::timing
