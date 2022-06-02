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
#include "hydra_utils/timing_utilities.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>

namespace hydra {
namespace timing {

decltype(ElapsedTimeRecorder::instance_) ElapsedTimeRecorder::instance_;

std::ostream& operator<<(std::ostream& out, const ElapsedStatistics& stats) {
  return out << "elapsed: " << stats.last_s << " [s] (" << stats.mean_s << " +/- "
             << stats.stddev_s << " [s] with " << stats.num_measurements
             << " measurements)";
}

ElapsedTimeRecorder::ElapsedTimeRecorder() : disable_output(false) {
  mutex_.reset(new std::mutex());
}

void ElapsedTimeRecorder::start(const std::string& timer_name,
                                const uint64_t& timestamp) {
  bool have_start_already = false;
  {  // start critical section
    std::unique_lock<std::mutex> lock(*mutex_);
    if (starts_.count(timer_name)) {
      have_start_already = true;
    } else {
      starts_[timer_name] = std::chrono::high_resolution_clock::now();
      start_stamps_[timer_name] = timestamp;
    }
  }  // end critical section

  if (have_start_already) {
    LOG(ERROR) << "Timer " << timer_name
               << " was already started. Discarding current time point";
  }
}

void ElapsedTimeRecorder::stop(const std::string& timer_name) {
  // we grab the time point first (to not mess up timing with later processing)
  const auto stop_point = std::chrono::high_resolution_clock::now();

  bool no_start_present = false;
  {  // start critical section
    std::unique_lock<std::mutex> lock(*mutex_);

    if (!starts_.count(timer_name)) {
      no_start_present = true;
    } else {
      if (!elapsed_.count(timer_name)) {
        elapsed_[timer_name] = TimeList();
        stamps_[timer_name] = TimeStamps();
      }

      elapsed_[timer_name].push_back(stop_point - starts_.at(timer_name));
      stamps_[timer_name].push_back(start_stamps_.at(timer_name));
      starts_.erase(timer_name);
    }
  }  // end critical section

  if (no_start_present) {
    LOG(ERROR) << "Timer " << timer_name
               << " was not started. Discarding current time point";
  }
}

void ElapsedTimeRecorder::reset() { instance_.reset(new ElapsedTimeRecorder()); }

std::optional<double> ElapsedTimeRecorder::getLastElapsed(
    const std::string& name) const {
  std::optional<std::chrono::nanoseconds> elapsed_ns = std::nullopt;

  {  // start critical section
    std::unique_lock<std::mutex> lock(*mutex_);
    if (elapsed_.count(name)) {
      elapsed_ns = elapsed_.at(name).back();
    }
  }  // end critical section

  if (!elapsed_ns) {
    return std::nullopt;
  }

  std::chrono::duration<double> elapsed_s = *elapsed_ns;
  return elapsed_s.count();
}

ElapsedStatistics ElapsedTimeRecorder::getStats(const std::string& name) const {
  TimeList durations;
  {  // start critical section
    std::unique_lock<std::mutex> lock(*mutex_);

    if (!elapsed_.count(name)) {
      return {0.0, 0.0, 0.0, 0.0, 0.0, 0};
    }

    durations = elapsed_.at(name);
  }  // end critical section

  const double last_elapsed = *getLastElapsed(name);

  if (durations.size() == 1) {
    return {last_elapsed, last_elapsed, last_elapsed, last_elapsed, 0.0, 1};
  }

  const size_t N = durations.size();
  const double mean = std::accumulate(
      durations.begin(), durations.end(), 0.0, [&](double total, const auto& elapsed) {
        std::chrono::duration<double> elapsed_s = elapsed;
        return total + (elapsed_s.count() / N);
      });

  std::chrono::duration<double> min_elapsed_s =
      *std::min_element(durations.begin(), durations.end());

  std::chrono::duration<double> max_elapsed_s =
      *std::max_element(durations.begin(), durations.end());

  const double variance = std::accumulate(
      durations.begin(), durations.end(), 0.0, [&](double total, const auto& elapsed) {
        std::chrono::duration<double> elapsed_s = elapsed;
        const double mean_diff = elapsed_s.count() - mean;
        return total + (mean_diff * mean_diff / N);
      });

  return {last_elapsed,
          mean,
          min_elapsed_s.count(),
          max_elapsed_s.count(),
          std::sqrt(variance),
          durations.size()};
}

void ElapsedTimeRecorder::logElapsed(const std::string& name,
                                     const std::string& output_folder) const {
  const std::string output_csv = output_folder + "/" + name + "_timing_raw.csv";
  std::ofstream output_file;
  output_file.open(output_csv);
  TimeList durations;
  TimeStamps stamps;
  {  // start critical section
    std::unique_lock<std::mutex> lock(*mutex_);

    if (!elapsed_.count(name)) {
      output_file.close();
      return;
    }

    durations = elapsed_.at(name);
    stamps = stamps_.at(name);
  }  // end critical section
  output_file << "timestamp(ns),elapsed(s)\n";
  TimeList::iterator d_it = durations.begin();
  TimeStamps::iterator s_it = stamps.begin();
  for (; d_it != durations.end() && s_it != stamps.end(); ++d_it, ++s_it) {
    std::chrono::duration<double> elapsed_s = *d_it;
    output_file << *s_it << "," << elapsed_s.count() << std::endl;
  }
  output_file.close();
}

void ElapsedTimeRecorder::logAllElapsed(const std::string& output_folder) const {
  for (const auto& str_timer_pair : elapsed_) {
    VLOG(1) << "Saving " << str_timer_pair.first;
    logElapsed(str_timer_pair.first, output_folder);
    VLOG(1) << "Saved " << str_timer_pair.first;
  }
}

void ElapsedTimeRecorder::logStats(const std::string& output_folder) const {
  const std::string output_csv = output_folder + "/timing_stats.csv";
  std::ofstream output_file;
  output_file.open(output_csv);

  // file format
  output_file << "name,mean[s],min[s],max[s],std-dev[s]\n";
  for (const auto& str_timer_pair : elapsed_) {
    const ElapsedStatistics& stats = getStats(str_timer_pair.first);
    output_file << str_timer_pair.first << "," << stats.mean_s << "," << stats.min_s
                << "," << stats.max_s << "," << stats.stddev_s << "\n";
  }
  output_file.close();
}

ScopedTimer::ScopedTimer(const std::string& name,
                         uint64_t timestamp,
                         bool verbose,
                         int verbosity,
                         bool elapsed_only,
                         bool verbosity_disables)
    : name_(name),
      verbose_(verbose),
      verbosity_(verbosity),
      elapsed_only_(elapsed_only),
      verbosity_disables_(verbosity_disables) {
  if (verbosity_disables_ and !VLOG_IS_ON(verbosity_)) {
    return;
  }
  ElapsedTimeRecorder::instance().start(name_, timestamp);
}

ScopedTimer::ScopedTimer(const std::string& name, uint64_t timestamp)
    : ScopedTimer(name, timestamp, false, 1, true, false) {}

ScopedTimer::~ScopedTimer() {
  if (verbosity_disables_ and !VLOG_IS_ON(verbosity_)) {
    return;
  }

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

}  // namespace timing
}  // namespace hydra
