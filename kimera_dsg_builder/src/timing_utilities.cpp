#include "kimera_dsg_builder/timing_utilities.h"

#include <glog/logging.h>

#include <cmath>
#include <numeric>

namespace kimera {

decltype(ElapsedTimeRecorder::instance_) ElapsedTimeRecorder::instance_;

std::ostream& operator<<(std::ostream& out, const ElapsedStatistics& stats) {
  return out << "elapsed: " << stats.last_s << " [s] (" << stats.mean_s << " +/- "
             << stats.stddev_s << " [s] with " << stats.num_measurements
             << " measurements)";
}

ElapsedTimeRecorder::ElapsedTimeRecorder() {}

void ElapsedTimeRecorder::start(const std::string& timer_name) {
  if (starts_.count(timer_name)) {
    LOG(ERROR) << "Timer " << timer_name
               << " was already started. Discarding current time point";
    return;
  }

  starts_[timer_name] = std::chrono::high_resolution_clock::now();
}

void ElapsedTimeRecorder::stop(const std::string& timer_name) {
  // we grab the time point first (to not mess up timing with later processing)
  const auto stop_point = std::chrono::high_resolution_clock::now();
  if (!starts_.count(timer_name)) {
    LOG(ERROR) << "Timer " << timer_name
               << " was not started. Discarding current time point";
    return;
  }

  if (!elapsed_.count(timer_name)) {
    elapsed_[timer_name] = TimeList();
  }

  elapsed_[timer_name].push_back(stop_point - starts_.at(timer_name));
  starts_.erase(timer_name);
}

void ElapsedTimeRecorder::reset() { instance_.reset(new ElapsedTimeRecorder()); }

ElapsedStatistics ElapsedTimeRecorder::getStats(const std::string& name) const {
  if (!elapsed_.count(name)) {
    return {0.0, 0.0, 0.0, 0};
  }

  const auto& durations = elapsed_.at(name);
  const size_t N = durations.size();
  const double last_elapsed = *getLastElapsed(name);
  if (durations.size() == 1) {
    return {last_elapsed, last_elapsed, 0.0, 1};
  }

  const double mean = std::accumulate(
      durations.begin(), durations.end(), 0.0, [&](double total, const auto& elapsed) {
        std::chrono::duration<double> elapsed_s = elapsed;
        return total + (elapsed_s.count() / N);
      });

  const double variance = std::accumulate(
      durations.begin(), durations.end(), 0.0, [&](double total, const auto& elapsed) {
        std::chrono::duration<double> elapsed_s = elapsed;
        const double mean_diff = elapsed_s.count() - mean;
        return total + (mean_diff * mean_diff / N);
      });

  return {last_elapsed, mean, std::sqrt(variance), durations.size()};
}

std::optional<double> ElapsedTimeRecorder::getLastElapsed(
    const std::string& name) const {
  if (!elapsed_.count(name)) {
    return std::nullopt;
  }

  std::chrono::duration<double> elapsed_s = elapsed_.at(name).back();
  return elapsed_s.count();
}

ScopedTimer::ScopedTimer(const std::string& name,
                         bool verbose,
                         int verbosity,
                         bool elapsed_only)
    : name_(name),
      verbose_(verbose),
      verbosity_(verbosity),
      elapsed_only_(elapsed_only) {
  ElapsedTimeRecorder::instance().start(name_);
}

ScopedTimer::ScopedTimer(const std::string& name) : ScopedTimer(name, false, 1, true) {}

ScopedTimer::~ScopedTimer() {
  ElapsedTimeRecorder::instance().stop(name_);
  if (!verbose_) {
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

}  // namespace kimera
