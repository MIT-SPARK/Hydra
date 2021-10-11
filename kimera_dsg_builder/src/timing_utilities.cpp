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

ElapsedTimeRecorder::ElapsedTimeRecorder() { mutex_.reset(new std::mutex()); }

void ElapsedTimeRecorder::start(const std::string& timer_name) {
  bool have_start_already = false;
  {  // start critical section
    std::unique_lock<std::mutex> lock(*mutex_);
    if (starts_.count(timer_name)) {
      have_start_already = true;
    } else {
      starts_[timer_name] = std::chrono::high_resolution_clock::now();
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
      }

      elapsed_[timer_name].push_back(stop_point - starts_.at(timer_name));
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
      return {0.0, 0.0, 0.0, 0};
    }

    durations = elapsed_.at(name);
  }  // end critical section

  const double last_elapsed = *getLastElapsed(name);

  const size_t N = durations.size();
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

ScopedTimer::ScopedTimer(const std::string& name,
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
  ElapsedTimeRecorder::instance().start(name_);
}

ScopedTimer::ScopedTimer(const std::string& name)
    : ScopedTimer(name, false, 1, true, false) {}

ScopedTimer::~ScopedTimer() {
  if (verbosity_disables_ and !VLOG_IS_ON(verbosity_)) {
    return;
  }

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
