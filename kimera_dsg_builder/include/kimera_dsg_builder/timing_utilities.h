#pragma once
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace kimera {

struct ElapsedStatistics {
  double last_s;
  double mean_s;
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

  void start(const std::string& timer_name);

  void stop(const std::string& timer_name);

  void reset();

  ElapsedStatistics getStats(const std::string& timer_name) const;

  std::optional<double> getLastElapsed(const std::string& timer_name) const;

 private:
  using TimeList = std::list<std::chrono::nanoseconds>;
  using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
  using TimeMap = std::map<std::string, TimePoint>;

  ElapsedTimeRecorder();

  static std::unique_ptr<ElapsedTimeRecorder> instance_;

  TimeMap starts_;
  std::map<std::string, TimeList> elapsed_;
};

class ScopedTimer {
 public:
  explicit ScopedTimer(const std::string& name);

  ScopedTimer(const std::string& name,
              bool verbose,
              int verbosity,
              bool elapsed_only = true);

  ~ScopedTimer();

 private:
  std::string name_;
  bool verbose_;
  int verbosity_;
  bool elapsed_only_;
};

}  // namespace kimera
