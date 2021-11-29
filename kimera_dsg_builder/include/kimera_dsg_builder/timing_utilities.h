#pragma once
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace kimera {

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

  void reset();

  std::optional<double> getLastElapsed(const std::string& timer_name) const;

  ElapsedStatistics getStats(const std::string& timer_name) const;

  void logElapsed(const std::string& name,
                  const std::string& output_folder) const;

  void logAllElapsed(const std::string& output_folder) const;

  void logStats(const std::string& output_folder) const;

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
  std::unique_ptr<std::mutex> mutex_;
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

 private:
  std::string name_;
  bool verbose_;
  int verbosity_;
  bool elapsed_only_;
  bool verbosity_disables_;
};

}  // namespace kimera
