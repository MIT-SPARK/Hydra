#pragma once
#include <hydra_multi/backend/module.h>
#include <ianvs/node_handle.h>

#include <mutex>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>

namespace hydra_multi {

class StatusMonitor {
 public:
  struct Config {
    std::string nickname = "hydra_multi";
    double report_period_s = 0.5;
    double max_time_between_spins_s = 10.0;
  } const config;

  StatusMonitor(const Config& config, ianvs::NodeHandle nh);

  void start();

  void recordSpin(const MultiBackendModuleStatus& status);

  const std::string node_name;

 private:
  void publish();

  std::mutex mutex_;
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<MultiBackendModuleStatus> last_status_;
  std::optional<std::chrono::nanoseconds> last_stamp_;
};

void declare_config(StatusMonitor::Config& config);

}  // namespace hydra_multi
