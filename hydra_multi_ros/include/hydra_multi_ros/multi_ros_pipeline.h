#pragma once
#include <hydra_multi/common/multi_pipeline.h>
#include <pose_graph_tools_ros/conversions.h>

#include <rclcpp/subscription.hpp>
#include <string>
#include <vector>

#include "hydra_multi_ros/interface/ros_unit_interface.h"
#include "hydra_multi_ros/status_monitor.h"

namespace hydra_multi {

class MultiRosPipeline : public MultiPipeline {
 public:
  struct Config {
    StatusMonitor::Config status_monitor;
  } const config;

  explicit MultiRosPipeline(int config_verbosity = 1);

  virtual ~MultiRosPipeline();

  void start() override;

 protected:
  void loopClosureCallback(const pose_graph_tools::PoseGraph& external_lcs);

 protected:
  std::unique_ptr<StatusMonitor> status_monitor_;
  pose_graph_tools::PoseGraphSubscription loop_closure_sub_;
};

void declare_config(MultiRosPipeline::Config& config);

}  // namespace hydra_multi
