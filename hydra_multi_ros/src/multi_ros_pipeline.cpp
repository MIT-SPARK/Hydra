#include "hydra_multi_ros/multi_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra_multi/backend/module.h>
#include <hydra_multi/common/types.h>

#include <memory>

#include "hydra_multi_ros/common.h"
#include "hydra_multi_ros/multi_backend_publisher.h"

namespace hydra_multi {

using pose_graph_tools::PoseGraphTypeAdapter;

void declare_config(MultiRosPipeline::Config& config) {
  using namespace config;
  name("MultiRosPipeline::Config");
  field(config.status_monitor, "status_monitor");
}

MultiRosPipeline::MultiRosPipeline(int config_verbosity)
    : MultiPipeline(config::fromContext<MultiPipelineConfig>(), config_verbosity),
      config(config::checkValid(config::fromContext<Config>())) {
  LOG(INFO) << "Starting Hydra-Multi ROS";

  auto nh = getHydraMultiNodeHandle("~");
  status_monitor_ = std::make_unique<StatusMonitor>(config.status_monitor, nh);

  auto bnh = nh / "backend";
  backend_->addSink(std::make_shared<MultiRosBackendPublisher>(bnh));
  backend_->addSink(MultiBackendModule::Sink::fromCallback(
      [this](
          uint64_t, const auto&, const auto&, const MultiBackendModuleStatus& status) {
        status_monitor_->recordSpin(status);
      }));
  loop_closure_sub_ = nh.create_subscription<PoseGraphTypeAdapter>(
      "external_loop_closures", 100, &MultiRosPipeline::loopClosureCallback, this);
}

void MultiRosPipeline::start() {
  MultiPipeline::start();
  status_monitor_->start();
}

MultiRosPipeline::~MultiRosPipeline() {}

void MultiRosPipeline::loopClosureCallback(
    const pose_graph_tools::PoseGraph& loop_closures) {
  LOG(INFO) << "Got " << loop_closures.edges.size() << " new loop closures";
  backend_->addLoopClosures(loop_closures);
}

}  // namespace hydra_multi
