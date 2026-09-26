#include "hydra_multi/common/multi_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/settings.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/config_utilities.h>
#include <hydra/utils/timing_utilities.h>

namespace hydra_multi {

MultiPipeline::MultiPipeline(const MultiPipelineConfig& pipeline_config,
                             int config_verbosity)
    : config_verbosity_(config_verbosity), interfaces_(new InterfaceContainer) {
  MultiGlobalInfo::init(pipeline_config);
  const auto& config = MultiGlobalInfo::instance().getConfig();
  for (const auto& unit_config : config.robots) {
    interfaces_->insert(unit_config.create());
  }

  backend_ = config.backend.create(interfaces_->states());
  LOG(INFO) << "[Hydra-Multi] Initialized pipeline with:\n" << config::toString(config);
}

MultiPipeline::~MultiPipeline() {}

void MultiPipeline::init() { interfaces_->init(); }

void MultiPipeline::start() {
  backend_->start();
  interfaces_->start();
}

void MultiPipeline::stop() {
  interfaces_->stop();
  backend_->stop();
}

void MultiPipeline::save(const DataDirectory& output) {
  if (!output) {
    return;
  }

  LOG(INFO) << "[Hydra-Multi] saving timing information to " << output.path();
  const auto& timer = hydra::timing::ElapsedTimeRecorder::instance();
  timer.logTimers(output.path("timing"));
  timer.logStats(output.path() / "timing_stats.csv");
  LOG(INFO) << "[Hydra-Multi] saved timing information";

  auto node = config::toYaml(MultiGlobalInfo::instance().getConfig());
  std::ofstream fout(output.path() / "hydra_multi_config.yaml");
  fout << node;

  interfaces_->save(output);
  backend_->save(output);
}

}  // namespace hydra_multi
