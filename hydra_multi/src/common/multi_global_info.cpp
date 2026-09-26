#include "hydra_multi/common/multi_global_info.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <filesystem>
#include <fstream>

#include "hydra/utils/pgmo_glog_sink.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra_multi {

using hydra::PgmoGlogSink;
using hydra::timing::ElapsedTimeRecorder;

decltype(MultiGlobalInfo::instance_) MultiGlobalInfo::instance_;

void declare_config(MultiPipelineConfig& config) {
  using namespace config;
  name("MultiPipeline::Config");
  field(config.timing_disabled, "timing_disabled");
  field(config.disable_timer_output, "disable_timer_output");
  field(config.enable_pgmo_logging, "enable_pgmo_logging");
  field(config.default_verbosity, "default_verbosity");
  field(config.default_num_threads, "default_num_threads");
  field(config.world_frame, "world_frame");
  field(config.backend, "backend");
  field(config.robots, "robots");
}

MultiGlobalInfo::MultiGlobalInfo() : force_shutdown_(false) {}

void MultiGlobalInfo::configureTimers() {
  ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
  timer.timing_disabled = config_.timing_disabled;
  timer.disable_output = config_.disable_timer_output;
}

void MultiGlobalInfo::initFromConfig(const MultiPipelineConfig& config) {
  config_ = config::checkValid(config);
  VLOG(5) << "Loading from config: " << config::toString(config);
  configureTimers();

  if (config_.enable_pgmo_logging) {
    logging::Logger::addSink("glog", std::make_shared<PgmoGlogSink>());
  }
}

MultiGlobalInfo& MultiGlobalInfo::instance() {
  if (!instance_) {
    instance_.reset(new MultiGlobalInfo());
  }
  return *instance_;
}

void MultiGlobalInfo::init(const MultiPipelineConfig& config) {
  auto& curr = instance();
  curr.initFromConfig(config);
}

void MultiGlobalInfo::reset() { instance_.reset(new MultiGlobalInfo()); }

void MultiGlobalInfo::setForceShutdown(bool force_shutdown) {
  force_shutdown_ = force_shutdown;
}

bool MultiGlobalInfo::force_shutdown() const { return force_shutdown_; }

const MultiPipelineConfig& MultiGlobalInfo::getConfig() const { return config_; }

hydra::SharedDsgInfo::Ptr MultiGlobalInfo::createSharedDsg() const {
  return std::make_shared<hydra::SharedDsgInfo>(config_.graph);
}

MultiDsgInfo::Ptr MultiGlobalInfo::createMultiDsg() const {
  return std::make_shared<MultiDsgInfo>(config_.graph);
}

std::ostream& operator<<(std::ostream& out, const MultiGlobalInfo& config) {
  out << config::toString(config.getConfig());
  return out;
}

}  // namespace hydra_multi
