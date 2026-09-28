#pragma once
#include <config_utilities/virtual_config.h>
#include <hydra/common/shared_dsg_info.h>

#include <array>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hydra_multi/backend/module.h"
#include "hydra_multi/interface/unit_interface.h"

namespace hydra_multi {

struct MultiPipelineConfig {
  bool timing_disabled = false;
  bool disable_timer_output = true;
  bool enable_pgmo_logging = true;

  // Default settings for other modules. Can be overwritten by other module configs.
  int default_verbosity = 1;
  int default_num_threads = -1;  // -1 means use all available threads.

  std::string world_frame;
  MultiDsgInfo::Config graph;

  std::vector<config::VirtualConfig<UnitInterface>> robots;
  config::VirtualConfig<MultiBackendModule> backend{MultiBackendModule::Config()};
};

void declare_config(MultiPipelineConfig& config);

class MultiGlobalInfo {
 public:
  static MultiGlobalInfo& instance();

  static void init(const MultiPipelineConfig& config);

  // this invalidates any instances (mostly intended for testing)
  static void reset();

  void setForceShutdown(bool force_shutdown);

  bool force_shutdown() const;

  const MultiPipelineConfig& getConfig() const;

  inline std::string getWorldFrame() const { return config_.world_frame; }

  hydra::SharedDsgInfo::Ptr createSharedDsg() const;

  MultiDsgInfo::Ptr createMultiDsg() const;

 private:
  MultiGlobalInfo();

  void configureTimers();

  void initFromConfig(const MultiPipelineConfig& config);

 private:
  static std::unique_ptr<MultiGlobalInfo> instance_;
  MultiPipelineConfig config_;
  std::atomic<bool> force_shutdown_;
};

std::ostream& operator<<(std::ostream& out, const MultiGlobalInfo& config);

}  // namespace hydra_multi
