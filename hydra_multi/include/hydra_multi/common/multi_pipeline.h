#pragma once
#include <config_utilities/virtual_config.h>

#include <map>

#include "hydra_multi/backend/module.h"
#include "hydra_multi/common/multi_global_info.h"
#include "hydra_multi/common/types.h"
#include "hydra_multi/interface/interface_container.h"

namespace hydra_multi {

class MultiPipeline {
 public:
  MultiPipeline(const MultiPipelineConfig& config, int config_verbosity = 1);

  virtual ~MultiPipeline();

  virtual void init();

  virtual void start();

  virtual void stop();

  virtual void save(const DataDirectory& output);

 protected:
  int config_verbosity_;

  InterfaceContainer::Ptr interfaces_;
  std::shared_ptr<MultiBackendModule> backend_;
};

}  // namespace hydra_multi
