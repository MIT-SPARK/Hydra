#include "hydra/openset/layer_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>

namespace hydra {

void declare_config(LayerClustering::Config& config) {
  using namespace config;
  name("LayerClustering::Config");
  config.queries.setOptional();
  field(config.queries, "queries");
  config.metric.setOptional();
  field(config.metric, "metric");
}

LayerClustering::LayerClustering(const Config& config)
    : config(config::checkValid(config)),
      queries_(config.queries.create()),
      metric_(config.metric.create()) {}

}  // namespace hydra
