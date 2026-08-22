#pragma once
#include "hydra/backend/update_functions.h"
#include "hydra/openset/clustering/agglomerative_clustering.h"
#include "hydra/utils/logging.h"

namespace hydra {

struct IBRegionsUpdateFunctor : public UpdateFunctor {
  using Clusters = std::vector<AgglomerativeClustering::Cluster::Ptr>;
  struct Config : VerbosityConfig {
    char id_prefix = 'r';
    std::string source_layer = spark_dsg::DsgLayers::PLACES;
    std::string target_layer = spark_dsg::DsgLayers::ROOMS;
    AgglomerativeClustering::Config clustering;

    Config();
  } const config;

  explicit IBRegionsUpdateFunctor(const Config& config);

  void call(const spark_dsg::SceneGraph& unmmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

 private:
  AgglomerativeClustering clustering_;
};

void declare_config(IBRegionsUpdateFunctor::Config& config);

}  // namespace hydra
