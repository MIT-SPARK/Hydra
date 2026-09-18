#pragma once
#include "hydra/backend/update_functions.h"
#include "hydra/openset/layer_clustering.h"
#include "hydra/utils/logging.h"

namespace hydra {

struct OpenVocabRegionsUpdateFunctor : public UpdateFunctor {
  struct Config : VerbosityConfig {
    char id_prefix = 'r';
    std::string source_layer = spark_dsg::DsgLayers::PLACES;
    std::string target_layer = spark_dsg::DsgLayers::ROOMS;
    size_t min_num_nodes = 5;
    config::VirtualConfig<LayerClustering> clustering;

    Config();
  } const config;

  explicit OpenVocabRegionsUpdateFunctor(const Config& config);

  void call(const spark_dsg::SceneGraph& unmmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

 private:
  std::unique_ptr<LayerClustering> clustering_;
};

void declare_config(OpenVocabRegionsUpdateFunctor::Config& config);

}  // namespace hydra
