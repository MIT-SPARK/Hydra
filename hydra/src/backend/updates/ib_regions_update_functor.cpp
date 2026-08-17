#include "hydra/backend/updates/ib_regions_update_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include "hydra/rooms/room_utilities.h"
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

static const auto functor_reg =
    config::RegistrationWithConfig<UpdateFunctor,
                                   IBRegionsUpdateFunctor,
                                   IBRegionsUpdateFunctor::Config>("RegionsIBFunctor");

void clearRegions(SceneGraph& graph, const std::string& layer) {
  std::vector<NodeId> prev_regions;
  for (const auto& [node_id, node] : graph.getLayer(layer).nodes()) {
    prev_regions.push_back(node_id);
  }

  for (const auto node : prev_regions) {
    graph.removeNode(node);
  }
}

void fillFeatures(const SceneGraphLayer& places,
                  AgglomerativeClustering::NodeEmbeddingMap& valid_features) {
  for (const auto& [node_id, node] : places.nodes()) {
    const auto attrs = node->tryAttributes<SemanticNodeAttributes>();
    if (!attrs || attrs->semantic_feature.size() <= 1) {
      continue;
    }

    valid_features[node_id] = attrs->semantic_feature.rightCols<1>();
  }
}

}  // namespace

using timing::ScopedTimer;
using namespace spark_dsg;

void declare_config(IBRegionsUpdateFunctor::Config& config) {
  using namespace config;
  name("RegionUpdateFunctorConfig::Config");
  base<VerbosityConfig>(config);
  field<CharConversion>(config.id_prefix, "id_prefix");
  field(config.source_layer, "source_layer");
  field(config.target_layer, "target_layer");
  field(config.clustering, "clustering");
}

IBRegionsUpdateFunctor::Config::Config() : VerbosityConfig("[IB Regions] ") {}

IBRegionsUpdateFunctor::IBRegionsUpdateFunctor(const Config& config)
    : config(config::checkValid(config)), clustering_(config.clustering) {}

void IBRegionsUpdateFunctor::call(const DynamicSceneGraph&,
                                  SharedDsgInfo& dsg,
                                  const UpdateInfo::ConstPtr& info) const {
  ScopedTimer timer("backend/region_clustering", info->timestamp_ns);

  auto& graph = *dsg.graph;
  const auto& places = graph.getLayer(config.source_layer);
  clearRegions(graph, config.target_layer);

  AgglomerativeClustering::NodeEmbeddingMap valid_features;
  fillFeatures(places, valid_features);
  if (valid_features.empty()) {
    MLOG(1) << "Need to have at least one valid place feature";
    return;
  }

  const auto clusters = clustering_.cluster(places, valid_features);
  MLOG(1) << "Got " << clusters.size() << " cluster(s)";

  std::set<NodeId> new_nodes;
  for (size_t i = 0; i < clusters.size(); ++i) {
    NodeSymbol new_node_id(config.id_prefix, i);
    auto attrs = std::make_unique<SemanticNodeAttributes>();
    attrs->semantic_label = 0;
    attrs->name = clusters[i]->best_task_name;
    attrs->semantic_feature = clusters[i]->feature;
    attrs->semantic_label = clusters[i]->best_task_index;
    graph.emplaceNode(config.target_layer, new_node_id, std::move(attrs));

    for (const auto node_id : clusters[i]->nodes) {
      graph.insertEdge(new_node_id, node_id);
    }

    new_nodes.insert(new_node_id);
  }

  for (const auto& [node_id, node] : graph.getLayer(config.target_layer).nodes()) {
    const std::unordered_set<NodeId> to_use(node->children().begin(),
                                            node->children().end());
    node->attributes().position = getRoomPosition(places, to_use);
  }

  addEdgesToRoomLayer(graph, new_nodes);
}

}  // namespace hydra
