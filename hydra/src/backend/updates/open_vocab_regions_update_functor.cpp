#include "hydra/backend/updates/open_vocab_regions_update_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra/rooms/room_utilities.h"
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

static const auto functor_reg =
    config::RegistrationWithConfig<UpdateFunctor,
                                   OpenVocabRegionsUpdateFunctor,
                                   OpenVocabRegionsUpdateFunctor::Config>(
        "OpenVocabRegionsUpdateFunctor");

void clearRegions(SceneGraph& graph, const std::string& layer) {
  std::vector<NodeId> prev_regions;
  for (const auto& [node_id, node] : graph.getLayer(layer).nodes()) {
    prev_regions.push_back(node_id);
  }

  for (const auto node : prev_regions) {
    graph.removeNode(node);
  }
}

}  // namespace

using timing::ScopedTimer;
using namespace spark_dsg;

void declare_config(OpenVocabRegionsUpdateFunctor::Config& config) {
  using namespace config;
  name("RegionUpdateFunctorConfig::Config");
  base<VerbosityConfig>(config);
  field<CharConversion>(config.id_prefix, "id_prefix");
  field(config.source_layer, "source_layer");
  field(config.target_layer, "target_layer");
  field(config.min_num_nodes, "min_num_nodes");
  field(config.clustering, "clustering");
}

OpenVocabRegionsUpdateFunctor::Config::Config() : VerbosityConfig("[IB Regions] ") {}

OpenVocabRegionsUpdateFunctor::OpenVocabRegionsUpdateFunctor(const Config& config)
    : config(config::checkValid(config)), clustering_(config.clustering.create()) {}

void OpenVocabRegionsUpdateFunctor::call(const SceneGraph&,
                                         SharedDsgInfo& dsg,
                                         const UpdateInfo::ConstPtr& info) const {
  ScopedTimer timer("backend/region_clustering", info->timestamp_ns);

  auto& graph = *dsg.graph;
  const auto& places = graph.getLayer(config.source_layer);
  clearRegions(graph, config.target_layer);

  const auto clusters = clustering_->cluster(places);
  MLOG(1) << "Got " << clusters.size() << " cluster(s)";

  std::set<NodeId> new_nodes;
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i]->nodes.size() < config.min_num_nodes) {
      MLOG(3) << "Dropping cluster of " << clusters[i]->nodes.size() << " node(s)";
      continue;
    }

    NodeSymbol new_node_id(config.id_prefix, i);
    auto attrs = std::make_unique<SemanticNodeAttributes>();
    attrs->semantic_label = 0;
    attrs->name = clusters[i]->best_query_name;
    attrs->semantic_feature = clusters[i]->feature;
    attrs->semantic_label = clusters[i]->best_query;
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
