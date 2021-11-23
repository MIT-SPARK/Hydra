#include "kimera_dsg_builder/dsg_lcd_module.h"
#include "kimera_dsg_builder/dsg_lcd_registration.h"
#include "kimera_dsg_builder/timing_utilities.h"

namespace kimera {
namespace lcd {

using incremental::SharedDsgInfo;
using DsgNode = DynamicSceneGraphNode;

DsgLcdModule::DsgLcdModule(
    const DsgLcdConfig& config,
    const std::map<LayerId, DescriptorFactoryFunc>& layer_factories,
    const std::map<LayerId, RegistrationFunc>& registration_funcs,
    const std::map<LayerId, ValidationFunc>& validation_funcs)
    : config_(config), layer_factories_(layer_factories) {
  for (const auto& id_func_pair : layer_factories_) {
    cache_map_[id_func_pair.first] = DescriptorCache();
  }

  size_t internal_idx = 1;  // agent is 0
  for (const auto& layer_config : config_.search_configs) {
    if (!layer_factories_.count(layer_config.layer)) {
      continue;
    }

    layer_to_internal_index_[layer_config.layer] = internal_idx;
    match_config_map_[internal_idx] = layer_config;

    if (registration_funcs.count(layer_config.layer)) {
      registration_funcs_[internal_idx] = registration_funcs.at(layer_config.layer);
    }

    if (validation_funcs.count(layer_config.layer)) {
      validation_funcs_[internal_idx] = validation_funcs.at(layer_config.layer);
    }

    internal_idx++;
  }
  max_internal_index_ = internal_idx;

  match_config_map_[0] = config.agent_search_config;
  registration_funcs_[0] = &registerAgentMatch;
  validation_funcs_[0] = [](const DynamicSceneGraph&,
                            const LayerSearchResults&) -> bool {
    throw std::runtime_error("can not validate at agent layer!");
  };
}

bool DsgLcdModule::addNewDescriptors(const DynamicSceneGraph& graph,
                                     const DynamicSceneGraphNode& agent_node) {
  auto parent = agent_node.getParent();
  if (!parent) {
    return false;
  }

  if (!root_leaf_map_.count(*parent)) {
    root_leaf_map_[*parent] = std::set<NodeId>();
  }

  root_leaf_map_[*parent].insert(agent_node.id);

  if (!leaf_cache_.count(*parent)) {
    leaf_cache_[*parent] = DescriptorCache();
  }

  leaf_cache_[*parent][agent_node.id] = makeAgentDescriptor(graph, agent_node);

  for (const auto& prefix_func_pair : layer_factories_) {
    if (cache_map_[prefix_func_pair.first].count(*parent)) {
      continue;
    }

    // guaranteed to exist by constructor
    Descriptor::Ptr layer_descriptor = prefix_func_pair.second(graph, agent_node);
    cache_map_[prefix_func_pair.first][*parent] = std::move(layer_descriptor);
  }

  return true;
}

void DsgLcdModule::updateDescriptorCache(
    SharedDsgInfo& dsg,
    const std::unordered_set<NodeId>& archived_places) {
  ScopedTimer timer("lcd/update_descriptors", true, 2, false);
  std::unique_lock<std::mutex> lock(dsg.mutex);

  std::set<NodeId> new_agent_nodes;
  for (const auto& place_id : archived_places) {
    if (!dsg.graph->hasNode(place_id)) {
      continue;  // ideally this doesn't happen in practice, but worth building in the
                 // sanity check for now
    }

    const SceneGraphNode& node = dsg.graph->getNode(place_id).value();

    for (const auto& child : node.children()) {
      if (dsg.graph->isDynamic(child)) {
        new_agent_nodes.insert(child);
      }
    }
  }

  for (const auto& agent_node : new_agent_nodes) {
    const DynamicSceneGraphNode& node = dsg.graph->getDynamicNode(agent_node).value();
    addNewDescriptors(*dsg.graph, node);
  }
}

DsgRegistrationSolution DsgLcdModule::registerAndVerify(
    SharedDsgInfo& dsg,
    const std::map<size_t, LayerSearchResults>& matches,
    NodeId agent_id) const {
  ScopedTimer timer("lcd/register", true, 2, false);
  DsgRegistrationSolution registration_result;

  size_t idx;
  for (idx = 0; idx < max_internal_index_; ++idx) {
    if (!matches.count(idx)) {
      continue;
    }

    CHECK(match_config_map_.count(idx));
    const LayerSearchResults& match = matches.at(idx);
    if (match.best_score < match_config_map_.at(idx).min_score) {
      continue;
    }

    if (idx == 0) {
      NodeId query_node = *match.query_nodes.begin();
      NodeId match_node = *match.match_nodes.begin();
      VLOG(3) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
      VLOG(3) << "Found Match! " << NodeSymbol(query_node).getLabel() << " -> "
              << NodeSymbol(match_node).getLabel();
      VLOG(3) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    }

    if (!registration_funcs_.count(idx)) {
      continue;
    }

    registration_result = registration_funcs_.at(idx)(dsg, match, agent_id);
    if (registration_result.valid) {
      ++idx;  // start validation at next layer up
      break;
    }
  }

  if (!registration_result.valid) {
    return registration_result;
  }

  for (; idx < max_internal_index_; ++idx) {
    if (!validation_funcs_.count(idx)) {
      continue;
    }

    CHECK(matches.count(idx));
    const LayerSearchResults& match = matches.at(idx);

    if (!validation_funcs_.at(idx)(*dsg.graph, match)) {
      registration_result.valid = false;
      break;
    }
  }

  return registration_result;
}

DsgRegistrationSolution DsgLcdModule::detect(SharedDsgInfo& dsg, NodeId agent_id) {
  ScopedTimer timer("lcd/detect", true, 2, false);
  std::set<NodeId> prev_valid_roots;
  for (const auto& id_desc_pair : cache_map_[config_.search_configs.front().layer]) {
    prev_valid_roots.insert(id_desc_pair.first);
  }

  std::map<size_t, LayerSearchResults> matches;
  for (size_t idx = max_internal_index_ - 1; idx > 0; --idx) {
    const auto& config = match_config_map_.at(idx);

    Descriptor::Ptr descriptor;
    {  // start dsg critical section
      std::unique_lock<std::mutex> lock(dsg.mutex);
      const DsgNode& latest_node = dsg.graph->getDynamicNode(agent_id).value();
      descriptor = layer_factories_[config.layer](*dsg.graph, latest_node);
    }  // end dsg critical section

    if (descriptor) {
      matches[idx] = searchDescriptors(*descriptor,
                                       config,
                                       prev_valid_roots,
                                       cache_map_[config.layer],
                                       root_leaf_map_,
                                       agent_id);
      prev_valid_roots = matches[idx].valid_matches;
    } else {
      prev_valid_roots = std::set<NodeId>();
      break;
    }
  }

  Descriptor::Ptr descriptor;
  {  // start dsg critical section
    std::unique_lock<std::mutex> lock(dsg.mutex);
    const DsgNode& latest_node = dsg.graph->getDynamicNode(agent_id).value();
    descriptor = makeAgentDescriptor(*dsg.graph, latest_node);
  }  // end dsg critical section

  if (descriptor) {
    matches[0] = searchLeafDescriptors(*descriptor,
                                       config_.agent_search_config,
                                       prev_valid_roots,
                                       leaf_cache_,
                                       agent_id);
  }

  if (matches.empty()) {
    VLOG(1) << "No LCD matches for node " << NodeSymbol(agent_id).getLabel()
            << " against " << numDescriptors() << " descriptors";
    return {};
  }

  VLOG(2) << "===========================================================";
  VLOG(2) << "LCD matching results for node " << NodeSymbol(agent_id).getLabel()
          << " against " << numAgentDescriptors() << " / " << numDescriptors()
          << " (agent / all) descriptors";
  for (const auto& id_match_pair : matches) {
    VLOG(2) << " - index " << id_match_pair.first << ": "
            << NodeSymbol(id_match_pair.second.best_node).getLabel() << " -> "
            << id_match_pair.second.best_score << " with "
            << id_match_pair.second.valid_matches.size() << " valid matches";
  }
  VLOG(2) << "===========================================================";

  // TODO(nathan) maybe don't call directly
  return registerAndVerify(dsg, matches, agent_id);
}

}  // namespace lcd
}  // namespace kimera
