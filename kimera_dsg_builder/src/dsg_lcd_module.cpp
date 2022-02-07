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
    const DynamicSceneGraph& dsg,
    const std::unordered_set<NodeId>& archived_places,
    uint64_t timestamp) {
  ScopedTimer timer("lcd/update_descriptors", timestamp, true, 2, false);

  std::set<NodeId> new_agent_nodes;
  for (const auto& place_id : archived_places) {
    if (!dsg.hasNode(place_id)) {
      continue;  // ideally this doesn't happen in practice, but worth building in the
                 // sanity check for now
    }

    const SceneGraphNode& node = dsg.getNode(place_id).value();

    for (const auto& child : node.children()) {
      if (dsg.isDynamic(child)) {
        new_agent_nodes.insert(child);
      }
    }
  }

  for (const auto& agent_node : new_agent_nodes) {
    const DynamicSceneGraphNode& node = dsg.getDynamicNode(agent_node).value();
    addNewDescriptors(dsg, node);
  }
}

std::vector<DsgRegistrationSolution> DsgLcdModule::registerAndVerify(
    const DynamicSceneGraph& dsg,
    const std::map<size_t, LayerSearchResults>& matches,
    NodeId agent_id,
    uint64_t timestamp) const {
  ScopedTimer timer("lcd/register", timestamp, true, 2, false);

  DsgRegistrationSolution registration_result;

  size_t idx;
  for (idx = 0; idx < max_internal_index_; ++idx) {
    if (!registration_funcs_.count(idx)) {
      continue;
    }

    if (!matches.count(idx)) {
      continue;
    }

    CHECK(match_config_map_.count(idx));
    const LayerSearchResults& match = matches.at(idx);
    if (match.valid_matches.empty()) {
      continue;
    }

    for (size_t i = 0; i < match.match_root.size(); i++) {
      if (match.score[i] < match_config_map_.at(idx).min_registration_score) {
        break;
      }

      if (idx == 0) {
        NodeId query_node = *match.query_nodes.begin();
        NodeId match_node = *match.match_nodes[i].begin();
        VLOG(3) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        VLOG(3) << "Found Match! " << NodeSymbol(query_node).getLabel() << " -> "
                << NodeSymbol(match_node).getLabel();
        VLOG(3) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
      }

      DsgRegistrationInput registration_input = {match.query_nodes,
                                                 match.match_nodes[i],
                                                 match.query_root,
                                                 match.match_root[i]};
      registration_result =
          registration_funcs_.at(idx)(dsg, registration_input, agent_id);
      registration_result.level = static_cast<int64_t>(idx);
      if (registration_result.valid) {
        break;
      }
    }

    if (registration_result.valid) {
      break;
    }
  }

  // TODO(nathan) fix validation
  // size_t vidx = idx;
  /*++vidx;  // start validation at next layer up*/
  /*for (; vidx < max_internal_index_; ++vidx) {*/
  /*if (!validation_funcs_.count(vidx)) {*/
  /*continue;*/
  /*}*/

  /*CHECK(matches.count(idx));*/
  /*const LayerSearchResults& vmatch = matches.at(vidx);*/

  /*if (!validation_funcs_.at(vidx)(dsg, vmatch)) {*/
  /*registration_result.valid = false;*/
  /*break;*/
  /*}*/
  /*}*/

  std::vector<DsgRegistrationSolution> results;
  if (registration_result.valid) {
    results.push_back(registration_result);
  }

  return results;
}

std::vector<DsgRegistrationSolution> DsgLcdModule::detect(const DynamicSceneGraph& dsg,
                                                          NodeId agent_id,
                                                          uint64_t timestamp) {
  ScopedTimer timer("lcd/detect", timestamp, true, 2, false);
  std::set<NodeId> prev_valid_roots;
  for (const auto& id_desc_pair : cache_map_[config_.search_configs.front().layer]) {
    prev_valid_roots.insert(id_desc_pair.first);
  }

  const DsgNode& latest_node = dsg.getDynamicNode(agent_id).value();
  VLOG(2) << "************************************************************";
  VLOG(2) << "LCD Matching: " << NodeSymbol(latest_node.id).getLabel();

  matches_.clear();
  for (size_t idx = max_internal_index_ - 1; idx > 0; --idx) {
    const auto& config = match_config_map_.at(idx);
    Descriptor::Ptr descriptor;
    descriptor = layer_factories_[config.layer](dsg, latest_node);

    if (descriptor) {
      VLOG(2) << "level " << idx << ": " << std::endl
              << "    " << descriptor->values.transpose();
      matches_[idx] = searchDescriptors(*descriptor,
                                        config,
                                        prev_valid_roots,
                                        cache_map_[config.layer],
                                        root_leaf_map_,
                                        agent_id);
      prev_valid_roots = matches_[idx].valid_matches;
    } else {
      VLOG(2) << "level " << idx << " -> ?";
      prev_valid_roots = std::set<NodeId>();
      break;
    }
  }

  Descriptor::Ptr descriptor;
  descriptor = makeAgentDescriptor(dsg, latest_node);

  if (descriptor) {
    matches_[0] = searchLeafDescriptors(*descriptor,
                                        config_.agent_search_config,
                                        prev_valid_roots,
                                        leaf_cache_,
                                        agent_id);
  }

  if (matches_.empty()) {
    VLOG(1) << "No LCD matches for node " << NodeSymbol(agent_id).getLabel()
            << " against " << numDescriptors() << " descriptors";
    return {};
  }

  VLOG(2) << "===========================================================";
  VLOG(2) << "LCD results for node " << NodeSymbol(agent_id).getLabel() << " against "
          << numDescriptors() / 2 << " roots";
  for (const auto& id_match_pair : matches_) {
    VLOG(2) << " - index " << id_match_pair.first << ": "
            << id_match_pair.second.valid_matches.size() << " valid matches, "
            << id_match_pair.second.match_root.size() << " registration matches";
    for (size_t i = 0; i < id_match_pair.second.match_root.size(); ++i) {
      VLOG(2) << "   - #" << i
              << ": query=" << NodeSymbol(id_match_pair.second.query_root).getLabel()
              << ", match=" << NodeSymbol(id_match_pair.second.match_root[i]).getLabel()
              << " -> " << id_match_pair.second.score[i];
    }
  }
  VLOG(2) << "===========================================================";

  return registerAndVerify(dsg, matches_, agent_id, timestamp);
}

}  // namespace lcd
}  // namespace kimera
