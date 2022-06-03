/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_dsg_builder/dsg_lcd_detector.h"

#include <hydra_utils/timing_utilities.h>

namespace hydra {
namespace lcd {

using DsgNode = DynamicSceneGraphNode;
using hydra::timing::ScopedTimer;

DsgLcdDetector::DsgLcdDetector(const DsgLcdDetectorConfig& config) : config_(config) {
  for (const auto& id_func_pair : layer_factories_) {
    cache_map_[id_func_pair.first] = DescriptorCache();
  }

  layer_factories_.emplace(DsgLayers::OBJECTS,
                           std::make_unique<ObjectDescriptorFactory>(
                               config_.object_radius_m, config_.num_semantic_classes));
  layer_factories_.emplace(DsgLayers::PLACES,
                           std::make_unique<PlaceDescriptorFactory>(
                               config_.place_radius_m, config_.place_histogram_config));
  agent_factory_ = std::make_unique<AgentDescriptorFactory>();

  size_t internal_idx = 1;  // agent is 0
  for (const auto& id_config_pair : config_.search_configs) {
    const LayerId layer = id_config_pair.first;

    if (!layer_factories_.count(layer)) {
      continue;
    }

    const auto& layer_config = id_config_pair.second;
    layer_to_internal_index_[layer] = internal_idx;
    internal_index_to_layer_[internal_idx] = layer;
    match_config_map_[internal_idx] = layer_config;

    auto iter = config_.registration_configs.find(layer);
    if (iter != config_.registration_configs.end()) {
      registration_solvers_.emplace(internal_idx,
                                    std::make_unique<DsgTeaserSolver>(
                                        layer, iter->second, config_.teaser_config));
    }

    // if (validation_funcs.count(layer_config.layer)) {
    // validation_funcs_[internal_idx] = validation_funcs.at(layer_config.layer);
    //}

    internal_idx++;
  }

  // TODO(nathan) make root layer requirements explicit
  max_internal_index_ = internal_idx;
  root_layer_ = internal_index_to_layer_.at(internal_idx - 1);

  match_config_map_[0] = config.agent_search_config;
  if (config_.enable_agent_registration) {
    registration_solvers_.emplace(0, std::make_unique<DsgAgentSolver>());
  }
}

bool DsgLcdDetector::addNewDescriptors(const DynamicSceneGraph& graph,
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

  leaf_cache_[*parent][agent_node.id] = agent_factory_->construct(graph, agent_node);

  for (const auto& prefix_func_pair : layer_factories_) {
    if (cache_map_[prefix_func_pair.first].count(*parent)) {
      continue;
    }

    // guaranteed to exist by constructor
    Descriptor::Ptr layer_descriptor =
        prefix_func_pair.second->construct(graph, agent_node);
    cache_map_[prefix_func_pair.first][*parent] = std::move(layer_descriptor);
  }

  return true;
}

void DsgLcdDetector::updateDescriptorCache(
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

std::vector<DsgRegistrationSolution> DsgLcdDetector::registerAndVerify(
    const DynamicSceneGraph& dsg,
    const std::map<size_t, LayerSearchResults>& matches,
    NodeId agent_id,
    uint64_t timestamp) const {
  ScopedTimer timer("lcd/register", timestamp, true, 2, false);

  DsgRegistrationSolution registration_result;

  size_t idx;
  for (idx = 0; idx < max_internal_index_; ++idx) {
    if (!registration_solvers_.count(idx)) {
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
          registration_solvers_.at(idx)->solve(dsg, registration_input, agent_id);
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

std::vector<DsgRegistrationSolution> DsgLcdDetector::detect(
    const DynamicSceneGraph& dsg,
    NodeId agent_id,
    uint64_t timestamp) {
  ScopedTimer timer("lcd/detect", timestamp, true, 2, false);
  std::set<NodeId> prev_valid_roots;
  for (const auto& id_desc_pair : cache_map_[root_layer_]) {
    prev_valid_roots.insert(id_desc_pair.first);
  }

  const DsgNode& latest_node = dsg.getDynamicNode(agent_id).value();
  VLOG(2) << "************************************************************";
  VLOG(2) << "LCD Matching: " << NodeSymbol(latest_node.id).getLabel();

  matches_.clear();
  for (size_t idx = max_internal_index_ - 1; idx > 0; --idx) {
    const auto& config = match_config_map_.at(idx);
    const LayerId layer = internal_index_to_layer_.at(idx);
    auto descriptor = layer_factories_[layer]->construct(dsg, latest_node);
    if (descriptor) {
      VLOG(2) << "level " << idx << ": " << std::endl
              << "    " << descriptor->values.transpose();
      matches_[idx] = searchDescriptors(*descriptor,
                                        config,
                                        prev_valid_roots,
                                        cache_map_[layer],
                                        root_leaf_map_,
                                        agent_id);
      prev_valid_roots = matches_[idx].valid_matches;
    } else {
      VLOG(2) << "level " << idx << " -> ?";
      prev_valid_roots = std::set<NodeId>();
      break;
    }
  }

  auto descriptor = agent_factory_->construct(dsg, latest_node);
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
}  // namespace hydra
