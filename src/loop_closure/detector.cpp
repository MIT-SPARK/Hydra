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
#include "hydra/loop_closure/detector.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <fstream>

#include "hydra/utils/display_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra::lcd {

using DsgNode = DynamicSceneGraphNode;
using hydra::timing::ScopedTimer;
using SearchConfigMap = std::map<LayerId, DescriptorMatchConfig>;
using RegConfigMap = std::map<LayerId, LayerRegistrationConfig>;
using MatchMap = std::map<size_t, LayerSearchResults>;
using SolutionList = std::vector<RegistrationSolution>;

void declare_config(LayerLcdConfig& conf) {
  using namespace config;
  name("LayerLcdConfig");
  field(conf.layer, "layer");
  field(conf.descriptors, "descriptors");
  field(conf.matching, "matching");
  // registration not explicitly required
  conf.registration.setOptional();
  field(conf.registration, "registration");
}

void declare_config(SensorLcdConfig& conf) {
  using namespace config;
  name("SensorLcdConfig");
  field(conf.descriptors, "descriptors");
  conf.descriptors.setOptional();
  field(conf.matching, "matching");
  // registration not explicitly required
  conf.registration.setOptional();
  field(conf.registration, "registration");
}

void declare_config(LcdDetectorConfig& conf) {
  using namespace config;
  name("LcdDetectorConfig");
  field(conf.agent_config, "agent");
  field(conf.graph_configs, "graph_configs");
}

LcdDetector::LcdDetector(const LcdDetectorConfig& config)
    : config(config::checkValid(config)) {
  agent_factory_ = config.agent_config.descriptors.create();
  agent_registration_ = config.agent_config.registration.create();

  size_t internal_idx = 1;  // agent is 0
  for (const auto& layer_config : config.graph_configs) {
    auto factory = layer_config.descriptors.create();
    VLOG(VLEVEL_INFO) << "[Hydra LCD] Initializing factory for layer "
                      << layer_config.layer << " at index " << internal_idx;

    if (!factory) {
      LOG(WARNING) << "Invalid descriptor factory for layer " << layer_config.layer;
      continue;
    }

    layer_factories_.emplace(layer_config.layer, std::move(factory));
    internal_index_to_layer_[internal_idx] = layer_config.layer;
    match_config_map_[internal_idx] = layer_config.matching;

    auto solver = layer_config.registration.create(layer_config.layer);
    if (solver) {
      registration_solvers_.emplace(internal_idx, std::move(solver));
    }

    ++internal_idx;
  }

  // TODO(nathan) make root layer requirements explicit
  max_internal_index_ = internal_idx;
  if (internal_idx == 1) {
    VLOG(VLEVEL_INFO) << "[Hydra LCD] Only using sensor-level LCD";
  } else {
    root_layer_ = internal_index_to_layer_.at(internal_idx - 1);
  }

  cache_map_.clear();
  for (const auto& id_func_pair : layer_factories_) {
    cache_map_[id_func_pair.first] = DescriptorCache();
  }
}

size_t LcdDetector::numDescriptors() const {
  size_t num_descriptors = 0;
  for (const auto& id_cache_pair : cache_map_) {
    num_descriptors += id_cache_pair.second.size();
  }

  return num_descriptors + numAgentDescriptors();
}

size_t LcdDetector::numGraphDescriptors(LayerId layer) const {
  if (!cache_map_.count(layer)) {
    return 0;
  }

  return cache_map_.at(layer).size();
}

size_t LcdDetector::numAgentDescriptors() const {
  size_t count = 0;
  for (const auto& id_cache_pair : leaf_cache_) {
    count += id_cache_pair.second.size();
  }

  return count;
}

const std::map<size_t, LayerSearchResults>& LcdDetector::getLatestMatches() const {
  return matches_;
}

const DescriptorCache& LcdDetector::getDescriptorCache(LayerId layer) {
  return cache_map_.at(layer);
}

void LcdDetector::dumpDescriptors(const std::string& log_path) const {
  const Eigen::IOFormat format(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  std::ofstream fout(log_path + "/descriptors.yaml");
  fout << "---" << std::endl;
  fout << "layers:";
  if (cache_map_.empty()) {
    fout << " []" << std::endl;
    return;
  }

  fout << std::endl;
  for (const auto& id_cache_pair : cache_map_) {
    fout << "  - layer_id: " << id_cache_pair.first << std::endl;
    if (id_cache_pair.second.empty()) {
      fout << "  layer_descriptors: []" << std::endl;
      continue;
    }

    fout << "    layer_descriptors:" << std::endl;
    for (const auto& root_descriptor_pair : id_cache_pair.second) {
      if (!root_descriptor_pair.second) {
        continue;
      }

      if (root_descriptor_pair.second->is_null) {
        continue;
      }

      fout << "      - {root: " << root_descriptor_pair.first
           << ", values: " << root_descriptor_pair.second->values.format(format) << "}"
           << std::endl;
    }
  }
}

void LcdDetector::addSensorDescriptor(const Sensor& sensor,
                                      const DynamicSceneGraph& graph,
                                      const NodeId agent_id,
                                      const FrameData& data) {
  if (!agent_factory_) {
    return;
  }

  const DynamicSceneGraphNode& node = graph.getDynamicNode(agent_id).value();
  auto result = agent_factory_->construct(sensor, node, data);
  if (!result.descriptor) {
    return;
  }

  queued_agent_descriptors_[node.id] = std::move(result.descriptor);
  if (result.features) {
    agent_features_[node.id] = result.features;
  }
}

void LcdDetector::updateDescriptorCache(const DynamicSceneGraph& graph,
                                        const std::unordered_set<NodeId>& places,
                                        uint64_t timestamp) {
  ScopedTimer timer("lcd/update_descriptors", timestamp, true, 2, false);
  for (const auto& place_id : places) {
    if (!graph.hasNode(place_id)) {
      continue;  // ideally this doesn't happen in practice, but worth building in the
                 // sanity check for now
    }

    const SceneGraphNode& node = graph.getNode(place_id).value();

    std::vector<NodeId> new_agent_nodes;
    for (const auto& child : node.children()) {
      if (graph.isDynamic(child)) {
        new_agent_nodes.push_back(child);
      }
    }

    if (new_agent_nodes.empty()) {
      VLOG(VLEVEL_FILE) << "[LCD] Discarding " << printNodeId(node.id)
                        << " with no agent children";
      continue;
    }

    const DsgNode& first_node = graph.getDynamicNode(new_agent_nodes.front()).value();
    for (auto&& [prefix, factory] : layer_factories_) {
      auto& curr_cache = cache_map_[prefix];
      if (curr_cache.count(place_id)) {
        VLOG(VLEVEL_FILE) << "[LCD] skipping " << printNodeId(node.id)
                          << " descriptor creation for index " << prefix;
        continue;
      }

      // guaranteed to exist by constructor
      auto descriptor = factory->construct(graph, first_node);
      curr_cache[place_id] = std::move(descriptor);
    }

    auto cache_iter = leaf_cache_.find(place_id);
    if (cache_iter == leaf_cache_.end()) {
      cache_iter = leaf_cache_.emplace(place_id, DescriptorCache()).first;
    }

    auto leaf_iter = root_leaf_map_.find(place_id);
    if (leaf_iter == root_leaf_map_.end()) {
      leaf_iter = root_leaf_map_.emplace(place_id, std::set<NodeId>()).first;
    }

    for (const auto agent_id : new_agent_nodes) {
      auto iter = queued_agent_descriptors_.find(agent_id);
      if (iter == queued_agent_descriptors_.end()) {
        continue;
      }

      auto desc = std::move(iter->second);
      queued_agent_descriptors_.erase(iter);
      desc->root_node = place_id;
      leaf_iter->second.insert(agent_id);
      cache_iter->second[agent_id] = std::move(desc);
    }
  }
}

inline NodeId getFirstNode(const std::set<NodeId>& nodes) {
  CHECK(!nodes.empty());
  return *nodes.begin();
}

RegistrationSolution LcdDetector::registerAgent(const DynamicSceneGraph& graph,
                                                const LayerSearchResults& match,
                                                NodeId agent_id,
                                                uint64_t timestamp) const {
  if (!agent_registration_) {
    return {};
  }

  ScopedTimer timer("lcd/register_agent", timestamp, true, 2, false);

  for (size_t i = 0; i < match.match_root.size(); ++i) {
    const auto root = match.match_root.at(i);
    CHECK(graph.hasNode(root)) << "Invalid match: " << printNodeId(root);
    CHECK(graph.hasNode(match.query_root))
        << "Invalid query: " << printNodeId(match.query_root);

    SensorRegistrationInput input;
    input.query = agent_id;
    input.match = getFirstNode(match.match_nodes[i]);
    auto qiter = agent_features_.find(input.query);
    if (qiter != agent_features_.end()) {
      input.query_features = qiter->second;
    }

    auto miter = agent_features_.find(input.match);
    if (miter != agent_features_.end()) {
      input.match_features = miter->second;
    }

    auto result = agent_registration_->solve(graph, input);
    if (result.valid) {
      return result;
    }
  }

  return {};
}

SolutionList LcdDetector::registerAndVerify(const DynamicSceneGraph& graph,
                                            const MatchMap& matches,
                                            NodeId agent_id,
                                            uint64_t timestamp) const {
  ScopedTimer timer("lcd/register", timestamp, true, 2, false);

  const auto iter = matches.find(0);
  if (iter != matches.end()) {
    auto result = registerAgent(graph, iter->second, agent_id, timestamp);
    if (result.valid) {
      result.level = 0;
      return {result};
    }
  }

  size_t idx;
  for (idx = 1; idx < max_internal_index_; ++idx) {
    if (!registration_solvers_.count(idx)) {
      continue;
    }

    if (!matches.count(idx)) {
      continue;
    }

    CHECK(match_config_map_.count(idx));
    const auto& match = matches.at(idx);
    if (match.valid_matches.empty()) {
      continue;
    }

    for (size_t i = 0; i < match.match_root.size(); i++) {
      if (match.score[i] < match_config_map_.at(idx).min_registration_score) {
        break;
      }

      const auto root = match.match_root.at(i);
      CHECK(graph.hasNode(root)) << "Invalid match: " << printNodeId(root);
      CHECK(graph.hasNode(match.query_root))
          << "Invalid query: " << printNodeId(match.query_root);

      GraphRegistrationInput registration_input = {match.query_nodes,
                                                   match.match_nodes[i],
                                                   match.query_root,
                                                   match.match_root[i]};
      auto result =
          registration_solvers_.at(idx)->solve(graph, registration_input, agent_id);
      result.level = static_cast<int64_t>(idx);
      if (result.valid) {
        return {result};
      }
    }
  }

  // TODO(nathan) fix validation
  return {};
}

Descriptor* LcdDetector::getAgentDescriptor(const DsgNode& node) {
  auto iter = queued_agent_descriptors_.find(node.id);
  if (iter != queued_agent_descriptors_.end()) {
    return iter->second.get();
  }

  auto parent = node.getParent();
  if (!parent) {
    return nullptr;
  }

  const auto piter = leaf_cache_.find(*parent);
  if (piter == leaf_cache_.end()) {
    return nullptr;
  }

  const auto diter = piter->second.find(node.id);
  if (diter == piter->second.end()) {
    return nullptr;
  } else {
    return diter->second.get();
  }
}

std::vector<RegistrationSolution> LcdDetector::detect(const DynamicSceneGraph& graph,
                                                      NodeId agent_id,
                                                      uint64_t timestamp) {
  ScopedTimer timer("lcd/detect", timestamp, true, 2, false);
  std::set<NodeId> prev_valid_roots;
  if (max_internal_index_ > 1) {
    // fill query with all avaible roots if we have a hierarchical pipeline
    for (const auto& id_desc_pair : cache_map_[root_layer_]) {
      prev_valid_roots.insert(id_desc_pair.first);
    }
  } else {
    for (const auto& id_set_pair : root_leaf_map_) {
      prev_valid_roots.insert(id_set_pair.first);
    }
  }

  const DsgNode& latest_node = graph.getDynamicNode(agent_id).value();
  VLOG(VLEVEL_INFO) << "************************************************************";
  VLOG(VLEVEL_INFO) << "LCD Matching: " << printNodeId(latest_node.id);

  matches_.clear();
  for (size_t idx = max_internal_index_ - 1; idx > 0; --idx) {
    const auto layer = internal_index_to_layer_.at(idx);
    auto descriptor = layer_factories_[layer]->construct(graph, latest_node);
    if (descriptor) {
      VLOG(VLEVEL_INFO) << "level " << idx << ": "
                        << showVector(descriptor->values, 3, 20, 9);
      matches_[idx] = searchDescriptors(*descriptor,
                                        match_config_map_.at(idx),
                                        prev_valid_roots,
                                        cache_map_[layer],
                                        root_leaf_map_,
                                        agent_id);
      prev_valid_roots = matches_[idx].valid_matches;
    } else {
      VLOG(VLEVEL_INFO) << "level " << idx << " -> ?";
      prev_valid_roots = std::set<NodeId>();
      break;
    }
  }

  auto descriptor = getAgentDescriptor(latest_node);
  if (descriptor) {
    const auto parent = latest_node.getParent();
    CHECK(parent);
    descriptor->root_node = parent.value();
    matches_[0] = searchLeafDescriptors(*descriptor,
                                        config.agent_config.matching,
                                        prev_valid_roots,
                                        leaf_cache_,
                                        agent_id);
  }

  if (matches_.empty()) {
    VLOG(VLEVEL_INFO) << "No LCD matches for node " << printNodeId(agent_id)
                      << " against " << numDescriptors() << " descriptors";
    return {};
  }

  VLOG(VLEVEL_INFO) << "-----------------------------------------------------------";
  VLOG(VLEVEL_INFO) << "LCD results for node " << printNodeId(agent_id) << " against "
                    << numDescriptors() / 2 << " roots";
  for (const auto& id_match_pair : matches_) {
    const auto& scores = id_match_pair.second.score;
    const auto best_score = std::max_element(scores.begin(), scores.end());
    VLOG(VLEVEL_INFO) << " - index " << id_match_pair.first << ": "
                      << id_match_pair.second.valid_matches.size() << " valid, "
                      << id_match_pair.second.match_root.size()
                      << " registration (best score "
                      << (best_score != scores.end() ? std::to_string(*best_score)
                                                     : "n/a")
                      << ")";

    for (size_t i = 0; i < id_match_pair.second.match_root.size(); ++i) {
      VLOG(VLEVEL_INFO) << "   - #" << i
                        << ": query=" << printNodeId(id_match_pair.second.query_root)
                        << ", match=" << printNodeId(id_match_pair.second.match_root[i])
                        << " -> " << id_match_pair.second.score[i];
    }
  }

  VLOG(VLEVEL_INFO) << "-----------------------------------------------------------";
  const auto results = registerAndVerify(graph, matches_, agent_id, timestamp);
  VLOG(VLEVEL_INFO) << "************************************************************";

  return results;
}

}  // namespace hydra::lcd
