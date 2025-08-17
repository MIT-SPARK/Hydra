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
#include "hydra/frontend/graph_connector.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/printing.h>

#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra {
namespace {

inline bool isChild(LayerKey layer,
                    const std::set<LayerId>& layer_ids,
                    const std::set<LayerId>& partition_layer_ids) {
  return !layer.partition ? layer_ids.count(layer.layer)
                          : partition_layer_ids.count(layer.layer);
}

}  // namespace

void declare_config(LayerConnector::Config::ChildLayerConfig& config) {
  using namespace config;
  name("ChildLayerConfig");
  field(config.layer, "layer");
  field(config.include_primary, "include_primary");
  field(config.include_partitions, "include_partitions");
}

void declare_config(LayerConnector::Config& config) {
  using namespace config;
  name("LayerConnector::Config");
  field(config.parent_layer, "parent_layer");
  field(config.child_layers, "child_layers");
  field(config.verbosity, "verbosity");
  field(config.clear_active_flag, "clear_active_flag");
}

LayerConnector::LayerConnector(const Config& config) : config(config) {}

void LayerConnector::updateParents(const DynamicSceneGraph& graph,
                                   const std::vector<NodeId>& new_nodes) {
  const auto parent_key = graph.getLayerKey(config.parent_layer);
  if (!parent_key) {
    LOG(ERROR) << "Invalid parent layer and partition!";
    return;
  }

  for (const auto& new_id : new_nodes) {
    const auto node = graph.findNode(new_id);
    if (!node || node->layer != *parent_key) {
      continue;
    }

    active_parents[new_id] = std::set<spark_dsg::NodeId>();
  }

  auto iter = active_parents.begin();
  while (iter != active_parents.end()) {
    const auto parent_id = iter->first;
    auto node = graph.findNode(parent_id);
    if (!node) {
      iter = active_parents.erase(iter);
      continue;
    }

    if (node->attributes().is_active) {
      ++iter;
      continue;
    }

    for (const auto& child_id : iter->second) {
      active_children.erase(child_id);
      auto node = graph.findNode(child_id);
      if (config.clear_active_flag && node) {
        node->attributes().is_active = false;
      }
    }

    iter = active_parents.erase(iter);
  }
}

void LayerConnector::connectChildren(DynamicSceneGraph& graph,
                                     const std::vector<NodeId>& new_nodes) {
  std::set<LayerId> child_layer_ids;
  std::set<LayerId> partition_child_layer_ids;
  for (const auto& child_config : config.child_layers) {
    const auto key = graph.getLayerKey(child_config.layer);
    if (!key) {
      LOG(WARNING) << "Invalid layer '" << child_config.layer << "' for graph!";
      continue;
    }

    if (child_config.include_primary) {
      child_layer_ids.insert(key->layer);
    }

    if (child_config.include_partitions) {
      partition_child_layer_ids.insert(key->layer);
    }
  }

  for (const auto& new_id : new_nodes) {
    const auto node = graph.findNode(new_id);
    if (!node || !isChild(node->layer, child_layer_ids, partition_child_layer_ids)) {
      continue;
    }

    active_children.insert(new_id);
  }

  if (active_parents.empty()) {
    return;
  }

  LOG_IF(INFO, config.verbosity >= 5)
      << "Detecting interlayer edges for layer " << config.parent_layer << " with "
      << active_parents.size() << " parents and " << active_children.size()
      << " children";

  std::vector<spark_dsg::NodeId> parent_ids;
  std::transform(active_parents.begin(),
                 active_parents.end(),
                 std::back_inserter(parent_ids),
                 [](const auto& id_child_pair) { return id_child_pair.first; });

  const auto& parents = graph.getLayer(config.parent_layer);
  NearestNodeFinder nn_finder(parents, parent_ids);

  auto iter = active_children.begin();
  while (iter != active_children.end()) {
    auto node = graph.findNode(*iter);
    if (!node) {
      iter = active_children.erase(iter);
      continue;
    }
    ++iter;

    auto prev_parent = node->getParent();
    if (prev_parent) {
      active_parents.at(*prev_parent).erase(node->id);
    }

    nn_finder.find(
        node->attributes().position, 1, false, [&](NodeId parent_id, size_t, double) {
          // add edge enforcing single-parent constraint
          graph.insertEdge(parent_id, node->id, nullptr, true);
          active_parents.at(parent_id).insert(node->id);
        });
  }
}

void declare_config(GraphConnector::Config& config) {
  using namespace config;
  name("GraphConnector::Config");
  field(config.layers, "layers");
}

GraphConnector::GraphConnector(const Config& config)
    : config(config::checkValid(config)) {
  for (const auto& layer_config : config.layers) {
    layers_.emplace_back(layer_config);
  }
}

GraphConnector::~GraphConnector() = default;

void GraphConnector::connect(DynamicSceneGraph& graph) {
  const auto new_nodes = graph.getNewNodes(true);
  for (auto& layer : layers_) {
    // 1. Update parent set
    //    - for every previous parent node
    //      - prune removed nodes
    //      - for every node that is now archived: mark all tracked children as archived
    //    - add new nodes to parent set
    layer.updateParents(graph, new_nodes);
    // 2. Update child set
    //   - add new nodes to child set
    //   - for every child node:
    //     - prune removed nodes
    //     - connect node to parent if active
    layer.connectChildren(graph, new_nodes);
  }
}

}  // namespace hydra
