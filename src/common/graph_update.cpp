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
#include "hydra/common/graph_update.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <algorithm>

#include "hydra/common/attribute_merger.h"

namespace YAML {

template <typename T>
struct convert<std::optional<T>> {
  static Node encode(const std::optional<T>& opt) {
    if (opt) {
      return YAML::convert<T>::encode(opt.value());
    }

    Node node("");
    node.SetTag("null");
    return node;
  }

  static bool decode(const Node& node, std::optional<T>& opt) {
    if (node) {
      if (node.Tag() == "null") {
        return true;
      }

      opt = node.as<T>();
    }

    return true;
  }
};

}  // namespace YAML

namespace hydra {

using namespace spark_dsg;

void declare_config(LayerTracker::Config& config) {
  using namespace config;
  name("LayerTracker::Config");
  field<CharConversion>(config.prefix, "prefix");
  field(config.target_layer, "target_layer");
  config.matcher.setOptional();
  field(config.matcher, "matcher");
  field<EarliestAttributeMerger::Config>(config.merger, "merger");
}

void declare_config(GraphUpdater::Config& config) {
  using namespace config;
  name("GraphUpdater::Config");
  field(config.layer_updates, "layer_updates");
  field(config.mark_active, "mark_active");
}

LayerUpdate::LayerUpdate(spark_dsg::LayerId layer) : layer(layer) {}

// TODO: Potentially implement merge resolution if multiple updates for same node are
// added in the same update.
void LayerUpdate::append(LayerUpdate&& rhs) {
  if (layer != rhs.layer) {
    return;
  }

  std::move(rhs.updates.begin(), rhs.updates.end(), std::back_inserter(updates));
  rhs.updates.clear();
}

LayerTracker::LayerTracker(const Config& config)
    : config(config),
      next_id(config.prefix, 0),
      matcher(config.matcher.create()),
      merger(config.merger.create()) {}

GraphUpdater::GraphUpdater(const Config& config) : config(config::checkValid(config)) {
  for (const auto& [layer_name, tracker_config] : config.layer_updates) {
    trackers_.emplace(layer_name, LayerTracker(tracker_config));
  }
}

void GraphUpdater::addNode(DynamicSceneGraph& graph,
                           LayerTracker& tracker,
                           LayerId target_layer_id,
                           LayerId source_layer_id,
                           const NodeUpdate& entry) {
  std::optional<NodeId> to_merge;
  const auto target_layer = graph.findLayer(target_layer_id);
  for (const auto& [node_id, node] : target_layer->nodes()) {
    auto& attrs = node->attributes();
    if (!attrs.is_active) {
      continue;
    }

    if (tracker.matcher && tracker.matcher->match(*entry.attributes, attrs)) {
      to_merge = node_id;
      break;
    }
  }

  if (to_merge) {
    VLOG(5) << "Merging attributes to " << NodeSymbol(*to_merge).str() << " @ "
            << target_layer_id << " for layer " << source_layer_id;
    if (entry.track_id) {
      tracker.track_to_node[*entry.track_id] = *to_merge;
    }
    const auto* existing_node = graph.findNode(*to_merge);
    if (existing_node) {
      auto merged =
          tracker.merger->merge({&existing_node->attributes(), entry.attributes.get()});
      if (merged) {
        if (config.mark_active) {
          merged->is_active = true;
        }
        graph.setNodeAttributes(*to_merge, std::move(merged));
      }
    }
    return;
  }

  auto attrs = entry.attributes->clone();
  if (config.mark_active) {
    attrs->is_active = true;
  }

  const NodeId new_id = tracker.next_id;
  VLOG(5) << "Emplacing " << tracker.next_id.str() << " @ " << target_layer_id
          << " for layer " << source_layer_id;
  graph.emplaceNode(target_layer_id, new_id, std::move(attrs));
  if (entry.track_id) {
    tracker.track_to_node[*entry.track_id] = new_id;
  }

  ++tracker.next_id;
  return;
}

void GraphUpdater::deleteNode(const NodeUpdate& entry,
                              LayerTracker& tracker,
                              DynamicSceneGraph& graph) {
  if (!entry.track_id) {
    LOG(WARNING) << "Delete graph update missing track_id";
    return;
  }

  const auto map_iter = tracker.track_to_node.find(*entry.track_id);
  if (map_iter == tracker.track_to_node.end()) {
    VLOG(5) << "Delete for unknown track_id " << *entry.track_id;
    return;
  }

  graph.removeNode(map_iter->second);
  tracker.track_to_node.erase(map_iter);
  return;
}

bool GraphUpdater::updateNode(const NodeUpdate& entry,
                              LayerTracker& tracker,
                              DynamicSceneGraph& graph) {
  const auto map_iter = tracker.track_to_node.find(*entry.track_id);
  if (map_iter != tracker.track_to_node.end()) {
    std::unique_ptr<NodeAttributes> updated;
    const auto* existing = graph.findNode(map_iter->second);
    if (existing) {
      updated =
          tracker.merger->merge({&existing->attributes(), entry.attributes.get()});
    }
    if (!updated) {
      updated = entry.attributes->clone();
    }
    if (config.mark_active) {
      updated->is_active = true;
    }

    graph.setNodeAttributes(map_iter->second, std::move(updated));
    return true;
  }

  return false;
}

void GraphUpdater::update(const GraphUpdate& update, DynamicSceneGraph& graph) {
  std::map<LayerId, LayerTracker&> trackers_by_id;
  for (auto& [name, tracker] : trackers_) {
    const auto key = graph.getLayerKey(name);
    if (!key) {
      LOG(WARNING) << "Invalid layer '" << name << "'";
      continue;
    }

    trackers_by_id.emplace(key->layer, tracker);
  }

  for (const auto& [layer_id, layer_update] : update) {
    if (!layer_update) {
      LOG(WARNING) << "Received invalid update for layer " << layer_id;
      continue;
    }

    auto iter = trackers_by_id.find(layer_id);
    if (iter == trackers_by_id.end()) {
      LOG(WARNING) << "Recieved updates for unhandled layer " << layer_id;
      continue;
    }

    auto& tracker = iter->second;
    const auto target_layer_id = tracker.config.target_layer.value_or(layer_id);

    std::map<NodeId, const NodeAttributes*> active_targets;
    for (auto&& entry : layer_update->updates) {
      switch (entry.update_type) {
        case NodeUpdate::UpdateType::Delete: {
          deleteNode(entry, tracker, graph);
          break;
        }
        case NodeUpdate::UpdateType::UpdateOrAdd: {
          if (!entry.attributes) {
            LOG(WARNING) << "Update graph update missing attributes";
            break;
          }
          if (!updateNode(entry, tracker, graph)) {
            addNode(graph, tracker, target_layer_id, layer_id, entry);
          }
          break;
        }
      }
    }
  }
}

}  // namespace hydra
