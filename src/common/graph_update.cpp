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
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/serialization/json_conversions.h>

#include <algorithm>
#include <nlohmann/json.hpp>

#include "hydra/common/config_utilities.h"

namespace {

// Serialize node attributes for merge log; object meshes are omitted to reduce size.
void to_json_merge_log(nlohmann::json& record, const spark_dsg::NodeAttributes& attrs) {
  const auto* khronos =
      dynamic_cast<const spark_dsg::KhronosObjectAttributes*>(&attrs);
  if (khronos) {
    auto copy = khronos->clone();
    static_cast<spark_dsg::KhronosObjectAttributes*>(copy.get())->mesh.clear();
    to_json(record, *copy);
  } else {
    to_json(record, attrs);
  }
}

}  // namespace

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
}

void declare_config(GraphUpdater::Config& config) {
  using namespace config;
  name("GraphUpdater::Config");
  field(config.layer_updates, "layer_updates");
  field(config.mark_active, "mark_active");
}

LayerUpdate::LayerUpdate(spark_dsg::LayerId layer) : layer(layer) {}

void LayerUpdate::append(LayerUpdate&& rhs) {
  if (layer != rhs.layer) {
    return;
  }

  std::move(
      rhs.attributes.begin(), rhs.attributes.end(), std::back_inserter(attributes));
  rhs.attributes.clear();
}

LayerTracker::LayerTracker(const Config& config)
    : config(config), next_id(config.prefix, 0), matcher(config.matcher.create()) {}

GraphUpdater::GraphUpdater(const Config& config) : config(config::checkValid(config)) {
  for (const auto& [layer_name, tracker_config] : config.layer_updates) {
    trackers_.emplace(layer_name, LayerTracker(tracker_config));
  }
}

void GraphUpdater::update(const GraphUpdate& update,
                          DynamicSceneGraph& graph,
                          uint64_t timestamp_ns,
                          uint64_t sequence_number,
                          MergeLogCallback log_callback) {
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
      return;
    }

    auto& tracker = iter->second;
    const auto target_layer_id = tracker.config.target_layer.value_or(layer_id);

    std::map<NodeId, const NodeAttributes*> active_targets;
    const auto target_layer = graph.findLayer(target_layer_id);
    if (tracker.matcher && target_layer) {
      for (const auto& [node_id, node] : target_layer->nodes()) {
        auto& attrs = node->attributes();
        if (attrs.is_active) {
          active_targets[node_id] = &attrs;
        }
      }
    }

    nlohmann::json log_record;
    if (log_callback) {
      log_record["timestamp_ns"] = timestamp_ns;
      log_record["sequence_number"] = sequence_number;
      log_record["layer_id"] = layer_id;
      log_record["target_layer_id"] = target_layer_id;
      log_record["frontend_nodes_before"] = nlohmann::json::array();
      if (target_layer) {
        for (const auto& [node_id, node] : target_layer->nodes()) {
          auto& attrs = node->attributes();
          if (!attrs.is_active) {
            continue;
          }
          nlohmann::json node_entry;
          node_entry["node_id"] = NodeSymbol(node_id).str();
          to_json_merge_log(node_entry["attributes"], node->attributes());
          log_record["frontend_nodes_before"].push_back(node_entry);
        }
      }
      log_record["updates"] = nlohmann::json::array();
    } else {
    }

    for (auto&& attrs : layer_update->attributes) {
      if (!attrs) {
        continue;
      }

      std::optional<NodeId> to_merge;
      for (const auto& [target_id, target_attrs] : active_targets) {
        if (tracker.matcher && tracker.matcher->match(*attrs, *target_attrs)) {
          to_merge = target_id;
          break;
        }
      }

      if (log_callback) {
        nlohmann::json upd;
          to_json_merge_log(upd["incoming_attributes"], *attrs);
          if (to_merge) {
            upd["action"] = "merge";
            upd["target_node_id"] = NodeSymbol(*to_merge).str();
            to_json_merge_log(upd["target_attributes"], *active_targets.at(*to_merge));
          } else {
            upd["action"] = "new";
            upd["target_node_id"] = tracker.next_id.str();
          }
        log_record["updates"].push_back(upd);
      }

      if (to_merge) {
        VLOG(5) << "Merging attributes to " << NodeSymbol(*to_merge).str() << " @ "
                << target_layer_id << " for layer " << layer_id;
        continue;
      }

      if (config.mark_active) {
        attrs->is_active = true;
      }

      VLOG(5) << "Emplacing " << tracker.next_id.str() << " @ " << target_layer_id
              << " for layer " << layer_id;
      graph.emplaceNode(target_layer_id, tracker.next_id, std::move(attrs));
      ++tracker.next_id;
    }

    if (log_callback) {
      log_callback(log_record.dump());
    }
  }
}

}  // namespace hydra
