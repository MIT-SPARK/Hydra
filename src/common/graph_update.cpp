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

#include "hydra/common/config_utilities.h"

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
}

void declare_config(GraphUpdater::Config& config) {
  using namespace config;
  name("GraphUpdater::Config");
  field(config.layer_updates, "layer_updates");
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
    : config(config), next_id(config.prefix, 0) {}

GraphUpdater::GraphUpdater(const Config& config) : config(config::checkValid(config)) {
  for (const auto& [layer_name, tracker_config] : config.layer_updates) {
    trackers_.emplace(DsgLayers::StringToLayerId(layer_name),
                      LayerTracker(tracker_config));
  }
}

void GraphUpdater::update(const GraphUpdate& update, DynamicSceneGraph& graph) {
  for (const auto& [layer_id, layer_update] : update) {
    if (!layer_update) {
      LOG(WARNING) << "Received invalid update for layer " << layer_id;
    }

    auto iter = trackers_.find(layer_id);
    if (iter == trackers_.end()) {
      LOG(WARNING) << "Recieved udpates for unhandled layer " << layer_id;
      return;
    }

    auto& tracker = iter->second;
    for (auto&& attrs : layer_update->attributes) {
      VLOG(5) << "Emplacing " << tracker.next_id.getLabel() << " @ "
              << tracker.config.target_layer.value_or(layer_id) << " for layer "
              << layer_id;
      graph.emplaceNode(tracker.config.target_layer.value_or(layer_id),
                        tracker.next_id,
                        std::move(attrs));
      ++tracker.next_id;
    }
  }
}

}  // namespace hydra
