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
#include "hydra/common/shared_dsg_info.h"

#include <config_utilities/config.h>
#include <spark_dsg/scene_graph.h>

#include <nlohmann/json.hpp>

#include "hydra/utils/logging.h"

namespace YAML {

template <>
struct convert<spark_dsg::LayerKey> {
  static Node encode(const spark_dsg::LayerKey& key) {
    Node node;
    node["layer"] = key.layer;
    node["partition"] = key.partition;
    return node;
  }

  static bool decode(const Node& node, spark_dsg::LayerKey& key) {
    if (node.IsScalar()) {
      key = node.as<spark_dsg::LayerId>();
      return true;
    }

    if (!node["layer"]) {
      HYDRA_LOG(ERROR, "Invalid layer key '{}', missing layer!", node);
      return false;
    }

    const spark_dsg::LayerId layer = node["layer"].as<spark_dsg::LayerId>();
    spark_dsg::PartitionId partition = 0;
    if (node["partition"]) {
      partition = node["partition"].as<spark_dsg::PartitionId>();
    }

    key = {layer, partition};
    return true;
  }
};

}  // namespace YAML

namespace hydra {

using namespace spark_dsg;

SharedDsgInfo::SharedDsgInfo(const Config& config)
    : updated(false), sequence_number(0) {
  graph = SceneGraph::fromNames(config.layers);
}

SharedDsgInfo::Ptr SharedDsgInfo::clone() const {
  auto other = std::make_shared<SharedDsgInfo>(config);
  other->updated = updated.load();
  other->sequence_number = sequence_number;
  other->graph = graph->clone();
  return other;
}

void SharedDsgInfo::updateMergesMetadata() const {
  auto mapping = nlohmann::json::object();
  for (const auto& [original, parent] : merges) {
    auto final_parent = parent;
    for (size_t i = 0; i < merges.size(); ++i) {
      const auto iter = merges.find(final_parent);
      if (iter == merges.end() || iter->second == final_parent) {
        break;
      }

      final_parent = iter->second;
    }

    mapping[std::to_string(original)] = final_parent;
  }

  auto metadata = graph->metadata.get();
  metadata["node_merges"] = std::move(mapping);
  graph->metadata.set(metadata);
}

void declare_config(SharedDsgInfo::Config& config) {
  using namespace config;
  name("SharedDsgInfo::Config");
  field(config.layers, "layers");
}

}  // namespace hydra
