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
#include "hydra/loop_closure/scene_graph_descriptors.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include "hydra/common/hydra_config.h"
#include "hydra/utils/display_utilities.h"

namespace hydra {
namespace lcd {

using Dsg = DynamicSceneGraph;
using DsgNode = DynamicSceneGraphNode;
using ObjectFactory = ObjectGraphDescriptorFactory;
using PlaceFactory = PlaceGraphDescriptorFactory;

ObjectFactory::ObjectGraphDescriptorFactory(const Config& config)
    : config(config::checkValid(config)) {}

Descriptor::Ptr ObjectFactory::construct(const Dsg& graph,
                                         const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  const Eigen::Vector3d root_position =
      graph.getNode(*parent).value().get().attributes().position;

  const auto num_classes = HydraConfig::instance().getTotalLabels();

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->values = decltype(descriptor->values)::Zero(num_classes, 1);
  descriptor->root_node = *parent;
  descriptor->timestamp = agent_node.timestamp;
  descriptor->root_position = root_position;
  descriptor->nodes = getSubgraphNodes(config.subgraph, graph, *parent, false);

  for (const auto node : descriptor->nodes) {
    const auto attrs = graph.getNode(node)->get().attributes<SemanticNodeAttributes>();
    const size_t label = attrs.semantic_label;
    if (label > static_cast<size_t>(descriptor->values.rows())) {
      LOG(ERROR) << "label " << static_cast<int>(label) << " for node "
                 << printNodeId(node) << " exceeds max label "
                 << descriptor->values.rows();
      continue;
    }

    descriptor->values(label) += 1.0f;
  }

  return descriptor;
}

void declare_config(ObjectGraphDescriptorFactory::Config& config) {
  using namespace config;
  name("ObjectGraphDescriptorFactory::Config");
  field(config.subgraph, "subgraph");
}

PlaceFactory::PlaceGraphDescriptorFactory(const Config& config)
    : config(config::checkValid(config)) {}

Descriptor::Ptr PlaceFactory::construct(const Dsg& graph,
                                        const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  const Eigen::Vector3d root_position =
      graph.getNode(*parent).value().get().attributes().position;

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->values = decltype(descriptor->values)::Zero(config.histogram.bins, 1);
  descriptor->root_node = *parent;
  descriptor->timestamp = agent_node.timestamp;
  descriptor->root_position = root_position;
  descriptor->nodes = getSubgraphNodes(config.subgraph, graph, *parent, true);

  const auto& places = graph.getLayer(DsgLayers::PLACES);
  for (const auto node : descriptor->nodes) {
    const auto& attrs = places.getNode(node)->get().attributes<PlaceNodeAttributes>();
    descriptor->values(config.histogram.getBin(attrs.distance)) += 1.0f;
  }

  return descriptor;
}

void declare_config(PlaceGraphDescriptorFactory::Config& config) {
  using namespace config;
  name("PlaceGraphDescriptorFactory::Config");
  field(config.subgraph, "subgraph");
  field(config.histogram, "histogram");
}

}  // namespace lcd
}  // namespace hydra
