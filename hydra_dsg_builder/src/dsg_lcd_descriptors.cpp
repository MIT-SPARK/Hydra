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
#include "hydra_dsg_builder/dsg_lcd_descriptors.h"

#include <glog/logging.h>

namespace hydra {
namespace lcd {

using Dsg = DynamicSceneGraph;
using DsgNode = DynamicSceneGraphNode;

Descriptor::Ptr AgentDescriptorFactory::construct(const Dsg& graph,
                                                  const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  const auto& attrs = agent_node.attributes<AgentNodeAttributes>();
  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = true;
  descriptor->words = attrs.dbow_ids;
  descriptor->values = attrs.dbow_values;
  descriptor->root_node = *parent;
  descriptor->nodes.insert(agent_node.id);
  descriptor->timestamp = agent_node.timestamp;
  descriptor->root_position =
      graph.getNode(*parent).value().get().attributes().position;
  return descriptor;
}

ObjectDescriptorFactory::ObjectDescriptorFactory(double radius, size_t num_classes)
    : radius(radius), num_classes(num_classes) {}

Descriptor::Ptr ObjectDescriptorFactory::construct(const Dsg& graph,
                                                   const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  const Eigen::Vector3d root_position =
      graph.getNode(*parent).value().get().attributes().position;

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->values = decltype(descriptor->values)::Zero(num_classes, 1);
  descriptor->root_node = *parent;
  descriptor->timestamp = agent_node.timestamp;
  descriptor->root_position = root_position;

  const auto& places = graph.getLayer(DsgLayers::PLACES);
  std::deque<NodeId> frontier{*parent};
  std::unordered_set<NodeId> visited{*parent};
  graph_utilities::breadthFirstSearch(
      places,
      frontier,
      visited,
      [&](const auto& curr_node) {
        for (const auto& child : curr_node.children()) {
          if (graph.isDynamic(child)) {
            continue;
          }

          const SceneGraphNode& child_node = graph.getNode(child).value();
          if ((root_position - child_node.attributes().position).norm() < radius) {
            return true;
          }
        }

        return (root_position - curr_node.attributes().position).norm() < radius;
      },
      [](const auto&) { return true; },
      [&](const SceneGraphLayer& layer, NodeId node_id) {
        const SceneGraphNode& node = layer.getNode(node_id).value();
        for (const auto& child : node.children()) {
          if (graph.isDynamic(child)) {
            continue;
          }

          const SceneGraphNode& child_node = graph.getNode(child).value();
          if ((root_position - child_node.attributes().position).norm() >= radius) {
            continue;
          }

          const size_t label =
              child_node.attributes<SemanticNodeAttributes>().semantic_label;
          if (label > static_cast<size_t>(descriptor->values.rows())) {
            LOG(WARNING) << "label " << label << " for node "
                         << NodeSymbol(child).getLabel() << " exceeds max label "
                         << descriptor->values.rows();
            continue;
          }

          descriptor->values(label) += 1.0f;
          descriptor->nodes.insert(child);
        }
      });

  return descriptor;
}

PlaceDescriptorFactory::PlaceDescriptorFactory(double radius,
                                               const HistogramConfig<double>& config)
    : radius(radius), config(config) {}

Descriptor::Ptr PlaceDescriptorFactory::construct(const Dsg& graph,
                                                  const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  const Eigen::Vector3d root_position =
      graph.getNode(*parent).value().get().attributes().position;

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->values = decltype(descriptor->values)::Zero(config.bins, 1);
  descriptor->root_node = *parent;
  descriptor->timestamp = agent_node.timestamp;
  descriptor->root_position = root_position;

  const auto& places = graph.getLayer(DsgLayers::PLACES);
  std::deque<NodeId> frontier{*parent};
  std::unordered_set<NodeId> visited{*parent};
  graph_utilities::breadthFirstSearch(
      places,
      frontier,
      visited,
      [&](const auto& curr_node) {
        return (root_position - curr_node.attributes().position).norm() < radius;
      },
      [](const auto&) { return true; },
      [&](const SceneGraphLayer& l, NodeId n) {
        const auto& attr = l.getNode(n).value().get().attributes<PlaceNodeAttributes>();
        descriptor->values(config.getBin(attr.distance)) += 1.0f;
        descriptor->nodes.insert(n);
      });

  return descriptor;
}

}  // namespace lcd
}  // namespace hydra
