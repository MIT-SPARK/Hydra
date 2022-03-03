#include "kimera_dsg_builder/dsg_lcd_descriptors.h"

#include <glog/logging.h>
#include <kimera_dsg/node_attributes.h>

namespace kimera {
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

  const auto& places = graph.getLayer(KimeraDsgLayers::PLACES);
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

  const auto& places = graph.getLayer(KimeraDsgLayers::PLACES);
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
}  // namespace kimera
