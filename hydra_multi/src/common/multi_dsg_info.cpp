#include "hydra_multi/common/multi_dsg_info.h"

#include <hydra/utils/logging.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph.h>

#include <nlohmann/json.hpp>

namespace hydra_multi {

using spark_dsg::LayerKey;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraph;

MultiDsgInfo::MultiDsgInfo(const hydra::SharedDsgInfo::Config& config)
    : hydra::SharedDsgInfo(config) {}

MultiDsgInfo::Ptr MultiDsgInfo::clone() const {
  auto other = std::make_shared<MultiDsgInfo>(config);
  other->updated = updated.load();
  other->sequence_number = sequence_number;
  other->graph = graph->clone();
  other->robot_node_map = robot_node_map;
  other->node_robot_map = node_robot_map;
  other->robot_vertex_offset = robot_vertex_offset;
  other->robot_num_vertices = robot_num_vertices;
  other->merges = merges;
  return other;
}

void MultiDsgInfo::update(const MultiDsgInfo& dsg) {
  if (!dsg.graph) {
    HYDRA_LOG(ERROR, "graph not initialized");
    return;
  }

  graph->clear();
  graph->mergeGraph(*dsg.graph);
  graph->setMesh(dsg.graph->mesh());
  updated = dsg.updated.load();
  sequence_number = dsg.sequence_number;
  merges = dsg.merges;
  robot_node_map = dsg.robot_node_map;
  node_robot_map = dsg.node_robot_map;
  robot_vertex_offset = dsg.robot_vertex_offset;
  robot_num_vertices = dsg.robot_num_vertices;
}

void MultiDsgInfo::clear() {
  graph->clear();
  robot_node_map.clear();
  node_robot_map.clear();
  robot_vertex_offset.clear();
  robot_num_vertices.clear();
  merges.clear();
  layer_partition_next_node_idx.clear();
  updated = false;
}

void MultiDsgInfo::addRobotGraph(size_t robot_id,
                                 const SceneGraph& dsg,
                                 Eigen::Isometry3d* transform) {
  if (!graph) {
    // Initialize Scene Graph
    graph.reset(new SceneGraph(dsg.layer_keys(), dsg.layer_names()));
    // TODO(Yun) forgot to update node mapping
  }

  // Check if robot has already been added
  if (robot_node_map.count(robot_id)) {
    HYDRA_LOG(ERROR, "Duplicated robot graphs detected when adding to MultiDsgInfo.");
    return;
  }

  if (!kimera_pgmo::isValidId(robot_id)) {
    HYDRA_LOG(FATAL, "Robot ID ({}) not assigned valid prefix.", robot_id);
  }

  robot_node_map[robot_id] = {};
  remapAndAddLayers(robot_id, dsg, transform);
  addLabelspaces(robot_id, dsg);
  updated = true;
}

void MultiDsgInfo::addLabelspaces(size_t robot_id, const SceneGraph& dsg) {
  // TODO(nathan) clean up this implementation
  // Semantic labels stay as they are when graphs are merged, so the merged graph
  // needs the robots' label -> category name maps to stay readable (without them,
  // every consumer, e.g. a prior map loaded into a planner, only sees label ids).
  const auto& incoming = dsg.metadata();
  const auto incoming_spaces = incoming.find("labelspaces");
  if (incoming_spaces == incoming.end() || !incoming_spaces->is_object()) {
    return;
  }

  const auto& current = graph->metadata();
  const auto current_spaces = current.find("labelspaces");
  auto to_add = nlohmann::json::object();
  for (const auto& [key, mapping] : incoming_spaces->items()) {
    if (mapping.empty()) {
      continue;
    }

    if (current_spaces != current.end() && current_spaces->contains(key) &&
        !current_spaces->at(key).empty()) {
      HYDRA_LOG_IF(WARNING,
                   current_spaces->at(key) != mapping,
                   "Robot {} has a different labelspace for '{}' than an earlier "
                   "robot; keeping the earlier one",
                   robot_id,
                   key);
      continue;
    }

    to_add[key] = mapping;
  }

  if (!to_add.empty()) {
    graph->metadata.add(nlohmann::json{{"labelspaces", to_add}});
  }
}

void MultiDsgInfo::remapAndAddLayer(size_t robot_id,
                                    LayerKey key,
                                    const SceneGraph& dsg,
                                    Eigen::Isometry3d* transform) {
  auto iter = layer_partition_next_node_idx.find(key);
  if (iter == layer_partition_next_node_idx.end()) {
    iter = layer_partition_next_node_idx.emplace(key, 0).first;
  }

  auto& node_map = robot_node_map[robot_id];

  const auto& layer = dsg.getLayer(key.layer, key.partition);
  for (const auto& node : layer.nodes()) {
    const NodeSymbol node_symb(node.id);
    const NodeId new_node_id = NodeSymbol(node_symb.category(), iter->second);

    // Add to graph.
    auto attrs = node.attributes().clone();
    if (transform) {
      attrs->transform(*transform);
    }

    graph->emplaceNode(key.layer, new_node_id, std::move(attrs), key.partition);

    // Update node book-keeping
    node_map[node.id] = new_node_id;
    node_robot_map[new_node_id] = robot_id;

    // Incremental next node index
    iter->second++;
  }

  for (const auto& edge : layer.edges()) {
    const NodeSymbol new_source = node_map.at(edge.source);
    const NodeSymbol new_target = node_map.at(edge.target);
    graph->insertEdge(new_source, new_target, edge.info->clone());
  }
}

void MultiDsgInfo::remapAndAddLayers(size_t robot_id,
                                     const SceneGraph& dsg,
                                     Eigen::Isometry3d* transform) {
  // Remap and add layers and partitions
  for (const auto& key : dsg.layer_keys()) {
    remapAndAddLayer(robot_id, key, dsg, transform);
  }

  // Add interlayer edges
  const auto& node_mapping = robot_node_map[robot_id];
  for (const auto& edge : dsg.interlayer_edges()) {
    const auto src_iter = node_mapping.find(edge.source);
    const auto tgt_iter = node_mapping.find(edge.target);
    if (src_iter == node_mapping.end() || tgt_iter == node_mapping.end()) {
      HYDRA_LOG(
          ERROR,
          "Edge between unmapped nodes: {} (remapped: {}) -> {} (remapped: {})",
          NodeSymbol(edge.source).str(),
          (src_iter == node_mapping.end() ? "???" : NodeSymbol(src_iter->second).str()),
          NodeSymbol(edge.target),
          (tgt_iter == node_mapping.end() ? "???"
                                          : NodeSymbol(tgt_iter->second).str()));
      continue;
    }

    if (!graph->insertEdge(src_iter->second, tgt_iter->second, edge.info->clone())) {
      HYDRA_LOG(ERROR,
                "Failed to insert interlayer edge {} - {}",
                NodeSymbol(node_mapping.at(edge.source)).str(),
                NodeSymbol(node_mapping.at(edge.target)).str());
    }
  }
}

}  // namespace hydra_multi
