#include "kimera_dsg_builder/dsg_update_functions.h"
#include "kimera_dsg_builder/incremental_room_finder.h"
#include "kimera_dsg_builder/pcl_types.h"

#include <gtsam/geometry/Pose3.h>

#include <glog/logging.h>

namespace kimera {
namespace dsg_updates {

using MeshVertices = DynamicSceneGraph::MeshVertices;
using Node = SceneGraph::Node;

void updateObjects(DynamicSceneGraph& graph,
                   const gtsam::Values&,
                   const gtsam::Values&,
                   bool allow_node_merging) {
  if (!graph.hasLayer(KimeraDsgLayers::OBJECTS)) {
    return;
  }

  const SceneGraphLayer& layer = *graph.getLayer(KimeraDsgLayers::OBJECTS);
  MeshVertices::Ptr mesh = graph.getMeshVertices();

  std::vector<std::pair<NodeId, NodeId> > nodes_to_merge;
  std::map<SemanticLabel, std::vector<NodeId> > semantic_nodes_map;
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();

    std::vector<size_t> connections =
        graph.getMeshConnectionIndices(id_node_pair.first);
    if (connections.empty()) {
      LOG(WARNING) << "Found empty object node "
                   << NodeSymbol(id_node_pair.first).getLabel();
      continue;
    }

    pcl::IndicesPtr indices;
    indices.reset(new std::vector<int>(connections.begin(), connections.end()));

    attrs.bounding_box = BoundingBox::extract(mesh, attrs.bounding_box.type, indices);

    Centroid centroid;
    for (const auto& idx : *indices) {
      const auto& point = mesh->at(idx);
      centroid.add(pcl::PointXYZ(point.x, point.y, point.z));
    }

    pcl::PointXYZ pcl_pos;
    centroid.get(pcl_pos);
    attrs.position << pcl_pos.x, pcl_pos.y, pcl_pos.z;

    if (allow_node_merging) {
      bool to_be_merged = false;
      // TODO(yun) faster and smarter way to find overlap?
      if (semantic_nodes_map.count(attrs.semantic_label) > 0) {
        for (const auto& node_target_id : semantic_nodes_map[attrs.semantic_label]) {
          if (graph.hasEdge(id_node_pair.first, node_target_id)) {
            // Do not merge two nodes already connected by an edge
            continue;
          }
          const Node& node_target = layer.getNode(node_target_id).value();
          auto& attrs_target = node_target.attributes<ObjectNodeAttributes>();
          // Check for overlap
          if (attrs.bounding_box.isInside(attrs_target.position)) {
            nodes_to_merge.push_back({id_node_pair.first, node_target_id});
            to_be_merged = true;
            break;
            // TODO(Yun) Merge ones with larger overlap? For now assume more
            // will be merged next round
          }
        }
      } else {
        semantic_nodes_map[attrs.semantic_label] = std::vector<NodeId>();
      }

      if (!to_be_merged) {
        // Prohibit merging to a node that is already to be merged
        semantic_nodes_map[attrs.semantic_label].push_back(id_node_pair.first);
      }
    }
  }

  if (nodes_to_merge.size() > 0) {
    LOG(INFO) << "In DSG update, found " << nodes_to_merge.size()
              << " pairs of overlapping objects. Merging...";
    for (const auto& node_pair : nodes_to_merge) {
      graph.mergeNodes(node_pair.first, node_pair.second);
    }
  }
  // TODO(yun) trigger to regenerate object place parents... or leave as is?
}

void updatePlaces(DynamicSceneGraph& graph,
                  const gtsam::Values& values,
                  const gtsam::Values&,
                  bool allow_node_merging) {
  if (!graph.hasLayer(KimeraDsgLayers::PLACES)) {
    return;
  }

  if (values.size() == 0) {
    return;
  }

  const SceneGraphLayer& layer = graph.getLayer(KimeraDsgLayers::PLACES).value();

  std::unordered_set<NodeId> missing_nodes;
  std::vector<NodeId> updated_nodes;
  std::vector<std::pair<NodeId, NodeId> > nodes_to_merge;
  for (const auto& id_node_pair : layer.nodes()) {
    if (!values.exists(id_node_pair.first)) {
      missing_nodes.insert(id_node_pair.first);
      continue;
    }

    id_node_pair.second->attributes().position =
        values.at<gtsam::Pose3>(id_node_pair.first).translation();

    if (auto* attrs = dynamic_cast<PlaceNodeAttributes*>(
            id_node_pair.second->getAttributesPtr())) {
      attrs->position = values.at<gtsam::Pose3>(id_node_pair.first).translation();

      // TODO(nathan) consider updating distance via parents + deformation graph

      // TODO(yun) faster and smarter way to find overlap?
      double threshold = 0.1;  // TODO make this a parameter
      double tolerance = 0.01;
      bool to_be_merged = false;
      // Only try merge if is not active
      if (!attrs->is_active) {
        for (const auto& node_target_id : updated_nodes) {
          if (graph.hasEdge(id_node_pair.first, node_target_id)) {
            // Do not merge nodes already connected by an edge
            continue;
          }

          const Node& node_target = layer.getNode(node_target_id).value();
          auto& attrs_target = node_target.attributes<PlaceNodeAttributes>();

          // Check for overlap
          if ((attrs->position - attrs_target.position).norm() < threshold &&
              abs(attrs->distance - attrs_target.distance) < tolerance) {
            nodes_to_merge.push_back({id_node_pair.first, node_target_id});
            to_be_merged = true;
            break;
          }
        }

        if (!to_be_merged) {
          // Prohibit merging to a node that is already to be merged
          updated_nodes.push_back(id_node_pair.first);
        }
      }
    }
  }

  if (nodes_to_merge.size() > 0) {
    LOG(INFO) << "In DSG update, found " << nodes_to_merge.size()
              << " pairs of overlapping places. Merging...";
    for (const auto& node_pair : nodes_to_merge) {
      graph.mergeNodes(node_pair.first, node_pair.second);
    }
  }
  // TODO(yun) regenerate rooms. Or trigger to regenerate rooms.

  if (!missing_nodes.empty()) {
    LOG(WARNING) << "[Places Layer]: could not update "
                 << displayNodeSymbolContainer(missing_nodes);
  }
}

// TODO(nathan) add unit test for this
void updateRooms(DynamicSceneGraph& graph,
                 const gtsam::Values&,
                 const gtsam::Values&,
                 bool) {
  if (!graph.hasLayer(KimeraDsgLayers::ROOMS)) {
    return;
  }

  std::set<NodeId> empty_rooms;
  const SceneGraphLayer& rooms = graph.getLayer(KimeraDsgLayers::ROOMS).value();
  for (const auto& id_node_pair : rooms.nodes()) {
    if (id_node_pair.second->children().empty()) {
      empty_rooms.insert(id_node_pair.first);
    } else {
      incremental::updateRoomCentroid(graph, id_node_pair.first);
    }
  }

  for (const auto& room : empty_rooms) {
    graph.removeNode(room);
  }
}

void updateBuildings(DynamicSceneGraph& graph,
                     const gtsam::Values&,
                     const gtsam::Values&,
                     bool) {
  if (!graph.hasLayer(KimeraDsgLayers::BUILDINGS)) {
    return;
  }

  const SceneGraphLayer& layer = graph.getLayer(KimeraDsgLayers::BUILDINGS).value();

  for (const auto& id_node_pair : layer.nodes()) {
    if (!id_node_pair.second->hasChildren()) {
      continue;
    }

    Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
    for (const auto room_child : id_node_pair.second->children()) {
      mean_pos += graph.getPosition(room_child);
    }

    mean_pos /= id_node_pair.second->children().size();
    id_node_pair.second->attributes().position = mean_pos;
  }
}

void updateAgents(DynamicSceneGraph& graph,
                  const gtsam::Values&,
                  const gtsam::Values& values,
                  bool) {
  if (values.size() == 0) {
    return;
  }

  const LayerId desired_layer = KimeraDsgLayers::AGENTS;

  for (const auto& prefix_layer_pair : graph.dynamicLayersOfType(desired_layer)) {
    std::set<NodeId> missing_nodes;

    for (const auto& node : prefix_layer_pair.second->nodes()) {
      auto& attrs = node->attributes<AgentNodeAttributes>();
      if (!values.exists(attrs.external_key)) {
        missing_nodes.insert(node->id);
        continue;
      }

      gtsam::Pose3 agent_pose = values.at<gtsam::Pose3>(attrs.external_key);
      attrs.position = agent_pose.translation();
      attrs.world_R_body = Eigen::Quaterniond(agent_pose.rotation().matrix());
    }

    if (!missing_nodes.empty()) {
      LOG(WARNING) << "Layer " << KimeraDsgLayers::AGENTS << "("
                   << prefix_layer_pair.first << "): could not update "
                   << displayNodeSymbolContainer(missing_nodes);
    }
  }
}

}  // namespace dsg_updates
}  // namespace kimera
