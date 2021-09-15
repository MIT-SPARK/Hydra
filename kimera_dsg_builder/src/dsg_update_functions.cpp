#include "kimera_dsg_builder/dsg_update_functions.h"
#include "kimera_dsg_builder/pcl_types.h"

#include <gtsam/geometry/Pose3.h>

#include <glog/logging.h>

namespace kimera {
namespace dsg_updates {

using MeshVertices = DynamicSceneGraph::MeshVertices;

void updateObjects(const DynamicSceneGraph& graph,
                   const gtsam::Values&,
                   const gtsam::Values&) {
  if (!graph.hasLayer(KimeraDsgLayers::OBJECTS)) {
    return;
  }

  const SceneGraphLayer& layer = *graph.getLayer(KimeraDsgLayers::OBJECTS);
  MeshVertices::Ptr mesh = graph.getMeshVertices();

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
  }
}

void updatePlaces(const DynamicSceneGraph& graph,
                  const gtsam::Values& values,
                  const gtsam::Values&) {
  if (!graph.hasLayer(KimeraDsgLayers::PLACES)) {
    return;
  }

  const SceneGraphLayer& layer = graph.getLayer(KimeraDsgLayers::PLACES).value();

  std::unordered_set<NodeId> missing_nodes;
  for (const auto& id_node_pair : layer.nodes()) {
    if (!values.exists(id_node_pair.first)) {
      missing_nodes.insert(id_node_pair.first);
      continue;
    }

    id_node_pair.second->attributes().position =
        values.at<gtsam::Pose3>(id_node_pair.first).translation();

    // TODO(nathan) consider updating distance via parents + deformation graph
  }

  if (!missing_nodes.empty()) {
    LOG(WARNING) << "[Places Layer]: could not update "
                 << displayNodeSymbolContainer(missing_nodes);
  }
}

void updateRooms(const DynamicSceneGraph& graph,
                 const gtsam::Values&,
                 const gtsam::Values&) {
  if (!graph.hasLayer(KimeraDsgLayers::ROOMS)) {
    return;
  }

  // TODO(nathan) fill this out once we have room detecction
}

void updateBuildings(const DynamicSceneGraph& graph,
                     const gtsam::Values&,
                     const gtsam::Values&) {
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

void updateAgents(const DynamicSceneGraph& graph,
                  const gtsam::Values&,
                  const gtsam::Values& values) {
  const LayerId desired_layer = static_cast<LayerId>(KimeraDsgLayers::AGENTS);

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
      LOG(WARNING) << "Layer " << static_cast<LayerId>(KimeraDsgLayers::AGENTS) << "("
                   << prefix_layer_pair.first << "): could not update "
                   << displayNodeSymbolContainer(missing_nodes);
    }
  }
}

}  // namespace dsg_updates
}  // namespace kimera
