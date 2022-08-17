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
#include "hydra_dsg_builder/dsg_update_functions.h"
#include "hydra_dsg_builder/incremental_room_finder.h"
#include "hydra_dsg_builder/object_helpers.h"

#include <gtsam/geometry/Pose3.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>

#include <glog/logging.h>

namespace hydra {
namespace dsg_updates {

using MeshVertices = DynamicSceneGraph::MeshVertices;
using Node = SceneGraphNode;

std::map<NodeId, NodeId> updateObjects(DynamicSceneGraph& graph,
                                       const gtsam::Values&,
                                       const gtsam::Values&,
                                       bool allow_node_merging) {
  if (!graph.hasLayer(DsgLayers::OBJECTS)) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::OBJECTS);
  MeshVertices::Ptr mesh = graph.getMeshVertices();

  std::map<NodeId, NodeId> nodes_to_merge;
  std::map<SemanticLabel, std::vector<NodeId>> semantic_nodes_map;
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<ObjectNodeAttributes>();

    std::vector<size_t> connections =
        graph.getMeshConnectionIndices(id_node_pair.first);
    if (connections.empty()) {
      VLOG(2) << "Found empty object node "
              << NodeSymbol(id_node_pair.first).getLabel();
      continue;
    }

    pcl::IndicesPtr indices;
    indices.reset(new std::vector<int>(connections.begin(), connections.end()));

    if (!updateObjectSpatialAttributes(mesh, indices, attrs)) {
      VLOG(2) << "Invalid centroid for object "
              << NodeSymbol(id_node_pair.first).getLabel();
      continue;
    }

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
            const bool curr_bigger =
                attrs.bounding_box.volume() > attrs_target.bounding_box.volume();
            VLOG(2) << "Merging " << NodeSymbol(id_node_pair.first).getLabel() << " ["
                    << attrs.bounding_box.volume() << "] "
                    << (curr_bigger ? " <- " : " -> ")
                    << NodeSymbol(node_target_id).getLabel() << " ["
                    << attrs_target.bounding_box.volume() << "]";

            if (curr_bigger) {
              nodes_to_merge[node_target_id] = id_node_pair.first;
            } else {
              nodes_to_merge[id_node_pair.first] = node_target_id;
            }
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
    VLOG(1) << "In DSG update, found " << nodes_to_merge.size()
            << " pairs of overlapping objects. Merging...";
    for (const auto& node_pair : nodes_to_merge) {
      graph.mergeNodes(node_pair.first, node_pair.second);
      VLOG(3) << "merging " << NodeSymbol(node_pair.first).getLabel() << " -> "
              << NodeSymbol(node_pair.second).getLabel();
    }
  }

  return nodes_to_merge;
}

std::map<NodeId, NodeId> updatePlaces(DynamicSceneGraph& graph,
                                      const gtsam::Values& values,
                                      const gtsam::Values&,
                                      bool allow_node_merging,
                                      double pos_threshold_m,
                                      double distance_tolerance_m) {
  if (!graph.hasLayer(DsgLayers::PLACES)) {
    return {};
  }

  if (values.size() == 0 && !allow_node_merging) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::PLACES);

  std::unordered_set<NodeId> missing_nodes;
  std::vector<NodeId> updated_nodes;
  std::map<NodeId, NodeId> nodes_to_merge;
  for (const auto& id_node_pair : layer.nodes()) {
    auto& attrs = id_node_pair.second->attributes<PlaceNodeAttributes>();
    if (!values.exists(id_node_pair.first)) {
      missing_nodes.insert(id_node_pair.first);
    } else {
      // TODO(nathan) consider updating distance via parents + deformation graph
      attrs.position = values.at<gtsam::Pose3>(id_node_pair.first).translation();
    }

    // TODO(yun) faster and smarter way to find overlap?
    if (!allow_node_merging) {
      continue;  // don't try to merge nodes when not allowed or active
    }

    bool to_be_merged = false;
    for (const auto& node_target_id : updated_nodes) {
      if (graph.hasEdge(id_node_pair.first, node_target_id)) {
        // Do not merge nodes already connected by an edge
        continue;
      }

      const Node& node_target = layer.getNode(node_target_id).value();
      const auto& attrs_target = node_target.attributes<PlaceNodeAttributes>();

      if ((attrs.position - attrs_target.position).norm() > pos_threshold_m) {
        continue;
      }

      if (std::abs(attrs.distance - attrs_target.distance) > distance_tolerance_m) {
        continue;
      }

      if (attrs_target.is_active) {
        // try to prefer merging active into non-active
        nodes_to_merge[node_target_id] = id_node_pair.first;
      } else {
        nodes_to_merge[id_node_pair.first] = node_target_id;
      }

      to_be_merged = true;
      break;
    }

    if (!to_be_merged) {
      // Prohibit merging to a node that is already to be merged
      updated_nodes.push_back(id_node_pair.first);
    }
  }

  if (nodes_to_merge.size() > 0) {
    VLOG(1) << "In DSG update, found " << nodes_to_merge.size()
            << " pairs of overlapping places. Merging...";
    for (const auto& node_pair : nodes_to_merge) {
      graph.mergeNodes(node_pair.first, node_pair.second);
      VLOG(3) << "merging " << NodeSymbol(node_pair.first).getLabel() << " -> "
              << NodeSymbol(node_pair.second).getLabel();
    }
  }
  // TODO(yun) regenerate rooms. Or trigger to regenerate rooms.

  if (!missing_nodes.empty()) {
    VLOG(6) << "[Places Layer]: could not update "
            << displayNodeSymbolContainer(missing_nodes);
    for (const auto& place_id : missing_nodes) {
      if (layer.hasNode(place_id)) {
        const Node& missing_node = layer.getNode(place_id).value();
        const auto& attrs = missing_node.attributes<PlaceNodeAttributes>();
        if (!attrs.is_active && !missing_node.hasSiblings()) {
          VLOG(4) << "[Places Layer]: removing node "
                  << gtsam::DefaultKeyFormatter(place_id);
          graph.removeNode(place_id);
        }
      }
    }
  }

  return nodes_to_merge;
}

// TODO(nathan) add unit test for this
std::map<NodeId, NodeId> updateRooms(DynamicSceneGraph& graph,
                                     const gtsam::Values&,
                                     const gtsam::Values&,
                                     bool) {
  if (!graph.hasLayer(DsgLayers::ROOMS)) {
    return {};
  }

  std::set<NodeId> empty_rooms;
  const auto& rooms = graph.getLayer(DsgLayers::ROOMS);
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

  return {};
}

std::map<NodeId, NodeId> updateBuildings(DynamicSceneGraph& graph,
                                         const gtsam::Values&,
                                         const gtsam::Values&,
                                         bool) {
  if (!graph.hasLayer(DsgLayers::BUILDINGS)) {
    return {};
  }

  const auto& layer = graph.getLayer(DsgLayers::BUILDINGS);

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

  return {};
}

std::map<NodeId, NodeId> updateAgents(DynamicSceneGraph& graph,
                                      const gtsam::Values&,
                                      const gtsam::Values& values,
                                      bool) {
  if (values.size() == 0) {
    return {};
  }

  const LayerId desired_layer = DsgLayers::AGENTS;

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
      LOG(WARNING) << "Layer " << DsgLayers::AGENTS << "(" << prefix_layer_pair.first
                   << "): could not update "
                   << displayNodeSymbolContainer(missing_nodes);
    }
  }

  return {};
}

}  // namespace dsg_updates
}  // namespace hydra
