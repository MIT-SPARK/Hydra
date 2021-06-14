#include "kimera_scene_graph/voxblox_conversions.h"
#include <kimera_dsg/node_attributes.h>

namespace voxblox {

std_msgs::ColorRGBA getVertexColor(const Mesh& mesh,
                                   const ColorMode& color_mode,
                                   const size_t index) {
  std_msgs::ColorRGBA color_msg;
  switch (color_mode) {
    case kColor:
      colorVoxbloxToMsg(mesh.colors[index], &color_msg);
      break;
    case kHeight:
      heightColorFromVertex(mesh.vertices[index], &color_msg);
      break;
    case kNormals:
      normalColorFromNormal(mesh.normals[index], &color_msg);
      break;
    case kLambert:
      lambertColorFromNormal(mesh.normals[index], &color_msg);
      break;
    case kLambertColor:
      lambertColorFromColorAndNormal(
          mesh.colors[index], mesh.normals[index], &color_msg);
      break;
    case kGray:
      color_msg.r = color_msg.g = color_msg.b = 0.5;
      color_msg.a = 1.0;
      break;
  }
  return color_msg;
}

}  // namespace voxblox

namespace kimera {

namespace utils {

// TODO(nathan) ditch when possible
void convertLayerToSkeleton(const SceneGraphLayer& layer,
                            vxb::SparseSkeletonGraph* skeleton) {
  CHECK_NOTNULL(skeleton);
  skeleton->clear();

  std::map<NodeId, int64_t> node_id_map;
  for (const auto& id_node_pair : layer.nodes) {
    vxb::SkeletonVertex skeleton_node;
    if (id_node_pair.second->hasParent()) {
      skeleton_node.room_id = *(id_node_pair.second->getParent());
    }

    skeleton_node.distance = 1u;  // This is the ESDF distance (set to random).
    skeleton_node.subgraph_id = 1u;  // Set all to be in the same subgraph.

    Eigen::Vector3d node_position = id_node_pair.second->attributes().position;
    skeleton_node.point[0] = node_position(0);
    skeleton_node.point[1] = node_position(1);
    skeleton_node.point[2] = node_position(2);

    node_id_map[id_node_pair.first] = skeleton->addVertex(skeleton_node);
  }

  for (const auto& id_edge_pair : layer.edges) {
    vxb::SkeletonEdge skeleton_edge;
    skeleton_edge.start_vertex = node_id_map.at(id_edge_pair.second.source);
    skeleton_edge.end_vertex = node_id_map.at(id_edge_pair.second.target);
    skeleton_edge.start_distance = 1u;
    skeleton_edge.end_distance = 1u;
    skeleton->addEdge(skeleton_edge);
  }
}

void fillLayerFromSkeleton(const vxb::SparseSkeletonGraph& skeleton,
                           SceneGraph* scene_graph) {
  CHECK(scene_graph);

  std::vector<int64_t> vertex_ids;
  skeleton.getAllVertexIds(&vertex_ids);

  for (const auto& idx : vertex_ids) {
    const vxb::SkeletonVertex& vertex = skeleton.getVertex(idx);

    // TODO(nathan) basis points aren't actually propagated through
    PlaceNodeAttributes::Ptr attrs =
        std::make_unique<PlaceNodeAttributes>(vertex.distance, 0);
    attrs->semantic_label = kPlaceSemanticLabel;
    attrs->position << vertex.point[0], vertex.point[1], vertex.point[2];
    attrs->color << 255u, 0u, 0u;

    scene_graph->emplaceNode(to_underlying(KimeraDsgLayers::PLACES),
                             NodeSymbol('P', idx),
                             std::move(attrs));
  }

  std::vector<int64_t> edge_ids;
  skeleton.getAllEdgeIds(&edge_ids);

  for (const auto& edge_id : edge_ids) {
    const vxb::SkeletonEdge& edge = skeleton.getEdge(edge_id);
    if (edge.start_vertex == edge.end_vertex) {
      continue;
    }

    scene_graph->insertEdge(NodeSymbol('P', edge.start_vertex),
                            NodeSymbol('P', edge.end_vertex));
  }
}

}  // namespace utils

}  // namespace kimera
