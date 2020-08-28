#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

std::string NodeAttributes::print() const {
  std::stringstream ss;
  // clang-format off
    ss << "Attributes: \n"
        << "Timestamp : " << std::to_string(timestamp_) << '\n'
        << "Position : " << position_ << '\n'
        << "Color : " << color_ << '\n'
        << "Semantic Label: " << std::to_string(semantic_label_) << '\n'
        << "Name: " << name_ << '\n'
        // << "Pcl size: " << pcl_ ? std::to_string(pcl_->size()) : "no pcl attached..."
           ;
  // clang-format on
  return ss.str();
}

SceneGraphNode::SceneGraphNode()
    : attributes_(),
      node_id_(-1),
      layer_id_(LayerId::kInvalidLayerId),
      siblings_edge_map_(),
      parent_edge_(),
      children_edge_map_() {}

bool SceneGraphNode::checkSiblingEdgeMap() const {
  for (const std::pair<EdgeId, SceneGraphEdge>& edge_pair :
       siblings_edge_map_) {
    const SceneGraphEdge& edge = edge_pair.second;
    if (edge.start_node_id_ != node_id_ && edge.end_node_id_ != node_id_) {
      // The current edge is not connected to this node...
      return false;
    } else {
      CHECK_EQ(edge.start_node_id_, edge.end_node_id_);
      if (edge.start_node_id_ == node_id_ &&
          edge.start_layer_id_ != layer_id_) {
        // The current edge start node is this node, but the layers do not
        // match...
        return false;
      }
      if (edge.end_node_id_ == node_id_ && edge.end_layer_id_ != layer_id_) {
        // The current edge end node is this node, but the layers do not
        // match...
        return false;
      }
    }
  }
  return true;
}

std::string SceneGraphNode::print() const {
  std::stringstream ss;
  ss << " - Node Id " << std::to_string(node_id_) << '\n'
     << " - Layer Id " << to_underlying(layer_id_) << '\n'
     << attributes_.print();
  return ss.str();
}

}  // namespace kimera
