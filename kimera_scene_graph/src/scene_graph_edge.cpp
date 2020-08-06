#include "kimera_scene_graph/scene_graph_edge.h"

namespace kimera {

std::string SceneGraphEdge::print() const {
  std::stringstream ss;
  // clang-format off
    ss << "SceneGraphEdge: \n"
       << "EdgeId : " << edge_id_ << '\n'
       << "Start Layer Id : " << to_underlying(start_layer_id_) << '\n'
       << "Start Node Id : " << start_node_id_ << '\n'
       << "End Layer Id : " << to_underlying(end_layer_id_) << '\n'
       << "End Node Id : " << end_node_id_;
  // clang-format on
  return ss.str();
}
}  // namespace kimera
