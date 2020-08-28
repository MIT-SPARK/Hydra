#include "kimera_scene_graph/scene_graph_layer.h"

#include <map>
#include <vector>

#include <glog/logging.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_node.h"

namespace kimera {

SceneGraphLayer::SceneGraphLayer()
    : SceneGraphLayer(LayerId::kInvalidLayerId) {}

SceneGraphLayer::SceneGraphLayer(const LayerId& layer_id)
    : layer_id_(layer_id),
      node_map_(),
      next_intra_layer_edge_id_(0u),
      intra_layer_edge_map_() {}

ColorPointCloud::Ptr SceneGraphLayer::convertLayerToPcl(
    std::map<int, NodeId>* cloud_to_graph_ids,
    std::vector<NodeId>* vertex_ids) const {
  ColorPointCloud::Ptr skeleton_graph_cloud(new ColorPointCloud);
  // Get a list of all vertices
  skeleton_graph_cloud->resize(getNumberOfNodes());
  size_t i = 0u;
  for (const std::pair<NodeId, SceneGraphNode>& node : node_map_) {
    CHECK_EQ(node.first, node.second.node_id_);
    CHECK(node.second.layer_id_ == layer_id_);
    const NodePosition& position = node.second.attributes_.position_;
    ColorPoint point;
    point.x = position.x;
    point.y = position.y;
    point.z = position.z;
    skeleton_graph_cloud->at(i) = point;
    if (cloud_to_graph_ids) (*cloud_to_graph_ids)[i] = node.first;
    if (vertex_ids) vertex_ids->push_back(node.first);
    ++i;
  }
  CHECK_EQ(i, getNumberOfNodes());
  return skeleton_graph_cloud;
}

bool SceneGraphLayer::addNode(const SceneGraphNode& node) {
  if (hasNode(node.node_id_)) {
    return false;
  } else {
    node_map_[node.node_id_] = node;
    return true;
  }
}

void SceneGraphLayer::addIntraLayerEdge(SceneGraphEdge* edge) {
  CHECK_NOTNULL(edge);
  CHECK(!edge->isInterLayerEdge())
      << "Use addIntraLayerEdge on a layer only for "
         "intra-layer edges. Use addInterEdge on the "
         "scene-graph for inter-layer edges.";

  int64_t edge_id = next_intra_layer_edge_id_++;
  CHECK(!hasEdge(edge_id)) << "Adding an already existing edge...";

  // Update edge id
  edge->edge_id_ = edge_id;

  // Then, update intra layer edge map
  intra_layer_edge_map_[edge_id] = *edge;

  // Update node's edges maps
  CHECK(edge->start_layer_id_ == layer_id_);
  CHECK(hasNode(edge->start_node_id_));
  SceneGraphNode& start_vertex = node_map_[edge->start_node_id_];
  start_vertex.siblings_edge_map_[edge_id] = *edge;

  CHECK(edge->end_layer_id_ == layer_id_);
  CHECK(hasNode(edge->end_node_id_));
  SceneGraphNode& end_vertex = node_map_[edge->end_node_id_];
  end_vertex.siblings_edge_map_[edge_id] = *edge;

  CHECK(edge->isEdgeValid()) << edge->print();
}

}  // namespace kimera
