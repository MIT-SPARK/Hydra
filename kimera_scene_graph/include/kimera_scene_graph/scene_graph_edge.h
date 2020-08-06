#pragma once

#include <map>
#include <string>
#include <vector>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"

namespace kimera {

class SceneGraphNode;

struct SceneGraphEdge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /**
   * @brief isInterLayerEdge
   * @return returns true if this edge is between layers (inter-layer)
   * or between the nodes in a layer (intra-layer).
   */
  inline bool isInterLayerEdge() const {
    return start_layer_id_ != end_layer_id_;
  }
  inline bool isEdgeValid() const {
    return edge_id_ != -1 && !isSelfEdge() &&
           start_layer_id_ != LayerId::kInvalidLayerId &&
           end_layer_id_ != LayerId::kInvalidLayerId;
  }
  inline bool isSelfEdge() const {
    return (start_layer_id_ == end_layer_id_) &&
           (start_node_id_ == end_node_id_);
  }
  inline bool swapDirection() {
    CHECK(isEdgeValid());
    std::swap(start_layer_id_, end_layer_id_);
    std::swap(start_node_id_, end_node_id_);
  }
  std::string print() const;

 public:
  // These could be protected if we add a ctor and a friend class for SceneGraph
  EdgeId edge_id_ = -1;
  LayerId start_layer_id_ = LayerId::kInvalidLayerId;
  NodeId start_node_id_ = -1;
  LayerId end_layer_id_ = LayerId::kInvalidLayerId;
  NodeId end_node_id_ = -1;
};
typedef std::map<EdgeId, SceneGraphEdge> EdgeIdMap;

inline std::vector<EdgeId> getEdgeIdsInEdgeIdMap(const EdgeIdMap& edge_id_map) {
  std::vector<EdgeId> edge_ids;
  for (const std::pair<EdgeId, SceneGraphEdge>& edge : edge_id_map) {
    edge_ids.push_back(edge.first);
  }
  return edge_ids;
}

}  // namespace kimera
