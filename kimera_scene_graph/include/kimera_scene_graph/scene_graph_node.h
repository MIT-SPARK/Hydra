#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph_edge.h"

namespace kimera {

typedef ColorPointCloud NodePcl;

struct NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Timestamp timestamp_ = 0u;
  NodePosition position_ = NodePosition();
  NodeOrientation orientation_ = NodeOrientation();
  NodeColor color_ = NodeColor();
  SemanticLabel semantic_label_ = 0u;
  // This is what will be shown as label in the scene graph visualization
  NodeName name_ = "";
  // 3D points associated to this Node.
  NodePcl::Ptr pcl_ = nullptr;
  BoundingBox<ColorPoint> bounding_box_ = BoundingBox<ColorPoint>();

 public:
  std::string print() const;
};

struct SceneGraphNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  inline bool hasParent() const { return parent_edge_.isEdgeValid(); }
  inline bool hasSiblings() const { return !siblings_edge_map_.empty(); }
  inline bool hasSiblingEdge(const EdgeId& edge_id) const {
    return siblings_edge_map_.find(edge_id) != siblings_edge_map_.end();
  }

  bool checkSiblingEdgeMap() const;
  std::string print() const;

 public:
  //! Properties of the node
  NodeAttributes attributes_ = NodeAttributes();

 public:
  // Try to keep these protected as they are critical to be updated
  // appropriately
  //! Ids
  // TODO(Toni): perhaps init the ids to 0u so we know for sure if the user
  // updated these fields appropriately and didn't just forget...
  NodeId node_id_ = -1;
  LayerId layer_id_ = LayerId::kInvalidLayerId;

  //! Neighborhood of the node: this contains only intra layer edges!
  //! This list should have no duplicated edges.
  EdgeIdMap siblings_edge_map_;
  //! Each node only has one parent
  SceneGraphEdge parent_edge_;
  //! Each node has perhaps many children
  EdgeIdMap children_edge_map_;

  // Give SceneGraph and Layer access to private members of Node
  // friend class SceneGraph;
  // friend class SceneGraphLayer;
};
typedef std::map<NodeId, SceneGraphNode> NodeIdMap;

}  // namespace kimera
