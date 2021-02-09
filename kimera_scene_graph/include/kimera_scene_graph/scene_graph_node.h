#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/serialization.hpp>

#include <kimera_semantics/common.h>

#include "kimera_scene_graph/common.h"
#include "kimera_scene_graph/scene_graph_edge.h"

namespace kimera {

typedef ColorPointCloud NodePcl;

// We don't use pcl or eigen in order to ease the boost serialization...
struct NodePosition {
 public:
  typedef double T;  // this is to avoid templating, but give flexibility...
  NodePosition() : NodePosition(0.0, 0.0, 0.0) {}
  NodePosition(T x, T y, T z) : x(x), y(y), z(z) {}
  ~NodePosition() = default;

 public:
  T x = 0.0;
  T y = 0.0;
  T z = 0.0;

 private:
  friend std::ostream& operator<<(std::ostream& os,
                                  const NodePosition& node_position);
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(x);
    ar& BOOST_SERIALIZATION_NVP(y);
    ar& BOOST_SERIALIZATION_NVP(z);
  }
};

inline std::ostream& operator<<(std::ostream& os,
                                const NodePosition& node_position) {
  os << "x: " << node_position.x << ", y: " << node_position.y
     << ", z: " << node_position.z;
  return os;
}

// We don't use pcl or eigen in order to ease the boost serialization...
struct NodeColor {
 public:
  typedef uint8_t T;  // from 0 to 255
  NodeColor() : NodeColor(0u, 0u, 0u) {}
  NodeColor(T r, T g, T b) : r(r), g(g), b(b) {}
  ~NodeColor() = default;

 public:
  T r = 0u;
  T g = 0u;
  T b = 0u;

 private:
  friend std::ostream& operator<<(std::ostream& os,
                                  const NodeColor& node_color);
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(r);
    ar& BOOST_SERIALIZATION_NVP(g);
    ar& BOOST_SERIALIZATION_NVP(b);
  }
};

inline std::ostream& operator<<(std::ostream& os, const NodeColor& node_color) {
  os << "r: " << node_color.r << ", g: " << node_color.g
     << ", b: " << node_color.b;
  return os;
}

struct NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Timestamp timestamp_ = 0u;
  NodePosition position_ = NodePosition();
  NodeColor color_ = NodeColor(0u, 0u, 0u);
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
  SceneGraphNode();
  ~SceneGraphNode() = default;

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
