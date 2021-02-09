#pragma once

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>

#include <glog/logging.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>

#include "kimera_scene_graph/scene_graph.h"
#include "kimera_scene_graph/scene_graph_edge.h"
#include "kimera_scene_graph/scene_graph_layer.h"
#include "kimera_scene_graph/scene_graph_node.h"

// Non-intrusive serialization functions
namespace boost {

namespace serialization {

template <class Archive>
void serialize(Archive& ar,
               kimera::SceneGraphEdge& edge,
               const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(edge.edge_id_);
  ar& BOOST_SERIALIZATION_NVP(edge.start_layer_id_);
  ar& BOOST_SERIALIZATION_NVP(edge.start_node_id_);
  ar& BOOST_SERIALIZATION_NVP(edge.end_layer_id_);
  ar& BOOST_SERIALIZATION_NVP(edge.end_node_id_);
}

template <class Archive>
void serialize(Archive& ar,
               kimera::SceneGraphNode& node,
               const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(node.attributes_);
  ar& BOOST_SERIALIZATION_NVP(node.node_id_);
  ar& BOOST_SERIALIZATION_NVP(node.layer_id_);
  ar& BOOST_SERIALIZATION_NVP(node.siblings_edge_map_);
  ar& BOOST_SERIALIZATION_NVP(node.parent_edge_);
  ar& BOOST_SERIALIZATION_NVP(node.children_edge_map_);
}

template <class Archive>
void serialize(Archive& ar,
               kimera::NodeAttributes& attributes,
               const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(attributes.timestamp_);
  // Need to provide PCL Point serialization
  ar& BOOST_SERIALIZATION_NVP(attributes.position_);
  // Need to provide Eigen serialization
  ar& BOOST_SERIALIZATION_NVP(attributes.color_);
  ar& BOOST_SERIALIZATION_NVP(attributes.semantic_label_);
  ar& BOOST_SERIALIZATION_NVP(attributes.name_);
  // Need to provide PCL serialization.
  // ar& BOOST_SERIALIZATION_NVP(attributes.pcl_);
  // Need to provide bounding box serialization.
  ar& BOOST_SERIALIZATION_NVP(attributes.bounding_box_);
}

template <class Archive>
void serialize(Archive& ar,
               pcl::PointXYZRGB& point,
               const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(point.x);
  ar& BOOST_SERIALIZATION_NVP(point.y);
  ar& BOOST_SERIALIZATION_NVP(point.z);
  ar& BOOST_SERIALIZATION_NVP(point.r);
  ar& BOOST_SERIALIZATION_NVP(point.g);
  ar& BOOST_SERIALIZATION_NVP(point.b);
}

}  // namespace serialization

}  // namespace boost

namespace kimera {

template <class T>
void save(const T& object, const std::string& filename) {
  std::ofstream ofstream(filename.c_str());
  boost::archive::text_oarchive oa(ofstream);
  oa << object;
}

template <class T>
void load(const std::string& filename, T* object) {
  CHECK_NOTNULL(object);
  std::ifstream ifstream(filename.c_str());
  boost::archive::text_iarchive ia(ifstream);
  ia >> *object;
}

}  // namespace kimera
