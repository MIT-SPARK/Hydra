#include "hydra_rviz_plugin/scene_graph_visual.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <spark_dsg/dynamic_scene_graph.h>

namespace hydra {

using spark_dsg::SceneGraphNode;

SceneGraphVisual::SceneGraphVisual(Ogre::SceneManager* manager, Ogre::SceneNode* parent)
    : manager_(manager) {
  node_ = parent->createChildSceneNode();
}

SceneGraphVisual::~SceneGraphVisual() { manager_->destroySceneNode(node_); }

void SceneGraphVisual::setPose(const Ogre::Vector3& pos, const Ogre::Quaternion& rot) {
  node_->setPosition(pos);
  node_->setOrientation(rot);
}

void SceneGraphVisual::makeNodes(const spark_dsg::SceneGraphLayer& layer) {
  if (!graph_nodes_) {
    graph_nodes_ = std::make_unique<rviz::PointCloud>();
    node_->attachObject(graph_nodes_.get());
  }

  graph_nodes_->setRenderMode(rviz::PointCloud::RM_SPHERES);
  graph_nodes_->setDimensions(0.1, 0.1, 0.1);
  graph_nodes_->clear();

  std::vector<rviz::PointCloud::Point> points(layer.numNodes());

  size_t point_index = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    const auto& attrs = id_node_pair.second->attributes();
    auto& point = points[point_index];
    point.position.x = attrs.position.x();
    point.position.y = attrs.position.y();
    point.position.z = attrs.position.z();
    point.setColor(1, 0, 0, 1);
    ++point_index;
  }

  graph_nodes_->addPoints(&points.front(), points.size());
}

inline Ogre::Vector3 eigen_to_ogre(const Eigen::Vector3d& v) {
  return {
      static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

void SceneGraphVisual::makeEdges(const spark_dsg::SceneGraphLayer& layer) {
  if (!graph_edges_) {
    graph_edges_ = std::make_unique<rviz::BillboardLine>(manager_, node_);
  }

  graph_edges_->setColor(0, 0, 0, 1);
  graph_edges_->clear();
  graph_edges_->setLineWidth(0.1);
  graph_edges_->setMaxPointsPerLine(2 * layer.numEdges());

  std::vector<rviz::PointCloud::Point> points(layer.numNodes());

  Ogre::ColourValue color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = 1;

  for (const auto& id_edge_pair : layer.edges()) {
    const SceneGraphNode& source = *layer.getNode(id_edge_pair.second.source);
    const auto& source_attrs = source.attributes();
    const SceneGraphNode& target = *layer.getNode(id_edge_pair.second.target);
    const auto& target_attrs = target.attributes();

    const Ogre::Vector3 source_pos = eigen_to_ogre(source_attrs.position);
    const Ogre::Vector3 target_pos = eigen_to_ogre(target_attrs.position);

    graph_edges_->addPoint(source_pos, color);
    graph_edges_->addPoint(target_pos, color);
  }
}

void SceneGraphVisual::setMessage(const spark_dsg::DynamicSceneGraph& graph) {
  node_->setVisible(true);

  const auto& places = graph.getLayer(spark_dsg::DsgLayers::PLACES);
  makeNodes(places);
  makeEdges(places);
}

}  // namespace hydra
