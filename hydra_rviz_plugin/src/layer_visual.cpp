#include "hydra_rviz_plugin/layer_visual.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "hydra_rviz_plugin/layer_config.h"

namespace hydra {

using spark_dsg::SceneGraphNode;

namespace {

inline Ogre::Vector3 eigen_to_ogre(const Eigen::Vector3d& v) {
  return {
      static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

}  // namespace

LayerVisual::LayerVisual(Ogre::SceneManager* const manager,
                         Ogre::SceneNode* const parent)
    : manager_(manager) {
  node_ = parent->createChildSceneNode();
}

LayerVisual::~LayerVisual() { manager_->destroySceneNode(node_); }

void LayerVisual::setPose(const Pose& pose) {
  node_->setPosition(pose.pos);
  node_->setOrientation(pose.rot);
}

void LayerVisual::makeNodes(const LayerConfig& config,
                            const spark_dsg::SceneGraphLayer& layer,
                            ColorFunctor* const color_callback) {
  if (!graph_nodes_) {
    graph_nodes_ = std::make_unique<rviz::PointCloud>();
    node_->attachObject(graph_nodes_.get());
  }

  graph_nodes_->setRenderMode(config.use_spheres ? rviz::PointCloud::RM_SPHERES
                                                 : rviz::PointCloud::RM_BOXES);
  const auto dim = config.node_scale;
  graph_nodes_->setDimensions(dim, dim, dim);
  graph_nodes_->clear();

  std::vector<rviz::PointCloud::Point> points(layer.numNodes());

  std::array<float, 3> color{0.0f, 0.0f, 0.0f};
  size_t point_index = 0;
  for (const auto& id_node_pair : layer.nodes()) {
    const auto& attrs = id_node_pair.second->attributes();
    auto& point = points[point_index];
    point.position.x = attrs.position.x();
    point.position.y = attrs.position.y();
    point.position.z = attrs.position.z();

    if (color_callback) {
      color_callback->call(*id_node_pair.second, point.color);
    } else {
      point.color.r = 0.0f;
      point.color.g = 0.0f;
      point.color.b = 0.0f;
    }
    point.color.a = config.node_alpha;
    ++point_index;
  }

  graph_nodes_->addPoints(&points.front(), points.size());
}

void LayerVisual::makeEdges(const LayerConfig& config,
                            const spark_dsg::SceneGraphLayer& layer) {
  if (!graph_edges_) {
    graph_edges_ = std::make_unique<rviz::BillboardLine>(manager_, node_);
  }

  graph_edges_->setColor(0, 0, 0, config.edge_alpha);
  graph_edges_->clear();
  graph_edges_->setLineWidth(config.edge_scale);
  graph_edges_->setMaxPointsPerLine(2);
  graph_edges_->setNumLines(layer.numEdges());

  std::vector<rviz::PointCloud::Point> points(layer.numNodes());

  Ogre::ColourValue color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = config.edge_alpha;

  bool is_first = true;
  for (const auto& id_edge_pair : layer.edges()) {
    const SceneGraphNode& source = *layer.getNode(id_edge_pair.second.source);
    const auto& source_attrs = source.attributes();
    const SceneGraphNode& target = *layer.getNode(id_edge_pair.second.target);
    const auto& target_attrs = target.attributes();

    const Ogre::Vector3 source_pos = eigen_to_ogre(source_attrs.position);
    const Ogre::Vector3 target_pos = eigen_to_ogre(target_attrs.position);

    if (!is_first) {
      graph_edges_->newLine();
    }
    is_first = false;
    graph_edges_->addPoint(source_pos, color);
    graph_edges_->addPoint(target_pos, color);
  }
}

void LayerVisual::setMessage(const LayerConfig& config,
                             const spark_dsg::SceneGraphLayer& layer,
                             ColorFunctor* const color_callback) {
  node_->setVisible(true);
  makeNodes(config, layer, color_callback);
  makeEdges(config, layer);
}

}  // namespace hydra
