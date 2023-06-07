#include "hydra_rviz_plugin/interlayer_edge_visual.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "hydra_rviz_plugin/edge_config.h"
#include "hydra_rviz_plugin/layer_config.h"
#include "hydra_rviz_plugin/scene_graph_display.h"

namespace hydra {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::SceneGraphNode;

using ContainerMap = std::map<LayerId, LayerContainer>;
using EdgeConfigMap = std::map<LayerPair, EdgeConfig>;
using EdgeContainer = std::map<LayerPair, std::vector<Edge>>;

namespace {

inline Ogre::Vector3 eigen_to_ogre(const Eigen::Vector3d& v) {
  const Eigen::Vector3f v_f = v.cast<float>();
  return {v_f.x(), v_f.y(), v_f.z()};
}

}  // namespace

InterlayerEdgeVisual::InterlayerEdgeVisual(Ogre::SceneManager* const manager,
                                           Ogre::SceneNode* const parent)
    : manager_(manager) {
  node_ = parent->createChildSceneNode();
}

InterlayerEdgeVisual::~InterlayerEdgeVisual() { manager_->destroySceneNode(node_); }

void InterlayerEdgeVisual::setPose(const Pose& pose) {
  node_->setPosition(pose.pos);
  node_->setOrientation(pose.rot);
}

void InterlayerEdgeVisual::fillEdgeContainers(const DynamicSceneGraph& graph,
                                              const ContainerMap& layers,
                                              const EdgeConfigMap& configs,
                                              EdgeContainer& edges) {
  std::map<LayerPair, size_t> num_since_last_insertion;
  for (const auto& edge : graph.interlayer_edges()) {
    const auto& source = graph.getNode(edge.second.source)->get();
    const auto& target = graph.getNode(edge.second.target)->get();
    const LayerPair layer_pair(source.layer, target.layer);
    const auto& parent_layer = layers.at(layer_pair.parent);
    const auto& child_layer = layers.at(layer_pair.child);

    auto citer = configs.find(layer_pair);
    if (citer == configs.end()) {
      continue;
    }
    const auto& config = citer->second;

    if (!parent_layer.config.visualize && !child_layer.config.visualize) {
      continue;
    }

    size_t num_between_insertions = configs.at(layer_pair).insertion_skip;

    auto iter = edges.find(layer_pair);
    if (iter == edges.end()) {
      iter = edges.emplace(layer_pair, std::vector<Edge>()).first;
      num_since_last_insertion[layer_pair] = num_between_insertions;
    }

    if (num_since_last_insertion[layer_pair] >= num_between_insertions) {
      num_since_last_insertion[layer_pair] = 0;
    } else {
      num_since_last_insertion[layer_pair]++;
      continue;
    }

    Edge new_edge;
    new_edge.start = eigen_to_ogre(source.attributes().position);
    new_edge.end = eigen_to_ogre(target.attributes().position);
    switch (config.color_mode) {
      case EdgeConfig::ColorMode::PARENT:
        break;
      case EdgeConfig::ColorMode::CHILD:
        break;
      case EdgeConfig::ColorMode::NONE:
      default:
        new_edge.color.r = 0.0;
        new_edge.color.g = 0.0;
        new_edge.color.b = 0.0;
        new_edge.color.a = config.edge_alpha;
        break;
    }
  }
}

void InterlayerEdgeVisual::resetEdges(const EdgeContainer& edges) {
  auto iter = edges_.begin();
  while (iter != edges_.end()) {
    if (!edges.count(iter->first)) {
      iter = edges_.erase(iter);
    }
    ++iter;
  }
}

void InterlayerEdgeVisual::makeEdges(const std::vector<Edge>& edges,
                                     const EdgeConfig& config,
                                     rviz::BillboardLine& edge_visual) {
  edge_visual.setColor(0, 0, 0, config.edge_alpha);
  edge_visual.clear();
  edge_visual.setLineWidth(config.edge_scale);
  edge_visual.setMaxPointsPerLine(2);
  edge_visual.setNumLines(edges.size());

  auto iter = edges.begin();
  while (iter != edges.end()) {
    if (iter != edges.begin()) {
      edge_visual.newLine();
    }

    edge_visual.addPoint(iter->start, iter->color);
    edge_visual.addPoint(iter->end, iter->end_color.value_or(iter->color));
    ++iter;
  }
}

void InterlayerEdgeVisual::setMessage(const spark_dsg::DynamicSceneGraph& graph,
                                      const std::map<LayerId, LayerContainer>& layers,
                                      const std::map<LayerPair, EdgeConfig>& configs) {
  node_->setVisible(true);

  EdgeContainer edges;
  fillEdgeContainers(graph, layers, configs, edges);

  for (auto&& [layer_pair, edge_list] : edges) {
    auto iter = edges_.find(layer_pair);
    if (iter == edges_.end()) {
      auto visual = std::make_unique<rviz::BillboardLine>(manager_, node_);
      iter = edges_.emplace(layer_pair, std::move(visual)).first;
    }

    auto& edge_visual = *iter->second;
    makeEdges(edge_list, configs.at(layer_pair), edge_visual);
  }
}

}  // namespace hydra
