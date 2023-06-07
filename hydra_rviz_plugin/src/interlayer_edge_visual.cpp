#include "hydra_rviz_plugin/interlayer_edge_visual.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "hydra_rviz_plugin/layer_config.h"

namespace hydra {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::SceneGraphNode;

struct Edge {
  Ogre::Vector3 start;
  Ogre::Vector3 end;
  Ogre::ColourValue color;
  std::optional<Ogre::ColourValue> end_color;
};

struct EdgeContainer {
  std::map<LayerId, std::map<LayerId, std::vector<Edge>>> edges;
};

namespace {

inline Ogre::Vector3 eigen_to_ogre(const Eigen::Vector3f& v) {
  return {v.x(), v.y(), v.z()};
}

inline void pushSegment(rviz::BillboardLine& line,
                        const Ogre::Vector3 start,
                        const Ogre::Vector3& end,
                        const Ogre::ColourValue& color,
                        bool& is_first) {
  if (!is_first) {
    line.newLine();
  }
  is_first = false;
  line.addPoint(start, color);
  line.addPoint(end, color);
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

void InterlayerEdgeVisual::makeEdges(LayerId source,
                                     LayerId target,
                                     const std::vector<Edge>& edges) {
  if (!edges_) {
    edges_ = std::make_unique<rviz::BillboardLine>(manager_, node_);
  }

  edges_->setColor(0, 0, 0, 1);
  edges_->clear();

  if (!layer.numNodes()) {
    return;
  }

  edges_->setLineWidth(config.bounding_box_scale);
  edges_->setMaxPointsPerLine(2);
  edges_->setNumLines(edges.size());

  auto iter = edges.begin();
  while (iter != edges.end()) {
    if (iter != edges.begin()) {
      edges_->newLine();
    }

    edges_->addPoint(iter->start, iter->color);
    edges_->addPoint(iter->end, iter->end_color.value_or(iter->color));
    ++iter;
  }
}

void InterlayerEdgeVisual::setMessage(const LayerConfig& config,
                                      const spark_dsg::SceneGraphLayer& layer,
                                      ColorFunctor* const color_callback) {
  node_->setVisible(true);

  EdgeContainer edges;
  fillEdgeContainers(graph, ..., edges);

  for (auto&& [source, targets] : edges) {
    for (auto&& [target, target_edges] : targets) {
    }
  }
}

void InterlayerEdgeVisual::fillEdgeContainers(const DynamicSceneGraph& graph,
                                              EdgeContainer& edges) {
  for (const auto& edge : graph.interlayer_edges()) {
    const auto& source = graph.getNode(edge.second.source)->get();
    const auto& target = graph.getNode(edge.second.target)->get();

    if (!configs.count(source.layer) || !configs.count(target.layer)) {
      continue;
    }

    if (!configs.at(source.layer).visualize) {
      continue;
    }

    if (!configs.at(target.layer).visualize) {
      continue;
    }

    size_t num_between_insertions =
        configs.at(source.layer).interlayer_edge_insertion_skip;

    // parent is always source
    if (layer_markers.count(source.layer) == 0) {
      layer_markers[source.layer] = makeNewEdgeList(
          header, configs.at(source.layer), ns_prefix, source.layer, target.layer);
      // make sure we always draw at least one edge
      num_since_last_insertion[source.layer] = num_between_insertions;
    }

    if (num_since_last_insertion[source.layer] >= num_between_insertions) {
      num_since_last_insertion[source.layer] = 0;
    } else {
      num_since_last_insertion[source.layer]++;
      continue;
    }

    Edge edge;
    edge.start = source.attributes().position;
    edge.end = target.attributes().position;
    // TODO(nathan): color
  }
}

}  // namespace hydra
