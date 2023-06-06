#include "hydra_rviz_plugin/bounding_box_visual.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "hydra_rviz_plugin/layer_config.h"

namespace hydra {

using spark_dsg::BoundingBox;
using spark_dsg::SceneGraphNode;
using spark_dsg::SemanticNodeAttributes;

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

BoundingBoxVisual::BoundingBoxVisual(Ogre::SceneManager* const manager,
                                     Ogre::SceneNode* const parent)
    : manager_(manager) {
  node_ = parent->createChildSceneNode();
}

BoundingBoxVisual::~BoundingBoxVisual() { manager_->destroySceneNode(node_); }

void BoundingBoxVisual::setPose(const Ogre::Quaternion& rot, const Ogre::Vector3& pos) {
  node_->setPosition(pos);
  node_->setOrientation(rot);
}

void BoundingBoxVisual::setMessage(const LayerConfig& config,
                                   const spark_dsg::SceneGraphLayer& layer,
                                   ColorFunctor* const color_callback) {
  node_->setVisible(true);
  if (!boxes_) {
    boxes_ = std::make_unique<rviz::BillboardLine>(manager_, node_);
  }

  boxes_->setColor(0, 0, 0, config.bounding_box_alpha);
  boxes_->clear();

  if (!layer.numNodes()) {
    return;
  }

  boxes_->setLineWidth(config.bounding_box_scale);
  boxes_->setMaxPointsPerLine(2);
  boxes_->setNumLines(12 * layer.numNodes());

  Ogre::ColourValue color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = config.bounding_box_alpha;

  bool is_first_line = true;
  std::vector<Ogre::Vector3> corners(8);
  std::array<float, 3> rgb_color{0.0f, 0.0f, 0.0f};
  for (const auto& id_node_pair : layer.nodes()) {
    const auto& attrs = id_node_pair.second->attributes<SemanticNodeAttributes>();
    if (color_callback) {
      color_callback->call(*id_node_pair.second, rgb_color);
    }
    color.r = rgb_color[0];
    color.g = rgb_color[1];
    color.b = rgb_color[2];

    const auto& bbox = attrs.bounding_box;
    const Eigen::Vector3f dims = bbox.max - bbox.min;
    for (int c = 0; c < 8; ++c) {
      // x: lsb, y: second lsb, z: third lsb
      Eigen::Vector3f offset;
      offset << (((c & 0x01) != 0) ? dims(0) : 0.0f),
          (((c & 0x02) != 0) ? dims(1) : 0.0f), (((c & 0x04) != 0) ? dims(2) : 0.0f);

      Eigen::Vector3f point = bbox.min + offset;
      if (bbox.type != BoundingBox::Type::AABB) {
        point = bbox.world_R_center * point + bbox.world_P_center;
      }
      corners[c] = eigen_to_ogre(point);
    }

    // top box corners are 4, 5, 6, 7
    for (int c = 0; c < 8; ++c) {
      // edges are 1-bit pertubations
      int x_neighbor = c | 0x01;
      int y_neighbor = c | 0x02;
      int z_neighbor = c | 0x04;

      if (c != x_neighbor) {
        pushSegment(*boxes_, corners[c], corners[x_neighbor], color, is_first_line);
      }
      if (c != y_neighbor) {
        pushSegment(*boxes_, corners[c], corners[y_neighbor], color, is_first_line);
      }
      if (c != z_neighbor) {
        pushSegment(*boxes_, corners[c], corners[z_neighbor], color, is_first_line);
      }
    }
  }
}

}  // namespace hydra
