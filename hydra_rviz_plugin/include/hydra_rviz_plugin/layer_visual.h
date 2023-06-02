#pragma once
#include <memory>

namespace Ogre {

class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;

}  // namespace Ogre

namespace rviz {

class PointCloud;
class BillboardLine;

}  // namespace rviz

namespace spark_dsg {

class SceneGraphLayer;

}  // namespace spark_dsg

namespace hydra {

struct LayerConfig;

class LayerVisual {
 public:
  LayerVisual(Ogre::SceneManager* const manager, Ogre::SceneNode* const parent);

  virtual ~LayerVisual();

  void setMessage(const LayerConfig& config, const spark_dsg::SceneGraphLayer& msg);

  void setPose(const Ogre::Quaternion& rot, const Ogre::Vector3& pos);

  void makeNodes(const LayerConfig& config, const spark_dsg::SceneGraphLayer& layer);

  void makeEdges(const LayerConfig& config, const spark_dsg::SceneGraphLayer& layer);

 private:
  Ogre::SceneManager* const manager_;
  Ogre::SceneNode* node_;

  std::unique_ptr<rviz::PointCloud> graph_nodes_;
  std::unique_ptr<rviz::BillboardLine> graph_edges_;
};

}  // namespace hydra
