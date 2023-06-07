#pragma once
#include <functional>
#include <memory>

#include "hydra_rviz_plugin/common.h"

namespace Ogre {

class SceneManager;
class SceneNode;

}  // namespace Ogre

namespace rviz {

class PointCloud;
class BillboardLine;

}  // namespace rviz

namespace hydra {

struct LayerConfig;

class LayerVisual {
 public:
  LayerVisual(Ogre::SceneManager* const manager, Ogre::SceneNode* const parent);

  virtual ~LayerVisual();

  void setMessage(const LayerConfig& config,
                  const spark_dsg::SceneGraphLayer& msg,
                  ColorFunctor* const color_callback);

  void setPose(const Pose& pose);

  void makeNodes(const LayerConfig& config,
                 const spark_dsg::SceneGraphLayer& layer,
                 ColorFunctor* const color_callback);

  void makeEdges(const LayerConfig& config, const spark_dsg::SceneGraphLayer& layer);

 private:
  Ogre::SceneManager* const manager_;
  Ogre::SceneNode* node_;

  std::unique_ptr<rviz::PointCloud> graph_nodes_;
  std::unique_ptr<rviz::BillboardLine> graph_edges_;
};

}  // namespace hydra
