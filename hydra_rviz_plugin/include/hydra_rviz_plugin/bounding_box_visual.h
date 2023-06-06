#pragma once
#include <functional>
#include <memory>

#include "hydra_rviz_plugin/common.h"

namespace Ogre {

class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;

}  // namespace Ogre

namespace rviz {

class BillboardLine;

}  // namespace rviz

namespace hydra {

struct LayerConfig;

class BoundingBoxVisual {
 public:
  BoundingBoxVisual(Ogre::SceneManager* const manager, Ogre::SceneNode* const parent);

  virtual ~BoundingBoxVisual();

  void setMessage(const LayerConfig& config,
                  const spark_dsg::SceneGraphLayer& layer,
                  ColorFunctor* const color_callback);

  void setPose(const Ogre::Quaternion& rot, const Ogre::Vector3& pos);

 private:
  Ogre::SceneManager* const manager_;
  Ogre::SceneNode* node_;

  std::unique_ptr<rviz::BillboardLine> boxes_;
};

}  // namespace hydra
