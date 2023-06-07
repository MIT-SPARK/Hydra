#pragma once
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSphere.h>
#include <spark_dsg/scene_graph_node.h>

#include <functional>
#include <memory>

namespace spark_dsg {

class SceneGraphLayer;

}  // namespace spark_dsg

namespace hydra {

struct ColorFunctor {
  using Ptr = std::unique_ptr<ColorFunctor>;

  virtual void call(const spark_dsg::SceneGraphNode& node,
                    std::array<float, 3>& color) = 0;

  inline void operator()(const spark_dsg::SceneGraphNode& node,
                         std::array<float, 3>& color) {
    call(node, color);
  }
};

struct Pose {
  Pose(const Ogre::Quaternion& rot, const Ogre::Vector3& pos);

  Pose(const Ogre::Vector3& pos);

  Pose();

  static Pose XOffset(float offset);

  static Pose YOffset(float offset);

  static Pose ZOffset(float offset);

  Pose operator*(const Pose& rhs);

  Ogre::Quaternion rot;
  Ogre::Vector3 pos;
};

}  // namespace hydra
