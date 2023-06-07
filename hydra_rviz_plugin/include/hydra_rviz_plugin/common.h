#pragma once
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSphere.h>
#include <spark_dsg/scene_graph_node.h>

#include <functional>
#include <memory>

namespace spark_dsg {

class SceneGraphLayer;
class DynamicSceneGraphLayer;

}  // namespace spark_dsg

namespace hydra {

using spark_dsg::LayerId;

struct ColorFunctor {
  using Ptr = std::unique_ptr<ColorFunctor>;

  virtual void call(const spark_dsg::SceneGraphNode& node,
                    Ogre::ColourValue& color) = 0;

  inline void operator()(const spark_dsg::SceneGraphNode& node,
                         Ogre::ColourValue& color) {
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

struct LayerPair {
  LayerPair(LayerId layer1, LayerId layer2);

  bool operator==(const LayerPair& other) const;

  bool operator<(const LayerPair& other) const;

  LayerId child;
  LayerId parent;
};

std::ostream& operator<<(std::ostream& out, const LayerPair& pair);

}  // namespace hydra
