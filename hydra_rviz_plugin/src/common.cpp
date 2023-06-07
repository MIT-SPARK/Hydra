#include "hydra_rviz_plugin/common.h"

namespace hydra {

Pose::Pose(const Ogre::Quaternion& rot, const Ogre::Vector3& pos)
    : rot(rot), pos(pos) {}

Pose::Pose(const Ogre::Vector3& pos) : Pose(Ogre::Quaternion(), pos) {}

Pose::Pose() : Pose(Ogre::Quaternion(), Ogre::Vector3(0.0f, 0.0f, 0.0f)) {}

Pose Pose::operator*(const Pose& rhs) {
  return Pose(rot * rhs.rot, rot * rhs.pos + pos);
}

Pose Pose::XOffset(float offset) { return Pose(Ogre::Vector3(offset, 0.0f, 0.0f)); }

Pose Pose::YOffset(float offset) { return Pose(Ogre::Vector3(0.0f, offset, 0.0f)); }

Pose Pose::ZOffset(float offset) { return Pose(Ogre::Vector3(0.0f, 0.0f, offset)); }

LayerPair::LayerPair(LayerId layer1, LayerId layer2)
    : child(std::min(layer1, layer2)), parent(std::max(layer1, layer2)) {}

bool LayerPair::operator==(const LayerPair& other) const {
  return child == other.child && parent == other.parent;
}

bool LayerPair::operator<(const LayerPair& other) const {
  if (child == other.child) {
    return parent < other.parent;
  }

  return child < other.child;
}

std::ostream& operator<<(std::ostream& out, const LayerPair& pair) {
  return out << pair.parent << " -> " << pair.child;
}

}  // namespace hydra
