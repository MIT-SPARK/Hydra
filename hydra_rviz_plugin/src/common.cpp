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

}  // namespace hydra
