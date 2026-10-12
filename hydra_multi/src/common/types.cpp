#include "hydra_multi/common/types.h"

#include <pcl/common/transforms.h>

namespace hydra_multi {

using spark_dsg::Mesh;

MeshData::MeshData() : mesh(new Mesh()) {}

MeshData::Ptr MeshData::clone() const {
  auto cloned = std::make_shared<MeshData>();
  cloned->mesh = mesh->clone();
  cloned->original = original;
  cloned->offsets = offsets;
  return cloned;
}

void MeshData::transform(const Eigen::Isometry3d& tf) {
  const Eigen::Isometry3f a_T_b = tf.cast<float>();
  mesh->transform(a_T_b);
  for (auto& v : original) {
    v.pos = (a_T_b * v.pos).eval();
  }
}

Timestamp MeshData::getLatestStamp() const { return original.back().stamp_ns; }

}  // namespace hydra_multi
