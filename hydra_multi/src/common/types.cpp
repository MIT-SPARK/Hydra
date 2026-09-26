#include "hydra_multi/common/types.h"

#include <pcl/common/transforms.h>

namespace hydra_multi {

MeshData::MeshData()
    : mesh(new Mesh),
      original_vertices(new pcl::PointCloud<pcl::PointXYZ>),
      vertex_stamps(new Timestamps) {}

Timestamps MeshData::getStamps() const {
  if (mesh->has_timestamps) {
    return mesh->stamps;
  }

  return *vertex_stamps;
}

MeshData::Ptr MeshData::clone() const {
  auto cloned = std::make_shared<MeshData>();
  cloned->mesh = mesh->clone();
  cloned->original_vertices.reset(
      new pcl::PointCloud<pcl::PointXYZ>(*original_vertices));
  cloned->vertex_stamps.reset(new Timestamps(*vertex_stamps));
  cloned->offsets = offsets;
  return cloned;
}

void MeshData::transform(const Eigen::Isometry3d& tf) {
  mesh->transform(tf.cast<float>());
  pcl::transformPointCloud(*original_vertices, *original_vertices, tf.cast<float>());
}

Timestamp MeshData::getLatestStamp() const { return getStamps().back(); }

}  // namespace hydra_multi
