#include "hydra_multi/operators/mesh_operator.h"

#include <glog/logging.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo/mesh_delta.h>

#include <queue>

#include "hydra_multi/interface/utils.h"

namespace hydra_multi {

using spark_dsg::Mesh;

bool MeshOperator::incrementalAppend(const MeshDelta& incremental_source) {
  incremental_source.updateMesh(*data_->mesh, data_->offsets, &data_T_orig_);
  incremental_source.updateVertices(data_->original);
  return true;
}

// Updating the mesh in data_ according to source
// Returns an error if data_ mesh vertices does not exist in source
bool MeshOperator::update(const MeshData& source) {
  if (data_->mesh->numVertices() != source.mesh->numVertices()) {
    return false;
  }

  if (data_->mesh->numFaces() != source.mesh->numFaces()) {
    return false;
  }

  const auto num_vertices = source.mesh->numVertices();
  for (size_t i = 0; i < num_vertices; i++) {
    data_->mesh->setPos(i, source.mesh->pos(i));
    if (source.mesh->has_colors && data_->mesh->has_colors) {
      data_->mesh->setColor(i, source.mesh->color(i));
    }

    if (source.mesh->has_labels && data_->mesh->has_labels) {
      data_->mesh->setLabel(i, source.mesh->label(i));
    }
  }

  updateAppendTransform();
  return true;
}

// Rebase by first finding overlapping parts (via timestamps), updating to source, and
// then appending the non-overlapping data_ meshes. Note that we don't add vertices or
// faces in source not in data_ to data_
// Implicit assumption that the original vertices in source is same as data
bool MeshOperator::rebase(const MeshData& source) {
  const auto latest_source_stamp = source.getLatestStamp();

  Mesh new_mesh(*data_->mesh);
  hydra::StampedMeshVertices points;
  points.reserve(data_->original.size());

  std::unordered_set<size_t> vertices_to_erase;
  for (size_t i = 0; i < data_->original.size(); ++i) {
    const auto& v = data_->original[i];
    if (v.stamp_ns <= latest_source_stamp) {
      vertices_to_erase.insert(i);
      continue;
    }

    new_mesh.setPos(points.size(), v.pos);
    points.push_back(v);
  }

  new_mesh.eraseVertices(vertices_to_erase);

  *data_->mesh = *source.mesh;
  data_->original = source.original;

  updateAppendTransform();

  new_mesh.transform(data_T_orig_);
  data_->mesh->append(new_mesh);
  data_->original.insert(data_->original.end(), points.begin(), points.end());
  return true;
}

bool MeshOperator::merge(const MeshData& source) {
  // To merge find the new parts of source and append
  const auto latest_stamp = data_->getLatestStamp();

  Mesh new_source_mesh(*source.mesh);
  hydra::StampedMeshVertices new_points;
  std::unordered_set<size_t> vertices_to_erase;

  // TODO(Yun) make this a param
  size_t num_vertices = 9;
  std::queue<Eigen::Vector3f> source_pts_queue;
  std::queue<Eigen::Vector3f> pts_queue;
  for (size_t i = 0; i < source.original.size(); i++) {
    if (source.original[i].stamp_ns <= latest_stamp) {
      vertices_to_erase.insert(i);
      source_pts_queue.push(source.mesh->pos(i));
      pts_queue.push(data_->mesh->pos(i));
      if (source_pts_queue.size() > num_vertices) {
        source_pts_queue.pop();
        pts_queue.pop();
      }

      continue;
    }

    new_points.push_back(source.original.at(i));
  }

  new_source_mesh.eraseVertices(vertices_to_erase);

  Eigen::Isometry3d source_T_data_d;
  std::vector<Eigen::Vector3d> source_pts;
  std::vector<Eigen::Vector3d> pts;
  for (size_t i = 0; i < num_vertices; i++) {
    source_pts.push_back(source_pts_queue.front().cast<double>());
    pts.push_back(pts_queue.front().cast<double>());
    source_pts_queue.pop();
    pts_queue.pop();
  }
  estimateRigidTransformSVD(source_pts, pts, source_T_data_d);
  Eigen::Isometry3f source_T_data = source_T_data_d.cast<float>();

  // TODO(Yun) Similar to append, here missing a transform of the new mesh
  new_source_mesh.transform(source_T_data);
  data_->mesh->append(new_source_mesh);
  data_->original.insert(data_->original.end(), new_points.begin(), new_points.end());
  data_->offsets = source.offsets;
  return true;
}

// Find transform from original to mesh
void MeshOperator::updateAppendTransform() {
  // TODO(Yun) make this a param
  size_t num_vertices = 9;
  std::vector<Eigen::Vector3d> pts;
  std::vector<Eigen::Vector3d> data_pts;
  const auto num_points = data_->mesh->numVertices();
  for (size_t i = num_points - num_vertices; i < num_points; i++) {
    pts.push_back(data_->original[i].pos.cast<double>());
    data_pts.push_back(data_->mesh->pos(i).cast<double>());
  }

  Eigen::Isometry3d data_T_orig;
  estimateRigidTransformSVD(pts, data_pts, data_T_orig);
  data_T_orig_ = data_T_orig.cast<float>();
}

}  // namespace hydra_multi
