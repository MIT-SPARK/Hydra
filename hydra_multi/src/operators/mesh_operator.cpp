#include "hydra_multi/operators/mesh_operator.h"

#include <glog/logging.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/pcl_mesh_traits.h>

#include <queue>

#include "hydra_multi/interface/utils.h"
namespace hydra_multi {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

bool MeshOperator::incrementalAppend(const MeshDelta& incremental_source) {
  // Appending new edges and nodes from source to data_
  incremental_source.updateMesh(*data_->mesh, data_->offsets, &data_T_orig_);
  kimera_pgmo::StampedCloud<pcl::PointXYZ> cloud_out{*data_->original_vertices,
                                                     *data_->vertex_stamps};
  incremental_source.updateVertices(cloud_out);
  return true;
}

bool MeshOperator::update(const MeshData& source) {
  // Updating the mesh in data_ according to source
  // Returns an error if data_ mesh vertices does not exist in source
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

  // Update the append transform
  updateAppendTransform();
  return true;
}

bool MeshOperator::rebase(const MeshData& source) {
  // Rebase by first finding overlapping parts (via timestamps), updating to source, and
  // then appending the non-overlapping data_ meshes. Note that we don't add vertices or
  // faces in source not in data_ to data_

  // Implicit assumption that the original vertices in source is same as data
  const auto latest_source_stamp = source.getLatestStamp();
  const auto stamps = data_->getStamps();

  Mesh new_mesh(*data_->mesh);
  PointCloud new_original_vertices;
  Timestamps new_vertex_stamps;
  std::unordered_set<size_t> vertices_to_erase;
  for (size_t i = 0; i < stamps.size(); i++) {
    if (stamps[i] <= latest_source_stamp) {
      vertices_to_erase.insert(i);
      continue;
    }
    auto original_pos = data_->original_vertices->at(i);
    new_mesh.setPos(new_original_vertices.size(),
                    {original_pos.x, original_pos.y, original_pos.z});
    new_original_vertices.push_back(original_pos);
    new_vertex_stamps.push_back(data_->vertex_stamps->at(i));
  }
  new_mesh.eraseVertices(vertices_to_erase);

  *data_->mesh = *source.mesh;
  *data_->vertex_stamps = *source.vertex_stamps;
  *data_->original_vertices = *source.original_vertices;

  updateAppendTransform();

  new_mesh.transform(data_T_orig_);
  data_->mesh->append(new_mesh);
  data_->vertex_stamps->insert(
      data_->vertex_stamps->end(), new_vertex_stamps.begin(), new_vertex_stamps.end());
  data_->original_vertices->points.insert(data_->original_vertices->points.end(),
                                          new_original_vertices.points.begin(),
                                          new_original_vertices.points.end());
  return true;
}

bool MeshOperator::merge(const MeshData& source) {
  // To merge find the new parts of source and append
  const auto latest_stamp = data_->getLatestStamp();
  const auto source_stamps = source.getStamps();

  Mesh new_source_mesh(*source.mesh);
  PointCloud new_original_vertices;
  Timestamps new_vertex_stamps;
  std::unordered_set<size_t> vertices_to_erase;

  // TODO(Yun) make this a param
  size_t num_vertices = 9;
  std::queue<Eigen::Vector3f> source_pts_queue;
  std::queue<Eigen::Vector3f> pts_queue;
  for (size_t i = 0; i < source_stamps.size(); i++) {
    if (source_stamps[i] <= latest_stamp) {
      vertices_to_erase.insert(i);
      source_pts_queue.push(source.mesh->pos(i));
      pts_queue.push(data_->mesh->pos(i));
      if (source_pts_queue.size() > num_vertices) {
        source_pts_queue.pop();
        pts_queue.pop();
      }

      continue;
    }

    new_original_vertices.push_back(source.original_vertices->at(i));
    new_vertex_stamps.push_back(source.vertex_stamps->at(i));
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
  Eigen::Isometry3f source_T_data;
  source_T_data = source_T_data_d.cast<float>();

  // TODO(Yun) Similar to append, here missing a transform of the new mesh
  new_source_mesh.transform(source_T_data);
  data_->mesh->append(new_source_mesh);
  data_->vertex_stamps->insert(
      data_->vertex_stamps->end(), new_vertex_stamps.begin(), new_vertex_stamps.end());
  data_->original_vertices->points.insert(data_->original_vertices->points.end(),
                                          new_original_vertices.points.begin(),
                                          new_original_vertices.points.end());

  data_->offsets = source.offsets;
  return true;
}

void MeshOperator::updateAppendTransform() {
  // Find transform from original to mesh
  const auto num_points = data_->mesh->numVertices();
  // TODO(Yun) make this a param
  size_t num_vertices = 9;
  std::vector<Eigen::Vector3d> pts;
  std::vector<Eigen::Vector3d> data_pts;

  for (size_t i = num_points - num_vertices; i < num_points; i++) {
    const auto orig_pt = data_->original_vertices->points[i];
    pts.push_back({orig_pt.x, orig_pt.y, orig_pt.z});
    data_pts.push_back(data_->mesh->pos(i).cast<double>());
  }

  Eigen::Isometry3d data_T_orig;
  estimateRigidTransformSVD(pts, data_pts, data_T_orig);
  data_T_orig_ = data_T_orig.cast<float>();
}

}  // namespace hydra_multi
