#include "hydra/utils/mesh_deduplication.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {
namespace {

void compactMesh(spark_dsg::Mesh& mesh,
                 double tolerance,
                 std::vector<GlobalIndex>* face_voxels) {
  MeshVertexDeduplicator deduplicator(tolerance, mesh.numVertices() / 3);
  if (tolerance == 0 || mesh.numVertices() == 0) {
    return;
  }
  spark_dsg::Mesh compact(mesh.has_colors,
                          mesh.has_timestamps,
                          mesh.has_labels,
                          mesh.has_first_seen_stamps);
  compact.reserveVertices(mesh.numVertices() / 3);
  compact.faces.reserve(mesh.numFaces());
  std::vector<size_t> remapping(mesh.numVertices());
  for (size_t i = 0; i < mesh.numVertices(); ++i) {
    const auto index =
        deduplicator.add(mesh.pos(i), mesh.has_labels ? mesh.label(i) : 0);
    remapping[i] = index;
    if (index == compact.numVertices()) {
      compact.points.push_back(mesh.pos(i));
      if (mesh.has_colors) {
        compact.colors.push_back(mesh.colors[i]);
      }
      if (mesh.has_labels) {
        compact.labels.push_back(mesh.labels[i]);
      }
      if (mesh.has_timestamps) {
        compact.stamps.push_back(mesh.stamps[i]);
      }
      if (mesh.has_first_seen_stamps) {
        compact.first_seen_stamps.push_back(mesh.first_seen_stamps[i]);
      }
      continue;
    }
    if (mesh.has_timestamps) {
      compact.stamps[index] = std::max(compact.stamps[index], mesh.stamps[i]);
    }
    if (mesh.has_first_seen_stamps) {
      compact.first_seen_stamps[index] =
          std::min(compact.first_seen_stamps[index], mesh.first_seen_stamps[i]);
    }
  }
  std::vector<GlobalIndex> retained;
  for (size_t i = 0; i < mesh.faces.size(); ++i) {
    const auto& face = mesh.faces[i];
    const spark_dsg::Mesh::Face mapped{
        remapping.at(face[0]), remapping.at(face[1]), remapping.at(face[2])};
    if (mapped[0] != mapped[1] && mapped[0] != mapped[2] && mapped[1] != mapped[2]) {
      compact.faces.push_back(mapped);
      if (face_voxels) {
        retained.push_back(face_voxels->at(i));
      }
    }
  }
  mesh = std::move(compact);
  if (face_voxels) {
    *face_voxels = std::move(retained);
  }
}

}  // namespace

MeshVertexDeduplicator::MeshVertexDeduplicator(double tolerance,
                                               size_t expected_vertices)
    : tolerance_(tolerance), radius_squared_(tolerance * tolerance) {
  if (!std::isfinite(tolerance) || tolerance < 0 || !std::isfinite(radius_squared_) ||
      (tolerance > 0 && radius_squared_ == 0)) {
    throw std::invalid_argument("Invalid vertex merge tolerance");
  }
  representatives_.reserve(expected_vertices);
  if (tolerance > 0) {
    cells_.reserve(expected_vertices);
  }
}

size_t MeshVertexDeduplicator::CellHash::operator()(const Cell& cell) const {
  size_t result = 0;
  for (const auto value : cell) {
    result ^= std::hash<int64_t>{}(value) + 0x9e3779b9 + (result << 6) + (result >> 2);
  }
  return result;
}

MeshVertexDeduplicator::Cell MeshVertexDeduplicator::cellFor(
    const Eigen::Vector3f& point, uint32_t label) const {
  Cell cell{label, 0, 0, 0};
  for (size_t i = 0; i < 3; ++i) {
    const auto value = std::floor(static_cast<double>(point[i]) / tolerance_);
    // Leave room for neighboring-cell offsets and avoid undefined conversion.
    if (value <= static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        value >= static_cast<double>(std::numeric_limits<int64_t>::max())) {
      throw std::out_of_range("Vertex merge cell exceeds int64 range");
    }
    cell[i + 1] = static_cast<int64_t>(value);
  }
  return cell;
}

size_t MeshVertexDeduplicator::find(const Cell& cell,
                                    const Eigen::Vector3f& point) const {
  const auto it = cells_.find(cell);
  if (it == cells_.end()) {
    return representatives_.size();
  }

  for (const auto index : it->second) {
    const auto delta = point.cast<double>() - representatives_[index].cast<double>();
    if (delta.squaredNorm() <= radius_squared_) {
      return index;
    }
  }
  return representatives_.size();
}

size_t MeshVertexDeduplicator::add(const Eigen::Vector3f& point, uint32_t label) {
  if (!point.allFinite()) {
    throw std::invalid_argument("Cannot merge a nonfinite mesh vertex");
  }
  const auto next = representatives_.size();
  if (tolerance_ == 0) {
    const auto [it, inserted] =
        exact_.emplace(ExactKey{label, point.x(), point.y(), point.z()}, next);
    if (inserted) {
      representatives_.push_back(point);
    }
    return it->second;
  }

  const auto cell = cellFor(point, label);
  const auto same_cell = find(cell, point);
  if (same_cell != next) {
    return same_cell;
  }
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) {
          continue;
        }
        const auto match =
            find({cell[0], cell[1] + x, cell[2] + y, cell[3] + z}, point);
        if (match != next) {
          return match;
        }
      }
    }
  }
  cells_[cell].push_back(next);
  representatives_.push_back(point);
  return next;
}

void deduplicateMesh(spark_dsg::Mesh& mesh, double tolerance) {
  compactMesh(mesh, tolerance, nullptr);
}

void deduplicateMesh(MeshBlock& mesh, double tolerance) {
  if (!mesh.face_voxels.empty() && mesh.face_voxels.size() != mesh.numFaces()) {
    throw std::invalid_argument("Face voxel indices do not match mesh faces");
  }
  compactMesh(mesh, tolerance, mesh.face_voxels.empty() ? nullptr : &mesh.face_voxels);
}

}  // namespace hydra
