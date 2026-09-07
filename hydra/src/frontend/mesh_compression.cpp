/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/frontend/mesh_compression.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <kimera_pgmo/compression/redundancy_checker.h>
#include <spatial_hash/grid.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <unordered_set>

#include "hydra/active_window/active_window_output.h"
#include "hydra/active_window/volumetric_window.h"
#include "hydra/utils/pgmo_mesh_traits.h"

namespace hydra {
namespace {

const auto registration =
    config::RegistrationWithConfig<MeshCompressor,
                                   MeshCompression,
                                   MeshCompression::Config>("MeshCompression");

constexpr auto invalid_index = std::numeric_limits<size_t>::max();

bool isValidFace(const kimera_pgmo::traits::Face& face) {
  const auto valid_indices =
      std::find(face.begin(), face.end(), invalid_index) == face.end();
  const auto distinct_indices =
      face[0] != face[1] && face[1] != face[2] && face[2] != face[0];
  return valid_indices && distinct_indices;
}

void updateVertexAttributes(MeshCompression::Vertex& vertex,
                            const Eigen::Vector3f& pos,
                            const kimera_pgmo::traits::VertexTraits& traits,
                            uint64_t timestamp_ns) {
  vertex.pos = pos;
  auto& dest = vertex.traits;
  dest.properties |= traits.properties;
  dest.properties.has_stamp = true;
  dest.properties.has_first_seen_stamp = true;
  const auto stamp = traits.properties.has_stamp ? traits.stamp : timestamp_ns;
  const auto first_seen =
      traits.properties.has_first_seen_stamp ? traits.first_seen_stamp : timestamp_ns;
  dest.stamp = std::max(dest.stamp, stamp);
  dest.first_seen_stamp = std::min(dest.first_seen_stamp, first_seen);
  if (traits.properties.has_color) {
    dest.color = traits.color;
  }

  if (traits.properties.has_label) {
    dest.label = traits.label;
  }
}

}  // namespace

void declare_config(MeshCompression::Config& config) {
  using namespace config;
  name("MeshCompression::Config");
  field(config.resolution, "resolution", "m");
  field(config.min_weight, "min_weight");
  field(config.min_clearance_m, "min_clearance_m", "m");
  field(config.clear_free_space, "clear_free_space");
  field(config.replace_reobserved_cells, "replace_reobserved_cells");
  check(config.resolution, GT, 0.0, "resolution");
  check(config.min_weight, GT, 0.0f, "min_weight");
  check(config.min_clearance_m, GE, 0.0, "min_clearance_m");
  checkCondition(std::isfinite(config.resolution), "resolution must be finite");
  checkCondition(std::isfinite(config.min_weight), "min_weight must be finite");
  checkCondition(std::isfinite(config.min_clearance_m),
                 "min_clearance_m must be finite");
}

MeshCompression::MeshCompression(double resolution)
    : MeshCompression(Config{resolution}) {}

MeshCompression::MeshCompression(const Config& config)
    : config_(config::checkValid(config)) {}

bool MeshCompression::isObservedFreeSpace(const VolumetricMap& map,
                                          const GlobalIndex& cell) const {
  const auto clearance =
      config_.min_clearance_m > 0.0 ? config_.min_clearance_m : config_.resolution;
  for (int corner = 0; corner < 8; ++corner) {
    const GlobalIndex offset(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1);
    const GlobalIndex index = cell + offset;
    const auto voxel = map.getTsdfLayer().getVoxelPtr(index);
    if (!voxel) {
      return false;
    }

    const auto observed =
        std::isfinite(voxel->weight) && voxel->weight >= config_.min_weight;
    const auto free = std::isfinite(voxel->distance) && voxel->distance > clearance;
    if (!observed || !free) {
      return false;
    }
  }

  return true;
}

GlobalIndex MeshCompression::compressionCell(const Eigen::Vector3f& pos) const {
  return spatial_hash::indexFromPoint<GlobalIndex>(pos, 1.0 / config_.resolution);
}

MeshCompression::UpdateState MeshCompression::initializeUpdate(
    const VolumetricMap& map) {
  for (const auto& block : map.getMeshLayer()) {
    if (block.face_cells.size() != block.faces.size()) {
      throw std::invalid_argument(
          "MeshCompression requires marching-cubes cell provenance");
    }
  }

  removals_.clear();
  diagnostic_positions_.clear();
  if (diagnostics_enabled_) {
    for (const auto& entry : vertices_) {
      diagnostic_positions_.push_back(entry.vertex.pos);
    }
  }

  return prepareUpdate();
}

MeshCompression::UpdateState MeshCompression::prepareUpdate() const {
  auto state = UpdateState{vertices_.size(),
                           faces_.size(),
                           std::vector<bool>(vertices_.size(), false),
                           std::vector<bool>(vertices_.size(), false),
                           {},
                           {},
                           {}};
  state.mutable_cells.reserve(vertices_.size());
  // An empty mesh block is not a clearing instruction. Only observed free space
  // can remove mutable geometry; frozen vertices support immutable faces.
  for (size_t i = 0; i < vertices_.size(); ++i) {
    const auto& entry = vertices_[i];
    if (entry.frozen) {
      continue;
    }

    state.mutable_cells.emplace(compressionCell(entry.vertex.pos), i);
  }

  return state;
}

std::vector<size_t> MeshCompression::integrateVertices(const MeshBlock& block,
                                                       uint64_t timestamp_ns,
                                                       UpdateState& state) {
  std::vector<size_t> remap(block.numVertices(), invalid_index);
  for (size_t i = 0; i < block.numVertices(); ++i) {
    kimera_pgmo::traits::VertexTraits traits;
    traits.properties = spark_dsg::pgmoGetVertexProperties(block);
    const auto pos = spark_dsg::pgmoGetVertex(block, i, &traits);
    if (!pos.allFinite()) {
      continue;
    }

    const auto cell = compressionCell(pos);
    const auto [iter, inserted] = state.mutable_cells.emplace(cell, vertices_.size());
    if (inserted) {
      vertices_.emplace_back();
      state.deleted.push_back(false);
      state.observed.push_back(false);
    }

    remap[i] = iter->second;
    state.observed[iter->second] = true;
    // Marching cubes observes an interpolated zero crossing. A nearby positive
    // projective TSDF sample must not clear that currently observed surface.
    state.deleted[iter->second] = false;
    updateVertexAttributes(vertices_[iter->second].vertex, pos, traits, timestamp_ns);
  }

  return remap;
}

void MeshCompression::integrateMeshBlock(const VolumetricMap& map,
                                         const MeshBlock& block,
                                         uint64_t timestamp_ns,
                                         UpdateState& state) {
  const auto remap = integrateVertices(block, timestamp_ns, state);
  for (size_t i = 0; i < block.faces.size(); ++i) {
    const auto& face = block.faces[i];
    const auto& cell = block.face_cells[i];
    const auto mapped = Face{remap.at(face[0]), remap.at(face[1]), remap.at(face[2])};
    const auto finite =
        std::find(mapped.begin(), mapped.end(), invalid_index) == mapped.end();
    if (finite) {
      state.reobserved_cells.insert(cell);
    }

    if (isValidFace(mapped)) {
      faces_.push_back({mapped, cell});
    } else if (diagnostics_enabled_) {
      recordRemovalPoints(
          map,
          cell,
          {block.points[face[0]], block.points[face[1]], block.points[face[2]]},
          "degeneracy");
    }
  }
}

void MeshCompression::removeClearedAndReplacedFaces(const VolumetricMap& map,
                                                    UpdateState& state) {
  size_t retained = 0;
  for (size_t i = 0; i < faces_.size(); ++i) {
    const auto& face = faces_[i];
    if (i < state.previous_faces) {
      const auto replaced =
          config_.replace_reobserved_cells && state.reobserved_cells.count(face.cell);
      if (replaced) {
        recordRemoval(map, face, "replacement");
        continue;
      }

      const auto [iter, inserted] = state.cleared_cells.emplace(face.cell, false);
      if (inserted && config_.clear_free_space) {
        iter->second = isObservedFreeSpace(map, face.cell);
      }

      if (iter->second) {
        recordRemoval(map, face, "free_space");
        continue;
      }
    }

    faces_[retained++] = face;
  }

  faces_.resize(retained);
}

void MeshCompression::enableDiagnostics(bool enabled) {
  diagnostics_enabled_ = enabled;
  removals_.clear();
}

void MeshCompression::recordRemoval(const VolumetricMap& map,
                                    const CellFace& face,
                                    const char* reason) {
  if (!diagnostics_enabled_) {
    return;
  }

  recordRemovalPoints(map,
                      face.cell,
                      {diagnostic_positions_[face.vertices[0]],
                       diagnostic_positions_[face.vertices[1]],
                       diagnostic_positions_[face.vertices[2]]},
                      reason);
}

void MeshCompression::recordRemovalPoints(const VolumetricMap& map,
                                          const GlobalIndex& cell,
                                          const std::array<Eigen::Vector3f, 3>& points,
                                          const char* reason) {
  auto& entry = removals_.emplace_back();
  entry.cell = cell;
  entry.points = points;
  entry.reason = reason;
  const auto unknown = std::numeric_limits<float>::quiet_NaN();
  for (int corner = 0; corner < 8; ++corner) {
    const GlobalIndex offset(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1);
    const GlobalIndex index = cell + offset;
    const auto voxel = map.getTsdfLayer().getVoxelPtr(index);
    entry.distances[corner] = voxel ? voxel->distance : unknown;
    entry.weights[corner] = voxel ? voxel->weight : unknown;
  }
}

void MeshCompression::saveDiagnostics(const std::filesystem::path& output) const {
  std::filesystem::create_directories(output);
  std::ofstream trace(output / "removed.csv");
  trace << "face,reason,cell_x,cell_y,cell_z";
  for (size_t i = 0; i < 8; ++i) {
    trace << ",distance_" << i << ",weight_" << i;
  }

  trace << "\n";
  spark_dsg::Mesh mesh(true, false, false, false);
  for (const auto& entry : removals_) {
    trace << mesh.numFaces() << ',' << entry.reason << ',' << entry.cell.x() << ','
          << entry.cell.y() << ',' << entry.cell.z();
    for (size_t i = 0; i < 8; ++i) {
      trace << ',' << entry.distances[i] << ',' << entry.weights[i];
    }

    trace << "\n";
    const auto offset = mesh.numVertices();
    mesh.points.insert(mesh.points.end(), entry.points.begin(), entry.points.end());
    auto color = spark_dsg::Color(0, 0, 255);
    if (entry.reason == "free_space") {
      color = spark_dsg::Color(255, 0, 0);
    } else if (entry.reason == "degeneracy") {
      color = spark_dsg::Color(255, 255, 0);
    }

    mesh.colors.insert(mesh.colors.end(), 3, color);
    mesh.faces.push_back({offset, offset + 1, offset + 2});
  }

  mesh.save(output / "removed.sparkdsg");
}

void MeshCompression::findUnusedVertices(UpdateState& state) const {
  std::fill(state.deleted.begin(), state.deleted.end(), true);
  for (const auto& face : faces_) {
    for (const auto index : face.vertices) {
      state.deleted[index] = false;
    }
  }

  for (size_t i = 0; i < vertices_.size(); ++i) {
    if (vertices_[i].frozen) {
      state.deleted[i] = false;
    }
  }
}

std::vector<bool> MeshCompression::findArchivableVertices(
    const UpdateState& state, const ArchivePredicate& archive) const {
  std::vector<bool> outside(vertices_.size(), false);
  for (size_t i = 0; i < vertices_.size(); ++i) {
    if (state.deleted[i]) {
      continue;
    }

    if (vertices_[i].frozen) {
      outside[i] = true;
      continue;
    }

    const auto unobserved = !state.observed[i];
    outside[i] = unobserved && archive && archive(vertices_[i].vertex);
  }

  auto archivable = outside;
  for (const auto& entry : faces_) {
    const auto& face = entry.vertices;
    if (outside[face[0]] && outside[face[1]] && outside[face[2]]) {
      continue;
    }

    for (const auto i : face) {
      archivable[i] = false;
    }
  }

  return archivable;
}

std::vector<size_t> MeshCompression::appendDeltaVertices(
    const UpdateState& state,
    const std::vector<bool>& archivable,
    kimera_pgmo::MeshDelta& delta) const {
  std::vector<size_t> remap(vertices_.size(), invalid_index);
  // MeshDelta requires newly archived vertices before all active vertices.
  for (const auto archiving : {true, false}) {
    for (size_t i = 0; i < vertices_.size(); ++i) {
      if (state.deleted[i] || archivable[i] != archiving) {
        continue;
      }

      const auto& vertex = vertices_[i].vertex;
      remap[i] = delta.addVertex(vertex.pos, vertex.traits, archiving);
    }
  }

  // Previous indices are ordered, allowing linear-time insertion into the map.
  auto& previous = *delta.info.prev_to_curr;
  for (size_t i = 0; i < state.previous_vertices; ++i) {
    if (!state.deleted[i]) {
      previous.emplace_hint(previous.end(), i, remap[i]);
    }
  }

  return remap;
}

void MeshCompression::appendDeltaFaces(const std::vector<bool>& archivable,
                                       const std::vector<size_t>& remap,
                                       kimera_pgmo::MeshDelta& delta) {
  size_t active_count = 0;
  std::unordered_set<Face, kimera_pgmo::RedundancyChecker::FaceHash> unique_faces;
  unique_faces.reserve(faces_.size());
  const auto archived_count = delta.getNumArchivedVertices();
  for (const auto& entry : faces_) {
    const auto& face = entry.vertices;
    const auto mapped = Face{remap[face[0]], remap[face[1]], remap[face[2]]};
    const auto archive_face = std::any_of(
        face.begin(), face.end(), [&](auto index) { return archivable[index]; });
    auto key = mapped;
    std::sort(key.begin(), key.end());
    if (unique_faces.insert(key).second) {
      delta.addFace(mapped, archive_face);
    }

    if (archive_face) {
      // Remaining endpoints support immutable faces. Keep remapping them, but
      // never merge new geometry into them or clear them using future TSDF data.
      for (const auto i : face) {
        vertices_[i].frozen = true;
      }
    } else {
      faces_[active_count++] = {{mapped[0] - archived_count,
                                 mapped[1] - archived_count,
                                 mapped[2] - archived_count},
                                entry.cell};
    }
  }

  faces_.resize(active_count);
}

void MeshCompression::retainActiveVertices(const UpdateState& state,
                                           const std::vector<bool>& archivable) {
  size_t remaining = 0;
  for (size_t i = 0; i < vertices_.size(); ++i) {
    if (!state.deleted[i] && !archivable[i]) {
      vertices_[remaining++] = vertices_[i];
    }
  }

  vertices_.resize(remaining);
}

void MeshCompression::updateTracking(const kimera_pgmo::MeshDelta& delta) {
  tracking_.prev_active_vertices = delta.getNumActiveVertices();
  tracking_.prev_active_faces = delta.getNumActiveFaces();
  if (++tracking_.sequence_number == 0) {
    tracking_.sequence_number = 1;
  }
}

kimera_pgmo::MeshDelta::Ptr MeshCompression::update(const VolumetricMap& map,
                                                    uint64_t timestamp_ns,
                                                    const ArchivePredicate& archive) {
  auto state = initializeUpdate(map);
  for (const auto& block : map.getMeshLayer()) {
    integrateMeshBlock(map, block, timestamp_ns, state);
  }

  removeClearedAndReplacedFaces(map, state);
  findUnusedVertices(state);
  const auto archivable = findArchivableVertices(state, archive);

  const auto info =
      kimera_pgmo::MeshDelta::TrackingInfo::with_remap(tracking_.sequence_number,
                                                       tracking_.prev_active_vertices,
                                                       tracking_.prev_active_faces);
  auto delta = std::make_unique<kimera_pgmo::MeshDelta>(info);
  const auto remap = appendDeltaVertices(state, archivable, *delta);
  appendDeltaFaces(archivable, remap, *delta);
  retainActiveVertices(state, archivable);
  delta->timestamp_ns = timestamp_ns;
  updateTracking(*delta);
  return delta;
}

kimera_pgmo::MeshDelta::Ptr MeshCompression::update(const ActiveWindowOutput& input,
                                                    const VolumetricWindow* window) {
  if (!window) {
    return update(input.map(), input.timestamp_ns);
  }

  const auto world_T_body = input.world_T_body();
  return update(input.map(), input.timestamp_ns, [&](const Vertex& vertex) {
    return !window->inBounds(input.timestamp_ns,
                             world_T_body,
                             vertex.traits.stamp,
                             vertex.pos.cast<double>());
  });
}

}  // namespace hydra
