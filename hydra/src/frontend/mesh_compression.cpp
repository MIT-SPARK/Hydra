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
                                          const Eigen::Vector3f& pos) const {
  const auto voxel = map.getTsdfLayer().getVoxelPtr(pos);
  if (!voxel) {
    return false;
  }

  const auto valid_weight =
      std::isfinite(voxel->weight) && voxel->weight >= config_.min_weight;
  if (!valid_weight || !std::isfinite(voxel->distance)) {
    return false;
  }

  // A TSDF sample is located at the voxel center, not at the mesh vertex.
  // Allow for this displacement as well as movement within a compression cell.
  const auto cell_tolerance = 0.5 * map.config.voxel_size + config_.resolution;
  const auto default_clearance = std::sqrt(3.0) * cell_tolerance;
  const auto configured_clearance = config_.min_clearance_m;
  const auto clearance =
      configured_clearance > 0.0 ? configured_clearance : default_clearance;
  return voxel->distance > clearance;
}

GlobalIndex MeshCompression::compressionCell(const Eigen::Vector3f& pos) const {
  return spatial_hash::indexFromPoint<GlobalIndex>(pos, 1.0 / config_.resolution);
}

MeshCompression::UpdateState MeshCompression::prepareUpdate(
    const VolumetricMap& map) const {
  auto state = UpdateState{vertices_.size(),
                           faces_.size(),
                           std::vector<bool>(vertices_.size(), false),
                           std::vector<bool>(vertices_.size(), false),
                           {}};
  state.mutable_cells.reserve(vertices_.size());
  // An empty mesh block is not a clearing instruction. Only observed free space
  // can remove mutable geometry; frozen vertices support immutable faces.
  for (size_t i = 0; i < vertices_.size(); ++i) {
    const auto& entry = vertices_[i];
    if (entry.frozen) {
      continue;
    }

    state.deleted[i] = isObservedFreeSpace(map, entry.vertex.pos);
    state.mutable_cells.emplace(compressionCell(entry.vertex.pos), i);
  }

  return state;
}

void MeshCompression::integrateMeshBlock(const MeshBlock& block,
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

  for (const auto& face : block.faces) {
    const auto mapped = Face{remap.at(face[0]), remap.at(face[1]), remap.at(face[2])};
    if (isValidFace(mapped)) {
      faces_.push_back(mapped);
    }
  }
}

void MeshCompression::markReobservedFrozenVertices(UpdateState& state) const {
  for (size_t i = 0; i < state.previous_vertices; ++i) {
    const auto& entry = vertices_[i];
    if (!entry.frozen) {
      continue;
    }

    const auto iter = state.mutable_cells.find(compressionCell(entry.vertex.pos));
    if (iter != state.mutable_cells.end() && state.observed[iter->second]) {
      // Replace active topology through the new mutable endpoint. The frozen
      // endpoint itself remains unchanged for already archived faces.
      state.observed[i] = true;
    }
  }
}

void MeshCompression::removeClearedAndReplacedFaces(const UpdateState& state) {
  std::unordered_set<Face, kimera_pgmo::RedundancyChecker::FaceHash> new_faces;
  new_faces.reserve(faces_.size() - state.previous_faces);
  size_t retained = 0;
  for (size_t i = 0; i < faces_.size(); ++i) {
    const auto& face = faces_[i];
    // Refresh topology only when all endpoints were reobserved, preserving
    // triangles across partial updates, including those with frozen endpoints.
    const auto all_observed = std::all_of(
        face.begin(), face.end(), [&](auto index) { return state.observed[index]; });
    const auto replaced = i < state.previous_faces && all_observed;
    const auto cleared =
        state.deleted[face[0]] || state.deleted[face[1]] || state.deleted[face[2]];
    if (replaced || cleared) {
      continue;
    }

    // Retained faces are already unique. They have an unobserved endpoint, so
    // cannot duplicate incoming faces, whose endpoints are all observed.
    if (i >= state.previous_faces) {
      auto key = face;
      std::sort(key.begin(), key.end());
      if (!new_faces.insert(key).second) {
        continue;
      }
    }

    faces_[retained++] = face;
  }

  faces_.resize(retained);
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
  for (const auto& face : faces_) {
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
  const auto archived_count = delta.getNumArchivedVertices();
  for (const auto& face : faces_) {
    const auto mapped = Face{remap[face[0]], remap[face[1]], remap[face[2]]};
    const auto archive_face = std::any_of(
        face.begin(), face.end(), [&](auto index) { return archivable[index]; });
    delta.addFace(mapped, archive_face);
    if (archive_face) {
      // Remaining endpoints support immutable faces. Keep remapping them, but
      // never merge new geometry into them or clear them using future TSDF data.
      for (const auto i : face) {
        vertices_[i].frozen = true;
      }
    } else {
      faces_[active_count++] = {mapped[0] - archived_count,
                                mapped[1] - archived_count,
                                mapped[2] - archived_count};
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
  auto state = prepareUpdate(map);
  for (const auto& block : map.getMeshLayer()) {
    integrateMeshBlock(block, timestamp_ns, state);
  }

  markReobservedFrozenVertices(state);
  removeClearedAndReplacedFaces(state);
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
