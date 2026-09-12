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

#include <cmath>
#include <limits>

#include "hydra/active_window/active_window_output.h"
#include "hydra/active_window/volumetric_window.h"
#include "hydra/utils/pgmo_mesh_traits.h"

using kimera_pgmo::MeshDelta;

namespace hydra {
namespace {

const auto registration =
    config::RegistrationWithConfig<MeshCompressor,
                                   MeshCompression,
                                   MeshCompression::Config>("MeshCompression");

static constexpr auto INVALID = std::numeric_limits<size_t>::max();

inline bool isFiniteFace(const kimera_pgmo::traits::Face& face) {
  return face[0] != INVALID && face[1] != INVALID && face[2] != INVALID;
}

inline bool isDistinctFace(const kimera_pgmo::traits::Face& face) {
  return face[0] != face[1] && face[1] != face[2] && face[2] != face[0];
}

inline bool canArchiveFace(const kimera_pgmo::traits::Face& face,
                           const std::vector<bool>& can_archive) {
  return can_archive[face[0]] || can_archive[face[1]] || can_archive[face[2]];
}

inline kimera_pgmo::traits::Face remapFace(const kimera_pgmo::traits::Face& face,
                                           const std::vector<size_t>& remap) {
  return {remap[face[0]], remap[face[1]], remap[face[2]]};
}

inline kimera_pgmo::traits::Face offsetFace(const kimera_pgmo::traits::Face& face,
                                            size_t offset) {
  return {face[0] - offset, face[1] - offset, face[2] - offset};
}

inline void addVerticesToDelta(const std::vector<MeshCompression::Entry>& vertices,
                               const std::vector<bool>& deleted,
                               const std::vector<bool>& can_archive,
                               MeshDelta& delta,
                               std::vector<size_t>& remap,
                               bool should_archive) {
  for (size_t i = 0; i < vertices.size(); ++i) {
    if (deleted[i] || can_archive[i] != should_archive) {
      continue;
    }

    const auto& vertex = vertices[i];
    remap[i] = delta.addVertex(vertex.pos, vertex.traits, should_archive);
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
  checkCondition(std::isfinite(config.resolution), "resolution not finite");
  checkCondition(std::isfinite(config.min_weight), "min_weight not finite");
  checkCondition(std::isfinite(config.min_clearance_m), "min_clearance_m not finite");
}

MeshCompression::UpdateState::UpdateState(size_t num_vertices, size_t num_faces)
    : previous_vertices(num_vertices),
      previous_faces(num_faces),
      observed(num_vertices, false) {
  active.reserve(num_vertices);
}

MeshCompression::MeshCompression(double resolution)
    : MeshCompression(Config{resolution}) {}

MeshCompression::MeshCompression(const Config& config)
    : config(config::checkValid(config)),
      min_clearance(config.min_clearance_m > 0.0 ? config.min_clearance_m
                                                 : config.resolution),
      inv_resolution(1.0 / config.resolution),
      tracking_(1) {}

bool MeshCompression::isFree(const VolumetricMap& map, const GlobalIndex& cell) const {
  for (size_t corner = 0; corner < 8; ++corner) {
    const GlobalIndex offset(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1);
    const GlobalIndex index = cell + offset;
    const auto voxel = map.getTsdfLayer().getVoxelPtr(index);
    if (!voxel) {
      return false;
    }

    if (!std::isfinite(voxel->weight) || voxel->weight < config.min_weight) {
      return false;
    }

    if (!std::isfinite(voxel->distance) || voxel->distance <= min_clearance) {
      return false;
    }
  }

  return true;
}

GlobalIndex MeshCompression::compressedIndex(const Eigen::Vector3f& pos) const {
  return spatial_hash::indexFromPoint<GlobalIndex>(pos, inv_resolution);
}

auto MeshCompression::prepare(const VolumetricMap& map) const -> UpdateState {
  for (const auto& block : map.getMeshLayer()) {
    if (block.face_voxels.size() != block.faces.size()) {
      throw std::invalid_argument("face voxel indices required for mesh compression");
    }
  }

  UpdateState state(vertices_.size(), faces_.size());
  for (size_t i = 0; i < vertices_.size(); ++i) {
    const auto& entry = vertices_[i];
    if (entry.frozen) {
      continue;
    }

    state.active.emplace(compressedIndex(entry.pos), i);
  }

  return state;
}

void MeshCompression::integrate(const MeshBlock& block,
                                const UpdateCallback& merge,
                                UpdateState& state) {
  const auto properties = spark_dsg::pgmoGetVertexProperties(block);

  // assign uncompressed vertices to appropriate voxels and record remapping
  std::vector<size_t> remap(block.numVertices(), INVALID);
  for (size_t i = 0; i < block.numVertices(); ++i) {
    kimera_pgmo::traits::VertexTraits traits;
    traits.properties = properties;
    const auto pos = spark_dsg::pgmoGetVertex(block, i, &traits);
    if (!pos.allFinite()) {
      continue;
    }

    const auto index = compressedIndex(pos);
    const auto [iter, inserted] = state.active.emplace(index, vertices_.size());
    if (inserted) {
      vertices_.emplace_back();
      state.observed.push_back(false);
    }

    remap[i] = iter->second;
    state.observed[iter->second] = true;
    merge(pos, traits, vertices_[iter->second]);
  }

  // remap block faces and record face voxels
  for (size_t i = 0; i < block.faces.size(); ++i) {
    const auto& face = block.faces[i];
    const auto& index = block.face_voxels[i];

    const Face mapped{remap.at(face[0]), remap.at(face[1]), remap.at(face[2])};
    const auto finite = isFiniteFace(mapped);
    if (finite) {
      state.reobserved.insert(index);
    }

    if (finite && isDistinctFace(mapped)) {
      faces_.push_back({mapped, index});
    }
  }
}

void MeshCompression::prune(const VolumetricMap& map, UpdateState& state) {
  size_t retained = 0;
  GlobalIndexMap<bool> cleared_cells;
  for (size_t i = 0; i < faces_.size(); ++i) {
    const auto& face = faces_[i];
    if (i < state.previous_faces) {
      const auto replaced = state.reobserved.count(face.voxel);
      if (replaced) {
        continue;  // drop faces if parent voxel was updated during marching cubes
      }

      const auto [iter, inserted] = cleared_cells.emplace(face.voxel, false);
      if (inserted) {
        // check if parent voxel is in freespace for first face in parent voxel
        iter->second = isFree(map, face.voxel);
      }

      if (iter->second) {
        continue;  // skip any faces where parent voxel is in freespace
      }
    }

    faces_[retained] = face;
    ++retained;
  }

  faces_.resize(retained);

  // mark any vertices that have no faces pointing at them
  state.deleted.assign(vertices_.size(), true);
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

MeshDelta::Ptr MeshCompression::makeDelta(const UpdateState& state,
                                          uint64_t timestamp_ns,
                                          const ArchivePredicate& archive) {
  tracking_.prev_to_curr = std::make_shared<std::map<size_t, size_t>>();
  auto result = std::make_unique<MeshDelta>(tracking_);
  result->timestamp_ns = timestamp_ns;

  // mark candidate archival vertices
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
    const auto& vertex = vertices_[i];
    outside[i] = unobserved && archive && archive({vertex.pos, vertex.traits});
  }

  // update archivable vertices with face boundary information
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

  // add archived then active vertices to delta and fill remapping
  auto& previous = *result->info.prev_to_curr;
  std::vector<size_t> remap(vertices_.size(), INVALID);
  addVerticesToDelta(vertices_, state.deleted, archivable, *result, remap, true);
  addVerticesToDelta(vertices_, state.deleted, archivable, *result, remap, false);
  for (size_t i = 0; i < state.previous_vertices; ++i) {
    if (!state.deleted[i]) {
      previous.emplace_hint(previous.end(), i, remap[i]);
    }
  }

  size_t active_count = 0;
  kimera_pgmo::RedundancyChecker face_checker(faces_.size());
  const auto archived_count = result->getNumArchivedVertices();
  for (const auto& entry : faces_) {
    const auto archive_face = canArchiveFace(entry.vertices, archivable);
    const auto mapped = remapFace(entry.vertices, remap);
    if (face_checker.tryAdd(mapped)) {
      result->addFace(mapped, archive_face);
    }

    if (archive_face) {
      // freeze any vertices that support archived faces
      vertices_[entry.vertices[0]].frozen = true;
      vertices_[entry.vertices[1]].frozen = true;
      vertices_[entry.vertices[2]].frozen = true;
    } else {
      faces_[active_count] = {offsetFace(mapped, archived_count), entry.voxel};
      ++active_count;
    }
  }

  faces_.resize(active_count);

  size_t remaining = 0;
  for (size_t i = 0; i < vertices_.size(); ++i) {
    if (!state.deleted[i] && !archivable[i]) {
      vertices_[remaining++] = vertices_[i];
    }
  }

  vertices_.resize(remaining);

  tracking_.prev_active_vertices = result->getNumActiveVertices();
  tracking_.prev_active_faces = result->getNumActiveFaces();
  ++tracking_.sequence_number;
  if (tracking_.sequence_number == 0) {
    tracking_.sequence_number = 1;
  }

  return result;
}

auto MeshCompression::update(const ActiveWindowOutput& input,
                             const VolumetricWindow* window) -> MeshDeltaPtr {
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
