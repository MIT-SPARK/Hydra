// Portions of the following code and their modifications are originally from
// https://github.com/ethz-asl/voxblox and are licensed under the following license:
//
// Copyright (c) 2016, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of voxblox nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modifications are also subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#include "hydra_topology/gvd_integrator.h"
#include "hydra_topology/gvd_utilities.h"

#include <voxblox/utils/timing.h>

namespace hydra {
namespace topology {

void UpdateStatistics::clear() {
  number_lowered_voxels = 0;
  number_raised_voxels = 0;
  number_new_voxels = 0;
  number_raise_updates = 0;
  number_voronoi_found = 0;
  number_lower_skipped = 0;
  number_lower_updated = 0;
  number_fixed_no_parent = 0;
  number_force_lowered = 0;
}

std::ostream& operator<<(std::ostream& out, const UpdateStatistics& stats) {
  out << "  - Voxel changes: ";
  out << stats.number_lowered_voxels << " lowered, ";
  out << stats.number_raised_voxels << " raised, ";
  out << stats.number_new_voxels << " new" << std::endl;
  out << "  - New Voronoi Cells: " << stats.number_voronoi_found << std::endl;
  out << "  - Fixed without parents (lower): " << stats.number_fixed_no_parent
      << std::endl;
  out << "  - Skipped (lower): " << stats.number_lower_skipped << std::endl;
  out << "  - Updated (lower): " << stats.number_lower_updated << std::endl;
  out << "  - Forced (lower): " << stats.number_force_lowered << std::endl;
  return out;
}

GvdIntegrator::GvdIntegrator(const GvdIntegratorConfig& config,
                             Layer<TsdfVoxel>* tsdf_layer,
                             const Layer<GvdVoxel>::Ptr& gvd_layer,
                             const MeshLayer::Ptr& mesh_layer)
    : config_(config),
      tsdf_layer_(tsdf_layer),
      gvd_layer_(gvd_layer),
      mesh_layer_(mesh_layer) {
  // TODO(nathan) we could consider an exception here for any of these
  CHECK(tsdf_layer_);
  CHECK(gvd_layer_);
  CHECK(mesh_layer_);
  CHECK(tsdf_layer_->voxel_size() == gvd_layer_->voxel_size());
  CHECK(tsdf_layer_->voxels_per_side() == gvd_layer_->voxels_per_side());

  voxel_size_ = gvd_layer_->voxel_size();

  lower_.setNumBuckets(config_.num_buckets, config_.max_distance_m);

  mesh_integrator_.reset(new VoxelAwareMeshIntegrator(config_.mesh_integrator_config,
                                                      tsdf_layer_,
                                                      gvd_layer_.get(),
                                                      mesh_layer_.get()));

  graph_extractor_.reset(new GraphExtractor(config_.graph_extractor_config));
}

uint8_t GvdIntegrator::updateGvdParentMap(const GlobalIndex& voxel_index,
                                          const GvdVoxel& neighbor) {
  const GlobalIndex neighbor_parent = Eigen::Map<const GlobalIndex>(neighbor.parent);
  if (!gvd_parents_.count(voxel_index)) {
    gvd_parents_[voxel_index] = voxblox::LongIndexSet();
  }

  uint8_t curr_extra_basis = gvd_parents_[voxel_index].size();
  for (const auto& other_parent : gvd_parents_[voxel_index]) {
    const bool is_unique = isParentUnique(
        config_.voronoi_config, voxel_index, other_parent, neighbor_parent);
    if (!is_unique) {
      return curr_extra_basis;
    }
  }

  // parent is unique enough
  gvd_parents_[voxel_index].insert(neighbor_parent);
  markNewGvdParent(neighbor_parent);
  return curr_extra_basis + 1;
}

void GvdIntegrator::markNewGvdParent(const GlobalIndex& parent) {
  if (gvd_parent_vertices_.count(parent)) {
    // make sure the parent vertex map stays alive for this gvd member
    gvd_parent_vertices_[parent].ref_count++;
    return;
  }

  GvdVoxel* parent_voxel = gvd_layer_->getVoxelPtrByGlobalIndex(parent);
  if (!parent_voxel || !parent_voxel->on_surface) {
    // we can't do anything for parents that have left the active mesh before being used
    // as a GVD parent, or parents that aren't registered to the mesh
    return;
  }

  GvdVertexInfo info;
  info.vertex = parent_voxel->block_vertex_index;
  info.ref_count = 1;
  std::memcpy(info.block, parent_voxel->mesh_block, sizeof(info.block));

  BlockIndex block_index = Eigen::Map<BlockIndex>(parent_voxel->mesh_block);
  const auto& mesh_block = mesh_layer_->getMeshByIndex(block_index);
  if (info.vertex < mesh_block.vertices.size()) {
    voxblox::Point vertex_pos = mesh_block.vertices.at(info.vertex);
    info.pos[0] = vertex_pos(0);
    info.pos[1] = vertex_pos(1);
    info.pos[2] = vertex_pos(2);
  }

  gvd_parent_vertices_[parent] = info;
}

void GvdIntegrator::removeVoronoiFromGvdParentMap(const GlobalIndex& voxel_index) {
  auto voxel_parents = gvd_parents_.find(voxel_index);
  if (voxel_parents != gvd_parents_.end()) {
    for (const auto& parent : voxel_parents->second) {
      if (gvd_parent_vertices_.count(parent)) {
        // decrement the ref count (we garbage collect later to avoid losing parents
        // due to thrashing)
        gvd_parent_vertices_[parent].ref_count--;
      }
    }

    gvd_parents_.erase(voxel_parents);
  }
}

void GvdIntegrator::updateVertexMapping() {
  auto iter = gvd_parent_vertices_.begin();
  while (iter != gvd_parent_vertices_.end()) {
    if (!iter->second.ref_count) {
      iter = gvd_parent_vertices_.erase(iter);
      continue;
    }

    GvdVoxel* voxel = gvd_layer_->getVoxelPtrByGlobalIndex(iter->first);
    if (!voxel) {
      ++iter;
      continue;
    }

    if (!voxel->on_surface) {
      iter = gvd_parent_vertices_.erase(iter);
      continue;
    }

    iter->second.vertex = voxel->block_vertex_index;

    const BlockIndex block_index = Eigen::Map<BlockIndex>(voxel->mesh_block);
    Eigen::Map<BlockIndex>(iter->second.block) = block_index;

    const auto& mesh_block = mesh_layer_->getMeshByIndex(block_index);
    if (voxel->block_vertex_index >= mesh_block.vertices.size()) {
      LOG(ERROR) << "Invalid vertex: " << voxel->block_vertex_index
                 << " >= " << mesh_block.vertices.size();
      iter = gvd_parent_vertices_.erase(iter);
      continue;
    }

    voxblox::Point vertex_pos = mesh_block.vertices.at(iter->second.vertex);
    iter->second.pos[0] = vertex_pos(0);
    iter->second.pos[1] = vertex_pos(1);
    iter->second.pos[2] = vertex_pos(2);

    ++iter;
  }
}

void GvdIntegrator::updateGvdVoxel(const GlobalIndex& voxel_index,
                                   GvdVoxel& voxel,
                                   GvdVoxel& other) {
  if (!isVoronoi(voxel)) {
    update_stats_.number_voronoi_found++;
    const GlobalIndex parent = Eigen::Map<const GlobalIndex>(voxel.parent);
    markNewGvdParent(parent);
  }

  auto new_basis = updateGvdParentMap(voxel_index, other);
  if (new_basis == voxel.num_extra_basis) {
    return;
  }

  voxel.num_extra_basis = new_basis;

  if (voxel.num_extra_basis == config_.min_basis_for_extraction) {
    graph_extractor_->pushGvdIndex(voxel_index);
  }
}

void GvdIntegrator::clearGvdVoxel(const GlobalIndex& index, GvdVoxel& voxel) {
  if (voxel.num_extra_basis) {
    // TODO(nathan) rethink how clearing voxels from graph extractor works
    graph_extractor_->clearGvdIndex(index);
    removeVoronoiFromGvdParentMap(index);
  }

  resetVoronoi(voxel);
}

void GvdIntegrator::updateVoronoiQueue(GvdVoxel& voxel,
                                       const GlobalIndex& voxel_idx,
                                       GvdVoxel& neighbor,
                                       const GlobalIndex& neighbor_idx) {
  VoronoiCondition result =
      checkVoronoi(config_.voronoi_config, voxel, voxel_idx, neighbor, neighbor_idx);

  if (!result.current_is_voronoi && !result.neighbor_is_voronoi) {
    return;
  }

  if (result.current_is_voronoi) {
    updateGvdVoxel(voxel_idx, voxel, neighbor);
  }

  if (result.neighbor_is_voronoi) {
    updateGvdVoxel(neighbor_idx, neighbor, voxel);
  }
}

BlockIndexList GvdIntegrator::removeDistantBlocks(const voxblox::Point& center,
                                                  double max_distance) {
  BlockIndexList blocks;
  gvd_layer_->getAllAllocatedBlocks(&blocks);

  BlockIndexList archived;
  for (const auto& idx : blocks) {
    Block<GvdVoxel>::Ptr block = gvd_layer_->getBlockPtrByIndex(idx);
    if ((center - block->origin()).norm() < max_distance) {
      continue;
    }

    for (size_t v = 0; v < block->num_voxels(); ++v) {
      const GvdVoxel& voxel = block->getVoxelByLinearIndex(v);
      if (!voxel.observed) {
        continue;
      }

      const GlobalIndex global_index =
          voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              idx,
              block->computeVoxelIndexFromLinearIndex(v),
              gvd_layer_->voxels_per_side());
      graph_extractor_->removeDistantIndex(global_index);

      removeVoronoiFromGvdParentMap(global_index);
    }

    // we explicitly tsdf and gvd blocks here to avoid potential weirdness
    tsdf_layer_->removeBlock(idx);
    gvd_layer_->removeBlock(idx);
    archived.push_back(idx);
  }

  return archived;
}

void GvdIntegrator::updateFromTsdfLayer(bool clear_updated_flag,
                                        bool clear_surface_flag,
                                        bool use_all_blocks) {
  update_stats_.clear();

  BlockIndexList blocks;
  if (use_all_blocks) {
    tsdf_layer_->getAllAllocatedBlocks(&blocks);
  } else {
    tsdf_layer_->getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
  }

  voxblox::timing::Timer gvd_timer("gvd");

  VLOG(1) << "[GVD update]: Using " << blocks.size() << " updated TSDF blocks";

  voxblox::timing::Timer allocate_timer("gvd/allocate_blocks");
  for (const auto& idx : blocks) {
    // make sure the blocks match the tsdf
    Block<GvdVoxel>::Ptr gvd_block = gvd_layer_->allocateBlockPtrByIndex(idx);
    if (clear_surface_flag) {
      for (size_t idx = 0u; idx < gvd_block->num_voxels(); ++idx) {
        // we need to reset these so that marching cubes can assign them correctly
        gvd_block->getVoxelByLinearIndex(idx).on_surface = false;
      }
    }
  }
  allocate_timer.Stop();

  // sets voxel surface flags
  VLOG(3) << "[GVD update]: starting marching cubes";
  voxblox::timing::Timer marching_cubes_timer("gvd/marching_cubes");
  mesh_integrator_->generateMesh(!use_all_blocks, clear_updated_flag);
  marching_cubes_timer.Stop();
  VLOG(3) << "[GVD update]: finished marching cubes";

  if (config_.mesh_only) {
    return;
  }

  VLOG(3) << "[GVD update]: propagating TSDF";
  voxblox::timing::Timer propagate_timer("gvd/propagate_tsdf");
  for (const BlockIndex& idx : blocks) {
    processTsdfBlock(tsdf_layer_->getBlockByIndex(idx), idx);
  }
  propagate_timer.Stop();
  VLOG(3) << "[GVD update]: finished propagating TSDF";

  VLOG(3) << "[GVD update]: raising invalid voxels";
  voxblox::timing::Timer raise_timer("gvd/raise_esdf");
  processRaiseSet();
  raise_timer.Stop();

  VLOG(3) << "[GVD update]: lowering all voxels";
  voxblox::timing::Timer update_timer("gvd/update_esdf");
  processLowerSet();
  update_timer.Stop();
  VLOG(3) << "[GVD update]: finished lowering all voxels";

  if (config_.extract_graph) {
    VLOG(3) << "[GVD update]: starting graph extraction";
    voxblox::timing::Timer extraction_timer("gvd/extract_graph");
    updateVertexMapping();
    graph_extractor_->extract(*gvd_layer_);
    graph_extractor_->assignMeshVertices(
        *gvd_layer_, gvd_parents_, gvd_parent_vertices_);
    extraction_timer.Stop();
    VLOG(3) << "[GVD update]: finished graph extraction";
  }

  gvd_timer.Stop();

  VLOG(2) << "[GVD update]: " << std::endl << update_stats_;

  if (!clear_updated_flag) {
    return;
  }

  for (const auto& idx : blocks) {
    if (tsdf_layer_->hasBlock(idx)) {
      tsdf_layer_->getBlockByIndex(idx).updated().reset(voxblox::Update::kEsdf);
    }
  }
}

void GvdIntegrator::updateUnobservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                                             const GlobalIndex& index,
                                             GvdVoxel& gvd_voxel) {
  gvd_voxel.observed = true;

  if (isTsdfFixed(tsdf_voxel) || gvd_voxel.on_surface) {
    gvd_voxel.distance = tsdf_voxel.distance;
    gvd_voxel.fixed = true;
    pushToQueue(index, gvd_voxel, PushType::NEW);
    return;
  }

  setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
  gvd_voxel.fixed = false;
}

void GvdIntegrator::updateObservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                                           const GlobalIndex& index,
                                           GvdVoxel& gvd_voxel) {
  if (gvd_voxel.on_surface) {
    gvd_voxel.fixed = true;

    if (std::abs(tsdf_voxel.distance - gvd_voxel.distance) < config_.min_diff_m) {
      return;  // hysterisis to avoid re-integrating near surfaces
    }

    // start a lower-wavefront from the surface if the distance has changed enough
    gvd_voxel.distance = tsdf_voxel.distance;
    pushToQueue(index, gvd_voxel, PushType::LOWER);
    return;
  }

  if (!gvd_voxel.on_surface && !gvd_voxel.has_parent) {
    // force a raise when on_surface flips. this can also potentially trigger for
    // non-surface isolated fixed voxels
    gvd_voxel.fixed = false;
    setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
    pushToQueue(index, gvd_voxel, PushType::RAISE);
    return;
  }

  if (isTsdfFixed(tsdf_voxel)) {
    gvd_voxel.fixed = true;

    if (std::abs(tsdf_voxel.distance - gvd_voxel.distance) < config_.min_diff_m) {
      return;  // hysterisis to avoid re-integrating near surfaces
    }

    // the fixed layer (when not on a surface) can also raise voxels
    const PushType action =
        (std::abs(tsdf_voxel.distance) < std::abs(gvd_voxel.distance)) ? PushType::LOWER
                                                                       : PushType::BOTH;

    pushToQueue(index, gvd_voxel, action);
    if (gvd_voxel.on_surface) {
      return;
    }

    // TODO(nathan) reseting here is suspect
    resetGvdParent(gvd_voxel);
    gvd_voxel.distance = tsdf_voxel.distance;
    return;
  }

  if (gvd_voxel.fixed) {
    // TODO(nathan) reseting here is suspect
    // tsdf voxel isn't fixed, so reset this voxel
    resetGvdParent(gvd_voxel);
    gvd_voxel.fixed = false;
    setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
    // TODO(nathan) this used to be both, but that only makes sense inside the fixed
    // layer
    pushToQueue(index, gvd_voxel, PushType::RAISE);
    return;
  }

  if (std::signbit(tsdf_voxel.distance) == std::signbit(gvd_voxel.distance)) {
    return;  // no need to update if the signs match
  }

  const PushType action =
      (tsdf_voxel.distance < gvd_voxel.distance) ? PushType::LOWER : PushType::RAISE;

  resetGvdParent(gvd_voxel);
  setDefaultDistance(gvd_voxel, tsdf_voxel.distance);

  pushToQueue(index, gvd_voxel, action);
}

void GvdIntegrator::processTsdfBlock(const Block<TsdfVoxel>& tsdf_block,
                                     const BlockIndex& block_index) {
  // Allocate the same block in the ESDF layer.
  auto gvd_block = gvd_layer_->getBlockPtrByIndex(block_index);
  gvd_block->set_updated(true);

  for (size_t idx = 0u; idx < tsdf_block.num_voxels(); ++idx) {
    const TsdfVoxel& tsdf_voxel = tsdf_block.getVoxelByLinearIndex(idx);
    if (tsdf_voxel.weight < config_.min_weight) {
      continue;  // If this voxel is unobserved in the original map, skip it.
    }

    GvdVoxel& gvd_voxel = gvd_block->getVoxelByLinearIndex(idx);
    GlobalIndex global_index = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
        block_index,
        gvd_block->computeVoxelIndexFromLinearIndex(idx),
        gvd_layer_->voxels_per_side());

    if (!gvd_voxel.observed) {
      updateUnobservedGvdVoxel(tsdf_voxel, global_index, gvd_voxel);
      gvd_voxel.observed = true;
    } else {
      updateObservedGvdVoxel(tsdf_voxel, global_index, gvd_voxel);
    }
  }
}

void GvdIntegrator::processRaiseSet() {
  voxblox::GlobalIndexVector neighbors;
  GvdNeighborhood::IndexMatrix neighbor_indices;
  VLOG(10) << "***************************************************";
  VLOG(10) << "* Raising voxels                                  *";
  VLOG(10) << "***************************************************";

  while (!raise_.empty()) {
    const GlobalIndex index = popFromRaise();
    // TODO(nathan) reference?
    GvdVoxel* voxel = gvd_layer_->getVoxelPtrByGlobalIndex(index);
    CHECK_NOTNULL(voxel);

    VLOG(10) << "==================";
    VLOG(10) << "before: " << *voxel << " @ " << index.transpose();
    VLOG(10) << "---";

    GvdNeighborhood::getFromGlobalIndex(index, &neighbor_indices);

    for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
      const GlobalIndex& neighbor_index = neighbor_indices.col(idx);

      GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
      if (neighbor == nullptr) {
        continue;
      }

      // TODO(nathan) neighbor->has_parent != neighbor->on_surface should be an
      // invariant
      if (!neighbor->observed || neighbor->fixed || !neighbor->has_parent) {
        continue;
      }

      update_stats_.number_raise_updates++;

      bool descended_from_current;
      Eigen::Map<const GlobalIndex> neighbor_parent(neighbor->parent);
      if (voxel->has_parent) {
        descended_from_current =
            neighbor_parent == Eigen::Map<const GlobalIndex>(voxel->parent);
      } else {
        descended_from_current = neighbor_parent == index;
      }

      VLOG(10) << "n: " << idx << " -> " << *neighbor << " from current? "
               << (descended_from_current ? "yes" : "no");
      if (descended_from_current) {
        pushToQueue(neighbor_index, *neighbor, PushType::RAISE);
        continue;
      }

      // TODO(nathan) Algorithm 2: 32 of Lau et al. 2013
      if (!neighbor->in_queue) {
        pushToQueue(neighbor_index, *neighbor, PushType::LOWER);
      }
    }

    raiseVoxel(*voxel, index);
    VLOG(10) << "---";
    VLOG(10) << "after: " << *voxel << " @ " << index.transpose();
  }
}

void GvdIntegrator::raiseVoxel(GvdVoxel& voxel, const GlobalIndex& voxel_index) {
  // no need to remove from parent list of voronoi voxel: lower wavefront will ensure
  // that the parent map is consistent
  voxel.is_voronoi_parent = false;

  clearGvdVoxel(voxel_index, voxel);

  // TODO(nathan) determining sign of distance here is optimistic
  setDefaultDistance(voxel, voxel.distance);
  resetGvdParent(voxel);
}

bool GvdIntegrator::processNeighbor(GvdVoxel& voxel,
                                    const GlobalIndex& voxel_idx,
                                    FloatingPoint distance,
                                    const GlobalIndex& neighbor_idx,
                                    GvdVoxel& neighbor) {
  DistancePotential candidate;
  if (config_.parent_derived_distance) {
    const voxblox::Point neighbor_pos =
        getVoxelPosition<float>(*gvd_layer_, neighbor_idx);
    voxblox::Point parent_pos;
    if (voxel.has_parent) {
      parent_pos = Eigen::Map<const voxblox::Point>(voxel.parent_pos);
    } else {
      parent_pos = getVoxelPosition<float>(*gvd_layer_, voxel_idx);
    }

    // TODO(nathan): neighbor should have correct sign, but not sure
    candidate.distance =
        std::copysign((neighbor_pos - parent_pos).norm(), neighbor.distance);
    candidate.is_lower = std::abs(candidate.distance) < std::abs(neighbor.distance);
  } else {
    candidate = getLowerDistance(
        voxel.distance, neighbor.distance, distance, config_.min_diff_m);
  }

  if (!neighbor.fixed && !candidate.is_lower && !neighbor.has_parent) {
    update_stats_.number_force_lowered++;
    candidate.is_lower = true;
  }

  if (!candidate.is_lower) {
    updateVoronoiQueue(voxel, voxel_idx, neighbor, neighbor_idx);
    return false;
  }

  if (neighbor.fixed || !candidate.is_lower) {
    return false;
  }

  neighbor.distance = candidate.distance;
  const voxblox::Point voxel_pos = getVoxelPosition<float>(*gvd_layer_, voxel_idx);
  setSdfParent(neighbor, voxel, voxel_idx, voxel_pos);

  if (config_.multi_queue || !neighbor.in_queue) {
    pushToQueue(neighbor_idx, neighbor, PushType::LOWER);
  }

  return true;
}

void GvdIntegrator::setFixedParent(const GvdNeighborhood::IndexMatrix& neighbor_indices,
                                   GvdVoxel& voxel) {
  FloatingPoint best_distance = 0.0;  // overwritten by first valid neighbor
  GvdVoxel* best_neighbor = nullptr;
  GlobalIndex best_neighbor_index;

  for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
    const GlobalIndex& neighbor_index = neighbor_indices.col(n);
    GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
    if (!neighbor) {
      continue;
    }

    if (!neighbor->observed || !neighbor->fixed) {
      // non-fixed voxels have greater distances than this voxel
      continue;
    }

    if (!neighbor->has_parent && !neighbor->on_surface) {
      // neighbor is another fixed layer voxel without a parent
      continue;
    }

    // TODO(nathan) might need to double check handling of negative distances here
    FloatingPoint distance = NeighborhoodLookupTables::kDistances[n] * voxel_size_;
    FloatingPoint neighbor_distance =
        neighbor->distance + std::copysign(distance, voxel.distance);
    if (!best_neighbor || neighbor_distance < best_distance) {
      best_neighbor = neighbor;
      best_neighbor_index = neighbor_index;
      best_distance = neighbor_distance;
    }
  }

  if (!best_neighbor) {
    update_stats_.number_fixed_no_parent++;
    VLOG(5) << "[GVD Update]: Unable to set parent for non-surface fixed layer voxel: "
            << voxel;
    // setting these voxels as surfaces probably distorts the gvd...
    // setGvdSurfaceVoxel(voxel);
  } else {
    const voxblox::Point neighbor_pos =
        getVoxelPosition<float>(*gvd_layer_, best_neighbor_index);
    setSdfParent(voxel, *best_neighbor, best_neighbor_index, neighbor_pos);
  }
}

void GvdIntegrator::processLowerSet() {
  GvdNeighborhood::IndexMatrix neighbor_indices;
  VLOG(10) << "***************************************************";
  VLOG(10) << "* Lowering voxels                                 *";
  VLOG(10) << "***************************************************";
  while (!lower_.empty()) {
    const GlobalIndex index = popFromLower();
    GvdVoxel& voxel = *CHECK_NOTNULL(gvd_layer_->getVoxelPtrByGlobalIndex(index));
    clearGvdVoxel(index, voxel);

    // TODO(nathan) Lau et al have some check for this
    voxel.in_queue = false;
    VLOG(10) << "-----------------------";
    VLOG(10) << "processing " << voxel << " @ " << index.transpose();

    if (!voxelHasDistance(voxel)) {
      VLOG(10) << "skipped";
      update_stats_.number_lower_skipped++;
      continue;
    }

    update_stats_.number_lower_updated++;
    GvdNeighborhood::getFromGlobalIndex(index, &neighbor_indices);

    if (voxel.fixed && !voxel.has_parent && !voxel.on_surface) {
      // we delay assigning parents for voxels in the fixed layer until this point
      // as it should be an invariant that all potential parents have been seen by
      // processLowerSet
      setFixedParent(neighbor_indices, voxel);
      VLOG(10) << "set new parent: " << voxel << " @ " << index.transpose();
    }

    for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
      const GlobalIndex& neighbor_index = neighbor_indices.col(n);
      GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
      if (!neighbor) {
        continue;
      }

      if (!neighbor->observed) {
        continue;
      }

      FloatingPoint distance = NeighborhoodLookupTables::kDistances[n] * voxel_size_;
      processNeighbor(voxel, index, distance, neighbor_index, *neighbor);
    }
  }
}

}  // namespace topology
}  // namespace hydra
