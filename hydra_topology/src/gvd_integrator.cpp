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

#include <hydra_utils/timing_utilities.h>

namespace hydra {
namespace topology {

using timing::ScopedTimer;
using EdgeNeighborhood = Neighborhood<voxblox::Connectivity::kTwentySix>;

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

void GvdIntegrator::updateGvdVoxel(const GlobalIndex& voxel_index,
                                   GvdVoxel& voxel,
                                   GvdVoxel& other) {
  if (!isVoronoi(voxel)) {
    update_stats_.number_voronoi_found++;
    const GlobalIndex parent = Eigen::Map<const GlobalIndex>(voxel.parent);
    parent_tracker_.markNewGvdParent(*gvd_layer_, *mesh_layer_, parent);
  }

  auto new_basis = parent_tracker_.updateGvdParentMap(
      *gvd_layer_, *mesh_layer_, config_.voronoi_config, voxel_index, other);
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
    parent_tracker_.removeVoronoiFromGvdParentMap(index);
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

      parent_tracker_.removeVoronoiFromGvdParentMap(global_index);
    }

    // we explicitly tsdf and gvd blocks here to avoid potential weirdness
    tsdf_layer_->removeBlock(idx);
    gvd_layer_->removeBlock(idx);
    archived.push_back(idx);
  }

  return archived;
}

void GvdIntegrator::updateFromTsdfLayer(uint64_t timestamp_ns,
                                        bool clear_updated_flag,
                                        bool clear_surface_flag,
                                        bool use_all_blocks) {
  ScopedTimer timer("topology/overall", timestamp_ns);
  update_stats_.clear();

  BlockIndexList blocks;
  if (use_all_blocks) {
    tsdf_layer_->getAllAllocatedBlocks(&blocks);
  } else {
    tsdf_layer_->getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
  }

  VLOG(1) << "[GVD update]: Using " << blocks.size() << " TSDF blocks";

  {  // timing scope
    ScopedTimer timer("topology/gvd_allocation", timestamp_ns);
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
  }  // timing scope

  // sets voxel surface flags
  VLOG(3) << "[GVD update]: starting marching cubes";
  {  // timing scope
    ScopedTimer timer("topology/mesh", timestamp_ns);
    mesh_integrator_->generateMesh(!use_all_blocks, clear_updated_flag);
  }  // timing scope

  if (config_.mesh_only) {
    return;
  }

  VLOG(3) << "[GVD update]: propagating TSDF";
  {  // timing scope
    ScopedTimer timer("topology/propagate_tsdf", timestamp_ns);
    for (const BlockIndex& idx : blocks) {
      processTsdfBlock(tsdf_layer_->getBlockByIndex(idx), idx);
    }
  }  // timing scope

  VLOG(3) << "[GVD update]: raising invalid voxels";
  {  // timing scope
    ScopedTimer timer("topology/raise", timestamp_ns);
    processRaiseSet();
  }  // timing scope

  VLOG(3) << "[GVD update]: lowering all voxels";
  {  // timing scope
    ScopedTimer timer("topology/lower", timestamp_ns);
    processLowerSet();
  }  // timing scope

  if (config_.extract_graph) {
    VLOG(3) << "[GVD update]: starting graph extraction";
    ScopedTimer timer("topology/graph_extractor", timestamp_ns);
    parent_tracker_.updateVertexMapping(*gvd_layer_, *mesh_layer_);
    graph_extractor_->extract(*gvd_layer_);
    graph_extractor_->assignMeshVertices(
        *gvd_layer_, parent_tracker_.parents, parent_tracker_.parent_vertices);
  }

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
  VLOG(10) << "[gvd] updating unobserved @ " << index.transpose()
           << " (d=" << tsdf_voxel.distance << ")";
  gvd_voxel.observed = true;
  const bool is_fixed = isTsdfFixed(tsdf_voxel);
  if (is_fixed) {
    gvd_voxel.distance = tsdf_voxel.distance;
    gvd_voxel.fixed = true;
    VLOG(10) << "[gvd] voxel @ " << index.transpose()
             << " pushed to queue as fixed voxel!";
    pushToQueue(index, gvd_voxel, PushType::NEW);
    return;
  }

  if (!is_fixed && gvd_voxel.on_surface) {
    VLOG(10) << "[gvd] flipping invalid surface @ " << index.transpose();
    // Flip surface flag if voxel marked as containing a surface, but outside the
    // trunction distance. This shouldn't happen if the tsdf is smooth, but doesn't
    // appear that this is always the case
    gvd_voxel.on_surface = false;
    update_stats_.number_surface_flipped++;
  }

  setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
  VLOG(10) << "[gvd] voxel @ " << index.transpose() << " set to default of "
           << gvd_voxel.distance;
  gvd_voxel.fixed = false;
}

void GvdIntegrator::updateObservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                                           const GlobalIndex& index,
                                           GvdVoxel& gvd_voxel) {
  VLOG(10) << "[gvd] updating observed @ " << index.transpose()
           << " (d=" << tsdf_voxel.distance << ", gd=" << gvd_voxel.distance << ")";
  // TODO(nathan) handle checking for cases where on_surface flips to false

  const bool is_fixed = isTsdfFixed(tsdf_voxel);
  if (is_fixed && !gvd_voxel.fixed) {
    VLOG(10) << "[gvd] new fixed voxel @ " << index.transpose();
    gvd_voxel.fixed = true;
    gvd_voxel.distance = tsdf_voxel.distance;
    // TODO(nathan) consider resetting parent if necessary
    // flipping to fixed will always result in a smaller distance value
    pushToQueue(index, gvd_voxel, PushType::LOWER);
    return;
  }

  if (is_fixed) {
    if (std::abs(tsdf_voxel.distance - gvd_voxel.distance) <= config_.min_diff_m) {
      VLOG(10) << "[gvd] no change @ " << index.transpose();
      return;  // hysterisis to avoid re-integrating near surfaces
    }

    // the fixed layer (when not on a surface) can also raise voxels
    VLOG(10) << "|d|=" << std::abs(tsdf_voxel.distance)
             << ", |gd|=" << std::abs(gvd_voxel.distance);
    const bool is_tsdf_lower =
        std::abs(tsdf_voxel.distance) < std::abs(gvd_voxel.distance);
    gvd_voxel.distance = tsdf_voxel.distance;
    VLOG(10) << "[gvd] updating fixed voxel @ " << index.transpose()
             << ", tsdf_lower: " << std::boolalpha << is_tsdf_lower;
    pushToQueue(index, gvd_voxel, is_tsdf_lower ? PushType::LOWER : PushType::RAISE);
    return;
  }

  // is_fixed is false after this point

  if (gvd_voxel.on_surface) {
    gvd_voxel.on_surface = false;
    update_stats_.number_surface_flipped++;
  }

  if (gvd_voxel.fixed) {
    gvd_voxel.fixed = false;
    resetGvdParent(gvd_voxel);
    setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
    pushToQueue(index, gvd_voxel, PushType::RAISE);
    VLOG(10) << "[gvd] raising previously fixed voxel @ " << index.transpose();
    return;
  }

  if (std::signbit(tsdf_voxel.distance) == std::signbit(gvd_voxel.distance)) {
    return;  // no need to update if the signs match
  }

  VLOG(10) << "[gvd] raising flipped voxel @ " << index.transpose();
  // TODO(nathan) add to tracked statistics
  // we raise any voxel where the sign flips
  resetGvdParent(gvd_voxel);
  setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
  pushToQueue(index, gvd_voxel, PushType::RAISE);
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
    if (voxel->fixed) {
      pushToQueue(index, *voxel, PushType::LOWER);
    }

    VLOG(10) << "---";
    VLOG(10) << "after: " << *voxel << " @ " << index.transpose();
  }
}

void GvdIntegrator::raiseVoxel(GvdVoxel& voxel, const GlobalIndex& voxel_index) {
  // no need to remove from parent list of voronoi voxel: lower wavefront will ensure
  // that the parent map is consistent
  voxel.is_voronoi_parent = false;

  clearGvdVoxel(voxel_index, voxel);

  if (!voxel.fixed) {
    const TsdfVoxel* tsdf_voxel = tsdf_layer_->getVoxelPtrByGlobalIndex(voxel_index);
    CHECK_NOTNULL(tsdf_voxel);
    setDefaultDistance(voxel, tsdf_voxel->distance);
  }

  if (!voxel.on_surface) {
    resetGvdParent(voxel);
  }
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

void GvdIntegrator::pushToQueue(const GlobalIndex& index,
                                GvdVoxel& voxel,
                                PushType action) {
  switch (action) {
    case PushType::NEW:
      voxel.in_queue = true;
      lower_.push(index, voxel.distance);
      update_stats_.number_new_voxels++;
      break;
    case PushType::LOWER:
      voxel.in_queue = true;
      lower_.push(index, voxel.distance);
      update_stats_.number_lowered_voxels++;
      break;
    case PushType::RAISE:
      raise_.push(index);
      update_stats_.number_raised_voxels++;
      break;
    case PushType::BOTH:
      voxel.in_queue = true;
      lower_.push(index, voxel.distance);
      raise_.push(index);
      update_stats_.number_raised_voxels++;
      break;
    default:
      LOG(FATAL) << "Invalid push type!";
      break;
  }
}

GlobalIndex GvdIntegrator::popFromLower() {
  GlobalIndex index = lower_.front();
  lower_.pop();
  return index;
}

GlobalIndex GvdIntegrator::popFromRaise() {
  GlobalIndex index = raise_.front();
  raise_.pop();
  return index;
}

void GvdIntegrator::setDefaultDistance(GvdVoxel& voxel, double signed_distance) {
  voxel.distance = std::copysign(config_.max_distance_m, signed_distance);
}

bool GvdIntegrator::isTsdfFixed(const TsdfVoxel& voxel) {
  return std::abs(voxel.distance) < config_.min_distance_m;
}

bool GvdIntegrator::voxelHasDistance(const GvdVoxel& voxel) {
  if (!voxel.observed) {
    return false;
  }

  if (voxel.distance >= config_.max_distance_m) {
    return false;
  }

  if (config_.positive_distance_only && voxel.distance < 0.0 && !voxel.fixed) {
    return false;
  }

  if (voxel.distance <= -config_.max_distance_m) {
    return false;
  }

  return true;
}

}  // namespace topology
}  // namespace hydra
