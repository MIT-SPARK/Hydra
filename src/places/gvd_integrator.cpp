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
#include "hydra/places/gvd_integrator.h"

#include "hydra/places/compression_graph_extractor.h"
#include "hydra/places/floodfill_graph_extractor.h"
#include "hydra/places/gvd_utilities.h"
#include "hydra/reconstruction/voxblox_utilities.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace places {

using timing::ScopedTimer;
using EdgeNeighborhood = Neighborhood<voxblox::Connectivity::kTwentySix>;

GvdIntegrator::GvdIntegrator(const GvdIntegratorConfig& config,
                             const Layer<GvdVoxel>::Ptr& gvd_layer)
    : default_distance_(config.max_distance_m), config_(config), gvd_layer_(gvd_layer) {
  // TODO(nathan) we could consider an exception here
  CHECK(gvd_layer_);

  voxel_size_ = gvd_layer_->voxel_size();
  // config_.positive_distance_only toggles between only integrating to the negative
  // truncation distance or integrating to the full max distance
  min_integration_distance_m_ = config_.positive_distance_only
                                    ? -config_.min_distance_m
                                    : -config_.max_distance_m;

  // we want at least enough buckets so that we bin voxels within approximately the same
  // distance (i.e. within half a voxel)
  const size_t num_buckets = 4 * config_.max_distance_m / voxel_size_;
  open_.setNumBuckets(num_buckets, config_.max_distance_m);

  if (config_.graph_extractor.use_compression_extractor) {
    graph_extractor_.reset(new CompressionGraphExtractor(config_.graph_extractor));
  } else {
    graph_extractor_.reset(new FloodfillGraphExtractor(config_.graph_extractor));
  }
}

const SceneGraphLayer& GvdIntegrator::getGraph() const {
  return graph_extractor_->getGraph();
}

const GvdGraph& GvdIntegrator::getGvdGraph() const {
  return graph_extractor_->getGvdGraph();
}

GraphExtractorInterface& GvdIntegrator::getGraphExtractor() const {
  return *graph_extractor_;
}

void GvdIntegrator::updateFromTsdf(uint64_t timestamp_ns,
                                   Layer<TsdfVoxel>& tsdf,
                                   const Layer<VertexVoxel>& vertices,
                                   const MeshLayer& mesh,
                                   bool clear_updated_flag,
                                   bool use_all_blocks) {
  BlockIndexList blocks;
  if (use_all_blocks) {
    tsdf.getAllAllocatedBlocks(&blocks);
  } else {
    tsdf.getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
  }

  VLOG(1) << "[GVD update] Propagating TSDF using " << blocks.size() << " TSDF blocks";
  ScopedTimer timer("places/propagate_tsdf", timestamp_ns);
  update_stats_.clear();

  for (const BlockIndex& idx : blocks) {
    propagateSurface(idx, mesh, vertices);
    processTsdfBlock(tsdf.getBlockByIndex(idx), idx);
  }

  if (!clear_updated_flag) {
    return;
  }

  for (const auto& idx : blocks) {
    tsdf.getBlockByIndex(idx).updated().reset(voxblox::Update::kEsdf);
  }
}

void GvdIntegrator::updateGvd(uint64_t timestamp_ns) {
  // TODO(nathan) this depends on marching cubes being called beforehand...
  ScopedTimer timer("places/overall_update", timestamp_ns);

  VLOG(3) << "[GVD update] Processing open queue";
  {  // timing scope
    ScopedTimer timer("places/open_queue", timestamp_ns);
    processOpenQueue();
  }  // timing scope

  parent_tracker_.updateVertexMapping(*gvd_layer_);

  if (config_.extract_graph) {
    VLOG(3) << "[GVD update] Starting graph extraction";
    ScopedTimer timer("places/graph_extractor", timestamp_ns);
    graph_extractor_->extract(*gvd_layer_, timestamp_ns);
    graph_extractor_->assignMeshVertices(
        *gvd_layer_, parent_tracker_.parents, parent_tracker_.parent_vertices);
  }

  VLOG(2) << "[GVD update]" << std::endl << update_stats_;
}

void GvdIntegrator::archiveBlocks(const BlockIndexList& blocks) {
  for (const auto& idx : blocks) {
    Block<GvdVoxel>::Ptr block = gvd_layer_->getBlockPtrByIndex(idx);

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

    VLOG(5) << "Removing block: " << idx.transpose();
    gvd_layer_->removeBlock(idx);
  }
}

bool GvdIntegrator::setFixedParent(const Layer<GvdVoxel>& layer,
                                   const GvdNeighborhood::IndexMatrix& neighbor_indices,
                                   GvdVoxel& voxel) {
  FloatingPoint best_distance = 0.0;  // overwritten by first valid neighbor
  const GvdVoxel* best_neighbor = nullptr;
  GlobalIndex best_index;

  for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
    const GlobalIndex& neighbor_index = neighbor_indices.col(n);
    const GvdVoxel* neighbor = layer.getVoxelPtrByGlobalIndex(neighbor_index);
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
    FloatingPoint dist = NeighborhoodLookupTables::kDistances[n] * layer.voxel_size();
    FloatingPoint dist_n = neighbor->distance + std::copysign(dist, voxel.distance);
    if (!best_neighbor || dist_n < best_distance) {
      best_neighbor = neighbor;
      best_index = neighbor_index;
      best_distance = dist_n;
    }
  }

  if (!best_neighbor) {
    return false;
  }

  const voxblox::Point p_n = getVoxelPosition<float>(layer, best_index);
  setSdfParent(voxel, *best_neighbor, best_index, p_n);
  return true;
}

/****************************************************************************************/
/* GVD membership */
/****************************************************************************************/

void GvdIntegrator::updateGvdVoxel(const GlobalIndex& voxel_index,
                                   GvdVoxel& voxel,
                                   GvdVoxel& other) {
  if (!isVoronoi(voxel)) {
    update_stats_.number_voronoi_found++;
    const GlobalIndex parent = Eigen::Map<const GlobalIndex>(voxel.parent);
    parent_tracker_.markNewGvdParent(*gvd_layer_, parent);
  }

  auto new_basis = parent_tracker_.updateGvdParentMap(
      *gvd_layer_, config_.voronoi_config, voxel_index, other);
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

/****************************************************************************************/
/* TSDF Propagation */
/****************************************************************************************/

void GvdIntegrator::propagateSurface(const BlockIndex& block_index,
                                     const MeshLayer& mesh,
                                     const Layer<VertexVoxel>& vertices) {
  const auto& vertex_block = vertices.getBlockByIndex(block_index);
  auto gvd_block = gvd_layer_->allocateBlockPtrByIndex(block_index);

  for (size_t idx = 0u; idx < vertex_block.num_voxels(); ++idx) {
    const auto& vertex_voxel = vertex_block.getVoxelByLinearIndex(idx);
    auto& gvd_voxel = gvd_block->getVoxelByLinearIndex(idx);

    // latches surface status from mesh integrator
    gvd_voxel.on_surface = vertex_voxel.on_surface;
    if (!gvd_voxel.on_surface) {
      continue;
    }

    resetParent(gvd_voxel);  // surface voxels don't have parents

    const auto vertex_idx = vertex_voxel.block_vertex_index;
    BlockIndex mesh_block_index = Eigen::Map<const BlockIndex>(vertex_voxel.mesh_block);
    const auto mesh_block = mesh.getMeshPtrByIndex(mesh_block_index);
    CHECK(mesh_block) << "bad mesh index: " << showIndex(mesh_block_index)
                      << " (block: " << showIndex(block_index) << ")";
    CHECK_LT(vertex_idx, mesh_block->vertices.size())
        << "gvd block: " << block_index.transpose()
        << " mesh index: " << mesh_block_index.transpose();

    const auto& pos = mesh_block->vertices.at(vertex_idx);
    gvd_voxel.parent_pos[0] = pos.x();
    gvd_voxel.parent_pos[1] = pos.y();
    gvd_voxel.parent_pos[2] = pos.z();

    std::memcpy(
        gvd_voxel.mesh_block, vertex_voxel.mesh_block, sizeof(gvd_voxel.mesh_block));
    gvd_voxel.block_vertex_index = vertex_idx;
  }
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
      updateUnobservedVoxel(tsdf_voxel, global_index, gvd_voxel);
      gvd_voxel.observed = true;
    } else {
      updateObservedVoxel(tsdf_voxel, global_index, gvd_voxel);
    }
  }
}

void GvdIntegrator::updateUnobservedVoxel(const TsdfVoxel& tsdf_voxel,
                                          const GlobalIndex& index,
                                          GvdVoxel& gvd_voxel) {
  VLOG(10) << "[gvd] updating unobserved @ " << index.transpose()
           << " (d=" << tsdf_voxel.distance << ")";
  gvd_voxel.observed = true;
  gvd_voxel.is_negative = tsdf_voxel.distance < 0.0;
  update_stats_.number_new_voxels++;

  const bool is_fixed = isTsdfFixed(tsdf_voxel);
  if (is_fixed) {
    gvd_voxel.distance = tsdf_voxel.distance;
    gvd_voxel.fixed = true;
    VLOG(10) << "[gvd] voxel @ " << index.transpose()
             << " pushed to queue as fixed voxel!";
    pushToQueue(index, gvd_voxel);
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

  setDefaultDistance(gvd_voxel, default_distance_);
  VLOG(10) << "[gvd] voxel @ " << index.transpose() << " set to default of "
           << gvd_voxel.distance;
  gvd_voxel.fixed = false;
  // TODO(nathan) pretty sure that we don't need to enqueue here, but should think
  // carefully about Lau version
}

void GvdIntegrator::updateObservedVoxel(const TsdfVoxel& tsdf_voxel,
                                        const GlobalIndex& index,
                                        GvdVoxel& gvd_voxel) {
  VLOG(10) << "[gvd] updating observed @ " << index.transpose()
           << " (d=" << tsdf_voxel.distance << ", gd=" << gvd_voxel.distance << ")";

  // we can use this to check more intelligently for flipped voxels later
  gvd_voxel.is_negative = tsdf_voxel.distance < 0.0;
  const bool is_fixed = isTsdfFixed(tsdf_voxel);

  if (!gvd_voxel.on_surface && !gvd_voxel.has_parent) {
    VLOG(10) << "[gvd] raising potential previous surface voxel";
    // raise any "cleared" voxels (equivalent to removeObstacle in Lau et al.)
    pushToQueue(index, gvd_voxel);
    gvd_voxel.fixed = is_fixed;
    setRaiseStatus(gvd_voxel, default_distance_);
    return;
  }

  if (is_fixed && !gvd_voxel.fixed) {
    // flipping to fixed will always result in a smaller distance value
    VLOG(10) << "[gvd] new fixed voxel @ " << index.transpose();
    gvd_voxel.fixed = true;
    // okay to rewire fixed parents
    resetParent(gvd_voxel);
    // push after updating distance because this is a lower wavefront
    gvd_voxel.distance = tsdf_voxel.distance;
    pushToQueue(index, gvd_voxel);
    return;
  }

  if (is_fixed) {
    if (std::abs(tsdf_voxel.distance - gvd_voxel.distance) <= config_.min_diff_m) {
      VLOG(10) << "[gvd] no change @ " << index.transpose();
      return;  // hysterisis to avoid re-integrating near surfaces
    }

    pushToQueue(index, gvd_voxel);  // not a huge distinction, but we push before
                                    // resetting the distance

    const FloatingPoint d_t = std::abs(tsdf_voxel.distance);
    const FloatingPoint d_g = std::abs(gvd_voxel.distance);
    const bool is_tsdf_lower = d_t < d_g;
    gvd_voxel.distance = tsdf_voxel.distance;

    // the fixed layer (when not on a surface) can also raise voxels
    gvd_voxel.to_raise = !is_tsdf_lower;

    VLOG(10) << "[gvd] |d| =" << d_t << ", |gd| =" << d_g;
    VLOG(10) << "[gvd] fixed @ " << index.transpose() << ", lower: " << std::boolalpha
             << is_tsdf_lower;
    return;
  }

  // is_fixed is false after this point

  if (gvd_voxel.on_surface) {
    gvd_voxel.on_surface = false;
    update_stats_.number_surface_flipped++;
  }

  if (gvd_voxel.fixed) {
    gvd_voxel.fixed = false;
    pushToQueue(index, gvd_voxel);  // push uses distance and needs to come before raise
    setRaiseStatus(gvd_voxel, default_distance_);
    VLOG(10) << "[gvd] raising previously fixed voxel @ " << index.transpose();
    return;
  }

  if (std::signbit(tsdf_voxel.distance) == std::signbit(gvd_voxel.distance)) {
    return;  // no need to update if the signs match
  }

  VLOG(10) << "[gvd] raising flipped voxel @ " << index.transpose();
  // TODO(nathan) add to tracked statistics
  // we raise any voxel where the sign flips
  pushToQueue(index, gvd_voxel);  // push uses distance and needs to come before raise
  setRaiseStatus(gvd_voxel, default_distance_);
}

/****************************************************************************************/
/* ESDF integration */
/****************************************************************************************/

void GvdIntegrator::pushToQueue(const GlobalIndex& index, GvdVoxel& voxel) {
  voxel.in_queue = true;
  open_.push({index, &voxel}, voxel.distance);
  update_stats_.number_queue_inserts++;
}

void GvdIntegrator::raiseVoxel(const GlobalIndex& index, GvdVoxel& voxel) {
  GvdNeighborhood::getFromGlobalIndex(index, &neighbor_indices_);
  for (unsigned int idx = 0u; idx < neighbor_indices_.cols(); ++idx) {
    const GlobalIndex& neighbor_index = neighbor_indices_.col(idx);

    GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);

    if (neighbor && neighbor->observed) {
      VLOG(10) << "[gvd] checking neighbor " << *neighbor << " @ "
               << neighbor_index.transpose() << " during raise for "
               << index.transpose();
    }

    if (neighbor == nullptr || !neighbor->observed || neighbor->to_raise ||
        !neighbor->has_parent || neighbor->on_surface) {
      // this covers the check on Lau et al. 32: we don't need to propagate the
      // wavefront if another voxel:
      // 1) doesn't exist
      // 2) has already been inserted in the raise queue
      // 3) if the parent has already been cleared
      // TODO(nathan) on surface should be covered by !has_parent
      continue;
    }

    // starts a lower wavefront at the end of raising (see Lau et al. 33)
    // or continues propagating raise wavefront
    pushToQueue(neighbor_index, *neighbor);
    VLOG(10) << "[gvd] pushing neighbor " << *neighbor << " @ "
             << neighbor_index.transpose() << " to queue";

    // Lau et al. 34: check to see if the parent of the neighbor still exists
    // as long as the bucket resolution is the same or less than the voxel resolution,
    // it should be an invariant that the parent gets cleared before the children
    // we can't promise that the parent exists though. Maybe we can promise that the
    // parent will exist if it gets cleared?
    // yes: we shouldn't be able to clear it if it doesn't exist.
    const GlobalIndex neighbor_parent = Eigen::Map<const GlobalIndex>(neighbor->parent);
    GvdVoxel* parent_ptr = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_parent);
    if (parent_ptr) {
      VLOG(10) << "[gvd] parent: " << *parent_ptr << " @ "
               << neighbor_parent.transpose();
    }

    if (!parent_ptr || parent_ptr->on_surface) {
      // if the parent doesn't exist, we know that we can't raise this
      // if the parent points to a surface, we also can't raise this
      continue;
    }

    VLOG(10) << "[gvd] raising neighbor " << neighbor_index.transpose();
    setRaiseStatus(*neighbor, default_distance_);
  }

  // TODO(nathan) make sure we handle raise criteria appropriately earlier
  update_stats_.number_raise_updates++;
  voxel.to_raise = false;
  VLOG(10) << "after raise: " << voxel << " @ " << index.transpose();
}

void GvdIntegrator::lowerVoxel(const GlobalIndex& index, GvdVoxel& voxel) {
  update_stats_.number_lower_updated++;
  GvdNeighborhood::getFromGlobalIndex(index, &neighbor_indices_);

  if (voxel.fixed && !voxel.has_parent && !voxel.on_surface) {
    // we delay assigning parents for voxels in the fixed layer until this point
    // as it should be an invariant that all potential parents have been seen by
    // processLowerSet
    if (!setFixedParent(*gvd_layer_, neighbor_indices_, voxel)) {
      update_stats_.number_fixed_no_parent++;
      VLOG(5) << "[GVD Update] Unable to set parent for fixed voxel: " << voxel;
      return;
    } else {
      VLOG(10) << "set new parent: " << voxel << " @ " << index.transpose();
    }
  }

  const voxblox::Point p_p = getParentPosition(index, voxel);
  const voxblox::Point p_v = getVoxelPosition<float>(*gvd_layer_, index);

  // get normal to parent for improved neighborhood expansion in section 4.3
  GlobalIndex parent_index;
  if (voxel.has_parent) {
    parent_index = Eigen::Map<const GlobalIndex>(voxel.parent);
  } else {
    parent_index = index;
  }
  const GlobalIndex w = index - parent_index;

  for (unsigned int n = 0u; n < neighbor_indices_.cols(); ++n) {
    const auto& offset = GvdNeighborhood::kLongOffsets.col(n);
    if (offset.x() * w.x() < 0) {
      continue;
    }

    if (offset.y() * w.y() < 0) {
      continue;
    }

    if (offset.z() * w.z() < 0) {
      continue;
    }

    const GlobalIndex& neighbor_index = neighbor_indices_.col(n);
    GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
    if (!neighbor || !neighbor->observed || neighbor->to_raise) {
      // this supplements the lau et al. check on 39 to also make sure the neighbor
      // exists
      continue;
    }

    DistancePotential candidate;
    const voxblox::Point p_n = getPosition(neighbor_index);
    candidate.distance = std::copysign((p_n - p_p).norm(), neighbor->distance);
    candidate.is_lower = std::abs(candidate.distance) < std::abs(neighbor->distance);
    if (!neighbor->fixed && !candidate.is_lower && !neighbor->has_parent) {
      update_stats_.number_force_lowered++;
      candidate.is_lower = true;
    }

    VLOG(10) << "[gvd] candidate: " << candidate.distance << " for " << *neighbor
             << " @ " << neighbor_index.transpose() << " (for " << index.transpose()
             << ")";

    // TODO(nathan) this used to be before calling lower, not for each neighbor. Both
    // seem correct, but this is closer to the Lau et al. paper
    if (candidate.distance >= config_.max_distance_m ||
        candidate.distance <= min_integration_distance_m_) {
      // TODO(nathan) track this as a statistic?
      continue;
    }

    if (!candidate.is_lower) {
      updateVoronoiQueue(voxel, index, *neighbor, neighbor_index);
      continue;
    }

    if (neighbor->fixed) {
      // TODO(nathan) maybe reconsider this
      continue;
    }

    neighbor->distance = candidate.distance;
    setSdfParent(*neighbor, voxel, index, p_v);
    VLOG(10) << "pushing neighbor " << *neighbor << " @ " << neighbor_index.transpose()
             << " to queue";
    pushToQueue(neighbor_index, *neighbor);
  }
}

// updateDistanceMap in "B. Lau et al., Efficient grid-based .. (2013)"
void GvdIntegrator::processOpenQueue() {
  VLOG(10) << "***************************************************";
  VLOG(10) << "* Processing Open Queue                           *";
  VLOG(10) << "***************************************************";
  while (!open_.empty()) {
    // TODO(nathan) potentially add telemetry
    auto entry = open_.front();
    open_.pop();

    // from Lau et al: Section 4.2
    if (!entry.voxel->in_queue) {
      continue;
    }
    entry.voxel->in_queue = false;

    GvdVoxel& voxel = *entry.voxel;
    const GlobalIndex& index = entry.index;

    VLOG(10) << "-----------------------";
    VLOG(10) << "processing " << voxel << " @ " << index.transpose();

    if (voxel.to_raise) {
      voxel.to_raise = false;
      raiseVoxel(index, voxel);
      // note that this is the same logic as the Lau et al. if/else if on line 27
      continue;
    }

    // this is slightly different than Lau et al, as there are voxels without parents
    // that still have a valid distance and can be used to lower neighboring voxels
    // (i.e., fixed voxels that still need an assigned parent)
    if ((!voxel.has_parent && !voxel.fixed) || !voxel.observed) {
      update_stats_.number_lower_skipped++;
      VLOG(10) << "skipped";
      continue;
    }

    clearGvdVoxel(index, voxel);
    lowerVoxel(index, voxel);
  }
}

/****************************************************************************************/
/* Helpers */
/****************************************************************************************/

bool GvdIntegrator::isTsdfFixed(const TsdfVoxel& voxel) {
  return std::abs(voxel.distance) < config_.min_distance_m;
}

voxblox::Point GvdIntegrator::getParentPosition(const GlobalIndex& index,
                                                const GvdVoxel& voxel) const {
  if (voxel.has_parent) {
    return voxblox::Point(
        voxel.parent_pos[0], voxel.parent_pos[1], voxel.parent_pos[2]);
  }

  return getVoxelPosition<float>(*gvd_layer_, index);
}

voxblox::Point GvdIntegrator::getPosition(const GlobalIndex& index) const {
  return getVoxelPosition<float>(*gvd_layer_, index);
}

}  // namespace places
}  // namespace hydra
