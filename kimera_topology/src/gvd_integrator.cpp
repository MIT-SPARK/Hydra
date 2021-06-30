// TODO(nathan) license statement
#include "kimera_topology/gvd_integrator.h"
#include "kimera_topology/gvd_utilities.h"

#include <voxblox/utils/timing.h>

namespace kimera {
namespace topology {

void UpdateStatistics::clear() {
  number_lowered_voxels = 0;
  number_raised_voxels = 0;
  number_new_voxels = 0;
  number_raise_updates = 0;
  number_voronoi_found = 0;
  number_lower_skipped = 0;
  number_lower_updated = 0;
}

std::ostream& operator<<(std::ostream& out, const UpdateStatistics& stats) {
  out << "  - Voxel changes: ";
  out << stats.number_lowered_voxels << " lowered, ";
  out << stats.number_raised_voxels << " raised, ";
  out << stats.number_new_voxels << " new" << std::endl;
  out << "  - New Voronoi Cells: " << stats.number_voronoi_found << std::endl;
  out << "  - Skipped (lower): " << stats.number_lower_skipped << std::endl;
  out << "  - Updated (lower): " << stats.number_lower_updated << std::endl;
  return out;
}

GvdIntegrator::GvdIntegrator(const GvdIntegratorConfig& config,
                             const Layer<TsdfVoxel>::Ptr& tsdf_layer,
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
                                                      tsdf_layer_.get(),
                                                      gvd_layer_.get(),
                                                      mesh_layer_.get()));
}

void GvdIntegrator::updateFromTsdfLayer(bool clear_updated_flag) {
  update_stats_.clear();

  BlockIndexList blocks;
  tsdf_layer_->getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);

  voxblox::timing::Timer gvd_timer("gvd");

  VLOG(3) << "[GVD update]: Using " << blocks.size() << " updated TSDF blocks";

  voxblox::timing::Timer allocate_timer("gvd/allocate_blocks");
  for (const auto& idx : blocks) {
    // make sure the blocks match the tsdf
    Block<GvdVoxel>::Ptr gvd_block = gvd_layer_->allocateBlockPtrByIndex(idx);
    for (size_t idx = 0u; idx < gvd_block->num_voxels(); ++idx) {
      // we need to reset these so that marching cubes can assign them correctly
      gvd_block->getVoxelByLinearIndex(idx).on_surface = false;
    }
  }
  allocate_timer.Stop();

  // sets voxel surface flags
  voxblox::timing::Timer marching_cubes_timer("gvd/marching_cubes");
  mesh_integrator_->generateMesh(true, false);
  marching_cubes_timer.Stop();

  voxblox::timing::Timer propagate_timer("gvd/propagate_tsdf");
  for (const BlockIndex& idx : blocks) {
    processTsdfBlock(tsdf_layer_->getBlockByIndex(idx), idx);
  }
  propagate_timer.Stop();

  voxblox::timing::Timer raise_timer("gvd/raise_esdf");
  processRaiseSet();
  raise_timer.Stop();

  voxblox::timing::Timer update_timer("gvd/update_esdf");
  processLowerSet();
  update_timer.Stop();

  gvd_timer.Stop();

  VLOG(3) << "[GVD update]: " << std::endl << update_stats_;

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

    resetGvdParent(gvd_voxel);
    gvd_voxel.distance = tsdf_voxel.distance;
    return;
  }

  if (gvd_voxel.fixed) {
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
  auto gvd_block = gvd_layer_->allocateBlockPtrByIndex(block_index);
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
  voxblox::Neighborhood<>::IndexMatrix neighbor_indices;
  while (!raise_.empty()) {
    const GlobalIndex index = popFromRaise();
    // TODO(nathan) reference?
    GvdVoxel* voxel = gvd_layer_->getVoxelPtrByGlobalIndex(index);
    CHECK_NOTNULL(voxel);

    Neighborhood<>::getFromGlobalIndex(index, &neighbor_indices);

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

      if (descended_from_current) {
        setDefaultDistance(*neighbor, neighbor->distance);
        resetGvdParent(*neighbor);
        pushToQueue(neighbor_index, *neighbor, PushType::RAISE);
        continue;
      }

      // TODO(nathan) Algorithm 2: 32 of Lau et al. 2013
      if (!neighbor->in_queue) {
        pushToQueue(neighbor_index, *neighbor, PushType::LOWER);
      }
    }
  }
}

void GvdIntegrator::updateVoronoiQueue(GvdVoxel& voxel,
                                       const GlobalIndex& voxel_idx,
                                       GvdVoxel& neighbor,
                                       const GlobalIndex& neighbor_idx) {
  VoronoiCondition result = checkVoronoi(voxel,
                                         voxel_idx,
                                         neighbor,
                                         neighbor_idx,
                                         config_.voronoi_min_distance_m,
                                         config_.voronoi_neighbor_l1_separation);

  if (!result.current_is_voronoi && !result.neighbor_is_voronoi) {
    VLOG(6) << "---------------------------------------";
    VLOG(6) << "c: " << voxel << " @ " << voxel_idx.transpose();
    VLOG(6) << "n: " << neighbor << " @ " << neighbor_idx.transpose();
    VLOG(6) << "rejected!";
    return;
  }

  if (result.current_is_voronoi) {
    if (!voxel.num_basis) {
      update_stats_.number_voronoi_found++;
    }
    voxel.is_voronoi = true;
    voxel.num_basis++;
  }

  if (result.neighbor_is_voronoi) {
    if (!neighbor.num_basis) {
      update_stats_.number_voronoi_found++;
    }
    neighbor.is_voronoi = true;
    neighbor.num_basis++;
  }

  VLOG(5) << "---------------------------------------";
  VLOG(5) << "c: " << voxel << " @ " << voxel_idx.transpose();
  VLOG(5) << "n: " << neighbor << " @ " << neighbor_idx.transpose();
  VLOG(5) << "c: " << (result.current_is_voronoi ? "yes" : "no")
          << " n: " << (result.neighbor_is_voronoi ? "yes" : "no");
  VLOG(5) << "basis: " << static_cast<int>(voxel.num_basis) << " (curr) "
          << static_cast<int>(neighbor.num_basis) << " (neighbor)";
}

bool GvdIntegrator::processNeighbor(GvdVoxel& voxel,
                                    const GlobalIndex& voxel_idx,
                                    FloatingPoint distance,
                                    const GlobalIndex& neighbor_idx,
                                    GvdVoxel& neighbor) {
  DistancePotential candidate;
  if (config_.parent_derived_distance) {
    const voxblox::Point neighbor_pos = getVoxelCoordinates(neighbor_idx);
    voxblox::Point parent_pos;
    if (voxel.has_parent) {
      parent_pos = getVoxelCoordinates(Eigen::Map<const GlobalIndex>(voxel.parent));
    } else {
      parent_pos = getVoxelCoordinates(voxel_idx);
    }

    candidate.distance =
        std::copysign((neighbor_pos - parent_pos).norm(), voxel.distance);
    candidate.is_lower = std::abs(candidate.distance) < std::abs(neighbor.distance);
  } else {
    candidate = getLowerDistance(
        voxel.distance, neighbor.distance, distance, config_.min_diff_m);
  }

  if (!candidate.is_lower && voxel.fixed) {
    return false;
  }

  if (!candidate.is_lower) {
    updateVoronoiQueue(voxel, voxel_idx, neighbor, neighbor_idx);
    return false;
  }

  neighbor.distance = candidate.distance;
  setGvdParent(neighbor, voxel, voxel_idx);
  // TODO(nathan) ?
  if (config_.multi_queue || !neighbor.in_queue) {
    pushToQueue(neighbor_idx, neighbor, PushType::LOWER);
  }

  return true;
}

void GvdIntegrator::setFixedParent(const Neighborhood<>::IndexMatrix& neighbor_indices,
                                   GvdVoxel& voxel) {
  FloatingPoint best_distance = std::numeric_limits<FloatingPoint>::infinity();
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

    FloatingPoint distance = NeighborhoodLookupTables::kDistances[n] * voxel_size_;
    FloatingPoint neighbor_distance =
        neighbor->distance + std::copysign(distance, voxel.distance);
    if (neighbor_distance < best_distance) {
      best_neighbor = neighbor;
      best_neighbor_index = neighbor_index;
      best_distance = neighbor_distance;
    }
  }

  if (!best_neighbor) {
    LOG(WARNING)
        << "[GVD Update]: Unable to set parent for non-surface fixed layer voxel: "
        << voxel;
    // it's unclear if this will ever occur, but if it does, it probably should act
    // as a parent for the surrounding voxels until a nearby surface is found
    setGvdSurfaceVoxel(voxel);
  } else {
    setGvdParent(voxel, *best_neighbor, best_neighbor_index);
  }
}

void GvdIntegrator::processLowerSet() {
  Neighborhood<>::IndexMatrix neighbor_indices;

  while (!lower_.empty()) {
    const GlobalIndex index = popFromLower();
    GvdVoxel& voxel = *CHECK_NOTNULL(gvd_layer_->getVoxelPtrByGlobalIndex(index));

    // TODO(nathan) Lau et al have some check for this
    voxel.in_queue = false;

    if (!voxelHasDistance(voxel)) {
      update_stats_.number_lower_skipped++;
      continue;
    }

    update_stats_.number_lower_updated++;
    Neighborhood<>::getFromGlobalIndex(index, &neighbor_indices);

    if (voxel.fixed && !voxel.has_parent && !voxel.on_surface) {
      // we delay assigning parents for voxels in the fixed layer until this point
      // as it should be an invariant that all potential parents have been seen by
      // processLowerSet
      setFixedParent(neighbor_indices, voxel);
    }

    for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
      const GlobalIndex& neighbor_index = neighbor_indices.col(n);
      GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
      if (!neighbor) {
        continue;
      }

      if (!neighbor->observed || neighbor->fixed) {
        continue;
      }

      FloatingPoint distance = NeighborhoodLookupTables::kDistances[n] * voxel_size_;
      processNeighbor(voxel, index, distance, neighbor_index, *neighbor);
    }
  }
}

}  // namespace topology
}  // namespace kimera
