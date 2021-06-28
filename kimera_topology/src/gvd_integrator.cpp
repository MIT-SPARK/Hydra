// TODO(nathan) license statement
#include "kimera_topology/gvd_integrator.h"
#include "kimera_topology/gvd_utilities.h"

#include <voxblox/utils/timing.h>

namespace kimera {
namespace topology {

inline void setParent(GvdVoxel& voxel,
                      const GvdVoxel& neighbor,
                      const GlobalIndex& neighbor_index) {
  voxel.has_parent = true;
  if (neighbor.has_parent) {
    std::memcpy(voxel.parent, neighbor.parent, sizeof(voxel.parent));
  } else {
    Eigen::Map<GlobalIndex>(voxel.parent) = neighbor_index;
  }
}

inline void resetParent(GvdVoxel& voxel) {
  // there's no possible invalid parent index value
  voxel.has_parent = false;
}

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
                             const Layer<GvdVoxel>::Ptr& gvd_layer)
    : config_(config), tsdf_layer_(tsdf_layer), gvd_layer_(gvd_layer) {
  // TODO(nathan) we could consider an exception here for any of these
  CHECK(tsdf_layer_);
  CHECK(gvd_layer_);
  CHECK(tsdf_layer_->voxel_size() == gvd_layer_->voxel_size());
  CHECK(tsdf_layer_->voxels_per_side() == gvd_layer_->voxels_per_side());

  sma_threshold_ = std::cos(config.sma_angle);
  sma_min_distance_ = config.sma_truncation_multiplier * config.min_distance_m;
  voxel_size_ = gvd_layer_->voxel_size();

  open_.setNumBuckets(config_.num_buckets, config_.max_distance_m);
}

void GvdIntegrator::updateFromTsdfLayer(bool clear_updated_flag) {
  update_stats_.clear();

  BlockIndexList blocks;
  tsdf_layer_->getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);

  voxblox::timing::Timer esdf_timer("gvd");

  VLOG(3) << "[GVD update]: Using " << blocks.size() << " updated TSDF blocks";

  voxblox::timing::Timer propagate_timer("esdf/propagate_tsdf");
  for (const BlockIndex& idx : blocks) {
    processTsdfBlock(tsdf_layer_->getBlockByIndex(idx), idx);
  }
  propagate_timer.Stop();

  voxblox::timing::Timer raise_timer("esdf/raise_esdf");
  processRaiseSet();
  raise_timer.Stop();

  voxblox::timing::Timer update_timer("esdf/update_esdf");
  processOpenSet();
  update_timer.Stop();

  esdf_timer.Stop();

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

bool GvdIntegrator::updateVoxelFromNeighbors(const GlobalIndex& index,
                                             GvdVoxel& voxel) {
  Neighborhood<>::IndexMatrix neighbor_indices;
  Neighborhood<>::getFromGlobalIndex(index, &neighbor_indices);

  bool found_lower_neighbor = false;
  for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
    const GlobalIndex& neighbor_index = neighbor_indices.col(n);

    GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
    if (!neighbor) {
      continue;
    }

    if (!voxelHasDistance(*neighbor)) {
      continue;
    }

    // opposite sides of an obstacle -> skip
    if (std::signbit(neighbor->distance) != std::signbit(voxel.distance)) {
      continue;
    }

    const FloatingPoint distance = voxel_size_ * Neighborhood<>::kDistances[n];
    double potential_distance =
        neighbor->distance + std::copysign(distance, neighbor->distance);

    if (std::abs(potential_distance) < std::abs(voxel.distance)) {
      // TODO(nathan) this is silly (should have full euclidean distance option)
      voxel.distance = potential_distance;
      setParent(voxel, *neighbor, neighbor_index);
      found_lower_neighbor = true;
    }
  }

  return found_lower_neighbor;
}

void GvdIntegrator::updateUnobservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                                             const GlobalIndex& index,
                                             GvdVoxel& gvd_voxel) {
  gvd_voxel.observed = true;
  resetParent(gvd_voxel);

  if (std::abs(tsdf_voxel.distance) < config_.min_distance_m) {
    gvd_voxel.distance = tsdf_voxel.distance;
    gvd_voxel.fixed = true;
    pushToQueue(index, gvd_voxel, PushType::NEW);
    return;
  }

  setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
  gvd_voxel.fixed = false;

  // if (updateVoxelFromNeighbors(index, gvd_voxel)) {
  // pushToQueue(index, gvd_voxel, PushType::NEW);
  //}
}

void GvdIntegrator::updateObservedGvdVoxel(const TsdfVoxel& tsdf_voxel,
                                           const GlobalIndex& index,
                                           GvdVoxel& gvd_voxel) {
  // handle gvd voxels in the fixed layer
  if (std::abs(tsdf_voxel.distance) < config_.min_distance_m) {
    if (std::abs(tsdf_voxel.distance - gvd_voxel.distance) < config_.min_diff_m) {
      return;  // hysterisis to avoid re-integrating near surfaces
    }

    const PushType action =
        (std::abs(tsdf_voxel.distance) < std::abs(gvd_voxel.distance)) ? PushType::LOWER
                                                                       : PushType::BOTH;

    resetParent(gvd_voxel);
    gvd_voxel.fixed = true;
    gvd_voxel.distance = tsdf_voxel.distance;

    pushToQueue(index, gvd_voxel, action);
    return;
  }

  if (gvd_voxel.fixed) {
    // tsdf voxel isn't fixed, so reset this voxel
    resetParent(gvd_voxel);
    gvd_voxel.fixed = false;
    setDefaultDistance(gvd_voxel, tsdf_voxel.distance);
    pushToQueue(index, gvd_voxel, PushType::BOTH);
    return;
  }

  if (std::signbit(tsdf_voxel.distance) == std::signbit(gvd_voxel.distance)) {
    return;  // no need to update if the signs match
  }

  const PushType action =
      (tsdf_voxel.distance < gvd_voxel.distance) ? PushType::LOWER : PushType::RAISE;

  resetParent(gvd_voxel);
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

      if (!neighbor->observed || neighbor->fixed || !neighbor->has_parent) {
        continue;
      }

      update_stats_.number_raise_updates++;

      bool is_neighbors_parent = Eigen::Map<GlobalIndex>(neighbor->parent) == index;
      if (is_neighbors_parent) {
        setDefaultDistance(*neighbor, neighbor->distance);
        resetParent(*neighbor);
        pushToQueue(neighbor_index, *neighbor, PushType::RAISE);
        continue;
      }

      if (!neighbor->in_queue) {
        pushToQueue(neighbor_index, *neighbor, PushType::LOWER);
      }
    }
  }
}

void GvdIntegrator::updateMedialAxisQueue(GvdVoxel& voxel,
                                          const GlobalIndex& voxel_idx,
                                          GvdVoxel& neighbor,
                                          const GlobalIndex& neighbor_idx,
                                          const SignedIndex& direction) {
  VLOG(5) << "---------------------------------------";
  VLOG(5) << "curr: " << voxel << " @ " << voxel_idx.transpose();
  VLOG(5) << "neighbor: " << neighbor << " @ " << neighbor_idx.transpose();
  VLOG(5) << "curr -> neighbor: " << direction.transpose();

  VoronoiCondition result =
      checkVoronoi(voxel, voxel_idx, neighbor, neighbor_idx, sma_min_distance_);

  if (result.current_is_voronoi) {
    if (!voxel.num_basis) {
      update_stats_.number_voronoi_found++;
    }
    voxel.is_voronoi = true;
    voxel.num_basis++;
    VLOG(5) << "curr voxel basis count: " << static_cast<int>(voxel.num_basis);
  }

  if (result.neighbor_is_voronoi) {
    if (!neighbor.num_basis) {
      update_stats_.number_voronoi_found++;
    }
    neighbor.is_voronoi = true;
    neighbor.num_basis++;
    VLOG(5) << "curr neighbor basis count: " << static_cast<int>(neighbor.num_basis);
  }

  if (!result.current_is_voronoi && !result.neighbor_is_voronoi) {
    VLOG(5) << "rejected: lau et al. check not met";
  }
}

bool GvdIntegrator::processNeighbor(GvdVoxel& voxel,
                                    const GlobalIndex& voxel_idx,
                                    unsigned int idx,
                                    const GlobalIndex& neighbor_idx,
                                    GvdVoxel& neighbor) {
  const SignedIndex& direction = NeighborhoodLookupTables::kOffsets.col(idx);
  FloatingPoint distance = NeighborhoodLookupTables::kDistances[idx] * voxel_size_;

  DistancePotential candidate =
      getLowerDistance(voxel.distance, neighbor.distance, distance, config_.min_diff_m);

  if (!candidate.is_lower && voxel.fixed) {
    return false;
  }

  if (!candidate.is_lower || neighbor.fixed) {
    if (!neighbor.has_parent) {
      return false;
    }

    updateMedialAxisQueue(voxel, voxel_idx, neighbor, neighbor_idx, direction);
    return false;
  }

  if (neighbor.fixed) {
    // we can't update any fixed cells (but we needed to check for the GVD)
    return false;
  }

  neighbor.distance = candidate.distance;
  setParent(neighbor, voxel, voxel_idx);
  // TODO(nathan) ?
  if (config_.multi_queue || !neighbor.in_queue) {
    pushToQueue(neighbor_idx, neighbor, PushType::LOWER);
  }

  return true;
}

void GvdIntegrator::processOpenSet() {
  Neighborhood<>::IndexMatrix neighbor_indices;

  while (!open_.empty()) {
    const GlobalIndex index = popFromOpen();
    GvdVoxel& voxel = *CHECK_NOTNULL(gvd_layer_->getVoxelPtrByGlobalIndex(index));
    // TODO(nathan) Lau et al have some check for this
    voxel.in_queue = false;

    if (!voxelHasDistance(voxel)) {
      update_stats_.number_lower_skipped++;
      continue;
    }

    update_stats_.number_lower_updated++;

    Neighborhood<>::getFromGlobalIndex(index, &neighbor_indices);
    for (unsigned int n = 0u; n < neighbor_indices.cols(); ++n) {
      const GlobalIndex& neighbor_index = neighbor_indices.col(n);
      GvdVoxel* neighbor = gvd_layer_->getVoxelPtrByGlobalIndex(neighbor_index);
      if (!neighbor) {
        continue;
      }

      if (!neighbor->observed) {
        continue;
      }

      processNeighbor(voxel, index, n, neighbor_index, *neighbor);
    }
  }
}

}  // namespace topology
}  // namespace kimera
