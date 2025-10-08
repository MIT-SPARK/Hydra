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
#include "hydra/places/region_growing_traversability_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <algorithm>
#include <queue>

#include "hydra/utils/timing_utilities.h"

namespace hydra::places {

using Timer = hydra::timing::ScopedTimer;
using spark_dsg::Boundary;
using spark_dsg::NodeId;
using spark_dsg::Side;
using spark_dsg::TraversabilityNodeAttributes;
using State = spark_dsg::TraversabilityState;

namespace {
static const auto registration =
    config::RegistrationWithConfig<TraversabilityClustering,
                                   RegionGrowingTraversabilityClustering,
                                   RegionGrowingTraversabilityClustering::Config>(
        "RegionGrowingTraversabilityClustering");
}  // namespace

void declare_config(RegionGrowingTraversabilityClustering::Config& config) {
  using namespace config;
  name("RegionGrowingTraversabilityClustering::Config");
  field(config.max_place_radius, "max_place_radius", "m");
  field(config.min_place_radius, "min_place_radius", "m");
  check(config.max_place_radius, GT, 0.0f, "max_place_radius");
}

RegionGrowingTraversabilityClustering::RegionGrowingTraversabilityClustering(
    const Config& config)
    : config(config::checkValid(config)) {}

void RegionGrowingTraversabilityClustering::updateGraph(
    const TraversabilityLayer& layer,
    const ActiveWindowOutput& msg,
    spark_dsg::DynamicSceneGraph& graph) {
  // Compute all updated places and store them in the place_infos.
  if (!graph.hasLayer(spark_dsg::DsgLayers::TRAVERSABILITY)) {
    LOG(WARNING) << "Layer '" << spark_dsg::DsgLayers::TRAVERSABILITY
                 << "' does not exist in the DSG.";
    return;
  }
  current_time_ns_ = msg.timestamp_ns;

  // Run voxel clustering.
  VoxelMap assignment = pruneExistingRegions(layer, graph);
  initializeRegions(layer, msg.world_t_body, assignment);
  detectPlaces(layer, assignment);

  // Compute neighbors and simplification.
  computeNeighbors(assignment);
  simplifyRegions(layer.voxel_size);

  // Update the DSG.
  updatePlaceNodesInDsg(graph);
  updatePlaceEdgesInDsg(graph);
  visualizeAssignments(layer, assignment);
}

RegionGrowingTraversabilityClustering::VoxelMap
RegionGrowingTraversabilityClustering::pruneExistingRegions(
    const TraversabilityLayer& layer, spark_dsg::DynamicSceneGraph& graph) {
  // Prune all voxels that are no longer traversable.
  VoxelMap assigned_voxels;
  std::vector<NodeId> to_remove;
  for (auto& [id, region] : regions_) {
    if (!region.is_active) {
      to_remove.push_back(id);
      continue;
    }
    size_t num_active_voxels = 0;
    auto it = region.voxels.begin();
    while (it != region.voxels.end()) {
      const auto* voxel = layer.voxel(*it);
      if (voxel && voxel->state != State::TRAVERSABLE) {
        it = region.voxels.erase(it);
        continue;
      }
      assigned_voxels[*it] = id;
      if (voxel) {
        ++num_active_voxels;
      }
      ++it;
    }
    if (region.voxels.empty()) {
      // This means the region became intraversable.
      graph.removeNode(id);
      to_remove.push_back(id);
    } else if (num_active_voxels == 0) {
      // This means the region became archived, but still exists.
      region.is_active = false;
    }
  }

  // Remove empty regions.
  for (const auto id : to_remove) {
    regions_.erase(id);
  }
  return assigned_voxels;
}

void RegionGrowingTraversabilityClustering::initializeRegions(
    const TraversabilityLayer& layer,
    const Eigen::Vector3d& start_position,
    VoxelMap& assigned_voxels) {
  for (const auto& [id, region] : regions_) {
    if (region.is_active) {
      return;
    }
  }

  // If empty start with the current robot position.
  Eigen::Vector3f start_2d = start_position.cast<float>();
  start_2d.z() = 0.0f;
  const auto start_index = layer.globalIndexFromPoint(start_2d);
  const auto* voxel = layer.voxel(start_index);
  if (!voxel || voxel->state != State::TRAVERSABLE) {
    return;
  }
  Region& region = allocateNewRegion();
  region.voxels.insert(start_index);
  assigned_voxels[start_index] = region.id;
}

void RegionGrowingTraversabilityClustering::detectPlaces(
    const TraversabilityLayer& layer, VoxelMap& assigned_voxels) {
  // Grow existing regions in order, recomputing their boundary voxels.
  Voxels frontier_voxels;
  for (auto& [id, region] : regions_) {
    if (!region.is_active) {
      continue;
    }
    growRegion(layer, region, assigned_voxels);
    updateFrontierVoxels(frontier_voxels, layer, region, assigned_voxels);
  }

  // Assign all unassigned traversable voxels to (new) regions.
  while (!frontier_voxels.empty()) {
    // Start from an arbitrary frontier voxel and grow the region.
    Region& region = allocateNewRegion();
    const auto& seed_index = *frontier_voxels.begin();
    region.voxels.insert(seed_index);
    assigned_voxels[seed_index] = region.id;
    growRegion(layer, region, assigned_voxels);
    updateFrontierVoxels(frontier_voxels, layer, region, assigned_voxels);
  }
}

void RegionGrowingTraversabilityClustering::simplifyRegions(double voxel_size) {
  for (auto& [id, region] : regions_) {
    region.computeCentroid(voxel_size);
    region.computeRadii(voxel_size);
  }

  bool updated = true;
  while (updated) {
    updated = false;
    for (auto& [id, region] : regions_) {
      if (region.is_new && region.max < config.min_place_radius) {
        if (region.neighbors.empty()) {
          continue;
        }
        double best_min = std::numeric_limits<double>::max();
        spark_dsg::NodeId best_id = 0;
        for (auto n_id : region.neighbors) {
          const Region& n_region = regions_[n_id];
          if (n_region.max > config.max_place_radius) {
            continue;
          }
          const double min = n_region.max;
          if (min < best_min) {
            best_min = min;
            best_id = n_id;
          }
        }
        if (best_id == 0) {
          continue;
        }
        Region& target = regions_[best_id];
        target.merge(region);
        regions_.erase(id);
        target.computeCentroid(voxel_size);
        target.computeRadii(voxel_size);
        updated = true;
        break;
      }
    }
  }
}

void RegionGrowingTraversabilityClustering::updatePlaceNodesInDsg(
    spark_dsg::DynamicSceneGraph& graph) {
  for (auto& [id, region] : regions_) {
    const auto* node = graph.findNode(id);
    if (!node) {
      // Node does not exist yet, create a new place node.
      auto attrs = std::make_unique<spark_dsg::TraversabilityNodeAttributes>();
      updatePlaceNodeAttributes(*attrs, region);
      graph.emplaceNode(spark_dsg::DsgLayers::TRAVERSABILITY, id, std::move(attrs));
      region.is_new = false;
      continue;
    }

    // Update the place attributes.
    auto& attrs = node->attributes<spark_dsg::TraversabilityNodeAttributes>();
    updatePlaceNodeAttributes(attrs, region);
  }
}

void RegionGrowingTraversabilityClustering::updatePlaceEdgesInDsg(
    spark_dsg::DynamicSceneGraph& graph) const {
  for (const auto& [id, region] : regions_) {
    // Remove edges that should no longer exist.
    const auto* node = graph.findNode(id);
    if (!node) {
      continue;
    }
    NodeIdSet to_remove;
    for (const auto n_id : node->siblings()) {
      if (!region.neighbors.count(n_id) && regions_.count(n_id)) {
        // Do not remove edges to archived regions.
        to_remove.insert(n_id);
      }
    }
    for (const auto n_id : to_remove) {
      graph.removeEdge(id, n_id);
    }

    // Add all new edges.
    for (const auto& n_id : region.neighbors) {
      auto attrs = std::make_unique<spark_dsg::EdgeAttributes>();
      graph.addOrUpdateEdge(id, n_id, std::move(attrs));
    }
  }
}

void RegionGrowingTraversabilityClustering::visualizeAssignments(
    const TraversabilityLayer& layer, const VoxelMap& assigned_voxels) const {
  for (const auto& block : layer) {
    for (size_t i = 0; i < block.voxels_per_side * block.voxels_per_side; ++i) {
      const auto global_index = block.globalFromLocalIndex(block.indexFromLinear(i));
      const auto it = assigned_voxels.find(global_index);
      if (it != assigned_voxels.end()) {
        block.voxels[i].debug_value =
            static_cast<float>(spark_dsg::NodeSymbol(it->second).categoryId());
      } else {
        block.voxels[i].debug_value = -1.0f;
      }
    }
  }
}

void RegionGrowingTraversabilityClustering::updatePlaceNodeAttributes(
    spark_dsg::TraversabilityNodeAttributes& attrs, const Region& region) const {
  // Position and extent.
  attrs.position = region.centroid;
  attrs.boundary.type = spark_dsg::BoundaryType::REGION;
  attrs.boundary.min.x() = region.min;
  attrs.boundary.max.x() = region.max;

  // General attributes.
  attrs.is_active = region.is_active;
  attrs.last_update_time_ns = current_time_ns_;
  if (attrs.first_observed_ns == 0) {
    attrs.first_observed_ns = current_time_ns_;
  }
  attrs.last_observed_ns = current_time_ns_;
}

RegionGrowingTraversabilityClustering::Region&
RegionGrowingTraversabilityClustering::allocateNewRegion() {
  const NodeId id = spark_dsg::NodeSymbol('t', ++current_id_);
  Region& region = regions_[id];
  region.id = id;
  return region;
}

void RegionGrowingTraversabilityClustering::growRegion(const TraversabilityLayer& layer,
                                                       Region& region,
                                                       VoxelMap& assigned_voxels) {
  // TODO(lschmid): This is currently quite inefficient but probably doesn't matter for
  // local windows.
  const float max_radius = config.max_place_radius / layer.voxel_size;
  // Recompute all boundary voxels. We keep a separate queue to get a breadth-first
  // growth.
  std::queue<BlockIndex> to_visit;
  region.boundary_voxels.clear();
  for (const auto& voxel : region.voxels) {
    for (const auto& offset : neighbors_) {
      const BlockIndex n_index = voxel + offset;
      if (region.voxels.count(n_index)) {
        continue;
      }
      region.boundary_voxels.insert(n_index);
      if (assigned_voxels.count(n_index)) {
        continue;
      }
      const auto* n_voxel = layer.voxel(n_index);
      if (n_voxel && n_voxel->state == State::TRAVERSABLE) {
        to_visit.push(n_index);
      }
    }
  }
  region.computeCentroid();

  // Perform region growing until the max size is reached.
  while (!to_visit.empty()) {
    const BlockIndex current_index = to_visit.front();
    to_visit.pop();
    if (assigned_voxels.count(current_index)) {
      continue;
    }
    const Eigen::Vector3d pos = current_index.cast<double>();
    if ((pos - region.centroid).norm() > max_radius) {
      continue;
    }
    region.voxels.insert(current_index);
    region.boundary_voxels.erase(current_index);
    assigned_voxels[current_index] = region.id;
    for (const auto& offset : neighbors_) {
      const BlockIndex n_index = current_index + offset;
      if (region.voxels.count(n_index)) {
        continue;
      }
      region.boundary_voxels.insert(n_index);
      if (assigned_voxels.count(n_index)) {
        continue;
      }
      const auto* n_voxel = layer.voxel(n_index);
      if (n_voxel && n_voxel->state == State::TRAVERSABLE) {
        to_visit.push(n_index);
      }
    }
    region.computeCentroid();
  }
}

void RegionGrowingTraversabilityClustering::updateFrontierVoxels(
    Voxels& frontier_voxels,
    const TraversabilityLayer& layer,
    const Region& region,
    const VoxelMap& assigned_voxels) const {
  // Erase all voxels that are now assigned to this region.
  for (const auto& voxel_index : region.voxels) {
    frontier_voxels.erase(voxel_index);
  }
  // Add all unassigned and traversable boundary voxels to the frontier set.
  for (const auto& voxel_index : region.boundary_voxels) {
    if (assigned_voxels.count(voxel_index)) {
      continue;
    }
    const auto* voxel = layer.voxel(voxel_index);
    if (voxel && voxel->state == State::TRAVERSABLE) {
      frontier_voxels.insert(voxel_index);
    }
  }
}

void RegionGrowingTraversabilityClustering::computeNeighbors(
    const VoxelMap& assigned_voxels) {
  // Compute all edges given by neighboring regions.
  for (auto& [id, region] : regions_) {
    region.neighbors.clear();
    for (const auto& voxel_index : region.boundary_voxels) {
      const auto it = assigned_voxels.find(voxel_index);
      if (it != assigned_voxels.end()) {
        region.neighbors.insert(it->second);
      }
    }
  }
}

void RegionGrowingTraversabilityClustering::Region::merge(const Region& other) {
  voxels.insert(other.voxels.begin(), other.voxels.end());
  boundary_voxels.insert(other.boundary_voxels.begin(), other.boundary_voxels.end());
  auto it = boundary_voxels.begin();
  while (it != boundary_voxels.end()) {
    if (voxels.count(*it)) {
      it = boundary_voxels.erase(it);
    } else {
      ++it;
    }
  }
  neighbors.insert(other.neighbors.begin(), other.neighbors.end());
}

void RegionGrowingTraversabilityClustering::Region::computeCentroid(double voxel_size) {
  centroid = Eigen::Vector3d::Zero();
  for (const auto& voxel : boundary_voxels) {
    centroid += voxel.cast<double>();
  }
  centroid = centroid / boundary_voxels.size() * voxel_size;
}

void RegionGrowingTraversabilityClustering::Region::computeRadii(double voxel_size) {
  min = std::numeric_limits<double>::max();
  max = 0.0;
  for (const auto& voxel : boundary_voxels) {
    const double distance = (voxel.cast<double>() * voxel_size - centroid).norm();
    min = std::min(min, distance);
    max = std::max(max, distance);
  }
}

}  // namespace hydra::places
