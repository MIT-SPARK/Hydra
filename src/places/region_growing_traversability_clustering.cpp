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
#include <hydra/utils/nearest_neighbor_utilities.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <algorithm>

#include "hydra/utils/timing_utilities.h"

namespace hydra::places {

using Timer = hydra::timing::ScopedTimer;
using spark_dsg::Boundary;
using spark_dsg::NodeId;
using spark_dsg::Side;
using spark_dsg::TraversabilityNodeAttributes;
using State = spark_dsg::TraversabilityState;
using VoxelSet = RegionGrowingTraversabilityClustering::VoxelSet;
using VoxelQueue = RegionGrowingTraversabilityClustering::VoxelQueue;
using VoxelMap = RegionGrowingTraversabilityClustering::VoxelMap;

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
  field(config.max_size, "max_size", "m");
  check(config.max_size, GT, 0.0f, "max_size");
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

  // Cache params for this pass.
  current_time_ns_ = msg.timestamp_ns;
  max_region_size_ = std::round(config.max_size / layer.voxel_size);

  // Initialize regions and voxels for this pass.
  VoxelSet all_voxels = initializeVoxels(layer, msg.world_t_body);
  if (all_voxels.empty()) {
    return;
  }
  VoxelMap assignment = initializeRegions(layer, all_voxels);

  // Run voxel clustering.
  growRegions(all_voxels, assignment);

  // Compute neighbors and simplification.
  computeNeighbors(assignment);
  mergeRegions(graph);

  // Update the DSG.
  updatePlaceNodesInDsg(graph, layer);
  updatePlaceEdgesInDsg(graph);
  visualizeAssignments(layer, assignment);
  pruneRegions();
}

VoxelSet RegionGrowingTraversabilityClustering::initializeVoxels(
    const TraversabilityLayer& layer, const Eigen::Vector3d& start_position) const {
  VoxelSet candidates;
  for (const auto& block : layer) {
    for (size_t i = 0; i < block.voxels.size(); ++i) {
      const auto voxel_index = block.globalFromLocalIndex(block.indexFromLinear(i));
      if (block.voxels[i].state == State::TRAVERSABLE) {
        candidates.insert(voxel_index);
      }
    }
  }

  // Start index from the current robot position.
  Eigen::Vector3f start_2d = start_position.cast<float>();
  start_2d.z() = 0.0f;
  const auto start_index = layer.globalIndexFromPoint(start_2d);
  return growRegion(candidates, start_index);
}

VoxelMap RegionGrowingTraversabilityClustering::initializeRegions(
    const TraversabilityLayer& layer, const VoxelSet& all_voxels) {
  // Assign all archived voxels to their regions and remove others.
  VoxelMap assigned_voxels;
  for (auto& [id, region] : regions_) {
    VoxelSet inactive_voxels;
    for (const auto& voxel_index : region.voxels) {
      const auto* voxel = layer.voxel(voxel_index);
      if (!voxel) {
        inactive_voxels.insert(voxel_index);
        assigned_voxels[voxel_index] = id;
      }
    }
    region.voxels = std::move(inactive_voxels);
    region.is_active = false;
    region.is_new = false;
  }

  // Initialize a new region if none exist.
  if (!all_voxels.empty() && regions_.empty()) {
    Region& region = allocateNewRegion();
    region.centroid = all_voxels.begin()->cast<float>();
  }
  return assigned_voxels;
}

void RegionGrowingTraversabilityClustering::growRegions(VoxelSet& all_voxels,
                                                        VoxelMap& assigned_voxels) {
  // Try to grow existing regions into the closest voxels to each region.
  const float max_dist_sq = static_cast<float>(max_region_size_ * max_region_size_);
  std::vector<Eigen::Vector3f> centroids;
  std::vector<NodeId> region_ids;
  for (const auto& [id, region] : regions_) {
    centroids.push_back(region.centroid);
    region_ids.push_back(id);
  }
  auto nn = std::make_unique<hydra::PointNeighborSearch>(centroids);
  std::map<NodeId, VoxelSet> region_candidates;
  for (const auto& voxel_index : all_voxels) {
    size_t nn_index;
    float nn_dist_sq;
    const Eigen::Vector3f voxel_pos = voxel_index.cast<float>();
    if (nn->search(voxel_pos, nn_dist_sq, nn_index) && nn_dist_sq <= max_dist_sq) {
      const NodeId nn_region_id = region_ids[nn_index];
      region_candidates[nn_region_id].insert(voxel_index);
    }
  }

  // Grow each region from its candidates.
  for (auto& [region_id, candidates] : region_candidates) {
    Region& region = regions_.at(region_id);
    region.is_active = true;
    VoxelSet grown_voxels = growRegion(candidates, *candidates.begin());
    region.voxels.insert(grown_voxels.begin(), grown_voxels.end());
    for (const auto& voxel_index : grown_voxels) {
      assigned_voxels[voxel_index] = region_id;
      all_voxels.erase(voxel_index);
    }
  }

  // Grow new regions from remaining unassigned voxels.
  while (!all_voxels.empty()) {
    Region& new_region = allocateNewRegion();
    new_region.is_active = true;
    const auto seed_index = *all_voxels.begin();
    new_region.centroid = seed_index.cast<float>();
    VoxelSet grown_voxels =
        growRegion(all_voxels, seed_index, [&](const BlockIndex& index) {
          return (index.cast<float>() - new_region.centroid).squaredNorm() <=
                 max_dist_sq;
        });
    new_region.voxels = std::move(grown_voxels);
    for (const auto& voxel_index : new_region.voxels) {
      assigned_voxels[voxel_index] = new_region.id;
      all_voxels.erase(voxel_index);
    }
  }
}

void RegionGrowingTraversabilityClustering::computeNeighbors(
    const VoxelMap& assigned_voxels) {
  // Compute all edges given by neighboring regions.
  for (auto& [id, region] : regions_) {
    region.neighbors.clear();
    region.computeBoundaryVoxels();
    region.computeCentroid();
    for (const auto& voxel_index : region.boundary_voxels) {
      const auto it = assigned_voxels.find(voxel_index);
      if (it != assigned_voxels.end()) {
        region.neighbors.insert(it->second);
      }
    }
  }
}

void RegionGrowingTraversabilityClustering::mergeRegions(
    spark_dsg::DynamicSceneGraph& graph) {
  // Merge all regions that don't exceed the max size.
  bool merged = true;
  while (merged) {
    merged = false;
    for (auto it = regions_.begin(); it != regions_.end(); ++it) {
      Region& region = it->second;
      if (!region.is_active) {
        continue;
      }
      for (const auto neighbor_id : region.neighbors) {
        auto neighbor_it = regions_.find(neighbor_id);
        if (neighbor_it == regions_.end()) {
          continue;
        }
        Region& neighbor_region = neighbor_it->second;
        if (!neighbor_region.is_active) {
          continue;
        }
        // Check size constraint.
        const Eigen::Vector2i combined_min =
            region.min_coordinates.cwiseMin(neighbor_region.min_coordinates);
        const Eigen::Vector2i combined_max =
            region.max_coordinates.cwiseMax(neighbor_region.max_coordinates);
        if ((combined_max - combined_min).maxCoeff() <= max_region_size_) {
          // Merge neighbor into this region.
          region.merge(neighbor_region);
          regions_.erase(neighbor_it);
          graph.removeNode(neighbor_id);
          merged = true;
          break;
        }
      }
      if (merged) {
        break;
      }
    }
  }
}

void RegionGrowingTraversabilityClustering::updatePlaceNodesInDsg(
    spark_dsg::DynamicSceneGraph& graph, const TraversabilityLayer& layer) {
  for (auto& [id, region] : regions_) {
    const bool is_valid = region.voxels.size() > 0;
    const auto* node = graph.findNode(id);

    if (!node) {
      // Node does not exist yet, create a new place node.
      if (is_valid) {
        auto attrs = std::make_unique<spark_dsg::TraversabilityNodeAttributes>();
        updatePlaceNodeAttributes(*attrs, region, layer);
        graph.emplaceNode(spark_dsg::DsgLayers::TRAVERSABILITY, id, std::move(attrs));
      }
      continue;
    }

    if (!is_valid) {
      // Node exists but is no longer valid, remove it.
      graph.removeNode(id);
      continue;
    }

    // Update the place attributes.
    auto& attrs = node->attributes<spark_dsg::TraversabilityNodeAttributes>();
    updatePlaceNodeAttributes(attrs, region, layer);
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
      // Do not remove edges to archived regions.
      if (!region.neighbors.count(n_id) && regions_.count(n_id)) {
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

void RegionGrowingTraversabilityClustering::pruneRegions() {
  // Remove all regions that are inactive and have no voxels assigned.
  auto it = regions_.begin();
  while (it != regions_.end()) {
    const Region& region = it->second;
    if (!region.is_active) {
      it = regions_.erase(it);
    } else {
      ++it;
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
    spark_dsg::TraversabilityNodeAttributes& attrs,
    const Region& region,
    const TraversabilityLayer& layer) const {
  // Position.
  attrs.position = region.centroid.cast<double>() * layer.voxel_size;
  attrs.boundary.type = spark_dsg::BoundaryType::REGION;

  // Min and max extents as radii.
  float min = std::numeric_limits<float>::max();
  float max = 0.0f;
  for (const auto& voxel : region.boundary_voxels) {
    const float distance = (voxel.cast<float>() - region.centroid).norm();
    min = std::min(min, distance);
    max = std::max(max, distance);
  }
  attrs.boundary.min.x() = min * layer.voxel_size;
  attrs.boundary.max.x() = max * layer.voxel_size;

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

VoxelSet RegionGrowingTraversabilityClustering::growRegion(
    const VoxelSet& candidates,
    const BlockIndex& seed_index,
    std::function<bool(const BlockIndex&)> condition) const {
  VoxelSet result;
  if (candidates.find(seed_index) == candidates.end()) {
    return result;
  }
  VoxelQueue queue;
  queue.push(seed_index);
  result.insert(seed_index);

  // Breadth-first search to find all connected traversable voxels.
  while (!queue.empty()) {
    const auto current_index = queue.front();
    queue.pop();
    for (const auto& offset : neighbors_) {
      const BlockIndex n_index = current_index + offset;
      if (candidates.find(n_index) == candidates.end() || !condition(n_index)) {
        continue;
      }
      if (result.insert(n_index).second) {
        queue.push(n_index);
      }
    }
  }
  return result;
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

void RegionGrowingTraversabilityClustering::Region::computeCentroid() {
  centroid = Eigen::Vector3f::Zero();
  min_coordinates = Eigen::Vector2i::Constant(std::numeric_limits<int>::max());
  max_coordinates = Eigen::Vector2i::Constant(std::numeric_limits<int>::lowest());
  for (const auto& voxel : boundary_voxels) {
    centroid += voxel.cast<float>();
    min_coordinates = min_coordinates.cwiseMin(voxel.head<2>());
    max_coordinates = max_coordinates.cwiseMax(voxel.head<2>());
  }
  centroid = centroid / boundary_voxels.size();
}

void RegionGrowingTraversabilityClustering::Region::computeBoundaryVoxels() {
  boundary_voxels.clear();
  for (const auto& voxel : voxels) {
    for (const auto& offset : neighbors_) {
      const BlockIndex n_index = voxel + offset;
      if (voxels.count(n_index)) {
        continue;
      }
      boundary_voxels.insert(n_index);
    }
  }
}

}  // namespace hydra::places
