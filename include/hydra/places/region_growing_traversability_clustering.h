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
#pragma once

#include <config_utilities/factory.h>
#include <spark_dsg/traversability_boundary.h>

#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "hydra/places/traversability_clustering.h"

namespace hydra::places {

/**
 * @brief Simple clustering that assigns places by block.
 */
class RegionGrowingTraversabilityClustering : public TraversabilityClustering {
 public:
  struct Config {
    //! Maximum radius of a place [m].
    float max_radius = 3.0f;

    //! Number of rays to consider for boundary computation.
    int num_orientation_bins = 16;
  } const config;

  using Voxels = VoxelIndices;
  using VoxelSet = VoxelIndexSet;
  using VoxelMap = VoxelIndexMap<spark_dsg::NodeId>;

  struct Region {
    // ID of the region. This is also the node ID in the DSG.
    spark_dsg::NodeId id;

    // VoxelSet assigned to this region. These can be in the current layer or outside.
    VoxelSet voxels;

    // Voxels bordering the voxels of this region. These are the exterior boundary of
    // the region and are ordered by traversal.
    VoxelSet exterior_boundary;

    // Unobserved or intraversable voxels inside the region.
    VoxelSet interior_boundary;

    // Archived boundary voxels with their last known traversability state.
    VoxelIndexMap<spark_dsg::TraversabilityState> archived_boundary;

    // True: this region has voxels in the current layer. False: all voxels are outside
    // the current layer.
    bool is_active = true;

    // Centroid of the region.
    Eigen::Vector3f centroid;

    // Axis-aligned bounding box of the region in voxel coordinates.
    Eigen::Vector2i min_coordinates;
    Eigen::Vector2i max_coordinates;

    // Regions connecting to this one. <region id, num connecting voxels>
    std::map<spark_dsg::NodeId, int> neighbors;

    // Merge other into this region.
    void merge(const Region& other);

    /**
     * @brief Compute the ordered exterior boundary, min/max extents, and centroid.
     */
    void computeBoundary();

    /**
     * @brief Compute region boundaries, extents, centroids, and compute the neighbors
     * from that.
     */
    void computeNeighbors(const VoxelMap& assigned_voxels);
  };

  RegionGrowingTraversabilityClustering(const Config& config);
  ~RegionGrowingTraversabilityClustering() = default;

  void updateGraph(const TraversabilityLayer& layer,
                   const ActiveWindowOutput& msg,
                   spark_dsg::DynamicSceneGraph& graph) override;

 protected:
  size_t current_id_ = 0;
  uint64_t current_time_ns_ = 0;
  int max_region_size_ = 0;  // Cached region size in voxels.

  // <id, region>
  std::map<spark_dsg::NodeId, Region> regions_;

  // Processing steps.
  /**
   * @brief Prune existing regions by removing all voxels that are no longer
   * traversable. Regions without any remaining voxels are marked as inactive.
   * @return Map of all inactive assigned voxels to region IDs.
  //  */
  VoxelMap initializeRegions(const TraversabilityLayer& layer,
                             const VoxelSet& all_voxels);

  /**
   * @brief Initialize all connected traversable voxels from the starting position of
   * the robot.
   */
  VoxelSet initializeVoxels(const TraversabilityLayer& layer,
                            const Eigen::Vector3d& start_position) const;

  /**
   * @brief Assign all traversable voxels by assigning them to closest existing regions
   * and assigning new ones where needed.
   */
  void growRegions(VoxelSet& all_voxels, VoxelMap& assigned_voxels);

  /**
   * @brief Merge all regions that don't exceed the max size.
   */
  void mergeRegions(const VoxelMap& assigned_voxels,
                    spark_dsg::DynamicSceneGraph& graph);

  void updatePlaceNodesInDsg(spark_dsg::DynamicSceneGraph& graph,
                             const TraversabilityLayer& layer);

  void updatePlaceEdgesInDsg(spark_dsg::DynamicSceneGraph& graph) const;

  void visualizeAssignments(const TraversabilityLayer& layer,
                            const VoxelMap& assigned_voxels) const;

  void pruneRegions();

  /**
   * @brief Allocate a new region, keeping track of the IDs. ID 0 is reserved.
   */
  Region& allocateNewRegion();

  /**
   * @brief Breadth-first search to grow a region from a seed index.
   */
  static VoxelSet growRegion(
      const VoxelSet& candidates,
      const VoxelIndex& seed_index,
      std::function<bool(const VoxelIndex&)> condition = [](const VoxelIndex&) {
        return true;
      });

  void updatePlaceNodeAttributes(spark_dsg::TravNodeAttributes& attrs,
                                 Region& region,
                                 const TraversabilityLayer& layer) const;

  inline static const std::array<VoxelIndex, 8> neighbors_ = {
      VoxelIndex(0, -1, 0),   // bottom
      VoxelIndex(-1, 0, 0),   // left
      VoxelIndex(0, 1, 0),    // top
      VoxelIndex(1, 0, 0),    // right
      VoxelIndex(-1, -1, 0),  // bottom-left
      VoxelIndex(-1, 1, 0),   // top-left
      VoxelIndex(1, 1, 0),    // top-right
      VoxelIndex(1, -1, 0)    // bottom-right
  };
};

void declare_config(RegionGrowingTraversabilityClustering::Config& config);

}  // namespace hydra::places
