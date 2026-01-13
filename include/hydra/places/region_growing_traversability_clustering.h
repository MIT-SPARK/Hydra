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
    //! Approximate maximum radius when growing regions in meters.
    float max_place_radius = 2.0f;

    float min_place_radius = 1.0f;
  } const config;

  using VoxelMap = BlockIndexMap<spark_dsg::NodeId>;
  using Voxels = BlockIndexSet;

  // TODO(lschmid): For now a simple data structure and algorithm, consdier aligning
  // better with blocks in the future and make more efficient.
  struct Region {
    // ID of the region. This is also the node ID in the DSG.
    spark_dsg::NodeId id;

    // Voxels assigned to this region. These can be in the current layer or outside.
    Voxels voxels;

    // Voxels bordering the voxels of this region.
    Voxels boundary_voxels;

    // True: this region has voxels in the current layer. False: all voxels are outside
    // the current layer.
    bool is_active = true;

    // True: Created this iteration. False: is in the scene graph.
    bool is_new = true;

    // Centroid of the region.
    Eigen::Vector3d centroid;

    // Regions connecting to this one.
    NodeIdSet neighbors;

    // Radii.
    double min;
    double max;

    // Merge other into this region.
    void merge(const Region& other);

    void computeCentroid(double voxel_size = 1.0);

    // Min-Max distances w.r.t. the centroid.
    void computeRadii(double voxel_size = 1.0);

    // Recompute all voxels bordering the voxels of this region.
    void computeBoundaryVoxels();
  };

  RegionGrowingTraversabilityClustering(const Config& config);
  ~RegionGrowingTraversabilityClustering() = default;

  void updateGraph(const TraversabilityLayer& layer,
                   const ActiveWindowOutput& msg,
                   spark_dsg::DynamicSceneGraph& graph) override;

 protected:
  size_t current_id_ = 0;
  uint64_t current_time_ns_ = 0;

  // <id, region>
  std::map<spark_dsg::NodeId, Region> regions_;

  // Processing steps.
  VoxelMap pruneExistingRegions(const TraversabilityLayer& layer,
                                spark_dsg::DynamicSceneGraph& graph);

  void initializeRegions(const TraversabilityLayer& layer,
                         const Eigen::Vector3d& start_position,
                         VoxelMap& assigned_voxels);

  void detectPlaces(const TraversabilityLayer& layer, VoxelMap& assigned_voxels);

  void simplifyRegions(double voxel_size);

  void updatePlaceNodesInDsg(spark_dsg::DynamicSceneGraph& graph);

  void updatePlaceEdgesInDsg(spark_dsg::DynamicSceneGraph& graph) const;

  void visualizeAssignments(const TraversabilityLayer& layer,
                            const VoxelMap& assigned_voxels) const;

  /* Helper functions */
  Region& allocateNewRegion();

  // Grow the region until the max is reached, also recomputing the frontier voxels.
  void growRegion(const TraversabilityLayer& layer,
                  Region& region,
                  VoxelMap& assigned_voxels);

  void updateFrontierVoxels(Voxels& frontier_voxels,
                            const TraversabilityLayer& layer,
                            const Region& region,
                            const VoxelMap& assigned_voxels) const;

  void updatePlaceNodeAttributes(spark_dsg::TraversabilityNodeAttributes& attrs,
                                 const Region& region) const;

  void computeNeighbors(const VoxelMap& assigned_voxels);

  double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;

  inline static const std::array<BlockIndex, 8> neighbors_ = {
      BlockIndex(0, -1, 0),   // bottom
      BlockIndex(-1, 0, 0),   // left
      BlockIndex(0, 1, 0),    // top
      BlockIndex(1, 0, 0),    // right
      BlockIndex(-1, -1, 0),  // bottom-left
      BlockIndex(1, -1, 0),   // bottom-right
      BlockIndex(-1, 1, 0),   // top-left
      BlockIndex(1, 1, 0)     // top-right
  };
};

void declare_config(RegionGrowingTraversabilityClustering::Config& config);

}  // namespace hydra::places
