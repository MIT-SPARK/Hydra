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

#include <queue>

#include "hydra/utils/timing_utilities.h"

namespace hydra::places {

using Timer = hydra::timing::ScopedTimer;
using spark_dsg::Boundary;
using spark_dsg::Side;
using spark_dsg::TraversabilityNodeAttributes;

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
  field(config.maximum_place_size, "maximum_place_size");
  check(config.maximum_place_size, GT, 0, "maximum_place_size");
}

RegionGrowingTraversabilityClustering::RegionGrowingTraversabilityClustering(
    const Config& config)
    : config(config::checkValid(config)) {}

void RegionGrowingTraversabilityClustering::updateGraph(
    const TraversabilityLayer& layer,
    const ActiveWindowOutput& msg,
    spark_dsg::DynamicSceneGraph& graph) {
  // Compute all updated places and store them in the place_infos.
  current_time_ns_ = msg.timestamp_ns;

  detectPlaces(layer, msg.world_t_body);
  // computePlaces();
  // classifyPlaceBoundaries();
  // updatePlaceNodesInDsg(graph);
  // updatePlaceEdgesInDsg(graph);
  // archivePlaceInfos(graph);
}

void RegionGrowingTraversabilityClustering::detectPlaces(
    const TraversabilityLayer& layer, const Eigen::Vector3d& start_position) {
  // Update all regions by checking whether they are active and still traversable.
  auto occupied_voxels = pruneExistingRegions(layer);

  // If empty start with the current robot position. --> What if intraversable?
  if (regions_.empty()) {
    auto& seed_region = regions_[++current_id_];
    Eigen::Vector3f start_2d = start_position.cast<float>();
    start_2d.z() = 0.0f;
    const auto start_index = layer.globalIndexFromPoint(start_2d);
    seed_region.voxels.insert(start_index);
    occupied_voxels[start_index] = current_id_;
  }

  // Seed with previous regions.

  // Assign all unassigned traversable voxels to (new) regions.
}

RegionGrowingTraversabilityClustering::RegionVoxels
RegionGrowingTraversabilityClustering::pruneExistingRegions(
    const TraversabilityLayer& layer) {
  // Prune all voxels that are no longer traversable.
  std::vector<int> to_remove;
  for (auto& [id, region] : regions_) {
    auto it = region.voxels.begin();
    while (it != region.voxels.end()) {
      // const auto* voxel = layer.voxel(*it);
      // if (!voxel || voxel->state != State::TRAVERSABLE) {
      //   it = region.voxels.erase(it);
      // } else {
      //   ++it;
      // }
    }
    if (region.voxels.empty()) {
      to_remove.push_back(id);
    }
  }
  // Remove empty regions.
  for (const auto id : to_remove) {
    regions_.erase(id);
  }
  return {};
}

}  // namespace hydra::places
