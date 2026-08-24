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
#include "hydra/eval/place_metrics.h"

#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>

#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra::eval {

using places::GvdLayer;
using spark_dsg::NodeId;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SceneGraph;

void fillGvdPositions(const GvdLayer& layer,
                      size_t min_gvd_basis,
                      std::vector<Eigen::Vector3f>& result) {
  result.clear();

  for (const auto& block : layer) {
    for (size_t i = 0; i < block.numVoxels(); ++i) {
      const auto& voxel = block.getVoxel(i);
      if (!voxel.observed || voxel.num_extra_basis < min_gvd_basis) {
        continue;
      }

      result.push_back(block.getVoxelPosition(i));
    }
  }
}

PlaceMetrics scorePlaces(const SceneGraph& graph,
                         const GvdLayer& gvd,
                         size_t min_gvd_basis,
                         const std::string& layer_id) {
  PlaceMetrics metrics;
  const auto places = graph.findLayer(layer_id);
  if (!places) {
    return metrics;
  }

  metrics.is_valid = true;
  std::vector<Eigen::Vector3f> gvd_positions;
  fillGvdPositions(gvd, min_gvd_basis, gvd_positions);
  const PointNeighborSearch finder(gvd_positions);

  for (auto&& [node_id, node] : places->nodes()) {
    const auto& attrs = node->attributes<PlaceNodeAttributes>();
    metrics.node_order.push_back(node_id);

    size_t idx = 0;
    float dist_squared = 0.0;
    const Point pos = attrs.position.cast<float>();
    finder.search(pos, dist_squared, idx);
    metrics.node_gvd_distances.push_back(std::sqrt(dist_squared));

    const auto* voxel = gvd.getVoxelPtr(pos);
    if (!voxel) {
      metrics.num_missing++;
      continue;
    }

    if (!voxel->observed) {
      metrics.num_unobserved++;
      continue;
    }

    metrics.num_valid++;
    metrics.gvd_distance_errors.push_back(std::abs(voxel->distance - attrs.distance));
  }

  return metrics;
}

}  // namespace hydra::eval
