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
#include <hydra_topology/gvd_voxel.h>
#include <hydra_topology/voxblox_types.h>
#include <voxblox/core/layer.h>
#include <voxblox/utils/evaluation_utils.h>

namespace voxblox {
namespace utils {

template <>
bool isObservedVoxel(const ::hydra::topology::GvdVoxel& voxel);

template <>
inline bool isObservedVoxel(const ::hydra::topology::GvdVoxel& voxel) {
  return voxel.observed;
}

}  // namespace utils
}  // namespace voxblox

namespace hydra {
namespace topology {
namespace test_helpers {

struct LayerComparisonResult {
  bool valid = false;
  size_t num_same = 0;
  size_t num_different = 0;
  size_t num_lhs_seen_rhs_unseen = 0;
  size_t num_rhs_seen_lhs_unseen = 0;
  size_t num_missing_lhs = 0;
  size_t num_missing_rhs = 0;
  double rmse = 0.0;
  double min_error = std::numeric_limits<double>::infinity();
  double max_error = 0.0;
};

template <typename LhsVoxel, typename RhsVoxel>
size_t getMissingBlocks(const voxblox::Layer<LhsVoxel>& layer,
                        const voxblox::BlockIndexList blocks,
                        const voxblox::Layer<RhsVoxel>& other_layer) {
  size_t num_missing = 0;
  for (const auto idx : blocks) {
    if (other_layer.hasBlock(idx)) {
      continue;
    }

    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      if (voxblox::utils::isObservedVoxel(block.getVoxelByLinearIndex(i))) {
        num_missing++;
      }
    }
  }
  return num_missing;
}

template <typename LhsVoxel, typename RhsVoxel, typename CompareFunc>
LayerComparisonResult compareLayers(const voxblox::Layer<LhsVoxel>& lhs,
                                    const voxblox::Layer<RhsVoxel>& rhs,
                                    const CompareFunc& compare_func) {
  LayerComparisonResult results;
  if (lhs.voxel_size() != rhs.voxel_size()) {
    return results;
  }

  if (lhs.voxels_per_side() != rhs.voxels_per_side()) {
    return results;
  }

  results.valid = true;

  voxblox::BlockIndexList lhs_blocks;
  lhs.getAllAllocatedBlocks(&lhs_blocks);

  voxblox::BlockIndexList rhs_blocks;
  rhs.getAllAllocatedBlocks(&rhs_blocks);

  results.num_missing_lhs = getMissingBlocks(rhs, rhs_blocks, lhs);
  results.num_missing_rhs = getMissingBlocks(lhs, lhs_blocks, rhs);

  for (const auto idx : lhs_blocks) {
    if (!rhs.hasBlock(idx)) {
      continue;
    }

    const auto& lhs_block = lhs.getBlockByIndex(idx);
    const auto& rhs_block = rhs.getBlockByIndex(idx);
    for (size_t i = 0; i < lhs_block.num_voxels(); ++i) {
      bool lhs_observed =
          voxblox::utils::isObservedVoxel(lhs_block.getVoxelByLinearIndex(i));
      bool rhs_observed =
          voxblox::utils::isObservedVoxel(rhs_block.getVoxelByLinearIndex(i));
      if (!lhs_observed && !rhs_observed) {
        results.num_same++;
        continue;
      }

      if (!lhs_observed || !rhs_observed) {
        results.num_rhs_seen_lhs_unseen += (!lhs_observed ? 1 : 0);
        results.num_lhs_seen_rhs_unseen += (!rhs_observed ? 1 : 0);
        continue;
      }

      bool are_same = compare_func(lhs_block.getVoxelByLinearIndex(i),
                                   rhs_block.getVoxelByLinearIndex(i));
      if (are_same) {
        results.num_same++;
      } else {
        results.num_different++;
      }

      double error = lhs_block.getVoxelByLinearIndex(i).distance -
                     rhs_block.getVoxelByLinearIndex(i).distance;
      results.min_error = std::min(results.min_error, std::abs(error));
      results.max_error = std::max(results.max_error, std::abs(error));
      results.rmse += error * error;
    }
  }

  results.rmse /= (results.num_same + results.num_different);
  results.rmse = std::sqrt(results.rmse);
  return results;
}

inline bool gvdEsdfVoxelsSame(const GvdVoxel& lhs, const voxblox::EsdfVoxel& rhs) {
  // TODO(nathan) consider comparing parent to esdf parent direction
  return lhs.distance == rhs.distance && lhs.fixed == rhs.fixed;
}

inline bool gvdVoxelsSame(const GvdVoxel& lhs, const GvdVoxel& rhs) {
  // TODO(nathan) consider comparing parent to esdf parent direction
  return lhs.distance == rhs.distance && lhs.fixed == rhs.fixed;
}

inline bool esdfVoxelsSame(const voxblox::EsdfVoxel& lhs,
                           const voxblox::EsdfVoxel& rhs) {
  return lhs.distance == rhs.distance && lhs.fixed == rhs.fixed &&
         lhs.parent == rhs.parent;
}

inline std::ostream& operator<<(std::ostream& out,
                                const LayerComparisonResult& result) {
  if (!result.valid) {
    out << "Invalid result!";
    return out;
  }

  out << "Comparison Results:" << std::endl;
  out << " - " << result.num_same << " same, " << result.num_different << " different"
      << std::endl;
  out << " - " << result.num_missing_lhs << " / " << result.num_missing_rhs
      << " unallocated (lhs / rhs) " << std::endl;
  out << " - " << result.num_lhs_seen_rhs_unseen << " / "
      << result.num_rhs_seen_lhs_unseen << " uniquely seen (lhs / rhs)" << std::endl;
  out << " - " << result.rmse << " rmse -> [" << result.min_error << ", "
      << result.max_error << "]";
  return out;
}

}  // namespace test_helpers
}  // namespace topology
}  // namespace hydra
