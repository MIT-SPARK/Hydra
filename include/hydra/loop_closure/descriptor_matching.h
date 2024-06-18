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
#include "hydra/loop_closure/scene_graph_descriptors.h"

namespace hydra::lcd {

enum class DescriptorScoreType { COSINE, L1 };

struct DescriptorMatchConfig {
  float min_score = 1.0;
  float min_registration_score = 1.0;
  double min_time_separation_s = 0.0;
  size_t max_registration_matches = 5;
  double min_score_ratio = 0.7;
  double min_match_separation_m = 5.0;
  DescriptorScoreType type = DescriptorScoreType::L1;
};

struct LayerSearchResults {
  std::vector<float> score;
  std::set<NodeId> valid_matches;
  std::set<NodeId> query_nodes;
  std::vector<std::set<NodeId>> match_nodes;
  NodeId query_root;
  std::vector<NodeId> match_root;
};

using DescriptorCache = std::map<NodeId, Descriptor::Ptr>;
using DescriptorCacheMap = std::map<NodeId, DescriptorCache>;

float computeDistance(const Descriptor& lhs,
                      const Descriptor& rhs,
                      const std::function<float(float, float)>& distance_func);

inline float computeCosineDistance(const Descriptor& lhs, const Descriptor& rhs) {
  float lhs_scale = lhs.normalized ? 1.0f : lhs.values.norm();
  float rhs_scale = rhs.normalized ? 1.0f : rhs.values.norm();

  if (lhs_scale == 0.0f && rhs_scale == 0.0f) {
    return 1.0f;
  }

  float scale = lhs_scale * rhs_scale;
  // TODO(nathan) we might want a looser check than this
  if (scale == 0.0f) {
    scale = 1.0f;  // force all zero descriptors to have 0 norm (instead of nan)
  }

  return computeDistance(
      lhs, rhs, [&scale](float lhs, float rhs) { return (lhs * rhs) / scale; });
}

inline float computeL1Distance(const Descriptor& lhs, const Descriptor& rhs) {
  float lhs_scale = lhs.normalized ? 1.0f : lhs.values.lpNorm<1>();
  float rhs_scale = rhs.normalized ? 1.0f : rhs.values.lpNorm<1>();

  if (rhs_scale == 0.0f and lhs_scale == 0.0f) {
    return 0.0f;
  }

  lhs_scale = lhs_scale == 0.0f ? 1.0f : lhs_scale;
  rhs_scale = rhs_scale == 0.0f ? 1.0f : rhs_scale;

  const float l1_diff = computeDistance(lhs, rhs, [&](float lhs, float rhs) {
    const float lhs_val = lhs / lhs_scale;
    const float rhs_val = rhs / rhs_scale;
    return std::abs(lhs_val - rhs_val) - std::abs(lhs_val) - std::abs(rhs_val);
  });
  return 2.0f + l1_diff;
}

float computeDescriptorScore(const Descriptor& lhs,
                             const Descriptor& rhs,
                             DescriptorScoreType type);

LayerSearchResults searchDescriptors(
    const Descriptor& descriptor,
    const DescriptorMatchConfig& match_config,
    const std::set<NodeId>& valid_matches,
    const DescriptorCache& descriptors,
    const std::map<NodeId, std::set<NodeId>>& root_leaf_map,
    NodeId query_id);

LayerSearchResults searchLeafDescriptors(const Descriptor& descriptor,
                                         const DescriptorMatchConfig& match_config,
                                         const std::set<NodeId>& valid_matches,
                                         const DescriptorCacheMap& leaf_cache_map,
                                         NodeId query_id);

}  // namespace hydra::lcd
