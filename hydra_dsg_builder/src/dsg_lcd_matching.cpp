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
#include "hydra_dsg_builder/dsg_lcd_matching.h"

#include <glog/logging.h>

namespace hydra {
namespace lcd {

using Dsg = DynamicSceneGraph;
using DsgNode = DynamicSceneGraphNode;

float computeDistanceHist(const Descriptor& lhs,
                          const Descriptor& rhs,
                          const std::function<float(float, float)>& distance_func) {
  CHECK_EQ(lhs.values.rows(), rhs.values.rows());

  float score = 0.0;
  for (int r = 0; r < lhs.values.rows(); ++r) {
    // see https://ieeexplore.ieee.org/document/1641018 for derivation,
    // but this needs to match the BoW based distance
    if (lhs.values(r, 0) == 0.0 || rhs.values(r, 0) == 0.0) {
      continue;
    }

    score += distance_func(lhs.values(r, 0), rhs.values(r, 0));
  }

  return score;
}

float computeDistanceBow(const Descriptor& lhs,
                         const Descriptor& rhs,
                         const std::function<float(float, float)>& distance_func) {
  CHECK_EQ(lhs.values.rows(), lhs.words.rows());
  CHECK_EQ(rhs.values.rows(), rhs.words.rows());
  float score = 0.0;
  int r1 = 0;
  int r2 = 0;
  while (r1 < lhs.values.rows() && r2 < rhs.values.rows()) {
    const uint32_t word1 = lhs.words(r1, 0);
    const uint32_t word2 = rhs.words(r2, 0);

    if (word1 == word2) {
      score += distance_func(lhs.values(r1, 0), rhs.values(r2, 0));
      ++r1;
      ++r2;
    } else if (word1 < word2) {
      ++r1;
    } else {
      ++r2;
    }
  }

  return score;
}

float computeDistance(const Descriptor& lhs,
                      const Descriptor& rhs,
                      const std::function<float(float, float)>& distance_func) {
  if (!lhs.words.size() && !rhs.words.size()) {
    return computeDistanceHist(lhs, rhs, distance_func);
  } else {
    return computeDistanceBow(lhs, rhs, distance_func);
  }
}

float computeDescriptorScore(const Descriptor& lhs,
                             const Descriptor& rhs,
                             DescriptorScoreType type) {
  switch (type) {
    case DescriptorScoreType::COSINE:
      // map [-1, 1] to [0, 1]
      return 0.5f * computeCosineDistance(lhs, rhs) + 0.5f;
    case DescriptorScoreType::L1:
    default:
      // map [2, 0] to [0, 1]
      return 1.0f - 0.5f * computeL1Distance(lhs, rhs);
  }
}

LayerSearchResults searchDescriptors(
    const Descriptor& descriptor,
    const DescriptorMatchConfig& match_config,
    const std::set<NodeId>& valid_matches,
    const DescriptorCache& descriptors,
    const std::map<NodeId, std::set<NodeId>>& root_leaf_map,
    NodeId query_id) {
  float best_score = 0.0f;
  std::vector<std::pair<NodeId, float>> new_valid_match_scores;
  std::set<NodeId> new_valid_matches;

  size_t num_same_parent = 0;
  size_t num_inside_horizon = 0;
  size_t num_low_score = 0;

  VLOG(10) << "--------------------------------------------------";

  for (const auto& valid_id : valid_matches) {
    if (root_leaf_map.at(valid_id).count(query_id)) {
      ++num_same_parent;
      continue;
    }

    const Descriptor& other_descriptor = *descriptors.at(valid_id);
    std::chrono::duration<double> diff_s =
        descriptor.timestamp - other_descriptor.timestamp;
    VLOG(10) << "diff: " << diff_s.count()
             << " (threshold: " << match_config.min_time_separation_s << ")";
    if (diff_s.count() < match_config.min_time_separation_s) {
      ++num_inside_horizon;
      continue;
    }

    const float curr_score =
        computeDescriptorScore(descriptor, other_descriptor, match_config.type);
    if (curr_score > best_score) {
      best_score = curr_score;
    }

    if (curr_score > match_config.min_score) {
      new_valid_matches.insert(valid_id);
      new_valid_match_scores.push_back({valid_id, curr_score});
    } else {
      ++num_low_score;
    }
  }

  // TODO(nathan) add layer id in again or handle stats better
  VLOG(1) << "matching "
          << " -> shared: " << num_same_parent << ", horizon: " << num_inside_horizon
          << ", low: " << num_low_score << ", valid: " << new_valid_match_scores.size();

  std::sort(new_valid_match_scores.begin(),
            new_valid_match_scores.end(),
            [](const std::pair<NodeId, float>& a, const std::pair<NodeId, float>& b) {
              return a.second > b.second;
            });
  std::vector<std::set<NodeId>> match_nodes;
  std::vector<NodeId> matches;
  std::vector<float> match_scores;
  for (const auto& id_score : new_valid_match_scores) {
    if (id_score.second < match_config.min_registration_score) {
      break;
    }

    if (id_score.second > best_score * match_config.min_score_ratio) {
      bool spatialy_distinct = true;
      for (const auto& m : matches) {
        if ((descriptors.at(id_score.first)->root_position -
             descriptors.at(m)->root_position)
                .norm() < match_config.min_match_separation_m) {
          spatialy_distinct = false;
          break;
        }
      }

      if (!spatialy_distinct) {
        continue;
      }

      match_nodes.push_back(descriptors.at(id_score.first)->nodes);
      matches.push_back(id_score.first);
      match_scores.push_back(id_score.second);
    }
    if (matches.size() == match_config.max_registration_matches) {
      break;
    }
  }

  return {match_scores,
          new_valid_matches,
          descriptor.nodes,
          match_nodes,
          descriptor.root_node,
          matches};
}

LayerSearchResults searchLeafDescriptors(const Descriptor& descriptor,
                                         const DescriptorMatchConfig& match_config,
                                         const std::set<NodeId>& valid_matches,
                                         const DescriptorCacheMap& leaf_cache_map,
                                         NodeId query_id) {
  float best_score = 0.0f;
  NodeId best_node = 0;  // gets overwritten on valid match
  NodeId best_root = 0;  // gets overwritten on valid match
  bool found_match = false;
  for (const auto& valid_id : valid_matches) {
    const DescriptorCache& leaf_cache = leaf_cache_map.at(valid_id);

    for (const auto& id_desc_pair : leaf_cache) {
      if (id_desc_pair.first == query_id) {
        continue;  // disallow self matches even if they probably can't happen
      }

      const Descriptor& other_descriptor = *id_desc_pair.second;
      std::chrono::duration<double> diff_s =
          descriptor.timestamp - other_descriptor.timestamp;
      if (diff_s.count() < match_config.min_time_separation_s) {
        continue;
      }

      const float curr_score =
          computeDescriptorScore(descriptor, other_descriptor, match_config.type);

      if (curr_score > best_score) {
        best_score = curr_score;
        best_node = id_desc_pair.first;
        best_root = other_descriptor.root_node;
        found_match = true;
      }
    }
  }

  if (!found_match) {
    return {{}, {}, descriptor.nodes, {}, descriptor.root_node, {}};
  }

  return {{best_score},
          {best_node},
          descriptor.nodes,
          {{best_node}},
          descriptor.root_node,
          {best_root}};
}

}  // namespace lcd
}  // namespace hydra
