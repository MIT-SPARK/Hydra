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
#include "hydra/loop_closure/descriptor_matching.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <glog/logging.h>

#include "hydra/common/common.h"
#include "hydra/utils/display_utilities.h"

namespace hydra::lcd {

using Dsg = DynamicSceneGraph;
using DsgNode = DynamicSceneGraphNode;
using RootLeafMap = std::map<NodeId, std::set<NodeId>>;

void declare_config(DescriptorMatchConfig& conf) {
  using namespace config;
  name("DescriptorMatchConfig");
  field(conf.min_score, "min_score");
  field(conf.min_registration_score, "min_registration_score");
  field(conf.min_time_separation_s, "min_time_separation_s");
  field(conf.max_registration_matches, "max_registration_matches");
  field(conf.min_score_ratio, "min_score_ratio");
  field(conf.min_match_separation_m, "min_match_separation_m");
  enum_field(
      conf.type,
      "type",
      {{DescriptorScoreType::COSINE, "COSINE"}, {DescriptorScoreType::L1, "L1"}});
}

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

struct ScoredMatch {
  ScoredMatch(NodeId leaf_id, NodeId parent_id, float score)
      : leaf_id(leaf_id), parent_id(parent_id), score(score) {}

  ScoredMatch(NodeId parent_id, float score)
      : leaf_id(parent_id), parent_id(parent_id), score(score) {}

  NodeId leaf_id;
  NodeId parent_id;
  float score;
};

struct MatchStatistics {
  size_t num_empty = 0;
  size_t num_same_parent = 0;
  size_t num_inside_horizon = 0;
  size_t num_low_score = 0;
  size_t num_null = 0;
  size_t num_default_match = 0;
  size_t num_shared_nodes = 0;

  std::string toString() const {
    std::stringstream ss;
    ss << "shared: " << num_same_parent << ", null: " << num_null
       << ", horizon: " << num_inside_horizon << ", low: " << num_low_score
       << ", default: " << num_default_match << ", shared: " << num_shared_nodes
       << ", empty: " << num_empty;
    return ss.str();
  }
};

LayerSearchResults searchDescriptors(const Descriptor& descriptor,
                                     const DescriptorMatchConfig& match_config,
                                     const std::set<NodeId>& valid_matches,
                                     const DescriptorCache& descriptors,
                                     const RootLeafMap& root_leaf_map,
                                     NodeId query_id) {
  VLOG(VLEVEL_DEBUG) << "--------------------------------------------------";
  MatchStatistics stats;
  LayerSearchResults results;
  results.query_nodes = descriptor.nodes;
  results.query_root = descriptor.root_node;

  float best_score = 0.0f;
  std::vector<ScoredMatch> new_scores;
  for (const auto valid_id : valid_matches) {
    const auto& leaves = root_leaf_map.at(valid_id);
    if (leaves.count(query_id)) {
      ++stats.num_same_parent;
      continue;
    }

    const auto& other_ptr = descriptors.at(valid_id);
    if (!other_ptr) {
      stats.num_null++;
      continue;
    }

    bool same_robot = true;
    if (!leaves.empty()) {
      const auto first_child = *leaves.begin();
      same_robot =
          NodeSymbol(query_id).category() == NodeSymbol(first_child).category();
      if (!same_robot) {
        VLOG(VLEVEL_DEBUG) << "Found different robot: query " << printNodeId(query_id)
                           << ", putative: " << printNodeId(first_child);
      }
    }

    const Descriptor& other = *other_ptr;
    std::chrono::duration<double> diff_s = descriptor.timestamp - other.timestamp;
    if (same_robot && diff_s.count() < match_config.min_time_separation_s) {
      VLOG(VLEVEL_DEBUG) << "diff: " << diff_s.count()
                         << " (threshold: " << match_config.min_time_separation_s
                         << ")";
      ++stats.num_inside_horizon;
      continue;
    }

    if (descriptor.is_null || other.is_null) {
      ++stats.num_default_match;
      results.valid_matches.insert(valid_id);
      new_scores.push_back({valid_id, -1.0});
      continue;
    }

    size_t curr_shared = 0;
    for (const auto& id : descriptor.nodes) {
      if (other.nodes.count(id)) {
        ++curr_shared;
      }
    }

    if (curr_shared > 0) {
      ++stats.num_shared_nodes;
      continue;
    }

    const float score = computeDescriptorScore(descriptor, other, match_config.type);
    if (score > best_score) {
      best_score = score;
    }

    if (score > match_config.min_score) {
      results.valid_matches.insert(valid_id);
      new_scores.push_back({valid_id, score});
    } else {
      ++stats.num_low_score;
    }
  }

  VLOG(VLEVEL_DEBUG) << "matching -> " << stats.toString()
                     << ", valid: " << new_scores.size();
  std::sort(new_scores.begin(), new_scores.end(), [](const auto& a, const auto& b) {
    return a.score > b.score;
  });

  for (const auto& match : new_scores) {
    if (match.score < match_config.min_registration_score) {
      break;
    }

    if (match.score > best_score * match_config.min_score_ratio) {
      bool spatialy_distinct = true;
      const auto match_pos = descriptors.at(match.parent_id)->root_position;
      for (const auto m : results.match_root) {
        const auto diff_m = (match_pos - descriptors.at(m)->root_position).norm();
        if (diff_m < match_config.min_match_separation_m) {
          spatialy_distinct = false;
          break;
        }
      }

      if (!spatialy_distinct) {
        continue;
      }

      results.match_nodes.push_back(descriptors.at(match.parent_id)->nodes);
      results.match_root.push_back(match.parent_id);
      results.score.push_back(match.score);
    }

    if (results.match_root.size() == match_config.max_registration_matches) {
      break;
    }
  }

  if (results.score.empty()) {
    results.score.push_back(best_score);
  }

  return results;
}

LayerSearchResults searchLeafDescriptors(const Descriptor& descriptor,
                                         const DescriptorMatchConfig& match_config,
                                         const std::set<NodeId>& valid_matches,
                                         const DescriptorCacheMap& leaf_cache_map,
                                         NodeId query_id) {
  MatchStatistics stats;
  LayerSearchResults results;
  results.query_nodes = descriptor.nodes;
  results.query_root = descriptor.root_node;

  VLOG(VLEVEL_DEBUG) << "Leaf matching: " << valid_matches.size() << " valid matches";

  float best_score = 0.0f;
  std::vector<ScoredMatch> new_scores;
  for (const auto valid_id : valid_matches) {
    const auto& leaf_cache = leaf_cache_map.at(valid_id);
    if (leaf_cache.empty()) {
      ++stats.num_empty;
    }

    for (auto&& [leaf_node, other] : leaf_cache) {
      bool same_robot = true;
      if (NodeSymbol(leaf_node).category() != NodeSymbol(query_id).category()) {
        same_robot = false;
      }

      if (leaf_node == query_id) {
        ++stats.num_same_parent;
        continue;  // disallow self matches even if they probably can't happen
      }

      std::chrono::duration<double> diff_s = descriptor.timestamp - other->timestamp;
      if (same_robot && diff_s.count() < match_config.min_time_separation_s) {
        ++stats.num_inside_horizon;
        continue;
      }

      const auto score = computeDescriptorScore(descriptor, *other, match_config.type);
      if (score > best_score) {
        best_score = score;
      }

      if (score > match_config.min_score) {
        results.valid_matches.insert(leaf_node);
        new_scores.push_back({leaf_node, other->root_node, score});
      }
    }
  }

  VLOG(VLEVEL_DEBUG) << "Leaf matching: " << stats.toString();

  std::sort(new_scores.begin(), new_scores.end(), [](const auto& a, const auto& b) {
    return a.score > b.score;
  });

  size_t match_idx = 0;
  for (const auto& match : new_scores) {
    bool passes_ratio =
        match_idx == 0 || match.score >= best_score * match_config.min_score_ratio;
    ++match_idx;

    if (!passes_ratio) {
      continue;
    }

    results.match_nodes.push_back({match.leaf_id});
    results.match_root.push_back(match.parent_id);
    results.score.push_back(match.score);

    if (results.match_root.size() == match_config.max_registration_matches) {
      break;
    }
  }

  if (results.score.empty()) {
    results.score.push_back(best_score);
  }

  return results;
}

}  // namespace hydra::lcd
