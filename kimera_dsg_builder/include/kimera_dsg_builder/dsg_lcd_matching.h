#pragma once
#include "kimera_dsg_builder/dsg_lcd_descriptors.h"

namespace kimera {
namespace lcd {

enum class DescriptorScoreType { COSINE, L1 };

struct DescriptorMatchConfig {
  LayerId layer;
  float min_score;
  float min_registration_score;
  double min_time_separation_s;
  size_t max_registration_matches;
  double min_score_ratio;
  double min_match_separation_m;
  DescriptorScoreType type = DescriptorScoreType::COSINE;
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

}  // namespace lcd
}  // namespace kimera
