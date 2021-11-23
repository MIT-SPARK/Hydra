#pragma once
#include "kimera_dsg_builder/dsg_lcd_descriptors.h"

namespace kimera {
namespace lcd {

enum class DescriptorScoreType { COSINE, L1 };

struct DescriptorMatchConfig {
  LayerId layer;
  float min_score;
  float min_registration_score;
  DescriptorScoreType type = DescriptorScoreType::COSINE;
};

struct LayerSearchResults {
  NodeId best_node;
  float best_score;
  std::set<NodeId> valid_matches;
  std::set<NodeId> query_nodes;
  std::set<NodeId> match_nodes;
  NodeId query_root;
  NodeId match_root;
};

using DescriptorCache = std::map<NodeId, Descriptor::Ptr>;
using DescriptorCacheMap = std::map<NodeId, DescriptorCache>;

float computeDistance(const Descriptor& lhs,
                      const Descriptor& rhs,
                      const std::function<float(float, float)>& distance_func);

inline float computeCosineDistance(const Descriptor& lhs, const Descriptor& rhs) {
  float scale = 1.0;
  if (!lhs.normalized) {
    scale *= lhs.values.norm();
  }
  if (!rhs.normalized) {
    scale *= rhs.values.norm();
  }
  // TODO(nathan) we might want a looser check than this
  if (scale == 0.0) {
    scale = 1.0;  // force all zero descriptors to have 0 norm (instead of nan)
  }

  return computeDistance(
      lhs, rhs, [&scale](float lhs, float rhs) { return (lhs * rhs) / scale; });
}

inline float computeL1Distance(const Descriptor& lhs, const Descriptor& rhs) {
  const float lhs_scale = lhs.normalized ? 1.0f : lhs.values.lpNorm<1>();
  const float rhs_scale = rhs.normalized ? 1.0f : rhs.values.lpNorm<1>();

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
