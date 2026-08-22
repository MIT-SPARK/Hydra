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

void declare_config(DescriptorMatchConfig& config);

struct LayerSearchResults {
  std::vector<float> score;
  std::set<spark_dsg::NodeId> valid_matches;
  std::set<spark_dsg::NodeId> query_nodes;
  std::vector<std::set<spark_dsg::NodeId>> match_nodes;
  spark_dsg::NodeId query_root;
  std::vector<spark_dsg::NodeId> match_root;
};

using DescriptorCache = std::map<spark_dsg::NodeId, Descriptor::Ptr>;
using DescriptorCacheMap = std::map<spark_dsg::NodeId, DescriptorCache>;

float computeDistance(const Descriptor& lhs,
                      const Descriptor& rhs,
                      const std::function<float(float, float)>& distance_func);

float computeCosineDistance(const Descriptor& lhs, const Descriptor& rhs);

float computeL1Distance(const Descriptor& lhs, const Descriptor& rhs);

float computeDescriptorScore(const Descriptor& lhs,
                             const Descriptor& rhs,
                             DescriptorScoreType type);

LayerSearchResults searchDescriptors(
    const Descriptor& descriptor,
    const DescriptorMatchConfig& match_config,
    const std::set<spark_dsg::NodeId>& valid_matches,
    const DescriptorCache& descriptors,
    const std::map<spark_dsg::NodeId, std::set<spark_dsg::NodeId>>& root_leaf_map,
    spark_dsg::NodeId query_id);

LayerSearchResults searchLeafDescriptors(
    const Descriptor& descriptor,
    const DescriptorMatchConfig& match_config,
    const std::set<spark_dsg::NodeId>& valid_matches,
    const DescriptorCacheMap& leaf_cache_map,
    spark_dsg::NodeId query_id);

}  // namespace hydra::lcd
