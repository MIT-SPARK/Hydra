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
#include "hydra/openset/clustering/vmf_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>

using namespace spark_dsg;

namespace hydra {
namespace {

struct ScoreEntry {
  Eigen::VectorXf scores;
  float kappa;
};

using ScoreWorkspace = std::unordered_map<NodeId, ScoreEntry>;

inline float median(std::vector<float>& values) {
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::sort(values.begin(), values.end());
  const auto mid = values.size() / 2;
  if (values.size() % 2 == 0) {
    // this is safe because values.size() >= 1 and 2 is the first value
    // where this will trigger
    return (values[mid] + values[mid + 1]) / 2.0;
  } else {
    return values[mid];
  }
}

ScoreWorkspace propagateScores(const SceneGraphLayer& layer,
                               const ScoreWorkspace& workspace,
                               float lambda) {
  ScoreWorkspace updated;
  for (const auto& [node_id, node] : layer.nodes()) {
    auto iter = workspace.find(node_id);
    if (iter == workspace.end()) {
      continue;
    }

    size_t num_valid = 0;
    Eigen::VectorXf scores;
    for (const auto& sibling : node->siblings()) {
      const auto sibling_iter = workspace.find(sibling);
      if (sibling_iter == workspace.end()) {
        continue;
      }

      if (num_valid) {
        scores += sibling_iter->second.scores;
      } else {
        scores = sibling_iter->second.scores;
      }
      ++num_valid;
    }

    if (!num_valid) {
      continue;
    }

    scores /= num_valid;
    const auto alpha = iter->second.kappa / (iter->second.kappa + num_valid * lambda);
    const auto new_scores = (1.0f - alpha) * iter->second.scores + alpha * scores;
    updated.emplace(iter->first, ScoreEntry{new_scores, iter->second.kappa});
  }

  return updated;
}

}  // namespace

void declare_config(VmfClustering::Config& config) {
  using namespace config;
  name("VmfClustering::Config");
  base<VerbosityConfig>(config);
  base<LayerClustering::Config>(config);
  field(config.label_propagation_iterations, "label_propagation_iterations");
  field(config.convergence_threshold, "convergence_threshold");
  check(config.convergence_threshold, GE, 0.0, "convergence_threshold");
}

VmfClustering::VmfClustering(const Config& config)
    : LayerClustering(config), config(config::checkValid(config)) {}

auto VmfClustering::cluster(const spark_dsg::SceneGraphLayer& layer) const -> Clusters {
  if (!queries_ || queries_->empty()) {
    LOG_FIRST_N(ERROR, 1) << "No queries present: cannot cluster";
    return {};
  }

  // 1. Build up query scores as k_i * <x_i, q_j>
  std::vector<float> kappas;
  kappas.reserve(layer.numNodes());
  ScoreWorkspace scores;
  for (const auto& [node_id, node] : layer.nodes()) {
    auto attrs = node->tryAttributes<SemanticNodeAttributes>();
    if (!attrs) {
      continue;
    }

    const auto has_vmf =
        attrs->semantic_feature.size() > 0 && attrs->feature_concentration.size() == 1;
    if (!has_vmf && config.allow_empty_scores) {
      scores[node_id] = {Eigen::VectorXf::Zero(queries_->size()), 0.0f};
      continue;
    }

    const auto kappa = attrs->feature_concentration(0, 0);
    scores[node_id] = {kappa * queries_->getScores(*metric_, attrs->semantic_feature),
                       kappa};
    kappas.push_back(kappa);
  }

  // 2. Propagate scores as alpha_i * mean(k_n * <x_n, q_j>)
  const auto lambda = median(kappas);
  for (size_t i = 0; i < config.label_propagation_iterations; ++i) {
    scores = propagateScores(layer, scores, lambda);
  }

  // 3. Pick the best scoring label

  // 4. Extract connected components as regions
  return {};
}

}  // namespace hydra
