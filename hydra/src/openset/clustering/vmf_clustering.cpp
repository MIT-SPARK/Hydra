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
#include "hydra/openset/clustering/vmf_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/graph_utilities.h>
#include <spark_dsg/node_attributes.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

#include "vmf_clustering_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

const auto registration =
    config::RegistrationWithConfig<LayerClustering,
                                   VmfClustering,
                                   VmfClustering::Config>("VmfClustering");

float median(std::vector<float> values) {
  std::sort(values.begin(), values.end());
  const auto mid = values.size() / 2;
  return values.size() % 2 ? values[mid] : (values[mid - 1] + values[mid]) / 2.0f;
}

bool validFeature(const SemanticNodeAttributes& attrs, Eigen::Index dimension) {
  const auto& feature = attrs.semantic_feature;
  return feature.size() == dimension && feature.allFinite() && feature.norm() > 0.0f &&
         attrs.feature_concentration.size() == 1 &&
         std::isfinite(attrs.feature_concentration(0, 0)) &&
         attrs.feature_concentration(0, 0) >= 0.0f;
}

}  // namespace

namespace vmf {

ScoreWorkspace computeScores(const SceneGraphLayer& layer,
                             const EmbeddingGroup& queries,
                             const EmbeddingDistance& metric,
                             bool allow_empty_scores) {
  if (queries.empty()) {
    return {};
  }

  const auto dimension = queries.embeddings.front().size();
  for (const auto& query : queries.embeddings) {
    if (dimension == 0 || query.size() != dimension || !query.allFinite() ||
        query.norm() == 0.0f) {
      return {};
    }
  }

  ScoreWorkspace scores;
  std::queue<NodeId> pending;
  std::unordered_set<NodeId> supported;
  for (const auto& node : layer.nodes()) {
    const auto attrs = node.tryAttributes<SemanticNodeAttributes>();
    if (!attrs) {
      continue;
    }

    if (!validFeature(*attrs, dimension)) {
      if (allow_empty_scores) {
        scores.emplace(node.id,
                       ScoreEntry{Eigen::VectorXf::Zero(queries.size()), 0.0f});
      }

      continue;
    }

    const auto kappa = attrs->feature_concentration(0, 0);
    const Eigen::VectorXf values =
        kappa * queries.getScores(metric, attrs->semantic_feature);
    if (!values.allFinite()) {
      continue;
    }

    scores.emplace(node.id, ScoreEntry{values, kappa});
    if (kappa > 0.0f) {
      supported.insert(node.id);
      pending.push(node.id);
    }
  }

  // Components without observation confidence cannot receive a meaningful label.
  while (!pending.empty()) {
    const auto id = pending.front();
    pending.pop();
    for (const auto sibling : layer.getNode(id).siblings()) {
      if (scores.count(sibling) && supported.insert(sibling).second) {
        pending.push(sibling);
      }
    }
  }

  for (auto iter = scores.begin(); iter != scores.end();) {
    if (!supported.count(iter->first)) {
      iter = scores.erase(iter);
    } else {
      ++iter;
    }
  }

  return scores;
}

ScoreWorkspace propagateScores(const SceneGraphLayer& layer,
                               const ScoreWorkspace& initial,
                               const VmfClustering::Config& config) {
  std::vector<float> kappas;
  std::vector<float> degrees;
  std::unordered_map<NodeId, size_t> degree;
  for (const auto& [id, entry] : initial) {
    if (entry.kappa > 0.0f) {
      kappas.push_back(entry.kappa);
    }

    auto& count = degree[id];
    for (const auto sibling : layer.getNode(id).siblings()) {
      if (initial.count(sibling)) {
        ++count;
      }
    }

    if (count) {
      degrees.push_back(count);
    }
  }

  if (kappas.empty() || degrees.empty()) {
    return initial;
  }

  const auto lambda = config.propagation_strength * median(kappas) / median(degrees);
  auto current = initial;
  const auto num_queries = initial.begin()->second.scores.size();
  std::vector<bool> converged(num_queries, false);
  for (size_t iteration = 0; iteration < config.label_propagation_iterations;
       ++iteration) {
    auto updated = current;
    Eigen::VectorXf delta = Eigen::VectorXf::Zero(num_queries);
    for (const auto& [id, entry] : initial) {
      const auto count = degree.at(id);
      if (!count) {
        continue;
      }

      Eigen::VectorXf mean = Eigen::VectorXf::Zero(entry.scores.size());
      for (const auto sibling : layer.getNode(id).siblings()) {
        const auto iter = current.find(sibling);
        if (iter != current.end()) {
          mean += iter->second.scores;
        }
      }

      mean /= count;
      const auto alpha = entry.kappa / (entry.kappa + lambda * count);
      auto& scores = updated.at(id).scores;
      scores = alpha * entry.scores + (1.0f - alpha) * mean;
      for (Eigen::Index q = 0; q < num_queries; ++q) {
        if (converged[q]) {
          scores(q) = current.at(id).scores(q);
          continue;
        }

        delta(q) = std::max(delta(q), std::abs(scores(q) - current.at(id).scores(q)));
      }
    }

    current = std::move(updated);
    // FOUND-IT processes one query at a time, so each column stops independently.
    for (Eigen::Index q = 0; q < num_queries; ++q) {
      converged[q] = converged[q] || delta(q) < config.convergence_threshold;
    }

    if (std::all_of(
            converged.begin(), converged.end(), [](bool value) { return value; })) {
      break;
    }
  }

  return current;
}

}  // namespace vmf

void declare_config(VmfClustering::Config& config) {
  using namespace config;
  name("VmfClustering::Config");
  base<VerbosityConfig>(config);
  base<LayerClustering::Config>(config);
  field(config.allow_empty_scores, "allow_empty_scores");
  field(config.propagation_strength, "propagation_strength");
  field(config.label_propagation_iterations, "label_propagation_iterations");
  field(config.convergence_threshold, "convergence_threshold");
  check(config.propagation_strength, GT, 0.0f, "propagation_strength");
  check(config.convergence_threshold, GE, 0.0f, "convergence_threshold");
}

VmfClustering::VmfClustering(const Config& config)
    : LayerClustering(config), config(config::checkValid(config)) {}

auto VmfClustering::cluster(const spark_dsg::SceneGraphLayer& layer) const -> Clusters {
  if (!queries_ || queries_->empty() || !metric_) {
    LOG_FIRST_N(ERROR, 1) << "Queries and a metric are required for vMF clustering";
    return {};
  }

  const auto initial =
      vmf::computeScores(layer, *queries_, *metric_, config.allow_empty_scores);
  const auto scores = vmf::propagateScores(layer, initial, config);

  std::unordered_map<NodeId, Eigen::Index> labels;
  for (const auto& [id, entry] : scores) {
    Eigen::Index label;
    entry.scores.maxCoeff(&label);
    labels.emplace(id, label);
  }

  const auto components = graph_utilities::getConnectedComponents(
      layer,
      [&](const auto& node) { return labels.count(node.id); },
      [&](const auto& edge) {
        const auto source = labels.find(edge.source);
        const auto target = labels.find(edge.target);
        return source != labels.end() && target != labels.end() &&
               source->second == target->second;
      });

  Clusters clusters;
  for (const auto& nodes : components) {
    auto cluster = std::make_shared<Cluster>();
    cluster->nodes.insert(nodes.begin(), nodes.end());
    cluster->best_query = labels.at(nodes.front());
    cluster->feature = Eigen::VectorXf::Zero(queries_->embeddings.front().size());
    size_t count = 0;
    for (const auto id : cluster->nodes) {
      const auto& attrs = layer.getNode(id).attributes<SemanticNodeAttributes>();
      if (validFeature(attrs, cluster->feature.size())) {
        cluster->feature += attrs.semantic_feature;
        ++count;
      }
    }

    if (count) {
      cluster->feature /= count;
    }

    cluster->score =
        metric_->score(queries_->embeddings.at(cluster->best_query), cluster->feature);
    clusters.push_back(cluster);
  }

  MLOG(1) << "finished clustering with " << clusters.size() << " cluster(s)";
  return clusters;
}

}  // namespace hydra
