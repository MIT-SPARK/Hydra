#include "hydra/openset/clustering/agglomerative_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include <numeric>

#include "hydra/utils/printing.h"
#include "hydra/utils/probability_utilities.h"

namespace hydra {

using namespace spark_dsg;

using Clusters = AgglomerativeClustering::Clusters;
using ClusterIds = std::vector<std::vector<NodeId>>;
using ClusterWorkspace = AgglomerativeClustering::Workspace;
using EmbeddingMap = std::map<NodeId, Eigen::VectorXf>;
using Indices = std::vector<std::pair<size_t, size_t>>;

namespace {

bool keysIntersect(EdgeKey key1, EdgeKey key2) {
  return key1.k1 == key2.k1 || key1.k1 == key2.k2 || key1.k2 == key2.k1 ||
         key1.k2 == key2.k2;
}

EmbeddingMap getEmbeddingMap(const std::map<NodeId, SceneGraphNode::Ptr>& nodes) {
  EmbeddingMap features;
  for (const auto& [node_id, node] : nodes) {
    const auto& attrs = node->attributes<SemanticNodeAttributes>();
    features[node_id] = attrs.semantic_feature.rowwise().mean();
  }

  return features;
}

EmbeddingMap getEmbeddingMap(const SceneGraphLayer& layer,
                             const std::vector<NodeId>& nodes) {
  EmbeddingMap features;
  for (const auto node : nodes) {
    const auto& attrs = layer.getNode(node).attributes<SemanticNodeAttributes>();
    features[node] = attrs.semantic_feature.rowwise().mean();
  }

  return features;
}

void clearEdgeWeights(const EdgeKey& key,
                      std::map<EdgeKey, double>& edges,
                      std::list<EdgeKey>& updated) {
  // clear all affected edge weights
  std::set<EdgeKey> seen;
  auto iter = edges.begin();
  while (iter != edges.end()) {
    if (keysIntersect(iter->first, key)) {
      seen.insert(iter->first);
      iter = edges.erase(iter);
    } else {
      ++iter;
    }
  }

  std::set<EdgeKey> new_keys;
  updated.clear();
  for (const auto& prev : seen) {
    const auto new_k1 = prev.k1 == key.k2 ? key.k1 : prev.k1;
    const auto new_k2 = prev.k2 == key.k2 ? key.k1 : prev.k2;
    if (new_k1 == new_k2) {
      continue;
    }

    const EdgeKey new_key(new_k1, new_k2);
    if (new_keys.count(new_key)) {
      continue;  // we got a duplicate key from a merge
    }

    edges.emplace(new_key, 0.0);
    updated.push_back(new_key);
  }
}

Indices findTopKIndicesCols(const Eigen::MatrixXd& m, size_t top_k) {
  // Find top k indices for each column
  size_t k = std::min(top_k, static_cast<size_t>(m.rows()));
  Indices top_indices;
  for (Eigen::Index c = 0; c < m.cols(); c++) {
    const auto& col = m.col(c);

    std::vector<size_t> idx(m.rows(), 0);
    std::iota(idx.begin(), idx.end(), 0);
    // Sort the indices according to their respective value
    std::sort(idx.begin(), idx.end(), [&col](auto& lhv, auto& rhv) {
      return col(lhv) > col(rhv);
    });

    for (size_t i = 0; i < k; i++) {
      top_indices.push_back({idx[i], c});
    }
  }

  return top_indices;
}

}  // namespace

ClusterWorkspace::Workspace(const SceneGraphLayer& layer)
    : Workspace(layer.edges(), getEmbeddingMap(layer.nodes())) {}

ClusterWorkspace::Workspace(const SceneGraphLayer& layer,
                            const std::vector<NodeId>& nodes)
    : Workspace(layer.edges(), getEmbeddingMap(layer, nodes)) {}

ClusterWorkspace::Workspace(const EdgeContainer::Edges& edges_,
                            const EmbeddingMap& node_embeddings) {
  size_t index = 0;
  for (const auto& [node_id, feature] : node_embeddings) {
    features[index] = feature;
    node_lookup[index] = node_id;
    order[node_id] = index;
    ++index;
  }

  for (const auto& [key, _] : edges_) {
    const auto source = order.find(key.k1);
    const auto target = order.find(key.k2);
    if (source == order.end() || target == order.end()) {
      continue;
    }

    edges.emplace(EdgeKey(source->second, target->second), 0.0);
  }

  assignments.resize(order.size());
  std::iota(assignments.begin(), assignments.end(), 0);
}

size_t ClusterWorkspace::size() const { return order.size(); }

size_t ClusterWorkspace::featureDim() const {
  if (features.empty()) {
    return 0;
  }

  return features.begin()->second.rows();
}

void ClusterWorkspace::reweight(double I_xy_, double delta_weight_) {
  I_xy = I_xy_;
  delta_weight = delta_weight_;
}

void ClusterWorkspace::setup(const IBProbabilityConfig& config,
                             const EmbeddingGroup& tasks,
                             const EmbeddingDistance& metric) {
  const auto N = order.size();
  const auto M = tasks.embeddings.size() + 1;

  // p(x) and p(y) are uniform
  px = Eigen::VectorXd::Constant(N, 1.0 / static_cast<double>(N));
  py = Eigen::VectorXd::Constant(M, 1.0 / static_cast<double>(M));
  pz_x = Eigen::MatrixXd::Identity(N, N);  // p(z|x) is identity
  pz = px;                                 // p(z) = p(x) initially

  py_x = compute_py_x(config, features, tasks, metric);
  py_z = py_x;  // p(y|z) = p(y|x) (as p(z) = p(x) and p(z|x) = I_n

  const auto fmt = getDefaultFormat();
  VLOG(10) << "p(x): " << px.format(fmt);
  VLOG(10) << "p(z): " << pz.format(fmt);
  VLOG(10) << "p(y): " << py.format(fmt);
  VLOG(10) << "p(y|x): " << py_x.format(fmt);
  VLOG(10) << "p(y|z): " << py_z.format(fmt);
  VLOG(10) << "p(z|x): " << pz_x.format(fmt);

  // initialize mutual information to starting values;
  I_xy = mutualInformation(py, px, py_x);
  I_zy_prev = I_xy;
  deltas.clear();
}

double ClusterWorkspace::score(const EdgeKey& edge) const {
  const auto p_s = pz(edge.k1);
  const auto p_t = pz(edge.k2);
  const auto total = p_s + p_t;

  Eigen::VectorXd prior(2);
  prior << p_s / total, p_t / total;

  Eigen::MatrixXd py_z_local(py_z.rows(), 2);
  py_z_local.col(0) = py_z.col(edge.k1);
  py_z_local.col(1) = py_z.col(edge.k2);

  const auto divergence = jensenShannonDivergence(py_z_local, prior);

  const auto fmt = getDefaultFormat();
  VLOG(20) << "Scoring edge (" << edge << "): prior: " << prior.format(fmt)
           << ", p(y|z=z): " << py_z_local.format(fmt)
           << ", divergence: " << divergence;

  return total * divergence;
}

bool AgglomerativeClustering::Workspace::merge(EdgeKey key,
                                               double max_delta,
                                               std::list<EdgeKey>& updated) {
  // we merge target -> source
  const auto p_s = pz(key.k1);
  const auto p_t = pz(key.k2);
  // update new cluster probabilities
  pz(key.k1) = p_s + p_t;
  py_z.col(key.k1) =
      ((p_s * py_z.col(key.k1) + p_t * py_z.col(key.k2)) / (p_s + p_t)).eval();
  pz_x.col(key.k1) += pz_x.col(key.k2);

  // zero-out merged nodes
  pz(key.k2) = 0.0;
  py_z.col(key.k2).setConstant(0.0);
  pz_x.col(key.k2).setConstant(0.0);

  // for I[a; b] order is p(a), p(b), p(a|b)
  const auto I_zy = mutualInformation(py, pz, py_z);
  const auto d_I_zy = I_zy_prev - I_zy;

  // avoid divide-by-zero and other weirdness with precision
  const auto delta = delta_weight * d_I_zy / I_xy;
  VLOG(10) << "delta for (" << key << "): " << delta;

  I_zy_prev = I_zy;
  deltas.push_back(delta);
  if (delta < max_delta) {
    return false;
  }

  for (auto& parent : assignments) {
    if (parent == key.k2) {
      parent = key.k1;
    }
  }

  clearEdgeWeights(key, edges, updated);
  return true;
}

std::string ClusterWorkspace::summary(double max_delta) const {
  if (deltas.empty()) {
    return "0 merge(s), δ_0=N/A, δ_n=N/A";
  }

  const size_t merges = deltas.back() <= max_delta ? deltas.size() : deltas.size() - 1;
  const std::string d0 = std::to_string(deltas.front());
  const std::string dn = std::to_string(deltas.back());

  std::stringstream ss;
  ss << merges << " merge(s), " << "δ_0=" << d0 << ", δ_n=" << dn;
  return ss.str();
}

ClusterIds ClusterWorkspace::getClusters() const {
  const std::set<size_t> cluster_ids(assignments.begin(), assignments.end());

  size_t index = 0;
  std::map<size_t, size_t> cluster_lookup;
  for (const auto cluster_id : cluster_ids) {
    cluster_lookup[cluster_id] = index;
    ++index;
  }

  ClusterIds to_return(cluster_ids.size());
  for (size_t i = 0; i < assignments.size(); ++i) {
    auto& cluster = to_return.at(cluster_lookup[assignments[i]]);
    cluster.push_back(node_lookup.at(i));
  }

  return to_return;
}

Eigen::MatrixXd ClusterWorkspace::compute_py_x(const IBProbabilityConfig& config,
                                               const FeatureMap& features,
                                               const EmbeddingGroup& tasks,
                                               const EmbeddingDistance& metric) {
  const auto fmt = getDefaultFormat();

  size_t N = features.size();
  size_t M = tasks.embeddings.size() + 1;

  Eigen::MatrixXd py_x = Eigen::MatrixXd::Ones(M, N) * 1e-12;
  Eigen::MatrixXd py_x_temp = Eigen::MatrixXd::Zero(M, N);
  py_x_temp.row(0).setConstant(config.score_threshold);

  VLOG(15) << "----------------------------------------";
  VLOG(15) << "Computing workspace feature scores";
  VLOG(15) << "----------------------------------------";

  for (auto&& [idx, feature] : features) {
    const auto scores = tasks.getScores(metric, feature);
    VLOG(15) << "scores @ " << idx << ": " << scores.format(fmt);
    py_x_temp.block(1, idx, M - 1, 1) = scores.cast<double>();
  }

  VLOG(15) << "----------------------------------------";

  size_t k = std::min(M, config.top_k);
  size_t l = k;
  if (config.cumulative) {
    l = 1;
  }

  while (l <= k) {
    const auto top_k_inds = findTopKIndicesCols(py_x_temp, l);
    for (const auto& idx : top_k_inds) {
      py_x(idx.first, idx.second) =
          py_x(idx.first, idx.second) + py_x_temp(idx.first, idx.second);
    }

    l++;
  }

  if (config.null_task_preprune) {
    // Null task processing
    const auto top_inds = findTopKIndicesCols(py_x_temp, 1);
    for (const auto& idx : top_inds) {
      // Null task corresponds to first row
      if (idx.first == 0) {
        py_x.block(1, idx.second, M - 1, 1).setConstant(1e-12);
        // Essentially 0 (but not 0 to avoid NaN error)
      }
    }
  }

  VLOG(10) << "raw: p(y|x): " << py_x.format(fmt);

  const auto scored = py_x.bottomRows(M - 1);
  const auto min = scored.rowwise().minCoeff();
  const auto max = scored.rowwise().maxCoeff();
  const auto avg = scored.rowwise().mean();

  VLOG(10) << "score average: " << avg.format(fmt) << ", range: " << min.format(fmt)
           << " -> " << max.format(fmt);

  const auto norm_factor = py_x.colwise().sum();
  py_x.array().rowwise() /= norm_factor.array();

  VLOG(10) << "p(y|x): " << py_x.format(fmt);

  return py_x;
}

void declare_config(IBProbabilityConfig& config) {
  using namespace config;
  name("IBProbabilityConfig");
  field(config.score_threshold, "score_threshold");
  field(config.top_k, "top_k");
  field(config.cumulative, "cumulative");
  field(config.null_task_preprune, "null_task_preprune");
  check(config.top_k, GT, 0, "top_k");
}

void declare_config(AgglomerativeClustering::Config& config) {
  using namespace config;
  name("AgglomerativeClustering::Config");
  base<VerbosityConfig>(config);
  field(config.probabilities, "probabilities");
  field(config.tasks, "tasks");
  config.metric.setOptional();
  field(config.metric, "metric");
  field(config.filter_clusters, "filter_clusters");
  field(config.max_delta, "max_delta");
  field(config.tolerance, "tolerance");
  check(config.max_delta, GE, 0.0, "max_delta");
  check(config.tolerance, LE, 0.0, "tolerance");
}

AgglomerativeClustering::AgglomerativeClustering(const Config& config)
    : config(config::checkValid(config)),
      tasks_(config.tasks.create()),
      metric_(config.metric.create()) {}

Clusters AgglomerativeClustering::cluster(const SceneGraphLayer& layer,
                                          const NodeEmbeddingMap& features) const {
  if (tasks_->empty()) {
    LOG_FIRST_N(ERROR, 5) << "No tasks present: cannot cluster";
    return {};
  }

  Workspace ws(layer.edges(), features);

  MLOG(1) << "starting clustering with " << ws.edges.size() << " edges";
  ws.setup(config.probabilities, *tasks_, *metric_);
  cluster(ws, config.max_delta);
  MLOG(1) << ws.summary(config.max_delta);

  const auto cluster_nodes = ws.getClusters();

  Clusters to_return;
  for (const auto& nodes : cluster_nodes) {
    auto cluster = std::make_shared<Cluster>();
    cluster->nodes.insert(nodes.begin(), nodes.end());

    auto iter = cluster->nodes.begin();
    cluster->feature = features.at(*iter);
    ++iter;
    while (iter != cluster->nodes.end()) {
      cluster->feature += features.at(*iter);
      ++iter;
    }

    cluster->feature /= cluster->nodes.size();

    const auto info = tasks_->getBestScore(*metric_, cluster->feature);
    if (config.filter_clusters && info.score < config.probabilities.score_threshold) {
      continue;
    }

    cluster->score = info.score;
    if (info.score >= config.probabilities.score_threshold) {
      cluster->best_task_index = info.index;
      cluster->best_task_name = tasks_->names.at(info.index);
    } else {
      cluster->best_task_name = "";
    }

    to_return.push_back(cluster);
  }

  MLOG(1) << "finished clustering with " << to_return.size() << " cluster(s)";
  return to_return;
}

void AgglomerativeClustering::cluster(Workspace& ws, double max_delta) {
  VLOG(10) << "-----------------------------------";
  VLOG(10) << "Scoring edges";
  VLOG(10) << "-----------------------------------";

  for (auto& [edge, weight] : ws.edges) {
    weight = ws.score(edge);
    VLOG(10) << "edge (" << edge << "): " << weight;
  }

  VLOG(10) << "-----------------------------------";

  for (size_t i = 0; i < ws.size(); ++i) {
    if (ws.edges.empty()) {
      // shouldn't happen unless |connected components| > 1
      break;
    }

    // iter will always be valid: always at least one edge
    auto best_edge_ptr = std::min_element(
        ws.edges.begin(), ws.edges.end(), [&](const auto& lhs, const auto& rhs) {
          return lhs.second < rhs.second;
        });
    CHECK(best_edge_ptr != ws.edges.end());

    if (VLOG_IS_ON(15)) {
      VLOG(15) << "***********************************";
      VLOG(15) << "Candidates";
      VLOG(15) << "***********************************";
      for (auto&& [edge, weight] : ws.edges) {
        VLOG(15) << "edge (" << edge << "): " << weight;
      }
      VLOG(15) << "***********************************";
    }

    const auto best_edge = best_edge_ptr->first;
    std::list<EdgeKey> changed_edges;
    if (!ws.merge(best_edge, max_delta, changed_edges)) {
      // we've hit a stop criteria
      break;
    }

    VLOG(10) << "-----------------------------------";
    VLOG(10) << "Scoring changed edges";
    VLOG(10) << "-----------------------------------";

    for (const auto& edge : changed_edges) {
      const auto score = ws.score(edge);
      ws.edges[edge] = score;
      VLOG(10) << "edge " << edge << ": " << score;
    }

    VLOG(10) << "-----------------------------------";
  }
}

}  // namespace hydra
