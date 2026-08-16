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
using EmbeddingMap = std::map<NodeId, Eigen::VectorXf>;
using Indices = std::vector<std::pair<size_t, size_t>>;
using ClusterIds = std::vector<std::vector<NodeId>>;

namespace {

bool keysIntersect(EdgeKey key1, EdgeKey key2) {
  return key1.k1 == key2.k1 || key1.k1 == key2.k2 || key1.k2 == key2.k1 ||
         key1.k2 == key2.k2;
}

EmbeddingMap getEmbeddingMap(const std::map<NodeId, SceneGraphNode::Ptr>& nodes) {
  EmbeddingMap features;
  for (const auto& id_node : nodes) {
    const auto& attrs = id_node.second->attributes<SemanticNodeAttributes>();
    // TODO(nathan) consider other pooling operations
    features[id_node.first] = attrs.semantic_feature.rowwise().mean();
  }

  return features;
}

EmbeddingMap getEmbeddingMap(const SceneGraphLayer& layer,
                             const std::vector<NodeId>& nodes) {
  EmbeddingMap features;
  for (const auto node : nodes) {
    const auto& attrs = layer.getNode(node).attributes<SemanticNodeAttributes>();
    // TODO(nathan) consider other pooling operations
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

}  // namespace

AgglomerativeClustering::Workspace::Workspace(const SceneGraphLayer& layer)
    : Workspace(layer, getEmbeddingMap(layer.nodes())) {}

AgglomerativeClustering::Workspace::Workspace(const SceneGraphLayer& layer,
                                              const std::vector<NodeId>& nodes)
    : Workspace(layer, getEmbeddingMap(layer, nodes)) {}

AgglomerativeClustering::Workspace::Workspace(const SceneGraphLayer& layer,
                                              const EmbeddingMap& node_embeddings) {
  size_t index = 0;
  for (auto&& [node_id, feature] : node_embeddings) {
    features[index] = feature;
    node_lookup[index] = node_id;
    order[node_id] = index;
    ++index;
  }

  for (auto&& [node_id, index] : order) {
    const auto& node = layer.getNode(node_id);
    for (const auto& sibling : node.siblings()) {
      auto iter = order.find(sibling);
      if (iter == order.end()) {
        continue;
      }

      edges.emplace(EdgeKey(index, iter->second), 0.0);
    }
  }

  assignments.resize(order.size());
  std::iota(assignments.begin(), assignments.end(), 0);
}

size_t AgglomerativeClustering::Workspace::size() const { return order.size(); }

size_t AgglomerativeClustering::Workspace::featureDim() const {
  if (features.empty()) {
    return 0;
  }

  return features.begin()->second.rows();
}

void AgglomerativeClustering::Workspace::setup(const EmbeddingGroup& tasks,
                                               const EmbeddingDistance& metric,
                                               bool reweight,
                                               double I_xy_,
                                               double delta_weight_) {
  const auto N = order.size();
  const auto M = tasks.embeddings.size() + 1;

  // p(x) and p(y) are uniform
  px = Eigen::VectorXd::Constant(N, 1.0 / static_cast<double>(N));
  py = Eigen::VectorXd::Constant(M, 1.0 / static_cast<double>(M));
  pz_x = Eigen::MatrixXd::Identity(N, N);  // p(z|x) is identity
  pz = px;                                 // p(z) = p(x) initially

  py_x = computeIBpyGivenX(ws, tasks, metric, config.py_x);
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

  if (reweight) {
    I_xy = I_xy_;
    delta_weight = delta_weight_;
  }
}

double AgglomerativeClustering::Workspace::score(const EdgeKey& edge) const {
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

std::string AgglomerativeClustering::Workspace::summary(double max_delta) const {
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

ClusterIds AgglomerativeClustering::Workspace::getClusters() const {
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

void declare_config(AgglomerativeClustering::Config& config) {
  using namespace config;
  name("AgglomerativeClustering::Config");
  field(config.tasks, "tasks");
  config.metric.setOptional();
  field(config.metric, "metric");
  field(config.filter_clusters, "filter_clusters");
  field(config.max_delta, "max_delta");
  field(config.tolerance, "tolerance");
  field(config.score_threshold, "score_threshold");
  field(config.top_k, "top_k");
  field(config.cumulative, "cumulative");
  field(config.null_task_preprune, "null_task_preprune");

  check(config.max_delta, GE, 0.0, "max_delta");
  check(config.tolerance, LE, 0.0, "tolerance");
  check(config.top_k, GT, 0, "top_k");
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

  Workspace ws(layer, features);
  cluster(ws, *tasks_, *metric_);
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
    if (config.filter_clusters && info.score < config.score_threshold) {
      continue;
    }

    cluster->score = info.score;
    if (info.score >= config.score_threshold) {
      cluster->best_task_index = info.index;
      cluster->best_task_name = tasks_->names.at(info.index);
    } else {
      cluster->best_task_name = "";
    }

    to_return.push_back(cluster);
  }

  VLOG(1) << "[IB] finished clustering with " << to_return.size() << " cluster(s)";
  return to_return;
}

void AgglomerativeClustering::cluster(Workspace& ws,
                                      const EmbeddingGroup& tasks,
                                      const EmbeddingDistance& metric,
                                      bool reweight,
                                      double I_xy,
                                      double delta_weight,
                                      int verbosity) {
  VLOG(verbosity) << "[IB] starting clustering with " << ws.edges.size() << " edges";

  ws.setup(tasks, metric, reweight, I_xy, delta_weight);

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
    if (!ws.merge(best_edge, config.max_delta, changed_edges)) {
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

  VLOG(verbosity) << "[IB] " << summarize();
}

}  // namespace hydra
