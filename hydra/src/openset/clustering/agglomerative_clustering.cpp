#include "hydra/openset/clustering/agglomerative_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include <numeric>

#include "hydra/utils/disjoint_set.h"
#include "hydra/utils/printing.h"
#include "hydra/utils/probability_utilities.h"

namespace hydra {

using namespace spark_dsg;
using Clusters = AgglomerativeClustering::Clusters;
using EmbeddingMap = std::map<NodeId, Eigen::VectorXf>;
using Indices = std::vector<std::pair<size_t, size_t>>;

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

ClusteringWorkspace::ClusteringWorkspace(const SceneGraphLayer& layer)
    : ClusteringWorkspace(layer, getEmbeddingMap(layer.nodes())) {}

ClusteringWorkspace::ClusteringWorkspace(const SceneGraphLayer& layer,
                                         const std::vector<NodeId>& nodes)
    : ClusteringWorkspace(layer, getEmbeddingMap(layer, nodes)) {}

ClusteringWorkspace::ClusteringWorkspace(const SceneGraphLayer& layer,
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

size_t ClusteringWorkspace::size() const { return order.size(); }

size_t ClusteringWorkspace::featureDim() const {
  if (features.empty()) {
    return 0;
  }

  return features.begin()->second.rows();
}

std::list<EdgeKey> ClusteringWorkspace::addMerge(EdgeKey key) {
  // TODO(nathan) there are more efficient ways to maintain this, but modfiying
  // disjoint set right now is not the best decision
  for (auto& parent : assignments) {
    if (parent == key.k2) {
      parent = key.k1;
    }
  }

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
  std::list<EdgeKey> to_update;
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
    to_update.push_back(new_key);
  }

  return to_update;
}

std::vector<std::vector<NodeId>> ClusteringWorkspace::getClusters() const {
  const std::set<size_t> cluster_ids(assignments.begin(), assignments.end());

  size_t index = 0;
  std::map<size_t, size_t> cluster_lookup;
  for (const auto cluster_id : cluster_ids) {
    cluster_lookup[cluster_id] = index;
    ++index;
  }

  std::vector<std::vector<NodeId>> to_return(cluster_ids.size());
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
  field(config.filter_regions, "filter_regions");
  field(config.max_delta, "max_delta");
  field(config.tolerance, "tolerance");
  field(config.py_x.score_threshold, "score_threshold");
  field(config.py_x.top_k, "top_k");
  field(config.py_x.cumulative, "cumulative");
  field(config.py_x.null_task_preprune, "null_task_preprune");

  check(config.max_delta, GE, 0.0, "max_delta");
  check(config.tolerance, LE, 0.0, "tolerance");
  check(config.py_x.top_k, GT, 0, "top_k");
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

  ClusteringWorkspace ws(layer, features);
  cluster(ws, *tasks_, *metric_);

  const auto to_return = getClusters(ws, features);
  VLOG(1) << "[IB] finished clustering with " << to_return.size() << " cluster(s)";
  return to_return;
}

Clusters AgglomerativeClustering::getClusters(const ClusteringWorkspace& ws,
                                              const NodeEmbeddingMap& features) const {
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
    if (config.filter_regions && info.score < config.selector.py_x.score_threshold) {
      continue;
    }

    cluster->score = info.score;
    if (info.score >= config.selector.py_x.score_threshold) {
      cluster->best_task_index = info.index;
      cluster->best_task_name = tasks_->names.at(info.index);
    } else {
      cluster->best_task_name = "";
    }

    to_return.push_back(cluster);
  }

  return to_return;
}

void AgglomerativeClustering::cluster(ClusteringWorkspace& ws,
                                      const EmbeddingGroup& tasks,
                                      const EmbeddingDistance& metric,
                                      bool reweight,
                                      double I_xy,
                                      double delta_weight,
                                      int verbosity) {
  VLOG(verbosity) << "[IB] starting clustering with " << ws.edges.size() << " edges";

  setup(ws, tasks, metric);

  if (reweight) {
    onlineReweighting(I_xy, delta_weight);
  }

  VLOG(10) << "-----------------------------------";
  VLOG(10) << "Scoring edges";
  VLOG(10) << "-----------------------------------";

  for (auto& [edge, weight] : ws.edges) {
    const auto score = scoreEdge(edge);
    VLOG(10) << "edge (" << edge << "): " << score;
    weight = score;
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
          return compareEdges(lhs, rhs);
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

    const EdgeKey best_edge = best_edge_ptr->first;
    if (!updateFromEdge(best_edge)) {
      // we've hit a stop criteria
      break;
    }

    VLOG(10) << "-----------------------------------";
    VLOG(10) << "Scoring changed edges";
    VLOG(10) << "-----------------------------------";

    const auto changed_edges = ws.addMerge(best_edge);
    for (const auto edge : changed_edges) {
      const auto score = scoreEdge(edge);
      VLOG(10) << "edge " << edge << ": " << score;
      ws.edges[edge] = score;
    }

    VLOG(10) << "-----------------------------------";
  }

  VLOG(verbosity) << "[IB] " << summarize();
}

void AgglomerativeClustering::setup(const ClusteringWorkspace& ws,
                                    const EmbeddingGroup& tasks,
                                    const EmbeddingDistance& metric) {
  const auto fmt = getDefaultFormat();
  size_t N = ws.size();

  // p(z) = p(x) initially
  px_ = computeIBpx(ws);
  pz_ = px_;
  // p(z|x) is identity
  pz_x_ = Eigen::MatrixXd::Identity(N, N);

  py_x_ = computeIBpyGivenX(ws, tasks, metric, config.py_x);

  // p(y|z) = p(y|x) (as p(z) = p(x) and p(z|x) = I_n
  py_z_ = py_x_;
  // p(y) is uniform
  py_ = computeIBpy(tasks);

  VLOG(10) << "p(x): " << px_.format(fmt);
  VLOG(10) << "p(z): " << pz_.format(fmt);
  VLOG(10) << "p(y): " << py_.format(fmt);
  VLOG(10) << "p(y|x): " << py_x_.format(fmt);
  VLOG(10) << "p(y|z): " << py_z_.format(fmt);
  VLOG(10) << "p(z|x): " << pz_x_.format(fmt);

  // initialize mutual information to starting values;
  I_xy_ = mutualInformation(py_, px_, py_x_);
  I_zy_prev_ = I_xy_;
  deltas_.clear();
}

double AgglomerativeClustering::scoreEdge(EdgeKey edge) {
  const auto fmt = getDefaultFormat();
  const auto p_s = pz_(edge.k1);
  const auto p_t = pz_(edge.k2);
  const auto total = p_s + p_t;
  Eigen::VectorXd prior(2);
  prior << p_s / total, p_t / total;
  Eigen::MatrixXd py_z_local(py_z_.rows(), 2);
  py_z_local.col(0) = py_z_.col(edge.k1);
  py_z_local.col(1) = py_z_.col(edge.k2);
  const auto divergence = jensenShannonDivergence(py_z_local, prior);
  VLOG(20) << "Scoring edge (" << edge << "): prior: " << prior.format(fmt)
           << ", p(y|z=z): " << py_z_local.format(fmt)
           << ", divergence: " << divergence;
  return total * divergence;
}

bool AgglomerativeClustering::updateFromEdge(EdgeKey edge) {
  // we merge target -> source
  const auto p_s = pz_(edge.k1);
  const auto p_t = pz_(edge.k2);
  // update new cluster probabilities
  pz_(edge.k1) = p_s + p_t;
  py_z_.col(edge.k1) =
      ((p_s * py_z_.col(edge.k1) + p_t * py_z_.col(edge.k2)) / (p_s + p_t)).eval();
  pz_x_.col(edge.k1) += pz_x_.col(edge.k2);

  // zero-out merged nodes
  pz_(edge.k2) = 0.0;
  py_z_.col(edge.k2).setConstant(0.0);
  pz_x_.col(edge.k2).setConstant(0.0);

  // for I[a; b] order is p(a), p(b), p(a|b)
  const auto I_zy = mutualInformation(py_, pz_, py_z_);
  const auto d_I_zy = I_zy_prev_ - I_zy;

  // avoid divide-by-zero and other weirdness with precision
  const auto delta = delta_weight_ * d_I_zy / I_xy_;
  VLOG(10) << "delta for (" << edge << "): " << delta;

  I_zy_prev_ = I_zy;
  deltas_.push_back(delta);
  return delta < config.max_delta;
}

bool AgglomerativeClustering::compareEdges(
    const std::pair<EdgeKey, double>& lhs,
    const std::pair<EdgeKey, double>& rhs) const {
  return lhs.second < rhs.second;
}

void AgglomerativeClustering::onlineReweighting(double Ixy, double delta_weight) {
  I_xy_ = Ixy;
  delta_weight_ = delta_weight;
}

std::string AgglomerativeClustering::summarize() const {
  if (deltas_.empty()) {
    return "0 merge(s), δ_0=N/A, δ_n=N/A";
  }

  const size_t num_merges =
      deltas_.back() <= config.max_delta ? deltas_.size() : deltas_.size() - 1;
  const std::string d0 = std::to_string(deltas_.front());
  const std::string dn = std::to_string(deltas_.back());

  std::stringstream ss;
  ss << num_merges << " merge(s), " << "δ_0=" << d0 << ", δ_n=" << dn;
  return ss.str();
}

}  // namespace hydra
