#include "clio/agglomerative_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/utils/disjoint_set.h>
#include <hydra/utils/display_utilities.h>
#include <spark_dsg/printing.h>

#include <numeric>

#include "clio/edge_selector.h"

namespace clio {

using namespace spark_dsg;
using Clusters = AgglomerativeClustering::Clusters;

void declare_config(AgglomerativeClustering::Config& config) {
  using namespace config;
  name("AgglomerativeClustering::Config");
  field(config.tasks, "tasks");
  config.metric.setOptional();
  field(config.metric, "metric");
  field(config.selector, "selector");
  field(config.filter_regions, "filter_regions");
}

void clusterAgglomerative(ClusteringWorkspace& ws,
                          const hydra::EmbeddingGroup& tasks,
                          EdgeSelector& edge_selector,
                          const hydra::EmbeddingDistance& metric,
                          bool reweight,
                          double I_xy,
                          double delta_weight,
                          int verbosity) {
  VLOG(verbosity) << "[IB] starting clustering with " << ws.edges.size() << " edges";

  edge_selector.setup(ws, tasks, metric);

  if (reweight) {
    edge_selector.onlineReweighting(I_xy, delta_weight);
  }

  VLOG(10) << "-----------------------------------";
  VLOG(10) << "Scoring edges";
  VLOG(10) << "-----------------------------------";
  for (auto& [edge, weight] : ws.edges) {
    const auto score = edge_selector.scoreEdge(edge);
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
          return edge_selector.compareEdges(lhs, rhs);
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
    if (!edge_selector.updateFromEdge(best_edge)) {
      // we've hit a stop criteria
      break;
    }

    const auto changed_edges = ws.addMerge(best_edge);
    VLOG(10) << "-----------------------------------";
    VLOG(10) << "Scoring changed edges";
    VLOG(10) << "-----------------------------------";
    for (const auto edge : changed_edges) {
      const auto score = edge_selector.scoreEdge(edge);
      VLOG(10) << "edge " << edge << ": " << score;
      ws.edges[edge] = score;
    }
    VLOG(10) << "-----------------------------------";
  }

  VLOG(verbosity) << "[IB] " << edge_selector.summarize();
}

AgglomerativeClustering::AgglomerativeClustering(const Config& config)
    : config(config::checkValid(config)),
      tasks_(config.tasks.create()),
      metric_(config.metric.create()),
      edge_selector_(new IBEdgeSelector(config.selector)) {}

Clusters AgglomerativeClustering::cluster(const SceneGraphLayer& layer,
                                          const NodeEmbeddingMap& features) const {
  if (tasks_->empty()) {
    LOG_FIRST_N(ERROR, 5) << "No tasks present: cannot cluster";
    return {};
  }

  ClusteringWorkspace ws(layer, features);
  clusterAgglomerative(ws, *tasks_, *edge_selector_, *metric_);

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

}  // namespace clio
