
#include "clio/ib_utils.h"

#include <glog/logging.h>
#include <hydra/utils/printing.h>

#include <numeric>
#include <vector>

namespace clio {

using namespace spark_dsg;
using Indices = std::vector<std::pair<size_t, size_t>>;

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

Eigen::MatrixXd computeIBpyGivenX(const ClusteringWorkspace& ws,
                                  const hydra::EmbeddingGroup& tasks,
                                  const hydra::EmbeddingDistance& metric,
                                  const PyGivenXConfig& config) {
  const auto fmt = hydra::getDefaultFormat();

  size_t N = ws.size();
  size_t M = tasks.embeddings.size() + 1;

  Eigen::MatrixXd py_x = Eigen::MatrixXd::Ones(M, N) * 1e-12;
  Eigen::MatrixXd py_x_temp = Eigen::MatrixXd::Zero(M, N);
  py_x_temp.row(0).setConstant(config.score_threshold);
  VLOG(15) << "----------------------------------------";
  VLOG(15) << "Computing workspace feature scores";
  VLOG(15) << "----------------------------------------";
  for (auto&& [idx, feature] : ws.features) {
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

Eigen::VectorXd computeIBpx(const ClusteringWorkspace& ws) {
  // p(x) is uniform
  size_t N = ws.size();
  return Eigen::VectorXd::Constant(N, 1.0 / static_cast<double>(N));
}

Eigen::VectorXd computeIBpy(const hydra::EmbeddingGroup& tasks) {
  size_t M = tasks.embeddings.size() + 1;
  return Eigen::VectorXd::Constant(M, 1.0 / static_cast<double>(M));
}

double computeDeltaWeight(const SceneGraphLayer& layer,
                          const std::vector<NodeId>& nodes) {
  return static_cast<double>(nodes.size()) / static_cast<double>(layer.numNodes());
}

}  // namespace clio
