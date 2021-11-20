#pragma once
#include "kimera_dsg_builder/dsg_lcd_matching.h"
#include "kimera_dsg_builder/incremental_types.h"

#include <gtsam/geometry/Pose3.h>
#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg/scene_graph_layer.h>
#include <teaser/registration.h>

namespace kimera {
namespace lcd {

struct LayerRegistrationConfig {
  size_t min_correspondences = 5;
  size_t min_inliers = 5;
};

using RegistrationFunc =
    std::function<DsgRegistrationSolution(incremental::SharedDsgInfo&,
                                          const LayerSearchResults&,
                                          NodeId)>;

using TeaserParams = teaser::RobustRegistrationSolver::Params;

struct PlaceRegistrationFunctor {
  PlaceRegistrationFunctor(const LayerRegistrationConfig& config,
                           const TeaserParams& params);

  DsgRegistrationSolution operator()(incremental::SharedDsgInfo& dsg,
                                     const LayerSearchResults& match,
                                     NodeId query_agent_id);

  LayerRegistrationConfig config;
  teaser::RobustRegistrationSolver solver;
};

struct ObjectRegistrationFunctor {
  ObjectRegistrationFunctor(const LayerRegistrationConfig& config,
                            const TeaserParams& params);

  DsgRegistrationSolution operator()(incremental::SharedDsgInfo& dsg,
                                     const LayerSearchResults& match,
                                     NodeId query_agent_id);

  LayerRegistrationConfig config;
  teaser::RobustRegistrationSolver solver;
};

DsgRegistrationSolution registerAgentMatch(incremental::SharedDsgInfo& dsg,
                                           const LayerSearchResults& match,
                                           NodeId query_agent_id);

// start of graph registration

using CorrespondenceFunc =
    std::function<bool(const SceneGraphNode&, const SceneGraphNode&)>;

template <typename NodeSet = std::list<NodeId>>
struct LayerRegistrationProblem {
  NodeSet src_nodes;
  NodeSet dest_nodes;
  SceneGraphLayer* dest_layer = nullptr;
  std::mutex* src_mutex = nullptr;
  std::mutex* dest_mutex = nullptr;
  size_t min_correspondences;
  size_t min_inliers;
};

struct LayerRegistrationSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  gtsam::Pose3 dest_T_src;
  std::vector<std::pair<NodeId, NodeId>> inliers;
};

template <typename NodeSet>
LayerRegistrationSolution registerDsgLayer(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem<NodeSet>& problem,
    const SceneGraphLayer& src,
    const CorrespondenceFunc& correspondence_func) {
  std::vector<std::pair<NodeId, NodeId>> correspondences;
  correspondences.reserve(problem.src_nodes.size() * problem.dest_nodes.size());

  if (problem.src_mutex) {
    problem.src_mutex->lock();
  }

  if (problem.dest_mutex) {
    problem.dest_mutex->lock();
  }

  const SceneGraphLayer& dest = problem.dest_layer ? *problem.dest_layer : src;
  for (const auto& src_id : problem.src_nodes) {
    for (const auto& dest_id : problem.dest_nodes) {
      const SceneGraphNode& src_node = src.getNode(src_id).value();
      const SceneGraphNode& dest_node = dest.getNode(dest_id).value();

      if (correspondence_func(src_node, dest_node)) {
        correspondences.emplace_back(src_id, dest_id);
      }
    }
  }

  if (problem.src_mutex) {
    problem.src_mutex->unlock();
  }

  if (problem.dest_mutex) {
    problem.dest_mutex->unlock();
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_points(3, correspondences.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> dest_points(3, correspondences.size());
  for (size_t i = 0; i < correspondences.size(); ++i) {
    const auto correspondence = correspondences[i];
    src_points.col(i) = src.getPosition(correspondence.first);
    dest_points.col(i) = dest.getPosition(correspondence.second);
  }

  if (correspondences.size() < config.min_correspondences) {
    VLOG(2) << "not enough correspondences for registration at layer " << src.id << ": "
            << correspondences.size() << " / " << config.min_correspondences;
    return {};
  }

  VLOG(3) << "=======================================================";
  VLOG(3) << "Source: " << std::endl << src_points;
  VLOG(3) << "Dest: " << std::endl << dest_points;

  VLOG(1) << "Registering layer " << src.id << " with " << correspondences.size()
          << " correspondences out of " << problem.src_nodes.size() << " source and "
          << problem.dest_nodes.size() << " destination nodes";

  auto params = solver.getParams();
  solver.reset(params);

  teaser::RegistrationSolution result = solver.solve(src_points, dest_points);
  if (!result.valid) {
    return {};
  }

  std::vector<std::pair<NodeId, NodeId>> valid_correspondences;
  valid_correspondences.reserve(
      std::min(problem.src_nodes.size(), problem.dest_nodes.size()));

  auto inliers = solver.getInlierMaxClique();
  if (inliers.size() < config.min_inliers) {
    VLOG(2) << "not enough inliers for registration at layer " << src.id << ": "
            << inliers.size() << " / " << config.min_inliers;
    return {};
  }

  for (const auto& index : inliers) {
    CHECK_LT(static_cast<size_t>(index), correspondences.size());
    valid_correspondences.push_back(correspondences.at(index));
  }

  return {true,
          gtsam::Pose3(gtsam::Rot3(result.rotation), result.translation),
          valid_correspondences};
}

template <typename NodeSet>
LayerRegistrationSolution registerDsgLayerPairwise(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem<NodeSet>& problem,
    const SceneGraphLayer& src) {
  return registerDsgLayer(
      config, solver, problem, src, [](const auto&, const auto&) { return true; });
}

template <typename NodeSet>
LayerRegistrationSolution registerDsgLayerSemantic(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem<NodeSet>& problem,
    const SceneGraphLayer& src) {
  return registerDsgLayer(
      config,
      solver,
      problem,
      src,
      [](const SceneGraphNode& src_node, const SceneGraphNode& dest_node) {
        return src_node.attributes<SemanticNodeAttributes>().semantic_label ==
               dest_node.attributes<SemanticNodeAttributes>().semantic_label;
      });
}

}  // namespace lcd
}  // namespace kimera
