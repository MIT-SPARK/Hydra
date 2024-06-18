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
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <teaser/registration.h>

#include <mutex>

#include "hydra/common/common.h"
#include "hydra/loop_closure/descriptor_matching.h"
#include "hydra/loop_closure/registration_solution.h"
#include "hydra/loop_closure/subgraph_extraction.h"

namespace hydra::lcd {

struct LayerRegistrationConfig {
  size_t min_correspondences = 5;
  size_t min_inliers = 5;
  size_t max_same_nodes = 2;
  bool log_registration_problem = false;
  bool use_pairwise_registration = false;
  std::string registration_output_path = "";
  bool recreate_subgraph = false;
  SubgraphConfig subgraph_extraction;
};

struct DsgRegistrationInput {
  std::set<NodeId> query_nodes;
  std::set<NodeId> match_nodes;
  NodeId query_root;
  NodeId match_root;
};

struct DsgRegistrationSolver {
  using Ptr = std::unique_ptr<DsgRegistrationSolver>;

  virtual ~DsgRegistrationSolver() = default;

  virtual RegistrationSolution solve(const DynamicSceneGraph& dsg,
                                     const DsgRegistrationInput& match,
                                     NodeId query_agent_id) const = 0;
};

using TeaserParams = teaser::RobustRegistrationSolver::Params;

struct DsgTeaserSolver : DsgRegistrationSolver {
  DsgTeaserSolver(LayerId layer_id,
                  const LayerRegistrationConfig& config,
                  const TeaserParams& params);

  virtual ~DsgTeaserSolver() = default;

  RegistrationSolution solve(const DynamicSceneGraph& dsg,
                             const DsgRegistrationInput& match,
                             NodeId query_agent_id) const override;

  LayerId layer_id;
  LayerRegistrationConfig config;
  std::string timer_prefix;
  std::string log_prefix;
  // registration call mutates the solver
  mutable teaser::RobustRegistrationSolver solver;
};

using CorrespondenceFunc =
    std::function<bool(const SceneGraphNode&, const SceneGraphNode&)>;

template <typename NodeSet = std::list<NodeId>>
struct LayerRegistrationProblem {
  mutable NodeSet src_nodes;
  mutable NodeSet dest_nodes;
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
std::list<NodeId> pruneSet(const SceneGraphLayer& layer, NodeSet& nodes) {
  std::list<NodeId> pruned;
  auto iter = nodes.begin();
  while (iter != nodes.end()) {
    if (layer.hasNode(*iter)) {
      ++iter;
    } else {
      pruned.push_back(*iter);
      iter = nodes.erase(iter);
    }
  }
  return pruned;
}

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

  const auto src_pruned = pruneSet(src, problem.src_nodes);
  if (!src_pruned.empty()) {
    VLOG(3) << "[DSG LCD] Found invalid source nodes in registration: "
            << displayNodeSymbolContainer(src_pruned);
  }

  const auto dest_pruned = pruneSet(dest, problem.dest_nodes);
  if (!dest_pruned.empty()) {
    VLOG(3) << "[DSG LCD] Found invalid destination nodes in registration: "
            << displayNodeSymbolContainer(dest_pruned);
  }

  for (const auto& src_id : problem.src_nodes) {
    auto src_node_opt = src.findNode(src_id);
    CHECK(src_node_opt);
    const auto& src_node = *src_node_opt;

    for (const auto& dest_id : problem.dest_nodes) {
      auto dest_node_opt = dest.findNode(dest_id);
      CHECK(dest_node_opt);
      const auto& dest_node = *dest_node_opt;

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

  VLOG(20) << "=======================================================";
  VLOG(20) << "Source: " << std::endl << src_points;
  VLOG(20) << "Dest: " << std::endl << dest_points;

  VLOG(1) << "[DSG LCD] Registering layer " << src.id << " with "
          << correspondences.size() << " correspondences out of "
          << problem.src_nodes.size() << " source and " << problem.dest_nodes.size()
          << " destination nodes";

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
    VLOG(2) << "[DSG LCD] Not enough inliers for registration at layer " << src.id
            << ": " << inliers.size() << " / " << config.min_inliers;
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

}  // namespace hydra::lcd
