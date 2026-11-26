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
#include <gtsam/geometry/Pose3.h>
#include <spark_dsg/scene_graph_layer.h>
#include <teaser/registration.h>

#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "hydra/loop_closure/registration_solution.h"
#include "hydra/loop_closure/subgraph_extraction.h"
#include "hydra/utils/verbosity_config.h"

namespace hydra::lcd {

struct LayerRegistrationConfig : VerbosityConfig {
  LayerRegistrationConfig() : VerbosityConfig("lcd_registration") {}

  size_t min_correspondences = 5;
  size_t min_inliers = 5;
  size_t max_same_nodes = 2;
  bool log_registration_problem = false;
  bool use_pairwise_registration = false;
  std::string registration_output_path = "";
  bool recreate_subgraph = false;
  SubgraphConfig subgraph_extraction;
};

void declare_config(LayerRegistrationConfig& config);

struct DsgRegistrationInput {
  std::set<spark_dsg::NodeId> query_nodes;
  std::set<spark_dsg::NodeId> match_nodes;
  spark_dsg::NodeId query_root;
  spark_dsg::NodeId match_root;
};

struct DsgRegistrationSolver {
  using Ptr = std::unique_ptr<DsgRegistrationSolver>;

  virtual ~DsgRegistrationSolver() = default;

  virtual RegistrationSolution solve(const spark_dsg::SceneGraph& dsg,
                                     const DsgRegistrationInput& match,
                                     spark_dsg::NodeId query_agent_id) const = 0;
};

using TeaserParams = teaser::RobustRegistrationSolver::Params;

struct DsgTeaserSolver : DsgRegistrationSolver {
  DsgTeaserSolver(const std::string& layer,
                  const LayerRegistrationConfig& config,
                  const TeaserParams& params);

  virtual ~DsgTeaserSolver() = default;

  RegistrationSolution solve(const spark_dsg::SceneGraph& dsg,
                             const DsgRegistrationInput& match,
                             spark_dsg::NodeId query_agent_id) const override;

  const std::string layer_id;
  LayerRegistrationConfig config;
  std::string timer_prefix;
  std::string log_prefix;
  // registration call mutates the solver
  mutable teaser::RobustRegistrationSolver solver;
};

using CorrespondenceFunc = std::function<bool(const spark_dsg::SceneGraphNode&,
                                              const spark_dsg::SceneGraphNode&)>;

//! Candidate IDs are unique and ordered; registration prunes missing nodes.
struct LayerRegistrationProblem {
  mutable std::set<spark_dsg::NodeId> src_nodes;
  mutable std::set<spark_dsg::NodeId> dest_nodes;
  spark_dsg::SceneGraphLayer* dest_layer = nullptr;
  std::mutex* src_mutex = nullptr;
  std::mutex* dest_mutex = nullptr;
  size_t min_correspondences;
  size_t min_inliers;
};

struct LayerRegistrationSolution {
  bool valid = false;
  gtsam::Pose3 dest_T_src;
  std::vector<std::pair<spark_dsg::NodeId, spark_dsg::NodeId>> inliers;
};

LayerRegistrationSolution registerDsgLayer(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem& problem,
    const spark_dsg::SceneGraphLayer& src,
    const CorrespondenceFunc& correspondence_func);

LayerRegistrationSolution registerDsgLayerPairwise(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem& problem,
    const spark_dsg::SceneGraphLayer& src);

LayerRegistrationSolution registerDsgLayerSemantic(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem& problem,
    const spark_dsg::SceneGraphLayer& src);

}  // namespace hydra::lcd
