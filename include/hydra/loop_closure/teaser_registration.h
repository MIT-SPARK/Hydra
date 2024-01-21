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
#include <config_utilities/factory.h>
#include <gtsam/geometry/Pose3.h>
#include <teaser/registration.h>

#include <mutex>

#include "hydra/loop_closure/graph_registration.h"

namespace hydra::lcd {

using TeaserParams = teaser::RobustRegistrationSolver::Params;
using CorrespondenceFunc =
    std::function<bool(const SceneGraphNode&, const SceneGraphNode&)>;

struct GraphTeaserSolver : GraphRegistrationSolver {
  struct Config : LayerRegistrationConfig {
    std::string timer_prefix;
    std::string log_prefix;
    TeaserParams teaser;
  };

  GraphTeaserSolver(const Config& config, LayerId layer_id);

  virtual ~GraphTeaserSolver() = default;

  RegistrationSolution solve(const DynamicSceneGraph& dsg,
                             const GraphRegistrationInput& match,
                             NodeId query_agent_id) const override;

  const Config config;
  const LayerId layer_id;
  // registration call mutates the solver
  mutable teaser::RobustRegistrationSolver solver;
  std::string timer_prefix;
  std::string log_prefix;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphRegistrationSolver,
                                     GraphTeaserSolver,
                                     Config,
                                     LayerId>("GraphTeaserSolver");
};

void declare_config(GraphTeaserSolver::Config& config);

struct LayerRegistrationProblem {
  mutable std::set<NodeId> src_nodes;
  mutable std::set<NodeId> dest_nodes;
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

LayerRegistrationSolution registerDsgLayer(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem& problem,
    const SceneGraphLayer& src,
    const CorrespondenceFunc& correspondence_func);

LayerRegistrationSolution registerDsgLayerPairwise(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem& problem,
    const SceneGraphLayer& src);

LayerRegistrationSolution registerDsgLayerSemantic(
    const LayerRegistrationConfig& config,
    teaser::RobustRegistrationSolver& solver,
    const LayerRegistrationProblem& problem,
    const SceneGraphLayer& src);

}  // namespace hydra::lcd
