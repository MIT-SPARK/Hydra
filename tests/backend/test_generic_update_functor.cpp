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
#include <config_utilities/printing.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <gtest/gtest.h>
#include <hydra/backend/generic_update_functor.h>
#include <kimera_pgmo/deformation_graph.h>

#include "hydra_test/resources.h"
#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {

namespace {

MergeList callWithUnmerged(const UpdateFunctor& functor,
                           SharedDsgInfo& dsg,
                           const UpdateInfo::ConstPtr& info,
                           bool enable_merging) {
  const auto unmerged = dsg.graph->clone();
  functor.call(*unmerged, dsg, info);
  const auto hooks = functor.hooks();
  if (enable_merging && hooks.find_merges) {
    return hooks.find_merges(*unmerged, info);
  } else {
    return {};
  }
}

GenericUpdateFunctor::Config defaultConfig() { return {5, "OBJECTS"}; }

}  // namespace

TEST(GenericUpdateFunctor, noUpdate) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  const Eigen::Vector3d expected(1.0, 2.0, 3.0);
  {  // scope limiting moved attrs access
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->position = expected;
    attrs->is_active = true;
    attrs->last_update_time_ns = 10u;
    graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs));
  }

  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  auto config = defaultConfig();
  config.enable_merging = false;
  GenericUpdateFunctor functor(config);
  callWithUnmerged(functor, *dsg, info, false);

  // No deformation, so nothing should change
  const auto& result = graph.getNode(0).attributes();
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);
}

TEST(GenericUpdateFunctor, shouldUpdate) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  {  // scope limiting moved attrs access
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->position << 0, 3, 1;
    attrs->is_active = true;
    attrs->last_update_time_ns = 10u;
    graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs));
  }

  kimera_pgmo::DeformationGraph dgraph;
  dgraph.load(test::get_resource_path() / "graph.dgrf");

  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}, &dgraph});
  auto config = defaultConfig();
  config.enable_merging = false;
  VLOG(1) << "Using config:\n" << config::toString(config);

  GenericUpdateFunctor functor(config);
  // the unmerged graph persists across spins and stays odometric; the functor only
  // ever writes the merged graph
  const auto unmerged = graph.clone();
  functor.call(*unmerged, *dsg, info);

  const auto& result = graph.getNode(0).attributes();
  const Eigen::Vector3d expected(1.0, 2.0, 3.0);
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);

  {  // the odometric source is never touched
    const Eigen::Vector3d odometric(0.0, 3.0, 1.0);
    const auto& src = unmerged->getNode(0).attributes();
    EXPECT_NEAR(0.0, (odometric - src.position).norm(), 1.0e-7);
  }

  // re-deforming an archived node must not compound
  graph.getNode(0).attributes().is_active = false;
  unmerged->getNode(0).attributes().is_active = false;
  functor.call(*unmerged, *dsg, info);
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);
}

}  // namespace hydra
