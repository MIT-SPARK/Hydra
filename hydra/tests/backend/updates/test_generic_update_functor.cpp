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
#include <hydra/backend/merge_tracker.h>
#include <hydra/backend/updates/generic_update_functor.h>
#include <kimera_pgmo/deformation_graph.h>
#include <spark_dsg/scene_graph.h>

#include "hydra_test/resources.h"
#include "hydra_test/rigid_deformation.h"
#include "hydra_test/shared_dsg_fixture.h"

using namespace spark_dsg;

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

Eigen::Isometry3d makeTransform(double yaw, const Eigen::Vector3d& translation) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
  transform.translation() = translation;
  return transform;
}

std::unique_ptr<ObjectNodeAttributes> makeObject(const Eigen::Vector3d& pos,
                                                 const Eigen::Quaterniond& rot) {
  auto attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->position = pos;
  attrs->world_R_object = rot;
  attrs->bounding_box =
      BoundingBox(Eigen::Vector3f(1.0f, 2.0f, 3.0f), pos.cast<float>());
  attrs->is_active = true;
  attrs->last_update_time_ns = 10u;
  return attrs;
}

void expectPose(const ObjectNodeAttributes& attrs,
                const Eigen::Isometry3d& transform,
                const Eigen::Vector3d& odom_pos,
                const Eigen::Quaterniond& odom_rot) {
  const Eigen::Vector3d expected_pos = transform * odom_pos;
  const Eigen::Matrix3d expected_rot = transform.linear() * odom_rot.toRotationMatrix();
  EXPECT_NEAR((attrs.position - expected_pos).norm(), 0.0, 1.0e-6);
  EXPECT_TRUE(attrs.world_R_object.toRotationMatrix().isApprox(expected_rot, 1.0e-6));
  EXPECT_NEAR((attrs.bounding_box.world_P_center.cast<double>() - expected_pos).norm(),
              0.0,
              1.0e-5);
  EXPECT_TRUE(attrs.bounding_box.world_R_center.cast<double>().isApprox(
      transform.linear(), 1.0e-5));
}

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

  const auto filepath = test::get_resource_path() / "graph.json";
  auto dgraph = kimera_pgmo::DeformationGraph::load(filepath);

  auto config = defaultConfig();
  config.enable_merging = false;
  VLOG(1) << "Using config:\n" << config::toString(config);

  GenericUpdateFunctor functor(config);
  auto info = std::make_shared<UpdateInfo>();
  info->deformation_graph = dgraph.get();
  callWithUnmerged(functor, *dsg, info, false);

  const auto& result = graph.getNode(0).attributes();
  const Eigen::Vector3d expected(1.0, 2.0, 3.0);
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);

  // second call shouldn't change position
  graph.getNode(0).attributes().is_active = false;
  callWithUnmerged(functor, *dsg, info, false);
  EXPECT_NEAR(0.0, (expected - result.position).norm(), 1.0e-7);
}

TEST(GenericUpdateFunctor, DeformsObjectPose) {
  const Eigen::Vector3d pos(1.0, 1.0, 0.5);
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
  auto unmerged = test::makeSharedDsg()->graph;
  auto dsg = test::makeSharedDsg();
  unmerged->emplaceNode(DsgLayers::OBJECTS, 0, makeObject(pos, rot));
  dsg->graph->emplaceNode(DsgLayers::OBJECTS, 0, makeObject(pos, rot));

  const auto transform = makeTransform(M_PI / 2.0, Eigen::Vector3d(2.0, -1.0, 0.0));
  auto dgraph = test::makeRigidDeformationGraph(transform);
  UpdateInfo::ConstPtr info(
      new UpdateInfo{0, nullptr, nullptr, true, {}, dgraph.get()});
  auto config = defaultConfig();
  config.enable_merging = false;
  GenericUpdateFunctor functor(config);
  functor.call(*unmerged, *dsg, info);

  expectPose(
      dsg->graph->getNode(0).attributes<ObjectNodeAttributes>(), transform, pos, rot);
  expectPose(
      unmerged->getNode(0).attributes<ObjectNodeAttributes>(), transform, pos, rot);
}

TEST(GenericUpdateFunctor, MergeHookAppliesLastTransform) {
  const Eigen::Vector3d pos(1.0, 1.0, 0.5);
  const Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
  auto unmerged = test::makeSharedDsg()->graph;
  auto dsg = test::makeSharedDsg();
  for (const auto node_id : {0, 1}) {
    unmerged->emplaceNode(DsgLayers::OBJECTS, node_id, makeObject(pos, rot));
    dsg->graph->emplaceNode(DsgLayers::OBJECTS, node_id, makeObject(pos, rot));
  }

  const auto transform = makeTransform(-M_PI / 3.0, Eigen::Vector3d(0.5, 0.5, 1.0));
  auto dgraph = test::makeRigidDeformationGraph(transform);
  UpdateInfo::ConstPtr info(
      new UpdateInfo{0, nullptr, nullptr, true, {}, dgraph.get()});
  GenericUpdateFunctor functor(defaultConfig());
  const auto hooks = functor.hooks();
  ASSERT_TRUE(hooks.merge);

  // no deformation yet: the merged attributes are the parent's
  auto attrs = hooks.merge(*unmerged, {0, 1});
  ASSERT_TRUE(attrs);
  EXPECT_EQ(*attrs, unmerged->getNode(0).attributes());

  functor.call(*unmerged, *dsg, info);

  // the unmerged parent is already deformed: merging must not deform it again
  attrs = hooks.merge(*unmerged, {0, 1});
  ASSERT_TRUE(attrs);
  expectPose(dynamic_cast<const ObjectNodeAttributes&>(*attrs), transform, pos, rot);

  // a parent that holds the odometric pose (e.g., refreshed by the frontend) is
  // brought into the optimized frame
  unmerged->setNodeAttributes(0, makeObject(pos, rot));
  attrs = hooks.merge(*unmerged, {0, 1});
  ASSERT_TRUE(attrs);
  expectPose(dynamic_cast<const ObjectNodeAttributes&>(*attrs), transform, pos, rot);

  // the merge tracker writes the hook result into the merged graph
  MergeTracker tracker;
  EXPECT_EQ(tracker.applyMerges(*unmerged, {{1, 0}}, *dsg, hooks.merge), 1u);
  EXPECT_FALSE(dsg->graph->hasNode(1));
  expectPose(
      dsg->graph->getNode(0).attributes<ObjectNodeAttributes>(), transform, pos, rot);
}

}  // namespace hydra
