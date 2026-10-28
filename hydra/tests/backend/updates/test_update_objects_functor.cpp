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
#include <gtest/gtest.h>
#include <hydra/backend/updates/update_objects_functor.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/scene_graph.h>

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

}  // namespace

TEST(UpdateObjectsFunctor, ObjectUpdate) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  {  // scope limiting moved attrs access
    auto attrs = std::make_unique<ObjectNodeAttributes>();
    attrs->position << 1.0, 2.0, 3.0;
    attrs->bounding_box.type = BoundingBox::Type::AABB;
    attrs->bounding_box.world_P_center << 1.0f, 2.0f, 3.0f;
    attrs->is_active = true;
    graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs));
  }

  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  UpdateObjectsFunctor functor(UpdateObjectsFunctor::Config{});
  callWithUnmerged(functor, *dsg, info, false);

  {
    // No mesh, so nothing should change
    const Eigen::Vector3d expected_pos(1.0, 2.0, 3.0);
    const Eigen::Vector3f expected_dims(0.0, 0.0, 0.0);
    const auto& result = graph.getNode(0).attributes<ObjectNodeAttributes>();
    EXPECT_NEAR(0.0, (expected_pos - result.position).norm(), 1.0e-7);
    EXPECT_NEAR(
        0.0,
        (expected_pos.cast<float>() - result.bounding_box.world_P_center).norm(),
        1.0e-7);
    EXPECT_NEAR(0.0, (expected_dims - result.bounding_box.dimensions).norm(), 1.0e-7);
  }

  auto mesh = std::make_shared<Mesh>();
  mesh->resizeVertices(2);
  mesh->setPos(0, Mesh::Pos(-1.0, -2.0, -3.0));
  mesh->setPos(1, Mesh::Pos(1.0, 2.0, 3.0));
  graph.setMesh(mesh);
  auto& attrs = graph.getNode(0).attributes<ObjectNodeAttributes>();
  attrs.mesh_connections = {0, 1};

  callWithUnmerged(functor, *dsg, info, false);

  {
    // valid mesh: things should change
    const Eigen::Vector3d expected_pos(0.0, 0.0, 0.0);
    const Eigen::Vector3f expected_dims(2.0, 4.0, 6.0);
    const auto& result = graph.getNode(0).attributes<ObjectNodeAttributes>();
    EXPECT_NEAR(0.0, (expected_pos - result.position).norm(), 1.0e-7);
    EXPECT_NEAR(
        0.0,
        (expected_pos.cast<float>() - result.bounding_box.world_P_center).norm(),
        1.0e-7);
    EXPECT_NEAR(0.0, (expected_dims - result.bounding_box.dimensions).norm(), 1.0e-7);
  }
}

TEST(UpdateObjectsFunctor, ObjectUpdateMergeLC) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  auto attrs0 = std::make_unique<ObjectNodeAttributes>();
  attrs0->position << 1.0, 2.0, 3.0;
  attrs0->bounding_box.type = BoundingBox::Type::AABB;
  attrs0->bounding_box.world_P_center << 1.0f, 2.0f, 3.0f;
  attrs0->semantic_label = 1u;
  attrs0->is_active = false;
  attrs0->mesh_connections = {0, 1};
  graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs0));

  auto attrs1 = std::make_unique<ObjectNodeAttributes>();
  attrs1->position << 2.0, 3.0, 4.0;
  attrs1->bounding_box.type = BoundingBox::Type::AABB;
  attrs1->bounding_box.world_P_center << 2.0f, 3.0f, 4.0f;
  attrs1->semantic_label = 1u;
  attrs1->is_active = true;
  attrs1->mesh_connections = {0, 1};
  graph.emplaceNode(DsgLayers::OBJECTS, 1, std::move(attrs1));

  auto mesh = std::make_shared<Mesh>();
  mesh->resizeVertices(2);
  mesh->setPos(0, Mesh::Pos(-1.0, -2.0, -3.0));
  mesh->setPos(1, Mesh::Pos(1.0, 2.0, 3.0));
  graph.setMesh(mesh);

  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, true, {}});
  UpdateObjectsFunctor functor(UpdateObjectsFunctor::Config{});
  const auto result_merges = callWithUnmerged(functor, *dsg, info, true);

  const auto& result0 = graph.getNode(0).attributes<ObjectNodeAttributes>();

  Eigen::Vector3d expected_pos(0.0, 0.0, 0.0);
  EXPECT_NEAR(0.0, (expected_pos - result0.position).norm(), 1.0e-7);
  EXPECT_NEAR(0.0,
              (expected_pos.cast<float>() - result0.bounding_box.world_P_center).norm(),
              1.0e-7);
  Eigen::Vector3f expected_max(2.0, 4.0, 6.0);
  EXPECT_NEAR(0.0, (expected_max - result0.bounding_box.dimensions).norm(), 1.0e-7);

  MergeList expected{{1, 0}};
  EXPECT_EQ(result_merges, expected);
}

TEST(UpdateObjectsFunctor, ObjectUpdateMergeNoLC) {
  auto dsg = test::makeSharedDsg();
  auto& graph = *dsg->graph;

  auto attrs0 = std::make_unique<ObjectNodeAttributes>();
  attrs0->position << 1.0, 2.0, 3.0;
  attrs0->bounding_box.type = BoundingBox::Type::AABB;
  attrs0->bounding_box.world_P_center << 1.0f, 2.0f, 3.0f;
  attrs0->semantic_label = 1u;
  attrs0->mesh_connections = {0, 1};
  attrs0->is_active = false;
  graph.emplaceNode(DsgLayers::OBJECTS, 0, std::move(attrs0));

  auto attrs1 = std::make_unique<ObjectNodeAttributes>();
  attrs1->position << 2.0, 3.0, 4.0;
  attrs1->bounding_box.type = BoundingBox::Type::AABB;
  attrs1->bounding_box.world_P_center << 2.0f, 3.0f, 4.0f;
  attrs1->semantic_label = 1u;
  attrs1->mesh_connections = {0, 1};
  attrs1->is_active = true;
  graph.emplaceNode(DsgLayers::OBJECTS, 1, std::move(attrs1));

  auto mesh = std::make_shared<Mesh>();
  mesh->resizeVertices(2);
  mesh->setPos(0, Mesh::Pos(-1.0, -2.0, -3.0));
  mesh->setPos(1, Mesh::Pos(1.0, 2.0, 3.0));
  graph.setMesh(mesh);

  UpdateInfo::ConstPtr info(new UpdateInfo{0, nullptr, nullptr, false, {}});
  UpdateObjectsFunctor functor(UpdateObjectsFunctor::Config{});
  const auto result_merges = callWithUnmerged(functor, *dsg, info, true);

  MergeList expected{{1, 0}};
  EXPECT_EQ(result_merges, expected);
}

// The geometry comes from the optimized mesh, but the orientation can only come from
// the deformation graph.
TEST(UpdateObjectsFunctor, ObjectOrientationDeformed) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).matrix();
  transform.translation() << 1.0, 0.0, 0.0;
  const Eigen::Quaterniond odom_rot(Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX()));
  const Eigen::Matrix3d expected_rot = transform.linear() * odom_rot.toRotationMatrix();

  auto make_attrs = [&](const Eigen::Vector3d& pos) {
    auto attrs = std::make_unique<ObjectNodeAttributes>();
    attrs->position = pos;
    attrs->world_R_object = odom_rot;
    attrs->bounding_box.type = BoundingBox::Type::AABB;
    attrs->bounding_box.world_P_center = pos.cast<float>();
    attrs->semantic_label = 1u;
    attrs->mesh_connections = {0, 1};
    attrs->is_active = true;
    attrs->last_update_time_ns = 10u;
    return attrs;
  };

  auto unmerged = test::makeSharedDsg()->graph;
  auto dsg = test::makeSharedDsg();
  for (const auto node_id : {0, 1}) {
    unmerged->emplaceNode(DsgLayers::OBJECTS, node_id, make_attrs({1.0, 1.0, 0.0}));
    dsg->graph->emplaceNode(DsgLayers::OBJECTS, node_id, make_attrs({1.0, 1.0, 0.0}));
  }

  // the "optimized" mesh
  auto mesh = std::make_shared<Mesh>();
  mesh->resizeVertices(2);
  mesh->setPos(0, Mesh::Pos(-1.0, -2.0, -3.0));
  mesh->setPos(1, Mesh::Pos(1.0, 2.0, 3.0));
  dsg->graph->setMesh(mesh);
  unmerged->setMesh(mesh);

  auto dgraph = test::makeRigidDeformationGraph(transform);
  UpdateInfo::ConstPtr info(
      new UpdateInfo{0, nullptr, nullptr, true, {}, dgraph.get()});
  UpdateObjectsFunctor functor(UpdateObjectsFunctor::Config{});

  for (size_t round = 0; round < 2; ++round) {
    if (round > 0) {
      // archived nodes are no longer refreshed by the frontend and keep the
      // deformed values; deforming them again must not compound
      for (const auto node_id : {0, 1}) {
        unmerged->getNode(node_id).attributes().is_active = false;
      }
    }

    functor.call(*unmerged, *dsg, info);
    for (const auto graph : {unmerged.get(), dsg->graph.get()}) {
      const auto& result = graph->getNode(0).attributes<ObjectNodeAttributes>();
      EXPECT_TRUE(
          result.world_R_object.toRotationMatrix().isApprox(expected_rot, 1.0e-6))
          << "round " << round;
      // geometry is computed from the mesh, not from the deformation graph
      EXPECT_NEAR(result.position.norm(), 0.0, 1.0e-6) << "round " << round;
      EXPECT_TRUE(
          result.bounding_box.dimensions.isApprox(Eigen::Vector3f(2.0, 4.0, 6.0)))
          << "round " << round;
    }
  }

  // merged attributes get the same orientation
  const auto hooks = functor.hooks();
  ASSERT_TRUE(hooks.merge);
  auto odom = make_attrs({1.0, 1.0, 0.0});
  unmerged->setNodeAttributes(0, std::move(odom));
  const auto merged = hooks.merge(*unmerged, {0, 1});
  ASSERT_TRUE(merged);
  const auto& merged_attrs = dynamic_cast<const ObjectNodeAttributes&>(*merged);
  EXPECT_TRUE(
      merged_attrs.world_R_object.toRotationMatrix().isApprox(expected_rot, 1.0e-6));
  EXPECT_NEAR(merged_attrs.position.norm(), 0.0, 1.0e-6);
}

}  // namespace hydra
