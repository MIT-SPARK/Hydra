#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph.h>

#include "hydra/backend/deformation_interpolator.h"
#include "hydra_test/rigid_deformation.h"
#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {

using spark_dsg::BoundingBox;
using spark_dsg::DsgLayers;
using spark_dsg::KhronosObjectAttributes;
using spark_dsg::LayerView;
using spark_dsg::NodeAttributes;
using spark_dsg::ObjectNodeAttributes;
using spark_dsg::SceneGraph;
using spark_dsg::SemanticNodeAttributes;

namespace {

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
  EXPECT_TRUE(attrs.world_R_object.toRotationMatrix().isApprox(expected_rot, 1.0e-6))
      << "expected:\n"
      << expected_rot << "\nactual:\n"
      << attrs.world_R_object.toRotationMatrix();
  EXPECT_NEAR((attrs.bounding_box.world_P_center.cast<double>() - expected_pos).norm(),
              0.0,
              1.0e-5);
  EXPECT_TRUE(attrs.bounding_box.world_R_center.cast<double>().isApprox(
      transform.linear(), 1.0e-5));
  EXPECT_TRUE(
      attrs.bounding_box.dimensions.isApprox(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
}

}  // namespace

TEST(DeformationInterpolator, KhronosFallbackTimestampOnInsert) {
  KhronosObjectAttributes attrs;
  attrs.last_update_time_ns = 0;
  attrs.last_observed_ns = {100};
  attrs.is_active = true;

  NodeCache cache;
  auto entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->timestamp, 100u);

  entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->timestamp, 100u);
}

TEST(DeformationInterpolator, BoundingBoxCorrect) {
  const BoundingBox bbox1(Eigen::Vector3f(1.0f, 1.0f, 1.0f),
                          Eigen::Vector3f(0.0f, 0.0f, 0.0f));
  const BoundingBox bbox2(Eigen::Vector3f(2.0f, 2.0f, 2.0f),
                          Eigen::Vector3f(3.0f, 3.0f, 3.0f));

  SemanticNodeAttributes attrs;
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;
  attrs.bounding_box = bbox1;

  NodeCache cache;
  auto entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox, bbox1);

  attrs.bounding_box = bbox2;
  entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox, bbox2);

  attrs.is_active = false;
  attrs.bounding_box = bbox1;
  entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox, bbox2);
}

// Updating an entry rotates the full pose and never compounds, even when the
// attributes were already deformed.
TEST(NodeCache, UpdateAppliesFullPose) {
  const Eigen::Vector3d pos(1.0, 2.0, 3.0);
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
  const auto attrs = makeObject(pos, rot);

  NodeCache cache;
  auto entry = cache.add(1, *attrs);
  ASSERT_NE(entry, nullptr);
  ASSERT_TRUE(entry->init_rot.has_value());

  const auto transform = makeTransform(M_PI / 2.0, Eigen::Vector3d(1.0, -1.0, 0.5));
  auto result = attrs->clone();
  auto& result_attrs = dynamic_cast<ObjectNodeAttributes&>(*result);
  entry->update(result_attrs, transform);
  expectPose(result_attrs, transform, pos, rot);

  entry->update(result_attrs, transform);  // must not compound
  expectPose(result_attrs, transform, pos, rot);
}

// Without a bounding box only position and orientation change.
TEST(NodeCache, UpdateKeepsInvalidBoundingBox) {
  NodeAttributes attrs;
  attrs.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;

  NodeCache cache;
  auto entry = cache.add(1, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_FALSE(entry->init_rot.has_value());

  SemanticNodeAttributes semantic;
  semantic.position = attrs.position;
  semantic.last_update_time_ns = 10u;
  semantic.is_active = true;
  entry = cache.add(2, semantic);
  ASSERT_NE(entry, nullptr);

  const auto transform = makeTransform(M_PI / 2.0, Eigen::Vector3d::Zero());
  const BoundingBox other(Eigen::Vector3f(1.0f, 1.0f, 1.0f),
                          Eigen::Vector3f(5.0f, 5.0f, 5.0f));
  semantic.bounding_box = other;
  entry->update(semantic, transform);
  EXPECT_NEAR((semantic.position - Eigen::Vector3d(0.0, 1.0, 0.0)).norm(), 0.0, 1.0e-9);
  EXPECT_EQ(semantic.bounding_box, other);
}

TEST(NodeCache, ApplyLastTransform) {
  const Eigen::Vector3d pos(1.0, 2.0, 3.0);
  const Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
  const auto attrs = makeObject(pos, rot);

  NodeCache cache;
  auto unchanged = attrs->clone();
  EXPECT_FALSE(cache.applyLastTransform(1, *unchanged));  // no entry

  auto entry = cache.add(1, *attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_FALSE(cache.applyLastTransform(1, *unchanged));  // never deformed
  EXPECT_EQ(*unchanged, *attrs);

  const auto transform = makeTransform(0.5, Eigen::Vector3d(0.0, 1.0, 0.0));
  entry->last_transform = transform;

  // odometric attributes are brought into the optimized frame...
  auto merged = attrs->clone();
  EXPECT_TRUE(cache.applyLastTransform(1, *merged));
  expectPose(dynamic_cast<const ObjectNodeAttributes&>(*merged), transform, pos, rot);

  // ...and attributes that were already deformed are not deformed twice
  EXPECT_TRUE(cache.applyLastTransform(1, *merged));
  expectPose(dynamic_cast<const ObjectNodeAttributes&>(*merged), transform, pos, rot);
}

// Two deformation rounds with different transforms: the second round deforms the
// odometric pose, not the result of the first round, in both graphs.
TEST(DeformationInterpolator, SuccessiveRoundsDoNotCompound) {
  const Eigen::Vector3d pos(1.0, 1.0, 0.0);
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
  const spark_dsg::NodeId node_id = 3;

  auto unmerged = test::makeSharedDsg()->graph;
  auto dsg = test::makeSharedDsg();
  unmerged->emplaceNode(DsgLayers::OBJECTS, node_id, makeObject(pos, rot));
  dsg->graph->emplaceNode(DsgLayers::OBJECTS, node_id, makeObject(pos, rot));

  DeformationInterpolator interpolator(DeformationInterpolator::Config{});
  const auto& layer = unmerged->getLayer(DsgLayers::OBJECTS);

  const auto first = makeTransform(0.4, Eigen::Vector3d(0.5, -0.2, 0.0));
  auto first_graph = test::makeRigidDeformationGraph(first);
  UpdateInfo::ConstPtr first_info(
      new UpdateInfo{0, nullptr, nullptr, true, {}, first_graph.get()});
  interpolator.interpolate(*unmerged, *dsg->graph, first_info, LayerView(layer));
  expectPose(
      unmerged->getNode(node_id).attributes<ObjectNodeAttributes>(), first, pos, rot);
  expectPose(
      dsg->graph->getNode(node_id).attributes<ObjectNodeAttributes>(), first, pos, rot);

  // the node is archived: the frontend no longer refreshes the unmerged graph, which
  // now holds the deformed pose
  unmerged->getNode(node_id).attributes().is_active = false;
  dsg->graph->getNode(node_id).attributes().is_active = false;

  const auto second = makeTransform(-0.7, Eigen::Vector3d(0.1, 0.3, 0.2));
  auto second_graph = test::makeRigidDeformationGraph(second);
  UpdateInfo::ConstPtr second_info(
      new UpdateInfo{0, nullptr, nullptr, true, {}, second_graph.get()});
  interpolator.interpolate(*unmerged, *dsg->graph, second_info, LayerView(layer));
  expectPose(
      unmerged->getNode(node_id).attributes<ObjectNodeAttributes>(), second, pos, rot);
  expectPose(dsg->graph->getNode(node_id).attributes<ObjectNodeAttributes>(),
             second,
             pos,
             rot);

  // an active node gets a fresh odometric pose from the frontend every round
  const Eigen::Vector3d new_pos(1.5, 0.5, 0.0);
  const Eigen::Quaterniond new_rot(Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitZ()));
  unmerged->setNodeAttributes(node_id, makeObject(new_pos, new_rot));
  interpolator.interpolate(*unmerged, *dsg->graph, second_info, LayerView(layer));
  expectPose(unmerged->getNode(node_id).attributes<ObjectNodeAttributes>(),
             second,
             new_pos,
             new_rot);
  expectPose(dsg->graph->getNode(node_id).attributes<ObjectNodeAttributes>(),
             second,
             new_pos,
             new_rot);

  // the merge hooks can re-derive the optimized pose from any copy of the node
  auto rebuilt = makeObject(new_pos, new_rot);
  EXPECT_TRUE(interpolator.applyLastTransform(node_id, *rebuilt));
  expectPose(*rebuilt, second, new_pos, new_rot);
}

}  // namespace hydra
