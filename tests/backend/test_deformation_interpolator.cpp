#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>

#include "hydra/backend/deformation_interpolator.h"

namespace hydra {

using spark_dsg::BoundingBox;
using spark_dsg::KhronosObjectAttributes;
using spark_dsg::NodeAttributes;
using spark_dsg::ObjectNodeAttributes;
using spark_dsg::SemanticNodeAttributes;

namespace {

Eigen::Isometry3d makeTransform() {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(10.0, -2.0, 1.0);
  return transform;
}

}  // namespace

// Khronos objects never populate last_update_time_ns (it stays 0), so NodeCache
// must fall back to last_observed_ns. Otherwise the cached entry is stamped 0
// and the deformation solver matches it against the earliest control points
// instead of the ones near the object's observation time.
TEST(NodeCache, KhronosFallbackTimestampOnInsert) {
  constexpr uint64_t observed_ns = 1782847154557208075ULL;
  KhronosObjectAttributes attrs;
  attrs.last_update_time_ns = 0;
  attrs.last_observed_ns = {observed_ns};
  attrs.is_active = true;

  NodeCache cache;
  const NodeId node_id = 42;

  auto* entry = cache.add(node_id, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->timestamp, observed_ns);

  // re-adding an active node must keep the fallback stamp, not reset it to 0
  entry = cache.add(node_id, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->timestamp, observed_ns);
}

// The unmerged graph holds odometric values by invariant, so the cached position
// (used as the pgmo vertex for control-point matching) refreshes on every add,
// including for archived nodes.
TEST(NodeCache, RefreshesPositionEvenWhenArchived) {
  NodeAttributes attrs;
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;
  attrs.position = Eigen::Vector3d(1.0, 1.0, 1.0);

  NodeCache cache;
  const NodeId node_id = 8;
  cache.add(node_id, attrs);

  attrs.is_active = false;
  attrs.position = Eigen::Vector3d(2.0, 3.0, 4.0);
  auto* entry = cache.add(node_id, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_TRUE(entry->pos.isApprox(attrs.position.cast<float>()));
}

// Merge hooks rebuild parent attributes from odometric constituents, so without a
// recorded deformation there is nothing to apply and the attributes stay untouched.
TEST(NodeCache, ApplyLastTransformNoEntryIsNoop) {
  NodeCache cache;

  NodeAttributes attrs;
  attrs.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  EXPECT_FALSE(cache.applyLastTransform(3, attrs));
  EXPECT_TRUE(attrs.position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  // an entry that was never deformed also applies nothing
  attrs.last_update_time_ns = 10u;
  cache.add(3, attrs);
  EXPECT_FALSE(cache.applyLastTransform(3, attrs));
  EXPECT_TRUE(attrs.position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

// Once the deform callback records a node's odometric->optimized transform, merge
// hooks can bring freshly-cloned odometric attributes into the optimized frame.
TEST(NodeCache, ApplyLastTransformAppliesStoredTransform) {
  const auto transform = makeTransform();

  ObjectNodeAttributes attrs;
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;
  attrs.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  attrs.bounding_box =
      BoundingBox(Eigen::Vector3f(1.0f, 2.0f, 3.0f), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  attrs.world_R_object = Eigen::Quaterniond::Identity();

  NodeCache cache;
  const NodeId node_id = 4;
  cache.add(node_id, attrs);
  cache.nodes.at(node_id).last_transform = transform;

  auto expected_box = attrs.bounding_box;
  expected_box.transform(transform);

  auto merged = attrs.clone();
  EXPECT_TRUE(cache.applyLastTransform(node_id, *merged));

  const auto& result = dynamic_cast<const ObjectNodeAttributes&>(*merged);
  EXPECT_TRUE(result.position.isApprox(transform * attrs.position));
  EXPECT_TRUE(result.bounding_box.world_P_center.isApprox(expected_box.world_P_center));
  EXPECT_TRUE(
      result.world_R_object.toRotationMatrix().isApprox(transform.linear(), 1.0e-9));
}

// The deform callback derives every optimized field from the odometric source, so
// applying the same transform repeatedly to an already-deformed destination must
// not compound (archived merged nodes are re-deformed every post-LC spin).
TEST(ApplyNodeDeformation, DerivesFromSourceIdempotently) {
  const auto transform = makeTransform();

  ObjectNodeAttributes src;
  src.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  src.bounding_box =
      BoundingBox(Eigen::Vector3f(2.0f, 4.0f, 6.0f), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  src.world_R_object = Eigen::Quaterniond::Identity();

  auto dst_ptr = src.clone();
  auto& dst = dynamic_cast<ObjectNodeAttributes&>(*dst_ptr);
  // simulate stale optimized state on the destination
  dst.position = Eigen::Vector3d(9.0, 9.0, 9.0);
  dst.bounding_box =
      BoundingBox(Eigen::Vector3f(9.0f, 9.0f, 9.0f), Eigen::Vector3f(9.0f, 9.0f, 9.0f));

  auto expected_box = src.bounding_box;
  expected_box.transform(transform);

  applyNodeDeformation(transform, src, dst);
  applyNodeDeformation(transform, src, dst);  // second application must not compound

  EXPECT_TRUE(dst.position.isApprox(transform * src.position));
  EXPECT_TRUE(dst.bounding_box.world_P_center.isApprox(expected_box.world_P_center));
  EXPECT_TRUE(dst.bounding_box.dimensions.isApprox(expected_box.dimensions));
  EXPECT_TRUE(
      dst.world_R_object.toRotationMatrix().isApprox(transform.linear(), 1.0e-9));
}

// Merged-only state (e.g. merge-accumulated mesh connections) must survive
// deformation: only spatial fields are written.
TEST(ApplyNodeDeformation, PreservesNonSpatialDestinationState) {
  const auto transform = makeTransform();

  KhronosObjectAttributes src;
  src.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  src.mesh_connections = {1, 2};

  auto dst_ptr = src.clone();
  auto& dst = dynamic_cast<KhronosObjectAttributes&>(*dst_ptr);
  dst.mesh_connections = {1, 2, 3, 4};  // accumulated by merges

  applyNodeDeformation(transform, src, dst);

  EXPECT_TRUE(dst.position.isApprox(transform * src.position));
  EXPECT_EQ(dst.mesh_connections, decltype(dst.mesh_connections)({1, 2, 3, 4}));
}

// Nodes without a bounding box (plain NodeAttributes, e.g. agent poses) only get
// their position updated; an INVALID source box is never copied over.
TEST(ApplyNodeDeformation, SkipsBoxForPlainAttributes) {
  const auto transform = makeTransform();

  NodeAttributes src;
  src.position = Eigen::Vector3d(1.0, 2.0, 3.0);

  NodeAttributes dst;
  applyNodeDeformation(transform, src, dst);
  EXPECT_TRUE(dst.position.isApprox(transform * src.position));

  // semantic destination keeps its box when the source box is INVALID
  SemanticNodeAttributes semantic_src;
  semantic_src.position = Eigen::Vector3d(1.0, 2.0, 3.0);

  SemanticNodeAttributes semantic_dst;
  semantic_dst.bounding_box =
      BoundingBox(Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
  applyNodeDeformation(transform, semantic_src, semantic_dst);
  EXPECT_EQ(semantic_dst.bounding_box.type, BoundingBox::Type::AABB);
  EXPECT_TRUE(
      semantic_dst.bounding_box.dimensions.isApprox(Eigen::Vector3f(1.0f, 1.0f, 1.0f)));
}

}  // namespace hydra
