#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>

#include "hydra/backend/deformation_interpolator.h"

namespace hydra {

using spark_dsg::BoundingBox;
using spark_dsg::KhronosObjectAttributes;
using spark_dsg::NodeAttributes;
using spark_dsg::SemanticNodeAttributes;

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

// The deform callback transforms a copy of the cached original box rather than
// mutating the live box in place, so NodeCache must capture the box for any node
// that carries one (i.e. dynamic-casts to SemanticNodeAttributes).
TEST(NodeCache, CachesBoundingBoxForSemanticNodes) {
  SemanticNodeAttributes attrs;
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;
  attrs.bounding_box =
      BoundingBox(Eigen::Vector3f(1.0f, 2.0f, 3.0f), Eigen::Vector3f(4.0f, 5.0f, 6.0f));

  NodeCache cache;
  const NodeId node_id = 7;

  auto* entry = cache.add(node_id, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox.type, BoundingBox::Type::AABB);
  EXPECT_TRUE(entry->init_bbox.dimensions.isApprox(attrs.bounding_box.dimensions));
  EXPECT_TRUE(
      entry->init_bbox.world_P_center.isApprox(attrs.bounding_box.world_P_center));
}

// Re-adding an active node must refresh the cached box to the current frontend
// original (mergeGraph re-clones the undeformed box onto active nodes each cycle).
TEST(NodeCache, RefreshesCachedBoundingBoxWhileActive) {
  SemanticNodeAttributes attrs;
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;
  attrs.bounding_box =
      BoundingBox(Eigen::Vector3f(1.0f, 1.0f, 1.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f));

  NodeCache cache;
  const NodeId node_id = 8;
  cache.add(node_id, attrs);

  attrs.bounding_box =
      BoundingBox(Eigen::Vector3f(2.0f, 2.0f, 2.0f), Eigen::Vector3f(3.0f, 3.0f, 3.0f));
  auto* entry = cache.add(node_id, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_TRUE(entry->init_bbox.dimensions.isApprox(attrs.bounding_box.dimensions));
  EXPECT_TRUE(
      entry->init_bbox.world_P_center.isApprox(attrs.bounding_box.world_P_center));
}

// Nodes without a bounding box (plain NodeAttributes, e.g. traversability places)
// leave the cached box INVALID, so the deform callback skips the box entirely.
TEST(NodeCache, LeavesBoundingBoxInvalidForNonSemanticNodes) {
  NodeAttributes attrs;
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;

  NodeCache cache;
  auto* entry = cache.add(9, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox.type, BoundingBox::Type::INVALID);
}

}  // namespace hydra
