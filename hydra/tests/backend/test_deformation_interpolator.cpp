#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>

#include "hydra/backend/deformation_interpolator.h"

namespace hydra {

using spark_dsg::BoundingBox;
using spark_dsg::KhronosObjectAttributes;
using spark_dsg::SemanticNodeAttributes;

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

  // the update that archives the node is still odometric
  const BoundingBox bbox3(Eigen::Vector3f(4.0f, 4.0f, 4.0f),
                          Eigen::Vector3f(5.0f, 5.0f, 5.0f));
  attrs.is_active = false;
  attrs.bounding_box = bbox3;
  entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox, bbox3);

  // later updates of an archived node hold deformed values and are ignored
  attrs.bounding_box = bbox1;
  entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox, bbox3);
}

TEST(DeformationInterpolator, ArchivingUpdateRefreshesPosition) {
  SemanticNodeAttributes attrs;
  attrs.last_update_time_ns = 10u;
  attrs.is_active = true;
  attrs.position << 1.0, 2.0, 3.0;

  NodeCache cache;
  ASSERT_NE(cache.add(0, attrs), nullptr);
  ASSERT_NE(cache.add(0, attrs), nullptr);

  // e.g., the final object from Khronos replaces the live one as it is archived
  attrs.is_active = false;
  attrs.position << 4.0, 5.0, 6.0;
  auto entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_TRUE(entry->init_pos.isApprox(Eigen::Vector3f(4.0f, 5.0f, 6.0f)));

  // a node first seen archived (or active for a single update, e.g., forced active by
  // the backend) is cached as is and then never refreshed
  attrs.position << 7.0, 8.0, 9.0;
  entry = cache.add(0, attrs);
  EXPECT_TRUE(entry->init_pos.isApprox(Eigen::Vector3f(4.0f, 5.0f, 6.0f)));
  entry = cache.add(1, attrs);
  attrs.position << 0.0, 0.0, 0.0;
  entry = cache.add(1, attrs);
  EXPECT_TRUE(entry->init_pos.isApprox(Eigen::Vector3f(7.0f, 8.0f, 9.0f)));
  attrs.is_active = true;
  attrs.position << 1.0, 1.0, 1.0;
  entry = cache.add(2, attrs);
  attrs.is_active = false;
  attrs.position << 2.0, 2.0, 2.0;
  entry = cache.add(2, attrs);
  EXPECT_TRUE(entry->init_pos.isApprox(Eigen::Vector3f(1.0f, 1.0f, 1.0f)));
}

}  // namespace hydra
