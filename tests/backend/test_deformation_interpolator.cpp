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

  attrs.is_active = false;
  attrs.bounding_box = bbox1;
  entry = cache.add(0, attrs);
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->init_bbox, bbox2);
}

}  // namespace hydra
